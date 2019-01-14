/*************************************************************************
	> File Name: motion_control_arm_group.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 14时34分07秒
 ************************************************************************/

#include <math.h>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <motion_control_arm_group.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <error_monitor.h>
#include <common_file_path.h>
#include <segment_alg.h>


using namespace std;
using namespace fst_base;
using namespace basic_alg;
using namespace fst_parameter;
using namespace fst_algorithm;

extern ComplexAxisGroupModel model;

namespace fst_mc
{

ErrorCode ArmGroup::initGroup(ErrorMonitor *error_monitor_ptr)
{
    vel_ratio_ = 0.0625;
    acc_ratio_ = 0.0625;
    cycle_time_ = 0.001;
    memset(&manual_traj_, 0, sizeof(ManualTrajectory));

    FST_INFO("Error monitor addr: 0x%x", error_monitor_ptr);
    if (error_monitor_ptr == NULL)
    {
        FST_ERROR("Invalid pointer of error-monitor.");
        return MOTION_INTERNAL_FAULT;
    }
    else
    {
        error_monitor_ptr_ = error_monitor_ptr;
    }

    FST_INFO("Initializing mutex ...");
    if (pthread_mutex_init(&cache_list_mutex_, NULL) != 0 ||
        pthread_mutex_init(&manual_traj_mutex_, NULL) != 0 ||
        pthread_mutex_init(&servo_mutex_, NULL) != 0)
    {
        FST_ERROR("Fail to initialize mutex.");
        return MOTION_INTERNAL_FAULT;
    }

    ParamGroup param;
    vector<double> upper;
    vector<double> lower;
    string path = AXIS_GROUP_DIR;

    // 加载硬限位
    FST_INFO("Loading hard constraints ...");

    if (param.loadParamFile(path + "hard_constraint.yaml") &&
        param.getParam("hard_constraint/upper", upper) &&
        param.getParam("hard_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            hard_constraint_.initConstraint(*(Joint*)(&lower[0]), *(Joint*)(&upper[0]), JOINT_OF_ARM);
        }
        else
        {
            FST_ERROR("Invalid array size of hard constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading hard constraint from config file");
        return param.getLastError();
    }

    // 加载软限位上下限
    FST_INFO("Loading firm constraints ...");
    param.reset();
    upper.clear();
    lower.clear();

    if (param.loadParamFile(path + "firm_constraint.yaml") &&
        param.getParam("firm_constraint/upper", upper) &&
        param.getParam("firm_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            firm_constraint_.initConstraint(*(Joint*)(&lower[0]), *(Joint*)(&upper[0]), JOINT_OF_ARM);
        }
        else
        {
            FST_ERROR("Invalid array size of firm constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading firm constraint from config file");
        return param.getLastError();
    }

    // 加载软限位
    FST_INFO("Loading soft constraints ...");
    param.reset();
    upper.clear();
    lower.clear();

    if (param.loadParamFile(path + "soft_constraint.yaml") &&
        param.getParam("soft_constraint/upper", upper) &&
        param.getParam("soft_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            soft_constraint_.initConstraint(*(Joint*)(&lower[0]), *(Joint*)(&upper[0]), JOINT_OF_ARM);
        }
        else
        {
            FST_ERROR("Invalid array size of soft constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading soft constraint from config file");
        return param.getLastError();
    }

    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Hard/firm/soft constraints from axis-%d to axis-%d:", 0, JOINT_OF_ARM - 1);
    FST_INFO("  hard-lower: %s", printDBLine(&hard_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  firm-lower: %s", printDBLine(&firm_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  soft-lower: %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  soft-upper: %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  firm-upper: %s", printDBLine(&firm_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  hard-upper: %s", printDBLine(&hard_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));

    // 加载motion_control参数设置
    param.reset();

    if (!param.loadParamFile(path + "base_group.yaml"))
    {
        FST_ERROR("Fail loading motion configuration from config file");
        return param.getLastError();
    }

    // 从配置文件加载轨迹FIFO的容量，并初始化轨迹FIFO
    int traj_fifo_size;
    if (param.getParam("trajectory_fifo_size", traj_fifo_size))
    {
        FST_INFO("Initializing trajectory fifo ... capacity = %d", traj_fifo_size);

        if (traj_fifo_.initTrajectoryFifo(traj_fifo_size, JOINT_OF_ARM) == SUCCESS)
        {
            FST_INFO("Success.");
        }
        else
        {
            FST_ERROR("Fail to initialize trajectory fifo.");
            return MOTION_INTERNAL_FAULT;
        }
    }
    else
    {
        FST_ERROR("Fail loading trajectory fifo size from config file");
        return param.getLastError();
    }

    // 从配置文件加载轨迹FIFO中每个segment的时间长度
    if (param.getParam("duration_of_segment_in_trajectory_fifo", duration_per_segment_))
    {
        FST_INFO("Duration of segments in trajectory fifo: %.6f", duration_per_segment_);

        if (duration_per_segment_ < cycle_time_)
        {
            FST_ERROR("Duration of segments in trajectory fifo < cycle-time : %.6f", cycle_time_);
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading duration of segments in trajectory fifo from config file");
        return param.getLastError();
    }

    // 从配置文件加载路径缓存池和轨迹缓存池的容量，并初始化两个缓存池
    int path_cache_size, traj_cache_size;

    if (param.getParam("path_cache_size", path_cache_size) && param.getParam("trajectory_cache_size", traj_cache_size))
    {
        FST_INFO("Initializing path cache ... capacity = %d", path_cache_size);

        if (path_cache_pool_.initCachePool(path_cache_size) == SUCCESS)
        {
            FST_INFO("Success.");
        }
        else
        {
            FST_ERROR("Fail to initialize path cache.");
            return MOTION_INTERNAL_FAULT;
        }

        FST_INFO("Initializing trajectory cache ... capacity = %d", traj_cache_size);

        if (traj_cache_pool_.initCachePool(traj_cache_size) == SUCCESS)
        {
            FST_INFO("Success.");
        }
        else
        {
            FST_ERROR("Fail to initialize trajectory cache.");
            return MOTION_INTERNAL_FAULT;
        }
    }
    else
    {
        FST_ERROR("Fail loading path cache size or trajectory cache size from config file");
        return param.getLastError();
    }

    // 从配置文件加载Fine指令的稳定周期和稳定门限
    int     stable_times;
    double  stable_threshold;
    if (param.getParam("stable_with_fine/cycle", stable_times) && param.getParam("stable_with_fine/threshold", stable_threshold))
    {
        if (stable_times > 0 && stable_threshold > 0)
        {
            FST_INFO("Fine settings: cycle = %d, threshold = %.4f", stable_times, stable_threshold);
            fine_waiter_.initFineWaiter(stable_times, stable_threshold);
        }
        else
        {
            FST_ERROR("Invalid fine settings: cycle = %d, threshold = %.4f", stable_times, stable_threshold);
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading fine setings from config file");
        return param.getLastError();
    }
    
    // 从配置文件加载笛卡尔空间线速度上下限
    if (param.getParam("cartesian_vel_limit/min", cartesian_vel_min_) && param.getParam("cartesian_vel_limit/max", cartesian_vel_max_))
    {
        FST_INFO("Cartesian velocity: %.2f ~ %.2f", cartesian_vel_min_, cartesian_vel_max_);
    }
    else
    {
        FST_ERROR("Fail loading cartesian velocity limit from config file");
        return param.getLastError();
    }

    // 从配置文件加载状态机中各个临时状态的超时时间
    int time_out;
    
    if (param.getParam("time_out_cycle/disable_to_standby", time_out) && time_out > 0)
    {
        disable_to_standby_timeout_ = time_out;
    }
    else
    {
        FST_ERROR("Fail loading disable->standby timeout from config file");
        return param.getLastError() != SUCCESS ? param.getLastError() : INVALID_PARAMETER;
    }

    if (param.getParam("time_out_cycle/standby_to_disable", time_out) && time_out > 0)
    {
        standby_to_disable_timeout_ = time_out;
    }
    else
    {
        FST_ERROR("Fail loading standby->disable timeout from config file");
        return param.getLastError() != SUCCESS ? param.getLastError() : INVALID_PARAMETER;
    }

    if (param.getParam("time_out_cycle/standby_to_auto", time_out) && time_out > 0)
    {
        standby_to_auto_timeout_ = time_out;
    }
    else
    {
        FST_ERROR("Fail loading standby->auto timeout from config file");
        return param.getLastError() != SUCCESS ? param.getLastError() : INVALID_PARAMETER;
    }

    if (param.getParam("time_out_cycle/auto_to_standby", time_out) && time_out > 0)
    {
        auto_to_standby_timeout_ = time_out;
    }
    else
    {
        FST_ERROR("Fail loading auto->standby timeout from config file");
        return param.getLastError() != SUCCESS ? param.getLastError() : INVALID_PARAMETER;
    }

    if (param.getParam("time_out_cycle/manual_to_standby", time_out) && time_out > 0)
    {
        manual_to_standby_timeout_ = time_out;
    }
    else
    {
        FST_ERROR("Fail loading manual->standby timeout from config file");
        return param.getLastError() != SUCCESS ? param.getLastError() : INVALID_PARAMETER;
    }

    if (param.getParam("time_out_cycle/trajectory_flow", time_out) && time_out > 0)
    {
        trajectory_flow_timeout_ = time_out;
    }
    else
    {
        FST_ERROR("Fail loading trajectory flow timeout from config file");
        return param.getLastError() != SUCCESS ? param.getLastError() : INVALID_PARAMETER;
    }

    if (param.getParam("time_out_cycle/servo_update", time_out) && time_out > 0)
    {
        servo_update_timeout_ = time_out;
    }
    else
    {
        FST_ERROR("Fail loading servo update timeout from config file");
        return param.getLastError() != SUCCESS ? param.getLastError() : INVALID_PARAMETER;
    }

    // 初始化裸核通信句柄
    FST_INFO("Initializing interface to bare core ...");

    if (!bare_core_.initInterface())
    {
        FST_ERROR("Fail to create communication with bare core.");
        return BARE_CORE_TIMEOUT;
    }

    // 初始化零位校验模块
    FST_INFO("Initializing calibrator of ArmGroup ...");
    ErrorCode err = calibrator_.initCalibrator(JOINT_OF_ARM, &bare_core_, log_ptr_);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to initialize calibrator, code = 0x%llx", err);
        return err;
    }

    // 如果某些轴的零位错误被屏蔽，必须同时屏蔽这些轴的软限位校验
    OffsetMask mask[NUM_OF_JOINT] = {OFFSET_UNMASK};
    calibrator_.getOffsetMask(mask);
    size_t index[JOINT_OF_ARM];
    size_t length = 0;

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        if (mask[i] == OFFSET_MASKED)
        {
            index[length++] = i;
        }
    }

    soft_constraint_.setMask(index, length);

    // 初始化运动学模块
    FST_INFO("Initializing kinematics of ArmGroup ...");
    double dh_matrix[NUM_OF_JOINT][4];
    kinematics_ptr_ = new ArmKinematics();

    if (kinematics_ptr_)
    {
        param.reset();

        if (param.loadParamFile(path + "arm_dh.yaml"))
        {
            if (param.getParam("dh_parameter/axis-0", dh_matrix[0], 4) &&
                param.getParam("dh_parameter/axis-1", dh_matrix[1], 4) &&
                param.getParam("dh_parameter/axis-2", dh_matrix[2], 4) &&
                param.getParam("dh_parameter/axis-3", dh_matrix[3], 4) &&
                param.getParam("dh_parameter/axis-4", dh_matrix[4], 4) &&
                param.getParam("dh_parameter/axis-5", dh_matrix[5], 4))
            {
                kinematics_ptr_->initKinematics(dh_matrix);
                FST_INFO("DH-matrix:");
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[0][0], dh_matrix[0][1], dh_matrix[0][2], dh_matrix[0][3]);
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[1][0], dh_matrix[1][1], dh_matrix[1][2], dh_matrix[1][3]);
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[2][0], dh_matrix[2][1], dh_matrix[2][2], dh_matrix[2][3]);
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[3][0], dh_matrix[3][1], dh_matrix[3][2], dh_matrix[3][3]);
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[4][0], dh_matrix[4][1], dh_matrix[4][2], dh_matrix[4][3]);
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[5][0], dh_matrix[5][1], dh_matrix[5][2], dh_matrix[5][3]);
            }
            else
            {
                FST_ERROR("Fail to load dh config, code = 0x%llx", param.getLastError());
                return param.getLastError();
            }
        }
        else
        {
            FST_ERROR("Fail to load dh config, code = 0x%llx", param.getLastError());
            return param.getLastError();
        }
    }
    else
    {
        FST_ERROR("Fail to create kinematics for ArmGroup.");
        return MOTION_INTERNAL_FAULT;
    }

    // 初始化动力学模块
    FST_INFO("Initializing dynamics of ArmGroup ...");
    dynamics_ptr_ = new DynamicsInterface();

    if (dynamics_ptr_ == NULL)
    {
        FST_ERROR("Fail to create dynamics for ArmGroup.");
        return MOTION_INTERNAL_FAULT;
    }

    // 初始化手动示教模块
    FST_INFO("Initializing manual teach of ArmGroup ...");
    err = manual_teach_.init(kinematics_ptr_, &soft_constraint_, log_ptr_, path + "arm_manual_teach.yaml");

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to initialize manual teach, code = 0x%llx", err);
        return err;
    }

    manual_teach_.setGlobalVelRatio(vel_ratio_);
    manual_teach_.setGlobalAccRatio(acc_ratio_);

    // 初始化路径和轨迹规划
    param.reset();
    path = COMPONENT_PARAM_FILE_DIR;
    SegmentAlgParam seg_param;
    seg_param.kinematics_ptr = kinematics_ptr_;
    seg_param.dynamics_ptr = dynamics_ptr_;

    if (param.loadParamFile(path + "segment_alg.yaml"))
    {
        if (param.getParam("accuracy_cartesian_factor", seg_param.accuracy_cartesian_factor) &&
            param.getParam("accuracy_joint_factor", seg_param.accuracy_joint_factor) &&
            param.getParam("max_traj_points_num", seg_param.max_traj_points_num) &&
            param.getParam("path_interval", seg_param.path_interval) &&
            param.getParam("joint_interval", seg_param.joint_interval) &&
            param.getParam("angle_interval", seg_param.angle_interval) &&
            param.getParam("angle_valve", seg_param.angle_valve) &&
            param.getParam("conservative_acc", seg_param.conservative_acc) &&
            param.getParam("time_factor_first", seg_param.time_factor_first) &&
            param.getParam("time_factor_last", seg_param.time_factor_last) &&
            param.getParam("is_fake_dynamics", seg_param.is_fake_dynamics) &&
            param.getParam("max_cartesian_acc", seg_param.max_cartesian_acc))
        {}
        else
        {
            FST_ERROR("Fail to load segment algorithm config, code = 0x%llx", param.getLastError());
            return param.getLastError();
        }
    }
    else
    {
        FST_ERROR("Fail to load segment algorithm config, code = 0x%llx", param.getLastError());
        return param.getLastError();
    }

    initComplexAxisGroupModel();
    initSegmentAlgParam(&seg_param);

    return SUCCESS;
}


size_t ArmGroup::getNumberOfJoint(void)
{
    return JOINT_OF_ARM;
}


size_t ArmGroup::getFIFOLength(void)
{
    if (group_state_ == MANUAL)
    {
        return ceil((manual_traj_.duration - manual_time_) * 1000);
    }
    else if (group_state_ == AUTO)
    {
        return 0;
    }
    else
    {
        return 0;
    }
}


ErrorCode ArmGroup::computeCompensate(const DynamicsProduct &product, const Joint &omega, const Joint &alpha, Joint &ma_cv_g)
{
    /*
    ma_cv_g[0] = product.m[0][0] * alpha[0] + product.m[0][1] * alpha[1] + product.m[0][2] * alpha[2] +
                 product.m[0][3] * alpha[3] + product.m[0][4] * alpha[4] + product.m[0][5] * alpha[5] +
                 product.c[0][0] * omega[0] + product.c[0][1] * omega[1] + product.c[0][2] * omega[2] +
                 product.c[0][3] * omega[3] + product.c[0][4] * omega[4] + product.c[0][5] * omega[5] +
                 product.g[0];

    ma_cv_g[1] = product.m[1][0] * alpha[0] + product.m[1][1] * alpha[1] + product.m[1][2] * alpha[2] +
                 product.m[1][3] * alpha[3] + product.m[1][4] * alpha[4] + product.m[1][5] * alpha[5] +
                 product.c[1][0] * omega[0] + product.c[1][1] * omega[1] + product.c[1][2] * omega[2] +
                 product.c[1][3] * omega[3] + product.c[1][4] * omega[4] + product.c[1][5] * omega[5] +
                 product.g[1];

    ma_cv_g[2] = product.m[2][0] * alpha[0] + product.m[2][1] * alpha[1] + product.m[2][2] * alpha[2] +
                 product.m[2][3] * alpha[3] + product.m[2][4] * alpha[4] + product.m[2][5] * alpha[5] +
                 product.c[2][0] * omega[0] + product.c[2][1] * omega[1] + product.c[2][2] * omega[2] +
                 product.c[2][3] * omega[3] + product.c[2][4] * omega[4] + product.c[2][5] * omega[5] +
                 product.g[2];

    ma_cv_g[3] = product.m[3][0] * alpha[0] + product.m[3][1] * alpha[1] + product.m[3][2] * alpha[2] +
                 product.m[3][3] * alpha[3] + product.m[3][4] * alpha[4] + product.m[3][5] * alpha[5] +
                 product.c[3][0] * omega[0] + product.c[3][1] * omega[1] + product.c[3][2] * omega[2] +
                 product.c[3][3] * omega[3] + product.c[3][4] * omega[4] + product.c[3][5] * omega[5] +
                 product.g[3];

    ma_cv_g[4] = product.m[4][0] * alpha[0] + product.m[4][1] * alpha[1] + product.m[4][2] * alpha[2] +
                 product.m[4][3] * alpha[3] + product.m[4][4] * alpha[4] + product.m[4][5] * alpha[5] +
                 product.c[4][0] * omega[0] + product.c[4][1] * omega[1] + product.c[4][2] * omega[2] +
                 product.c[4][3] * omega[3] + product.c[4][4] * omega[4] + product.c[4][5] * omega[5] +
                 product.g[4];

    ma_cv_g[5] = product.m[5][0] * alpha[0] + product.m[5][1] * alpha[1] + product.m[5][2] * alpha[2] +
                 product.m[5][3] * alpha[3] + product.m[5][4] * alpha[4] + product.m[5][5] * alpha[5] +
                 product.c[5][0] * omega[0] + product.c[5][1] * omega[1] + product.c[5][2] * omega[2] +
                 product.c[5][3] * omega[3] + product.c[5][4] * omega[4] + product.c[5][5] * omega[5] +
                 product.g[5];
    */
    ma_cv_g[0] = product.m[0][0] * alpha[0] + product.m[0][1] * alpha[1] + product.m[0][2] * alpha[2] +
                 product.m[0][3] * alpha[3] + product.m[0][4] * alpha[4] + product.m[0][5] * alpha[5];

    ma_cv_g[1] = product.m[1][0] * alpha[0] + product.m[1][1] * alpha[1] + product.m[1][2] * alpha[2] +
                 product.m[1][3] * alpha[3] + product.m[1][4] * alpha[4] + product.m[1][5] * alpha[5];

    ma_cv_g[2] = product.m[2][0] * alpha[0] + product.m[2][1] * alpha[1] + product.m[2][2] * alpha[2] +
                 product.m[2][3] * alpha[3] + product.m[2][4] * alpha[4] + product.m[2][5] * alpha[5];

    ma_cv_g[3] = product.m[3][0] * alpha[0] + product.m[3][1] * alpha[1] + product.m[3][2] * alpha[2] +
                 product.m[3][3] * alpha[3] + product.m[3][4] * alpha[4] + product.m[3][5] * alpha[5];

    ma_cv_g[4] = product.m[4][0] * alpha[0] + product.m[4][1] * alpha[1] + product.m[4][2] * alpha[2] +
                 product.m[4][3] * alpha[3] + product.m[4][4] * alpha[4] + product.m[4][5] * alpha[5];

    ma_cv_g[5] = product.m[5][0] * alpha[0] + product.m[5][1] * alpha[1] + product.m[5][2] * alpha[2] +
                 product.m[5][3] * alpha[3] + product.m[5][4] * alpha[4] + product.m[5][5] * alpha[5];

    return SUCCESS;
}


char* ArmGroup::printDBLine(const int *data, char *buffer, size_t length)
{
    int len = 0;

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        len += snprintf(buffer + len, length - len, "%d, ", data[i]);
    }

    if (len > 1)
    {
        len --;
        len --;
    }

    buffer[len] = '\0';
    return buffer;
}

char* ArmGroup::printDBLine(const double *data, char *buffer, size_t length)
{
    int len = 0;

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        len += snprintf(buffer + len, length - len, "%.6f ", data[i]);
        //len += snprintf(buffer + len, length - len, "%.9f, ", data[i]);
        //len += snprintf(buffer + len, length - len, "%.12f, ", data[i]);
    }

    if (len > 1)
    {
        len --;
        len --;
    }

    buffer[len] = '\0';
    return buffer;
}





}



