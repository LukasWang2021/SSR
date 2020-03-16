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
#include <dynamic_alg_rtm.h>


using namespace std;
using namespace fst_ctrl;
using namespace fst_base;
using namespace basic_alg;
using namespace fst_parameter;


namespace fst_mc
{

ErrorCode ArmGroup::initGroup(ErrorMonitor *error_monitor_ptr, CoordinateManager *coordinate_manager_ptr, ToolManager *tool_manager_ptr)
{
    vel_ratio_ = 0.1;
    acc_ratio_ = 1.0;
    //acc_ratio_ = 0.0625;
    cycle_time_ = 0.001;

    FST_INFO("Error monitor addr: 0x%x", error_monitor_ptr);
    if (error_monitor_ptr == NULL || coordinate_manager_ptr == NULL || tool_manager_ptr == NULL)
    {
        FST_ERROR("Invalid pointer of error-monitor or coordinate-manager or tool-manager.");
        return MC_INTERNAL_FAULT;
    }
    else
    {
        error_monitor_ptr_ = error_monitor_ptr;
    }

    coordinate_manager_ptr_ = coordinate_manager_ptr;
    tool_manager_ptr_ = tool_manager_ptr;

    FST_INFO("Initializing mutex ...");
    if (pthread_mutex_init(&planner_list_mutex_, NULL) != 0 ||
        pthread_mutex_init(&manual_traj_mutex_, NULL) != 0 ||
        pthread_mutex_init(&servo_mutex_, NULL) != 0 ||
        pthread_mutex_init(&offline_mutex_, NULL) != 0)
    {
        FST_ERROR("Fail to initialize mutex.");
        return MC_INTERNAL_FAULT;
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

    // 初始化运动学模块
    FST_INFO("Initializing kinematics of ArmGroup ...");
    kinematics_ptr_ = new KinematicsRTM(path);

    if (kinematics_ptr_ == NULL || !kinematics_ptr_->isValid())
    {
        FST_ERROR("Fail to create kinematics for this Group.");
        return MC_INTERNAL_FAULT;
    }

    // 初始化动力学模块
    FST_INFO("Initializing dynamics of ArmGroup ...");
    dynamics_ptr_ = new DynamicAlgRTM();

    if (dynamics_ptr_ == NULL)
    {
        FST_ERROR("Fail to create dynamics for ArmGroup.");
        return MC_INTERNAL_FAULT;
    }

    if (!dynamics_ptr_->initDynamicAlg(path))
    {
        FST_ERROR("Fail to init dynamics for ArmGroup.");
        return MC_FAIL_IN_INIT;
    }

    // 加载motion_control参数设置
    param.reset();

    if (!param.loadParamFile(path + "base_group.yaml"))
    {
        FST_ERROR("Fail loading motion configuration from config file");
        return param.getLastError();
    }

    // 从配置文件加载轨迹FIFO的容量，并初始化轨迹FIFO
    int traj_fifo_config;

    if (!param.getParam("trajectory_fifo/fifo_size", traj_fifo_config))
    {
        FST_ERROR("Fail loading trajectory FIFO size from config file");
        return param.getLastError();
    }

    FST_INFO("Initializing trajectory FIFO ... capacity = %d", traj_fifo_config);

    if (!traj_fifo_.init(traj_fifo_config))
    {
        FST_ERROR("Fail to initialize trajectory FIFO.");
        return MC_INTERNAL_FAULT;
    }

    if (!param.getParam("trajectory_fifo/lower_limit", traj_fifo_config))
    {
        FST_ERROR("Fail loading trajectory FIFO lower limit size from config file");
        return param.getLastError();
    }

    traj_fifo_lower_limit_ = traj_fifo_config;
    FST_INFO("Trajectory FIFO lower limit size = %d", traj_fifo_lower_limit_);
    FST_INFO("Success.");

    // 初始化轨迹缓存
    trajectory_a_.prev = &trajectory_b_;
    trajectory_a_.next = &trajectory_b_;
    trajectory_b_.prev = &trajectory_a_;
    trajectory_b_.next = &trajectory_a_;
    trajectory_a_.valid = false;
    trajectory_b_.valid = false;
    
    if (!trajectory_a_.trajectory.initPlanner(JOINT_OF_ARM, cycle_time_, kinematics_ptr_, dynamics_ptr_, &soft_constraint_, log_ptr_) ||
        !trajectory_b_.trajectory.initPlanner(JOINT_OF_ARM, cycle_time_, kinematics_ptr_, dynamics_ptr_, &soft_constraint_, log_ptr_))
    {
        FST_ERROR("Fail to initialize trajectory planner.");
        return MC_FAIL_IN_INIT;
    }

    Joint omega_limit, alpha_limit, beta_limit;
    trajectory_a_.smooth.initPlanner(JOINT_OF_ARM, cycle_time_, kinematics_ptr_, &soft_constraint_, log_ptr_);
    trajectory_b_.smooth.initPlanner(JOINT_OF_ARM, cycle_time_, kinematics_ptr_, &soft_constraint_, log_ptr_);
    trajectory_a_.smooth.setLimit(omega_limit, alpha_limit, beta_limit, 2000, 10000, 100000, 1, 10, 100);
    trajectory_b_.smooth.setLimit(omega_limit, alpha_limit, beta_limit, 2000, 10000, 100000, 1, 10, 100);

    plan_traj_ptr_ = &trajectory_a_;
    pick_traj_ptr_ = &trajectory_a_;

    // 从配置文件加载Fine指令的稳定周期和稳定门限
    int     stable_times;
    double  stable_threshold;

    if (param.getParam("stable_with_fine/cycle", stable_times) && param.getParam("stable_with_fine/threshold", stable_threshold))
    {
        if (stable_times > 0 && stable_threshold > 0)
        {
            fine_cycle_ = stable_times;
            fine_threshold_ = stable_threshold;
            FST_INFO("Fine settings: cycle = %d, threshold = %.4f", stable_times, stable_threshold);
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

    if (param.getParam("time_out_cycle/offline_to_standby", time_out) && time_out > 0)
    {
        offline_to_standby_timeout_ = time_out;
    }
    else
    {
        FST_ERROR("Fail loading offline->standby timeout from config file");
        return param.getLastError() != SUCCESS ? param.getLastError() : INVALID_PARAMETER;
    }

    if (param.getParam("time_out_cycle/auto_to_pause", time_out) && time_out > 0)
    {
        auto_to_pause_timeout_ = time_out;
    }
    else
    {
        FST_ERROR("Fail loading auto->pause timeout from config file");
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

    int cycles;

    if (param.getParam("joint_record/update_cycle", cycles) && cycles > 0)
    {
        joint_record_update_cycle_ = cycles;
    }
    else
    {
        FST_ERROR("Fail loading record cycle from config file");
        return param.getLastError() != SUCCESS ? param.getLastError() : INVALID_PARAMETER;
    }

    if (param.getParam("joint_record/update_timeout", cycles) && cycles > 0)
    {
        joint_record_update_timeout_ = cycles;
    }
    else
    {
        FST_ERROR("Fail loading record timeout from config file");
        return param.getLastError() != SUCCESS ? param.getLastError() : INVALID_PARAMETER;
    }

    // 从配置文件中加载关节位置跟随误差门限
    double tracking_accuracy[JOINT_OF_ARM];

    if (!param.getParam("joint_tracking_accuracy", tracking_accuracy, JOINT_OF_ARM))
    {
        FST_ERROR("Fail to load joint tracking error threshold for each axis, code = 0x%llx", param.getLastError());
        return param.getLastError();
    }

    memcpy(&joint_tracking_accuracy_.j1_, tracking_accuracy, sizeof(tracking_accuracy));

    // 初始化裸核通信句柄
    FST_INFO("Initializing interface to bare core ...");

    if (!bare_core_.initInterface(JOINT_OF_ARM))
    {
        FST_ERROR("Fail to create communication with bare core.");
        return MC_FAIL_IN_INIT;
    }

    // 下发伺服参数
    ErrorCode err = downloadServoParam("/root/install/share/configuration/machine/servo_param.yaml");
    if (err != SUCCESS)
    {
        FST_ERROR("Fail to download servo parameter");
        return err;
    }

    // 初始化零位校验模块
    FST_INFO("Initializing calibrator of ArmGroup ...");
    err = calibrator_.initCalibrator(JOINT_OF_ARM, &bare_core_, log_ptr_);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to initialize calibrator, code = 0x%llx", err);
        return err;
    }

    // 如果某些轴的零位错误被屏蔽，必须同时屏蔽这些轴的软限位校验
    OffsetMask mask[NUM_OF_JOINT] = {OFFSET_UNMASK};
    calibrator_.getOffsetMask(mask);

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        if (mask[i] == OFFSET_MASKED)
        {
            soft_constraint_.setMask(i);
        }
    }

    // 初始化坐标变换模块
    if (!transformation_.init(kinematics_ptr_))
    {
        FST_ERROR("Fail to init transformation for ArmGroup.");
        return MC_INTERNAL_FAULT;
    }

    // 初始化手动示教模块
    FST_INFO("Initializing manual teach of ArmGroup ...");
    err = manual_teach_.init(kinematics_ptr_, dynamics_ptr_, &soft_constraint_, log_ptr_, path + "arm_manual_teach.yaml");

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to initialize manual teach, code = 0x%llx", err);
        return err;
    }

    manual_teach_.setGlobalVelRatio(vel_ratio_);
    manual_teach_.setGlobalAccRatio(acc_ratio_);

    double omega[JOINT_OF_ARM] = {0};
    param.reset();

    if (!param.loadParamFile(path + "arm_group.yaml"))
    {
        FST_ERROR("Fail to load config file of arm group, code = 0x%llx", param.getLastError());
        return param.getLastError();
    }

    if (!param.getParam("joint/omega", omega, JOINT_OF_ARM))
    {
        FST_ERROR("Fail to load max velocity of each axis, code = 0x%llx", param.getLastError());
        return param.getLastError();
    }
    
    FST_INFO("Max velocity of each axis: %s", printDBLine(omega, buffer, LOG_TEXT_SIZE));

    int types[JOINT_OF_ARM] = {0};

    if (!param.getParam("type_of_axis", types, JOINT_OF_ARM))
    {
        FST_ERROR("Fail to load types of each axis, code = 0x%llx", param.getLastError());
        return param.getLastError();
    }

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        type_of_axis_[i] = AxisType(types[i]);
    }

    FST_INFO("Types of each axis: %s", printDBLine(types, buffer, LOG_TEXT_SIZE));
    FST_WARN("ArmGroup init success.");
    return SUCCESS;
}


size_t ArmGroup::getNumberOfJoint(void)
{
    return JOINT_OF_ARM;
}


bool ArmGroup::isPostureMatch(const basic_alg::Posture &posture_1, const basic_alg::Posture &posture_2)
{
    return (posture_1.arm == posture_2.arm) && (posture_1.elbow == posture_2.elbow) && (posture_1.wrist == posture_2.wrist);
}


char* ArmGroup::printDBLine(const int *data, char *buffer, size_t length)
{
    size_t len = 0;

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        len += snprintf(buffer + len, length - len, "%d ", data[i]);

        if (len >= length)
        {
            break;
        }
    }

    return buffer;
}

char* ArmGroup::printDBLine(const double *data, char *buffer, size_t length)
{
    size_t len = 0;

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        len += snprintf(buffer + len, length - len, "%.6f ", data[i]);
        //len += snprintf(buffer + len, length - len, "%.9f ", data[i]);
        //len += snprintf(buffer + len, length - len, "%.12f ", data[i]);

        if (len >= length)
        {
            break;
        }
    }

    return buffer;
}





}




