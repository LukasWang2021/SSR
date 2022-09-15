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
#include <error_queue.h>
#include <common_file_path.h>
#include <dynamic_alg_rtm.h>
#include "yaml_help.h"
#include "log_manager_producer.h"

using namespace std;
using namespace fst_ctrl;
using namespace base_space;
using namespace basic_alg;
using namespace log_space;
using namespace hal_space;


namespace group_space
{

ErrorCode ArmGroup::initGroup(CoordinateManager *coordinate_manager_ptr, ToolManager *tool_manager_ptr,
    std::map<int32_t,axis_space::Axis*>* axis_group_ptr, GroupSm* sm_ptr, servo_comm_space::ServoCpuCommBase* cpu_comm_ptr,
    system_model_space::GroupModel_t* db_ptr, BaseDevice *fio_dev_ptr)
{
    vel_ratio_ = 0.1;
    acc_ratio_ = 1.0;
    cycle_time_ = 0.001;
    id_ = 1;
    fio_ptr_ = fio_dev_ptr;
    
    if (coordinate_manager_ptr == NULL || tool_manager_ptr == NULL || axis_group_ptr == NULL 
        || sm_ptr == NULL || cpu_comm_ptr == NULL)
    {
        LogProducer::error("mc_arm_group","Invalid pointer of error-monitor or coordinate-manager or tool-manager.");
        return MC_INTERNAL_FAULT;
    }

    coordinate_manager_ptr_ = coordinate_manager_ptr;
    tool_manager_ptr_ = tool_manager_ptr;
    axis_group_ptr_ = axis_group_ptr;
    sm_ptr_ = sm_ptr;
    cpu_comm_ptr_ = cpu_comm_ptr;
    db_ptr_ = db_ptr;

    if (pthread_mutex_init(&planner_list_mutex_, NULL) != 0 ||
        pthread_mutex_init(&manual_traj_mutex_, NULL) != 0 ||
        pthread_mutex_init(&servo_mutex_, NULL) != 0 ||
        pthread_mutex_init(&offline_mutex_, NULL) != 0 ||
        pthread_mutex_init(&manual_rpc_mutex_, NULL) != 0)
    {
        LogProducer::error("mc_arm_group","Fail to initialize mutex.");
        return MC_INTERNAL_FAULT;
    }

    // 初始化运动学模块
    LogProducer::info("mc_arm_group","Initializing kinematics of ArmGroup ...");
    string path = AXIS_GROUP_DIR;
    kinematics_ptr_ = new KinematicsRTM(path);

    if (kinematics_ptr_ == NULL || !kinematics_ptr_->isValid())
    {
        LogProducer::error("mc_arm_group","Fail to create kinematics(%p, %d) for this Group.",kinematics_ptr_, kinematics_ptr_->isValid());
        return MC_INTERNAL_FAULT;
    }

    // 初始化动力学模块
    LogProducer::info("mc_arm_group","Initializing dynamics of ArmGroup ...");
    dynamics_ptr_ = new DynamicAlgRTM();

    if (dynamics_ptr_ == NULL)
    {
        LogProducer::error("mc_arm_group","Fail to create dynamics for ArmGroup.");
        return MC_INTERNAL_FAULT;
    }

    if (!dynamics_ptr_->initDynamicAlg(path))
    {
        LogProducer::error("mc_arm_group","Fail to init dynamics for ArmGroup.");
        return MC_FAIL_IN_INIT;
    }

    // 加载限位文件参数
    ErrorCode result = LoadConstraintParameters(path);
    if (result != SUCCESS) return result;

    // 加载motion_control参数设置
    result = LoadBaseGroupParameters(path);
    if (result != SUCCESS) return result;

    // 加载arm group文件参数
    result = LoadArmGroupParameters(path);
    if (result != SUCCESS) return result;

    LogProducer::info("mc_arm_group","Load parameters success.");

    // 初始化轨迹缓存
    LogProducer::info("mc_arm_group","Initializing trajectory planner.");
    trajectory_a_.prev = &trajectory_b_;
    trajectory_a_.next = &trajectory_b_;
    trajectory_b_.prev = &trajectory_a_;
    trajectory_b_.next = &trajectory_a_;
    trajectory_a_.valid = false;
    trajectory_b_.valid = false;
    
    if (!planner_for_check_.initPlanner(JOINT_OF_ARM, cycle_time_, kinematics_ptr_, dynamics_ptr_, &soft_constraint_) ||
        !trajectory_a_.trajectory.initPlanner(JOINT_OF_ARM, cycle_time_, kinematics_ptr_, dynamics_ptr_, &soft_constraint_) ||
        !trajectory_b_.trajectory.initPlanner(JOINT_OF_ARM, cycle_time_, kinematics_ptr_, dynamics_ptr_, &soft_constraint_))
    {
        LogProducer::error("mc_arm_group","Fail to initialize trajectory planner.");
        return MC_FAIL_IN_INIT;
    }

    uint32_t jerk_num;
    Joint omega_limit, alpha_limit, beta_limit[MAX_JERK_NUM];
    trajectory_a_.trajectory.getTrajectoryLimit(omega_limit, alpha_limit, beta_limit, jerk_num);
    trajectory_a_.smooth.initPlanner(JOINT_OF_ARM, cycle_time_, kinematics_ptr_, dynamics_ptr_, &soft_constraint_);
    trajectory_b_.smooth.initPlanner(JOINT_OF_ARM, cycle_time_, kinematics_ptr_, dynamics_ptr_, &soft_constraint_);
    // trajectory_a_.smooth.setLimit(2000, 10000, 100000, 1, 10, 100);
    // trajectory_b_.smooth.setLimit(2000, 10000, 100000, 1, 10, 100);

    plan_traj_ptr_ = &trajectory_a_;
    pick_traj_ptr_ = &trajectory_a_;

    joint_planner_.initPlanner(JOINT_OF_ARM, 1);
    joint_planner_.setLimit(omega_limit, alpha_limit, beta_limit);

    //初始化暂停恢复轨迹规划模块
    if(!pause_resume_planner_.initPausePlanner(JOINT_OF_ARM))
    {
        LogProducer::error("mc_arm_group","Fail to initialize Pause trajectory planner.");
        return MC_FAIL_IN_INIT;
    }

    // 初始化裸核通信句柄
    LogProducer::info("mc_arm_group","Initializing interface to bare core ...");

    if (!bare_core_.initInterface(JOINT_OF_ARM, axis_group_ptr_, sm_ptr_, cpu_comm_ptr_, db_ptr_))
    {
        LogProducer::error("mc_arm_group","Fail to create communication with bare core.");
        return MC_FAIL_IN_INIT;
    }

    // 初始化零位校验模块
    LogProducer::info("mc_arm_group","Initializing calibrator of ArmGroup ...");
    ErrorCode err = calibrator_.initCalibrator(JOINT_OF_ARM, &bare_core_);

    if (err != SUCCESS)
    {
        LogProducer::error("mc_arm_group","Fail to initialize calibrator, code = 0x%llx", err);
        return err;
    }
#if 0
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

    //启动检查零位是否跑偏
    CalibrateState calib_state;
    OffsetState offset_state[NUM_OF_JOINT];
    LogProducer::info("mc_arm_group","calibrator checkOffset start");
    err = calibrator_.checkOffset(calib_state, offset_state);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_arm_group","calibrator checkOffset failed, code = 0x%llx", err);
        return err;
    }
    if(calib_state != MOTION_NORMAL)
    {
        for(int i=0; i < JOINT_OF_ARM; ++i)
        {
            if(offset_state[i] == OFFSET_LOST)
            {
                ErrorQueue::instance().push(ZERO_OFFSET_LOST);
            }
            else if(offset_state[i] == OFFSET_INVALID)
            {
                ErrorQueue::instance().push(ZERO_OFFSET_INVALID);
            }
        }
        if(calib_state == MOTION_FORBIDDEN)
        {
            ErrorQueue::instance().push(CONTROLLER_OFFSET_NEED_CALIBRATE);
        }
    }
#endif
    // 初始化坐标变换模块
    if (!transformation_.init(kinematics_ptr_))
    {
        LogProducer::error("mc_arm_group","Fail to init transformation for ArmGroup.");
        return MC_INTERNAL_FAULT;
    }

    // 初始化手动示教模块
    LogProducer::info("mc_arm_group","Initializing manual teach of ArmGroup ...");
    err = manual_teach_.init(kinematics_ptr_, dynamics_ptr_, &soft_constraint_, path + "arm_manual_teach.yaml");

    if (err != SUCCESS)
    {
        LogProducer::error("mc_arm_group","Fail to initialize manual teach, code = 0x%llx", err);
        return err;
    }

    manual_teach_.setGlobalVelRatio(vel_ratio_);
    manual_teach_.setGlobalAccRatio(acc_ratio_);

    if (getServoState() == SERVO_IDLE) 
    {
        updateStartJoint();
    }
    LogProducer::info("mc_arm_group","ArmGroup init success.");
    return SUCCESS;
}

ErrorCode ArmGroup::checkGroupZeroOffset(void)
{
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

    //启动检查零位是否跑偏
    CalibrateState calib_state;
    OffsetState offset_state[NUM_OF_JOINT];
    LogProducer::info("mc_arm_group","calibrator checkOffset start");
    ErrorCode err = calibrator_.checkOffset(calib_state, offset_state);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_arm_group","calibrator checkOffset failed, code = 0x%llx", err);
        return err;
    }
    if(calib_state != MOTION_NORMAL)
    {
        for(int i=0; i < JOINT_OF_ARM; ++i)
        {
            if(offset_state[i] == OFFSET_LOST)
            {
                ErrorQueue::instance().push(ZERO_OFFSET_LOST);
            }
            else if(offset_state[i] == OFFSET_INVALID)
            {
                ErrorQueue::instance().push(ZERO_OFFSET_INVALID);
            }
        }
        if(calib_state == MOTION_FORBIDDEN)
        {
            ErrorQueue::instance().push(CONTROLLER_OFFSET_NEED_CALIBRATE);
        }
    }
    return 0;
}

char* ArmGroup::getModelName(char *buffer, size_t length)
{
    snprintf(buffer, length, "RTM-P7A");
    return buffer;
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

ErrorCode ArmGroup::LoadConstraintParameters(std::string dir_path)
{
    base_space::YamlHelp hard_constraint_param;
    vector<double> upper;
    vector<double> lower;

    // 加载硬限位
    LogProducer::info("mc_arm_group","Loading hard constraints ...");

    if (hard_constraint_param.loadParamFile(dir_path + "hard_constraint.yaml") &&
        hard_constraint_param.getParam("hard_constraint/upper", upper) &&
        hard_constraint_param.getParam("hard_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            hard_constraint_.initConstraint(*(Joint*)(&lower[0]), *(Joint*)(&upper[0]), JOINT_OF_ARM);
        }
        else
        {
            LogProducer::error("mc_arm_group","Invalid array size of hard constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading hard constraint from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    // 加载软限位上下限
    LogProducer::info("mc_arm_group","Loading firm constraints ...");
    upper.clear();
    lower.clear();

    base_space::YamlHelp firm_constraint_param;
    if (firm_constraint_param.loadParamFile(dir_path + "firm_constraint.yaml") &&
        firm_constraint_param.getParam("firm_constraint/upper", upper) &&
        firm_constraint_param.getParam("firm_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            firm_constraint_.initConstraint(*(Joint*)(&lower[0]), *(Joint*)(&upper[0]), JOINT_OF_ARM);
        }
        else
        {
            LogProducer::error("mc_arm_group","Invalid array size of firm constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading firm constraint from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    // 加载软限位
    LogProducer::info("mc_arm_group","Loading soft constraints ...");
    upper.clear();
    lower.clear();

    base_space::YamlHelp soft_constraint_param;
    if (soft_constraint_param.loadParamFile(dir_path + "soft_constraint.yaml") &&
        soft_constraint_param.getParam("soft_constraint/upper", upper) &&
        soft_constraint_param.getParam("soft_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            soft_constraint_.initConstraint(*(Joint*)(&lower[0]), *(Joint*)(&upper[0]), JOINT_OF_ARM);
        }
        else
        {
            LogProducer::error("mc_arm_group","Invalid array size of soft constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading soft constraint from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_arm_group","Hard/firm/soft constraints from axis-%d to axis-%d:", 0, JOINT_OF_ARM - 1);
    LogProducer::info("mc_arm_group","  hard-lower: %s", printDBLine(&hard_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_arm_group","  firm-lower: %s", printDBLine(&firm_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_arm_group","  soft-lower: %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_arm_group","  soft-upper: %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_arm_group","  firm-upper: %s", printDBLine(&firm_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_arm_group","  hard-upper: %s", printDBLine(&hard_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}

ErrorCode ArmGroup::LoadBaseGroupParameters(std::string dir_path)
{
    // 加载motion_control参数设置
    base_space::YamlHelp base_group_param;
    if (!base_group_param.loadParamFile(dir_path + "base_group.yaml"))
    {
        LogProducer::error("mc_arm_group","Fail loading motion configuration from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    if (!base_group_param.getParam("output_trajectory", traj_log_enable_))
    {
        LogProducer::error("mc_arm_group","Fail loading trajectory output of/off config");
        return MC_LOAD_PARAM_FAILED;
    }

    LogProducer::info("mc_arm_group","Trajectory output: %s", traj_log_enable_ ? "true" : "false");

    if (traj_log_enable_)
    {
        traj_log_enable_ = initTrajectoryLogSpace();
    }

    // 从配置文件加载轨迹FIFO的容量，并初始化轨迹FIFO
    int traj_fifo_config;

    if (!base_group_param.getParam("trajectory_fifo/fifo_size", traj_fifo_config))
    {
        LogProducer::error("mc_arm_group","Fail loading trajectory FIFO size from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    LogProducer::info("mc_arm_group","Initializing trajectory FIFO ... capacity = %d", traj_fifo_config);

    if (!traj_fifo_.init(traj_fifo_config))
    {
        LogProducer::error("mc_arm_group","Fail to initialize trajectory FIFO.");
        return MC_INTERNAL_FAULT;
    }

    if (!base_group_param.getParam("trajectory_fifo/lower_limit", traj_fifo_config))
    {
        LogProducer::error("mc_arm_group","Fail loading trajectory FIFO lower limit size from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    traj_fifo_lower_limit_ = traj_fifo_config;
    LogProducer::info("mc_arm_group","Trajectory FIFO lower limit size = %d", traj_fifo_lower_limit_);

    if (!base_group_param.getParam("trajectory_fifo/manual_fifo_size", traj_fifo_config))
    {
        LogProducer::error("mc_arm_group","Fail loading manual trajectory FIFO size from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    LogProducer::info("mc_arm_group","Initializing manual trajectory FIFO ... capacity = %d", traj_fifo_config);

    if (!manual_fifo_.init(traj_fifo_config))
    {
        LogProducer::error("mc_arm_group","Fail to initialize manual trajectory FIFO.");
        return MC_INTERNAL_FAULT;
    }
    if (!base_group_param.getParam("trajectory_fifo/online_fifo_size", traj_fifo_config))
    {
        LogProducer::error("mc_arm_group","Fail loading online trajectory FIFO size from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    LogProducer::info("mc_arm_group","Initializing Online trajectory FIFO ... capacity = %d", traj_fifo_config);

    if (!online_fifo_.init(traj_fifo_config))
    {
        LogProducer::error("mc_arm_group","Fail to initialize online trajectory FIFO.");
        return MC_INTERNAL_FAULT;
    }

        // 从配置文件加载Fine指令的稳定周期和稳定门限
    int     stable_times;
    double  stable_threshold;

    if (base_group_param.getParam("stable_with_fine/cycle", stable_times) && base_group_param.getParam("stable_with_fine/threshold", stable_threshold))
    {
        if (stable_times > 0 && stable_threshold > 0)
        {
            fine_cycle_ = stable_times;
            fine_threshold_ = stable_threshold;
            LogProducer::info("mc_arm_group","Fine settings: cycle = %d, threshold = %.4f", stable_times, stable_threshold);
        }
        else
        {
            LogProducer::error("mc_arm_group","Invalid fine settings: cycle = %d, threshold = %.4f", stable_times, stable_threshold);
            return INVALID_PARAMETER;
        }
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading fine setings from config file");
        return MC_LOAD_PARAM_FAILED;
    }
    
    // 从配置文件加载笛卡尔空间线速度上下限
    if (base_group_param.getParam("cartesian_vel_limit/min", cartesian_vel_min_) && base_group_param.getParam("cartesian_vel_limit/max", cartesian_vel_max_))
    {
        LogProducer::info("mc_arm_group","Cartesian velocity: %.2f ~ %.2f", cartesian_vel_min_, cartesian_vel_max_);
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading cartesian velocity limit from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    // 从配置文件加载状态机中各个临时状态的超时时间
    int time_out;

    if (base_group_param.getParam("time_out_cycle/pause_to_standby", time_out) && time_out > 0)
    {
        pause_to_standby_timeout_ = time_out;
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading pause->standby timeout from config file");
        return MC_LOAD_PARAM_FAILED;
    }
    
    if (base_group_param.getParam("time_out_cycle/disable_to_standby", time_out) && time_out > 0)
    {
        disable_to_standby_timeout_ = time_out;
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading disable->standby timeout from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    if (base_group_param.getParam("time_out_cycle/standby_to_disable", time_out) && time_out > 0)
    {
        standby_to_disable_timeout_ = time_out;
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading standby->disable timeout from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    if (base_group_param.getParam("time_out_cycle/standby_to_auto", time_out) && time_out > 0)
    {
        standby_to_auto_timeout_ = time_out;
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading standby->auto timeout from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    if (base_group_param.getParam("time_out_cycle/auto_to_standby", time_out) && time_out > 0)
    {
        auto_to_standby_timeout_ = time_out;
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading auto->standby timeout from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    if (base_group_param.getParam("time_out_cycle/manual_to_standby", time_out) && time_out > 0)
    {
        manual_to_standby_timeout_ = time_out;
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading manual->standby timeout from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    if (base_group_param.getParam("time_out_cycle/offline_to_standby", time_out) && time_out > 0)
    {
        offline_to_standby_timeout_ = time_out;
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading offline->standby timeout from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    if (base_group_param.getParam("time_out_cycle/auto_to_pause", time_out) && time_out > 0)
    {
        auto_to_pause_timeout_ = time_out;
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading auto->pause timeout from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    if (base_group_param.getParam("time_out_cycle/trajectory_flow", time_out) && time_out > 0)
    {
        trajectory_flow_timeout_ = time_out;
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading trajectory flow timeout from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    int cycles;
    if (base_group_param.getParam("joint_record/update_cycle", cycles) && cycles > 0)
    {
        joint_record_update_cycle_ = cycles;
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading record cycle from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    if (base_group_param.getParam("joint_record/update_timeout", cycles) && cycles > 0)
    {
        joint_record_update_timeout_ = cycles;
    }
    else
    {
        LogProducer::error("mc_arm_group","Fail loading record timeout from config file");
        return MC_LOAD_PARAM_FAILED;
    }

    // 从配置文件中加载关节位置跟随误差门限
    double tracking_accuracy[JOINT_OF_ARM];

    if (!base_group_param.getParam("joint_tracking_accuracy", tracking_accuracy, JOINT_OF_ARM))
    {
        LogProducer::error("mc_arm_group","Fail to load joint tracking error threshold for each axis");
        return MC_LOAD_PARAM_FAILED;
    }

    memcpy(&joint_tracking_accuracy_.j1_, tracking_accuracy, sizeof(tracking_accuracy));

    return SUCCESS;
}
ErrorCode ArmGroup::LoadArmGroupParameters(std::string dir_path)
{
    base_space::YamlHelp arm_group_param;
    double omega[JOINT_OF_ARM] = {0};
    if (!arm_group_param.loadParamFile(dir_path + "arm_group.yaml"))
    {
        LogProducer::error("mc_arm_group","Fail to load config file of arm group");
        return MC_LOAD_PARAM_FAILED;
    }

    if (!arm_group_param.getParam("joint/omega", omega, JOINT_OF_ARM))
    {
        LogProducer::error("mc_arm_group","Fail to load max velocity of each axis");
        return MC_LOAD_PARAM_FAILED;
    }
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_arm_group","Max velocity of each axis: %s", printDBLine(omega, buffer, LOG_TEXT_SIZE));

    int types[JOINT_OF_ARM] = {0};

    if (!arm_group_param.getParam("type_of_axis", types, JOINT_OF_ARM))
    {
        LogProducer::error("mc_arm_group","Fail to load types of each axis");
        return MC_LOAD_PARAM_FAILED;
    }

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        type_of_axis_[i] = AxisType(types[i]);
    }

    LogProducer::info("mc_arm_group","Types of each axis: %s", printDBLine(types, buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


}




