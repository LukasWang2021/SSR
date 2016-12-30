/**********************************************************************
    Copyright:  Foresight-Robotics
    File:       fst_controller.cpp
    Author:     Feng Yun
    Data:       Aug.1  2016
    Modify:     Aug.30 2016
    Description:ArmGroup--Source code.
**********************************************************************/


#include <motion_controller/arm_group.h>
#include <struct_to_mem/struct_feedback_joint_states.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <ros/ros.h>
#include <XmlRpc.h>
#include <assert.h>

using std::cout;
using std::endl;

static const unsigned int UNDEFINED     = 0x5555;
static const unsigned int INITIALIZED   = 0x5556;

// -------------------------------------public function--------------------------------------------



//------------------------------------------------------------------------------
// Function:    ArmGroup
// Summary: The constructor of class
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
fst_controller::ArmGroup::ArmGroup(void) {
    planner_    = NULL;
    calibrator_ = NULL;
    current_state_      = UNDEFINED;
    enable_calibration_ = false;
    m_next_cmd_type     = enum_Type_Other;
    m_point_level       = enum_Level_Start;
    m_suspend_state.is_suspended    = false;

    m_velocity_range.max        = 0.0;
    m_velocity_range.min        = 0.0;
    m_acceleration_range.max    = 0.0;
    m_acceleration_range.min    = 0.0;
    m_cycle_time_range.max      = 0.0;
    m_cycle_time_range.min      = 0.0;
    m_JOINT_FIFO_LEN            = 0;
    m_vmax_scaling_factor       = 0.0;
    m_amax_scaling_factor       = 0.0;
    m_jump_threshold_scaling    = 0.0;
    m_cycle_time    = 0.0;
    m_vmax          = 0.0;
    m_amax          = 0.0;
    m_vu_start      = 0.0;
    m_v_start       = 0.0;

    memset((void*)&m_tool_frame,    0, sizeof(m_tool_frame));
    memset((void*)&m_user_frame,    0, sizeof(m_user_frame));
    memset((void*)&m_pose_start,    0, sizeof(m_pose_start));
    memset((void*)&m_pose_previous, 0, sizeof(m_pose_previous));
    memset((void*)&m_joint_start,   0, sizeof(m_joint_start));
    memset((void*)&m_joint_constraints,   0, sizeof(m_joint_constraints));
    memset((void*)&m_latest_ik_reference, 0, sizeof(m_latest_ik_reference));
    memset((void*)&m_joint_state_current, 0, sizeof(m_joint_state_current));
    memset((void*)&m_pose_state_current,  0, sizeof(m_pose_state_current));

    m_planned_path_fifo.resize(0);
    m_joint_trajectory_fifo.resize(0);

    //m_log_file_name = "";
    //m_log_file_content = "";

    return;
}


//------------------------------------------------------------------------------
// Function:    ~ArmGroup
// Summary: The destructor of class
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
fst_controller::ArmGroup::~ArmGroup(void) {
    if (calibrator_->getCurrentState() == CALIBRATED)
        if (!calibrator_->recordLastJoint()) {
            log.error("Fail to record last joint, error code=0x%llx", calibrator_->getLastError());
        }
    /*
    if (m_log_file_content.size() > 0) {
        std::ofstream log_file_handle(m_log_file_name.c_str(), ios::app);
        log_file_handle << m_log_file_content;
        log_file_handle.close();
        m_log_file_content.clear();
    }
    */
    pthread_mutex_destroy(&m_group_mutex);
    delete planner_;    planner_   = NULL;
    delete calibrator_; calibrator_ = NULL;
    log.warn("ArmGroup exit");
}


//------------------------------------------------------------------------------
// Function:    initArmGroup
// Summary: Initial all the resources in the class
// In:      joint_values -> initial state of all joints
// Out:     error_code   -> error code
// Return:  None
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::initArmGroup(ErrorCode &err) {
    err = SUCCESS;
    
    if (current_state_ != UNDEFINED) {
        err = MOTION_FAIL_IN_INIT;
        return false;
    }

    log.info("Initializing ArmGroup...");

    log.info("Constructing ArmGroup logger ...");
    if (!log.initLogger("ArmGroup")) {
        err = MOTION_FAIL_IN_INIT;
        log.error("Cannot construct ArmGroup log file, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        // return false;
    }
    else {
        log.info("Success!");
    }
    
    current_state_ = INITIALIZED;
    pthread_mutex_init(&m_group_mutex, NULL);
    
    m_planned_path_fifo.reserve(5000);
    m_joint_trajectory_fifo.reserve(200);

    m_suspend_state.is_suspended = false;
    // m_suspend_state.pattern = e_suspend_pattern_origin;
    // m_suspend_state.last_point = joint_values;
    m_suspend_state.fifo2_cache.reserve(200);
    m_suspend_state.replan_trajectory.reserve(400);
    
    if (!loadArmGroupParameters(err)) {
        log.error("Fail loading parameters from remote server, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    
    log.info("Constructing planner and calibrator ...");
    
    if (calibrator_ != NULL) {
        delete calibrator_;
        calibrator_ = NULL;
    }
    if (planner_ != NULL) {
        delete planner_;
        planner_ = NULL;
    }
    calibrator_ = new fst_controller::Calibrator();
    planner_    = new fst_controller::TrajPlan();
    if (planner_ == NULL || calibrator_ == NULL) {
        err = MOTION_FAIL_IN_INIT;
        log.error("Cannot construct planner or calibrator, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    log.info("Success!");

    setCycleTime(m_cycle_time);
    setMaxVelocity((m_velocity_range.max) / 2);
    setMaxAcceleration((m_acceleration_range.max) / 2);
    setVelocityScalingFactor(1.0);
    setAccelerationScalingFactor(1.0);
    setJumpThresholdScalingFactor(1.0);

    JointConstraints constraint;
    if (loadJointConstraints(constraint, err)) {
        setJointConstraints(constraint);
    }
    else {
        log.error("Error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    
    log.info("Initializing calibrator ...");
    bool res = calibrator_->initCalibrator("share/motion_controller/config/jtac.yaml", 
                                           "share/motion_controller/config/robot_recorder.yaml");        
    if (!res) {
        err = calibrator_->getLastError();
        log.error("Cannot initialize calibrator, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    
    log.info("Downloading JTAC parameters ...");
    if (!calibrator_->transmitJtacParam()) {
        err = calibrator_->getLastError();
        log.error("Download parameter failed, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    log.info("Success!");
    usleep(256 * 1000);
    
    log.info("Using current joint values to initialize ArmGroup:");
    FeedbackJointState fbjs;
    calibrator_->getCurrentJoint(fbjs);
    JointValues joint;
    joint.j1 = fbjs.position[0];
    joint.j2 = fbjs.position[1];
    joint.j3 = fbjs.position[2];
    joint.j4 = fbjs.position[3];
    joint.j5 = fbjs.position[4];
    joint.j6 = fbjs.position[5];
    printJointValues("  joints=", joint);
    if (!setStartState(joint, err)) {
        log.error("Cannot set start state with current joint values, error code=0x%llx", err);
        if (err == JOINT_OUT_OF_CONSTRAINT) {
            log.warn("Reset zero offset to a temporary position, need calibration.");
            if (calibrator_->setTemporaryZeroOffset()) {
                log.info("Success!");
                calibrator_->getCurrentJoint(fbjs);
                JointValues joint;
                joint.j1 = fbjs.position[0];
                joint.j2 = fbjs.position[1];
                joint.j3 = fbjs.position[2];
                joint.j4 = fbjs.position[3];
                joint.j5 = fbjs.position[4];
                joint.j6 = fbjs.position[5];
                printJointValues("  joints=", joint);
                if (!setStartState(joint, err)) {
                    current_state_ = UNDEFINED;
                    log.error("Cannot set start state with current joint values, error code=0x%llx", err);
                    return false;
                }
            }
            else {
                log.error("Cannot set temporary zero offset");
                current_state_ = UNDEFINED;
                return false;
            }
        }
        else {
            current_state_ = UNDEFINED;
            return false;
        }
    }
    
    if (err == SUCCESS) {
        log.info("ArmGroup is ready to make life easier. Have a good time!");
        log.info("**********************************************************************************************");
        return true;
    }
    else {
        log.warn("ArmGroup initialized with errors, error code=0x%llx.", err);
        log.info("**********************************************************************************************");
        return false;
    }
}

bool fst_controller::ArmGroup::checkZeroOffset(unsigned int &calibrate_result, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    err = SUCCESS;
    if (enable_calibration_ == true) {
        log.info("Reviewing starting joint ...");
        unsigned int result;
        if (calibrator_->reviewLastJoint(result)) {
            calibrate_result = result;
            if (result == OFFSET_NORMAL) {
                log.info("Success!");
            }
            else {
                if ((result & OFFSET_LOST_MASK) != 0) {
                    err = ZERO_OFFSET_LOST;
                    log.error("Fault: zero offset lost, need calibration. Error code=0x%llx", err);
                }
                else if ((result & OFFSET_DEVIATE_MASK) != 0) {
                    err = ZERO_OFFSET_DEVIATE;
                    log.error("Fault: zero offset deviated, need calibration. Error code=0x%llx", err);
                }
                else {
                    err = MOTION_INTERNAL_FAULT;
                    log.error("Program internal fault, error code=0x%llx", err);
                }
            }
        }
        else {
            err = calibrator_->getLastError();
            log.error("Fail, error code=0x%llx", err);
        }
    }
    else {
        err = CALIBRATION_FAULT;
        log.error("Fail, calibration is disabled, error code=0x%llx", err);
    }
    
    log.info("Using current joint values to initialize ArmGroup:");
    FeedbackJointState fbjs;
    calibrator_->getCurrentJoint(fbjs);
    JointValues joint;
    memcpy(&joint, fbjs.position, 6 * sizeof(double));
    printJointValues("  joints=", joint);
    setStartState(joint, err);

    if (err == SUCCESS)
        return true;
    else
        return false;
}

bool fst_controller::ArmGroup::calibrateZeroOffset(unsigned int &calibrate_result, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    if (enable_calibration_ != true) {
        err = CALIBRATION_FAULT;
        return false;
    }

    err = SUCCESS;
    if (!calibrator_->recordZeroOffset()) {
        err = calibrator_->getLastError();
        return false;
    }
    if (!calibrator_->reloadJTACParam()) {
        err = calibrator_->getLastError();
        return false;
    }

    log.info("Downloading JTAC parameters ...");
    if (!calibrator_->transmitJtacParam()) {
        err = calibrator_->getLastError();
        log.error("Download parameter failed, error code=0x%llx", err);
        // current_state_ = UNDEFINED;
        return false;
    }
    log.info("Success!");
    usleep(256 * 1000);

    // do calibrate again
    log.info("Reviewing starting joint ...");
    unsigned int result;
    if (calibrator_->reviewCalibratedJoint(result)) {
        calibrate_result = result;
        if (result == OFFSET_NORMAL) {
            log.info("Success!");
        }
        else {
            if ((result & OFFSET_LOST_MASK) != 0) {
                err = ZERO_OFFSET_LOST;
                log.error("Fault: zero offset lost, need calibration. Error code=0x%llx", err);
                return false;
            }
            else if ((result & OFFSET_DEVIATE_MASK) != 0) {
                err = ZERO_OFFSET_DEVIATE;
                log.error("Fault: zero offset deviated, need calibration. Error code=0x%llx", err);
                return false;
            }
            else {
                err = MOTION_INTERNAL_FAULT;
                log.error("Program internal fault, error code=0x%llx", err);
                return false;
            }
        }
    }
    else {
        err = calibrator_->getLastError();
        log.error("Fail, error code=0x%llx", err);
        return false;
    }

    return true;
}

bool fst_controller::ArmGroup::loadArmGroupParameters(ErrorCode &err) {
    log.info("Loading ArmGroup configs from parameter server ...");
    err = SUCCESS;
    
    XmlRpc::XmlRpcValue params;
    if (ros::param::get("/fst_param/motion_controller/arm_group/", params))
    {
        try {
            if (params.hasMember("enable_calibration")) {
                enable_calibration_ = params["enable_calibration"];
            }
            else
                err = FAIL_LOADING_PARAMETER;

            if (params.hasMember("cartesian_velocity_max")) {
                m_velocity_range.max = params["cartesian_velocity_max"];
            }
            else
                err = FAIL_LOADING_PARAMETER;

            if (params.hasMember("cartesian_velocity_min")) {
                m_velocity_range.min = params["cartesian_velocity_min"];
            }
            else
                err = FAIL_LOADING_PARAMETER;

            if (params.hasMember("cartesian_acceleration_max")) {
                m_acceleration_range.max = params["cartesian_acceleration_max"];
            }
            else
                err = FAIL_LOADING_PARAMETER;

            if (params.hasMember("cartesian_acceleration_min")) {
                m_acceleration_range.min = params["cartesian_acceleration_min"];
            }
            else
                err = FAIL_LOADING_PARAMETER;
            
            if (err == SUCCESS) {
                m_velocity_range.max *= 1000;
                m_velocity_range.min *= 1000;
                m_acceleration_range.max *= 1000;
                m_acceleration_range.min *= 1000;
            }
            
            if (params.hasMember("cycle_time_max")) {
                m_cycle_time_range.max = params["cycle_time_max"];
            }
            else
                err = FAIL_LOADING_PARAMETER;

            if (params.hasMember("cycle_time_min")) {
                m_cycle_time_range.min = params["cycle_time_min"];
            }
            else
                err = FAIL_LOADING_PARAMETER;

            if (params.hasMember("cycle_time")) {
                m_cycle_time = params["cycle_time"];
                // if (!setCycleTime(params["cycle_time"]))
                //     err = INVALID_PARAMETER;
            }
            else
                err = FAIL_LOADING_PARAMETER;

            if (params.hasMember("joint_fifo_length_max")) {
                m_JOINT_FIFO_LEN = params["joint_fifo_length_max"];
            }
            else
                err = FAIL_LOADING_PARAMETER;
        }
        catch (XmlRpc::XmlRpcException &exception) {
            err = INVALID_PARAMETER;
            log.error("Exception:");
            log.error(exception.getMessage().c_str());
        }
    }
    else {
        err = FAIL_LOADING_PARAMETER;
    }

    if (err == SUCCESS) {
        log.info("Success!");
        return true;
    }
    else {
        log.error("Cannot load parameters from Remote server, error code=0x%llx", err);
        return false;
    }
}



//------------------------------------------------------------------------------
// Function:    getCycleTime
// Summary: To get cycle time of interpolation algorithm.
// In:      None
// Out:     None
// Return:  cycle time
//------------------------------------------------------------------------------
double fst_controller::ArmGroup::getCycleTime(void) {
    return m_cycle_time;
}


//------------------------------------------------------------------------------
// Function:    getJointConstraints
// Summary: To get joint constraints from Kinematics algorithm.
// In:      None
// Out:     None
// Return:  joint constraints
//------------------------------------------------------------------------------
const fst_controller::JointConstraints& fst_controller::ArmGroup::getJointConstraints(void) {
    return m_joint_constraints;
}


//------------------------------------------------------------------------------
// Function:    getMaxVelocity
// Summary: To get max velocity settings.
// In:      None
// Out:     None
// Return:  value of max velocity
//------------------------------------------------------------------------------
double fst_controller::ArmGroup::getMaxVelocity(void) {
    return m_vmax;
}


//------------------------------------------------------------------------------
// Function:    getMaxAcceleration
// Summary: To get max acceleration settings.
// In:      None
// Out:     None
// Return:  value of max acceleration
//------------------------------------------------------------------------------
double fst_controller::ArmGroup::getMaxAcceleration(void) {
    return m_amax;
}


//------------------------------------------------------------------------------
// Function:    getVelocityScalingFactor
// Summary: To get velocity scaling factor value.
// In:      None
// Out:     None
// Return:  global scaling factor for velocity
//------------------------------------------------------------------------------
double fst_controller::ArmGroup::getVelocityScalingFactor(void) {
    return m_vmax_scaling_factor;
}


//------------------------------------------------------------------------------
// Function:    getAccelerationScalingFactor
// Summary: To get acceleration scaling factor value.
// In:      None
// Out:     None
// Return:  global scaling factor for acceleration
//------------------------------------------------------------------------------
double fst_controller::ArmGroup::getAccelerationScalingFactor(void) {
    return m_amax_scaling_factor;
}


//------------------------------------------------------------------------------
// Function:    getCurrentJointValues
// Summary: To get current values of all joints in the robot.
// In:      None
// Out:     None
// Return:  current values of all six joints
//------------------------------------------------------------------------------
const fst_controller::JointValues& fst_controller::ArmGroup::getCurrentJointValues(void) {
    return m_joint_state_current;
}


//------------------------------------------------------------------------------
// Function:    getCurrentPose
// Summary: To get current pose of endpoint in the arm group.
// In:      None
// Out:     None
// Return:  current pose of the endpoint
//------------------------------------------------------------------------------
const fst_controller::Pose& fst_controller::ArmGroup::getCurrentPose(void) {
    return m_pose_state_current;
}


const fst_controller::JointPoint& fst_controller::ArmGroup::getStartJoint(void) {
    return m_joint_start;
}


const fst_controller::Pose& fst_controller::ArmGroup::getStartPose(void) {
    return m_pose_start;
}


//------------------------------------------------------------------------------
// Function:    getPlannedPathFIFOLength
// Summary: To get the length of planned_path_FIFO.
// In:      None
// Out:     None
// Return:  length of the FIFO
//------------------------------------------------------------------------------
int fst_controller::ArmGroup::getPlannedPathFIFOLength(void) {
    return m_planned_path_fifo.size();
}


//------------------------------------------------------------------------------
// Function:    getJointTrajectoryFIFOLength
// Summary: To get the length of joitn_trajectory_FIFO.
// In:      None
// Out:     None
// Return:  length of the FIFO
//------------------------------------------------------------------------------
int fst_controller::ArmGroup::getJointTrajectoryFIFOLength(void) {
    return m_joint_trajectory_fifo.size();
}


/*
//------------------------------------------------------------------------------
// Function:    getJointTrajectoryFIFOIsLocked
// Summary: To get joitn_trajectory_FIFO lock state.
// In:      None
// Out:     None
// Return:  true    -> FIFO is locked
//          false   -> FIFO is not locked
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::getJointTrajectoryFIFOIsLocked(void) {
    return m_joint_trajectory_FIFO_islocked;
}
*/


//------------------------------------------------------------------------------
// Function:    getPointsFromJointTrajectoryFIFO
// Summary: To get points from joitn_trajectory_FIFO.
// In:      num -> number of joint points want to get
// Out:     traj-> joint trajectory consisting of joint points
//          error_code  -> error code
// Return:  <0  -> joint_trajectory_fifo locked, or any other errors
//          >=0 -> number of joint points get actually
//------------------------------------------------------------------------------
int fst_controller::ArmGroup::getPointsFromJointTrajectoryFIFO(
        std::vector<JointPoint> &traj, int num, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    err = SUCCESS;
    if (num < 0) {
        err = INVALID_PARAMETER;
        return num-1;
    }
    else if (num == 0) {
        return 0;
    }

    /*
    if (getJointTrajectoryFIFOIsLocked()) {
        error_code = Error::Trajectory_FIFO_Locked;
        return 0;
    }
    */

    lockArmGroup();
    int cnt;
    std::vector<JointPoint>::iterator itr = m_joint_trajectory_fifo.begin();

    for (cnt = 0; cnt < num; ++cnt) {
        if (itr != m_joint_trajectory_fifo.end()) {
            traj.push_back(*itr);
            if ((itr->id & 0x3) == enum_Level_Ending) {
                // printJointPoint(*itr);
                ++cnt;
                break;
            }
            ++itr;
        }
        else {
            err = NO_ENOUGH_POINTS_FIFO2;
            break;
        }
    }

    if (cnt != 0) {
        itr = m_joint_trajectory_fifo.begin();
        m_joint_trajectory_fifo.erase(itr, itr+cnt);
    }

    unlockArmGroup();
    return cnt;
}


//------------------------------------------------------------------------------
// Function:    setCycleTime
// Summary: To set cycle time of interpolation algorithm.
// In:      tc  -> desired cycle time
// Out:     None
// Return:  true    -> cycle time setted to given value
//          false   -> cycle time NOT changed
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::setCycleTime(double tc) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (tc > m_cycle_time_range.max || tc < m_cycle_time_range.min) {
        return false;
    }
    else {
        m_cycle_time = tc;
        setCycleTime();
        log.info("Set cycle time to %.4fs", m_cycle_time);
        return true;
    }
}


//------------------------------------------------------------------------------
// Function:    setJointConstraints
// Summary: To set joint constraints in Kinematics algorithm.
// In:      constraints -> joint constraints
// Out:     None
// Return:  true    -> set successfully
//          false   -> set UNsuccessfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::setJointConstraints(const JointConstraints &constraints) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    printJointConstraints(constraints);
    m_joint_constraints = constraints;
    setJointConstraints();
}


//------------------------------------------------------------------------------
// Function:    setMaxVelocity
// Summary: To change max velocity settings.
// In:      max_v   -> desired max velocity
// Out:     None
// Return:  true    -> max velocity changed to given value
//          false   -> max velocity NOT changed
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::setMaxVelocity(double max_v) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (max_v > m_velocity_range.max || max_v < m_velocity_range.min) {
        return false;
    }
    else {
        m_vmax = max_v;
        log.info("Set max velocity to %.2fmm/s", m_vmax);
        return true;
    }
}


//------------------------------------------------------------------------------
// Function:    setMaxAcceleration
// Summary: To change max acceleration settings.
// In:      max_a   -> desired max acceleration, unit: mm/s*s
// Out:     None
// Return:  true    -> max acceleration changed to given value
//          false   -> max acceleration NOT changed
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::setMaxAcceleration(double max_a) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (max_a > m_acceleration_range.max || max_a < m_acceleration_range.min) {
        return false;
    }
    else {
        m_amax = max_a;
        setMaxAcceleration();
        log.info("Set max acceleration to %.2fmm/(s*s)", m_amax);
        return true;
    }
}


//------------------------------------------------------------------------------
// Function:    setVelocityScalingFactor
// Summary: To set velocity scaling factor.
// In:      v_factor -> desired velocity scaling factor
// Out:     None
// Return:  true  -> velocity scaling factor changed to given value
//          false -> velocity scaling factor NOT changed
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::setVelocityScalingFactor(double v_factor) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (v_factor < 0.0 || v_factor > 1.0) {
        return false;
    }
    else {
        m_vmax_scaling_factor = v_factor;
        log.info("Set velocity scaling factor to %.2f%%", m_vmax_scaling_factor * 100);
        return true;
    }
}


//------------------------------------------------------------------------------
// Function:    setAccelerationScalingFactor
// Summary: To set acceleration scaling factor.
// In:      a_factor    -> desired acceleration scaling factor
// Out:     None
// Return:  true    -> acceleration scaling factor changed to given value
//          false   -> acceleration scaling factor NOT changed
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::setAccelerationScalingFactor(double a_factor) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (a_factor < 0.0 || a_factor > 1.0) {
        return false;
    }
    else {
        m_amax_scaling_factor = a_factor;
        setMaxAcceleration();
        log.info("Set acceleration scaling factor to %.2f%%", m_amax_scaling_factor * 100);
        return true;
    }
}


//------------------------------------------------------------------------------
// Function:    setJumpThresholdScalingFactor
// Summary: To set jump threshold scaling factor.
// In:      j_factor    -> desired jump threshold scaling factor
// Out:     None
// Return:  true    -> jump threshold scaling factor changed to given value
//          false   -> jump threshold scaling factor NOT changed
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::setJumpThresholdScalingFactor(double j_factor) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (j_factor < 0.0 || j_factor > 1.0) {
        return false;
    }
    else {
        m_jump_threshold_scaling = j_factor;
        setJumpThresholdScalingFactor();
        log.info("Set jump threshold scaling factor to %.2f%%", m_jump_threshold_scaling * 100);
        return true;
    }
}


//------------------------------------------------------------------------------
// Function:    setToolFrame
// Summary: To set current tool frame.
// In:      tool_frame  -> current tool frame
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
void fst_controller::ArmGroup::setToolFrame(const Transformation &tool_frame) {
    if (current_state_ != INITIALIZED) {
        return;
    }

    m_tool_frame = tool_frame;
    setToolFrame();
}


//------------------------------------------------------------------------------
// Function:    setUserFrame
// Summary: To set current user frame.
// In:      user_frame  -> current user frame
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
void fst_controller::ArmGroup::setUserFrame(const Transformation &user_frame) {
    if (current_state_ != INITIALIZED) {
        return;
    }

    m_user_frame = user_frame;
    setUserFrame();
}


//------------------------------------------------------------------------------
// Function:    setCurrentJointValues
// Summary: To set current joint values using encoder data.
//          Current pose values will be updated automaticly.
// In:      current_joint   -> joint values from the encoder
// Out:     error_code  -> error code
// Return:  true    -> current joint/pose values updated successfully
//          false   -> either joint or pose values NOT updated
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::setCurrentJointValues(const JointValues &current_joint,
                                                     ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    if (checkJointBoundary(current_joint)) {
        m_joint_state_current = current_joint;
        if (computeFK(m_joint_state_current, m_pose_state_current, err))
            return true;
        else
            return false;
    }
    else {
        err = JOINT_OUT_OF_CONSTRAINT;
        log.error("setCurrentJOintValues() get a joint group out of boundary");
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setLatestIKReference
// Summary: To set latest IK reference.
// In:      joint_reference -> new IK reference
// Out:     error_code  -> error code
// Return:  true    -> latest IK reference setted to joint_reference scucessfully
//          false   -> failed to set latest IK reference
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::setLatestIKReference(const JointValues &joint_reference, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    if (checkJointBoundary(joint_reference)) {
        m_latest_ik_reference = joint_reference;
        return true;
    }
    else {
        err = JOINT_OUT_OF_CONSTRAINT;
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setStartState
// Summary: To set robot start state.
// In:      joint_start -> robot start state
// Out:     error_code  -> error code
// Return:  true    -> robot start state setted to joint_start scucessfully
//          false   -> failed to set robot start state
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::setStartState(const JointValues &joint_start, ErrorCode &err) {
    lockArmGroup();
    err = SUCCESS;
    if (setCurrentJointValues(joint_start, err)) {
        m_pose_start = getCurrentPose();
        m_pose_previous = getCurrentPose();
        m_vu_start = 0.0;
        m_v_start = 0.0;

        m_joint_start.joints = joint_start;
        m_joint_start.omegas.j1 = 0;
        m_joint_start.omegas.j2 = 0;
        m_joint_start.omegas.j3 = 0;
        m_joint_start.omegas.j4 = 0;
        m_joint_start.omegas.j5 = 0;
        m_joint_start.omegas.j6 = 0;
        m_point_level = enum_Level_Start;
        m_next_cmd_type = enum_Type_Other;

        if (setLatestIKReference(joint_start, err)) {
            unlockArmGroup();
            return true;
        }
        else {
            unlockArmGroup();
            return false;
        }
    }
    else {
        unlockArmGroup();
        log.error("Fail to set start joint values. Error code=0x%llx", err);
        printJointValues("  start joint=", joint_start);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    clearPlannedPathFIFO
// Summary: To clear planned_path_fifo.
// In:      None
// Out:     err -> error code
// Return:  true    -> planned path FIFo is cleared successfully
//          false   -> FIFO cleared with some errors
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::clearPlannedPathFIFO(ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    bool result = true;

    JointValues start_joint;
    lockArmGroup();
    if (m_joint_trajectory_fifo.size() > 0) {
        start_joint = m_joint_trajectory_fifo.back().joints;
        m_joint_trajectory_fifo.back().id &= ~0x3;
        m_joint_trajectory_fifo.back().id |= enum_Level_Ending;
        unlockArmGroup();
        result = setStartState(start_joint, err);
    }
    else {
        start_joint = getLatestIKReference();
        unlockArmGroup();
        result = setStartState(getLatestIKReference(), err);
        if (m_suspend_state.is_suspended = false)
            log.warn("Empty FIFO2, cannot add ending point.");

    }

    lockArmGroup();
    m_planned_path_fifo.clear();
    m_suspend_state.is_suspended = false;
    unlockArmGroup();

    if (result == true) {
        log.info("Planned path FIFO cleared.");
    }
    else {
        log.warn("Cannot set start state. But planned path FIFO is cleared!");
        log.warn("WARNNING: start state is mis-matched, need to be setted manually before next planning");
    }

    return result;
}


/*
//------------------------------------------------------------------------------
// Function:    clearJointTrajectoryFIFO
// Summary: To clear joint_trajectory_fifo(FIFO2) and planned_path_fifo(FIFO1).
// In:      None
// Out:     err -> error code
// Return:  true    -> two FIFOs are both cleared successfully
//          false   -> FIFOs cleared with some errors
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::clearJointTrajectoryFIFO(ErrorCode &err) {
    bool result = true;

    if (m_joint_trajectory_fifo.size() > 0) {
      result = setStartState(m_joint_trajectory_fifo.front().joints, err);
    }
    else if (m_planned_path_fifo.size() > 0){
        if (m_planned_path_fifo.front().type == enum_Type_MoveJ) {
            result = setStartState(m_planned_path_fifo.front().joints, err);
        }
        else if (m_planned_path_fifo.front().type == enum_Type_MoveL ||
                 m_planned_path_fifo.front().type == enum_Type_MoveC) {
            JointValues joints;
            if (computeInverseKinematics(m_planned_path_fifo.front().pose,
	                                     getLatestIKReference(), joints, err)) {
                result = setStartState(joints, err);
            }
            else {
                result = setStartState(getLatestIKReference(), err);
            }
        }
        else {
            err = Error::Undefined_Error;
            result = false;
        }
    }
    else {
        ROS_INFO("Empty FIFO1 and FIFO2. Nothing need to do.");
        return true;
    }

    if (result == true) {
        m_joint_trajectory_fifo.clear();
        m_planned_path_fifo.clear();
        ROS_INFO("Joint trajectory FIFO and planned path FIFO are both cleared.");
    }
    else {
        m_joint_trajectory_fifo.clear();
        m_planned_path_fifo.clear();
        ROS_ERROR("Cannot set start state. But joint trajectory FIFO and planned path FIFO are both cleared!");
        ROS_ERROR("WARNNING: Wrong start state, need to be setted manually before next planning");
    }

    return result;
}
*/

bool fst_controller::ArmGroup::resetArmGroup(const JointValues &joint, ErrorCode &err) {
    log.info("Reset ArmGroup ... ");
    printJointValues("Using new joint values to reset ArmGroup: joints=",joint);
    lockArmGroup();
    if (setCurrentJointValues(joint, err)) {
        m_pose_start = getCurrentPose();
        m_pose_previous = m_pose_start;
        m_vu_start = 0.0;
        m_v_start = 0.0;
        m_latest_ik_reference = joint;
        m_joint_start.joints = joint;
        m_joint_start.omegas.j1 = 0;
        m_joint_start.omegas.j2 = 0;
        m_joint_start.omegas.j3 = 0;
        m_joint_start.omegas.j4 = 0;
        m_joint_start.omegas.j5 = 0;
        m_joint_start.omegas.j6 = 0;
        log.info("Success!");
    }
    else {
        log.error("Fail to reset ArmGroup using given joint values. Error cpde=0x%llx", err);
    }

    m_point_level = enum_Level_Start;
    m_next_cmd_type = enum_Type_Other;
    
    m_suspend_state.is_suspended = false;

    m_planned_path_fifo.clear();
    m_joint_trajectory_fifo.clear();

    unlockArmGroup();
    log.info("Success!");
}

//------------------------------------------------------------------------------
// Function:    computeIK
// Summary: To compute IK with a given pose in cartesian space.
// In:      poes    -> cartesian space pose needed to compute IK
// Out:     joint_result-> IK result
//          error_code  -> error code
// Return:  true    -> IK solution found
//          false   -> IK solution NOT found
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::computeIK(const Pose &pose, JointValues &joint_result, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    return computeInverseKinematics(pose, getLatestIKReference(), joint_result, err);
}


//------------------------------------------------------------------------------
// Function:    computeFK
// Summary: To compute FK with given joint values.
// In:      joint_result -> joint values needed to compute FK
// Out:     poes -> FK result
//          error_code -> error code
// Return:  true  -> FK computed successfully
//          false -> FK computed UNsuccessfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::computeFK(const JointValues &joint, Pose &pose, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    return computeForwardKinematics(joint, pose, err);
}


//------------------------------------------------------------------------------
// Function:    MoveJ
// Summary: To plan a path in joint space to touch target pose, without smooth.
// In:      joint_target-> target in joint space
//          v_max   -> max velocity
//          a_max   -> max acceleration
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max,
                                     int id, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    if (m_next_cmd_type != enum_Type_MoveJ && m_next_cmd_type != enum_Type_Other) {
        err = INVALID_SEQUENCE;
        return false;
    }

    if (!m_planned_path_fifo.empty() &&
        (m_planned_path_fifo.back().type == enum_Type_MoveL ||
         m_planned_path_fifo.back().type == enum_Type_MoveC)) {
        err = CARTESIAN_PATH_EXIST;
        return false;
    }

    log.info("MoveJ (without smooth) request accepted, planning joint path...");

    if (!checkJointBoundary(joint_target)) {
        err = TARGET_OUT_OF_CONSTRAINT;
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        return false;
    }

    v_max = v_max / m_velocity_range.max;
    double v_percent = v_max > 1.0 ? 1.0 : v_max;
    std::vector<JointValues> planned_path;
    JointPoint jp_target;
    jp_target.joints = joint_target;
    memset(&jp_target.omegas, 0, sizeof(JointVelocity));
    
    lockArmGroup();

    bool res = MoveJ2J(m_joint_start,
                       jp_target,
                       jp_target,
                       v_percent,
                       0,
                       planned_path,
                       err);

    if (res) {
        if (0 == planned_path.size()) {
            unlockArmGroup();
            log.warn("Joint path generated with 0 point, are we standing on the target point already?");
            return true;
        }

        if (setLatestIKReference(planned_path.back(), err)) {
            PathPoint pp;
            pp.type = enum_Type_MoveJ;
            pp.user_frame = 0;
            pp.tool_frame = 0;
            pp.velocity = 0.0;

            if (enum_Level_Start == m_point_level) {
                pp.id = (id << 2) + m_point_level;
                pp.joints = planned_path.front();
                m_planned_path_fifo.push_back(pp);
                m_point_level = enum_Level_Middle;
                pp.id = (id << 2) + m_point_level;

                for (std::vector<JointValues>::iterator itr = planned_path.begin() + 1;
                        itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }
            else if (enum_Level_Middle == m_point_level) {
                pp.id = (id << 2) + m_point_level;

                for (std::vector<JointValues>::iterator itr = planned_path.begin(); itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }
            else {
                err = MOTION_INTERNAL_FAULT;
                unlockArmGroup();
                return false;
            }

            m_planned_path_fifo.back().id = (id << 2) + enum_Level_Ending;
            m_point_level = enum_Level_Start;
            m_next_cmd_type = enum_Type_Other;

            computeFK(planned_path.back(), m_pose_start, err);
            m_pose_previous = m_pose_start;
            m_vu_start = 0;
            m_v_start = 0;

            unlockArmGroup();
            log.info("Joint path generated successfully with %zd points, and added into planned_path_FIFO.",
                     planned_path.size());

            return true;
        }
        else {
            unlockArmGroup();
            return false;
        }
    }
    else {
        unlockArmGroup();
        log.error("ERROR occurred during generating joint path, error code:0x%llx", err);
        log.error("Invalid path obtained and dropped.");
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveJ (smooth to MoveJ)
// Summary: To plan a path in joint space to touch target pose, with smooth.
// In:      joint_target-> target in joint space
//          v_max   -> max velocity
//          a_max   -> max acceleration
//          cnt     -> smooth degree
//          joint_next  -> next target in joint space
//          v_next  -> max velocity in the next path
//          a_next  -> max acceleration in the next path
//          cnt_next    -> smooth degree in the next path
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
                                     const JointValues &joint_next, double v_next, double a_next, int cnt_next,
                                     int id, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    if (m_next_cmd_type != enum_Type_MoveJ && m_next_cmd_type != enum_Type_Other) {
        err = INVALID_SEQUENCE;
        return false;
    }

    if (cnt <= 0 || cnt > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    if (!m_planned_path_fifo.empty() &&
        (m_planned_path_fifo.back().type == enum_Type_MoveL ||
         m_planned_path_fifo.back().type == enum_Type_MoveC)) {
        err = CARTESIAN_PATH_EXIST;
        return false;
    }

    log.info("MoveJ (smooth to MoveJ) request accepted, planning joint path...");
    
    if (!checkJointBoundary(joint_target) || !checkJointBoundary(joint_next)) {
        err = TARGET_OUT_OF_CONSTRAINT;
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        return false;
    }

    v_max = v_max / m_velocity_range.max;
    double v_percent = v_max > 1.0 ? 1.0 : v_max;
    std::vector<JointValues> planned_path;
    JointPoint jp_target, jp_next;
    jp_target.joints = joint_target;
    jp_next.joints = joint_next;
    memset(&jp_target.omegas, 0, sizeof(JointVelocity));

    lockArmGroup();

    bool res = MoveJ2J(m_joint_start,
                       jp_target,
                       jp_next,
                       v_percent,
                       cnt,
                       planned_path,
                       err);

    if (res) {
        if (0 == planned_path.size()) {
            log.warn("Joint path generated with 0 point, are we standing on the target point already?");
            unlockArmGroup();
            return true;
        }

        if (setLatestIKReference(planned_path.back(), err)) {
            PathPoint pp;
            pp.type = enum_Type_MoveJ;
            pp.user_frame = 0;
            pp.tool_frame = 0;
            pp.velocity = 0.0;

            if (enum_Level_Start == m_point_level) {
                pp.id = (id << 2) + m_point_level;
                pp.joints = planned_path.front();
                m_planned_path_fifo.push_back(pp);
                m_point_level = enum_Level_Middle;
                pp.id = (id << 2) + m_point_level;

                for (std::vector<JointValues>::iterator itr = planned_path.begin() + 1;
                        itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }
            else if (enum_Level_Middle == m_point_level) {
                pp.id = (id << 2) + m_point_level;

                for (std::vector<JointValues>::iterator itr = planned_path.begin(); itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }
            else {
                err = MOTION_INTERNAL_FAULT;
                unlockArmGroup();
                return false;
            }

            m_next_cmd_type = enum_Type_MoveJ;

            unlockArmGroup();
            log.info("Joint path generated successfully with %zd points, and added into planned_path_FIFO.",
                     planned_path.size());

            return true;
        }
        else {
            unlockArmGroup();
            return false;
        }
    }
    else {
        unlockArmGroup();
        log.error("ERROR occurred during generating joint path, error code=0x%llx", err);
        log.error("Invalid path obtained and dropped.");
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveJ (smooth to MoveL)
// Summary: To plan a path in joint space to touch target pose, with smooth.
// In:      joint_target-> target in joint space
//          v_max   -> max velocity
//          a_max   -> max acceleration
//          cnt     -> smooth degree
//          pose_next   -> next target in Cartesian space
//          v_next  -> max velocity in the next path
//          a_next  -> max acceleration in the next path
//          cnt_next    -> smooth degree in the next path
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
                                     const Pose &pose_next, double v_next, double a_next, int cnt_next,
                                     int id, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    if (m_next_cmd_type != enum_Type_MoveJ && m_next_cmd_type != enum_Type_Other) {
        err = INVALID_SEQUENCE;
        return false;
    }

    if (cnt <= 0 || cnt > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        return false;
    }

    if (!m_planned_path_fifo.empty() &&
        (m_planned_path_fifo.back().type == enum_Type_MoveL ||
         m_planned_path_fifo.back().type == enum_Type_MoveC)) {
        err = CARTESIAN_PATH_EXIST;
        return false;
    }

    log.info("MoveJ (smooth to MoveL) request accepted, planning joint path...");

    if (!checkJointBoundary(joint_target)) {
        err = TARGET_OUT_OF_CONSTRAINT;
        return false;
    }

    v_max = v_max/m_velocity_range.max;
    double v_percent = v_max > 1.0 ? 1.0 : v_max;
    std::vector<JointValues> planned_path;
    JointValues j_target = joint_target;

    lockArmGroup();

    bool res = MoveJ2L(m_joint_start,
                       j_target,
                       v_percent,
                       cnt,
                       pose_next,
                       v_next,
                       cnt_next,
                       planned_path,
                       m_pose_start,
                       m_pose_previous,
                       m_v_start,
                       m_vu_start,
                       err);

    if (res) {
        if (0 == planned_path.size()) {
            log.warn("Joint path generated with 0 point, are we standing on the target point already?");
            unlockArmGroup();
            return true;
        }

        if (setLatestIKReference(planned_path.back(), err)) {
            PathPoint pp;
            pp.type = enum_Type_MoveJ;
            pp.user_frame = 0;
            pp.tool_frame = 0;
            pp.velocity = 0.0;

            if (enum_Level_Start == m_point_level) {
                pp.id = (id << 2) + m_point_level;
                pp.joints = planned_path.front();
                m_planned_path_fifo.push_back(pp);
                m_point_level = enum_Level_Middle;
                pp.id = (id << 2) + m_point_level;

                for (std::vector<JointValues>::iterator itr = planned_path.begin() + 1;
                        itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }
            else if (enum_Level_Middle == m_point_level) {
                pp.id = (id << 2) + m_point_level;

                for (std::vector<JointValues>::iterator itr = planned_path.begin(); itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }
            else {
                err = MOTION_INTERNAL_FAULT;
                unlockArmGroup();
                return false;
            }

            m_next_cmd_type = enum_Type_MoveL;

            unlockArmGroup();
            log.info("Joint path generated successfully with %zd points, and added into planned_path_FIFO.",
                     planned_path.size());

            return true;
        }
        else {
            unlockArmGroup();
            return false;
        }
    }
    else {
        unlockArmGroup();
        log.error("ERROR occurred during generating joint path, error code=0x%llx", err);
        log.error("Invalid path obtained and dropped.");
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveJ (smooth to MoveL)
// Summary: To plan a path in joint space to touch target pose, with smooth.
// In:      joint_target-> target in joint space
//          v_max   -> max velocity
//          a_max   -> max acceleration
//          cnt     -> smooth degree
//          pose_next   -> next target in Cartesian space
//          v_next  -> max velocity in the next path
//          a_next  -> max acceleration in the next path
//          cnt_next-> smooth degree in the next path
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
                                     const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
                                     int id, ErrorCode &err) {
    return MoveJ(joint_target, v_max, a_max, cnt,
                 transformPoseEuler2Pose(pose_next), v_next, a_next, cnt_next,
                 id, err);
}


//------------------------------------------------------------------------------
// Function:    MoveL
// Summary: To plan a linear path to touch target pose, without smooth.
// In:      pose_target -> target pose of the linear path
//          v_max   -> max velocity of endpoint
//          a_max   -> max acceleration of endpoint
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_cartesian_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL(const Pose &pose_target, double v_max, double a_max,
                                     int id, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }

    if (m_next_cmd_type != enum_Type_MoveL && m_next_cmd_type != enum_Type_Other) {
        err = INVALID_SEQUENCE;
        return false;
    }

    log.info("MoveL (without smooth) request accepted, planning cartesian path...");

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        return false;
    }

    double v_target = m_vmax * m_vmax_scaling_factor;
    std::vector<Pose> planned_path;

    lockArmGroup();

    bool res = MoveL2L(m_pose_start,
                       m_v_start,
                       m_vu_start,
                       pose_target,
                       v_target,
                       0,
                       pose_target,
                       0,
                       m_pose_previous,
                       planned_path,
                       err);

    if (res) {
        PathPoint pp;
        pp.type = enum_Type_MoveL;
        pp.user_frame = 0;
        pp.tool_frame = 0;
        pp.velocity = 0.0;

        if (enum_Level_Start == m_point_level) {
            pp.id = (id << 2) + m_point_level;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            m_point_level = enum_Level_Middle;
            pp.id = (id << 2) + m_point_level;

            for (std::vector<Pose>::iterator itr = planned_path.begin()+1; itr != planned_path.end(); ++itr) {
		        /*
		        // ------------------- FOR TEST ONLY --------------------
		        ROS_INFO("Pose :%f,%f,%f,%f,%f,%f,%f",
                         itr->position.x,
                         itr->position.y,
                         itr->position.z,
                         itr->orientation.w,
                         itr->orientation.x,
                         itr->orientation.y,
                         itr->orientation.z);
		        // ^^^^^^^^^^^^^^^^^^^ FOR TEST ONLY ^^^^^^^^^^^^^^^^^^^^
		        */
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else if (enum_Level_Middle == m_point_level) {
            pp.id = (id << 2) + m_point_level;

            for (std::vector<Pose>::iterator itr = planned_path.begin(); itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            err = MOTION_INTERNAL_FAULT;
            unlockArmGroup();
            return false;
        }

        m_planned_path_fifo.back().id = (id << 2) + enum_Level_Ending;
        m_point_level = enum_Level_Start;
        m_next_cmd_type = enum_Type_Other;

        unlockArmGroup();
        log.info("Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());

        return true;
    }
    else {
        unlockArmGroup();
        log.error("ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("Invalid path obtained and dropped.");
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveL
// Summary: To plan a linear path to touch target pose, without smooth.
// In:      pose_target -> target pose of the linear path
//          v_max   -> max velocity of endpoint
//          a_max   -> max acceleration of endpoint
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_cartesian_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL(const PoseEuler &pose_target, double v_max, double a_max,
                                     int id, ErrorCode &err) {
    return MoveL(transformPoseEuler2Pose(pose_target), v_max, a_max, id, err);
}


//------------------------------------------------------------------------------
// Function:    MoveL (smooth to MoveL)
// Summary: To plan a linear path to touch target pose, with smooth.
// In:      pose_target -> target pose of the linear path
//          v_max   -> max velocity of endpoint
//          a_max   -> max acceleration of endpoint
//          cnt_target  -> smooth degree
//          pose_next   -> target pose of the next path
//          v_next  -> max velocity of endpoint in the next path
//          a_next  -> max acceleration of endpoint in the next path
//          cnt_next-> smooth degree in the next path
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_cartesian_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL(const Pose &pose_target, double v_max, double a_max, int cnt_target,
                                     const Pose &pose_next, double v_next, double a_next, int cnt_next,
                                     int id, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }

    if (m_next_cmd_type != enum_Type_MoveL && m_next_cmd_type != enum_Type_Other) {
        err = INVALID_SEQUENCE;
        return false;
    }

    if (cnt_target <= 0 || cnt_target > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    log.info("MoveL (smooth to MoveL) request accepted, planning cartesian path...");
    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        return false;
    }

    v_max  = m_vmax * m_vmax_scaling_factor;
    v_next = v_next * m_vmax_scaling_factor;
    std::vector<Pose> planned_path;

    lockArmGroup();

    bool res = MoveL2L(m_pose_start,
                       m_v_start,
                       m_vu_start,
                       pose_target,
                       v_max,
                       cnt_target,
                       pose_next,
                       v_next,
                       m_pose_previous,
                       planned_path,
                       err);
    /*
    if (!res && Error::MoveL_Unsmoothable == error_code) {
        res = MoveL2L(m_pose_start,
                      m_v_start,
                      m_vu_start,
                      pose_target,
                      v_max,
                      0,
                      pose_next,
                      v_next,
                      cnt_next,
                      m_pose_previous,
                      planned_path,
                      error_code);
    }
    */

    if (res) {
        PathPoint pp;
        pp.type = enum_Type_MoveL;
        pp.user_frame = 0;
        pp.tool_frame = 0;
        pp.velocity = 0.0;

        if (enum_Level_Start == m_point_level) {
            pp.id = (id << 2) + m_point_level;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            m_point_level = enum_Level_Middle;
            pp.id = (id << 2) + m_point_level;

            for (std::vector<Pose>::iterator itr = planned_path.begin() + 1; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else if (enum_Level_Middle == m_point_level) {
            pp.id = (id << 2) + m_point_level;

            for (std::vector<Pose>::iterator itr = planned_path.begin(); itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            err = MOTION_INTERNAL_FAULT;
            unlockArmGroup();
            return false;
        }

        m_next_cmd_type = enum_Type_MoveL;

        unlockArmGroup();
        log.info("Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());

        return true;
    }
    else if (err == PLANNING_UNSMOOTHABLE) {
        unlockArmGroup();
        log.info("Path unsmoothable using cnt=%d, replanning using cnt=0...", cnt_target);
        return MoveL(pose_target, v_max / m_vmax_scaling_factor, a_max, id, err);
    }
    else {
        unlockArmGroup();
        log.error("ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("Invalid path obtained and dropped.");
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveL (smooth to MoveL)
// Summary: To plan a linear path to touch target pose, with smooth.
// In:      pose_target -> target pose of the linear path
//          v_max   -> max velocity of endpoint
//          a_max   -> max acceleration of endpoint
//          cnt_target  -> smooth degree
//          pose_next   -> target pose of the next path
//          v_next  -> max velocity of endpoint in the next path
//          a_next  -> max acceleration of endpoint in the next path
//          cnt_next-> smooth degree in the next path
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_cartesian_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL(const PoseEuler &pose_target, double v_max, double a_max, int cnt_target,
                                     const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
                                     int id, ErrorCode &err) {
    return MoveL(transformPoseEuler2Pose(pose_target), v_max, a_max, cnt_target,
                 transformPoseEuler2Pose(pose_next), v_next, a_next, cnt_next,
                 id, err);
}


//------------------------------------------------------------------------------
// Function:    MoveL (smooth to MoveJ)
// Summary: To plan a linear path to touch target pose, with smooth.
// In:      pose_target -> target pose of the linear path
//          v_max   -> max velocity of endpoint
//          a_max   -> max acceleration of endpoint
//          cnt_target  -> smooth degree
//          joint_next  -> target pose in joint space of the next path
//          v_next  -> max velocity of endpoint in the next path
//          a_next  -> max acceleration of endpoint in the next path
//          cnt_next    -> smooth degree in the next path
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL(const Pose &pose_target, double v_max, double a_max, int cnt_target,
                                     const JointValues &joint_next, double v_next, double a_next, int cnt_next,
                                     int id, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }

    if (m_next_cmd_type != enum_Type_MoveL && m_next_cmd_type != enum_Type_Other) {
        err = INVALID_SEQUENCE;
        return false;
    }

    if (cnt_target <= 0 || cnt_target > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    log.info("MoveL (smooth to MoveJ) request accepted, planning cartesian path...");
    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        return false;
    }

    v_max  = m_vmax * m_vmax_scaling_factor;
    v_next = v_next * m_vmax_scaling_factor;
    std::vector<Pose> planned_path;

    lockArmGroup();

    bool res = MoveL2J(m_pose_start,
                       m_v_start,
                       m_vu_start,
                       pose_target,
                       v_max,
                       cnt_target,
                       m_pose_previous,
                       planned_path,
                       err);

    if (res) {
        fst_controller::PathPoint pp;
        pp.type = enum_Type_MoveL;
        pp.user_frame = 0;
        pp.tool_frame = 0;
        pp.velocity = 0.0;

        if (enum_Level_Start == m_point_level) {
            pp.id = (id << 2) + m_point_level;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            m_point_level = enum_Level_Middle;
            pp.id = (id << 2) + m_point_level;

            for (std::vector<Pose>::iterator itr = planned_path.begin()+1; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else if (enum_Level_Middle == m_point_level) {
            pp.id = (id << 2) + m_point_level;

            for (std::vector<Pose>::iterator itr = planned_path.begin(); itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            err = MOTION_INTERNAL_FAULT;
            unlockArmGroup();
            return false;
        }

        m_next_cmd_type = enum_Type_MoveJ;

        unlockArmGroup();
        log.info("Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());
        return true;
    }
    else {
        unlockArmGroup();
        log.error("ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("Invalid path obtained and dropped.");
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveL (smooth to MoveJ)
// Summary: To plan a linear path to touch target pose, with smooth.
// In:      pose_target -> target pose of the linear path
//          v_max   -> max velocity of endpoint
//          a_max   -> max acceleration of endpoint
//          cnt_target  -> smooth degree
//          joint_next  -> target pose in joint space of the next path
//          v_next  -> max velocity of endpoint in the next path
//          a_next  -> max acceleration of endpoint in the next path
//          cnt_next-> smooth degree in the next path
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL(const PoseEuler &pose_target, double v_max, double a_max, int cnt_target,
                                     const JointValues &joint_next, double v_next, double a_next, int cnt_next,
                                     int id, ErrorCode &err) {
    return MoveL(transformPoseEuler2Pose(pose_target), v_max, a_max, cnt_target,
                 joint_next, v_next, a_next, cnt_next, id, err);
}


//------------------------------------------------------------------------------
// Function:    MoveC (without smooth)
bool fst_controller::ArmGroup::MoveC(const Pose pose_2nd, const Pose pose_3rd,
                                     double v_target, double a_target,
                                     int id, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }

    if (m_next_cmd_type != enum_Type_MoveC && m_next_cmd_type != enum_Type_Other) {
        err = INVALID_SEQUENCE;
        return false;
    }

    log.info("MoveC (without smooth) request accepted, planning cartesian path...");
    if (!setMaxVelocity(v_target) || !setMaxAcceleration(a_target)) {
        err = INVALID_PARAMETER;
        log.error("MoveC get an invalid parameter.");
        return false;
    }

    double velocity  = getMaxVelocity() * getVelocityScalingFactor();
    std::vector<Pose> planned_path;

    lockArmGroup();

    bool res = MoveC2J(m_pose_start, m_v_start, pose_2nd, pose_3rd, velocity, 0, planned_path, err);

    if (res) {
        PathPoint pp;
        pp.type = enum_Type_MoveC;
        pp.user_frame = 0;
        pp.tool_frame = 0;
        pp.velocity = 0.0;

        if (enum_Level_Start == m_point_level) {
            pp.id = (id << 2) + m_point_level;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            m_point_level = enum_Level_Middle;
            pp.id = (id << 2) + m_point_level;

            for (std::vector<Pose>::iterator itr = planned_path.begin()+1; itr != planned_path.end(); ++itr) {
		        /*
		        // ------------------- FOR TEST ONLY --------------------
		        ROS_INFO("Pose :%f,%f,%f,%f,%f,%f,%f",
                         itr->position.x,
                         itr->position.y,
                         itr->position.z,
                         itr->orientation.w,
                         itr->orientation.x,
                         itr->orientation.y,
                         itr->orientation.z);
		        // ^^^^^^^^^^^^^^^^^^^ FOR TEST ONLY ^^^^^^^^^^^^^^^^^^^^
		        */
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else if (enum_Level_Middle == m_point_level) {
            pp.id = (id << 2) + m_point_level;

            for (std::vector<Pose>::iterator itr = planned_path.begin(); itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            err = MOTION_INTERNAL_FAULT;
            unlockArmGroup();
            return false;
        }

        m_planned_path_fifo.back().id = (id << 2) + enum_Level_Ending;
        m_point_level = enum_Level_Start;
        m_next_cmd_type = enum_Type_Other;

        unlockArmGroup();
        log.info("Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());
        return true;                                            
    }
    else {
        log.error("ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("Invalid path obtained and dropped.");
        unlockArmGroup();
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveC (without smooth)
bool fst_controller::ArmGroup::MoveC(const PoseEuler pose_2nd, const PoseEuler pose_3rd,
                                     double v_target, double a_target,
                                     int id, ErrorCode &err) {
    return MoveC(transformPoseEuler2Pose(pose_2nd), transformPoseEuler2Pose(pose_3rd),
                 v_target, a_target, id, err);
}



//------------------------------------------------------------------------------
// Function:    convertPathToTrajectory
// Summary: To convert numbers of posepoint in m_cartesian_path_FIFO
//          into jointpoint in m_joint_trajectory_FIFO.
// In:      num        -> number of pose that needed to be converted
// Out:     error_code -> error code
// Return:  <0         -> ERROR occurred during converting
//          >=0        -> number of pose that convered actually
//------------------------------------------------------------------------------
int fst_controller::ArmGroup::convertPathToTrajectory(int num, ErrorCode &err) {
    if (num < 0) {
        err = INVALID_PARAMETER;
        return false;
    }
    else if (num == 0) {
        return true;
    }

    if (m_suspend_state.is_suspended) {
        err = ARM_GROUP_SUSPENDED;
        return 0;
    }
    
    lockArmGroup();
    int result = convertPath2Trajectory(num, err);
    unlockArmGroup();

    return result;
}


//------------------------------------------------------------------------------
// Function:    convertPath2Trajectory
// Summary: To convert numbers of posepoint in m_cartesian_path_FIFO
//          into jointpoint in m_joint_trajectory_FIFO.
// In:      num        -> number of pose that needed to be converted
// Out:     error_code -> error code
// Return:  <0         -> ERROR occurred during converting
//          >=0        -> number of pose that convered actually
//------------------------------------------------------------------------------
int fst_controller::ArmGroup::convertPath2Trajectory(int num, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    err = SUCCESS;
    if (num > m_JOINT_FIFO_LEN - getJointTrajectoryFIFOLength()) {
        err = TRAJECTORY_FIFO_FULL;
        num = m_JOINT_FIFO_LEN - getJointTrajectoryFIFOLength();
    }

    if (num > getPlannedPathFIFOLength()) {
        err = NO_ENOUGH_POINTS_FIFO1;
        num = getPlannedPathFIFOLength();
    }

    int conv_cnt = 0;
    std::vector<fst_controller::PathPoint>::iterator itr = m_planned_path_fifo.begin();
    fst_controller::JointPoint jp;
    
    for (conv_cnt = 0; conv_cnt < num; ++conv_cnt) {
        /*
        if (itr == m_planned_path_fifo.end()) {
            error_code = Error::No_Enough_Points;
            break;
        }
        */

        if (itr->type == enum_Type_MoveL || itr->type == enum_Type_MoveC) {
            if (computeIK(itr->pose, jp.joints, err)) {
            /*
	        //computeIK(itr->pose,jp.joints,err); {
	    	// ------------------- FOR TEST ONLY --------------------
		    JointValues j=m_latest_ik_reference;
    		// Pose p = itr->pose;
	    	ErrorCode err1;
		    // computeFK(j,p,err1);
    		// ROS_INFO("reference:%f,%f,%f,%f,%f,%f", j.j1,j.j2,j.j3,j.j4,j.j5,j.j6);
	    	j=jp.joints;
		    ROS_INFO("IK result:%f,%f,%f,%f,%f,%f", j.j1,j.j2,j.j3,j.j4,j.j5,j.j6);
    		// ROS_INFO("Pose plann:%f,%f,%f,%f,%f,%f,%f", p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
	    	// computeFK(jp.joints,p,err1);
		    // ROS_INFO("FK-%d:%f,%f,%f,%f,%f,%f,%f",error_code, p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
		    // ^^^^^^^^^^^^^^^^^^^ FOR TEST ONLY ^^^^^^^^^^^^^^^^^^^^
		    */
                if (itr == m_planned_path_fifo.end() - 1) {
                    JointValues joints = getLatestIKReference();
                    m_joint_start.joints = jp.joints;
                    m_joint_start.omegas.j1 = (jp.joints.j1 - joints.j1) / m_cycle_time;
                    m_joint_start.omegas.j2 = (jp.joints.j2 - joints.j2) / m_cycle_time;
                    m_joint_start.omegas.j3 = (jp.joints.j3 - joints.j3) / m_cycle_time;
                    m_joint_start.omegas.j4 = (jp.joints.j4 - joints.j4) / m_cycle_time;
                    m_joint_start.omegas.j5 = (jp.joints.j5 - joints.j5) / m_cycle_time;
                    m_joint_start.omegas.j6 = (jp.joints.j6 - joints.j6) / m_cycle_time;
                }
                jp.id = itr->id;
                
                m_joint_trajectory_fifo.push_back(jp);
                m_latest_ik_reference = jp.joints;
            }
            else {
                log.error("Error while converting cartesian point to joint space, Error code=0x%llx", err);
                log.error("%d points has been converted before the error", conv_cnt);
                printJointValues("IK reference: ", m_latest_ik_reference);
                printJointValues("IK result:    ", jp.joints);

                // ------------------- FOR TEST ONLY --------------------
                /*
                Pose tmp;
                computeFK(m_latest_ik_reference, tmp, err);
                printPose("reference: ", tmp);
                computeFK(jp.joints, tmp, err);
                printPose("result: ", tmp);
                */
                // ^^^^^^^^^^^^^^^^^^^ FOR TEST ONLY ^^^^^^^^^^^^^^^^^^^^
                break;
            }
        }
        else if (itr->type == enum_Type_MoveJ) {
            jp.id = itr->id;
            jp.joints = itr->joints;
            m_joint_trajectory_fifo.push_back(jp);
            m_latest_ik_reference = jp.joints;
        }
        else {
            err = MOTION_INTERNAL_FAULT;
            break;
        }

        ++itr;
    }
    
    if (conv_cnt != 0) {
        itr = m_planned_path_fifo.begin();
        m_planned_path_fifo.erase(itr, itr+conv_cnt);
    }

    return conv_cnt;
}


//------------------------------------------------------------------------------
// Function:    suspendArmMotion
// Summary: To replan a slow-down path and stop. Used when pause event
//          or IK failure raised.
// In:      None
// Out:     None
// Return:  true  -> replan successfully
//          false -> replan UNsuccessfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::suspendArmMotion(ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    log.info("Suspend request accepted.");

    lockArmGroup();
    
    if (m_joint_trajectory_fifo.size() == 0) {
        unlockArmGroup();
        err = NO_ENOUGH_POINTS_FIFO2;
        log.warn("FIFO2 is empty, cannot replan a slow-down trajectory. Are we at a standstill?");
        return false;
    }

    std::vector<JointPoint>::iterator itr_ending_point = m_joint_trajectory_fifo.end();
    for (std::vector<JointPoint>::iterator itr = m_joint_trajectory_fifo.begin(); 
         itr != m_joint_trajectory_fifo.end(); ++itr) {
        if (enum_Level_Ending == (itr->id & 0x3)) {
            itr_ending_point = itr;
            break;
        }
    }

    if (itr_ending_point != m_joint_trajectory_fifo.end()) {
        // There is an ending point in FIFO2, we will stop following the original plan.
        log.info("An ending point found in FIFO2, prepareing slow-down trajectory...");
        // Move points behind the ending point from FIFO2 to cache.
        m_suspend_state.fifo2_cache.clear();
        m_suspend_state.fifo2_cache.insert(m_suspend_state.fifo2_cache.begin(),
                                           ++itr_ending_point,
                                           m_joint_trajectory_fifo.end());
        m_joint_trajectory_fifo.erase(itr_ending_point, itr_ending_point + m_suspend_state.fifo2_cache.size());
        
        // refresh the m_suspend_state to record that we are suspended. 
        m_suspend_state.pattern = e_suspend_pattern_origin;
        m_suspend_state.last_point = m_joint_trajectory_fifo.back().joints;
        m_suspend_state.replan_trajectory.clear();
        m_suspend_state.is_suspended = true;
        
        log.info("Slow-down trajectory ready, we will stop the arm group in %zu points",
                 m_joint_trajectory_fifo.size());
        
        unlockArmGroup();
        return true;
    } // if (itr_ending_point != m_joint_trajectory_fifo.end()) {
    else {
        // None ending point found in FIFO2, we need to replan a slow-down trajectory in FIFO2
        log.info("Replanning slow-down trajectory in FIFO2...");
        if (m_joint_trajectory_fifo.size() < 10) {
            err = NO_ENOUGH_POINTS_FIFO2;
            log.error("Too few points in FIFO2, we cannot replan a slow-down trajectory");
            unlockArmGroup();
            return false;
        }
        // std::vector<JointValues> stop_trajectory;
        // stop_trajectory.reserve(400);
        m_suspend_state.replan_trajectory.clear();
        for (std::vector<JointPoint>::iterator itr = m_joint_trajectory_fifo.begin();
                itr != m_joint_trajectory_fifo.end(); ++itr) {
             m_suspend_state.replan_trajectory.push_back(itr->joints);
        }
        int result = planner_->Pause(m_suspend_state.replan_trajectory);
        if (0 == result) {
            JointPoint jp;
            jp.id = m_joint_trajectory_fifo.front().id & ~0x3 | enum_Level_Middle;
            m_joint_trajectory_fifo.clear();
            for (std::vector<JointValues>::iterator itr = m_suspend_state.replan_trajectory.begin();
                 itr != m_suspend_state.replan_trajectory.end(); ++itr) {
                jp.joints = *itr;
                m_joint_trajectory_fifo.push_back(jp);
            }
            // change the ID of last point in FIFO2, to make an ending point.
            m_joint_trajectory_fifo.back().id &= ~0x3;
            m_joint_trajectory_fifo.back().id |= enum_Level_Ending;
            // refresh the m_suspend_state to record that we are suspended.
            m_suspend_state.pattern = e_suspend_pattern_replan;
            m_suspend_state.last_point = m_joint_trajectory_fifo.back().joints;
            m_suspend_state.fifo2_cache.clear();
            m_suspend_state.replan_trajectory.clear();
            m_suspend_state.is_suspended = true;

            log.info("Slow-down trajectory ready, we will stop the arm group in %zu points",
                     m_joint_trajectory_fifo.size());
            unlockArmGroup();
            return true;
        } // if (0 == result) {
        else {
            err = MOTION_INTERNAL_FAULT;
            log.error("ERROR while replanning slow-down trajectory, error code = %d", result);
            unlockArmGroup();
            return false;
        } // else {
    } // else {
}


//------------------------------------------------------------------------------
// Function:    resumeArmMotion
// Summary: To replan a start-up path and resume arm motion.
// In:      None
// Out:     None
// Return:  true  -> replan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::resumeArmMotion(ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    log.info("Resume request accepted.");
    
    lockArmGroup();
    if (!m_suspend_state.is_suspended) {
        unlockArmGroup();
        err = ARM_GROUP_NOT_SUSPENDED;
        log.error("Arm group has not been suspended.");
        return false;
    }

    if (getJointTrajectoryFIFOLength() != 0) {
        unlockArmGroup();
        err = INVALID_SEQUENCE;
        log.error("FIFO2 is not empty, resume is not allowed now.");
        return false;
    }

    int num = 0;
    err = SUCCESS;
    switch (m_suspend_state.pattern) {
        case e_suspend_pattern_origin:
            m_joint_trajectory_fifo.insert(m_joint_trajectory_fifo.begin(),
                                           m_suspend_state.fifo2_cache.begin(),
                                           m_suspend_state.fifo2_cache.end());
            /*
            ErrorCode err = Error::Success;
            int point_num = 100 - getJointTrajectoryFIFOLength();
            if ( point_num == convertPathToTrajectory(point_num, err)) {
                planner_->Restart()
            }
            */
            m_suspend_state.is_suspended = false;
            unlockArmGroup();
            log.info("Resume successfully.");
            return true;
        case e_suspend_pattern_replan:
            num = m_planned_path_fifo.size()<100 ? m_planned_path_fifo.size() : 100;
            if (convertPath2Trajectory(num, err) == num) {
                m_suspend_state.replan_trajectory.clear();
                for (std::vector<JointPoint>::iterator itr = m_joint_trajectory_fifo.begin();
                     itr != m_joint_trajectory_fifo.end(); ++itr) {
                    m_suspend_state.replan_trajectory.push_back(itr->joints);
                }
                int result = planner_->Restart(m_suspend_state.replan_trajectory, m_suspend_state.last_point);
                if (0 == result) {
                    JointPoint jp;
                    jp.id = m_joint_trajectory_fifo.front().id;
                    m_joint_trajectory_fifo.clear();
                    for (std::vector<JointValues>::iterator itr = m_suspend_state.replan_trajectory.begin();
                            itr != m_suspend_state.replan_trajectory.end(); ++itr) {
                        jp.joints = *itr;
                        m_joint_trajectory_fifo.push_back(jp);
                    }
                }
                else {
                    err = MOTION_INTERNAL_FAULT;
                    m_suspend_state.is_suspended = false;
                    unlockArmGroup();
                    log.error("ERROR while replanning resume trajectory, error_code=%d", result);
                    return false;
                }
            }
            else {
                m_suspend_state.is_suspended = false;
                unlockArmGroup();
                log.error("ERROR while filling FIFO2, error_code=0x%llx", err);
                return false;
            }
            m_suspend_state.is_suspended = false;
            unlockArmGroup();
            log.info("Resume successfully.");
            return true;
        default:
            unlockArmGroup();
            err = MOTION_INTERNAL_FAULT;
            log.error("Program internal fault, error code=0x%llx", err);
            return false;
    } // switch (m_suspend_state.pattern) {
}




// ------------------------------------private function---------------------------------------------

inline void fst_controller::ArmGroup::printJointValues(const JointValues &joint) {
    log.info("%lf, %lf, %lf, %lf, %lf, %lf",
             joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6);
}

inline void fst_controller::ArmGroup::printJointValues(const char *str, const JointValues &joint) {
    log.info("%s%lf, %lf, %lf, %lf, %lf, %lf", str,
             joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6);
}

inline void fst_controller::ArmGroup::printJointLimit(const JointLimit &joint_limit) {
    log.info("lower=%lf, home=%lf, upper=%lf, max_omega=%lf, max_alpha=%lf",
             joint_limit.lower, joint_limit.home, joint_limit.upper,
             joint_limit.max_omega, joint_limit.max_alpha);
}

inline void fst_controller::ArmGroup::printJointLimit(const char *str, const JointLimit &joint_limit) {
    log.info("%slower=%lf, home=%lf, upper=%lf, max_omega=%lf, max_alpha=%lf", str,
             joint_limit.lower, joint_limit.home, joint_limit.upper,
             joint_limit.max_omega, joint_limit.max_alpha);
}

inline void fst_controller::ArmGroup::printJointConstraints(const JointConstraints &constraint) {
    log.info("Joint constraints:");
    printJointLimit("  J1 limit: ", constraint.j1);
    printJointLimit("  J2 limit: ", constraint.j2);
    printJointLimit("  J3 limit: ", constraint.j3);
    printJointLimit("  J4 limit: ", constraint.j4);
    printJointLimit("  J5 limit: ", constraint.j5);
    printJointLimit("  J6 limit: ", constraint.j6);
}

inline void fst_controller::ArmGroup::printPose(const Pose &pose) {
    log.info("position=%lf,%lf,%lf orientation=%lf,%lf,%lf,%lf",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

inline void fst_controller::ArmGroup::printPose(const char *str, const Pose &pose) {
    log.info("%sposition=%lf,%lf,%lf orientation=%lf,%lf,%lf,%lf", str,
             pose.position.x, pose.position.y, pose.position.z,
             pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

inline void fst_controller::ArmGroup::printPoseEuler(const PoseEuler &pose_euler) {
    log.info("position=%lf,%lf,%lf orientation=%lf,%lf,%lf",
            pose_euler.position.x, pose_euler.position.y, pose_euler.position.z,
            pose_euler.orientation.a, pose_euler.orientation.b, pose_euler.orientation.c);
}

inline void fst_controller::ArmGroup::printPoseEuler(const char *str, const PoseEuler &pose_euler) {
    log.info("%sposition=%lf,%lf,%lf orientation=%lf,%lf,%lf", str,
            pose_euler.position.x, pose_euler.position.y, pose_euler.position.z,
            pose_euler.orientation.a, pose_euler.orientation.b, pose_euler.orientation.c);
}

inline void fst_controller::ArmGroup::printJointPoint(const JointPoint &point) {
    log.info("Joint point: ID=%d", point.id);
    log.info("  joints=%lf, %lf, %lf, %lf, %lf, %lf",
             point.joints.j1, point.joints.j2, point.joints.j3, 
             point.joints.j4, point.joints.j5, point.joints.j6);
    log.info("  omegas=%lf, %lf, %lf, %lf, %lf, %lf",
             point.omegas.j1, point.omegas.j2, point.omegas.j3,
             point.omegas.j4, point.omegas.j5, point.omegas.j6);
}

inline void fst_controller::ArmGroup::printJointPoint(const char *str, const JointPoint &point) {
    log.info(str);
    printJointPoint(point);
}

inline void fst_controller::ArmGroup::printPathPoint(const PathPoint &point) {
    switch (point.type) {
        case enum_Type_MoveJ:
            log.info("Path point: ID=%d, type=MoveJ, user_frame=%d, tool_frame=%d, velocity=%lf",
                     point.id, point.user_frame, point.tool_frame, point.velocity);
            printJointValues("  joints=", point.joints);
            break;

        case enum_Type_MoveL:
            log.info("Path point: ID=%d, type=MoveL, user_frame=%d, tool_frame=%d, velocity=%lf",
                     point.id, point.user_frame, point.tool_frame, point.velocity);
            printPose("  pose: ", point.pose);
            break;

        case enum_Type_MoveC:
            log.info("Path point: ID=%d, type=MoveC, user_frame=%d, tool_frame=%d, velocity=%lf",
                     point.id, point.user_frame, point.tool_frame, point.velocity);
            printPose("  pose: ", point.pose);
            break;

        default:
            log.error("Type error: undefined point type.");
    }
}

inline void fst_controller::ArmGroup::printPathPoint(const char *str, const PathPoint &point) {
    log.info(str);
    printPathPoint(point);
}


inline void fst_controller::ArmGroup::lockArmGroup(void) {
    //log.info("Try to lock arm group.");
    pthread_mutex_lock(&m_group_mutex);
}

inline void fst_controller::ArmGroup::unlockArmGroup(void) {
    pthread_mutex_unlock(&m_group_mutex);
    //log.info("Arm group unlocked.");
}


//------------------------------------------------------------------------------
// Function:    setCycleTime
// Summary: To set cycle time of interpolation algorithm.
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
void fst_controller::ArmGroup::setCycleTime(void) {
    planner_->setCycleTime(m_cycle_time);
}


//------------------------------------------------------------------------------
// Function:    setMaxAcceleration
// Summary: To set the max acceleration in algorithm
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
void fst_controller::ArmGroup::setMaxAcceleration(void) {
    planner_->setAcceleration(m_amax*m_amax_scaling_factor);
}

//------------------------------------------------------------------------------
// Function:    setJumpThresholdScalingFactor
// Summary: To set jump threshold scaling factor.
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
void fst_controller::ArmGroup::setJumpThresholdScalingFactor(void) {
    planner_->setLimitScale(m_jump_threshold_scaling);
}


//------------------------------------------------------------------------------
// Function:    setJointConstraints
// Summary: To set joint constraints in Kinematics algorithm.
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
void fst_controller::ArmGroup::setJointConstraints(void) {
    planner_->setAxisLimit(m_joint_constraints);
}


void fst_controller::ArmGroup::setToolFrame(void) {
    PoseEuler pose;
    pose.position = m_tool_frame.position;
    pose.orientation = m_tool_frame.orientation;
    planner_->setToolFrame(pose);
}

void fst_controller::ArmGroup::setUserFrame(void) {
    PoseEuler pose;
    pose.position = m_user_frame.position;
    pose.orientation = m_user_frame.orientation;
    planner_->setToolFrame(pose);
}


bool fst_controller::ArmGroup::loadJointLimit(const std::string joint, JointLimit &limit) {
    std::string locate = "/fst_param/motion_controller/kinematics_limit/" + joint;
    XmlRpc::XmlRpcValue params;
    if (ros::param::get(locate, params)) {
        if (params.hasMember("home"))
            limit.home = params["home"];
        else
            return false;

        if (params.hasMember("upper"))
            limit.upper = params["upper"];
        else
            return false;

        if (params.hasMember("lower"))
            limit.lower = params["lower"];
        else
            return false;

        if (params.hasMember("omega_max"))
            limit.max_omega = params["omega_max"];
        else
            return false;

        if (params.hasMember("alpha_max"))
            limit.max_alpha = params["alpha_max"];
        else
            return false;
    }
    else {
        return false;
    }

    return true;
}


//------------------------------------------------------------------------------
// Function:    loadJointConstraints
// Summary: To load joint constraints from parameter server.
// In:      None
// Out:     None
// Return:  true  -> load parameter successfully
//          false -> load parameter UNsuccessfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::loadJointConstraints(JointConstraints &constraint, ErrorCode &err) {
    log.info("Loading joint constraints from parameter server...");
    
    if (loadJointLimit("j1", constraint.j1) &&
        loadJointLimit("j2", constraint.j2) &&
        loadJointLimit("j3", constraint.j3) &&
        loadJointLimit("j4", constraint.j4) &&
        loadJointLimit("j5", constraint.j5) &&
        loadJointLimit("j6", constraint.j6)) {
            err = SUCCESS;
            log.info("Success!");
            return true;
    }
    else {
        err = FAIL_LOADING_CONSTRAINT;
        log.error("Fail to load joint constraints, error code=0x%llx!", err);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    checkJointBoundary
// Summary: To check whether a group of joint values are valid according to
//          joint constraints.
// In:      joint_values -> joint_values needed to be checked
// Out:     None
// Return:  true  -> valid
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::checkJointBoundary(const JointValues &joint_values) {
    return joint_values.j1 > m_joint_constraints.j1.lower &&
           joint_values.j1 < m_joint_constraints.j1.upper &&
           joint_values.j2 > m_joint_constraints.j2.lower &&
           joint_values.j2 < m_joint_constraints.j2.upper &&
           joint_values.j3 > m_joint_constraints.j3.lower &&
           joint_values.j3 < m_joint_constraints.j3.upper &&
           joint_values.j4 > m_joint_constraints.j4.lower &&
           joint_values.j4 < m_joint_constraints.j4.upper &&
           joint_values.j5 > m_joint_constraints.j5.lower &&
           joint_values.j5 < m_joint_constraints.j5.upper &&
           joint_values.j6 > m_joint_constraints.j6.lower &&
           joint_values.j6 < m_joint_constraints.j6.upper;
}


//------------------------------------------------------------------------------
// Function:    getLatestIKReference
// Summary: To get latest IK reference values.
// In:      None
// Out:     None
// Return:  a group of joint values used as IK reference
//------------------------------------------------------------------------------
const fst_controller::JointValues& fst_controller::ArmGroup::getLatestIKReference(void) {
    return m_latest_ik_reference;
}


//------------------------------------------------------------------------------
// Function:    transformPoseEuler2Pose
// Summary: To transform a poseEuler point to a pose point.
// In:      poes_e -> the poseEuler to be transformed
// Out:     None
// Return:  pose point
//------------------------------------------------------------------------------
fst_controller::Pose fst_controller::ArmGroup::transformPoseEuler2Pose(const PoseEuler &pose_e) {
    fst_controller::Pose pose;
    
    int result = planner_->Euler2Quatern(pose_e, pose);
    if (result != 0)
        log.error("Error while transform Euler to Quaternion, error code=%d", result);

    return pose;
}


//------------------------------------------------------------------------------
// Function:    transformPose2PoseEuler
// Summary: To transform a pose point to a poseEuler point.
// In:      poes -> the pose to be transformed
// Out:     None
// Return:  poseEuler point
//------------------------------------------------------------------------------
fst_controller::PoseEuler fst_controller::ArmGroup::transformPose2PoseEuler(const Pose &pose) {
    fst_controller::PoseEuler pose_e;
    
    int result = planner_->Quatern2Euler(pose, pose_e);
    if (result != 0)
        log.error("Error while transform Quaternion to Euler, error code=%d", result);
    
    return pose_e;
}


//------------------------------------------------------------------------------
// Function:    computeInverseKinematics
// Summary: To compute IK with a given pose in cartesian space.
// In:      poes         -> cartesian space pose needed to compute IK
//          joint_reference -> joint reference used during compute IK
// Out:     joint_result -> IK result
//          error_code   -> error code
// Return:  true         -> IK solution found
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::computeInverseKinematics(
        const Pose &pose,
        const JointValues &joint_reference,
        JointValues &joint_result,
        ErrorCode &err) {
    int res = planner_->InverseKinematics(pose, joint_reference, joint_result);

    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1001:
            err = IK_OUT_OF_WORKSPACE;
            return false;
        case 1002:
            err = IK_JOINT_OUT_OF_LIMIT;
            return false;
        case 1003:
            err = IK_EXCESSIVE_DISTANCE;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}


//------------------------------------------------------------------------------
// Function:    computeForwardKinematics
// Summary: To compute FK with given joint values.
// In:      joint_result -> joint values needed to compute FK
// Out:     poes         -> FK result
//          error_code   -> error code
// Return:  true         -> FK computed successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::computeForwardKinematics(
        const JointValues &joint, Pose &pose, ErrorCode &err) {
    int res = planner_->ForwardKinematics(joint, pose);
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1011:
            err = FK_JOINT_OUT_OF_LIMIT;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveJ2L
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:      jp_start    -> initial position and omega of six joints
//          joint_target-> joint target of the plan
//          v_percent   -> percentage of velocity, range: 0.0-100.0
//          cnt         -> desired cnt value of the plan, range:0-100
//          pose_next   -> the target pose of the next plan used for smoothing
//          v_next      -> desired velocity of the next plan used for smoothing
//          cnt_next    -> desired cnt value of the next plan used for smoothing
// Out:     pose_start  -> the end pose of this plan, also the initial pose of the next plan
//          pose_previous -> the pose_previous input in the next plan
//          v_start     -> the end velocity of this plan, also the initial velocity of the next plan
//          vu_start    -> the end value of intermediate-variable in this plan, also the initial value in next plan
//          planned_path-> planned path in joint space
//          error_code  -> error code
// Return:  true        -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ2L(const JointPoint &jp_start,
                                       JointValues &joint_target, double v_percent, int cnt,
                                       const Pose &pose_next, double v_next, int cnt_next,
                                       std::vector<JointValues> &planned_path,
                                       Pose &pose_start, Pose &pose_previous,
                                       double &v_start, double &vu_start,
                                       ErrorCode &err) {
    int res = planner_->MoveJ2L_Jspace(jp_start,
                                        joint_target, 100*v_percent, cnt,
                                        pose_next, v_next,
                                        planned_path,
                                        pose_start, v_start, vu_start,
                                        pose_previous);

    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1031:
            err = AXIS_OVERSHOOT;;
            return false;
        case 1032:
            err = AXIS_APPROACHING_LIMIT;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveJ2L
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:      jp_start    -> initial position and omega of six joints
//          pose_target -> joint target of the plan
//          v_percent   -> percentage of velocity, range: 0.0-100.0
//          cnt         -> desired cnt value of the plan, range:0-100
//          pose_next   -> the target pose of the next plan used for smoothing
//          v_next      -> desired velocity of the next plan used for smoothing
//          cnt_next    -> desired cnt value of the next plan used for smoothing
// Out:     pose_start  -> the end pose of this plan, also the initial pose of the next plan
//          pose_previous -> the pose_previous input in the next plan
//          v_start     -> the end velocity of this plan, also the initial velocity of the next plan
//          vu_start    -> the end value of intermediate-variable in this plan, also the initial value in next plan
//          planned_path-> planned path in joint space
//          error_code  -> error code
// Return:  true  -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ2L(const JointPoint &jp_start,
                                       const Pose &pose_target, double v_percent, int cnt,
                                       const Pose &pose_next, double v_next, int cnt_next,
                                       std::vector<JointValues> &planned_path,
                                       Pose &pose_start, Pose &pose_previous,
                                       double &v_start, double &vu_start,
                                       ErrorCode &err) {
    int res = planner_->MoveJ2L_Cspace(jp_start,
                                        pose_target, 100*v_percent, cnt,
                                        pose_next, v_next,
                                        planned_path,
                                        pose_start, v_start, vu_start,
                                        pose_previous);

    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1031:
            err = AXIS_OVERSHOOT;
            return false;
        case 1032:
            err = AXIS_APPROACHING_LIMIT;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveJ2J
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:      jp_start    -> initial position and omega of six joints
//          jp_target   -> joint target of the plan
//          jp_next     -> the target joint of the next plan used for smoothing
//          v_percent   -> percentage of velocity, range: 0.0-100.0
//          cnt         -> desired cnt value of the plan, range:0-100
// Out:     jp_end      -> the ending position and omega of six joints in this plan
//          planned_path-> planned path in joint space
//          error_code  -> error code
// Return:  true        -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ2J(JointPoint &jp_start,
                                       JointPoint &jp_target, JointPoint &jp_next, double v_percent, int cnt,
                                       std::vector<JointValues> &planned_path,
                                       ErrorCode &err) {
    int res = planner_->MoveJ2J_Jspace(jp_start, jp_target, jp_next, 100*v_percent, cnt, planned_path);
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1031:
            err = AXIS_OVERSHOOT;
            return false;
        case 1032:
            err = AXIS_APPROACHING_LIMIT;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}

//------------------------------------------------------------------------------
// Function:    MoveJ2J
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:      jp_start    -> initial position and omega of six joints
//          pose_target -> pose target of the plan
//          pose_next   -> the target pose of the next plan used for smoothing 
//          v_percent   -> percentage of velocity, range: 0.0-100.0
//          cnt         -> desired cnt value of the plan, range:0-100
// Out:     jp_end      -> the ending position and omega of six joints in this plan
//          planned_path-> planned path in joint space
//          error_code  -> error code
// Return:  true        -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ2J(JointPoint &jp_start,
                                       const Pose &pose_target, const Pose &pose_next, double v_percent, int cnt,
                                       std::vector<JointValues> &planned_path,
                                       ErrorCode &err) {
    int res = planner_->MoveJ2J_Cspace(jp_start, pose_target, pose_next, 100*v_percent, cnt, planned_path);
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1031:
            err = AXIS_OVERSHOOT;
            return false;
        case 1032:
            err = AXIS_APPROACHING_LIMIT;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveL2L
// Summary: To plan a linear path to touch target pose, with/without smooth.
// In:      pose_start  -> initial pose of the plan
//          v_start     -> initial velocity of the plan
//          vu_start    -> initial value of the intermediate-variable
//          pose_target -> target pose of the plan
//          v_target    -> desired velocity of the plan
//          cnt_target  -> desired cnt value of the plan
//          pose_next   -> the target pose of the next plan used for smoothing
//          v_next      ->  desired velocity of the next plan used for smoothing
//          cnt_next    -> desired cnt value of the next plan used for smoothing
//          pose_previuos -> the previous pose used for interpolation
// Out:     pose_start  -> the end pose of this plan, also the initial pose of the next plan
//          v_start     -> the end velocity of this plan, also the initial pose of the next plan
//          vu_start    -> the end value of intermediate-variable in this plan, also the initial value in next plan
//          pose_previous -> the pose_previous in the next plan
//          planned_path-> planned path in cartesian space
//          error_code  -> error code
// Return:  true        -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL2L(Pose &pose_start, double &v_start, double &vu_start,
                                       const Pose &pose_target, double v_target, int cnt_target,
                                       const Pose &pose_next, double v_next,
                                       Pose &pose_previous,
                                       std::vector<Pose> &planned_path,
                                       ErrorCode &err) {

    /*
    ROS_INFO("Call planner_->MoveL");
    ROS_INFO("pose start");
    printPose(pose_start);
    ROS_INFO("v0 = %f, vu0 = %f", v_start, vu_start);
    ROS_INFO("pose target");
    printPose(pose_target);
    ROS_INFO("v_target = %f, cnt_target = %d", v_target, cnt_target);
    ROS_INFO("pose next");
    printPose(pose_next);
    ROS_INFO("v_next = %f", v_next);
    ROS_INFO("pose pre");
    printPose(pose_previous);
    */

    int res = planner_->MoveL(pose_start, v_start, vu_start,
                               pose_target, v_target, cnt_target,
                               pose_next, v_next,
                               pose_previous, planned_path);

    /*
    // For test 
    // print all planned points 
    int cnt=0;
    std::vector<fst_controller::Pose>::iterator itr=planned_path.begin();
    for(;itr!=planned_path.end();++itr) {
	ROS_INFO("Pose-%d: position(%f,%f,%f), orientation(%f,%f,%f,%f)",++cnt,
		itr->position.x,itr->position.y,itr->position.z,
		itr->orientation.w,itr->orientation.x,itr->orientation.y,itr->orientation.z);
    }
    */
    
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1021:
            err = PLANNING_UNSMOOTHABLE;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}

//------------------------------------------------------------------------------
// Function:    MoveL2J
// Summary: To plan a linear path to touch target pose, with/without smooth.
// In:      pose_start  -> initial pose of the plan
//          v_start     -> initial velocity of the plan
//          vu_start    -> initial value of the intermediate-variable
//          pose_target -> target pose of the plan
//          v_target    -> desired velocity of the plan
//          cnt_target  -> desired cnt value of the plan
//          pose_previuos-> the previous pose used for interpolation
// Out:     planned_path-> planned path in cartesian space
//          jp          -> position and omega of six joints at the ending of this path
//          error_code  -> error code
// Return:  true  -> plan successfully
//------------------------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL2J(const Pose &pose_start, double v_start, double vu_start,
                                       const Pose &pose_target, double v_target, int cnt_target,
                                       Pose &pose_previous, std::vector<Pose> &planned_path,
                                       ErrorCode &err) {
    int res = planner_->MoveL2J(pose_start, v_start, vu_start,
                                 pose_target, v_target, cnt_target,
                                 pose_previous, planned_path);

    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1021:
            err = PLANNING_UNSMOOTHABLE;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}


bool fst_controller::ArmGroup::MoveC2CPart1(Pose &pose_start, double &v_start,
                                           const Pose &pose_2nd, const Pose &pose_3rd, double v_target, int cnt,
                                           const Pose &pose_2nd_next, const Pose &pose_3rd_next, double v_next,
                                           std::vector<Pose> &planned_path, Pose &pose_start_ptc,
                                           ErrorCode err) {
    int res = planner_->MoveC2C_part1(pose_start, v_start,
                                       pose_2nd, pose_3rd, v_target, cnt,
                                       pose_2nd_next, pose_3rd_next, v_next,
                                       planned_path, pose_start_ptc);

    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1051:
        case 1052:
        case 1053:
        case 1054:
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}


bool fst_controller::ArmGroup::MoveC2CPart2(const JointPoint &jp_start, const Pose &pose_start,
                                            const Pose &pose_start_ptc, std::vector<JointValues> &planned_path,
                                            ErrorCode err) {
    int res = planner_->MoveC2C_part2(jp_start, pose_start, pose_start_ptc, planned_path);
    
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}


bool fst_controller::ArmGroup::MoveC2J(const Pose &pose_start, double v_start,
                                       const Pose &pose_2nd, const Pose &pose_3rd, double v_target, int cnt,
                                       std::vector<Pose> &planned_path, ErrorCode err) {
    int res = planner_->MoveC2J(pose_start, v_start, pose_2nd, pose_3rd, v_target, cnt, planned_path);
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        default:
            err = MOTION_INTERNAL_FAULT;
        return false;
    }
}




