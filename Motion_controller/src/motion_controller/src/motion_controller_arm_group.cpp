/**********************************************************************
    Copyright:  Foresight-Robotics
    File:       fst_controller.cpp
    Author:     Feng Yun
    Data:       Aug.1  2016
    Modify:     Aug.30 2016
    Description:ArmGroup--Source code.
**********************************************************************/


#include <motion_controller/motion_controller_arm_group.h>
#include <struct_to_mem/struct_feedback_joint_states.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <ros/ros.h>
#include <XmlRpc.h>
#include <assert.h>

using std::cout;
using std::endl;

namespace fst_controller {

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
ArmGroup::ArmGroup(void) {
    planning_interface_ = NULL;
    calibrator_         = NULL;
    current_state_      = UNDEFINED;
    enable_calibration_ = false;
    allowed_motion_type_= MOTION_UNDEFINED;
    m_suspend_state.is_suspended    = false;
    
    last_motion_.id = 0;
    last_motion_.addition_smooth = false;
    last_motion_.smooth_type = SMOOTH_NONE;
    last_motion_.motion_type = MOTION_UNDEFINED;

    m_JOINT_FIFO_LEN            = 0;
    m_vu_start      = 0.0;
    m_v_start       = 0.0;

    memset((void*)&m_tool_frame,    0, sizeof(m_tool_frame));
    memset((void*)&m_user_frame,    0, sizeof(m_user_frame));
    memset((void*)&m_pose_start,    0, sizeof(m_pose_start));
    memset((void*)&m_pose_previous, 0, sizeof(m_pose_previous));
    memset((void*)&m_joint_start,   0, sizeof(m_joint_start));
    memset((void*)&m_pose_start_past,     0, sizeof(m_pose_start));
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
ArmGroup::~ArmGroup(void) {
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
    delete planning_interface_; planning_interface_ = NULL;
    delete calibrator_;         calibrator_         = NULL;
    log.warn("ArmGroup exit");
}




//------------------------------------------------------------------------------
// Function:    initArmGroup
// Summary: Initial all the resources in the class
// In:      joint_values -> initial state of all joints
// Out:     error_code   -> error code
// Return:  None
//------------------------------------------------------------------------------
bool ArmGroup::initArmGroup(ErrorCode &err) {
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
    
    log.info("Loading ArmGroup configs from parameter server ...");
    XmlRpc::XmlRpcValue params;
    if (ros::param::get("/fst_param/motion_controller/arm_group", params)) {
        if (setArmGroupParameters(params)) {
            log.info("  set FIFO2 size in normal option to %d", m_JOINT_FIFO_LEN);
            enable_calibration_ ? log.info("  zero offset calibration: enabled")
                                : log.info("  zero offset calibration: disabled");
            log.info("Success!");
        }
        else {
            err = INVALID_PARAMETER;
            log.error("Cannot set ArmGroup parameters, error code=0x%llx", err);
            current_state_ = UNDEFINED;
            return false;
        }
    }
    else {
        err = FAIL_LOADING_PARAMETER;
        log.error("Fail loading parameters from remote server, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    
    log.info("Constructing planner and calibrator ...");
    if (calibrator_ != NULL) {
        delete calibrator_;
        calibrator_ = NULL;
    }
    if (planning_interface_ != NULL) {
        delete planning_interface_;
        planning_interface_ = NULL;
    }
    calibrator_         = new fst_controller::Calibrator(log);
    planning_interface_ = new fst_controller::PlanningInterface();
    

    if (planning_interface_ == NULL || calibrator_ == NULL) {
        err = MOTION_FAIL_IN_INIT;
        log.error("Cannot construct planner or calibrator, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    else {
        XmlRpc::XmlRpcValue params;
        if (!ros::param::get("/fst_param/motion_controller/planning_interface", params)) {
            err = FAIL_LOADING_PARAMETER;
            log.error("Fail loading parameters from remote server, error code=0x%llx", err);
            current_state_ = UNDEFINED;
            return false;
        }
        if (planning_interface_->initPlanningInterface(params)) {
            log.info("  set cycle time to %.4lf s", planning_interface_->getCycleTime());
            log.info("  set velocity to %.2lf mm/s", planning_interface_->getVelocity());
            log.info("  set acceleration to %.2lf mm/s^2", planning_interface_->getAcceleration());
            log.info("  set velocity scaling to %.2lf%%", planning_interface_->getVelocityScaling() * 100);
            log.info("  set acceleration scaling to %.2lf%%", planning_interface_->getAccelerationScaling() * 100);
            log.info("  set jerk to %.2lf", planning_interface_->getJerk());
            log.info("  set joint overshoot to %.4lf rad", planning_interface_->getJointOvershoot());
            log.info("  set joint error angle to %.4lf rad", planning_interface_->getJointErrorAngle());
            log.info("  set omega overload to %.2lf%%", planning_interface_->getOmegaOverload() * 100);
            log.info("  set alpha overload to %.2lf%%", planning_interface_->getAlphaOverload() * 100);
            log.info("  set smooth radius coefficient to %.2lf", planning_interface_->getSmoothRadiusCoefficient());
            log.info("  set smooth curve mode to '%s'", ((string)params["smooth_curve_mode"]).c_str());
            log.info("Success!");
        }
        else {
            err = MOTION_FAIL_IN_INIT;
            log.error("Fail to initialize planning_interface");
            current_state_ = UNDEFINED;
            return false;
        }
    }

    log.info("Loading joint constraints from parameter server ...");
    XmlRpc::XmlRpcValue kinematics_limit;
    if (ros::param::get("/fst_param/motion_controller/kinematics_limit", kinematics_limit)) {
        JointConstraints constraints;
        try {
            constraints.j1.home     = kinematics_limit["j1"]["home"];
            constraints.j1.upper    = kinematics_limit["j1"]["upper"];
            constraints.j1.lower    = kinematics_limit["j1"]["lower"];
            constraints.j1.max_omega= kinematics_limit["j1"]["omega_max"];
            constraints.j1.max_alpha= kinematics_limit["j1"]["alpha_max"];
            constraints.j2.home     = kinematics_limit["j2"]["home"];
            constraints.j2.upper    = kinematics_limit["j2"]["upper"];
            constraints.j2.lower    = kinematics_limit["j2"]["lower"];
            constraints.j2.max_omega= kinematics_limit["j2"]["omega_max"];
            constraints.j2.max_alpha= kinematics_limit["j2"]["alpha_max"];
            constraints.j3.home     = kinematics_limit["j3"]["home"];
            constraints.j3.upper    = kinematics_limit["j3"]["upper"];
            constraints.j3.lower    = kinematics_limit["j3"]["lower"];
            constraints.j3.max_omega= kinematics_limit["j3"]["omega_max"];
            constraints.j3.max_alpha= kinematics_limit["j3"]["alpha_max"];
            constraints.j4.home     = kinematics_limit["j4"]["home"];
            constraints.j4.upper    = kinematics_limit["j4"]["upper"];
            constraints.j4.lower    = kinematics_limit["j4"]["lower"];
            constraints.j4.max_omega= kinematics_limit["j4"]["omega_max"];
            constraints.j4.max_alpha= kinematics_limit["j4"]["alpha_max"];
            constraints.j5.home     = kinematics_limit["j5"]["home"];
            constraints.j5.upper    = kinematics_limit["j5"]["upper"];
            constraints.j5.lower    = kinematics_limit["j5"]["lower"];
            constraints.j5.max_omega= kinematics_limit["j5"]["omega_max"];
            constraints.j5.max_alpha= kinematics_limit["j5"]["alpha_max"];
            constraints.j6.home     = kinematics_limit["j6"]["home"];
            constraints.j6.upper    = kinematics_limit["j6"]["upper"];
            constraints.j6.lower    = kinematics_limit["j6"]["lower"];
            constraints.j6.max_omega= kinematics_limit["j6"]["omega_max"];
            constraints.j6.max_alpha= kinematics_limit["j6"]["alpha_max"];
        }
        catch (XmlRpc::XmlRpcException &exception) {
            log.error("Exception:");
            log.error(exception.getMessage().c_str());
            err = MOTION_FAIL_IN_INIT;
            current_state_ = UNDEFINED;
            return false;
        }
        
        if (setJointConstraints(constraints)) {
            log.info("Success!");
        }
        else {
            err = MOTION_FAIL_IN_INIT;
            current_state_ = UNDEFINED;
            return false;
        }
    }
    else {
        err = FAIL_LOADING_PARAMETER;
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

    unsigned int calibrate_result;
    if (checkZeroOffset(calibrate_result, err)) {
        log.info("Using current joint values to initialize ArmGroup:");
        FeedbackJointState fbjs;
        calibrator_->getCurrentJoint(fbjs);
        JointValues joint;
        memcpy(&joint, fbjs.position, 6 * sizeof(double));
        printJointValues("  joints=", joint);
        if (!setStartState(joint, err)) {
            log.error("Cannot set start state with current joint values, error code=0x%llx", err);
            if (err == JOINT_OUT_OF_CONSTRAINT) {
                log.warn("Reset zero offset to a temporary position, need calibration.");
                if (calibrator_->setTemporaryZeroOffset()) {
                    log.info("Success!");
                    log.info("Using temporary joint values to initialize start state:");
                    calibrator_->getCurrentJoint(fbjs);
                    JointValues joint;
                    memcpy(&joint, fbjs.position, 6 * sizeof(double));
                    printJointValues("  joints=", joint);
                    if (!setStartState(joint, err)) {
                        current_state_ = UNDEFINED;
                        log.error("Cannot set start state with temporary joint values, error code=0x%llx", err);
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
    }
    else {
        // reset zero offset
        log.warn("Reset zero offset to a temporary position, need calibration.");
        if (calibrator_->setTemporaryZeroOffset()) {
            log.info("Success!");
            log.info("Using temporary joint values to initialize start state:");
            FeedbackJointState fbjs;
            calibrator_->getCurrentJoint(fbjs);
            JointValues joint;
            memcpy(&joint, fbjs.position, 6 * sizeof(double));
            printJointValues("  joints=", joint);
            if (!setStartState(joint, err)) {
                current_state_ = UNDEFINED;
                log.error("Cannot set start state with temporary joint values, error code=0x%llx", err);
                return false;
            }
        }
        else {
            log.error("Cannot set temporary zero offset");
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

bool ArmGroup::checkZeroOffset(unsigned int &calibrate_result, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    if (enable_calibration_ == true) {
        log.info("Reviewing starting joint ...");
        unsigned int result;
        if (calibrator_->reviewLastJoint(result)) {
            calibrate_result = result;
            if (result == OFFSET_NORMAL) {
                log.info("Success!");
                err = SUCCESS;
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
    
    /*
    log.info("Using current joint values to initialize ArmGroup:");
    FeedbackJointState fbjs;
    calibrator_->getCurrentJoint(fbjs);
    JointValues joint;
    memcpy(&joint, fbjs.position, 6 * sizeof(double));
    printJointValues("  joints=", joint);
    setStartState(joint, err);
    */

    if (err == SUCCESS)
        return true;
    else
        return false;
}

bool ArmGroup::calibrateZeroOffset(unsigned int &calibrate_result, ErrorCode &err) {
    err = SUCCESS;
    
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    if (enable_calibration_ != true) {
        err = CALIBRATION_FAULT;
        return false;
    }

    if (!calibrator_->recordZeroOffset()) {
        err = calibrator_->getLastError();
        return false;
    }

    if (!calibrator_->reloadJTACParam()) {
        err = calibrator_->getLastError();
        return false;
    }

    log.info("Downloading zero offset parameters ...");
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


//------------------------------------------------------------------------------
// Function:    getCycleTime
// Summary: To get cycle time of interpolation algorithm.
// In:      None
// Out:     None
// Return:  cycle time
//------------------------------------------------------------------------------
double ArmGroup::getCycleTime(void) {
    return planning_interface_->getCycleTime();
}


//------------------------------------------------------------------------------
// Function:    getJointConstraints
// Summary: To get joint constraints from Kinematics algorithm.
// In:      None
// Out:     None
// Return:  joint constraints
//------------------------------------------------------------------------------
const JointConstraints& fst_controller::ArmGroup::getJointConstraints(void) {
    return planning_interface_->getJointConstraints();
}


//------------------------------------------------------------------------------
// Function:    getMaxVelocity
// Summary: To get max velocity settings.
// In:      None
// Out:     None
// Return:  value of max velocity
//------------------------------------------------------------------------------
double ArmGroup::getMaxVelocity(void) {
    return planning_interface_->getVelocity();
}


//------------------------------------------------------------------------------
// Function:    getMaxAcceleration
// Summary: To get max acceleration settings.
// In:      None
// Out:     None
// Return:  value of max acceleration
//------------------------------------------------------------------------------
double ArmGroup::getMaxAcceleration(void) {
    return planning_interface_->getAcceleration();
}


//------------------------------------------------------------------------------
// Function:    getVelocityScalingFactor
// Summary: To get velocity scaling factor value.
// In:      None
// Out:     None
// Return:  global scaling factor for velocity
//------------------------------------------------------------------------------
double ArmGroup::getVelocityScalingFactor(void) {
    return planning_interface_->getVelocityScaling();
}


//------------------------------------------------------------------------------
// Function:    getAccelerationScalingFactor
// Summary: To get acceleration scaling factor value.
// In:      None
// Out:     None
// Return:  global scaling factor for acceleration
//------------------------------------------------------------------------------
double ArmGroup::getAccelerationScalingFactor(void) {
    return planning_interface_->getAccelerationScaling();
}


//------------------------------------------------------------------------------
// Function:    getCurrentJointValues
// Summary: To get current values of all joints in the robot.
// In:      None
// Out:     None
// Return:  current values of all six joints
//------------------------------------------------------------------------------
const JointValues& ArmGroup::getCurrentJointValues(void) {
    return m_joint_state_current;
}


//------------------------------------------------------------------------------
// Function:    getCurrentPose
// Summary: To get current pose of endpoint in the arm group.
// In:      None
// Out:     None
// Return:  current pose of the endpoint
//------------------------------------------------------------------------------
const Pose& ArmGroup::getCurrentPose(void) {
    return m_pose_state_current;
}


const JointPoint& ArmGroup::getStartJoint(void) {
    return m_joint_start;
}


const Pose& ArmGroup::getStartPose(void) {
    return m_pose_start;
}


//------------------------------------------------------------------------------
// Function:    getPlannedPathFIFOLength
// Summary: To get the length of planned_path_FIFO.
// In:      None
// Out:     None
// Return:  length of the FIFO
//------------------------------------------------------------------------------
int ArmGroup::getPlannedPathFIFOLength(void) {
    return m_planned_path_fifo.size();
}


//------------------------------------------------------------------------------
// Function:    getJointTrajectoryFIFOLength
// Summary: To get the length of joitn_trajectory_FIFO.
// In:      None
// Out:     None
// Return:  length of the FIFO
//------------------------------------------------------------------------------
int ArmGroup::getJointTrajectoryFIFOLength(void) {
    return m_joint_trajectory_fifo.size();
}


//------------------------------------------------------------------------------
// Function:    getPointsFromJointTrajectoryFIFO
// Summary: To get points from joitn_trajectory_FIFO.
// In:      num -> number of joint points want to get
// Out:     traj-> joint trajectory consisting of joint points
//          error_code  -> error code
// Return:  <0  -> joint_trajectory_fifo locked, or any other errors
//          >=0 -> number of joint points get actually
//------------------------------------------------------------------------------
int ArmGroup::getPointsFromJointTrajectoryFIFO(vector<JointPoint> &traj, int num, ErrorCode &err) {
    err = SUCCESS;
    
    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

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
            if ((itr->id & POINT_LEVEL_MASK) == POINT_ENDING) {
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
bool ArmGroup::setCycleTime(double tc) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setCycleTime(tc)) {
        log.info("Set cycle time to %.4fs", tc);
        return true;
    }
    else {
        log.error("Cannot set cycle time to %.4fs", tc);
        return false;
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
bool ArmGroup::setJointConstraints(const JointConstraints &constraints) {
    if (current_state_ != INITIALIZED) {
        return false;
    }
    
    m_joint_constraints = constraints;
    printJointConstraints(constraints);
    return planning_interface_->setJointConstraints(constraints);
}


//------------------------------------------------------------------------------
// Function:    setMaxVelocity
// Summary: To change max velocity settings.
// In:      v       -> desired max velocity
// Out:     None
// Return:  true    -> max velocity changed to given value
//          false   -> max velocity NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setMaxVelocity(double v) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setVelocity(v)) {
        log.info("Set velocity to %.2fmm/s", v);
        return true;
    }
    else {
        log.error("Cannot set velocity to %.2fmm/s", v);
        return false;
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
bool ArmGroup::setMaxAcceleration(double a) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setAcceleration(a)) {
        log.info("Set acceleration to %.2fmm/s^2", a);
        return true;
    }
    else {
        log.error("Cannot set acceleration to %.2fmm/s^2", a);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setVelocityScalingFactor
// Summary: To set velocity scaling factor.
// In:      factor-> desired velocity scaling factor
// Out:     None
// Return:  true  -> velocity scaling factor changed to given value
//          false -> velocity scaling factor NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setVelocityScalingFactor(double factor) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setVelocityScaling(factor)) {
        log.info("Set velocity scaling factor to %.2f%%", factor * 100);
        return true;
    }
    else {
        log.error("Cannot set velocity scaling factor to %.2f%%", factor * 100);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setAccelerationScalingFactor
// Summary: To set acceleration scaling factor.
// In:      factor  -> desired acceleration scaling factor
// Out:     None
// Return:  true    -> acceleration scaling factor changed to given value
//          false   -> acceleration scaling factor NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setAccelerationScalingFactor(double factor) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setAccelerationScaling(factor)) {
        log.info("Set acceleration scaling factor to %.2f%%", factor * 100);
        return true;
    }
    else {
        log.error("Cannot set acceleration scaling factor to %.2f%%", factor * 100);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setJerk
// Summary: To set the ratio of jerk to acceleration.
// In:      jerk    -> desired ratio
// Out:     None
// Return:  true    -> ratio changed to given value
//          false   -> ratio NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setJerk(double jerk) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setJerk(jerk)) {
        log.info("Set the ratio of jerk/acceleration to %.2f%%", jerk * 100);
        return true;
    }
    else {
        log.error("Cannot set the ratio of jerk/acceleration to %.2f%%", jerk * 100);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setJointOvershoot
// Summary: To set joint overshoot.
// In:      angle   -> desired angle
// Out:     None
// Return:  true    -> joint overshoot changed to given value
//          false   -> joint overshoot NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setJointOvershoot(double angle) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setJointOvershoot(angle)) {
        log.info("Set joint overshoot to %.4f rad", angle);
        return true;
    }
    else {
        log.error("Cannot set joint overshoot to %.4f rad", angle);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setJointErrorAngle
// Summary: To set joint error angle.
// In:      angle   -> desired angle
// Out:     None
// Return:  true    -> joint error angle changed to given value
//          false   -> joint error angle NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setJointErrorAngle(double angle) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setJointErrorAngle(angle)) {
        log.info("Set joint error angle to %.4f rad", angle);
        return true;
    }
    else {
        log.error("Cannot set joint error angle to %.4f rad", angle);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setOmegaOverload
// Summary: To set omega overload.
// In:      value   -> desired value
// Out:     None
// Return:  true    -> omega overload changed to given value
//          false   -> omega overload NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setOmegaOverload(double value) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setOmegaOverload(value)) {
        log.info("Set omega overload to %.2f%%", value * 100);
        return true;
    }
    else {
        log.error("Cannot set omega overload to %.2f%%", value * 100);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setAlphaOverload
// Summary: To set alpha overload.
// In:      value   -> desired value
// Out:     None
// Return:  true    -> alpha overload changed to given value
//          false   -> alpha overload NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setAlphaOverload(double value) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setAlphaOverload(value)) {
        log.info("Set alpha overload to %.2f%%", value * 100);
        return true;
    }
    else {
        log.error("Cannot set alpha overload to %.2f%%", value * 100);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setSmoothRadiusCoefficient
// Summary: To set smooth radius coefficient.
// In:      coeff   -> desired value
// Out:     None
// Return:  true    -> smooth radius coefficient changed to given value
//          false   -> smooth radius coefficient NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setSmoothRadiusCoefficient(double coeff) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setSmoothRadiusCoefficient(coeff)) {
        log.info("Set smooth radius coefficient to %.2f", coeff);
        return true;
    }
    else {
        log.error("Cannot set smooth radius coefficient to %.2f", coeff);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setSmoothCurveMode
// Summary: To set smooth curve mode.
// In:      mode    -> desired smooth curve mode
// Out:     None
// Return:  true    -> desired smooth curve mode changed to given value
//          false   -> desired smooth curve mode NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setSmoothCurveMode(int mode) {
    if (current_state_ != INITIALIZED) {
        return false;
    }

    if (planning_interface_->setSmoothCurveMode(mode)) {
        log.info("Set smooth curve mode to %d", mode);
        return true;
    }
    else {
        log.error("Cannot set smooth curve mode to %d", mode);
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    setToolFrame
// Summary: To set current tool frame.
// In:      tool_frame  -> current tool frame
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
void ArmGroup::setToolFrame(const Transformation &tool_frame) {
    if (current_state_ != INITIALIZED) {
        return;
    }

    m_tool_frame = tool_frame;
    planning_interface_->setToolFrame(tool_frame);
}


//------------------------------------------------------------------------------
// Function:    setUserFrame
// Summary: To set current user frame.
// In:      user_frame  -> current user frame
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
void ArmGroup::setUserFrame(const Transformation &user_frame) {
    if (current_state_ != INITIALIZED) {
        return;
    }

    m_user_frame = user_frame;
    planning_interface_->setUserFrame(user_frame);
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
bool ArmGroup::setCurrentJointValues(const JointValues &current_joint,
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
bool ArmGroup::setLatestIKReference(const JointValues &joint_reference, ErrorCode &err) {
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
bool ArmGroup::setStartState(const JointValues &joint_start, ErrorCode &err) {
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

        allowed_motion_type_ = MOTION_UNDEFINED;
        last_motion_.addition_smooth = false;
        last_motion_.smooth_type = SMOOTH_NONE;

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
bool ArmGroup::clearPlannedPathFIFO(ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    bool result = true;

    JointValues start_joint;
    lockArmGroup();
    if (m_joint_trajectory_fifo.size() > 0) {
        start_joint = m_joint_trajectory_fifo.back().joints;
        m_joint_trajectory_fifo.back().id &= ~POINT_LEVEL_MASK;
        m_joint_trajectory_fifo.back().id |= POINT_ENDING;
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

bool ArmGroup::resetArmGroup(const JointValues &joint, ErrorCode &err) {
    err = SUCCESS;

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

        allowed_motion_type_ = MOTION_UNDEFINED;
        last_motion_.smooth_type = SMOOTH_NONE;
        last_motion_.addition_smooth = false;
        m_suspend_state.is_suspended = false;

        m_planned_path_fifo.clear();
        m_joint_trajectory_fifo.clear();

        log.info("Success!");
        unlockArmGroup();
        return true;
    }
    else {
        log.error("Fail to reset ArmGroup using given joint values. Error cpde=0x%llx", err);
        unlockArmGroup();
        return false;
    }

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
bool ArmGroup::computeIK(const Pose &pose, JointValues &joint_result, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    return planning_interface_->computeInverseKinematics(pose, getLatestIKReference(), joint_result, err);
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
bool ArmGroup::computeFK(const JointValues &joint, Pose &pose, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }

    return planning_interface_->computeForwardKinematics(joint, pose, err);
}

//------------------------------------------------------------
// Function:    transformPoseEuler2Pose
// Summary: To transform a poseEuler point to a pose point.
// In:      poes_e  -> the poseEuler to be transformed
// Out:     None
// Return:  pose point
//------------------------------------------------------------
Pose ArmGroup::transformPoseEuler2Pose(const PoseEuler &pose_e) {
    return planning_interface_->transformPoseEuler2Pose(pose_e);
}

//------------------------------------------------------------
// Function:    transformPose2PoseEuler
// Summary: To transform a pose point to a poseEuler point.
// In:      poes    -> the pose to be transformed
// Out:     None
// Return:  poseEuler point
//------------------------------------------------------------
PoseEuler ArmGroup::transformPose2PoseEuler(const Pose &pose) {
    return planning_interface_->transformPose2PoseEuler(pose);
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
int ArmGroup::convertPathToTrajectory(int num, ErrorCode &err) {
    err = SUCCESS;

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
int ArmGroup::convertPath2Trajectory(int num, ErrorCode &err) {
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

        if (itr->type == MOTION_LINE || itr->type == MOTION_CIRCLE) {
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
                    double cycle_time = planning_interface_->getCycleTime();
                    m_joint_start.omegas.j1 = (jp.joints.j1 - joints.j1) / cycle_time;
                    m_joint_start.omegas.j2 = (jp.joints.j2 - joints.j2) / cycle_time;
                    m_joint_start.omegas.j3 = (jp.joints.j3 - joints.j3) / cycle_time;
                    m_joint_start.omegas.j4 = (jp.joints.j4 - joints.j4) / cycle_time;
                    m_joint_start.omegas.j5 = (jp.joints.j5 - joints.j5) / cycle_time;
                    m_joint_start.omegas.j6 = (jp.joints.j6 - joints.j6) / cycle_time;
                    
                    if (last_motion_.addition_smooth) {
                        insertAdditionSmooth(err);
                        last_motion_.smooth_type = SMOOTH_NONE;
                        last_motion_.addition_smooth = false;
                    }
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
        else if (itr->type == MOTION_JOINT) {
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

bool ArmGroup::isMotionExecutable(MotionType motion_type) {
    switch (motion_type) {
        case MOTION_JOINT:
            if (!m_planned_path_fifo.empty()
                && (m_planned_path_fifo.back().type == MOTION_LINE
                    ||  m_planned_path_fifo.back().type == MOTION_CIRCLE)) {
                return false;
            }
            else {
                return true;
            }
        case MOTION_LINE:
            if (last_motion_.addition_smooth && !m_planned_path_fifo.empty()) {
                return false;
            }
            else {
                return true;
            }
        case MOTION_CIRCLE:
            if (last_motion_.addition_smooth && !m_planned_path_fifo.empty()) {
                return false;
            }
            else {
                return true;
            }
        default:
            log.error("API:'isMotionExecutable' received an invalid motion type: %d", motion_type);
            return false;
    }
}

bool ArmGroup::insertAdditionSmooth(ErrorCode &err) {
    log.info("Insert additional smooth ...");
    vector<JointValues> planned_path;

    if (last_motion_.smooth_type == SMOOTH_L2C) {
        log.info("  Planning smooth path from MoevL to MoveC");
        if (planning_interface_->MoveL2CAdditionSmooth(m_joint_start, m_pose_start,
                                                       m_pose_start_past, planned_path, err)) {
            if (setLatestIKReference(planned_path.back(), err)) {
                PathPoint pp;
                pp.type = MOTION_JOINT;
                pp.id = (last_motion_.id << 2) + POINT_MIDDLE;

                vector<JointValues>::iterator itr = planned_path.begin();
                for (; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
                log.info("  Smooth path generated successfully with %zd points, and added into planned_path_FIFO.",
                         planned_path.size());
                return true;
            }
            else {
                log.error("  Cannot set new IK reference, error code=0x%llx", err);
                return false;
            }
        }
        else {
            log.error("  Fail to plan additional smooth path, error code=0x%llx", err);
            return false;
        }
    }
    else if (last_motion_.smooth_type == SMOOTH_C2L) {
        log.info("  Planning smooth path from MoevC to MoveL");
        if (planning_interface_->MoveC2LAdditionSmooth(m_joint_start, m_pose_start,
                                                       m_pose_start_past, planned_path, err)) {
            if (setLatestIKReference(planned_path.back(), err)) {
                PathPoint pp;
                pp.type = MOTION_JOINT;
                pp.id = (last_motion_.id << 2) + POINT_MIDDLE;

                vector<JointValues>::iterator itr = planned_path.begin();
                for (; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
                log.info("  Smooth path generated successfully with %zd points, and added into planned_path_FIFO.",
                         planned_path.size());
                return true;
            }
            else {
                log.error("  Cannot set new IK reference, error code=0x%llx", err);
                return false;
            }
        }
        else {
            log.error("  Fail to plan additional smooth path, error code=0x%llx", err);
            return false;
        }
    }
    else if (last_motion_.smooth_type == SMOOTH_C2C) {
        log.info("  Planning smooth path from MoevC to MoveC");
        if (planning_interface_->MoveC2CAdditionSmooth(m_joint_start, m_pose_start,
                                                       m_pose_start_past, planned_path, err)) {
            if (setLatestIKReference(planned_path.back(), err)) {
                PathPoint pp;
                pp.type = MOTION_JOINT;
                pp.id = (last_motion_.id << 2) + POINT_MIDDLE;

                vector<JointValues>::iterator itr = planned_path.begin();
                for (; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
                log.info("  Smooth path generated successfully with %zd points,  and added into planned_path_FIFO.",
                         planned_path.size());
                return true;
            }
            else {
                log.error("  Cannot set new IK reference, error code=0x%llx", err);
                return false;
            }
        }
        else {
            log.error("  Fail to plan additional smooth path, error code=0x%llx", err);
            return false;
        }
    }

    err = MOTION_INTERNAL_FAULT;
    log.error("  Unexpected smooth type, error code=0x%llx", err);
    return false;
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
bool ArmGroup::suspendArmMotion(ErrorCode &err) {
    err = SUCCESS;

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
        if ((itr->id & POINT_LEVEL_MASK) == POINT_ENDING) {
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
        bool result = planning_interface_->replanPauseTrajectory(m_suspend_state.replan_trajectory, err);
        if (result == true) {
            JointPoint jp;
            jp.id = m_joint_trajectory_fifo.front().id & ~POINT_LEVEL_MASK | POINT_MIDDLE;
            m_joint_trajectory_fifo.clear();
            for (std::vector<JointValues>::iterator itr = m_suspend_state.replan_trajectory.begin();
                 itr != m_suspend_state.replan_trajectory.end(); ++itr) {
                jp.joints = *itr;
                m_joint_trajectory_fifo.push_back(jp);
            }
            // change the ID of last point in FIFO2, to make an ending point.
            m_joint_trajectory_fifo.back().id &= ~POINT_LEVEL_MASK;
            m_joint_trajectory_fifo.back().id |= POINT_ENDING;
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
bool ArmGroup::resumeArmMotion(ErrorCode &err) {
    err = SUCCESS;

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
            log.info("Resume pattern: origin.");
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
            log.info("Resume pattern: origin, trajectory length: %d", m_joint_trajectory_fifo.size());
            m_suspend_state.is_suspended = false;
            unlockArmGroup();
            log.info("Resume successfully.");
            return true;
        case e_suspend_pattern_replan:
            num = m_planned_path_fifo.size()<100 ? m_planned_path_fifo.size() : 100;
            if (convertPath2Trajectory(num, err) == num) {
                bool last_point_is_ending_point = false;
                m_suspend_state.replan_trajectory.clear();
                for (std::vector<JointPoint>::iterator itr = m_joint_trajectory_fifo.begin();
                     itr != m_joint_trajectory_fifo.end(); ++itr) {
                    m_suspend_state.replan_trajectory.push_back(itr->joints);
                    if ((itr->id & POINT_LEVEL_MASK) == POINT_ENDING) {
                        log.info("The last point is ending point.");
                        last_point_is_ending_point = true;
                    }
                }
                bool result = planning_interface_->replanRestartTrajectory(m_suspend_state.replan_trajectory,
                                                                           m_suspend_state.last_point,
                                                                           err);
                if (result == true) {
                    JointPoint jp;
                    jp.id = m_joint_trajectory_fifo.front().id;
                    jp.id = jp.id & ~POINT_LEVEL_MASK | POINT_MIDDLE;
                    m_joint_trajectory_fifo.clear();
                    for (std::vector<JointValues>::iterator itr = m_suspend_state.replan_trajectory.begin();
                            itr != m_suspend_state.replan_trajectory.end(); ++itr) {
                        jp.joints = *itr;
                        m_joint_trajectory_fifo.push_back(jp);
                    }
                    log.info("Resume pattern: replan, trajectory length: %d", m_joint_trajectory_fifo.size());
                    if (last_point_is_ending_point && !m_joint_trajectory_fifo.empty()) {
                        m_joint_trajectory_fifo.back().id &= ~POINT_LEVEL_MASK;
                        m_joint_trajectory_fifo.back().id |= POINT_ENDING;
                        log.info("Turning the last point to ending point.");
                    }
                }
                else {
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


bool ArmGroup::isArmGroupSuspended(void) {
    return m_suspend_state.is_suspended;
}


// ------------------------------------private function---------------------------------------------

inline void ArmGroup::printJointValues(const JointValues &joint) {
    log.info("%lf, %lf, %lf, %lf, %lf, %lf",
             joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6);
}

inline void ArmGroup::printJointValues(const char *str, const JointValues &joint) {
    log.info("%s%lf, %lf, %lf, %lf, %lf, %lf", str,
             joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6);
}

inline void ArmGroup::printJointLimit(const JointLimit &joint_limit) {
    log.info("lower=%lf, home=%lf, upper=%lf, max_omega=%lf, max_alpha=%lf",
             joint_limit.lower, joint_limit.home, joint_limit.upper,
             joint_limit.max_omega, joint_limit.max_alpha);
}

inline void ArmGroup::printJointLimit(const char *str, const JointLimit &joint_limit) {
    log.info("%slower=%lf, home=%lf, upper=%lf, max_omega=%lf, max_alpha=%lf", str,
             joint_limit.lower, joint_limit.home, joint_limit.upper,
             joint_limit.max_omega, joint_limit.max_alpha);
}

inline void ArmGroup::printJointConstraints(const JointConstraints &constraint) {
    log.info("Joint constraints:");
    printJointLimit("  J1 limit: ", constraint.j1);
    printJointLimit("  J2 limit: ", constraint.j2);
    printJointLimit("  J3 limit: ", constraint.j3);
    printJointLimit("  J4 limit: ", constraint.j4);
    printJointLimit("  J5 limit: ", constraint.j5);
    printJointLimit("  J6 limit: ", constraint.j6);
}

inline void ArmGroup::printPose(const Pose &pose) {
    log.info("position=%lf,%lf,%lf orientation=%lf,%lf,%lf,%lf",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

inline void ArmGroup::printPose(const char *str, const Pose &pose) {
    log.info("%sposition=%lf,%lf,%lf orientation=%lf,%lf,%lf,%lf", str,
             pose.position.x, pose.position.y, pose.position.z,
             pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

inline void ArmGroup::printPoseEuler(const PoseEuler &pose_euler) {
    log.info("position=%lf,%lf,%lf orientation=%lf,%lf,%lf",
            pose_euler.position.x, pose_euler.position.y, pose_euler.position.z,
            pose_euler.orientation.a, pose_euler.orientation.b, pose_euler.orientation.c);
}

inline void ArmGroup::printPoseEuler(const char *str, const PoseEuler &pose_euler) {
    log.info("%sposition=%lf,%lf,%lf orientation=%lf,%lf,%lf", str,
            pose_euler.position.x, pose_euler.position.y, pose_euler.position.z,
            pose_euler.orientation.a, pose_euler.orientation.b, pose_euler.orientation.c);
}

inline void ArmGroup::printJointPoint(const JointPoint &point) {
    log.info("Joint point: ID=%d", point.id);
    log.info("  joints=%lf, %lf, %lf, %lf, %lf, %lf",
             point.joints.j1, point.joints.j2, point.joints.j3, 
             point.joints.j4, point.joints.j5, point.joints.j6);
    log.info("  omegas=%lf, %lf, %lf, %lf, %lf, %lf",
             point.omegas.j1, point.omegas.j2, point.omegas.j3,
             point.omegas.j4, point.omegas.j5, point.omegas.j6);
}

inline void ArmGroup::printJointPoint(const char *str, const JointPoint &point) {
    log.info(str);
    printJointPoint(point);
}

inline void ArmGroup::printPathPoint(const PathPoint &point) {
    switch (point.type) {
        case MOTION_JOINT:
            log.info("Path point: ID=%d, type=Joint", point.id);
            printJointValues("  joints=", point.joints);
            break;

        case MOTION_LINE:
            log.info("Path point: ID=%d, type=Line", point.id);
            printPose("  pose: ", point.pose);
            break;

        case MOTION_CIRCLE:
            log.info("Path point: ID=%d, type=Circle", point.id);
            printPose("  pose: ", point.pose);
            break;

        default:
            log.error("Type error: undefined point type.");
    }
}

inline void ArmGroup::printPathPoint(const char *str, const PathPoint &point) {
    log.info(str);
    printPathPoint(point);
}


void ArmGroup::lockArmGroup(void) {
    //log.info("Try to lock arm group.");
    pthread_mutex_lock(&m_group_mutex);
}

void ArmGroup::unlockArmGroup(void) {
    pthread_mutex_unlock(&m_group_mutex);
    //log.info("Arm group unlocked.");
}


bool ArmGroup::setArmGroupParameters(XmlRpc::XmlRpcValue &params) {
    try {
        enable_calibration_ = params["enable_calibration"];
        m_JOINT_FIFO_LEN    = params["joint_fifo_length_max"];
    }
    catch (XmlRpc::XmlRpcException &exception) {
        log.error("Exception:");
        log.error(exception.getMessage().c_str());
        return false;
    }

    return true;
}

//------------------------------------------------------------------------------
// Function:    checkJointBoundary
// Summary: To check whether a group of joint values are valid according to
//          joint constraints.
// In:      joint_values -> joint_values needed to be checked
// Out:     None
// Return:  true  -> valid
//------------------------------------------------------------------------------
bool ArmGroup::checkJointBoundary(const JointValues &joint_values) {
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
const JointValues& ArmGroup::getLatestIKReference(void) {
    return m_latest_ik_reference;
}


}
