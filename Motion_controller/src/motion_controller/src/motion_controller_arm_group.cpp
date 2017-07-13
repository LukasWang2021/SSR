/**********************************************************************
    Copyright:  Foresight-Robotics
    File:       fst_controller.cpp
    Author:     Feng Yun
    Data:       Aug.1  2016
    Modify:     Aug.30 2016
    Description:ArmGroup--Source code.
**********************************************************************/


#include <motion_controller/motion_controller_arm_group.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <struct_to_mem/struct_feedback_joint_states.h>
#include <boost/filesystem.hpp>
#include <fstream>
// #include <ros/ros.h>
// #include <assert.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using fst_parameter::ParamGroup;
using fst_parameter::ParamValue;

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
    suspend_state_.is_suspended    = false;
    
    last_motion_.id = 0;
    last_motion_.smooth_type = SMOOTH_NONE;
    last_motion_.motion_type = MOTION_UNDEFINED;

    trajectory_fifo_dynamic_length_ = 0;
    m_vu_start      = 0.0;
    m_v_start       = 0.0;

    memset((void*)&tool_frame_,    0, sizeof(tool_frame_));
    memset((void*)&user_frame_,    0, sizeof(user_frame_));
    memset((void*)&m_pose_start,    0, sizeof(m_pose_start));
    memset((void*)&m_pose_previous, 0, sizeof(m_pose_previous));
    memset((void*)&m_joint_start,   0, sizeof(m_joint_start));
    memset((void*)&m_pose_start_past,     0, sizeof(m_pose_start));
    memset((void*)&joint_constraints_,   0, sizeof(joint_constraints_));
    memset((void*)&current_joint_,  0, sizeof(current_joint_));
    memset((void*)&current_pose_,   0, sizeof(current_pose_));
    memset((void*)&latest_ik_reference_,    0, sizeof(latest_ik_reference_));

    planned_path_fifo_.resize(0);
    planned_path_fifo_.reserve(10000);
    trajectory_fifo_.resize(0);
    trajectory_fifo_.reserve(1000);

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


    pthread_mutex_destroy(&group_mutex_);
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
    if (current_state_ != UNDEFINED) {err = MOTION_FAIL_IN_INIT; return false;}

    log.info("Initializing ArmGroup...");
    current_state_ = INITIALIZED;
    pthread_mutex_init(&group_mutex_, NULL);
    planned_path_fifo_.reserve(5000);
    trajectory_fifo_.reserve(500);

    suspend_state_.is_suspended = false;
    // suspend_state_.pattern = e_suspend_pattern_origin;
    // suspend_state_.last_point = joint_values;
    suspend_state_.fifo2_cache.reserve(500);
    suspend_state_.replan_trajectory.reserve(1000);

    bool    enable_logger = false;
    string  robot_parameter_path;
    log.info("Loading ArmGroup parameters ...");
    fst_parameter::ParamGroup arm_param("share/motion_controller/config/motion_controller.yaml");
    if (arm_param.getLastError() == SUCCESS) {
        bool result = arm_param.getParam("arm_group/enable_logger", enable_logger);
        result = result && arm_param.getParam("arm_group/enable_calibration", enable_calibration_);
        result = result && arm_param.getParam("arm_group/trajectory_fifo_length", trajectory_fifo_dynamic_length_);
        result = result && arm_param.getParam("arm_group/robot_parameter_path", robot_parameter_path);
        if (result == true) {
            log.info("  initial FIFO2 size to %d", trajectory_fifo_dynamic_length_);
            enable_logger       ? log.info("  log service: enabled")
                                : log.info("  log service: disabled");
            enable_calibration_ ? log.info("  zero offset calibration: enabled")
                                : log.info("  zero offset calibration: disabled");
            log.info("  robot parameter path: %s", robot_parameter_path.c_str());
            log.info("Success!");
        }
        else {
            err = arm_param.getLastError();
            log.error("Cannot set ArmGroup parameters, error code=0x%llx", err);
            current_state_ = UNDEFINED;
            return false;
        }
    }
    else {
        err = arm_param.getLastError();
        log.error("Fail loading ArmGroup parameters, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }

    if (enable_logger) {
        log.info("Constructing ArmGroup logger ...");
        if (!log.initLogger("ArmGroup")) {
            err = MOTION_FAIL_IN_INIT;
            log.error("Cannot construct ArmGroup log file, error code=0x%llx", err);
            // current_state_ = UNDEFINED;
            // return false;
        }
        else {
            log.info("Success!");
        }
    }

    log.info("Constructing planning interface ...");
    if (planning_interface_ != NULL) {
        delete planning_interface_;
        planning_interface_ = NULL;
    }
    planning_interface_ = new fst_controller::PlanningInterface();

    if (planning_interface_ == NULL) {
        err = MOTION_FAIL_IN_INIT;
        log.error("Cannot construct planning interface, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    else {
        ParamValue params;
        /*
        if (!ros::param::get("/fst_param/motion_controller/planning_interface", params)) {
            err = FAIL_LOADING_PARAMETER;
            log.error("Fail loading parameters from remote server, error code=0x%llx", err);
            current_state_ = UNDEFINED;
            return false;
        }*/
        if (!arm_param.getParam("planning_interface", params)) {
            err = arm_param.getLastError();
            log.error("Fail loading planning interface parameters, error code=0x%llx", err);
            current_state_ = UNDEFINED;
            return false;
        }
        if (planning_interface_->initPlanningInterface(params, err)) {
            log.info("  algorithm version: %s", planning_interface_->getAlgorithmVersion().c_str());
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
            // err = MOTION_FAIL_IN_INIT;
            log.error("Fail to initialize planning_interface");
            current_state_ = UNDEFINED;
            return false;
        }
    }

    log.info("Loading joint constraints from robot parameter path ...");
    string constraints_file = robot_parameter_path + "kinematics_constraints.yaml";
    fst_parameter::ParamGroup kinematic_constraint(constraints_file.c_str());
    ParamValue kinematics_limit;
    if (kinematic_constraint.getLastError() == SUCCESS && 
        kinematic_constraint.getParam("kinematics_limit", kinematics_limit)) {
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
        catch (fst_parameter::ParamException &exception) {
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
        err = kinematic_constraint.getLastError();
        log.error("Fail loading kinematics constraints, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    
    log.info("Loading DH parameters from robot parameter path ...");
    string dh_file = robot_parameter_path + "dh.yaml";
    fst_parameter::ParamGroup dh_params(dh_file.c_str());
    ParamValue dh_param;
    if (dh_params.getLastError() == SUCCESS && 
        dh_params.getParam("DH_parameter", dh_param)) {
        DHGroup dh;
        try {
            dh.j1.alpha = dh_param["j1"]["alpha"];
            dh.j1.a     = dh_param["j1"]["a"];
            dh.j1.d     = dh_param["j1"]["d"];
            dh.j1.theta = dh_param["j1"]["theta"];
            dh.j2.alpha = dh_param["j2"]["alpha"];
            dh.j2.a     = dh_param["j2"]["a"];
            dh.j2.d     = dh_param["j2"]["d"];
            dh.j2.theta = dh_param["j2"]["theta"];
            dh.j3.alpha = dh_param["j3"]["alpha"];
            dh.j3.a     = dh_param["j3"]["a"];
            dh.j3.d     = dh_param["j3"]["d"];
            dh.j3.theta = dh_param["j3"]["theta"];
            dh.j4.alpha = dh_param["j4"]["alpha"];
            dh.j4.a     = dh_param["j4"]["a"];
            dh.j4.d     = dh_param["j4"]["d"];
            dh.j4.theta = dh_param["j4"]["theta"];
            dh.j5.alpha = dh_param["j5"]["alpha"];
            dh.j5.a     = dh_param["j5"]["a"];
            dh.j5.d     = dh_param["j5"]["d"];
            dh.j5.theta = dh_param["j5"]["theta"];
            dh.j6.alpha = dh_param["j6"]["alpha"];
            dh.j6.a     = dh_param["j6"]["a"];
            dh.j6.d     = dh_param["j6"]["d"];
            dh.j6.theta = dh_param["j6"]["theta"];
        }
        catch (fst_parameter::ParamException &exception) {
            log.error("Exception:");
            log.error(exception.getMessage().c_str());
            err = MOTION_FAIL_IN_INIT;
            current_state_ = UNDEFINED;
            return false;
        }

        if(planning_interface_->setDH(dh)) {
            log.info("Success!");
        }
        else {
            err = MOTION_FAIL_IN_INIT;
            current_state_ = UNDEFINED;
            return false;
        }
    }
    else {
        err = dh_params.getLastError();
        log.error("Fail loading DH parameters, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    
    log.info("Constructing calibrator ...");
    if (calibrator_ != NULL) {
        delete calibrator_;
        calibrator_ = NULL;
    }

    calibrator_ = new fst_controller::Calibrator(log);
    if (calibrator_ == NULL) {
        err = MOTION_FAIL_IN_INIT;
        log.error("Cannot construct calibrator, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }

    bool res = calibrator_->initCalibrator(robot_parameter_path);
    if (!res) {
        err = calibrator_->getLastError();
        log.error("Cannot initialize calibrator, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    else {
        log.info("Success!");
    }
    
    log.info("Downloading JTAC parameters ...");
    if (!calibrator_->transmitJtacParam("all")) {
        err = calibrator_->getLastError();
        log.error("Download parameter failed, error code=0x%llx", err);
        current_state_ = UNDEFINED;
        return false;
    }
    else {
        log.info("Success!");
        usleep(256 * 1000);
    }

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
                if (calibrator_->setTempZeroOffset()) {
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
        err = calibrator_->getLastError();
        if (calibrator_->setTempZeroOffset()) {
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
            err = calibrator_->getLastError();
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

bool ArmGroup::recordLastJoint(ErrorCode &err) {
    err = SUCCESS;
    if (current_state_ != INITIALIZED)  {err = NEED_INITIALIZATION; return false;}
    if (enable_calibration_ != true)    {err = CALIBRATION_FAULT; return false;}

    if (calibrator_->getCurrentState() == CALIBRATED) {
        log.info("Calibrator is recording current joint ...");
        if (calibrator_->recordLastJoint()) {log.info("Done!"); return true;}
        else {
            err = calibrator_->getLastError();
            log.error("Fail to record last joint, error code=0x%llx", err);
            return false;
        }
    }
    else {err = CALIBRATION_FAULT; return false;}
}

bool ArmGroup::checkZeroOffset(unsigned int &calibrate_result, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    if (enable_calibration_ == true) {
        log.info("Reviewing current joint and joint in recorder ...");
        unsigned int result;
        if (calibrator_->reviewCurrentJoint(result)) {
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

    if (err == SUCCESS) return true;
    else return false;
}

bool ArmGroup::calibrateZeroOffset(unsigned int &calibrate_result, ErrorCode &err) {
    err = SUCCESS;
    if (current_state_ != INITIALIZED)      {err = NEED_INITIALIZATION;         return false;}
    if (enable_calibration_ != true)        {err = CALIBRATION_FAULT;           return false;}
    if (!calibrator_->recordZeroOffset())   {err = calibrator_->getLastError(); return false;}

    log.info("Downloading zero offset parameters ...");
    if (!calibrator_->transmitJtacParam("zero_offset")) {
        err = calibrator_->getLastError();
        log.error("Download parameter failed, error code=0x%llx", err);
        // current_state_ = UNDEFINED;
        return false;
    }
    log.info("Success!");
    usleep(256 * 1000);

    // check zero offset again
    log.info("Reviewing starting joint ...");
    unsigned int result;
    if (calibrator_->reviewCalibratedJoint(result)) {
        calibrate_result = result;
        if (result == OFFSET_NORMAL) {
            log.info("Success!");
            return true;
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
    return current_joint_;
}


//------------------------------------------------------------------------------
// Function:    getCurrentPose
// Summary: To get current pose of endpoint in the arm group.
// In:      None
// Out:     None
// Return:  current pose of the endpoint
//------------------------------------------------------------------------------
const Pose& ArmGroup::getCurrentPose(void) {
    return current_pose_;
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
    return planned_path_fifo_.size() + planning_interface_->getFIFOLength();
}


//------------------------------------------------------------------------------
// Function:    getJointTrajectoryFIFOLength
// Summary: To get the length of joitn_trajectory_FIFO.
// In:      None
// Out:     None
// Return:  length of the FIFO
//------------------------------------------------------------------------------
int ArmGroup::getJointTrajectoryFIFOLength(void) {
    return trajectory_fifo_.size();
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
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
    if (num < 0) { err = INVALID_PARAMETER; return num - 1;}
    else if (num == 0) {return 0;}

    lockArmGroup();
    int cnt;
    std::vector<JointPoint>::iterator itr = trajectory_fifo_.begin();

    for (cnt = 0; cnt < num; ++cnt) {
        if (itr != trajectory_fifo_.end()) {
            traj.push_back(*itr);
            if ((itr->id & POINT_LEVEL_MASK) == POINT_ENDING) {
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
        itr = trajectory_fifo_.begin();
        trajectory_fifo_.erase(itr, itr + cnt);
    }

    unlockArmGroup();
    return cnt;
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
    if (current_state_ != INITIALIZED)  return false;

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
    if (current_state_ != INITIALIZED)  return false;

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
    if (current_state_ != INITIALIZED)  return false;

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
    if (current_state_ != INITIALIZED)  return false;

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
// Function:    setToolFrame
// Summary: To set current tool frame.
// In:      tool_frame  -> current tool frame
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
void ArmGroup::setToolFrame(const Transformation &tool_frame) {
    if (current_state_ != INITIALIZED)  return;

    tool_frame_ = tool_frame;
    planning_interface_->setToolFrame(tool_frame_);
}


//------------------------------------------------------------------------------
// Function:    setUserFrame
// Summary: To set current user frame.
// In:      user_frame  -> current user frame
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
void ArmGroup::setUserFrame(const Transformation &user_frame) {
    if (current_state_ != INITIALIZED)  return;

    user_frame_ = user_frame;
    planning_interface_->setUserFrame(user_frame_);
}

//------------------------------------------------------------------------------
// Function:    setCurveMode
// Summary: To set curve mode.
// In:      mode    -> desired curve mode
// Out:     None
// Return:  true    -> desired curve mode changed to given value
//          false   -> desired curve mode NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setCurveMode(int mode) {
    if (current_state_ != INITIALIZED)  return false;

    if (planning_interface_->setCurveMode(mode)) {
        log.info("Set curve mode to %d", mode);
        return true;
    }
    else {
        log.error("Cannot set curve mode to %d", mode);
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
    if (current_state_ != INITIALIZED)  return false;

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
// Function:    setCurrentJointValues
// Summary: To set current joint values using encoder data.
//          Current pose values will be updated automaticly.
// In:      current_joint   -> joint values from the encoder
// Out:     error_code  -> error code
// Return:  true    -> current joint/pose values updated successfully
//          false   -> either joint or pose values NOT updated
//------------------------------------------------------------------------------
bool ArmGroup::setCurrentJointValues(const JointValues &current_joint, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    if (checkJointBoundary(current_joint)) {
        current_joint_ = current_joint;
        return computeFK(current_joint_, current_pose_, err);
    }
    else {
        err = JOINT_OUT_OF_CONSTRAINT;
        log.error("setCurrentJOintValues() get a joint group out of boundary");
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

    if (setStartStateImpl(joint_start, err)) {
        allowed_motion_type_ = MOTION_UNDEFINED;
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
    bool result = true;
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    JointValues start_joint;
    lockArmGroup();
    if (trajectory_fifo_.size() > 0) {
        start_joint = trajectory_fifo_.back().joints;
        trajectory_fifo_.back().id &= ~POINT_LEVEL_MASK;
        trajectory_fifo_.back().id |= POINT_ENDING;
        unlockArmGroup();
        result = setStartState(start_joint, err);
    }
    else {
        start_joint = getLatestIKReference();
        unlockArmGroup();
        result = setStartState(getLatestIKReference(), err);
        if (suspend_state_.is_suspended = false)
            log.warn("Empty FIFO2, cannot add ending point.");

    }

    lockArmGroup();
    planned_path_fifo_.clear();
    planning_interface_->clearFIFO();
    suspend_state_.is_suspended = false;
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

    if (trajectory_fifo_.size() > 0) {
      result = setStartState(trajectory_fifo_.front().joints, err);
    }
    else if (planned_path_fifo_.size() > 0){
        if (planned_path_fifo_.front().type == enum_Type_MoveJ) {
            result = setStartState(planned_path_fifo_.front().joints, err);
        }
        else if (planned_path_fifo_.front().type == enum_Type_MoveL ||
                 planned_path_fifo_.front().type == enum_Type_MoveC) {
            JointValues joints;
            if (computeInverseKinematics(planned_path_fifo_.front().pose,
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
        trajectory_fifo_.clear();
        planned_path_fifo_.clear();
        ROS_INFO("Joint trajectory FIFO and planned path FIFO are both cleared.");
    }
    else {
        trajectory_fifo_.clear();
        planned_path_fifo_.clear();
        ROS_ERROR("Cannot set start state. But joint trajectory FIFO and planned path FIFO are both cleared!");
        ROS_ERROR("WARNNING: Wrong start state, need to be setted manually before next planning");
    }

    return result;
}
*/

bool ArmGroup::resetArmGroup(const JointValues &joint, ErrorCode &err) {
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    log.info("Reset ArmGroup ... ");
    printJointValues("Using new joint values to reset ArmGroup: joints=",joint);
    lockArmGroup();
    if (setCurrentJointValues(joint, err)) {
        m_pose_start = getCurrentPose();
        m_pose_previous = m_pose_start;
        m_vu_start = 0.0;
        m_v_start = 0.0;
        latest_ik_reference_ = joint;
        m_joint_start.joints = joint;
        m_joint_start.omegas.j1 = 0;
        m_joint_start.omegas.j2 = 0;
        m_joint_start.omegas.j3 = 0;
        m_joint_start.omegas.j4 = 0;
        m_joint_start.omegas.j5 = 0;
        m_joint_start.omegas.j6 = 0;

        allowed_motion_type_ = MOTION_UNDEFINED;
        last_motion_.smooth_type = SMOOTH_NONE;
        suspend_state_.is_suspended = false;

        planned_path_fifo_.clear();
        trajectory_fifo_.clear();
        planning_interface_->clearFIFO();

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
// Function:    getJointFromPose
// Summary: To compute IK with a given pose in cartesian space.
// In:      poes    -> cartesian space pose needed to compute IK
// Out:     joint_result-> IK result
//          error_code  -> error code
// Return:  true    -> IK solution found
//          false   -> IK solution NOT found
//------------------------------------------------------------------------------
bool ArmGroup::getJointFromPose(const Pose &pose, JointValues &joint_result,
                                double time_interval, ErrorCode &err) {
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
    return planning_interface_->getJointFromPose(pose, getLatestIKReference(), joint_result, time_interval, err);
}

bool ArmGroup::getPoseFromJoint(const JointValues &joint, Pose &pose, ErrorCode &err) {
    return computeFK(joint, pose, err);
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
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
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
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
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
//          into jointpoint in trajectory_fifo_.
// In:      num        -> number of pose that needed to be converted
// Out:     error_code -> error code
// Return:  <0         -> ERROR occurred during converting
//          >=0        -> number of pose that convered actually
//------------------------------------------------------------------------------
int ArmGroup::convertPathToTrajectory(int num, ErrorCode &err) {
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return (num > 0) ? 0 : (num - 1);}
    if (num < 0) {err = INVALID_PARAMETER; return num - 1;}
    else if (num == 0) return 0;
    if (suspend_state_.is_suspended) {err = INVALID_SEQUENCE; return 0;}
    
    lockArmGroup();
    int result = convertPath2Trajectory(num, err);
    unlockArmGroup();

    return result;
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
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("Suspend request accepted.");
    if (trajectory_fifo_.size() == 0) {
        err = NO_ENOUGH_POINTS_FIFO2;
        log.warn("FIFO2 is empty, cannot replan a slow-down trajectory. Are we at a standstill?");
        unlockArmGroup();
        return false;
    }

    std::vector<JointPoint>::iterator itr_ending_point = trajectory_fifo_.end();
    for (std::vector<JointPoint>::iterator itr = trajectory_fifo_.begin(); 
         itr != trajectory_fifo_.end(); ++itr) {
        if ((itr->id & POINT_LEVEL_MASK) == POINT_ENDING) {
            itr_ending_point = itr;
            break;
        }
    }

    if (itr_ending_point != trajectory_fifo_.end()) {
        // There is an ending point in FIFO2, we will stop following the original plan.
        log.info("An ending point found in FIFO2, prepareing slow-down trajectory...");
        // Move points behind the ending point from FIFO2 to cache.
        suspend_state_.fifo2_cache.clear();
        suspend_state_.fifo2_cache.insert(suspend_state_.fifo2_cache.begin(),
                                          ++itr_ending_point,
                                          trajectory_fifo_.end());
        trajectory_fifo_.erase(itr_ending_point, itr_ending_point + suspend_state_.fifo2_cache.size());
        
        // refresh the suspend_state_ to record that we are suspended. 
        suspend_state_.pattern = e_suspend_pattern_origin;
        suspend_state_.last_point = trajectory_fifo_.back().joints;
        suspend_state_.replan_trajectory.clear();
        suspend_state_.is_suspended = true;
        
        log.info("Slow-down trajectory ready, we will stop the arm group in %zu points",
                 trajectory_fifo_.size());
        
        unlockArmGroup();
        return true;
    } // if (itr_ending_point != trajectory_fifo_.end()) {
    else {
        // None ending point found in FIFO2, we need to replan a slow-down trajectory in FIFO2
        log.info("Replanning slow-down trajectory in FIFO2...");
        if (trajectory_fifo_.size() < 10) {
            err = NO_ENOUGH_POINTS_FIFO2;
            log.error("Too few points in FIFO2, we cannot replan a slow-down trajectory");
            unlockArmGroup();
            return false;
        }
        // std::vector<JointValues> stop_trajectory;
        // stop_trajectory.reserve(400);
        suspend_state_.replan_trajectory.clear();
        for (std::vector<JointPoint>::iterator itr = trajectory_fifo_.begin();
                itr != trajectory_fifo_.end(); ++itr) {
             suspend_state_.replan_trajectory.push_back(itr->joints);
        }
        bool result = planning_interface_->replanPauseTrajectory(suspend_state_.replan_trajectory, err);
        if (result == true) {
            JointPoint jp;
            jp.id = trajectory_fifo_.front().id & ~POINT_LEVEL_MASK | POINT_MIDDLE;
            trajectory_fifo_.clear();
            for (std::vector<JointValues>::iterator itr = suspend_state_.replan_trajectory.begin();
                 itr != suspend_state_.replan_trajectory.end(); ++itr) {
                jp.joints = *itr;
                trajectory_fifo_.push_back(jp);
            }
            // change the ID of last point in FIFO2, to make an ending point.
            trajectory_fifo_.back().id &= ~POINT_LEVEL_MASK;
            trajectory_fifo_.back().id |= POINT_ENDING;
            // refresh the suspend_state_ to record that we are suspended.
            suspend_state_.pattern = e_suspend_pattern_replan;
            suspend_state_.last_point = trajectory_fifo_.back().joints;
            suspend_state_.fifo2_cache.clear();
            suspend_state_.replan_trajectory.clear();
            suspend_state_.is_suspended = true;

            log.info("Slow-down trajectory ready, we will stop the arm group in %zu points",
                     trajectory_fifo_.size());
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
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("Resume request accepted.");
    if (!suspend_state_.is_suspended) {
        err = INVALID_SEQUENCE;
        log.error("Arm group has not been suspended.");
        unlockArmGroup();
        return false;
    }

    if (getJointTrajectoryFIFOLength() != 0) {
        err = INVALID_SEQUENCE;
        log.error("FIFO2 is not empty, resume is not allowed now.");
        unlockArmGroup();
        return false;
    }

    bool result;
    switch (suspend_state_.pattern) {
        case e_suspend_pattern_origin:
            result = resumeByOrigin(err);
            unlockArmGroup();
            return result;
        case e_suspend_pattern_replan:
            result = resumeByReplan(err);
            unlockArmGroup();
            return result;
        default:
            err = MOTION_INTERNAL_FAULT;
            log.error("Program internal fault, error code=0x%llx", err);
            unlockArmGroup();
            return false;
    } // switch (suspend_state_.pattern) {
}

bool ArmGroup::isArmGroupSuspended(void) {
    return suspend_state_.is_suspended;
}



}
