/**********************************************************************
    Copyright:  Foresight-Robotics
    File:       fst_controller.cpp
    Author:     Feng Yun
    Data:       Aug.1  2016
    Modify:     Aug.30 2016
    Description:ArmGroup--Source code.
**********************************************************************/

#include <motion_controller_version.h>
#include <motion_controller/motion_controller_arm_group.h>
#include <struct_to_mem/struct_feedback_joint_states.h>
#include <boost/filesystem.hpp>
#include <fstream>

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
ArmGroup::ArmGroup(void)
{
    log.info("ArmGroup Version %d.%d.%d",   motion_controller_VERSION_MAJOR,
                                            motion_controller_VERSION_MINOR,
                                            motion_controller_VERSION_PATCH);

    current_state_      = UNDEFINED;
    planning_interface_ = NULL;
    calibrator_         = NULL;

    // memset((void*)&tool_frame_,    0, sizeof(tool_frame_));
    // memset((void*)&user_frame_,    0, sizeof(user_frame_));
    // memset((void*)&joint_constraint_,   0, sizeof(joint_constraint_));
    memset((void*)&latest_ik_reference_,0, sizeof(latest_ik_reference_));

    memset((void*)&temp_zero_offset_,   0,  sizeof(TempZeroOffset));
    memset((void*)&temp_constraint_,    0,  sizeof(TempConstraint));
    memset((void*)&hard_constraint_,    0,  sizeof(JointConstraint));
    memset((void*)&suspend_state_,      0,  sizeof(SuspendState));

    planned_path_fifo_.resize(0);
    planned_path_fifo_.reserve(2000);
    trajectory_fifo_.resize(0);
    trajectory_fifo_.reserve(2000);

    trajectory_fifo_dynamic_length_ = 0;

    robot_parameter_path_ = "config/";
    enable_calibration_ = false;
    suspend_state_.is_suspended    = false;
    temp_zero_offset_.is_using_temp_zero_offset = false;
    temp_constraint_.is_using_temp_constraint   = false;

    /*
    cout << "Test in ArmGroup constructor:" << endl;
    double* c = new double;
    cout << "construct a double" << endl;
    *c = 5.5;
    delete c;
    cout << "delete a double" << endl;
    double* a = new double[100];
    cout << "construct a array" << endl;
    a[1] = 1; a[2] = 2; a[3] = 3;
    delete[] a;
    cout << "delete a array" << endl;
    MotionTarget t1,t2;
    TrajPlan *plan = new TrajPlan();
    MoveCommand *p = new MoveCommand(plan, t1, t2, 15);
    if (p) {
        cout << "construct MoveCommand Success" << endl;
        delete p;
        cout << "delete success" << endl;
    }*/
}


//------------------------------------------------------------------------------
// Function:    ~ArmGroup
// Summary: The destructor of class
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
ArmGroup::~ArmGroup(void)
{
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
bool ArmGroup::initArmGroup(vector<ErrorCode> &err)
{
    ErrorCode error;
    err.clear();
    if (current_state_ != UNDEFINED) {err.push_back(MOTION_FAIL_IN_INIT); return false;}

    log.info("Initializing ArmGroup...");
    current_state_ = INITIALIZED;
    pthread_mutex_init(&group_mutex_, NULL);

    bool    enable_logger = false;
    log.info("Loading ArmGroup parameters ...");
    ParamGroup arm_param("share/motion_controller/config/motion_controller.yaml");
    if (arm_param.getLastError() == SUCCESS) {
        bool result = true;
        result = result && arm_param.getParam("arm_group/enable_logger", enable_logger);
        result = result && arm_param.getParam("arm_group/enable_calibration", enable_calibration_);
        result = result && arm_param.getParam("arm_group/default_trajectory_fifo_length", trajectory_fifo_dynamic_length_);
        result = result && arm_param.getParam("arm_group/robot_parameter_path", robot_parameter_path_);
        if (result == true) {
            log.info("  initial FIFO2 size to %d", trajectory_fifo_dynamic_length_);
            enable_logger       ? log.info("  log service: enabled")
                                : log.info("  log service: disabled");
            enable_calibration_ ? log.info("  zero offset calibration: enabled")
                                : log.info("  zero offset calibration: disabled");
            log.info("  robot parameter path: %s", robot_parameter_path_.c_str());
            log.info("Success!");
        }
        else {
            err.push_back(arm_param.getLastError());
            log.error("Cannot set ArmGroup basic parameters, error code=0x%llx", err.back());
            current_state_ = UNDEFINED;
            return false;
        }
    }
    else {
        err.push_back(arm_param.getLastError());
        log.error("Fail to load ArmGroup parameter file, error code=0x%llx", err.back());
        current_state_ = UNDEFINED;
        return false;
    }

    if (enable_logger) {
        log.info("Register ArmGroup-logger into log server ...");
        if (log.initLogger("ArmGroup")) {
            log.info("Success!");
        }
        else {
            err.push_back(MOTION_FAIL_IN_INIT);
            log.error("Cannot register ArmGroup-logger into log server, error code=0x%llx", err.back());
            current_state_ = UNDEFINED;
            return false;
        }
    }

    ParamValue param;
    if (arm_param.getParam("planning_interface", param)) {
        error = constructPlanningInterface(param);
        if (error != SUCCESS) {
            log.error("Error code=0x%llx", error);
            current_state_ = UNDEFINED;
            err.push_back(error);
            return false;
        }
    }
    else {
        err.push_back(arm_param.getLastError());
        log.error("Fail loading planning interface parameters, error code=0x%llx", err.back());
        current_state_ = UNDEFINED;
        return false;
    }

    error = loadJointConstraint(robot_parameter_path_);
    if (error != SUCCESS) {
        log.error("Error code=0x%llx", error);
        current_state_ = UNDEFINED;
        err.push_back(error);
        return false;
    }

    error = loadDHParameter(robot_parameter_path_);
    if (error != SUCCESS) {
        log.error("Error code=0x%llx", error);
        current_state_ = UNDEFINED;
        err.push_back(error);
        return false;
    }
    
    error = constructCalibrator(robot_parameter_path_);
    if (error != SUCCESS) {
        log.error("Failed to construct calibrator, error code=0x%llx", error);
        current_state_ = UNDEFINED;
        err.push_back(error);
        return false;
    }

    unsigned int calibrate_result;
    if (checkZeroOffset(calibrate_result, error)) {
        // Set start state using current joint
        resetArmGroup(error);
        if (error != SUCCESS) {
            err.push_back(error);
            if (error != JOINT_OUT_OF_CONSTRAINT) {
                log.error("Fail in initArmGroup(). Error code=0x%llx", err.back());
                current_state_ = UNDEFINED;
                return false;
            }
            else {
                log.error("Current joint is out of constraint. Error code=0x%llx", err.back());
            }
        }
    }
    else {
        // zero offset lost, need to set a temp zero offset
        // or calibration fault
        err.push_back(error);
        if (error == ZERO_OFFSET_DEVIATE || error == ZERO_OFFSET_LOST) {
            log.warn("One or more joint lost its zero offset, need a temporary zero offset and calibration.");
            log.warn("Error code=0x%llx", error);
        }
        else {
            log.error("Calibration fault, error code=0x%llx", error);
            current_state_ = UNDEFINED;
            return false;
        }
    }

    if (err.empty()) {
        err.push_back(SUCCESS);
        log.info("ArmGroup is ready to make life easier. Have a good time!");
        log.info("**********************************************************************************************");
        return true;
    }
    else {
        log.warn("ArmGroup initialized with errors:");
        for (int i = 0; i < err.size(); ++i) {
            log.warn("  Error Code %d: 0x%llx", i, err[i]);
        }
        log.info("**********************************************************************************************");
        return false;
    }
}

bool ArmGroup::recordLastJoint(ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED)  {err = NEED_INITIALIZATION; return false;}
    
    if (enable_calibration_) {
        log.info("Calibrator is recording current joint ...");
        if (calibrator_->recordCurrentJoint()) {
            log.info("Done!");
            return true;
        }
        else {
            err = calibrator_->getLastError();
            log.error("Fail to record last joint, error code=0x%llx", err);
            return false;
        }
    
    }
    else {
        return true;
    }
}

bool ArmGroup::setTempZeroOffset(ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED)  {err = NEED_INITIALIZATION; return false;}
    
    log.info("Calibrator is setting temp zero offset ...");
    if (calibrator_->setTempZeroOffset()) {
        log.info("Done!");
        log.info("Reviewing the new offset ...");
        unsigned int bitmap;
        if (checkZeroOffset(bitmap, err)) {
            log.info("Success!");
            return true;
        }
        else {
            log.error("It seems that the temp offset doesn't work.");
            return false;
        }
    }
    else {
        err = calibrator_->getLastError();
        log.error("Fail to set temp zero offset, error code=0x%llx", err);
        return false;
    }
}

bool ArmGroup::calibrateZeroOffset(unsigned int &calibrate_result, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED)      {err = NEED_INITIALIZATION;         return false;}
    if (!enable_calibration_) return true;
    
    log.info("Calibrating new zero offset ...");
    if (calibrator_->setZeroOffset()) {
        log.info("Done!");
        log.info("Reviewing the new offset ...");
        unsigned int bitmap;
        if (checkZeroOffset(bitmap, err)) {
            log.info("Success!");
            return true;
        }
        else {
            log.error("It seems that the new offset doesn't work.");
            return false;
        }
    }
    else {
        err = calibrator_->getLastError();
        return false;
    }
    /*
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
    }*/
}

bool ArmGroup::checkZeroOffset(unsigned int &calibrate_result, ErrorCode &err)
{
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    if (enable_calibration_) {
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
        err = SUCCESS;
        // err = CALIBRATION_FAULT;
        // log.error("Fail, calibration is disabled, error code=0x%llx", err);
    }
    
    /*
    log.info("Using current joint values to initialize ArmGroup:");
    FeedbackJointState fbjs;
    calibrator_->getCurrentJoint(fbjs);
    Joint joint;
    memcpy(&joint, fbjs.position, 6 * sizeof(double));
    printJoint("  joints=", joint);
    setStartState(joint, err);
    */

    if (err == SUCCESS) return true;
    else                return false;
}


//------------------------------------------------------------------------------
// Function:    getCycleTime
// Summary: To get cycle time of interpolation algorithm.
// In:      None
// Out:     None
// Return:  cycle time
//------------------------------------------------------------------------------
double ArmGroup::getCycleTime(void)
{
    if (current_state_ != INITIALIZED)  return 0.0;
    return planning_interface_->getCycleTime();
}


//------------------------------------------------------------------------------
// Function:    getJointConstraint
// Summary: To get joint constraint from Kinematics algorithm.
// In:      None
// Out:     None
// Return:  joint constraint
//------------------------------------------------------------------------------
const JointConstraint& ArmGroup::getSoftConstraint(void)
{
    if (current_state_ != INITIALIZED)  {
        return hard_constraint_;
    }
    return planning_interface_->getJointConstraint();
}

const JointConstraint& ArmGroup::getHardConstraint(void)
{
    return hard_constraint_;
}

Transformation ArmGroup::getUserFrame(void)
{
    if (current_state_ != INITIALIZED)  {
        Transformation frame;
        memset(&frame, 0, sizeof(Transformation));
        return frame;
    }
    return planning_interface_->getUserFrame();
}

Transformation ArmGroup::getToolFrame(void)
{
    if (current_state_ != INITIALIZED)  {
        Transformation frame;
        memset(&frame, 0, sizeof(Transformation));
        return frame;
    }
    return planning_interface_->getToolFrame();
}

/*
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
*/

//------------------------------------------------------------------------------
// Function:    getMaxAcceleration
// Summary: To get max acceleration settings.
// In:      None
// Out:     None
// Return:  value of max acceleration
//------------------------------------------------------------------------------
double ArmGroup::getMaxAcceleration(void)
{
    if (current_state_ != INITIALIZED)  return 0.0;
    return planning_interface_->getAcceleration();
}


//------------------------------------------------------------------------------
// Function:    getVelocityScalingFactor
// Summary: To get velocity scaling factor value.
// In:      None
// Out:     None
// Return:  global scaling factor for velocity
//------------------------------------------------------------------------------
double ArmGroup::getVelocityScalingFactor(void)
{
    if (current_state_ != INITIALIZED)  return 0.0;
    return planning_interface_->getVelocityScaling();
}


//------------------------------------------------------------------------------
// Function:    getAccelerationScalingFactor
// Summary: To get acceleration scaling factor value.
// In:      None
// Out:     None
// Return:  global scaling factor for acceleration
//------------------------------------------------------------------------------
double ArmGroup::getAccelerationScalingFactor(void)
{
    if (current_state_ != INITIALIZED)  return 0.0;
    return planning_interface_->getAccelerationScaling();
}

CurveMode ArmGroup::getCurveMode(void)
{
    if (current_state_ != INITIALIZED)  return T_CURVE;
    return planning_interface_->getCurveMode();
}

const DHGroup ArmGroup::getDH(void)
{
    if (current_state_ != INITIALIZED) {
        DHGroup dh;
        memset(&dh, 0, sizeof(DHGroup));
        return dh;
    }
    else {
        return planning_interface_->getDH();
    }
}

//------------------------------------------------------------------------------
// Function:    getPlannedPathFIFOLength
// Summary: To get the length of planned_path_FIFO.
// In:      None
// Out:     None
// Return:  length of the FIFO
//------------------------------------------------------------------------------
int ArmGroup::getPlannedPathFIFOLength(void)
{
    if (current_state_ != INITIALIZED)  return 0;
    return planned_path_fifo_.size() + planning_interface_->getAllCommandLength();
}


//------------------------------------------------------------------------------
// Function:    getJointTrajectoryFIFOLength
// Summary: To get the length of joitn_trajectory_FIFO.
// In:      None
// Out:     None
// Return:  length of the FIFO
//------------------------------------------------------------------------------
int ArmGroup::getTrajectoryFIFOLength(void)
{
    return trajectory_fifo_.size();
}

int ArmGroup::getTrajectoryFIFOReserveLength(void)
{
    return trajectory_fifo_dynamic_length_;
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
int ArmGroup::getPointsFromJointTrajectoryFIFO(vector<JointOutput> &traj, ErrorCode &err)
{
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
    //if (num < 0) {err = INVALID_PARAMETER; return num - 1;}
    //else if (num == 0) {return 0;}

    lockArmGroup();
    err = SUCCESS;
    if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
        if (!suspend_state_.is_suspended) {
            int cnt = convertPath2Trajectory(20, err);
            log.info("Convert FIFO1->FIFO2 %d points", cnt);
        }
    }

    int cnt;
    int num = 10;
    JointOutput jout;
    std::vector<JointPoint>::iterator itr = trajectory_fifo_.begin();
    traj.clear();

    for (cnt = 0; cnt < num; ++cnt) {
        if (itr != trajectory_fifo_.end()) {
            jout.id     = itr->source->getMotionID();
            jout.joint  = itr->joint;
            jout.level  = itr->level;
            traj.push_back(jout);
            if (itr->level == POINT_ENDING) {
                if (temp_constraint_.is_using_temp_constraint && isJointBackIntoNormalConstraint(jout.joint)) {
                    deleteTempConstraint();
                    log.info("Delete temporary constraint, using normal constraint instead.");
                }
                log.info("Ending point, delete Motion Object: 0x%x", itr->source);
                planning_interface_->deleteMotionCommandBefore(itr->source);
                ++cnt;
                break;
            }
            else if (itr->level == POINT_LAST) {
                if (temp_constraint_.is_using_temp_constraint && isJointBackIntoNormalConstraint(jout.joint)) {
                    deleteTempConstraint();
                    log.info("Delete temporary constraint, using normal constraint instead.");
                }
                log.info("Last point, delete Motion Object: 0x%x", itr->source);
                planning_interface_->deleteMotionCommandBefore(itr->source);
                traj.back().level = POINT_MIDDLE;
            }
            ++itr;
        }
        else {
            log.warn("FIFO2 has burnt out, but there is no ending point.");
            log.warn("It might be a truble.");
            break;
        }
    }

    if (cnt != 0) {
        itr = trajectory_fifo_.begin();
        trajectory_fifo_.erase(itr, itr + cnt);
    }
    log.info("  %d points got from FIFO2", cnt);
    unlockArmGroup();

    return cnt;
}

/*
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
*/

//------------------------------------------------------------------------------
// Function:    setMaxAcceleration
// Summary: To change max acceleration settings.
// In:      max_a   -> desired max acceleration, unit: mm/s*s
// Out:     None
// Return:  true    -> max acceleration changed to given value
//          false   -> max acceleration NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setMaxAcceleration(double a)
{
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
bool ArmGroup::setVelocityScalingFactor(double factor)
{
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
bool ArmGroup::setAccelerationScalingFactor(double factor)
{
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
void ArmGroup::setToolFrame(const Transformation &tool_frame)
{
    if (current_state_ != INITIALIZED)  return;

    // tool_frame_ = tool_frame;
    planning_interface_->setToolFrame(tool_frame);
}


//------------------------------------------------------------------------------
// Function:    setUserFrame
// Summary: To set current user frame.
// In:      user_frame  -> current user frame
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
void ArmGroup::setUserFrame(const Transformation &user_frame)
{
    if (current_state_ != INITIALIZED)  return;

    // user_frame_ = user_frame;
    planning_interface_->setUserFrame(user_frame);
}

//------------------------------------------------------------------------------
// Function:    setCurveMode
// Summary: To set curve mode.
// In:      mode    -> desired curve mode
// Out:     None
// Return:  true    -> desired curve mode changed to given value
//          false   -> desired curve mode NOT changed
//------------------------------------------------------------------------------
void ArmGroup::setCurveMode(CurveMode mode)
{
    if (current_state_ != INITIALIZED)  return;

    planning_interface_->setCurveMode(mode);
    log.info("Set curve mode to %s", mode == T_CURVE ? "T shape" : "S shape");
}

//------------------------------------------------------------------------------
// Function:    setJerk
// Summary: To set the ratio of jerk to acceleration.
// In:      jerk    -> desired ratio
// Out:     None
// Return:  true    -> ratio changed to given value
//          false   -> ratio NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setJerk(double jerk)
{
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
// Function:    setJointConstraint
// Summary: To set joint constraint in Kinematics algorithm.
// In:      constraint -> joint constraint
// Out:     None
// Return:  true    -> set successfully
//          false   -> set UNsuccessfully
//------------------------------------------------------------------------------
bool ArmGroup::setSoftConstraint(const JointConstraint &constraint)
{
    if (current_state_ != INITIALIZED)  return false;

    printJointConstraint("New soft constraint:", constraint);

    try {
        string kinematic_file = robot_parameter_path_ + "kinematics_constraints.yaml";
        ParamValue value;
        ParamGroup group(kinematic_file);
        group.getParam("soft_constraint", value);
        if (group.getLastError() != SUCCESS) {
            log.error("Error while loading kinematic_constraint.yaml");
            return false;
        }
        
        value["j1"]["home"]     = constraint.j1.home;
        value["j1"]["upper"]    = constraint.j1.upper;
        value["j1"]["lower"]    = constraint.j1.lower;
        value["j1"]["omega_max"]= constraint.j1.max_omega;
        value["j1"]["alpha_max"]= constraint.j1.max_alpha;
        value["j2"]["home"]     = constraint.j2.home;
        value["j2"]["upper"]    = constraint.j2.upper;
        value["j2"]["lower"]    = constraint.j2.lower;
        value["j2"]["omega_max"]= constraint.j2.max_omega;
        value["j2"]["alpha_max"]= constraint.j2.max_alpha;
        value["j3"]["home"]     = constraint.j3.home;
        value["j3"]["upper"]    = constraint.j3.upper;
        value["j3"]["lower"]    = constraint.j3.lower;
        value["j3"]["omega_max"]= constraint.j3.max_omega;
        value["j3"]["alpha_max"]= constraint.j3.max_alpha;
        value["j4"]["home"]     = constraint.j4.home;
        value["j4"]["upper"]    = constraint.j4.upper;
        value["j4"]["lower"]    = constraint.j4.lower;
        value["j4"]["omega_max"]= constraint.j4.max_omega;
        value["j4"]["alpha_max"]= constraint.j4.max_alpha;
        value["j5"]["home"]     = constraint.j5.home;
        value["j5"]["upper"]    = constraint.j5.upper;
        value["j5"]["lower"]    = constraint.j5.lower;
        value["j5"]["omega_max"]= constraint.j5.max_omega;
        value["j5"]["alpha_max"]= constraint.j5.max_alpha;
        value["j6"]["home"]     = constraint.j6.home;
        value["j6"]["upper"]    = constraint.j6.upper;
        value["j6"]["lower"]    = constraint.j6.lower;
        value["j6"]["omega_max"]= constraint.j6.max_omega;
        value["j6"]["alpha_max"]= constraint.j6.max_alpha;
        
        if (!group.setParam("soft_constraint", value) || !group.dumpParamFile()) {
            log.error("Error while dumping kinematic_constraint.yaml");
            return false;
        }
    }
    catch (fst_parameter::ParamException &exception) {
        log.error("Set soft constraint exception:");
        log.error(exception.getMessage().c_str());
        return false;
    }

    if (temp_constraint_.is_using_temp_constraint) {
        temp_constraint_.normal_constraint = constraint;
        log.info("ArmGroup is using temp constraint, the new soft constaint will be in force later");
    }
    else {
        if (setSoftConstraintImpl(constraint)) {
            log.info("Success");
            return true;
        }
        else {
            log.error("Invalid soft constraint");
            return false;
        }
    }
}

/*
//------------------------------------------------------------------------------
// Function:    setCurrentJoint
// Summary: To set current joint values using encoder data.
//          Current pose values will be updated automaticly.
// In:      current_joint   -> joint values from the encoder
// Out:     error_code  -> error code
// Return:  true    -> current joint/pose values updated successfully
//          false   -> either joint or pose values NOT updated
//------------------------------------------------------------------------------
bool ArmGroup::setCurrentJoint(const Joint &current_joint, ErrorCode &err) {
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    if (checkJointBoundary(current_joint)) {
        current_joint_ = current_joint;
        return computeFK(current_joint_, current_pose_, err);
    }
    else {
        err = JOINT_OUT_OF_CONSTRAINT;
        log.error("setCurrentJoint() get a joint group out of boundary");
        return false;
    }
}*/

//------------------------------------------------------------------------------
// Function:    setStartState
// Summary: To set robot start state.
// In:      joint_start -> robot start state
// Out:     error_code  -> error code
// Return:  true    -> robot start state setted to joint_start scucessfully
//          false   -> failed to set robot start state
//------------------------------------------------------------------------------
bool ArmGroup::setStartState(const Joint &joint_start, ErrorCode &err)
{
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
    err = SUCCESS;

    if (checkJointBoundary(joint_start)) {
        lockArmGroup();
        planning_interface_->setInitialJoint(joint_start);
        latest_ik_reference_ = joint_start;
        unlockArmGroup();
        return true;
        /*
        if (setStartStateImpl(joint_start, err)) {
            setLatestIKReference(joint_start);

            unlockArmGroup();
            return true;
        }
        else {
            log.error("Fail to set start joint values. Error code=0x%llx", err);
            printJoint("  start joint=", joint_start);
            unlockArmGroup();
            return false;
        }
        */
    }
    else {
        err = JOINT_OUT_OF_CONSTRAINT;
        return false;
    }
}

bool ArmGroup::clearArmGroup(ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
        
    lockArmGroup();
    log.info("Clear ArmGroup ... ");
    planning_interface_->clearMotionList();
    planned_path_fifo_.clear();
    trajectory_fifo_.clear();
    suspend_state_.is_suspended = false;
    log.info("Success!");
    unlockArmGroup();

    return true;
}

bool ArmGroup::resetArmGroup(ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
        
    log.info("Reset ArmGroup ... ");
    if (calibrator_->getCurrentState() < CALIBRATED && !calibrator_->isUsingTempZeroOffset()) {
        log.error("INVALID operation: need temp zero offset");
        err = NEED_CALIBRATION;
        return false;
    }

    
    lockArmGroup();
    err = setCurrentJointToStartJoint();
    unlockArmGroup();
    return true;      // joint is out of mechanical limit?
}

/*
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

    Joint start_joint;
    lockArmGroup();
    if (trajectory_fifo_.size() > 0) {
        start_joint = trajectory_fifo_.back().joint;
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
*/

//------------------------------------------------------------------------------
// Function:    getJointFromPose
// Summary: To compute IK with a given pose in cartesian space.
// In:      poes    -> cartesian space pose needed to compute IK
// Out:     joint_result-> IK result
//          error_code  -> error code
// Return:  true    -> IK solution found
//          false   -> IK solution NOT found
//------------------------------------------------------------------------------
bool ArmGroup::getJointFromPose(const Pose &pose, Joint &joint_result,
                                double time_interval, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
    return planning_interface_->getJointFromPose(pose, latest_ik_reference_, joint_result, time_interval, err);
}

bool ArmGroup::getPoseFromJoint(const Joint &joint, Pose &pose, ErrorCode &err)
{
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
bool ArmGroup::computeIK(const Pose &pose, Joint &joint_result, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
    return planning_interface_->computeInverseKinematics(pose, latest_ik_reference_, joint_result, err);
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
bool ArmGroup::computeFK(const Joint &joint, Pose &pose, ErrorCode &err)
{
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
Pose ArmGroup::transformPoseEuler2Pose(const PoseEuler &pose_e)
{
    return planning_interface_->transformPoseEuler2Pose(pose_e);
}

//------------------------------------------------------------
// Function:    transformPose2PoseEuler
// Summary: To transform a pose point to a poseEuler point.
// In:      poes    -> the pose to be transformed
// Out:     None
// Return:  poseEuler point
//------------------------------------------------------------
PoseEuler ArmGroup::transformPose2PoseEuler(const Pose &pose)
{
    return planning_interface_->transformPose2PoseEuler(pose);
}

/*
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
    

    int result = 0;
    lockArmGroup();
    if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
        num     = trajectory_fifo_dynamic_length_ - trajectory_fifo_.size();
        result  = convertPath2Trajectory(num, err);
    }
    unlockArmGroup();

    return result;
}
*/

bool ArmGroup::suspendArmMotion(ErrorCode &err)
{
    if (current_state_ != INITIALIZED)  {err = NEED_INITIALIZATION; return false;}
    if (trajectory_fifo_.size() == 0)   {err = INVALID_SEQUENCE;    return false;}
    
    lockArmGroup();
    err = SUCCESS;
    log.info("Suspend request accepted.");
    vector<JointPoint>::iterator jp_it = trajectory_fifo_.begin();
    for (; jp_it != trajectory_fifo_.end(); ++jp_it) {
        if (jp_it->level == POINT_ENDING) {
            break;
        }
    }
    if (jp_it != trajectory_fifo_.end()) {
        log.info("An ending point found in FIFO2, prepareing slow-down trajectory ...");
        suspend_state_.is_suspended = true;
        suspend_state_.type = SUSPEND_BY_TRAJECTORY;
        suspend_state_.last_point = *jp_it;
        trajectory_fifo_.erase(jp_it + 1, trajectory_fifo_.end());
        planned_path_fifo_.clear();
        // set pick start position?
        planning_interface_->setPickPosition(suspend_state_.last_point);
    }
    else {
        log.info("Replanning slow-down trajectory in FIFO2 ...");
        suspend_state_.is_suspended = true;
        suspend_state_.type = SUSPEND_BY_REPLAN;
        // replan use all points in FIFO2?
        planning_interface_->replanPauseTrajectory(trajectory_fifo_, suspend_state_.last_point);
        suspend_state_.pause_joint = trajectory_fifo_.back().joint;
        // set pick start position?
        planned_path_fifo_.clear();
        planning_interface_->setPickPosition(suspend_state_.last_point);
    }

    log.info("Slow-down trajectory ready, we will stop the robot in %zu points", trajectory_fifo_.size());
    unlockArmGroup();
    return true;
}

void ArmGroup::declareEstop(void)
{
    if (current_state_ != INITIALIZED)  return;

    lockArmGroup();
    if (trajectory_fifo_.size() > 0) {
        suspend_state_.is_suspended = true;
        suspend_state_.type = SUSPEND_BY_ESTOP;
        suspend_state_.last_point = trajectory_fifo_.front();
        planning_interface_->setPickPosition(suspend_state_.last_point);
    }
    else {
        // TODO
        // How to record last_joint when trajectory_fifo_ is empty?
    }

    planned_path_fifo_.clear();
    trajectory_fifo_.clear();
    unlockArmGroup();
}

bool ArmGroup::resumeArmMotion(ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED)  {err = NEED_INITIALIZATION; return false;}
    if (suspend_state_.is_suspended == false) {err = INVALID_SEQUENCE; return false;}
    if (trajectory_fifo_.size() != 0) {err = INVALID_SEQUENCE; return false;}

    lockArmGroup();
    log.info("Resume request accepted.");
    if (suspend_state_.type == SUSPEND_BY_ESTOP) {
        log.info("  resume from Estop");
        planning_interface_->resumeFromEstop(suspend_state_.last_point, trajectory_fifo_, err);
    }
    else if (suspend_state_.type == SUSPEND_BY_TRAJECTORY) {
        log.info("  resume from pause, type = Trajectory");
    }
    else {
        log.info("  resume from pause, type = Replan");
        planning_interface_->resumeFromPause(suspend_state_.pause_joint,
                                             suspend_state_.last_point,
                                             trajectory_fifo_,
                                             err);
    }

    if (err == SUCCESS) {
        suspend_state_.is_suspended = false;
        log.info("Resume trajectory ready.");
        unlockArmGroup();
        return true;
    }
    else {
        log.error("Fail to construct resume trajectory, error code=0x%llx", err);
        unlockArmGroup();
        return false;
    }
}

/*
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
        suspend_state_.last_point = trajectory_fifo_.back().joint;
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
        // std::vector<Joint> stop_trajectory;
        // stop_trajectory.reserve(400);
        suspend_state_.replan_trajectory.clear();
        for (std::vector<JointPoint>::iterator itr = trajectory_fifo_.begin();
                itr != trajectory_fifo_.end(); ++itr) {
             suspend_state_.replan_trajectory.push_back(itr->joint);
        }
        bool result = planning_interface_->replanPauseTrajectory(suspend_state_.replan_trajectory, err);
        if (result == true) {
            JointPoint jp;
            jp.id = trajectory_fifo_.front().id & ~POINT_LEVEL_MASK | POINT_MIDDLE;
            trajectory_fifo_.clear();
            for (std::vector<Joint>::iterator itr = suspend_state_.replan_trajectory.begin();
                 itr != suspend_state_.replan_trajectory.end(); ++itr) {
                jp.joint = *itr;
                trajectory_fifo_.push_back(jp);
            }
            // change the ID of last point in FIFO2, to make an ending point.
            trajectory_fifo_.back().id &= ~POINT_LEVEL_MASK;
            trajectory_fifo_.back().id |= POINT_ENDING;
            // refresh the suspend_state_ to record that we are suspended.
            suspend_state_.pattern = e_suspend_pattern_replan;
            suspend_state_.last_point = trajectory_fifo_.back().joint;
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
*/

bool ArmGroup::isArmGroupSuspended(void) {
    return suspend_state_.is_suspended;
}


}
