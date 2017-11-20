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
    log.info("ArmGroup Version %d.%d.%d",   
                                            motion_controller_VERSION_MAJOR,
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
    fifo1_backup_ptr_    = 0;
    fifo1_backup_enable_ = false;

    trajectory_fifo_dynamic_length_ = 0;

    robot_recorder_path_ = "/opt/fst_controller/runtime/";
    enable_calibration_ = false;
    allow_convert_ = true;
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
    pthread_mutex_destroy(&fifo1_mutex_);
    pthread_mutex_destroy(&fifo2_mutex_);
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
bool ArmGroup::initArmGroup(vector<ErrorCode> &err_list)
{
    vector<ErrorCode> err;
    ErrorCode error;
    if (current_state_ != UNDEFINED) {err.push_back(MOTION_FAIL_IN_INIT); return false;}

    log.info("Initializing ArmGroup...");
    current_state_ = INITIALIZED;
    pthread_mutex_init(&group_mutex_, NULL);
    pthread_mutex_init(&fifo1_mutex_, NULL);
    pthread_mutex_init(&fifo2_mutex_, NULL);
    pthread_mutex_init(&fifo1_backup_mutex_, NULL);

    bool    enable_logger = false;
    log.info("Loading ArmGroup parameters ...");
    ParamGroup arm_param("share/configuration/configurable/motion_controller.yaml");
    if (arm_param.getLastError() == SUCCESS) {
        bool    result = true;
        int     size;

        result = result && arm_param.getParam("arm_group/enable_logger", enable_logger);
        result = result && arm_param.getParam("arm_group/enable_calibration", enable_calibration_);
        result = result && arm_param.getParam("arm_group/default_trajectory_fifo_length", trajectory_fifo_dynamic_length_);
        result = result && arm_param.getParam("arm_group/robot_recorder_path", robot_recorder_path_);
        result = result && arm_param.getParam("arm_group/trajectory_fifo_length", size);

        
        if (result == true) {
            log.info("  initial FIFO2 size to %d", trajectory_fifo_dynamic_length_);

            if (size > 0) {
                traj_fifo_.initLockFreeFIFO(size);
                
                if (traj_fifo_.capacity() == size) {
                    log.info("  initial FIFO2 (lokc free FIFO) size to %d", size);
                }
                else {
                    log.error("FIFO2 (lock free FIFO) size not as expected, actual=%d, expect=%d", traj_fifo_.capacity(), size);
                    err.push_back(MOTION_FAIL_IN_INIT);
                    current_state_ = UNDEFINED;
                    return false;
                }
            }
            else {
                log.error("Configuration value of FIFO2 size invalid, size=%d", size);
                err.push_back(INVALID_PARAMETER);
                current_state_ = UNDEFINED;
                return false;
            }
            
            enable_logger       ? log.info("  log service: enabled")
                                : log.info("  log service: disabled");

            enable_calibration_ ? log.info("  zero offset calibration: enabled")
                                : log.info("  zero offset calibration: disabled");
            
            log.info("  robot recorder path: %s", robot_recorder_path_.c_str());
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
    //log.setDisplayLevel(MSG_LEVEL_NONE);

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

    error = loadJointConstraint();
    if (error != SUCCESS) {
        log.error("Error code=0x%llx", error);
        current_state_ = UNDEFINED;
        err.push_back(error);
        return false;
    }

    error = loadDHParameter();
    if (error != SUCCESS) {
        log.error("Error code=0x%llx", error);
        current_state_ = UNDEFINED;
        err.push_back(error);
        return false;
    }
    
    error = constructCalibrator(robot_recorder_path_);
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
            if (error != CURRENT_JOINT_OUT_OF_CONSTRAINT) {
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
        log.info("ArmGroup is ready to make life easier. Have a good time!");
        log.info("**********************************************************************************************");
        return true;
    }
    else {
        log.warn("ArmGroup initialized with errors:");
        for (int i = 0; i < err.size(); ++i) {
            err_list.push_back(err[i]);
            log.warn("  Error Code %d: 0x%llx", i, err[i]);
        }
        log.info("**********************************************************************************************");
        return false;
    }
}

bool ArmGroup::startManualTeach(ManualFrameMode frame, ManualMotionMode mode)
{
    if (current_state_ != INITIALIZED)  {return false;}

    if (trajectory_fifo_.empty() && planned_path_fifo_.empty()) {
        if (setCurrentJointToStartJoint() == SUCCESS) {
            planning_interface_->setManualFrameMode(frame);
            planning_interface_->setManualMotionMode(mode);
            return planning_interface_->startManualTeach();
        }
    }
    else {
        return false;
    }
}

bool ArmGroup::stopManualTeach(void)
{
    return planning_interface_->stopManualTeach();
}

void ArmGroup::setManualStepLength(double step)
{
    planning_interface_->setManualStepLength(step);
}

void ArmGroup::stepManualTeach(vector<int> &button)
{
    if (planning_interface_->isManualTeaching()) {
        vector<PathPoint> traj;
        planning_interface_->stepManualTeach(button, traj);
        planned_path_fifo_.insert(planned_path_fifo_.end(), traj.begin(), traj.end());
    }
}

void ArmGroup::startRecordingFIFO1(void)
{
    lockFIFO1Backup();
    if (fifo1_backup_enable_ == false) {
        fifo1_backup_enable_ = true;
        fifo1_backup_ptr_    = 0;
    }
    unlockFIFO1Backup();
}

void ArmGroup::stopRecordingFIFO1(void)
{
    lockFIFO1Backup();
    if (fifo1_backup_enable_ == true) {
        fifo1_backup_enable_ = false;
    }
    unlockFIFO1Backup();
}

void ArmGroup::getFIFO1RecordingData(vector<PoseEuler> &data)
{
    data.clear();

    lockFIFO1Backup();
    if (fifo1_backup_ptr_ > 0) {
        data.resize(fifo1_backup_ptr_);
        
        for (size_t i = 0; i < fifo1_backup_ptr_; ++i)
            data[i] = fifo1_backup_[i];
        
        fifo1_backup_ptr_ = 0;
    }
    unlockFIFO1Backup();
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
    if (calibrator_->isUsingTempZeroOffset()) {
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
    }
    else {
        log.error("Cannot calibrate new offset, set temp offset first.");
        err = INVALID_SEQUENCE;
        return false;
    }
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
    else if (temp_constraint_.is_using_temp_constraint) {
        return temp_constraint_.normal_constraint;
    }
    else {
        return planning_interface_->getJointConstraint();
    }
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
ErrorCode ArmGroup::getPointsFromJointTrajectoryFIFO(vector<JointOutput> &traj)
{
    if (current_state_ != INITIALIZED) {return NEED_INITIALIZATION;}

    int num = 20;
    ErrorCode err = SUCCESS;

    //lockArmGroup();
    lockFIFO1();
    lockFIFO2();
    if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
        if (!suspend_state_.is_suspended) {
            if (allow_convert_) {
                err = convertPath2Trajectory(num);
                // log.info("Convert FIFO1->FIFO2 %d points", cnt);
                if (err != SUCCESS) {
                    allow_convert_ = false;
                    unlockFIFO2();
                    unlockFIFO1();
                    return err;
                }
            }
        }
    }
    unlockFIFO1();

    num = 10;
    err = getPointsFromJointTrajectoryFIFOImpl(traj, num);
    //planning_interface_->displayMotionList();
    unlockFIFO2();
    //unlockArmGroup();

    return err;
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
        log.info("Set velocity scaling factor to %.2f%%", factor);
        return true;
    }
    else {
        log.error("Cannot set velocity scaling factor to %.2f%%", factor);
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
ErrorCode ArmGroup::setSoftConstraint(const JointConstraint &constraint)
{
    if (current_state_ != INITIALIZED)  return NEED_INITIALIZATION;

    log.info("Set soft constraint");

    if (temp_constraint_.is_using_temp_constraint) {
        JointConstraint temp = planning_interface_->getJointConstraint();
        if (isFirstConstraintCoveredBySecond(temp, constraint)) {
            ErrorCode err = setSoftConstraintImpl(constraint);
            if (err == SUCCESS) {
                temp_constraint_.is_using_temp_constraint = false;
                temp_constraint_.normal_constraint = constraint;
                log.info("Success");
                printJointConstraint("New soft constraint:", constraint);
                return SUCCESS;
            }
            else {
                log.error("Fail to set soft constraint, error code=0x%llx", err);
                return err;
            }
        }
        else {
            temp_constraint_.normal_constraint = constraint;
            log.info("ArmGroup is using temp constraint, the new soft constaint will be in force later");
        }
    }
    else {
        ErrorCode err = setSoftConstraintImpl(constraint);
        if (err == SUCCESS) {
            log.info("Success");
            printJointConstraint("New soft constraint:", constraint);
            return SUCCESS;
        }
        else {
            log.error("Fail to set soft constraint, error code=0x%llx", err);
            return err;
        }
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
        
    log.info("Clear ArmGroup ... ");
    lockFIFO1();
    planned_path_fifo_.clear();
    unlockFIFO1();
    lockFIFO2();
    trajectory_fifo_.clear();
    unlockFIFO2();
    lockArmGroup();
    planning_interface_->clearMotionList();
    suspend_state_.is_suspended = false;
    allow_convert_ = true;
    unlockArmGroup();
    log.info("Success!");

    return true;
}

bool ArmGroup::resetArmGroup(ErrorCode &err)
{
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
        
    err = SUCCESS;
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

bool ArmGroup::isJointFallInConstraint(const Joint &joint, const JointConstraint &cons)
{
    return  isJointInConstraint(joint, cons);
}

bool ArmGroup::isTrajectoryTotallyFinished(void)
{
    return planning_interface_->isMotionCommandListEmpty();
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
bool ArmGroup::getJointFromPose(const PoseEuler &pose, Joint &joint_result,
                                double time_interval, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
    return planning_interface_->getJointFromPose(pose, latest_ik_reference_, joint_result, time_interval, err);
}

bool ArmGroup::getPoseFromJoint(const Joint &joint, PoseEuler &pose, ErrorCode &err)
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
bool ArmGroup::computeIK(const PoseEuler &pose, Joint &joint_result, ErrorCode &err)
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
bool ArmGroup::computeFK(const Joint &joint, PoseEuler &pose, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
    return planning_interface_->computeForwardKinematics(joint, pose, err);
}

bool ArmGroup::computeFKInWorldCoordinate(const Joint &joint, PoseEuler &flange, PoseEuler &tcp, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}
    return planning_interface_->computeForwardKinematicsWorld(joint, flange, tcp, err);
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

//------------------------------------------------------------------------------
// Function:    convertPathToTrajectory
// Summary: To convert numbers of posepoint in m_cartesian_path_FIFO
//          into jointpoint in trajectory_fifo_.
// In:      num        -> number of pose that needed to be converted
// Out:     error_code -> error code
// Return:  <0         -> ERROR occurred during converting
//          >=0        -> number of pose that convered actually
//------------------------------------------------------------------------------
ErrorCode ArmGroup::convertPathToTrajectory(int num)
{
    if (current_state_ != INITIALIZED) {return NEED_INITIALIZATION;}
    if (num < 0) {return INVALID_PARAMETER;}
    else if (num == 0) return SUCCESS;
    if (suspend_state_.is_suspended) {return INVALID_SEQUENCE;}
    

    ErrorCode result = SUCCESS;
    lockFIFO1();
    lockFIFO2();
    if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
        num     = trajectory_fifo_dynamic_length_ - trajectory_fifo_.size();
        result  = convertPath2Trajectory(num);
    }
    unlockFIFO2();
    unlockFIFO1();

    return result;
}

bool ArmGroup::suspendArmMotion(ErrorCode &err)
{
    if (current_state_ != INITIALIZED)  {err = NEED_INITIALIZATION; return false;}
    
    err = SUCCESS;
    log.info("Suspend request accepted.");
    if (suspend_state_.is_suspended == true) {
        log.warn("already in pause state");
        err = INVALID_SEQUENCE;
        return false;
    }

    lockFIFO2();
    log.info("  FIFO2 length = %d", trajectory_fifo_.size());
    if (trajectory_fifo_.size() == 0) {
        suspend_state_.is_suspended = true;
        suspend_state_.fake_suspend = true;
        suspend_state_.type = SUSPEND_BY_TRAJECTORY;
        log.info("  fake pause executed.");
        unlockFIFO2();
        return true;
    }
    
    //lockArmGroup();
    vector<JointPoint>::iterator jp_it = trajectory_fifo_.begin();
    for (; jp_it != trajectory_fifo_.end(); ++jp_it) {
        if (jp_it->level == POINT_ENDING) {
            break;
        }
    }
    if (jp_it != trajectory_fifo_.end()) {
        log.info("An ending point found in FIFO2, prepareing slow-down trajectory ...");
        suspend_state_.is_suspended = true;
        suspend_state_.fake_suspend = false;
        suspend_state_.type = SUSPEND_BY_TRAJECTORY;
        suspend_state_.last_point = *jp_it;
        trajectory_fifo_.erase(jp_it + 1, trajectory_fifo_.end());
        // set pick start position?
        planning_interface_->setPickPosition(suspend_state_.last_point);
    }
    else {
        log.info("Replanning slow-down trajectory in FIFO2 ...");
        
        while (trajectory_fifo_.size() > 0 && trajectory_fifo_.back().level == POINT_LAST) {
            // if last point of fifo2 is a POINT-LAST, the command object will be deleted before resumeArmMotion
            // which will cause a invalid pointer used by resume
            trajectory_fifo_.pop_back();
            log.info("last point of FIFO2 is last point of a command, it has been dropped.");
        }

        suspend_state_.is_suspended = true;
        suspend_state_.fake_suspend = false;
        suspend_state_.type = SUSPEND_BY_REPLAN;
        // replan use all points in FIFO2?
        planning_interface_->replanPauseTrajectory(trajectory_fifo_, suspend_state_.last_point);
        log.warn("trajectory_fifo_ size=%d", trajectory_fifo_.size());
        suspend_state_.pause_joint = trajectory_fifo_.back().joint;
        // set pick start position?
        planning_interface_->setPickPosition(suspend_state_.last_point);
    }
    log.info("Slow-down trajectory ready, we will stop the robot in %zu points", trajectory_fifo_.size());
    unlockFIFO2();
    
    lockFIFO1();
    planned_path_fifo_.clear();
    unlockFIFO1();
    //unlockArmGroup();
    return true;
}

void ArmGroup::declareEstop(void)
{
    if (current_state_ != INITIALIZED)  return;

    log.info("declare ESTOP");
    if (suspend_state_.is_suspended && suspend_state_.type == SUSPEND_BY_ESTOP) {
        log.warn("already in ESTOP state");
        return;
    }

    lockFIFO2();
    if (trajectory_fifo_.size() > 0) {
        //cout << "&Obj=" << trajectory_fifo_.front().source << ", id=" << trajectory_fifo_.front().source->getMotionID() << endl;
        suspend_state_.is_suspended = true;
        suspend_state_.fake_suspend = false;
        suspend_state_.type = SUSPEND_BY_ESTOP;
        suspend_state_.last_point = trajectory_fifo_.front();
        planning_interface_->setPickPosition(suspend_state_.last_point);
    }
    else {
        // How to record last_joint when trajectory_fifo_ is empty?
        suspend_state_.is_suspended = true;
        suspend_state_.fake_suspend = true;
        suspend_state_.type = SUSPEND_BY_ESTOP;
        log.warn("A fake ESTOP is decalred, because FIFO2 is empty. Are we standing still?");
    }
    trajectory_fifo_.clear();
    unlockFIFO2();

    lockFIFO1();
    planned_path_fifo_.clear();
    unlockFIFO1();
    
    return;
}

bool ArmGroup::resumeArmMotion(ErrorCode &err)
{
    if (current_state_ != INITIALIZED)  {err = NEED_INITIALIZATION; return false;}
    
    err = SUCCESS;
    log.info("Resume request accepted.");
    
    if (suspend_state_.is_suspended == false) {
        err = INVALID_SEQUENCE;
        log.error("  not in suspend or estop state.");
        return false;
    }

    if (trajectory_fifo_.size() != 0) {
        err = INVALID_SEQUENCE;
        log.info("  fifo2 is not empty, maybe caused by pause trajectory is executing.");
        return false;
    }

    if (suspend_state_.fake_suspend == true) {
        suspend_state_.is_suspended = false;
        suspend_state_.fake_suspend = false;
        if (getPlannedPathFIFOLength() > 0) {
            lockFIFO1();
            lockFIFO2();
            err = convertPath2Trajectory(20);
            unlockFIFO2();
            unlockFIFO1();
        }
        if (err != SUCCESS) return false;
        log.info("  fake suspend detected, resume request return with success");
        return true;
    }

    lockArmGroup();
    lockFIFO1();
    lockFIFO2();
    if (suspend_state_.type == SUSPEND_BY_ESTOP) {
        log.info("  resume from Estop");
        planning_interface_->resumeFromEstop(suspend_state_.last_point, trajectory_fifo_, err);
    }
    else if (suspend_state_.type == SUSPEND_BY_TRAJECTORY) {
        log.info("  fifo1 size: %d", planned_path_fifo_.size());
        if (getPlannedPathFIFOLength() > 0) {
            err = convertPath2Trajectory(20);
        }
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
        log.info("Resume trajectory with %d points ready.", trajectory_fifo_.size());
        if (trajectory_fifo_.size() > 0) {
            latest_ik_reference_ = trajectory_fifo_.back().joint;
            printJoint("  new reference:", latest_ik_reference_);
        }
        else {
            log.warn("  resumeFromPause got 0 points, picking from next command");
            err = convertPath2Trajectory(20);
            if (err != SUCCESS) {
                log.error("  fail in convertPath2Trajectory, code=0x%llx", err);
                unlockFIFO2();
                unlockFIFO1();
                unlockArmGroup();
                return false;
            }
        }

        if (planned_path_fifo_.size() < planning_interface_->getTrajectorySegmentLength()) {
            err = pickPoints();
            if (err != SUCCESS) {
                unlockFIFO2();
                unlockFIFO1();
                unlockArmGroup();
                return false;
            }
            else {
                printPathPoint("first point after resume:", planned_path_fifo_.front());
            }
        }
    }
    else {
        log.error("Fail to construct resume trajectory, error code=0x%llx", err);
    }

    unlockFIFO2();
    unlockFIFO1();
    unlockArmGroup();

    if (err == SUCCESS)  return true;
    else                 return false;
}


bool ArmGroup::isArmGroupSuspended(void) {
    return suspend_state_.is_suspended;
}


}
