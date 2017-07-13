/*************************************************************************
	> File Name: motion_controller_arm_group_private.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年03月02日 星期四 15时48分17秒
 ************************************************************************/

#include <motion_controller/motion_controller_arm_group.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using fst_parameter::ParamValue;

namespace fst_controller {

static const unsigned int UNDEFINED     = 0x5555;
static const unsigned int INITIALIZED   = 0x5556;


ErrorCode ArmGroup::loadJointConstraint(const string &path)
{
    string constraint_file = path + "kinematics_constraints.yaml";
    log.info("Loading hard and soft constraint from '%s' ...", constraint_file.c_str());
    fst_parameter::ParamGroup kinematic_constraint(constraint_file.c_str());
    ParamValue kinematics_limit;
    if (kinematic_constraint.getLastError() == SUCCESS && 
        kinematic_constraint.getParam("hard_constraint", kinematics_limit))
    {
        try {
            hard_constraint_.j1.home     = kinematics_limit["j1"]["home"];
            hard_constraint_.j1.upper    = kinematics_limit["j1"]["upper"];
            hard_constraint_.j1.lower    = kinematics_limit["j1"]["lower"];
            hard_constraint_.j1.max_omega= kinematics_limit["j1"]["omega_max"];
            hard_constraint_.j1.max_alpha= kinematics_limit["j1"]["alpha_max"];
            hard_constraint_.j2.home     = kinematics_limit["j2"]["home"];
            hard_constraint_.j2.upper    = kinematics_limit["j2"]["upper"];
            hard_constraint_.j2.lower    = kinematics_limit["j2"]["lower"];
            hard_constraint_.j2.max_omega= kinematics_limit["j2"]["omega_max"];
            hard_constraint_.j2.max_alpha= kinematics_limit["j2"]["alpha_max"];
            hard_constraint_.j3.home     = kinematics_limit["j3"]["home"];
            hard_constraint_.j3.upper    = kinematics_limit["j3"]["upper"];
            hard_constraint_.j3.lower    = kinematics_limit["j3"]["lower"];
            hard_constraint_.j3.max_omega= kinematics_limit["j3"]["omega_max"];
            hard_constraint_.j3.max_alpha= kinematics_limit["j3"]["alpha_max"];
            hard_constraint_.j4.home     = kinematics_limit["j4"]["home"];
            hard_constraint_.j4.upper    = kinematics_limit["j4"]["upper"];
            hard_constraint_.j4.lower    = kinematics_limit["j4"]["lower"];
            hard_constraint_.j4.max_omega= kinematics_limit["j4"]["omega_max"];
            hard_constraint_.j4.max_alpha= kinematics_limit["j4"]["alpha_max"];
            hard_constraint_.j5.home     = kinematics_limit["j5"]["home"];
            hard_constraint_.j5.upper    = kinematics_limit["j5"]["upper"];
            hard_constraint_.j5.lower    = kinematics_limit["j5"]["lower"];
            hard_constraint_.j5.max_omega= kinematics_limit["j5"]["omega_max"];
            hard_constraint_.j5.max_alpha= kinematics_limit["j5"]["alpha_max"];
            hard_constraint_.j6.home     = kinematics_limit["j6"]["home"];
            hard_constraint_.j6.upper    = kinematics_limit["j6"]["upper"];
            hard_constraint_.j6.lower    = kinematics_limit["j6"]["lower"];
            hard_constraint_.j6.max_omega= kinematics_limit["j6"]["omega_max"];
            hard_constraint_.j6.max_alpha= kinematics_limit["j6"]["alpha_max"];
        }
        catch (fst_parameter::ParamException &exception) {
            log.error("Loading hard constraint exception:");
            log.error(exception.getMessage().c_str());
            return exception.getCode();
        }
        printJointConstraint("Hard constraint:", hard_constraint_);
        log.info("Success!");
    }
    else {
        log.error("Fail loading hard constraint");
        return kinematic_constraint.getLastError();
    }
    
    if (kinematic_constraint.getLastError() == SUCCESS && 
        kinematic_constraint.getParam("soft_constraint", kinematics_limit))
    {
        JointConstraint constraint;
        memset(&constraint, 0, sizeof(constraint));
        try {
            constraint.j1.home     = kinematics_limit["j1"]["home"];
            constraint.j1.upper    = kinematics_limit["j1"]["upper"];
            constraint.j1.lower    = kinematics_limit["j1"]["lower"];
            constraint.j1.max_omega= kinematics_limit["j1"]["omega_max"];
            constraint.j1.max_alpha= kinematics_limit["j1"]["alpha_max"];
            constraint.j2.home     = kinematics_limit["j2"]["home"];
            constraint.j2.upper    = kinematics_limit["j2"]["upper"];
            constraint.j2.lower    = kinematics_limit["j2"]["lower"];
            constraint.j2.max_omega= kinematics_limit["j2"]["omega_max"];
            constraint.j2.max_alpha= kinematics_limit["j2"]["alpha_max"];
            constraint.j3.home     = kinematics_limit["j3"]["home"];
            constraint.j3.upper    = kinematics_limit["j3"]["upper"];
            constraint.j3.lower    = kinematics_limit["j3"]["lower"];
            constraint.j3.max_omega= kinematics_limit["j3"]["omega_max"];
            constraint.j3.max_alpha= kinematics_limit["j3"]["alpha_max"];
            constraint.j4.home     = kinematics_limit["j4"]["home"];
            constraint.j4.upper    = kinematics_limit["j4"]["upper"];
            constraint.j4.lower    = kinematics_limit["j4"]["lower"];
            constraint.j4.max_omega= kinematics_limit["j4"]["omega_max"];
            constraint.j4.max_alpha= kinematics_limit["j4"]["alpha_max"];
            constraint.j5.home     = kinematics_limit["j5"]["home"];
            constraint.j5.upper    = kinematics_limit["j5"]["upper"];
            constraint.j5.lower    = kinematics_limit["j5"]["lower"];
            constraint.j5.max_omega= kinematics_limit["j5"]["omega_max"];
            constraint.j5.max_alpha= kinematics_limit["j5"]["alpha_max"];
            constraint.j6.home     = kinematics_limit["j6"]["home"];
            constraint.j6.upper    = kinematics_limit["j6"]["upper"];
            constraint.j6.lower    = kinematics_limit["j6"]["lower"];
            constraint.j6.max_omega= kinematics_limit["j6"]["omega_max"];
            constraint.j6.max_alpha= kinematics_limit["j6"]["alpha_max"];
        }
        catch (fst_parameter::ParamException &exception) {
            log.error("Exception:");
            log.error(exception.getMessage().c_str());
            return exception.getCode();
        }
        
        if (setSoftConstraint(constraint)) {
            temp_constraint_.normal_constraint = constraint;
            return SUCCESS;
        }
        else {
            log.error("Cannot set joint constraint, because it is INVALID");
            return MOTION_FAIL_IN_INIT;
        }
    }
    else {
        log.error("Fail loading soft constraint");
        return kinematic_constraint.getLastError();
    }
}

ErrorCode ArmGroup::loadDHParameter(const string &path)
{
    log.info("Loading DH parameters from '%s' ...", path.c_str());
    string dh_file = path + "dh.yaml";
    fst_parameter::ParamGroup dh_params(dh_file.c_str());
    ParamValue dh_param;
    if (dh_params.getLastError() == SUCCESS && dh_params.getParam("DH_parameter", dh_param)) {
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
            
            planning_interface_->setDH(dh);
            log.info("Success!");
            return SUCCESS;
        }
        catch (fst_parameter::ParamException &exception) {
            log.error("Exception:");
            log.error(exception.getMessage().c_str());
            return exception.getCode();
        }
    }
    else {
        log.error("Fail loading DH parameters");
        return dh_params.getLastError();
    }
}

ErrorCode ArmGroup::constructPlanningInterface(ParamValue &param)
{
    log.info("Constructing planning interface ...");
    if (planning_interface_ != NULL) {delete planning_interface_; planning_interface_ = NULL;}
    planning_interface_ = new fst_controller::PlanningInterface();

    ErrorCode error = SUCCESS;
    if (planning_interface_ != NULL) {
        if (planning_interface_->initPlanningInterface(param, error)) {
            log.info("  algorithm version: %s", planning_interface_->getAlgorithmVersion().c_str());
            log.info("  set cycle time to %.4lf s", planning_interface_->getCycleTime());
            //log.info("  set velocity to %.2lf mm/s", planning_interface_->getVelocity());
            log.info("  set acceleration to %.2lf mm/s^2", planning_interface_->getAcceleration());
            log.info("  set velocity scaling to %.2lf%%", planning_interface_->getVelocityScaling() * 100);
            log.info("  set acceleration scaling to %.2lf%%", planning_interface_->getAccelerationScaling() * 100);
            log.info("  set jerk to %.2lf", planning_interface_->getJerk());
            log.info("  set joint overshoot to %.4lf rad", planning_interface_->getJointOvershoot());
            log.info("  set joint error angle to %.4lf rad", planning_interface_->getJointErrorAngle());
            log.info("  set omega overload to %.2lf%%", planning_interface_->getOmegaOverload() * 100);
            log.info("  set alpha overload to %.2lf%%", planning_interface_->getAlphaOverload() * 100);
            log.info("  set smooth radius coefficient to %.2lf", planning_interface_->getSmoothRadiusCoefficient());
            log.info("  set smooth curve mode to '%s'", ((string)param["smooth_curve_mode"]).c_str());
            log.info("Success!");
        }
        else {
            log.error("Fail to initialize planning_interface, error code=0x%llx", error);
        }
    }
    else {
        error = MOTION_FAIL_IN_INIT;
        log.error("Cannot construct planning interface, error code=0x%llx", error);
    }

    return error;
}

ErrorCode ArmGroup::constructCalibrator(const string &path)
{  
    log.info("Constructing calibrator ...");
    if (calibrator_ != NULL) {delete calibrator_; calibrator_ = NULL;}

    calibrator_ = new fst_controller::Calibrator(log);
    if (calibrator_ == NULL) {
        log.error("Memory error, cannot construct calibrator");
        return MOTION_FAIL_IN_INIT;
    }
    
    if (calibrator_->initCalibrator(path)) {
        log.info("Success!");
    }
    else {
        log.error("Initialize calibrator failed");
        return calibrator_->getLastError();
    }

    log.info("Downloading JTAC parameters ...");
    if (calibrator_->transmitJtacParam("all")) {
        log.info("Success!");
        usleep(256 * 1000);
    }
    else {
        log.error("Failed");
        return calibrator_->getLastError();
    }

    return SUCCESS;
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
// Function:    setJointOvershoot
// Summary: To set joint overshoot.
// In:      angle   -> desired angle
// Out:     None
// Return:  true    -> joint overshoot changed to given value
//          false   -> joint overshoot NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setJointOvershoot(double angle) {
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
    if (planning_interface_->setJointErrorAngle(angle)) {
        log.info("Set joint error angle to %.4f rad", angle);
        return true;
    }
    else {
        log.error("Cannot set joint error angle to %.4f rad", angle);
        return false;
    }
}

/*
//------------------------------------------------------------------------------
// Function:    setOmegaOverload
// Summary: To set omega overload.
// In:      value   -> desired value
// Out:     None
// Return:  true    -> omega overload changed to given value
//          false   -> omega overload NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setOmegaOverload(double value) {
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
    if (planning_interface_->setAlphaOverload(value)) {
        log.info("Set alpha overload to %.2f%%", value * 100);
        return true;
    }
    else {
        log.error("Cannot set alpha overload to %.2f%%", value * 100);
        return false;
    }
}
*/

//------------------------------------------------------------------------------
// Function:    setSmoothRadiusCoefficient
// Summary: To set smooth radius coefficient.
// In:      coeff   -> desired value
// Out:     None
// Return:  true    -> smooth radius coefficient changed to given value
//          false   -> smooth radius coefficient NOT changed
//------------------------------------------------------------------------------
bool ArmGroup::setSmoothRadiusCoefficient(double coeff) {
    if (planning_interface_->setSmoothRadiusCoefficient(coeff)) {
        log.info("Set smooth radius coefficient to %.2f", coeff);
        return true;
    }
    else {
        log.error("Cannot set smooth radius coefficient to %.2f", coeff);
        return false;
    }
}

ErrorCode ArmGroup::setCurrentJointToStartJoint(void)
{
    ErrorCode err = SUCCESS;
    FeedbackJointState fbjs;
    if (!calibrator_->getCurrentJoint(fbjs)) {
        err = calibrator_->getLastError();
        log.error("Fail to get current joint. Error code=0x%llx", err);
        return err;
    }
    Joint joint;
    memset(&joint, 0, sizeof(joint));
    memcpy(&joint, fbjs.position, NUM_OF_JOINT * sizeof(double));
    printJoint("Using current joint values to set start state: joints=", joint);
    
    if (!checkJointBoundary(joint)) {
        //err = JOINT_OUT_OF_CONSTRAINT;
        log.error("Given joint values out of constraint. Error code=0x%llx", JOINT_OUT_OF_CONSTRAINT);
        log.info("Set temporary constraint using given joint.");
        setTempConstraint(joint);
    }
        
    planning_interface_->setInitialJoint(joint);
    latest_ik_reference_ = joint;
    log.info("Success!");
    
    return err;
}

/*
void ArmGroup::setStartPoseImpl(const Pose &pose) {
    m_pose_start    = pose;
    m_pose_previous = pose;
    m_v_start       = 0.0;
    m_vu_start      = 0.0;
}

bool ArmGroup::setStartStateImpl(const Joint &joint, ErrorCode &err) {
    Pose pose;
    if (planning_interface_->computeForwardKinematics(joint, pose, err)) {
        setStartJointImpl(joint);
        setStartPoseImpl(pose);
        return true;
    }
    else {
        return false;
    }
}*/

void ArmGroup::setTempConstraint(const Joint &joint)
{
    if (!temp_constraint_.is_using_temp_constraint) {
        // temp_constraint_.normal_constraint = joint_constraint_;
        temp_constraint_.normal_constraint = planning_interface_->getJointConstraint();
        temp_constraint_.is_using_temp_constraint = true;
    }

    JointConstraint &cons = temp_constraint_.temp_constraint;
    cons = temp_constraint_.normal_constraint;
    if (cons.j1.upper < joint.j1) cons.j1.upper = joint.j1 + MINIMUM_ALLOWANCE;
    if (cons.j1.lower > joint.j1) cons.j1.lower = joint.j1 - MINIMUM_ALLOWANCE;
    if (cons.j2.upper < joint.j2) cons.j2.upper = joint.j2 + MINIMUM_ALLOWANCE;
    if (cons.j2.lower > joint.j2) cons.j2.lower = joint.j2 - MINIMUM_ALLOWANCE;
    if (cons.j3.upper < joint.j3) cons.j3.upper = joint.j3 + MINIMUM_ALLOWANCE;
    if (cons.j3.lower > joint.j3) cons.j3.lower = joint.j3 - MINIMUM_ALLOWANCE;
    if (cons.j4.upper < joint.j4) cons.j4.upper = joint.j4 + MINIMUM_ALLOWANCE;
    if (cons.j4.lower > joint.j4) cons.j4.lower = joint.j4 - MINIMUM_ALLOWANCE;
    if (cons.j5.upper < joint.j5) cons.j5.upper = joint.j5 + MINIMUM_ALLOWANCE;
    if (cons.j5.lower > joint.j5) cons.j5.lower = joint.j5 - MINIMUM_ALLOWANCE;
    if (cons.j6.upper < joint.j6) cons.j6.upper = joint.j6 + MINIMUM_ALLOWANCE;
    if (cons.j6.lower > joint.j6) cons.j6.lower = joint.j6 - MINIMUM_ALLOWANCE;

    if (setSoftConstraintImpl(cons)) {
        printJointConstraint("Temp constraint:", cons);
    }
    else {
        temp_constraint_.is_using_temp_constraint = false;
    }
}

bool ArmGroup::setSoftConstraintImpl(const JointConstraint &constraint)
{
    if (constraint.j1.upper > hard_constraint_.j1.upper ||
        constraint.j1.lower < hard_constraint_.j1.lower ||
        constraint.j2.upper > hard_constraint_.j2.upper ||
        constraint.j2.lower < hard_constraint_.j2.lower ||
        constraint.j3.upper > hard_constraint_.j3.upper ||
        constraint.j3.lower < hard_constraint_.j3.lower ||
        constraint.j4.upper > hard_constraint_.j4.upper ||
        constraint.j4.lower < hard_constraint_.j4.lower ||
        constraint.j5.upper > hard_constraint_.j5.upper ||
        constraint.j5.lower < hard_constraint_.j5.lower ||
        constraint.j6.upper > hard_constraint_.j6.upper ||
        constraint.j6.lower < hard_constraint_.j6.lower)
    {
        return false;
    }
    else {
        planning_interface_->setJointConstraint(constraint);
        return true;
    }
}

void ArmGroup::deleteTempConstraint()
{
    if (temp_constraint_.is_using_temp_constraint) {
        log.info("Using back to normal constraint, remove temp constraint");
        if (setSoftConstraintImpl(temp_constraint_.normal_constraint)) {
            log.info("Success");
            temp_constraint_.is_using_temp_constraint = false;
        }
        else {
            log.error("Set normal constraint failed");
        }
    }
}

bool ArmGroup::isJointBackIntoNormalConstraint(const Joint &joint)
{
    return  joint.j1 > temp_constraint_.normal_constraint.j1.lower &&
            joint.j1 < temp_constraint_.normal_constraint.j1.upper &&
            joint.j2 > temp_constraint_.normal_constraint.j2.lower &&
            joint.j2 < temp_constraint_.normal_constraint.j2.upper &&
            joint.j3 > temp_constraint_.normal_constraint.j3.lower &&
            joint.j3 < temp_constraint_.normal_constraint.j3.upper &&
            joint.j4 > temp_constraint_.normal_constraint.j4.lower &&
            joint.j4 < temp_constraint_.normal_constraint.j4.upper &&
            joint.j5 > temp_constraint_.normal_constraint.j5.lower &&
            joint.j5 < temp_constraint_.normal_constraint.j5.upper &&
            joint.j6 > temp_constraint_.normal_constraint.j6.lower &&
            joint.j6 < temp_constraint_.normal_constraint.j6.upper;
}

/*
bool ArmGroup::rebuildPlanningVariable(ErrorCode &err) {
    if (!planned_path_fifo_.empty()) {
        if (planned_path_fifo_.back().type == MOTION_JOINT) {
            return setStartStateImpl(planned_path_fifo_.back().joint, err);
        }
        else if (planned_path_fifo_.back().type == MOTION_LINE
                 || planned_path_fifo_.back().type == MOTION_CIRCLE) {
            m_pose_start    = planned_path_fifo_.back().pose;
            m_pose_previous = m_pose_start;
            m_v_start   = 0.0;
            m_vu_start  = 0.0;
            return true;
        }
        else {
            err = MOTION_INTERNAL_FAULT;
            return false;
        }
    }
    else if (!trajectory_fifo_.empty()) {
        return setStartStateImpl(trajectory_fifo_.back().joint, err);
    }
    else {
        return setStartStateImpl(getLatestIKReference(), err);
    }
}

void ArmGroup::abstractLastMotion(int id, MotionType motion_t, SmoothType smooth_t) {
    last_motion_.id = id;
    last_motion_.smooth_type = smooth_t;
    last_motion_.motion_type = motion_t;
}*/

/*
bool ArmGroup::fillPlannedPathFIFO(int id, MotionType motion_t, SmoothType smooth_t, vector<Pose> &path) {
    if (motion_t != MOTION_LINE && motion_t != MOTION_CIRCLE || path.empty()) {
        return false;
    }

    PathPoint pp;
    pp.type = motion_t;

    if (last_motion_.smooth_type == SMOOTH_NONE) {
        pp.id = (id << 2) + POINT_START;
        pp.pose = path.front();
        planned_path_fifo_.push_back(pp);
        pp.id = (id << 2) + POINT_MIDDLE;

        vector<Pose>::iterator itr = path.begin() + 1;
        for ( ; itr != path.end(); ++itr) {
            pp.pose = *itr;
            planned_path_fifo_.push_back(pp);
        }
    }
    else {
        pp.id = (id << 2) + POINT_MIDDLE;

        vector<Pose>::iterator itr = path.begin();
        for ( ; itr != path.end(); ++itr) {
            pp.pose = *itr;
            planned_path_fifo_.push_back(pp);
        }
    }

    if (smooth_t == SMOOTH_NONE) {
        planned_path_fifo_.back().id = (id << 2) + POINT_ENDING;
    }

    return true;
}

bool ArmGroup::fillPlannedPathFIFO(int id, MotionType motion_t, SmoothType smooth_t, vector<Joint> &path) {
    if (motion_t != MOTION_JOINT || path.empty()) {
        return false;
    }

    PathPoint pp;
    pp.type = motion_t;

    if (last_motion_.smooth_type == SMOOTH_NONE) {
        pp.id = (id << 2) + POINT_START;
        pp.joints = path.front();
        planned_path_fifo_.push_back(pp);
        pp.id = (id << 2) + POINT_MIDDLE;

        vector<Joint>::iterator itr = path.begin() + 1;
        for ( ; itr != path.end(); ++itr) {
            pp.joints = *itr;
            planned_path_fifo_.push_back(pp);
        }
    }
    else {
        pp.id = (id << 2) + POINT_MIDDLE;

        vector<Joint>::iterator itr = path.begin();
        for ( ; itr != path.end(); ++itr) {
            pp.joints = *itr;
            planned_path_fifo_.push_back(pp);
        }
    }

    if (smooth_t == SMOOTH_NONE) {
        planned_path_fifo_.back().id = (id << 2) + POINT_ENDING;
    }

    return true;
}*/
/*
bool ArmGroup::managePlanningResult(int id, MotionType motion_t, SmoothType smooth_t, ErrorCode &err) {
    bool result = true;
    switch (smooth_t) {
        case SMOOTH_J2J:
        case SMOOTH_L2J:
        case SMOOTH_C2J:    
                allowed_motion_type_ = MOTION_JOINT;
                break;
        case SMOOTH_J2L:
        case SMOOTH_L2L:
        case SMOOTH_C2L:
                allowed_motion_type_ = MOTION_LINE;
                break;
        case SMOOTH_J2C:
        case SMOOTH_L2C:
        case SMOOTH_C2C:
                allowed_motion_type_ = MOTION_CIRCLE;
                break;
        case SMOOTH_NONE:
                allowed_motion_type_ = MOTION_NONE;
                break;
        default:
                err    = MOTION_INTERNAL_FAULT;
                result = false;
    }

    if (last_motion_.smooth_type == SMOOTH_NONE) {
        size_t length = planned_path_fifo_.size();
        abstractLastMotion(id, motion_t, smooth_t);
        result = result && pickPoints(err);
        if (result) planned_path_fifo_.at(length).id = last_motion_.id << 2 | POINT_START;
    }
    else {
        abstractLastMotion(id, motion_t, smooth_t);
        result = result && pickPoints(err);
    }

    return result;
}
*/

/*
//------------------------------------------------------------------------------
// Function:    setLatestIKReference
// Summary: To set latest IK reference.
// In:      joint_reference -> new IK reference
// Out:     error_code  -> error code
// Return:  true    -> latest IK reference setted to joint_reference scucessfully
//          false   -> failed to set latest IK reference
//------------------------------------------------------------------------------
void ArmGroup::setLatestIKReference(const Joint &joint_reference) {
    latest_ik_reference_ = joint_reference;
}

//------------------------------------------------------------------------------
// Function:    getLatestIKReference
// Summary: To get latest IK reference values.
// In:      None
// Out:     None
// Return:  a group of joint values used as IK reference
//------------------------------------------------------------------------------
const Joint& ArmGroup::getLatestIKReference(void) {
    return latest_ik_reference_;
}
*/

//------------------------------------------------------------------------------
// Function:    checkJointBoundary
// Summary: To check whether a group of joint values are valid according to
//          joint constraint.
// In:      joint -> joint_values needed to be checked
// Out:     None
// Return:  true  -> valid
//------------------------------------------------------------------------------
bool ArmGroup::checkJointBoundary(const Joint &joint) {
    return planning_interface_->checkJointBoundry(joint);
}


//------------------------------------------------------------------------------
// Function:    convertPath2Trajectory
// Summary: To convert numbers of posepoint in m_cartesian_path_FIFO
//          into jointpoint in trajectory_fifo_.
// In:      num        -> number of pose that needed to be converted
// Out:     error_code -> error code
// Return:  <0         -> ERROR occurred during converting
//          >=0        -> number of pose that convered actually
//------------------------------------------------------------------------------
int ArmGroup::convertPath2Trajectory(int num, ErrorCode &err) {
    err = SUCCESS;
    if (planned_path_fifo_.size() < planning_interface_->getTrajectorySegmentLength()) {
        pickPoints();
    }

    num = num > planned_path_fifo_.size() ? planned_path_fifo_.size() : num;

    int conv_cnt = 0;
    vector<PathPoint>::iterator pp_it = planned_path_fifo_.begin();
    JointPoint jp;
    memset(&jp, 0, sizeof(JointPoint));
    
    for (conv_cnt = 0; conv_cnt < num; ++conv_cnt) {
        if (pp_it->type == MOTION_LINE || pp_it->type == MOTION_CIRCLE) {
            if (computeIK(pp_it->pose, jp.joint, err)) {
                jp.stamp    = pp_it->stamp;
                jp.level    = pp_it->level;
                jp.source   = pp_it->source;
                trajectory_fifo_.push_back(jp);
                if (pp_it->level == POINT_LAST) {
                    // last point in a commond
                    pp_it->source->GetJointStatus(latest_ik_reference_, jp.joint);
                }
                else if (pp_it->level == POINT_ENDING) {
                    // last point in a commond and a motion
                    pp_it->source->GetJointStatus(latest_ik_reference_, jp.joint);
                    planning_interface_->setInitialJoint(jp.joint);
                }
                latest_ik_reference_ = jp.joint;
            }
            else {
                log.error("Error while converting cartesian point to joint space, Error code=0x%llx", err);
                log.error("%d points has been converted before the error", conv_cnt);
                printJoint("IK reference: ", latest_ik_reference_);
                printJoint("IK result:    ", jp.joint);
                printPose("Target Pose:  ", pp_it->pose);
                break;
            }
        }
        else if (pp_it->type == MOTION_JOINT) {
            jp.stamp    = pp_it->stamp;
            jp.level    = pp_it->level;
            jp.source   = pp_it->source;
            jp.joint    = pp_it->joint;

            if (pp_it->level == POINT_ENDING) {
                // last point in a commond and a motion
                planning_interface_->setInitialJoint(jp.joint);
            }

            trajectory_fifo_.push_back(jp);
            latest_ik_reference_ = jp.joint;
        }
        else {
            log.error("INVALID point type (%d) found while converting FIFO1 > FIFO2", pp_it->type);
            err = MOTION_INTERNAL_FAULT;
            break;
        }

        ++pp_it;
    }
    
    if (conv_cnt != 0) {
        pp_it = planned_path_fifo_.begin();
        planned_path_fifo_.erase(pp_it, pp_it + conv_cnt);
    }

    return conv_cnt;
}

bool ArmGroup::isMotionExecutable(MotionType motion_t, ErrorCode &err) {
    /*
    if (planning_interface_->getFIFOLength() > planning_interface_->getTrajectorySegmentLength()) {
        err = INVALID_SEQUENCE;
        return false;
    }
    if (planning_interface_->getFIFOLength() > 0) {
        if (pickPoints() != true)    return false;
    }
    */

    if (!planning_interface_->isMotionExecutable(motion_t)) {err = INVALID_SEQUENCE; return false;}
    if (calibrator_->getCurrentState() < CALIBRATED && !calibrator_->isUsingTempZeroOffset()) {
        err = NEED_CALIBRATION;
        return false;
    }
    switch (motion_t) {
        case MOTION_JOINT:
            if (!planned_path_fifo_.empty() &&
                (planned_path_fifo_.back().type == MOTION_LINE || planned_path_fifo_.back().type == MOTION_CIRCLE))
            {
                err = INVALID_SEQUENCE;
                return false;
            }
            return true;
        case MOTION_LINE:
        case MOTION_CIRCLE:
            if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
                err = NEED_CALIBRATION;
                return false;
            }
            return true;
        default:
            err = MOTION_INTERNAL_FAULT;
            log.error("API:'isMotionExecutable' received an invalid motion type: %d", motion_t);
            return false;
    }
}

bool ArmGroup::pickPoints() {
    resizeJointTrajectoryFIFO();
    //log.info("Picking points ...");
    vector<PathPoint> points;
    
    if (planning_interface_->pickPoints(points)) {
        planned_path_fifo_.insert(planned_path_fifo_.end(), points.begin(), points.end());
        //log.info("  Success, %d points have been picked into FIFO1", points.size());
        return true;
    }
    else {
        //log.warn("  failed to pick points");
        return false;
    }
}

/*
bool ArmGroup::pickPoints(ErrorCode &err) {
    resizeJointTrajectoryFIFO();
    log.info("Pick points ...");
    if (planning_interface_->getFIFOLength() > 0) {
        if (last_motion_.motion_type == MOTION_JOINT) {
            vector<Joint> points;
            if (planning_interface_->pickPoints(points, err)) {
                PathPoint pp;
                pp.id   = last_motion_.id << 2 | POINT_MIDDLE;
                pp.type = last_motion_.motion_type;
                vector<Joint>::iterator it;
                for(it = points.begin(); it != points.end(); ++it) {
                    pp.joint = *it;
                    planned_path_fifo_.push_back(pp);
                }
                if (planning_interface_->getFIFOLength() == 0 && last_motion_.smooth_type == SMOOTH_NONE)
                    planned_path_fifo_.back().id = last_motion_.id << 2 | POINT_ENDING;
                log.info("  %d points have been picked into FIFO1, and %d points left",
                        points.size(), planning_interface_->getFIFOLength());
                return true;
            }
            else {
                log.error("Fail to pick points, error_code=0x%llx", err);
                return false;
            }
        }
        else {
            vector<Pose> points;
            if (planning_interface_->pickPoints(points, err)) {
                PathPoint pp;
                pp.id   = last_motion_.id << 2 | POINT_MIDDLE;
                pp.type = last_motion_.motion_type;
                vector<Pose>::iterator it;
                for(it = points.begin(); it != points.end(); ++it) {
                    pp.pose = *it;
                    planned_path_fifo_.push_back(pp);
                }
                if (planning_interface_->getFIFOLength() == 0 && last_motion_.smooth_type == SMOOTH_NONE)
                    planned_path_fifo_.back().id = last_motion_.id << 2 | POINT_ENDING;
                log.info("  %d points have been picked into FIFO1, and %d points left",
                        points.size(), planning_interface_->getFIFOLength());
                return true;
            }
            else {
                log.error("Fail to pick points, error_code=0x%llx", err);
                return false;
            }
        }
    }
    else {
        return false;
    }
}
*/

bool ArmGroup::resizeJointTrajectoryFIFO(void) {
    // log.info("Resize FIFO2 ...");
    if (trajectory_fifo_.size() >= 2) {
        vector<JointPoint>::iterator it1 = trajectory_fifo_.end() - 1;
        vector<JointPoint>::iterator it2 = trajectory_fifo_.end() - 2;
        int size = planning_interface_->estimateFIFOLength(it1->joint, it2->joint);
        if      (size < 50)     size = 50;
        else if (size > 500)    size = 500;
        
        if (size != trajectory_fifo_dynamic_length_) {
            trajectory_fifo_dynamic_length_ = size;
            log.info("Resie FIFO2 to %d", trajectory_fifo_dynamic_length_);
        }
        return true;
    }
    else {
        // log.error("  failed: no enough points in FIFO2, need 2 but has %d only", trajectory_fifo_.size());
        return false;
    }
}

/*
bool ArmGroup::resumeByOrigin(ErrorCode &err) {
    trajectory_fifo_.insert(trajectory_fifo_.begin(),
                            suspend_state_.fifo2_cache.begin(),
                            suspend_state_.fifo2_cache.end());
    suspend_state_.is_suspended = false;
    log.info("Resume successfully, pattern=origin, trajectory length=%zd", trajectory_fifo_.size());
    return true;
}

bool ArmGroup::resumeByReplan(ErrorCode &err) {
    int num = planned_path_fifo_.size() < 100 ? planned_path_fifo_.size() : 100;
    if (convertPath2Trajectory(num, err) == num) {
        bool last_point_is_ending_point = false;
        suspend_state_.replan_trajectory.clear();
        vector<JointPoint>::iterator itr = trajectory_fifo_.begin();
        for ( ; itr != trajectory_fifo_.end(); ++itr) {
            suspend_state_.replan_trajectory.push_back(itr->joint);
            if ((itr->id & POINT_LEVEL_MASK) == POINT_ENDING) {
                log.info("The last point is ending point.");
                last_point_is_ending_point = true;
            }
        }
        bool result = planning_interface_->replanRestartTrajectory(suspend_state_.replan_trajectory,
                                                                   suspend_state_.last_point,
                                                                   err);
        if (result == true) {
            JointPoint jp;
            memset(&jp.omega, 0, sizeof(JointOmega));
            jp.id = trajectory_fifo_.front().id;
            jp.id = jp.id & ~POINT_LEVEL_MASK | POINT_MIDDLE;
            trajectory_fifo_.clear();
            vector<Joint>::iterator itr = suspend_state_.replan_trajectory.begin();
            for ( ; itr != suspend_state_.replan_trajectory.end(); ++itr) {
                jp.joint = *itr;
                trajectory_fifo_.push_back(jp);
            }
            log.info("Resume pattern: replan, trajectory length: %d", trajectory_fifo_.size());
            if (last_point_is_ending_point && !trajectory_fifo_.empty()) {
                trajectory_fifo_.back().id &= ~POINT_LEVEL_MASK;
                trajectory_fifo_.back().id |= POINT_ENDING;
                log.info("Turning the last point to ending point.");
            }
        }
        else {
            suspend_state_.is_suspended = false;
            log.error("ERROR while replanning resume trajectory, error_code=%d", result);
            return false;
        }
    }
    else {
        suspend_state_.is_suspended = false;
        log.error("ERROR while filling FIFO2, error_code=0x%llx", err);
        return false;
    }
    suspend_state_.is_suspended = false;
    log.info("Resume successfully.");
    return true;
}
*/

void ArmGroup::printJoint(const Joint &joint) {
    log.info("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
             joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6, joint.j7, joint.j8, joint.j9);
}

void ArmGroup::printJoint(const char *str, const Joint &joint) {
    log.info("%s%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", str,
             joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6, joint.j7, joint.j8, joint.j9);
}

void ArmGroup::printJointLimit(const JointLimit &joint_limit) {
    log.info("lower=%lf, home=%lf, upper=%lf, max_omega=%lf, max_alpha=%lf",
             joint_limit.lower, joint_limit.home, joint_limit.upper,
             joint_limit.max_omega, joint_limit.max_alpha);
}

void ArmGroup::printJointLimit(const char *str, const JointLimit &joint_limit) {
    log.info("%slower=%lf, home=%lf, upper=%lf, max_omega=%lf, max_alpha=%lf", str,
             joint_limit.lower, joint_limit.home, joint_limit.upper,
             joint_limit.max_omega, joint_limit.max_alpha);
}

void ArmGroup::printJointConstraint(const char *str, const JointConstraint &constraint) {
    log.info("%s", str);
    printJointLimit("  J1 limit: ", constraint.j1);
    printJointLimit("  J2 limit: ", constraint.j2);
    printJointLimit("  J3 limit: ", constraint.j3);
    printJointLimit("  J4 limit: ", constraint.j4);
    printJointLimit("  J5 limit: ", constraint.j5);
    printJointLimit("  J6 limit: ", constraint.j6);
    printJointLimit("  J7 limit: ", constraint.j7);
    printJointLimit("  J8 limit: ", constraint.j8);
    printJointLimit("  J9 limit: ", constraint.j9);
}

void ArmGroup::printPose(const Pose &pose) {
    log.info("position=%lf,%lf,%lf orientation=%lf,%lf,%lf,%lf",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

void ArmGroup::printPose(const char *str, const Pose &pose) {
    log.info("%sposition=%lf,%lf,%lf orientation=%lf,%lf,%lf,%lf", str,
             pose.position.x, pose.position.y, pose.position.z,
             pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

void ArmGroup::printPoseEuler(const PoseEuler &pose_euler) {
    log.info("position=%lf,%lf,%lf orientation=%lf,%lf,%lf",
            pose_euler.position.x, pose_euler.position.y, pose_euler.position.z,
            pose_euler.orientation.a, pose_euler.orientation.b, pose_euler.orientation.c);
}

void ArmGroup::printPoseEuler(const char *str, const PoseEuler &pose_euler) {
    log.info("%sposition=%lf,%lf,%lf orientation=%lf,%lf,%lf", str,
            pose_euler.position.x, pose_euler.position.y, pose_euler.position.z,
            pose_euler.orientation.a, pose_euler.orientation.b, pose_euler.orientation.c);
}

void ArmGroup::printJointPoint(const JointPoint &point) {
    log.info("Joint point: ID=%d, stamp=%d, level=%s", point.source->getMotionID(), point.stamp,
             point.level == POINT_MIDDLE ? "middle point" : (point.level == POINT_START ? "start point" : "end point"));
    log.info("  joint=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
             point.joint.j1, point.joint.j2, point.joint.j3, 
             point.joint.j4, point.joint.j5, point.joint.j6,
             point.joint.j7, point.joint.j8, point.joint.j9);
    log.info("  omega=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
             point.omega.j1, point.omega.j2, point.omega.j3,
             point.omega.j4, point.omega.j5, point.omega.j6,
             point.omega.j7, point.omega.j8, point.omega.j9);
}

void ArmGroup::printJointPoint(const char *str, const JointPoint &point) {
    log.info(str);
    printJointPoint(point);
}

void ArmGroup::printJointOutput(const JointOutput &point)
{
    log.info("Joint output: ID=%d, level=%s", point.id,
        point.level == POINT_MIDDLE ? "middle point" : (point.level == POINT_START ? "start point" : "end point"));
    log.info("  joint=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
        point.joint.j1, point.joint.j2, point.joint.j3, 
        point.joint.j4, point.joint.j5, point.joint.j6,
        point.joint.j7, point.joint.j8, point.joint.j9);
}

void ArmGroup::printJointOutput(const char *str, const JointOutput &point) {
    log.info(str);
    printJointOutput(point);
}

void ArmGroup::printPathPoint(const PathPoint &point) {
    string point_t, motion_t;
    if      (point.type == MOTION_JOINT)    point_t = "joint";
    else if (point.type == MOTION_LINE)     point_t = "line";
    else if (point.type == MOTION_CIRCLE)   point_t = "circle";
    else {log.warn("get an INVALID path point in API:printPathPoint"); return;}

    switch (point.source->getMotionType()) {
        case MOTION_JOINT:
            motion_t = "joint";
            break;
        case MOTION_LINE:
            motion_t = "line";
            break;
        case MOTION_CIRCLE:
            motion_t = "circle";
            break;
        default:
            log.warn("get an INVALID motion type in API:printPathPoint");
            return;
    }

    log.info("PathPoint: ID=%d, stamp=%d, level=%s, point-type=%s, motion_type=%s",
             point.source->getMotionID(), point.stamp,
             point.level == POINT_MIDDLE ? "middle point" : (point.level == POINT_START ? "start point" : "end point"),
             point_t.c_str(), motion_t.c_str());
    if (point.type == MOTION_JOINT) printJoint("  joints=", point.joint);
    else                            printPose("  pose: ", point.pose);
}

void ArmGroup::printPathPoint(const char *str, const PathPoint &point) {
    log.info(str);
    printPathPoint(point);
}



void ArmGroup::lockArmGroup(void) {
    //log.info("Try to lock arm group.");
    pthread_mutex_lock(&group_mutex_);
}

void ArmGroup::unlockArmGroup(void) {
    pthread_mutex_unlock(&group_mutex_);
    //log.info("Arm group unlocked.");
}

}
