/*************************************************************************
	> File Name: motion_controller_planning_interface.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年02月20日 星期一 09时59分15秒
 ************************************************************************/

#include <motion_controller/motion_controller_planning_interface.h>

using std::cout;
using std::endl;
using std::vector;
using std::string;

namespace fst_controller {

PlanningInterface::PlanningInterface() {
    planner_    = NULL;
    trajectory_segment_length_range_.min = 50;
    trajectory_segment_length_range_.max = 5000;
    cycle_time_range_.min   = 0.0005;       // s
    cycle_time_range_.max   = 0.05;         // s
    velocity_range_.min     = 0.1;          // mm/s
    velocity_range_.max     = 4000.0001;    // mm/s
    acceleration_range_.min = 999.9999;     // mm/s^2
    acceleration_range_.max = 16000.0001;   // mm/s^2
    velocity_scaling_range_.min = 0.01;
    velocity_scaling_range_.max = 1.0001;
    acceleration_scaling_range_.min = 0.01;
    acceleration_scaling_range_.max = 1.0001;
    jerk_range_.min = 5.0;
    jerk_range_.max = 50.0; 
    joint_overshoot_range_.min  = 0.001;        // 0.001 rad
    joint_overshoot_range_.max  = 1.571;        // Pi/2 rad
    joint_errorangle_range_.min = 0.001;        // 0.001 rad
    joint_errorangle_range_.max = 0.524;        // Pi/6 rad
    omega_overload_range_.min  = 0.01;
    omega_overload_range_.max  = 1.2;
    alpha_overload_range_.min = 0.9999;
    alpha_overload_range_.max = 3.0001;
    smooth_radius_coefficient_range_.min = 0.9999;
    smooth_radius_coefficient_range_.max = 5.0001;

    trajectory_segment_length_ = 500;
    cycle_time_ = 0.001;
    velocity_   = 500.0;
    acceleration_   = 5000.0;
    velocity_scaling_       = 1.0;
    acceleration_scaling_   = 1.0;
    jerk_   = 10.0;
    joint_overshoot_    = 0.25;
    joint_errorangle_   = 0.15;
    omega_overload_     = 1.0;
    alpha_overload_     = 1.0;
    smooth_radius_coefficient_ = 1.0;
    curve_mode_  = 1;
    fifo_length_ = 0;
}

bool PlanningInterface::initPlanningInterface(fst_parameter::ParamValue &params, ErrorCode &err) {
    if (planner_ != NULL) {
        delete planner_;
        planner_ = NULL;
    }
    planner_ = new fst_controller::TrajPlan();
    if (planner_ == NULL) {
        err = MOTION_FAIL_IN_INIT;
        return false;
    }
    try {
        if (setTrajectorySegmentLength((int)params["trajectory_segment_length"]) &&
            setCycleTime(params["cycle_time"]) &&
            setVelocity(params["velocity"]) &&
            setAcceleration(params["acceleration"]) &&
            setVelocityScaling(params["velocity_scaling"]) &&
            setAccelerationScaling(params["acceleration_scaling"]) &&
            setJerk(params["jerk"]) &&
            setJointOvershoot(params["joint_overshoot"]) &&
            setJointErrorAngle(params["joint_errorangle"]) &&
            setOmegaOverload(params["omega_overload"]) &&
            setAlphaOverload(params["alpha_overload"]) &&
            setSmoothRadiusCoefficient(params["smooth_radius_coefficient"]) &&
            setCurveMode((params["smooth_curve"])[(string)params["smooth_curve_mode"]])) {
                return true;
        }
        else {
            err = MOTION_FAIL_IN_INIT;
            return false;
        }
    }
    catch (fst_parameter::ParamException &e) {
        err = e.getCode();
        // std::cout << e.getMessage() << std::endl << "error_code=" << e.getCode() << std::endl;
        return false;
    }
}

PlanningInterface::~PlanningInterface() {
    delete planner_;
    planner_ = NULL;
}

string PlanningInterface::getAlgorithmVersion(void) {
    return planner_->getVersion();
}

double PlanningInterface::getCycleTime(void) {
    return cycle_time_;
}

double PlanningInterface::getVelocity(void) {
    return velocity_;
}

double PlanningInterface::getAcceleration(void) {
    return acceleration_;
}

double PlanningInterface::getVelocityScaling(void) {
    return velocity_scaling_;
}

double PlanningInterface::getAccelerationScaling(void) {
    return acceleration_scaling_;
}

double PlanningInterface::getJerk(void) {
    return jerk_;
}

double PlanningInterface::getJointOvershoot(void) {
    return joint_overshoot_;
}

double PlanningInterface::getJointErrorAngle(void) {
    return joint_errorangle_;
}

double PlanningInterface::getOmegaOverload(void) {
    return omega_overload_;
}

double PlanningInterface::getAlphaOverload(void) {
    return alpha_overload_;
}

double PlanningInterface::getSmoothRadiusCoefficient(void) {
    return smooth_radius_coefficient_;
}

double PlanningInterface::getCurveMode(void) {
    return curve_mode_;
}

const JointConstraints& PlanningInterface::getJointConstraints(void) {
    return joint_constraints_;
}

void PlanningInterface::clearFIFO(void) {
    fifo_length_ = 0;
}

int PlanningInterface::getFIFOLength(void) {
    return fifo_length_;
}

unsigned int PlanningInterface::getTrajectorySegmentLength(void) {
    return trajectory_segment_length_;
}

bool PlanningInterface::setTrajectorySegmentLength(unsigned int length) {
    if (length > trajectory_segment_length_range_.min && length < trajectory_segment_length_range_.max) {
        this->trajectory_segment_length_ = length;
        this->planner_->setVlength(length);
        return true;
    }
    else {
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    setCycleTime
// Summary: To set cycle time of interpolation algorithm.
// In:      cycle_time  -> desired cycle time
// Out:     None
// Return:  true        -> cycle time changed to new value
//          false       -> cycle time NOT changed
//------------------------------------------------------------------------------
bool PlanningInterface::setCycleTime(double cycle_time) {
    if (cycle_time > cycle_time_range_.min && cycle_time < cycle_time_range_.max) {
        this->cycle_time_ = cycle_time;
        this->planner_->setCycleTime(cycle_time);
        return true;
    }
    else {
        return false;
    }
}

bool PlanningInterface::setVelocity(double vel) {
    if (vel > velocity_range_.min && vel < velocity_range_.max) {
        this->velocity_ = vel;
        return true;
    }
    else {
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    setAcceleration
// Summary: To set the algorithm acceleration in cartesian space.
// In:      acce    -> desired acceleration
// Out:     None
// Return:  true    -> acceleration changed to new value
//          false   -> acceleration NOT changed
//------------------------------------------------------------------------------
bool PlanningInterface::setAcceleration(double acce) {
    if (acce > acceleration_range_.min && acce < acceleration_range_.max) {
        this->acceleration_ = acce;
        this->planner_->setAcceleration(acce * acceleration_scaling_);
        return true;
    }
    else {
        return false;
    }
}

bool PlanningInterface::setVelocityScaling(double scaling) {
    if (scaling > velocity_scaling_range_.min && scaling < velocity_scaling_range_.max) {
        this->velocity_scaling_ = scaling;
        return true;
    }
    else {
        return false;
    }
}

bool PlanningInterface::setAccelerationScaling(double scaling) {
    if (scaling > acceleration_scaling_range_.min && scaling < acceleration_scaling_range_.max) {
        this->acceleration_scaling_ = scaling;
        this->planner_->setAcceleration(acceleration_ * scaling);
        return true;
    }
    else {
        return false;
    }
}

bool PlanningInterface::setJerk(double jerk) {
    if (jerk > jerk_range_.min && jerk < jerk_range_.max) {
        this->jerk_ = jerk;
        this->planner_->setJaratio(jerk);
        return true;
    }
    else {
        return false;
    }
}

bool PlanningInterface::setJointOvershoot(double angle) {
    if (angle > joint_overshoot_range_.min && angle < joint_overshoot_range_.max) {
        this->joint_overshoot_ = angle;
        this->planner_->setOvershoot(angle);
        return true;
    }
    else {
        return false;
    }
}

bool PlanningInterface::setJointErrorAngle(double angle) {
    if (angle > joint_errorangle_range_.min && angle < joint_errorangle_range_.max) {
        this->joint_errorangle_ = angle;
        this->planner_->seterrorangle(angle);
        return true;
    }
    else {
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    setOmegaOverload
// Summary: To set omega overload scaling factor.
// In:      value   -> desired value
// Out:     None
// Return:  true    -> omega overload scaling factor changed to new value
//          false   -> omega overload scaling factor NOT changed
//------------------------------------------------------------------------------
bool PlanningInterface::setOmegaOverload(double value) {
    if (value > omega_overload_range_.min && value < omega_overload_range_.max) {
        this->omega_overload_ = value;
        this->planner_->setLimitScale(value);
        return true;
    }
    else {
        return false;
    }
}

bool PlanningInterface::setAlphaOverload(double value) {
    if (value > alpha_overload_range_.min && value < alpha_overload_range_.max) {
        this->alpha_overload_ = value;
        this->planner_->setoverload(value);
        return true;
    }
    else {
        return false;
    }
}

bool PlanningInterface::setSmoothRadiusCoefficient(double coeff) {
    if (coeff > smooth_radius_coefficient_range_.min && coeff < smooth_radius_coefficient_range_.max) {
        this->smooth_radius_coefficient_ = coeff;
        this->planner_->setRadiusCoefficient(coeff);
        return true;
    }
    else {
        return false;
    }
}

bool PlanningInterface::setCurveMode(int mode) {
    this->curve_mode_ = mode;
    this->planner_->setCurveMode(mode);
    return true;
}

//------------------------------------------------------------------------------
// Function:    setJointConstraints
// Summary: To set joint constraints in Kinematics algorithm.
// In:      constraints -> joint constraints
// Out:     None
// Return:  true        -> joint constraints changed to new values
//          false       -> joint constraints NOT changed
//------------------------------------------------------------------------------
bool PlanningInterface::setJointConstraints(const JointConstraints &constraints) {
    joint_constraints_ = constraints;
    this->planner_->setAxisLimit(constraints);
    return true;
}

bool PlanningInterface::setAlphaScaling(double percent) {
    if (percent > 0.0 && percent < 100.0) {
        percent = percent / 100.0;
        JointConstraints temp_constraints = joint_constraints_;
        temp_constraints.j1.max_alpha *= percent;
        temp_constraints.j2.max_alpha *= percent;
        temp_constraints.j3.max_alpha *= percent;
        temp_constraints.j4.max_alpha *= percent;
        temp_constraints.j5.max_alpha *= percent;
        temp_constraints.j6.max_alpha *= percent;
        this->planner_->setAxisLimit(temp_constraints);
        return true;
    }
    else {
        return false;
    }
}

bool PlanningInterface::setDH(const DHGroup &dh) {
    dh_parameter_ = dh;
    this->planner_->setDH(dh_parameter_);
    return true;
}

void PlanningInterface::setToolFrame(const Transformation &tool_frame) {
    PoseEuler pose;
    pose.position = tool_frame.position;
    pose.orientation = tool_frame.orientation;
    this->planner_->setToolFrame(pose);
}

void PlanningInterface::setUserFrame(const Transformation &user_frame) {
    PoseEuler pose;
    pose.position = user_frame.position;
    pose.orientation = user_frame.orientation;
    this->planner_->setUserFrame(pose);
}

//------------------------------------------------------------------------------
// Function:    transformPoseEuler2Pose
// Summary: To transform a poseEuler point to a pose point.
// In:      poes_e -> the poseEuler to be transformed
// Out:     None
// Return:  pose point
//------------------------------------------------------------------------------
Pose PlanningInterface::transformPoseEuler2Pose(const PoseEuler &pose_e) {
    fst_controller::Pose pose;
    
    memset(&pose, 0, sizeof(pose));
    if (0 == this->planner_->Euler2Quatern(pose_e, pose)) {
        return pose;
    }
    else {
        // log.error("Error while transform Euler to Quaternion, error code=%d", result);
        return pose;
    }
}


//------------------------------------------------------------------------------
// Function:    transformPose2PoseEuler
// Summary: To transform a pose point to a poseEuler point.
// In:      poes -> the pose to be transformed
// Out:     None
// Return:  poseEuler point
//------------------------------------------------------------------------------
PoseEuler PlanningInterface::transformPose2PoseEuler(const Pose &pose) {
    fst_controller::PoseEuler pose_e;
    
    memset(&pose_e, 0, sizeof(pose_e));
    if (0 == this->planner_->Quatern2Euler(pose, pose_e)) {
        return pose_e;
    }
    else {
        // log.error("Error while transform Quaternion to Euler, error code=%d", result);
        return pose_e;
    }
}

bool PlanningInterface::getJointFromPose(const Pose &pose, const JointValues &reference,
                                         JointValues &joint, double time_interval, ErrorCode &err) {
    computeInverseKinematics(pose, reference, joint, err);
    if (err == SUCCESS) {
        return true;
    }
    else if (err == IK_EXCESSIVE_DISTANCE) {
        if (abs(reference.j1 - joint.j1) < joint_constraints_.j1.max_omega * time_interval &&
            abs(reference.j2 - joint.j2) < joint_constraints_.j2.max_omega * time_interval &&
            abs(reference.j3 - joint.j3) < joint_constraints_.j3.max_omega * time_interval &&
            abs(reference.j4 - joint.j4) < joint_constraints_.j4.max_omega * time_interval &&
            abs(reference.j5 - joint.j5) < joint_constraints_.j5.max_omega * time_interval &&
            abs(reference.j6 - joint.j6) < joint_constraints_.j6.max_omega * time_interval) {
                err = SUCCESS;
                return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
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
bool PlanningInterface::computeInverseKinematics(const Pose &pose,
                                                 const JointValues &joint_reference,
                                                 JointValues &joint_result,
                                                 ErrorCode &err) {
    int res = this->planner_->InverseKinematics(pose, joint_reference, joint_result);

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
bool PlanningInterface::computeForwardKinematics(const JointValues &joint,
                                                 Pose &pose,
                                                 ErrorCode &err) {
    int res = this->planner_->ForwardKinematics(joint, pose);
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
// Function:    MoveJ2J
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:      jp_start    -> initial position and omega of six joints
//          jp_target   -> joint target of the plan
//          jp_next     -> the target joint of the next plan used for smoothing
//          v_ref       -> reference velocity
//          cnt         -> desired cnt value of the plan, range:0-100
// Out:     jp_end      -> the ending position and omega of six joints in this plan
//          planned_path-> planned path in joint space
//          error_code  -> error code
// Return:  true        -> plan successfully
//------------------------------------------------------------------------------
bool PlanningInterface::MoveJ2J(JointPoint &jp_start,
                                const JointValues &j_target, double v_target, int cnt_target,
                                const JointValues &j_next,   double v_next,   int cnt_next,
                                ErrorCode &err) {
    if (fifo_length_ >0) {err = INVALID_SEQUENCE; return false;}

    if (v_target > 100.0)  v_target = 100.0;
    if (v_next   > 100.0)  v_next   = 100.0;
    v_target = v_target * velocity_scaling_;
    v_next   = v_next   * velocity_scaling_;

    JointPoint  jp_out;
    JCommand    obj, next_obj;
    obj.joints          = j_target;
    obj.v_percentage    = v_target;
    obj.cnt             = cnt_target;
    next_obj.joints         = j_next;
    next_obj.v_percentage   = v_next;
    next_obj.cnt            = cnt_next;
    int traj_length = 0;
    int res = this->planner_->MoveJ2J(jp_start, obj, next_obj, jp_out, traj_length);
    
    switch (res) {
        case 0:
            fifo_length_ = traj_length;
            jp_start = jp_out;
            err = SUCCESS;
            return true;
        case 1021:
            err = TARGET_REPEATED;
            return false;
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

/*
//------------------------------------------------------------------------------
// Function:    MoveJ2J
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:      jp_start    -> initial position and omega of six joints
//          pose_target -> pose target of the plan
//          pose_next   -> the target pose of the next plan used for smoothing 
//          v_ref       -> reference velocity
//          cnt         -> desired cnt value of the plan, range:0-100
// Out:     jp_end      -> the ending position and omega of six joints in this plan
//          planned_path-> planned path in joint space
//          error_code  -> error code
// Return:  true        -> plan successfully
//------------------------------------------------------------------------------
bool PlanningInterface::MoveJ2J(JointPoint &jp_start,
                                const Pose &pose_target, const Pose &pose_next, double v_ref, int cnt,
                                std::vector<JointValues> &planned_path,
                                ErrorCode &err) {
    double v_percent = v_ref * velocity_scaling_ / velocity_range_.max * 100;
    if (v_percent > 100.0) {
        v_percent = 100.0;
    }

    int res = this->planner_->MoveJ2J_Cspace(jp_start, pose_target, pose_next, v_percent, cnt, planned_path);
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1021:
            err = TARGET_REPEATED;
            return false;
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
*/


//------------------------------------------------------------------------------
// Function:    MoveJ2L
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:      jp_start    -> initial position and omega of six joints
//          joint_target-> joint target of the plan
//          v_ref       -> reference velocity
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
bool PlanningInterface::MoveJ2L(const JointPoint &jp_start,
                                const JointValues &joint_target, double v_target, int cnt_target,
                                const Pose &pose_next, double v_next, int cnt_next,
                                Pose &pose_start, Pose &pose_previous,
                                double &v_start, double &vu_start,
                                ErrorCode &err) {
    if (fifo_length_ >0) {err = INVALID_SEQUENCE; return false;}

    if (v_target > 100.0)  v_target = 100.0;
    v_next   = v_next * velocity_scaling_;
    v_target = v_target * velocity_scaling_;

    JCommand    obj_target;
    LCommand    obj_next;
    LInitial    line_out;
    obj_target.v_percentage = v_target;
    obj_target.joints   = joint_target;
    obj_target.cnt  = cnt_target;
    obj_next.velocity   = v_next;
    obj_next.pose   = pose_next;
    obj_next.cnt    = cnt_next;
    int traj_length = 0;
    int res = this->planner_->MoveJ2L(jp_start, obj_target, obj_next, line_out, traj_length);

    switch (res) {
        case 0:
            fifo_length_    = traj_length;
            pose_previous   = line_out.pose_prv;
            pose_start      = line_out.pose;
            vu_start    = line_out.vu;
            v_start     = line_out.velocity;
            err     = SUCCESS;
            return true;
        case 1021:
            err = TARGET_REPEATED;
            return false;
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

/*
//------------------------------------------------------------------------------
// Function:    MoveJ2L
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:      jp_start    -> initial position and omega of six joints
//          pose_target -> joint target of the plan
//          v_ref       -> reference velocity
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
bool PlanningInterface::MoveJ2L(const JointPoint &jp_start,
                                const Pose &pose_target, double v_ref, int cnt,
                                const Pose &pose_next, double v_next, int cnt_next,
                                std::vector<JointValues> &planned_path,
                                Pose &pose_start, Pose &pose_previous,
                                double &v_start, double &vu_start,
                                ErrorCode &err) {
    v_next = v_next * velocity_scaling_;
    double v_percent = v_ref * velocity_scaling_ / velocity_range_.max * 100;
    if (v_percent > 100.0) {
        v_percent = 100.0;
    }

    int res = this->planner_->MoveJ2L_Cspace(jp_start,
                                        pose_target, v_percent, cnt,
                                        pose_next, v_next,
                                        planned_path,
                                        pose_start, v_start, vu_start,
                                        pose_previous);

    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1021:
            err = TARGET_REPEATED;
            return false;
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
*/

//------------------------------------------------------------------------------
// Function:    MoveJ2C
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:      jp_start    -> initial position and omega of six joints
//          j_target    -> joint target of the plan
//          v_ref       -> reference velocity
//          cnt         -> desired cnt value of the plan, range:0-100
//          pose2_circle-> the target pose of the next circle plan used for smoothing 
//          pose3_circle-> the target pose of the next circle plan used for smoothing 
// Out:     jp_end      -> the ending position and omega of six joints in this plan
//          planned_path-> planned path in joint space
//          error_code  -> error code
// Return:  true        -> plan successfully
//------------------------------------------------------------------------------
bool PlanningInterface::MoveJ2C(const JointPoint &jp_start,
                                const JointValues &j_target, double v_target, int cnt_target,
                                const Pose &pose1_circle, const Pose &pose2_circle, double v_circle, int cnt_next,
                                Pose &pose_start, double &v_start,
                                ErrorCode &err) {
    if (fifo_length_ >0) {err = INVALID_SEQUENCE; return false;}

    if (v_target > 100.0)  v_target = 100.0;
    v_circle = v_circle * velocity_scaling_;
    v_target = v_target * velocity_scaling_;

    JCommand obj_target;
    CCommand obj_next;
    CInitial circle_out;
    obj_target.v_percentage = v_target;
    obj_target.joints   = j_target;
    obj_target.cnt  = cnt_target;
    obj_next.velocity   = v_circle;
    obj_next.pose1  = pose1_circle;
    obj_next.pose2  = pose2_circle;
    obj_next.cnt    = cnt_next;
    int traj_length = 0;
    int res = this->planner_->MoveJ2C(jp_start, obj_target, obj_next, circle_out, traj_length);
    
    switch (res) {
        case 0:
            fifo_length_    = traj_length;
            pose_start  = circle_out.pose;
            v_start     = circle_out.velocity;
            err = SUCCESS;
            return true;
        case 1021:
            err = TARGET_REPEATED;
            return false;
        case 1031:
            err = AXIS_OVERSHOOT;
            return false;
        case 1032:
            err = AXIS_APPROACHING_LIMIT;
            return false;
        case 1053:
            err = CURVE_PRECISION_TOO_LOW;
            return false;
        case 1054:
            err = THREE_POINTS_COLINEAR;
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
//          pose_previous-> the previous pose used for interpolation
// Out:     planned_path-> planned path in cartesian space
//          jp          -> position and omega of six joints at the ending of this path
//          error_code  -> error code
// Return:  true  -> plan successfully
//------------------------------------------------------------------------------
bool PlanningInterface::MoveL2J(const Pose &pose_start, double v_start, double vu_start,
                                const Pose &pose_target, double v_target, int cnt_target,
                                Pose &pose_previous, ErrorCode &err) {
    if (fifo_length_ >0) {err = INVALID_SEQUENCE; return false;}

    v_target = v_target * velocity_scaling_;
    LInitial line_start;
    LCommand obj_target;
    line_start.velocity = v_start;
    line_start.pose_prv = pose_previous;
    line_start.pose = pose_start;
    line_start.vu   = vu_start;
    obj_target.pose = pose_target;
    obj_target.velocity = v_target;
    obj_target.cnt  = cnt_target;
    int traj_length = 0;
    int res = this->planner_->MoveL2J(line_start, obj_target, traj_length);

    switch (res) {
        case 0:
            fifo_length_ = traj_length;
            err = SUCCESS;
            return true;
        case 1021:
            err = TARGET_REPEATED;
            return false;
        case 1024:
            err = CUBIC_CURVE_PLANNING_FAILED;
            return false;
        case 1025:
            err = MOTION_SPEED_TOO_LOW;
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
//          pose_previous -> the previous pose used for interpolation
// Out:     pose_start  -> the end pose of this plan, also the initial pose of the next plan
//          v_start     -> the end velocity of this plan, also the initial pose of the next plan
//          vu_start    -> the end value of intermediate-variable in this plan, also the initial value in next plan
//          pose_previous -> the pose_previous in the next plan
//          planned_path-> planned path in cartesian space
//          error_code  -> error code
// Return:  true        -> plan successfully
//------------------------------------------------------------------------------
bool PlanningInterface::MoveL2L(Pose &pose_start, double &v_start, double &vu_start,
                                const Pose &pose_target, double v_target, int cnt_target,
                                const Pose &pose_next, double v_next, int cnt_next,
                                Pose &pose_previous, ErrorCode &err) {
    if (fifo_length_ >0) {err = INVALID_SEQUENCE; return false;}

    v_target = v_target * velocity_scaling_;
    v_next   = v_next   * velocity_scaling_;

    LCommand obj_target, obj_next;
    LInitial line_start, line_out;
    obj_target.velocity = v_target;
    obj_target.pose = pose_target;
    obj_target.cnt  = cnt_target;
    obj_next.velocity   = v_next;
    obj_next.pose   = pose_next;
    obj_next.cnt    = cnt_next;
    line_start.velocity = v_start;
    line_start.pose_prv = pose_previous;
    line_start.pose = pose_start;
    line_start.vu   = vu_start;
    int traj_length = 0;
    int res = this->planner_->MoveL2L(line_start, obj_target, obj_next, line_out, traj_length);

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
            fifo_length_    = traj_length;
            pose_previous   = line_out.pose_prv;
            pose_start  = line_out.pose;
            vu_start    = line_out.vu;
            v_start = line_out.velocity;
            err = SUCCESS;
            return true;
        case 1021:
            err = TARGET_REPEATED;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveL2C
// Summary: To plan a linear path to touch target pose, with/without smooth.
// In:      jp_start    -> initial position and omega of six joints
//          j_target    -> joint target of the plan
//          v_percent   -> percentage of velocity, range: 0.0-100.0
//          cnt         -> desired cnt value of the plan, range:0-100
//          pose2_circle-> the target pose of the next circle plan used for smoothing 
//          pose3_circle-> the target pose of the next circle plan used for smoothing 
// Out:     jp_end      -> the ending position and omega of six joints in this plan
//          planned_path-> planned path in joint space
//          error_code  -> error code
// Return:  true        -> plan successfully
//------------------------------------------------------------------------------
bool PlanningInterface::MoveL2C(Pose &pose_start, double &v_start, double &vu_start,
                                const Pose &pose_target, double v_target, int cnt_target,
                                const Pose &pose1_circle, const Pose &pose2_circle, double v_circle, int cnt_next,
                                Pose &pose_previous, ErrorCode &err) {
    if (fifo_length_ >0) {err = INVALID_SEQUENCE; return false;}

    v_target = v_target * velocity_scaling_;
    v_circle = v_circle * velocity_scaling_;

    LInitial line_start;
    CInitial circle_out;
    LCommand obj_target;
    CCommand obj_next;
    line_start.velocity = v_start;
    line_start.pose_prv = pose_previous;
    line_start.pose = pose_start;
    line_start.vu   = vu_start;
    obj_target.velocity = v_target;
    obj_target.pose = pose_target;
    obj_target.cnt  = cnt_target;
    obj_next.velocity   = v_circle;
    obj_next.pose1  = pose1_circle;
    obj_next.pose2  = pose2_circle;
    obj_next.cnt    = cnt_next;
    int traj_length = 0;
    int res = this->planner_->MoveL2C(line_start, obj_target, obj_next, circle_out, traj_length);

    switch (res) {
        case 0:
            fifo_length_ = traj_length;
            pose_start   = circle_out.pose;
            v_start = circle_out.velocity;
            err     = SUCCESS;
            return true;
        case 1021:
            err = TARGET_REPEATED;
            return false;
        case 1053:
            err = CURVE_PRECISION_TOO_LOW;
            return false;
        case 1054:
            err = THREE_POINTS_COLINEAR;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}

/*
bool PlanningInterface::MoveL2CAdditionSmooth(const JointPoint &jp_start, const Pose &pose_start,
                                              const Pose &pose_start_past, vector<JointValues> &planned_path,
                                              ErrorCode &err) {
    int res = planner_->MoveL2C_part2(jp_start, pose_start, pose_start_past, planned_path);
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1003:
            err = IK_EXCESSIVE_DISTANCE;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}*/


bool PlanningInterface::MoveC2J(const Pose &pose_start, double v_start,
                                const Pose &pose1, const Pose &pose2, double v_target, int cnt_target,
                                ErrorCode err) {
    if (fifo_length_ >0) {err = INVALID_SEQUENCE; return false;}

    v_target = v_target * velocity_scaling_;
    CInitial circle_start;
    CCommand obj_target;
    circle_start.velocity   = v_start;
    circle_start.pose   = pose_start;
    obj_target.velocity = v_target;
    obj_target.pose1    = pose1;
    obj_target.pose2    = pose2;
    obj_target.cnt  = cnt_target;
    int traj_length = 0;
    int res = this->planner_->MoveC2J(circle_start, obj_target, traj_length);

    switch (res) {
        case 0:
            fifo_length_ = traj_length;
            err = SUCCESS;
            return true;
        case 1053:
            err = CURVE_PRECISION_TOO_LOW;
            return false;
        case 1054:
            err = THREE_POINTS_COLINEAR;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
        return false;
    }
}


bool PlanningInterface::MoveC2L(Pose &pose_start, double &v_start,
                                const Pose &pose1, const Pose &pose2, double v_target, int cnt_target,
                                const Pose &pose_next, double v_next, int cnt_next,
                                Pose &pose_previous, double &vu_start, ErrorCode &err) {
    if (fifo_length_ >0) {err = INVALID_SEQUENCE; return false;}

    v_target = v_target * velocity_scaling_;
    v_next   = v_next   * velocity_scaling_;
    CInitial circle_start;
    CCommand obj_target;
    LCommand obj_next;
    LInitial line_out;
    circle_start.velocity   = v_start;
    circle_start.pose   = pose_start;
    obj_target.velocity = v_target;
    obj_target.pose1    = pose1;
    obj_target.pose2    = pose2;
    obj_target.cnt      = cnt_target;
    obj_next.velocity   = v_next;
    obj_next.pose   = pose_next;
    obj_next.cnt    = cnt_next;
    int traj_length = 0;
    int res = this->planner_->MoveC2L(circle_start, obj_target, obj_next, line_out, traj_length);
    
    switch (res) {
        case 0:
            fifo_length_    = traj_length;
            pose_previous   = line_out.pose_prv;
            pose_start  = line_out.pose;
            vu_start    = line_out.vu;
            v_start     = line_out.velocity;
            err = SUCCESS;
            return true;
        case 1053:
            err = CURVE_PRECISION_TOO_LOW;
            return false;
        case 1054:
            err = THREE_POINTS_COLINEAR;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}

/*
bool PlanningInterface::MoveC2LAdditionSmooth(const JointPoint &jp_start, const Pose &pose_start,
                                              const Pose &pose_start_past, std::vector<JointValues> &planned_path,
                                              ErrorCode &err) {
    int res = planner_->MoveC2L_part2(jp_start, pose_start, pose_start_past, planned_path);
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1003:
            err = IK_EXCESSIVE_DISTANCE;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}*/


bool PlanningInterface::MoveC2C(Pose &pose_start, double &v_start,
                                const Pose &pose1_target, const Pose &pose2_target, double v_target, int cnt_target,
                                const Pose &pose1_next, const Pose &pose2_next, double v_next, int cnt_next,
                                ErrorCode &err) {
    if (fifo_length_ >0) {err = INVALID_SEQUENCE; return false;}

    v_target = v_target * velocity_scaling_;
    v_next   = v_next   * velocity_scaling_;

    CInitial circle_start, circle_out;
    CCommand obj_target, obj_next;
    circle_start.velocity   = v_start;
    circle_start.pose   = pose_start;
    obj_target.velocity = v_target;
    obj_target.pose1    = pose1_target;
    obj_target.pose2    = pose2_target;
    obj_target.cnt      = cnt_target;
    obj_next.velocity   = v_next;
    obj_next.pose1  = pose1_next;
    obj_next.pose2  = pose2_next;
    obj_next.cnt    = cnt_next;
    int traj_length = 0;
    int res = this->planner_->MoveC2C(circle_start, obj_target, obj_next, circle_out, traj_length);

    switch (res) {
        case 0:
            fifo_length_ = traj_length;
            pose_start   = circle_out.pose;
            v_start = circle_out.velocity;
            err = SUCCESS;
            return true;
        case 1053:
            err = CURVE_PRECISION_TOO_LOW;
            return false;
        case 1054:
            err = THREE_POINTS_COLINEAR;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}

/*
bool PlanningInterface::MoveC2CAdditionSmooth(const JointPoint &jp_start, const Pose &pose_start,
                                              const Pose &pose_start_past, std::vector<JointValues> &planned_path,
                                              ErrorCode &err) {
    int res = planner_->MoveC2C_part2(jp_start, pose_start, pose_start_past, planned_path);
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        case 1003:
            err = IK_EXCESSIVE_DISTANCE;
            return false;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}*/

bool PlanningInterface::pickPoints(vector<Pose> &points, ErrorCode &err) {
    points.clear();
    if (fifo_length_ <= 0) {err = MOTION_INTERNAL_FAULT; return false;}
    
    //std::cout << "call planner_->PickPoint()" << std::endl;
    int res = this->planner_->PickPoint(points);
    //std::cout << "planner_->PickPoint() returned" << std::endl;
    switch (res) {
        case 0:
            fifo_length_ -= points.size();
            err = SUCCESS;
            return true;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}

bool PlanningInterface::pickPoints(vector<JointValues> &points, ErrorCode &err) {
    points.clear();
    if (fifo_length_ <= 0) {err = MOTION_INTERNAL_FAULT; return false;}

    //std::cout << "call planner_->PickPoint()" << std::endl;
    int res = this->planner_->PickPoint(points);
    //std::cout << "planner_->PickPoint() returned" << std::endl;
    switch (res) {
        case 0:
            fifo_length_ -= points.size();
            err = SUCCESS;
            return true;
        case 1021:
            err = TARGET_REPEATED;
            return false;
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

int PlanningInterface::estimateFIFOLength(JointValues joint1, JointValues joint2) {
    return this->planner_->EstimateFIFOlength(joint1, joint2);
}

bool PlanningInterface::replanPauseTrajectory(std::vector<JointValues> &trajectory, ErrorCode &err) {
    int res = this->planner_->Pause(trajectory);
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}

bool PlanningInterface::replanRestartTrajectory(std::vector<JointValues> &trajectory,
                                                JointValues &start_point,
                                                ErrorCode &err) {
    int res = this->planner_->Restart(trajectory, start_point);
    switch (res) {
        case 0:
            err = SUCCESS;
            return true;
        default:
            err = MOTION_INTERNAL_FAULT;
            return false;
    }
}

bool PlanningInterface::isPointCoincident(const Pose &pose1, const Pose &pose2) {
    if (0 == this->planner_->Check_coincidence_c(pose1, pose2))
        // two points are not coincident
        return false;
    else
        // two points are coincident
        return true;
}

bool PlanningInterface::isPointCoincident(const Pose &pose, const JointValues &joint) {
    if (0 == this->planner_->Check_coincidence_jc(joint, pose))
        // two points are not coincident
        return false;
    else
        // two points are coincident
        return true;
}

bool PlanningInterface::isPointCoincident(const JointValues &joint, const Pose &pose) {
    if (0 == this->planner_->Check_coincidence_jc(joint, pose))
        // two points are not coincident
        return false;
    else
        // two points are coincident
        return true;
}

bool PlanningInterface::isPointCoincident(const JointValues &joint1, const JointValues &joint2) {
    if (0 == this->planner_->Check_coincidence_j(joint1, joint2))
        // two points are not coincident
        return false;
    else
        // two points are coincident
        return true;
}



}


