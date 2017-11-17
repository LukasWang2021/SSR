/*************************************************************************
	> File Name: motion_controller_planning_interface.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年02月20日 星期一 09时59分15秒
 ************************************************************************/

#include <motion_controller/motion_controller_planning_interface.h>
#include <motion_controller/motion_controller_arm_group.h>

using std::cout;
using std::endl;
using std::vector;
using std::string;

namespace fst_controller {

PlanningInterface::PlanningInterface()
{
    planner_    = NULL;
    motion_list_front_  = NULL;
    motion_list_back_   = NULL;
    picking_command_    = NULL;
    
    initial_joint_valid_ = false;
    memset(&tool_frame_, 0, sizeof(tool_frame_));
    memset(&user_frame_, 0, sizeof(user_frame_));
    memset(&dh_parameter_, 0, sizeof(dh_parameter_));
    memset(&joint_constraint_, 0, sizeof(joint_constraint_));

    trajectory_segment_length_range_.min = 50;
    trajectory_segment_length_range_.max = 5000;
    cycle_time_range_.min   = 0.0005;       // s
    cycle_time_range_.max   = 0.05;         // s
    velocity_range_.min     = 0.1;          // mm/s
    velocity_range_.max     = 4000;    // mm/s
    acceleration_range_.min = 10;   // mm/s^2
    acceleration_range_.max = 16000;   // mm/s^2
    velocity_scaling_range_.min = 0.1;
    velocity_scaling_range_.max = 100.0;
    acceleration_scaling_range_.min = 0.1;
    acceleration_scaling_range_.max = 100;
    jerk_range_.min = 5;
    jerk_range_.max = 50; 
    joint_overshoot_range_.min  = 0.001;        // 0.001 rad
    joint_overshoot_range_.max  = 1.571;        // Pi/2 rad
    //joint_errorangle_range_.min = 0.000;        // 0 rad
    //joint_errorangle_range_.max = 0.524;        // Pi/6 rad
    omega_overload_range_.min  = 0.1;
    omega_overload_range_.max  = 120.0;
    alpha_overload_range_.min = 100.0;
    alpha_overload_range_.max = 300.0;
    smooth_radius_coefficient_range_.min = 1.0;
    smooth_radius_coefficient_range_.max = 5.0;

    trajectory_segment_length_ = 500;
    cycle_time_ = 0.001;
    velocity_   = 500;
    acceleration_   = 5000;
    velocity_scaling_       = 100.0;
    acceleration_scaling_   = 100.0;
    jerk_   = 10;
    joint_overshoot_    = 0.25;
    //joint_errorangle_   = 0.0;
    omega_overload_     = 100.0;
    alpha_overload_     = 100.0;
    smooth_radius_coefficient_ = 1.0;
    curve_mode_  = T_CURVE;
    smooth_mode_ = MODE_VELOCITY;
}

bool PlanningInterface::initPlanningInterface(fst_parameter::ParamValue &params, ErrorCode &err)
{
    if (planner_ != NULL) {delete planner_; planner_ = NULL;}
    planner_ = new fst_controller::TrajPlan();
    if (planner_ == NULL) {err = MOTION_FAIL_IN_INIT; return false;}
    while (motion_list_front_) deleteFirstMotionCommand();

    try {
        if (setTrajectorySegmentLength(params["trajectory_segment_length"]) &&
            setCycleTime(params["cycle_time"]) &&
            //setVelocity(params["velocity"]) &&
            setAcceleration(params["acceleration"]) &&
            setVelocityScaling(params["velocity_scaling"]) &&
            setAccelerationScaling(params["acceleration_scaling"]) &&
            setJerk(params["jerk"]) &&
            setJointOvershoot(params["joint_overshoot"]) &&
            //setJointErrorAngle(params["joint_errorangle"]) &&
            //setOmegaOverload(params["omega_overload"]) &&
            //setAlphaOverload(params["alpha_overload"]) &&
            setSmoothRadiusCoefficient(params["smooth_radius_coefficient"]))
        {
            setCurveMode(params["smooth_curve_mode"] == "T_CURVE" ? T_CURVE : S_CURVE);
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

PlanningInterface::~PlanningInterface()
{
    while (motion_list_front_) deleteFirstMotionCommand();
    delete planner_;
    planner_ = NULL;
}

string PlanningInterface::getAlgorithmVersion(void)
{
    return planner_->getVersion();
}

double PlanningInterface::getCycleTime(void)
{
    return cycle_time_;
}

/*
double PlanningInterface::getVelocity(void)
{
    return velocity_;
}
*/

double PlanningInterface::getAcceleration(void)
{
    return acceleration_;
}

double PlanningInterface::getVelocityScaling(void)
{
    return velocity_scaling_;
}

double PlanningInterface::getAccelerationScaling(void)
{
    return acceleration_scaling_;
}

double PlanningInterface::getJerk(void)
{
    return jerk_;
}

double PlanningInterface::getJointOvershoot(void)
{
    return joint_overshoot_;
}

/*
double PlanningInterface::getJointErrorAngle(void)
{
    return joint_errorangle_;
}
*/

double PlanningInterface::getOmegaOverload(void)
{
    return omega_overload_;
}

double PlanningInterface::getAlphaOverload(void)
{
    return alpha_overload_;
}

double PlanningInterface::getSmoothRadiusCoefficient(void)
{
    return smooth_radius_coefficient_;
}

CurveMode PlanningInterface::getCurveMode(void)
{
    return curve_mode_;
}

const JointConstraint& PlanningInterface::getJointConstraint(void)
{
    return joint_constraint_;
}

const DHGroup& PlanningInterface::getDH(void)
{
    return dh_parameter_;
}

const Transformation& PlanningInterface::getToolFrame(void)
{
    return tool_frame_;
}

const Transformation& PlanningInterface::getUserFrame(void)
{
    return user_frame_;
}

void PlanningInterface::clearMotionList(void)
{
    while (motion_list_back_)   deleteMotionCommand(motion_list_back_);
    picking_command_ = NULL;
}

unsigned int PlanningInterface::getTrajectorySegmentLength(void)
{
    return trajectory_segment_length_;
}

int PlanningInterface::getAllCommandLength(void)
{
    int length = 0;
    MoveCommand *cmd = picking_command_;

    while (cmd) {
        length += cmd->getTrajLength() - cmd->getPickedlength();
        cmd = cmd->next;
    }
    return length;
}

bool PlanningInterface::setTrajectorySegmentLength(int length)
{
    if (length < trajectory_segment_length_range_.min || length > trajectory_segment_length_range_.max) {
        return false;
    }
    else {
        this->trajectory_segment_length_ = length;
        this->planner_->setVlength(length);
        return true;
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
bool PlanningInterface::setCycleTime(double cycle_time)
{
    if (cycle_time < cycle_time_range_.min || cycle_time > cycle_time_range_.max) {
        return false;
    }
    else {
        this->cycle_time_ = cycle_time;
        this->planner_->setCycleTime(cycle_time);
        return true;
    }
}

/*
bool PlanningInterface::setVelocity(double vel)
{
    if (vel < velocity_range_.min || vel > velocity_range_.max) {
        return false;
    }
    else {
        this->velocity_ = vel;
        return true;
    }
}
*/

//------------------------------------------------------------------------------
// Function:    setAcceleration
// Summary: To set the algorithm acceleration in cartesian space.
// In:      acce    -> desired acceleration
// Out:     None
// Return:  true    -> acceleration changed to new value
//          false   -> acceleration NOT changed
//------------------------------------------------------------------------------
bool PlanningInterface::setAcceleration(double acce)
{
    if (acce < acceleration_range_.min || acce > acceleration_range_.max) {
        return false;
    }
    else {
        this->acceleration_ = acce;
        this->planner_->setAcceleration(acce * acceleration_scaling_ / 100);
        return true;
    }
}

bool PlanningInterface::setVelocityScaling(double percent)
{
    if (percent < velocity_scaling_range_.min || percent > velocity_scaling_range_.max) {
        return false;
    }
    else {
        this->velocity_scaling_ = percent;
        planner_->setOverallSpeedRatio(percent / 100);      // percent to scaling
        return true;
    }
}

bool PlanningInterface::setAccelerationScaling(double percent)
{
    if (percent < acceleration_scaling_range_.min || percent > acceleration_scaling_range_.max) {
        return false;
    }
    else {
        this->acceleration_scaling_ = percent;
        this->planner_->setAcceleration(acceleration_ * percent / 100);     // percent to scaling
        return true;
    }
}

bool PlanningInterface::setJerk(double jerk)
{
    if (jerk < jerk_range_.min || jerk > jerk_range_.max) {
        return false;
    }
    else {
        this->jerk_ = jerk;
        this->planner_->setJaratio(jerk);
        return true;
    }
}

bool PlanningInterface::setJointOvershoot(double angle)
{
    if (angle < joint_overshoot_range_.min || angle > joint_overshoot_range_.max) {
        return false;
    }
    else {
        this->joint_overshoot_ = angle;
        this->planner_->setOvershoot(angle);
        return true;
    }
}

/*
bool PlanningInterface::setJointErrorAngle(double angle)
{
    if (angle < joint_errorangle_range_.min || angle > joint_errorangle_range_.max) {
        return false;
    }
    else {
        this->joint_errorangle_ = angle;
        this->planner_->seterrorangle(angle);
        return true;
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
bool PlanningInterface::setOmegaOverload(double percent)
{
    if (value < omega_overload_range_.min || value > omega_overload_range_.max) {
        return false;
    }
    else {
        this->omega_overload_ = value;
        this->planner_->setLimitScale(percent / 100);
        return true;
    }
}

bool PlanningInterface::setAlphaOverload(double value)
{
    if (value < alpha_overload_range_.min || value > alpha_overload_range_.max) {
        return false;
    }
    else {
        this->alpha_overload_ = value;
        this->planner_->setoverload(percent / 100);
        return true;
    }
}*/

bool PlanningInterface::setSmoothRadiusCoefficient(double coeff)
{
    if (coeff < smooth_radius_coefficient_range_.min || coeff > smooth_radius_coefficient_range_.max) {
        return false;
    }
    else {
        this->smooth_radius_coefficient_ = coeff;
        this->planner_->setRadiusCoefficient(coeff);
        return true;
    }
}

void PlanningInterface::setCurveMode(CurveMode mode)
{
    this->curve_mode_ = mode;
    this->planner_->setCurveMode(mode);
}

//------------------------------------------------------------------------------
// Function:    setJointConstraint
// Summary: To set joint constraint in Kinematics algorithm.
// In:      constraint -> joint constraint
// Out:     None
// Return:  true        -> joint constraint changed to new values
//          false       -> joint constraint NOT changed
//------------------------------------------------------------------------------
void PlanningInterface::setJointConstraint(const JointConstraint &constraint)
{
    joint_constraint_ = constraint;
    this->planner_->setAxisLimit(constraint);
}

bool PlanningInterface::setAlphaScaling(double percent)
{
    if (percent < 0.0 || percent > 100.0) {
        return false;
    }
    else {
        percent = percent / 100.0;
        JointConstraint temp_constraint = joint_constraint_;
        temp_constraint.j1.max_alpha *= percent;
        temp_constraint.j2.max_alpha *= percent;
        temp_constraint.j3.max_alpha *= percent;
        temp_constraint.j4.max_alpha *= percent;
        temp_constraint.j5.max_alpha *= percent;
        temp_constraint.j6.max_alpha *= percent;
        this->planner_->setAxisLimit(temp_constraint);
        return true;
    }
}

void PlanningInterface::setDH(const DHGroup &dh)
{
    dh_parameter_ = dh;
    this->planner_->setDH(dh_parameter_);
}

void PlanningInterface::setToolFrame(const Transformation &tool_frame)
{
    //PoseEuler pose;
    //memcpy(&pose, &tool_frame, sizeof(PoseEuler));
    //this->planner_->setToolFrame(pose);
    this->tool_frame_ = tool_frame;
    this->planner_->setToolFrame(tool_frame);
}

void PlanningInterface::setUserFrame(const Transformation &user_frame)
{
    //PoseEuler   pose;
    //memcpy(&pose, &user_frame, sizeof(PoseEuler));
    //this->planner_->setUserFrame(pose);
    this->user_frame_ = user_frame;
    this->planner_->setUserFrame(user_frame);
}

void PlanningInterface::setInitialJoint(const Joint &joint)
{
    this->initial_joint_ = joint;
    initial_joint_valid_ = true;
}

//------------------------------------------------------------------------------
// Function:    transformPoseEuler2Pose
// Summary: To transform a poseEuler point to a pose point.
// In:      poes_e -> the poseEuler to be transformed
// Out:     None
// Return:  pose point
//------------------------------------------------------------------------------
Pose PlanningInterface::transformPoseEuler2Pose(const PoseEuler &pose_e)
{
    Pose    pose;
    memset(&pose, 0, sizeof(pose));
    this->planner_->Euler2Quatern(pose_e, pose);
    return pose;
}

//------------------------------------------------------------------------------
// Function:    transformPose2PoseEuler
// Summary: To transform a pose point to a poseEuler point.
// In:      poes -> the pose to be transformed
// Out:     None
// Return:  poseEuler point
//------------------------------------------------------------------------------
PoseEuler PlanningInterface::transformPose2PoseEuler(const Pose &pose)
{
    PoseEuler   pose_e;
    memset(&pose_e, 0, sizeof(pose_e));
    this->planner_->Quatern2Euler(pose, pose_e);
    return pose_e;
}

bool PlanningInterface::getJointFromPose(const  Pose &pose,
                                         const  Joint &reference,
                                         Joint  &joint,
                                         double time_interval,
                                         ErrorCode  &err)
{
    computeInverseKinematics(pose, reference, joint, err);
    if (err == SUCCESS) {
        return true;
    }
    else if (err == IK_EXCESSIVE_DISTANCE) {
        if (fabs(reference.j1 - joint.j1) < joint_constraint_.j1.max_omega * time_interval &&
            fabs(reference.j2 - joint.j2) < joint_constraint_.j2.max_omega * time_interval &&
            fabs(reference.j3 - joint.j3) < joint_constraint_.j3.max_omega * time_interval &&
            fabs(reference.j4 - joint.j4) < joint_constraint_.j4.max_omega * time_interval &&
            fabs(reference.j5 - joint.j5) < joint_constraint_.j5.max_omega * time_interval &&
            fabs(reference.j6 - joint.j6) < joint_constraint_.j6.max_omega * time_interval)
        {
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
                                                 const Joint &joint_reference,
                                                 Joint &joint_result,
                                                 ErrorCode &err)
{
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
bool PlanningInterface::computeForwardKinematics(const Joint &joint,
                                                 Pose &pose,
                                                 ErrorCode &err)
{
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

bool PlanningInterface::computeForwardKinematicsWorld(const Joint &joint,
                                                      PoseEuler &flange,
                                                      PoseEuler &tcp,
                                                      ErrorCode &err)
{
    int res = this->planner_->ForwardKinematics_world(joint, flange, tcp);

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

MoveCommand* PlanningInterface::createMotionCommand(
                int id,
                MotionTarget &target,
                MotionTarget &next,
                ErrorCode &err)
{
    if (!checkMotionTarget(target) || !checkMotionTarget(next)) {
        err = INVALID_PARAMETER;
        return NULL;
    }

    MoveCommand *new_motion = new MoveCommand(planner_, target, next, id);
    if (new_motion == NULL) {err = MOTION_INTERNAL_FAULT; return NULL;}
    //cout << "motion address=" << new_motion << endl;



    if (motion_list_back_ == NULL) {
        // empty command list, insert the first command object
        new_motion->next = NULL;
        new_motion->prev = NULL;
        picking_command_ = new_motion;
        motion_list_front_  = new_motion;
        motion_list_back_   = new_motion;
        new_motion->setInitialJoint(initial_joint_);
    }
    else {
        // insert the command into the end of list
        new_motion->next        = NULL;
        new_motion->prev        = motion_list_back_;
        motion_list_back_->next = new_motion;
        motion_list_back_       = new_motion;
        if (picking_command_ == NULL)   picking_command_ = new_motion;
    }

    MoveCommand *cmd = motion_list_front_;
    int result = new_motion->PlanTraj();
    switch (result) {
        case SUCCESS:
            cout << "command list:";
            while (cmd) {cout << cmd << " <-> "; cmd = cmd->next;}
            cout << "NULL" << endl;
            if (new_motion->getTrajLength() == 0)
                initial_joint_valid_ = true;
            err = SUCCESS;
            return new_motion;
        case 1021:
            err = TARGET_REPEATED;
            break;
        case 1024:
            err = CUBIC_CURVE_PLANNING_FAILED;
            break;
        case 1025:
            err = MOTION_SPEED_TOO_LOW;
            break;
        case 1027:
            err = S_CURVE_PLANNING_FAILED;
            break;
        case 1029:
            err = INVALID_CNT;
            break;
        case 1031:
            err = AXIS_OVERSHOOT;
            break;
        case 1032:
            err = AXIS_APPROACHING_LIMIT;
            break;
        case 1033:
            err = AXIS_INCONSISTENT;
            break;
        case 1053:
            err = CURVE_PRECISION_TOO_LOW;
            break;
        case 1054:
            err = THREE_POINTS_COLINEAR;
            break;
        case 1055:
            err = PLANNING_MAJOR_ARC;
            break;
        default:
            err = MOTION_INTERNAL_FAULT;
            break;
    }

    deleteLastMotionCommand();
    return NULL;
}

void PlanningInterface::deleteFirstMotionCommand(void)
{
    deleteMotionCommand(motion_list_front_);
}

void PlanningInterface::deleteLastMotionCommand(void)
{
    deleteMotionCommand(motion_list_back_);
}

void PlanningInterface::deleteMotionCommandBefore(MoveCommand *command)
{
    MoveCommand *cmd = motion_list_back_;
    while (cmd && cmd != command) cmd = cmd->prev;
    if (cmd) {
        MoveCommand *cmd_to_del = cmd;
        while (cmd_to_del) {
            cmd = cmd->prev;
            deleteMotionCommand(cmd_to_del);
            cmd_to_del = cmd;
        }
        return;
    }
}

void PlanningInterface::deleteMotionCommand(MoveCommand *command)
{
    if (command != NULL) {
        if (command == motion_list_front_ && command == motion_list_back_) {
            // only one command in command list, delete it
            motion_list_front_  = NULL;
            motion_list_back_   = NULL;
        }
        else if (command == motion_list_front_) {
            // two or more commands in the list, delete the first one
            motion_list_front_ = motion_list_front_->next;
            motion_list_front_->prev = NULL;
        }
        else if (command == motion_list_back_) {
            // two or more commands in the list, delete the last one
            motion_list_back_ = motion_list_back_->prev;
            motion_list_back_->next = NULL;
        }
        else {
            // three or more commands in the list, delete the command given by pointer
            command->prev->next = command->next;
            command->next->prev = command->prev;
        }
        delete command;
        return;
    }
}

bool PlanningInterface::isMotionCommandListEmpty(void)
{
    return motion_list_front_ == NULL;
}


ErrorCode PlanningInterface::pickPoints(vector<PathPoint> &points)
{
    points.clear();
    if (picking_command_ != NULL) {
        int res = picking_command_->PickPoint(points);
        if (res == 1031)        return AXIS_OVERSHOOT;
        else if (res == 1032)   return AXIS_APPROACHING_LIMIT;
        else if (res != 0)      return MOTION_INTERNAL_FAULT;

        if (points.size() > 0) {
            /*
            vector<PathPoint>::iterator it = points.begin();
            for (; it != points.end(); ++it)  cout << it->type << " ";
            cout << endl;
            */
            if (points.back().level == POINT_ENDING) {
                picking_command_ = picking_command_->next;
                initial_joint_valid_ = false;
            }
            else if (points.back().level == POINT_LAST) {
                picking_command_ = picking_command_->next;
            }
            return SUCCESS;
        }
        else {
            if (picking_command_->getTrajLength() == 0) {
                // is a empty command, target repeated?
                //printf("PlanningInterface: insert a dummy point\n");
                PathPoint pp;
                pp.type   = MOTION_NONE;
                pp.stamp  = 0;
                pp.level  = POINT_ENDING;
                pp.source = picking_command_;

                points.push_back(pp);
                picking_command_ = picking_command_->next;
                /*
                while (picking_command_) {
                    int res = picking_command_->PickPoint(points);
                    if (res == 1031)        return AXIS_OVERSHOOT;
                    else if (res == 1032)   return AXIS_APPROACHING_LIMIT;
                    else if (res != 0)      return MOTION_INTERNAL_FAULT;
                    
                    if (points.size() > 0) {
                        if (points.back().level == POINT_ENDING) {
                            picking_command_ = picking_command_->next;
                            initial_joint_valid_ = false;
                        }
                        else if (points.back().level == POINT_LAST) {
                            picking_command_ = picking_command_->next;
                        }
                        break;
                    }
                    else {
                        picking_command_ = picking_command_->next;
                        continue;
                    }
                }
                */
            }

            if (points.size() > 0)
                return SUCCESS;
            else 
                return INVALID_SEQUENCE;     // undefine error code;
        }
    }
    else {
        return INVALID_SEQUENCE;
    }
}

int PlanningInterface::estimateFIFOLength(Joint joint1, Joint joint2)
{
    return this->planner_->EstimateFIFOlength(joint1, joint2);
}

void PlanningInterface::displayMotionList(void)
{
    MoveCommand *cmd = motion_list_front_;
    cout << "Motion list: ";
    while (cmd != NULL) {cout << cmd << "->"; cmd = cmd->next;}
    cout << "NULL" << endl;
}

void PlanningInterface::setPickPosition(const JointPoint &point)
{
    if (point.level == POINT_ENDING || point.level == POINT_LAST) {
        picking_command_ = point.source->next;
        MoveCommand *cmd = picking_command_;
        while (cmd != NULL) {cmd->setPickedlength(0); cmd = cmd->next;}
    }
    else {
        picking_command_ = point.source;
        picking_command_->setPickedlength(point.stamp + 1);
        MoveCommand *cmd = picking_command_->next;
        while (cmd != NULL) {cmd->setPickedlength(0); cmd = cmd->next;}
    }
}

bool PlanningInterface::replanPauseTrajectory(vector<JointPoint> &traj, JointPoint &joint_end)
{
    vector<JointPoint> new_traj;
    int res = this->planner_->Pause(traj, new_traj, joint_end);
    
    if (res == 0) {
        traj.assign(new_traj.begin(), new_traj.end());
        return true;
    }
    else {
        return false;
    }
}

bool PlanningInterface::resumeFromPause(const Joint &pause_joint,
                                        const JointPoint &point,
                                        vector<JointPoint> &traj,
                                        ErrorCode &err)
{
    traj.clear();
    int res = 0;
    vector<JointPoint> new_traj;
    // point.source->setPickedlength(point.stamp + 1);
    if (initial_joint_valid_) {
        initial_joint_valid_ = false;
        res = point.source->Resume(initial_joint_, point.joint, new_traj);
    }
    else {
        res = point.source->Resume(pause_joint, point.joint, new_traj);
    }

    if (res == 0) {
        MoveCommand *cmd = point.source->next;
        while (cmd != NULL && res == 0) {
            res = cmd->PlanTraj();
            cmd = cmd->next;
        }
    }

    if (res == 0) {
        traj.assign(new_traj.begin(), new_traj.end());
        
        if (traj.size() == 0 || traj.back().level == POINT_LAST || traj.back().level == POINT_ENDING) {
            picking_command_ = point.source->next;
        }

        err = SUCCESS;
        return true;
    }
    else {
        switch (res) {
            case 1021:
                err = TARGET_REPEATED;
                return false;
            case 1024:
                err = CUBIC_CURVE_PLANNING_FAILED;
                return false;
            case 1025:
                err = MOTION_SPEED_TOO_LOW;
                return false;
            case 1027:
                err = S_CURVE_PLANNING_FAILED;
                return false;
            case 1029:
                err = INVALID_CNT;
                return false;
            case 1031:
                err = AXIS_OVERSHOOT;
                return false;
            case 1032:
                err = AXIS_APPROACHING_LIMIT;
                return false;
            case 1033:
                err = AXIS_INCONSISTENT;
                return false;
            case 1053:
                err = CURVE_PRECISION_TOO_LOW;
                return false;
            case 1054:
                err = THREE_POINTS_COLINEAR;
                return false;
            case 1055:
                err = PLANNING_MAJOR_ARC;
                return false;
            default:
                err = MOTION_INTERNAL_FAULT;
                return false;
        }
    }
}

bool PlanningInterface::resumeFromEstop(const JointPoint &point, vector<JointPoint> &traj, ErrorCode &err)
{
    traj.clear();
    int res;
    vector<JointPoint> new_traj;
    // point.source->setPickedlength(point.stamp);
    cout << "planning interface report:" << endl;
    if (initial_joint_valid_) {
        initial_joint_valid_ = false;
        cout << "&Obj=" << point.source << ", call resume" << endl;
        res = point.source->Resume(initial_joint_, point.joint, new_traj);
        cout << "resume returned, res=" << res << " traj length=" << new_traj.size() << endl;
    }
    else {
        err = INVALID_SEQUENCE;
        return false;
    }

    if (res == 0) {
        traj.assign(new_traj.begin(), new_traj.end());
        
        vector<JointPoint>::iterator it = new_traj.begin();
        cout << "id in traj points: ";
        for (; it != new_traj.end(); ++it) {
            cout << it->source->getMotionID() << " ";
        }
        cout << endl;
        
        if (traj.size() == 0 || traj.back().level == POINT_LAST || traj.back().level == POINT_ENDING) {
            picking_command_ = point.source->next;
        }

        err = SUCCESS;
        return true;
    }
    else {
        switch (res) {
            case 1021:
                err = TARGET_REPEATED;
                return false;
            case 1024:
                err = CUBIC_CURVE_PLANNING_FAILED;
                return false;
            case 1025:
                err = MOTION_SPEED_TOO_LOW;
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
            case 1055:
                err = PLANNING_MAJOR_ARC;
                return false;
            default:
                err = MOTION_INTERNAL_FAULT;
                return false;
        }
    }
}

bool PlanningInterface::isMotionExecutable(MotionType motion_t)
{
    if (motion_list_back_) {
        SmoothType smooth_t = motion_list_back_->getSmoothType();
        if (smooth_t != SMOOTH_NONE) {
            if (motion_t == MOTION_LINE   && smooth_t != SMOOTH_2L ||
                motion_t == MOTION_JOINT  && smooth_t != SMOOTH_2J ||
                motion_t == MOTION_CIRCLE && smooth_t != SMOOTH_2C)
            {
                return false;
            }
        }
    }
    return true;
}

bool PlanningInterface::isPointCoincident(const Pose &pose1, const Pose &pose2)
{
    if (0 == this->planner_->Check_coincidence(pose1, pose2))
        // two points are not coincident
        return false;
    else
        // two points are coincident
        return true;
}

bool PlanningInterface::isPointCoincident(const Pose &pose, const Joint &joint)
{
    if (0 == this->planner_->Check_coincidence(joint, pose))
        // two points are not coincident
        return false;
    else
        // two points are coincident
        return true;
}

bool PlanningInterface::isPointCoincident(const Joint &joint, const Pose &pose)
{
    if (0 == this->planner_->Check_coincidence(joint, pose))
        // two points are not coincident
        return false;
    else
        // two points are coincident
        return true;
}

bool PlanningInterface::isPointCoincident(const Joint &joint1, const Joint &joint2)
{
    if (0 == this->planner_->Check_coincidence(joint1, joint2))
        // two points are not coincident
        return false;
    else
        // two points are coincident
        return true;
}

bool PlanningInterface::checkMotionTarget(const MotionTarget &target)
{
    if (target.type == MOTION_LINE) {
        if (smooth_mode_ == MODE_VELOCITY && (target.cnt < 0 || target.cnt > 100))
            return false;
        else if (smooth_mode_ == MODE_DISTANCE && (target.smooth_radius < 0 || target.smooth_radius > 300))
            return false;
        if (target.linear_velocity < velocity_range_.min || target.linear_velocity > velocity_range_.max)
            return false;

        Joint joint_ref, joint_out;
        memset(&joint_ref, 0, sizeof(Joint));
        ErrorCode err;
        if (!getJointFromPose(target.pose_target, joint_ref, joint_out, 2.0, err)) {
            return false;
        }
    }
    else if (target.type == MOTION_JOINT) {
        if (smooth_mode_ == MODE_VELOCITY && (target.cnt < 0 || target.cnt > 100))
            return false;
        else if (smooth_mode_ == MODE_DISTANCE && (target.smooth_radius < 0 || target.smooth_radius > 300))
            return false;
        if (target.percent_velocity < 0.0 || target.percent_velocity > 100.0)
            return false;
        if (!checkJointBoundry(target.joint_target))
            return  false;
    }
    else if (target.type == MOTION_CIRCLE) {
        if (smooth_mode_ == MODE_VELOCITY && (target.cnt < 0 || target.cnt > 100))
            return false;
        else if (smooth_mode_ == MODE_DISTANCE && (target.smooth_radius < 0 || target.smooth_radius > 300))
            return false;
        if (target.linear_velocity < velocity_range_.min || target.linear_velocity > velocity_range_.max)
            return false;
        
        Joint joint_ref, joint_out;
        memset(&joint_ref, 0, sizeof(Joint));
        ErrorCode err;
        if (!getJointFromPose(target.circle_target.pose1, joint_ref, joint_out, 2.0, err) ||
            !getJointFromPose(target.circle_target.pose2, joint_ref, joint_out, 2.0, err))
        {
            return false;
        }
    }
    return true;
}

bool PlanningInterface::checkJointBoundry(const Joint &joint)
{
    return  joint.j1 > joint_constraint_.j1.lower && joint.j1 < joint_constraint_.j1.upper &&
            joint.j2 > joint_constraint_.j2.lower && joint.j2 < joint_constraint_.j2.upper &&
            joint.j3 > joint_constraint_.j3.lower && joint.j3 < joint_constraint_.j3.upper &&
            joint.j4 > joint_constraint_.j4.lower && joint.j4 < joint_constraint_.j4.upper &&
            joint.j5 > joint_constraint_.j5.lower && joint.j5 < joint_constraint_.j5.upper &&
            joint.j6 > joint_constraint_.j6.lower && joint.j6 < joint_constraint_.j6.upper;
            //joint.j7 > joint_constraint_.j7.lower && joint.j7 < joint_constraint_.j7.upper &&
            //joint.j8 > joint_constraint_.j8.lower && joint.j8 < joint_constraint_.j8.upper &&
            //joint.j9 > joint_constraint_.j9.lower && joint.j9 < joint_constraint_.j9.upper;
}


}


