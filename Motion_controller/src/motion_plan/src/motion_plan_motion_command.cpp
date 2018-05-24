/*************************************************************************
	> File Name: motion_plan_motion_command.cpp
	> Author:   Feng Yun
	> Mail:     yun.feng@foresight-robotics.com
	> Created Time: 2018年02月01日 星期四 09时22分42秒
 ************************************************************************/

#include <string.h>
#include <motion_plan_motion_command.h>
#include <motion_plan_basic_function.h>
#include <motion_plan_kinematics.h>
#include <motion_plan_variable.h>
#include <motion_plan_reuse.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace fst_algorithm;

namespace fst_controller
{

MotionCommand::MotionCommand(void)
{
    motion_id_      = -1;
    motion_type_    = MOTION_NONE;

    memset(&beginning_joint_, 0, sizeof(beginning_joint_));
    memset(&target_joint_, 0, sizeof(target_joint_));
    memset(&target_pose1_, 0, sizeof(target_pose1_));
    memset(&target_pose2_, 0, sizeof(target_pose2_));

    cnt_ = -1;
    vel_ = -1;
    acc_ = -1;

    average_duration_ = -1;
    max_stamp_  = 0;

    is_planned_ = false;
}

ErrorCode MotionCommand::setTarget(const MotionTarget &target, int id)
{
    motion_id_   = id;
    motion_type_ = target.type;

    cnt_ = target.cnt;
    vel_ = target.vel;
    acc_ = target.acc;

    average_duration_ = -1;
    max_stamp_  = 0;
    is_planned_ = false;
    
    if (motion_type_ == MOTION_JOINT) {
        target_joint_ = target.joint_target;
    }
    else if (motion_type_ = MOTION_LINE) {
        PoseEuler2Pose(target.pose_target, target_pose1_);
    }
    else if (motion_type_ = MOTION_CIRCLE) {
        PoseEuler2Pose(target.circle_target.pose1, target_pose1_);
        PoseEuler2Pose(target.circle_target.pose2, target_pose2_);
    }

    return SUCCESS;
}

ErrorCode MotionCommand::setStartJoint(const Joint &joint)
{
    beginning_joint_        = joint;

    return SUCCESS;
}

ErrorCode MotionCommand::planPath(void)
{
    ErrorCode err = SUCCESS;

    switch (motion_type_)
    {
        case MOTION_JOINT:
            err = planJointPath();
            break;

        case MOTION_LINE:
            err = planLinePath();
            break;
            
        case MOTION_CIRCLE:
            // TODO
            //planCirclePath();
            break;

        default:
            FST_ERROR("Cannot plan path, cause motion type = %d is unsupported.", motion_type_);
            err = MOTION_INTERNAL_FAULT;
    }

    is_planned_ = (err == SUCCESS);

    return err;
}

ErrorCode MotionCommand::pickPathPoint(Tick stamp, PathPoint &point)
{
    if (!is_planned_)
    {
        FST_ERROR("pickPathPoint: cannot pick before plan");
        return INVALID_SEQUENCE;
    }

    if (stamp > max_stamp_)
    {
        FST_ERROR("pickPathPoint: given stamp (=%d) invalid, max-stamp = %d", stamp, max_stamp_);
        return INVALID_PARAMETER;
    }

    switch (motion_type_)
    {
        case MOTION_JOINT:
            if (stamp == 0)
            {
                point.type   = MOTION_JOINT;
                point.stamp  = 0;
                point.source = this;
                point.joint  = joint_starting_;
            }
            else if (stamp == max_stamp_)
            {
                point.type   = MOTION_JOINT;
                point.stamp  = max_stamp_;
                point.source = this;
                point.joint  = joint_ending_;
            }
            else
            {
                point.type   = MOTION_JOINT;
                point.stamp  = stamp;
                point.source = this;
                interpolateJoint(stamp, point.joint);
            }
            
            break;

        case MOTION_LINE:
            if (stamp == 0)
            {
                point.type   = MOTION_LINE;
                point.stamp  = 0;
                point.source = this;
                point.pose   = pose_starting_;
            }
            else if (stamp == max_stamp_)
            {
                point.type   = MOTION_LINE;
                point.stamp  = max_stamp_;
                point.source = this;
                point.pose   = pose_ending_;
            }
            else
            {
                point.type   = MOTION_LINE;
                point.stamp  = stamp;
                point.source = this;
                interpolateLine(stamp, point.pose);
            }
            
            break;

        case MOTION_CIRCLE:
            // TODO
            break;

        default:
            FST_ERROR("Cannot pick point, cause motion type = %d is unsupported.", motion_type_);
            return INVALID_SEQUENCE;
    }
    
    return SUCCESS;
}

ErrorCode MotionCommand::pickAllPoint(vector<PathPoint> &points)
{
    if (!is_planned_)
    {
        FST_ERROR("pickAllPoint: cannot pick before plan.");
        return INVALID_SEQUENCE;
    }

    switch (motion_type_)
    {
        case MOTION_JOINT:
            points.resize(max_stamp_ + 1);
            
            points[0].id     = motion_id_;
            points[0].type   = MOTION_JOINT;
            points[0].stamp  = 0;
            points[0].source = this;
            points[0].joint  = joint_starting_;
            
            points[max_stamp_].type   = MOTION_JOINT;
            points[max_stamp_].stamp  = max_stamp_;
            points[max_stamp_].source = this;
            points[max_stamp_].joint  = joint_ending_;
            
            for (size_t i = 1; i < max_stamp_; i++)
            {
                points[i].id     = motion_id_;
                points[i].type   = MOTION_JOINT;
                points[i].stamp  = i;
                points[i].source = this;
                interpolateJoint(i, points[i].joint);
            }

            break;

        case MOTION_LINE:
            points.resize(max_stamp_ + 1);

            points[0].id     = motion_id_;
            points[0].type   = MOTION_LINE;
            points[0].stamp  = 0;
            points[0].source = this;
            points[0].pose   = pose_starting_;
            
            points[max_stamp_].type   = MOTION_LINE;
            points[max_stamp_].stamp  = max_stamp_;
            points[max_stamp_].source = this;
            points[max_stamp_].pose   = pose_ending_;
            
            for (size_t i = 1; i < max_stamp_; i++)
            {
                points[i].id     = motion_id_;
                points[i].type   = MOTION_LINE;
                points[i].stamp  = i;
                points[i].source = this;
                interpolateLine(i, points[i].pose);
            }
            
            break;

        case MOTION_CIRCLE:
            // TODO
            break;

        default:
            FST_ERROR("pickAllPoint: cannot pick point, cause motion type = %d is unsupported.", motion_type_);
            return INVALID_SEQUENCE;
    }

    return SUCCESS;
}

int MotionCommand::getMotionID(void)
{
    return motion_type_ != MOTION_NONE ? motion_id_ : -1;
}

MotionType MotionCommand::getMotionType(void)
{
    return motion_type_;
}

size_t MotionCommand::getPathLength(void)
{
    return is_planned_ ? max_stamp_ : 0;
}

bool MotionCommand::isCommandPlanned(void)
{
    return is_planned_;
}

double MotionCommand::getCommandVelocity(void)
{
    return vel_;
}

double MotionCommand::getCommandAcc(void)
{
    return acc_;
}

double MotionCommand::getCommandCNT(void)
{
    return cnt_;
}

double MotionCommand::getCommandDuration(void)
{
    if (is_planned_)
    {
        return average_duration_;
    }
    else
    {
        return -1;
    }
}

ErrorCode MotionCommand::planJointPath(void)
{
    ErrorCode err = SUCCESS;

    // set end point
    joint_ending_ = target_joint_;

    // set start point
    joint_starting_ = beginning_joint_;
    
    /*
    if(begin_from_given_joint_){
        joint_starting_ = beginning_joint_;
    }
    else{
        // FIXME: current getJointEnding() only support MOVJ, except MOVL and MOVC
        joint_starting_ = prev_ptr_->getJointEnding();
    }
    */

    /*
    // set joint limits
    Omega velocity_max[AXIS_IN_ALGORITHM];
    Alpha acc_max[AXIS_IN_ALGORITHM];
    velocity_max[0] = g_soft_constraint.j1.max_omega;
    velocity_max[1] = g_soft_constraint.j2.max_omega;
    velocity_max[2] = g_soft_constraint.j3.max_omega;
    velocity_max[3] = g_soft_constraint.j4.max_omega;
    velocity_max[4] = g_soft_constraint.j5.max_omega;
    velocity_max[5] = g_soft_constraint.j6.max_omega;
    velocity_max[0] = vel_*g_soft_constraint.j1.max_omega;
    velocity_max[1] = vel_*g_soft_constraint.j2.max_omega;
    velocity_max[2] = vel_*g_soft_constraint.j3.max_omega;
    velocity_max[3] = vel_*g_soft_constraint.j4.max_omega;
    velocity_max[4] = vel_*g_soft_constraint.j5.max_omega;
    velocity_max[5] = vel_*g_soft_constraint.j6.max_omega;

    acc_max[0] = acc_*g_soft_constraint.j1.max_alpha;
    acc_max[1] = acc_*g_soft_constraint.j2.max_alpha;
    acc_max[2] = acc_*g_soft_constraint.j3.max_alpha;
    acc_max[3] = acc_*g_soft_constraint.j4.max_alpha;
    acc_max[4] = acc_*g_soft_constraint.j5.max_alpha;
    acc_max[5] = acc_*g_soft_constraint.j6.max_alpha;
    */
    /*FST_INFO("velocity_max: axis1 =%f, axis2 =%f, axis3 =%f, axis4 =%f, axis5 =%f, axis6 =%f",
                g_soft_constraint.j1.max_omega, g_soft_constraint.j2.max_omega,
                g_soft_constraint.j3.max_omega, g_soft_constraint.j4.max_omega,
                g_soft_constraint.j5.max_omega, g_soft_constraint.j6.max_omega);
    FST_INFO("acc_max: axis1 =%f, axis2 =%f, axis3 =%f, axis4 =%f, axis5 =%f, axis6 =%f",
                g_soft_constraint.j1.max_alpha, g_soft_constraint.j2.max_alpha,
                g_soft_constraint.j3.max_alpha, g_soft_constraint.j4.max_alpha,
                g_soft_constraint.j5.max_alpha, g_soft_constraint.j6.max_alpha);*/
    
    // compute minimum time of axes
    /*
    MotionTime duration_max = -1;
    int duration_max_index = 0;

    MotionTime duration[AXIS_IN_ALGORITHM];
    Angle delta_joint[AXIS_IN_ALGORITHM];
    Angle* joint_start_ptr = (Angle*)&joint_starting_;
    Angle* joint_end_ptr = (Angle*)&joint_ending_;
    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        delta_joint[i] = joint_end_ptr[i] - joint_start_ptr[i];
        Angle delta_joint_fabs = fabs(delta_joint[i]);
        if(fabs(delta_joint_fabs) < MINIMUM_E6) // deal with the problem of computational accuracy
        {
            delta_joint[i] = 0;
            delta_joint_fabs = 0;
        }
        double condition = velocity_max[i] * velocity_max[i] / acc_max[i];
        if(condition >= delta_joint_fabs)    // triangle velocity curve
        {
            duration[i] = 2*sqrt(delta_joint_fabs / acc_max[i]);
        }
        else    // trapezoid velocity curve
        {
            duration[i] = delta_joint_fabs / velocity_max[i] + velocity_max[i] / acc_max[i];
        }

        if(duration[i] > duration_max)
        {
            duration_max = duration[i];
            duration_max_index = i;
        }
    }
    */

    size_t duration_index = 0;
    MotionTime  duration_max = -1;
    MotionTime  durations[AXIS_IN_ALGORITHM];
    JointLimit  *jnt_limit = &g_soft_constraint.j1;
    double  *js = &joint_starting_.j1;
    double  *je = &joint_ending_.j1;

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        durations[i] = fabs(je[i] - js[i]) / g_omega_limit[i];
        if (durations[i] > duration_max)
        {
            duration_max = durations[i];
            duration_index = i;
        }
    }

    // compute max_stamp
    max_stamp_ = ceil(fabs(je[duration_index] - js[duration_index]) / g_cycle_radian);
    joint_coeff_[0] = (je[0] - js[0]) / max_stamp_;
    joint_coeff_[1] = (je[1] - js[1]) / max_stamp_;
    joint_coeff_[2] = (je[2] - js[2]) / max_stamp_;
    joint_coeff_[3] = (je[3] - js[3]) / max_stamp_;
    joint_coeff_[4] = (je[4] - js[4]) / max_stamp_;
    joint_coeff_[5] = (je[5] - js[5]) / max_stamp_;

    average_duration_ = fabs(joint_coeff_[duration_index]) / g_omega_limit[duration_index] / vel_;
    /*
    max_stamp_ = ceil(duration_max / (g_cycle_radian / velocity_max[duration_max_index]));
    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        joint_coeff_[i] = delta_joint[i] / max_stamp_;
    }
    FST_INFO("joint_coeff: axis1 =%f, axis2 =%f, axis3 =%f, axis4 =%f, axis5 =%f, axis6 =%f",
                joint_coeff_[0], joint_coeff_[1], joint_coeff_[2],
                joint_coeff_[3], joint_coeff_[4], joint_coeff_[5]);*/

    return SUCCESS;
}

ErrorCode MotionCommand::planLinePath(void)
{
    // 'pose_ending_' is where should we go
    pose_ending_ = target_pose1_;

    // 'pose_starting_' is where should we start
    forwardKinematics(beginning_joint_, pose_starting_);
    
    /*
    if (begin_from_given_joint_)
    {
        forwardKinematics(beginning_joint_, pose_starting_);
    }
    else if (prev_ptr_ != NULL)
    {
        if (prev_ptr_->getMotionType() == MOTION_JOINT)
        {
            forwardKinematics(prev_ptr_->getJointEnding(), pose_starting_);
        }
        else if (prev_ptr_->getMotionType() == MOTION_LINE || prev_ptr_->getMotionType() == MOTION_CIRCLE)
        {
            pose_starting_ = prev_ptr_->getCartesianEnding();
        }
        else
        {
            FST_ERROR("Error while getting start pose from previous motion command.");
            FST_ERROR("It is assigned as an undefined type, type=%d", prev_ptr_->getMotionType());
            return MOTION_INTERNAL_FAULT;
        }
    }
    else
    {
        FST_ERROR("Beginning of the path undefined.");
        return INVALID_SEQUENCE;
    }
    */

    double cycle_distance, cycle_radian;

    if      (vel_ < 1000)   {cycle_distance = g_cycle_distance;     cycle_radian = g_cycle_radian;      }
    else if (vel_ < 2000)   {cycle_distance = g_cycle_distance * 2; cycle_radian = g_cycle_radian * 2;  }
    else if (vel_ < 3000)   {cycle_distance = g_cycle_distance * 3; cycle_radian = g_cycle_radian * 3;  }
    else                    {cycle_distance = g_cycle_distance * 4; cycle_radian = g_cycle_radian * 4;  }

    if (innerProductQuatern(pose_starting_.orientation, pose_ending_.orientation) < 0)
    {
        pose_starting_.orientation.w = -pose_starting_.orientation.w;
        pose_starting_.orientation.x = -pose_starting_.orientation.x;
        pose_starting_.orientation.y = -pose_starting_.orientation.y;
        pose_starting_.orientation.z = -pose_starting_.orientation.z;
    }

    // Get position distance and orientateion rotate angle
    // Resolve the max stamp of this path
    double distance = getDistance(pose_starting_.position, pose_ending_.position);
    double rotation = getOrientationAngle(pose_starting_.orientation, pose_ending_.orientation);
    double stamp_position    = distance / cycle_distance;
    double stamp_orientation = rotation / cycle_radian;
    max_stamp_ = stamp_position > stamp_orientation ? ceil(stamp_position) : ceil(stamp_orientation);

    line_coeff_.position_coeff_x  = (pose_ending_.position.x - pose_starting_.position.x) / max_stamp_;
    line_coeff_.position_coeff_y  = (pose_ending_.position.y - pose_starting_.position.y) / max_stamp_;
    line_coeff_.position_coeff_z  = (pose_ending_.position.z - pose_starting_.position.z) / max_stamp_;
    line_coeff_.orientation_angle = rotation;

    if (rotation > g_ort_linear_polation_threshold)
    {
        // Spherical interpolation
        line_coeff_.spherical_flag = true;
    }
    else
    {
        // Linear interpolation
        line_coeff_.spherical_flag = false;
    }

    average_duration_ = distance / max_stamp_ / vel_;

    return SUCCESS;
}

void MotionCommand::interpolateJoint(Tick stamp, Joint &joint)
{
    joint.j1 = joint_starting_.j1 + joint_coeff_[0] * stamp;
    joint.j2 = joint_starting_.j2 + joint_coeff_[1] * stamp;
    joint.j3 = joint_starting_.j3 + joint_coeff_[2] * stamp;
    joint.j4 = joint_starting_.j4 + joint_coeff_[3] * stamp;
    joint.j5 = joint_starting_.j5 + joint_coeff_[4] * stamp;
    joint.j6 = joint_starting_.j6 + joint_coeff_[5] * stamp;
}

void MotionCommand::interpolateLine(Tick stamp, Pose &pose)
{
    pose.position.x = pose_starting_.position.x + line_coeff_.position_coeff_x * stamp;
    pose.position.y = pose_starting_.position.y + line_coeff_.position_coeff_y * stamp;
    pose.position.z = pose_starting_.position.z + line_coeff_.position_coeff_z * stamp;
    
    if (line_coeff_.spherical_flag) {
        double a1 = sin(line_coeff_.orientation_angle * (max_stamp_ - stamp ) / max_stamp_) / sin(line_coeff_.orientation_angle);
        double a2 = sin(line_coeff_.orientation_angle * stamp / max_stamp_) / sin(line_coeff_.orientation_angle);

        pose.orientation.w = a1 * pose_starting_.orientation.w + a2 * pose_ending_.orientation.w;
        pose.orientation.x = a1 * pose_starting_.orientation.x + a2 * pose_ending_.orientation.x;
        pose.orientation.y = a1 * pose_starting_.orientation.y + a2 * pose_ending_.orientation.y;
        pose.orientation.z = a1 * pose_starting_.orientation.z + a2 * pose_ending_.orientation.z;
    }
    else {
        double a1 = (double)(max_stamp_ - stamp ) / max_stamp_;
        double a2 = (double)stamp / max_stamp_;

        pose.orientation.w = a1 * pose_starting_.orientation.w + a2 * pose_ending_.orientation.w;
        pose.orientation.x = a1 * pose_starting_.orientation.x + a2 * pose_ending_.orientation.x;
        pose.orientation.y = a1 * pose_starting_.orientation.y + a2 * pose_ending_.orientation.y;
        pose.orientation.z = a1 * pose_starting_.orientation.z + a2 * pose_ending_.orientation.z;
    }
}








//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////  Temporary code area  /////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Tick MotionCommand::getNextStamp(void)
{
    return is_planned_ ? pick_stamp_ : 0;
}

bool MotionCommand::setNextStamp(Tick stamp)
{
    if (is_planned_) {
        if (stamp <= max_stamp_) {
            pick_stamp_ = stamp;
            return true;
        }
        else {
            FST_ERROR("Cannot set next pick stamp to %d, it's larger than max_stamp = %d on this path.",
                      stamp, max_stamp_);
            return false;
        }
    }
    else {
        FST_ERROR("Cannot set next pick stamp before MotionCommand is planned!");
        return false;
    }
}

bool MotionCommand::isCommandFinished(void)
{
    return pick_stamp_ > max_stamp_;
}

Joint MotionCommand::getJointEnding(void)
{
    return joint_ending_;
}

Pose MotionCommand::getCartesianEnding(void)
{
    return pose_ending_;
}
*/


/*
SmoothType MotionCommand::getSmoothType(void)
{
    return smooth_type_;
}
*/

/*
MotionCommand* MotionCommand::getPrevCommandPtr(void)
{
    return prev_ptr_;
}

MotionCommand* MotionCommand::getNextCommandPtr(void)
{
    return next_ptr_;
}

void MotionCommand::setPrevCommandPtr(MotionCommand *ptr)
{
    prev_ptr_ = ptr;
}

void MotionCommand::setNextCommandPtr(MotionCommand *ptr)
{
    next_ptr_ = ptr;
}
*/

/*
ErrorCode MotionCommand::pickPathPoint(size_t num, vector<PathPoint> &points)
{
    points.clear();

    if (!is_planned_) {
        FST_ERROR("pickPathPoint: cannot pick before plan.");
        return INVALID_SEQUENCE;
    }

    if (pick_stamp_ > max_stamp_) {
        FST_ERROR("pickPathPoint: command has burnt out. pick_stamp = %d, max_stamp = %d",
                    pick_stamp_, max_stamp_);
        return INVALID_SEQUENCE;
    }

    switch (motion_type_)
    {
        case MOTION_JOINT:
            break;
        case MOTION_LINE:
            if (num + pick_stamp_ > max_stamp_ + 1)
                num = max_stamp_ + 1 - pick_stamp_;

            points.resize(num);
            
            for (size_t i = 0; i < num; i++) {
                points[i].type   = MOTION_LINE;
                points[i].stamp  = pick_stamp_;
                points[i].source = this;
                interpolateLine(pick_stamp_, points[i].pose);
                pick_stamp_++;
            }
            
            break;
        case MOTION_CIRCLE:
            break;

        default:
            FST_ERROR("Cannot pick point, cause motion type = %d is unsupported.", motion_type_);
            return INVALID_SEQUENCE;
    }

    return SUCCESS;
}

ErrorCode MotionCommand::planPathAndTrajectory(ControlPoint* traj_path, size_t& traj_head, size_t& traj_tail)
{
    FST_INFO("running planPathAndTrajectory----------");
    ErrorCode err = SUCCESS;

    // set end point
    joint_ending_ = target_joint_;

    // set start point
    if(begin_from_given_joint_){
        joint_starting_ = beginning_joint_;
    }
    else{
        // FIXME: current getJointEnding() only support MOVJ, except MOVL and MOVC
        joint_starting_ = prev_ptr_->getJointEnding();
    }

    // set joint limits
    Omega velocity_max[AXIS_IN_ALGORITHM];
    Alpha acc_max[AXIS_IN_ALGORITHM];
    velocity_max[0] = vel_*g_soft_constraint.j1.max_omega;
    velocity_max[1] = vel_*g_soft_constraint.j2.max_omega;
    velocity_max[2] = vel_*g_soft_constraint.j3.max_omega;
    velocity_max[3] = vel_*g_soft_constraint.j4.max_omega;
    velocity_max[4] = vel_*g_soft_constraint.j5.max_omega;
    velocity_max[5] = vel_*g_soft_constraint.j6.max_omega;
    acc_max[0] = acc_*g_soft_constraint.j1.max_alpha;
    acc_max[1] = acc_*g_soft_constraint.j2.max_alpha;
    acc_max[2] = acc_*g_soft_constraint.j3.max_alpha;
    acc_max[3] = acc_*g_soft_constraint.j4.max_alpha;
    acc_max[4] = acc_*g_soft_constraint.j5.max_alpha;
    acc_max[5] = acc_*g_soft_constraint.j6.max_alpha;
    FST_INFO("velocity_max: axis1 =%f, axis2 =%f, axis3 =%f, axis4 =%f, axis5 =%f, axis6 =%f",
                g_soft_constraint.j1.max_omega, g_soft_constraint.j2.max_omega,
                g_soft_constraint.j3.max_omega, g_soft_constraint.j4.max_omega,
                g_soft_constraint.j5.max_omega, g_soft_constraint.j6.max_omega);
    FST_INFO("acc_max: axis1 =%f, axis2 =%f, axis3 =%f, axis4 =%f, axis5 =%f, axis6 =%f",
                g_soft_constraint.j1.max_alpha, g_soft_constraint.j2.max_alpha,
                g_soft_constraint.j3.max_alpha, g_soft_constraint.j4.max_alpha,
                g_soft_constraint.j5.max_alpha, g_soft_constraint.j6.max_alpha);
    
    // compute minimum time of axes
    MotionTime duration_max = -1;
    int duration_max_index = 0;
    MotionTime duration[AXIS_IN_ALGORITHM];
    Angle delta_joint[AXIS_IN_ALGORITHM];
    Angle* joint_start_ptr = (Angle*)&joint_starting_;
    Angle* joint_end_ptr = (Angle*)&joint_ending_;
    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        delta_joint[i] = joint_end_ptr[i] - joint_start_ptr[i];
        Angle delta_joint_fabs = fabs(delta_joint[i]);
        if(fabs(delta_joint_fabs) < MINIMUM_E6) // deal with the problem of computational accuracy
        {
            delta_joint[i] = 0;
            delta_joint_fabs = 0;
        }
        double condition = velocity_max[i] * velocity_max[i] / acc_max[i];
        if(condition >= delta_joint_fabs)    // triangle velocity curve
        {
            duration[i] = 2*sqrt(delta_joint_fabs / acc_max[i]);
        }
        else    // trapezoid velocity curve
        {
            duration[i] = delta_joint_fabs / velocity_max[i] + velocity_max[i] / acc_max[i];
        }

        if(duration[i] > duration_max)
        {
            duration_max = duration[i];
            duration_max_index = i;
        }
    }

    FST_INFO("delta_joint: axis1 =%f, axis2 =%f, axis3 =%f, axis4 =%f, axis5 =%f, axis6 =%f", 
                delta_joint[0], delta_joint[1], delta_joint[2], delta_joint[3], delta_joint[4], delta_joint[5]);
    FST_INFO("duration: axis1 =%f, axis2 =%f, axis3 =%f, axis4 =%f, axis5 =%f, axis6 =%f", 
                duration[0], duration[1], duration[2], duration[3], duration[4], duration[5]);
    FST_INFO("duration_max = %f, duration_max_index = %d", duration_max, duration_max_index);

    // rescale the velocity and acc/deacc time of axes according to duration_max
    Omega velocity_actual[AXIS_IN_ALGORITHM];
    MotionTime acc_duration[AXIS_IN_ALGORITHM];
    bool is_triangle[AXIS_IN_ALGORITHM];
    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        Angle delta_joint_fabs = fabs(delta_joint[i]);
        double condition = pow(acc_max[i] * duration_max, 2) - 4 * acc_max[i] * delta_joint_fabs;
        if(condition > 0)   // trapezoid velocity curve
        {
            acc_duration[i] = duration_max / 2 - sqrt(condition) / (2 * acc_max[i]);
            velocity_actual[i] = acc_max[i] * acc_duration[i];
            is_triangle[i] = false;
        }
        else    // triangle velocity curve
        {
            acc_duration[i] = duration_max / 2;
            velocity_actual[i] = 2 * delta_joint_fabs / duration_max;
            is_triangle[i] = true;
        }
    }
    FST_INFO("velocity_actual: axis1 =%f, axis2 =%f, axis3 =%f, axis4 =%f, axis5 =%f, axis6 =%f", 
             velocity_actual[0], velocity_actual[1], velocity_actual[2], velocity_actual[3], velocity_actual[4], velocity_actual[5]);
    FST_INFO("acc_duration: axis1 =%f, axis2 =%f, axis3 =%f, axis4 =%f, axis5 =%f, axis6 =%f", 
             acc_duration[0], acc_duration[1], acc_duration[2], acc_duration[3], acc_duration[4], acc_duration[5]);
    FST_INFO("is_triangle: axis1 =%d, axis2 =%d, axis3 =%d, axis4 =%d, axis5 =%d, axis6 =%d", 
             is_triangle[0], is_triangle[1], is_triangle[2], is_triangle[3], is_triangle[4], is_triangle[5]);

    // compute joint function coefficient
    JointPathCoeff coeff[AXIS_IN_ALGORITHM];
    memset(coeff, 0, AXIS_IN_ALGORITHM*sizeof(JointPathCoeff));
    FST_INFO("coeffs are:");
    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        coeff[i].coeff1 = acc_max[i] / 2;
        if(is_triangle[i])
        {
            coeff[i].coeff2 = velocity_actual[i] * duration_max / 2;
            FST_INFO("axis%d: coeff1 = %f, coeff2 = %f", i, coeff[i].coeff1, coeff[i].coeff2);
        }
        else
        {
            coeff[i].coeff3 = acc_duration[i] * (coeff[i].coeff1 * acc_duration[i] - velocity_actual[i]);
            coeff[i].coeff4 = velocity_actual[i] * (duration_max - acc_duration[i]);
            FST_INFO("axis%d: coeff1 = %f, coeff3 = %f, coeff4 = %f", i, coeff[i].coeff1, coeff[i].coeff3, coeff[i].coeff4); 
        }
    }

    // discrete the joint function with sample rate g_cycle_time, compute angle/velocity/acc
    traj_head = 0;
    traj_tail = 0;
    for(double t = 0; t <= duration_max; t += g_cycle_time, ++traj_tail)
    {
        for(int j = 0; j < AXIS_IN_ALGORITHM; ++j)
        {
            if(is_triangle[j])
            {
                if(t <= duration_max / 2)
                {
                    traj_path[traj_tail].point.joint[j] = coeff[j].coeff1 * t * t;
                    traj_path[traj_tail].point.omega[j] = acc_max[j] * t;
                    traj_path[traj_tail].point.alpha[j] = acc_max[j];
                }
                else
                {
                    traj_path[traj_tail].point.joint[j] = coeff[j].coeff2 - coeff[j].coeff1 * (duration_max - t);
                    traj_path[traj_tail].point.omega[j] = coeff[j].coeff2 - acc_max[j] * (t - duration_max / 2);
                    traj_path[traj_tail].point.alpha[j] = -acc_max[j];
                }
            }
            else
            {
                if(t <= acc_duration[j])
                {
                    traj_path[traj_tail].point.joint[j] = coeff[j].coeff1 * t * t;
                    traj_path[traj_tail].point.omega[j] = acc_max[j] * t;
                    traj_path[traj_tail].point.alpha[j] = acc_max[j];
                }
                else if(t > duration_max - acc_duration[j])
                {
                    traj_path[traj_tail].point.joint[j] = coeff[j].coeff4 - coeff[j].coeff1 * pow((duration_max - t), 2);
                    traj_path[traj_tail].point.omega[j] = velocity_actual[j] - acc_max[j] * (t - (duration_max - acc_duration[j]));
                    traj_path[traj_tail].point.alpha[j] = -acc_max[j];
                }
                else
                {
                    traj_path[traj_tail].point.joint[j] = coeff[j].coeff3 + velocity_actual[j] * t;
                    traj_path[traj_tail].point.omega[j] = velocity_actual[j];
                    traj_path[traj_tail].point.alpha[j] = 0;
                }
            }
            
            // judge signal of angle/velocity/acc
            if(delta_joint[j] >=  0)
            {
                traj_path[traj_tail].point.joint[j] = joint_start_ptr[j] + traj_path[traj_tail].point.joint[j];
            }
            else
            {
                traj_path[traj_tail].point.joint[j] = joint_start_ptr[j] - traj_path[traj_tail].point.joint[j];
                traj_path[traj_tail].point.omega[j] = -traj_path[traj_tail].point.omega[j];
                traj_path[traj_tail].point.alpha[j] = -traj_path[traj_tail].point.alpha[j];
            }
        }
        traj_path[traj_tail].time_from_start = t;
    }

    // deal with the first time piece problem
    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
    {
        traj_path[0].point.joint[i] = joint_start_ptr[i];
        traj_path[0].point.omega[i] = 0;
        traj_path[0].point.alpha[i] = 0;
    }
    
    // deal with the last time piece if it exists
    if(traj_path[traj_tail].time_from_start < duration_max)
    {
        ++traj_tail;
        for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)
        {
            traj_path[traj_tail].point.joint[i] = joint_end_ptr[i];
            traj_path[traj_tail].point.omega[i] = 0;
            traj_path[traj_tail].point.alpha[i] = 0;
        }
        traj_path[traj_tail].time_from_start = duration_max;
    }

    // output to file
    std::ofstream os("/home/fst/myworkspace/jout.txt");

    for (size_t i = 0; i < traj_tail + 1; ++i)
    {
        os  << traj_path[i].point.joint[0] << " "
            << traj_path[i].point.joint[1] << " "
            << traj_path[i].point.joint[2] << " "
            << traj_path[i].point.joint[3] << " "
            << traj_path[i].point.joint[4] << " "
            << traj_path[i].point.joint[5] << " "
            << traj_path[i].point.omega[0] << " "
            << traj_path[i].point.omega[1] << " "
            << traj_path[i].point.omega[2] << " "
            << traj_path[i].point.omega[3] << " "
            << traj_path[i].point.omega[4] << " "
            << traj_path[i].point.omega[5] << " "
            << traj_path[i].point.alpha[0] << " "
            << traj_path[i].point.alpha[1] << " "
            << traj_path[i].point.alpha[2] << " "
            << traj_path[i].point.alpha[3] << " "
            << traj_path[i].point.alpha[4] << " "
            << traj_path[i].point.alpha[5] << endl;            
    }

    os.close();
    
    cout << "end" << endl;

    return SUCCESS;
}
*/

}
