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


using std::vector;
using namespace fst_algorithm;

namespace fst_controller
{

MotionCommand::MotionCommand(void)
{
    motion_id_      = -1;
    motion_type_    = MOTION_NONE;
    smooth_type_    = SMOOTH_NONE;

    memset(&target_joint_, 0, sizeof(target_joint_));
    memset(&target_pose1_, 0, sizeof(target_pose1_));
    memset(&target_pose2_, 0, sizeof(target_pose2_));

    cnt_ = -1;
    vel_ = -1;
    acc_ = -1;

    prev_ptr_ = NULL;
    next_ptr_ = NULL;

    max_stamp_  = 0;
    pick_stamp_ = 0;

    is_planned_ = false;
}

ErrorCode MotionCommand::setTarget(const MotionTarget &target, int id)
{
    motion_id_   = id;
    motion_type_ = target.type;
    smooth_type_ = SMOOTH_UNKNOWN;

    cnt_ = target.cnt;
    vel_ = target.vel;
    acc_ = target.acc;

    max_stamp_  = 0;
    pick_stamp_ = 0;
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

    /*
    if (motion_type_ == MOTION_JOINT) {
        if (vel_ < 0.0 || vel_ > 1.0) {
            FST_ERROR("Target vel = %.4f, valid joint vel should between 0.0 - 1.0", vel_);
            return INVALID_PARAMETER;
        }

        if (acc_ < 0.0 || acc_ > 1.0) {
            FST_ERROR("Target acc = %.4f, valid joint acc should between 0.0 - 1.0", acc_);
            return INVALID_PARAMETER;
        }
    }
    else {
        FST_ERROR("Target motion type unsupported, type = %d", motion_type_);
        return INVALID_PARAMETER;
    }
    */

    return SUCCESS;
}

ErrorCode MotionCommand::setStartJoint(const Joint &joint)
{
    beginning_joint_        = joint;
    begin_from_given_joint_ = true;

    return SUCCESS;
    
    /*
    if (isJointInConstraint(joint, g_soft_constraint)) {
        beginning_joint_        = joint;
        begin_from_given_joint_ = true;

        return SUCCESS;
    }
    else {
        FST_ERROR("Cannot setStartJoint, cause given joint is out of soft constraint");
        
        JointConstraint &cons = g_soft_constraint;
        int res = contrastJointWithConstraint(joint, g_soft_constraint);
        
        if ((res >> 0) % 2 == 0)    FST_INFO ("  j1=%.6f - (%.6f, %.6f)", joint.j1, cons.j1.lower, cons.j1.upper);
        else                        FST_ERROR("  j1=%.6f - (%.6f, %.6f)", joint.j1, cons.j1.lower, cons.j1.upper);
        if ((res >> 1) % 2 == 0)    FST_INFO ("  j2=%.6f - (%.6f, %.6f)", joint.j2, cons.j2.lower, cons.j2.upper);
        else                        FST_ERROR("  j2=%.6f - (%.6f, %.6f)", joint.j2, cons.j2.lower, cons.j2.upper);
        if ((res >> 2) % 2 == 0)    FST_INFO ("  j3=%.6f - (%.6f, %.6f)", joint.j3, cons.j3.lower, cons.j3.upper);
        else                        FST_ERROR("  j3=%.6f - (%.6f, %.6f)", joint.j3, cons.j3.lower, cons.j3.upper);
        if ((res >> 3) % 2 == 0)    FST_INFO ("  j4=%.6f - (%.6f, %.6f)", joint.j4, cons.j4.lower, cons.j4.upper);
        else                        FST_ERROR("  j4=%.6f - (%.6f, %.6f)", joint.j4, cons.j4.lower, cons.j4.upper);
        if ((res >> 4) % 2 == 0)    FST_INFO ("  j5=%.6f - (%.6f, %.6f)", joint.j5, cons.j5.lower, cons.j5.upper);
        else                        FST_ERROR("  j5=%.6f - (%.6f, %.6f)", joint.j5, cons.j5.lower, cons.j5.upper);
        if ((res >> 5) % 2 == 0)    FST_INFO ("  j6=%.6f - (%.6f, %.6f)", joint.j6, cons.j6.lower, cons.j6.upper);
        else                        FST_ERROR("  j6=%.6f - (%.6f, %.6f)", joint.j6, cons.j6.lower, cons.j6.upper);
        
        begin_from_given_joint_ = false;
        return JOINT_OUT_OF_CONSTRAINT;
    }
    */
}

ErrorCode MotionCommand::planPath(void)
{
    ErrorCode err = SUCCESS;

    switch (motion_type_)
    {
        case MOTION_JOINT:
            //planJointPath();
            break;

        case MOTION_LINE:
            err = planLinePath();
            break;
            
        case MOTION_CIRCLE:
            //planCirclePath();
            break;

        default:
            FST_ERROR("Cannot plan path, cause motion type = %d is unsupported.", motion_type_);
            err = MOTION_INTERNAL_FAULT;
    }

    if (err == SUCCESS) {
        is_planned_ = true;
        pick_stamp_ = 1;
    }

    return err;
}

ErrorCode MotionCommand::pickPathPoint(Tick stamp, PathPoint &point)
{
    if (!is_planned_) {
        FST_ERROR("pickPathPoint: cannot pick point before the command is planned");
        return INVALID_SEQUENCE;
    }

    if (stamp > max_stamp_) {
        FST_ERROR("pickPathPoint: given stamp (=%d) invalid, max-stamp = %d", stamp, max_stamp_);
        return INVALID_PARAMETER;
    }

    switch (motion_type_)
    {
        case MOTION_JOINT:
            break;

        case MOTION_LINE:
            point.type   = MOTION_LINE;
            point.stamp  = stamp;
            point.source = this;
            interpolateLine(stamp, point.pose);
            
            if (pick_stamp_ > transition_stamp_)   point.speed_down = true;
            else                                   point.speed_down = false;
            
            return SUCCESS;

        case MOTION_CIRCLE:
            break;

        default:
            FST_ERROR("Cannot pick point, cause motion type = %d is unsupported.", motion_type_);
            return INVALID_SEQUENCE;
    }
}

ErrorCode MotionCommand::pickPathPoint(size_t num, vector<PathPoint> &points)
{
    points.clear();

    if (!is_planned_) {
        FST_ERROR("Cannot pickPathPoint before the command is planned.");
        return INVALID_SEQUENCE;
    }

    if (pick_stamp_ > max_stamp_) {
        FST_ERROR("Cannot pickPathPoint from this command, it has burnt out. pick_stamp = %d, max_stamp = %d",
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
                
                if (pick_stamp_ > transition_stamp_)   points[i].speed_down = true;
                else                                   points[i].speed_down = false;

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

ErrorCode MotionCommand::pickCommonPoint(vector<PathPoint> &points)
{
    if (!is_planned_) {
        FST_ERROR("pickCommonPoint: cannot pickPathPoint before the command is planned.");
        return INVALID_SEQUENCE;
    }

    switch (motion_type_)
    {
        case MOTION_JOINT:
            break;

        case MOTION_LINE:
            points.resize(transition_stamp_);
            for (size_t i = 0; i < transition_stamp_; i++)
            {
                points[i].type   = MOTION_LINE;
                points[i].stamp  = i + 1;
                points[i].source = this;
                interpolateLine(i + 1, points[i].pose);

                points[i].style  = POINT_COMMON;
            }
            
            break;

        case MOTION_CIRCLE:
            break;

        default:
            FST_ERROR("pickCommonPoint: cannot pick point, cause motion type = %d is unsupported.", motion_type_);
            return INVALID_SEQUENCE;
    }

    return SUCCESS;
}

ErrorCode MotionCommand::pickAllPoint(vector<PathPoint> &points)
{
    if (!is_planned_) {
        FST_ERROR("pickAllPoint: cannot pickPathPoint before the command is planned.");
        return INVALID_SEQUENCE;
    }

    switch (motion_type_)
    {
        case MOTION_JOINT:
            break;

        case MOTION_LINE:
            points.resize(max_stamp_);
            for (size_t i = 0; i < max_stamp_; i++)
            {
                points[i].type   = MOTION_LINE;
                points[i].stamp  = i + 1;
                points[i].source = this;
                interpolateLine(i + 1, points[i].pose);

                if (i < transition_stamp_)
                    points[i].style  = POINT_COMMON;
                else
                    points[i].style  = POINT_TRANSITION;
            }
            
            break;

        case MOTION_CIRCLE:
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

size_t MotionCommand::getPathLength(void)
{
    return is_planned_ ? max_stamp_ : 0;
}

size_t MotionCommand::getCommonLength(void)
{
    return transition_stamp_;
}

size_t MotionCommand::getTransitionLength(void)
{
    return max_stamp_ - transition_stamp_;
}

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

bool MotionCommand::isCommandPlanned(void)
{
    return is_planned_;
}

bool MotionCommand::isCommandFinished(void)
{
    return pick_stamp_ > max_stamp_;
}

Joint MotionCommand::getJointEnding(void)
{
    Joint j;
    return j;
}

Pose MotionCommand::getCartesianEnding(void)
{
    return pose_ending_;
}

MotionType MotionCommand::getMotionType(void)
{
    return motion_type_;
}

SmoothType MotionCommand::getSmoothType(void)
{
    return smooth_type_;
}

double MotionCommand::getCommandVelocity(void)
{
    return vel_;
}

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



ErrorCode MotionCommand::planLinePath(void)
{
    // 'pose_ending_' is where should we go
    pose_ending_ = target_pose1_;

    // 'pose_starting_' is where should we start
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

    if (innerProductQuatern(pose_starting_.orientation, pose_ending_.orientation) < 0)
    {
        pose_starting_.orientation.w = -pose_starting_.orientation.w;
        pose_starting_.orientation.x = -pose_starting_.orientation.x;
        pose_starting_.orientation.y = -pose_starting_.orientation.y;
        pose_starting_.orientation.z = -pose_starting_.orientation.z;
    }

    // Get position move reference time and orientateion rotate reference time
    double distance = getDistance(pose_starting_, pose_ending_);
    double rotation = getOrientationAngle(pose_starting_, pose_ending_);
    Time   move_tm = distance / vel_;
    Time rotate_tm = rotation / g_orientation_omega_reference;

    // Use the larger time to plan path.
    Time  time = move_tm > rotate_tm ? move_tm : rotate_tm;

    max_stamp_ = ceil(time / (g_cycle_distance / vel_));

    line_coeff_.position_coeff_x  = (pose_ending_.position.x - pose_starting_.position.x) / max_stamp_;
    line_coeff_.position_coeff_y  = (pose_ending_.position.y - pose_starting_.position.y) / max_stamp_;
    line_coeff_.position_coeff_z  = (pose_ending_.position.z - pose_starting_.position.z) / max_stamp_;
    line_coeff_.orientation_angle = getOrientationAngle(pose_starting_, pose_ending_);

    if (rotation > g_ort_linear_polation_threshold_)
    {
        // Spherical interpolation
        line_coeff_.spherical_flag = true;
    }
    else
    {
        // Linear interpolation
        line_coeff_.spherical_flag = false;
    }

    // find where is stop-point, at this point we begin to back-step
    double transition_velocity = vel_ * cnt_;
    double transition_distance = transition_velocity * transition_velocity / g_cart_acc_reference / 2;
    transition_distance = transition_distance < distance / 2 ? transition_distance : distance / 2;
    
    Tick stop_tick = ceil(transition_distance / (distance / max_stamp_));
    transition_stamp_ = max_stamp_ - stop_tick;

    return SUCCESS;
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



}
