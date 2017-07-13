/*************************************************************************
	> File Name: motion_controller_arm_group_planning.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年02月28日 星期二 17时14分42秒
 ************************************************************************/

#include <motion_controller/motion_controller_arm_group.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;

namespace fst_controller {

static const unsigned int UNDEFINED     = 0x5555;
static const unsigned int INITIALIZED   = 0x5556;



//------------------------------------------------------------------------------
// Function:    MoveJ (without smooth)
// Summary: To plan a path in joint space to touch target pose, without smooth.
// In:      joint_target    -> target in joint space
//          v_percent       -> velocity
//          a_percent       -> acceleration
//          id      -> command id
// Out:     err     -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveJ(const Joint &joint_target, double v_percent, double a_percent,
                     int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED)  {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveJ (without smooth) request accepted, planning joint path...");
    if (!isMotionExecutable(MOTION_JOINT, err)) {
        log.error("  Cannot plan a path in joint space, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!checkJointBoundary(joint_target)) {
        err = TARGET_OUT_OF_CONSTRAINT;
        log.error("  Target point out of joint constraint, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!planning_interface_->setAlphaScaling(a_percent)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    MotionTarget target, next;
    target.type = MOTION_JOINT;
    target.cnt  = 0;
    target.joint_target     = joint_target;
    target.percent_velocity = v_percent;
    next.type   = MOTION_NONE;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);

    if (err == SUCCESS) {
        log.info("  Joint path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
            else {
                log.warn("Suspend_state flag is true, cannot convert points FIFO1 > FIFO2");
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() >0) {
            return true;
        }
        else {
            err = TARGET_REPEATED;
            return false;
        }
    }
    else {
        log.error("  ERROR occurred during generating joint path, error code:0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
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
// Out:     path(hidden)-> outputs added into planned_path_fifo_ automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveJ(const Joint &joint_target, double v_percent, double a_percent, int cnt,
                     const Joint &joint_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveJ (smooth to MoveJ) request accepted, planning joint path...");
    if (!isMotionExecutable(MOTION_JOINT, err)) {
        log.error("  Cannot plan a path in joint space, planning abort", err);
        unlockArmGroup();
        return false;
    }
    
    if (!checkJointBoundary(joint_target) || !checkJointBoundary(joint_next)) {
        err = TARGET_OUT_OF_CONSTRAINT;
        log.error("  Target out of constraint, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!planning_interface_->setAlphaScaling(a_percent))
    {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    //if (planning_interface_->isPointCoincident(joint_target, joint_next)) {
    //    log.warn("  Target point coincided with next point, reduce CNT from %d to 0", cnt);
    //    cnt = 0;
    //}

    MotionTarget target, next;
    target.type = MOTION_JOINT;
    target.cnt  = cnt;
    target.joint_target     = joint_target;
    target.percent_velocity = v_percent;
    next.type   = MOTION_JOINT;
    next.cnt    = cnt_next;
    next.joint_target       = joint_next;
    next.percent_velocity   = v_next;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);

    if (err == SUCCESS) {
        log.info("  Joint path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() > 0) {
            return true;
        }
        else {
            err = TARGET_REPEATED;
            return false;
        }
    }
    //else if (err == TARGET_REPEATED) {
    //    log.warn("  Target repeated, error code=0x%llx, rebuild planning scene", err);
    //    rebuildPlanningVariable(err); 
    //    unlockArmGroup();
    //    return false;
    //}
    else {
        log.error("  ERROR occurred during generating joint path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
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
// Out:     path(hidden)-> outputs added into planned_path_fifo_ automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveJ(const Joint &joint_target, double v_percent, double a_percent, int cnt,
                     const Pose &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveJ (smooth to MoveL) request accepted, planning joint path...");
    if (!isMotionExecutable(MOTION_JOINT, err)) {
        log.error("  Cannot plan a path in joint space, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!checkJointBoundary(joint_target)) {
        err = TARGET_OUT_OF_CONSTRAINT;
        log.error("  Target point out of joint constraint, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!planning_interface_->setAlphaScaling(a_percent))
    {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    MotionTarget target, next;
    target.type = MOTION_JOINT;
    target.cnt  = cnt;
    target.joint_target     = joint_target;
    target.percent_velocity = v_percent;
    next.type   = MOTION_LINE;
    next.cnt    = cnt_next;
    next.pose_target        = pose_next;
    next.linear_velocity    = v_next;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);

    if (err == SUCCESS) {
        log.info("  Joint path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() > 0) {
            return true;
        }
        else {
            err = TARGET_REPEATED;
            return false;
        }
    }
    else {
        log.error("  ERROR occurred during generating joint path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
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
// Out:     path(hidden)-> outputs added into planned_path_fifo_ automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveJ(const Joint &joint_target, double v_percent, double a_percent, int cnt,
                     const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err)
{
    return MoveJ(joint_target, v_percent, a_percent, cnt,
                 transformPoseEuler2Pose(pose_next), v_next, a_next, cnt_next,
                 id, err);
}

//------------------------------------------------------------------------------
// Function:    MoveJ (smooth to MoveC)
// Summary: To plan a path in joint space to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveJ(const Joint &joint_target, double v_percent, double a_percent, int cnt,
                     const Pose &pose1_circle, const Pose &pose2_circle,
                     double v_circle, double a_circle, int cnt_circle,
                     int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveJ (smooth to MoveC) request accepted, planning joint path...");
    if (!isMotionExecutable(MOTION_JOINT, err)) {
        log.error("  Cannot plan a path in joint space, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!checkJointBoundary(joint_target)) {
        err = TARGET_OUT_OF_CONSTRAINT;
        log.error("  Target point out of joint constraint, planning abort");
        unlockArmGroup();
        return false;
    }

    if (planning_interface_->setAlphaScaling(a_percent)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    MotionTarget target, next;
    target.type                 = MOTION_JOINT;
    target.cnt                  = cnt;
    target.joint_target         = joint_target;
    target.percent_velocity     = v_percent;
    next.type                   = MOTION_CIRCLE;
    next.cnt                    = cnt_circle;
    next.circle_target.pose1    = pose1_circle;
    next.circle_target.pose2    = pose2_circle;
    next.linear_velocity        = v_circle;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);
 
    if (err == SUCCESS) {
        log.info("  Joint path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() > 0) {
            return true;
        }
        else {
            err = TARGET_REPEATED;
            return false;
        }
    }
    else {
        log.error("  ERROR occurred during generating joint path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    MoveJ (smooth to MoveC)
// Summary: To plan a path in joint space to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveJ(const Joint &joint_target, double v_percent, double a_percent, int cnt,
                     const PoseEuler &pose2_circle, const PoseEuler &pose3_circle,
                     double v_circle, double a_circle, int cnt_circle,
                     int id, ErrorCode &err)
{
    MoveJ(joint_target, v_percent, a_percent, cnt,
          transformPoseEuler2Pose(pose2_circle), transformPoseEuler2Pose(pose3_circle),
          v_circle, a_circle, cnt_circle, id, err);
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
bool ArmGroup::MoveL(const Pose &pose, double velocity, double acceleration,
                     int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveL (without smooth) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_LINE, err)) {
        log.error("  Cannot plan a path in cartesian space, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!planning_interface_->setAcceleration(acceleration)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    MotionTarget target, next;
    target.type                 = MOTION_LINE;
    target.cnt                  = 0;
    target.pose_target          = pose;
    target.linear_velocity      = velocity;
    next.type   = MOTION_NONE;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);

    if (err == SUCCESS) {
        log.info("  Cartesian path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() > 0) {return true;}
        else {err = TARGET_REPEATED; return false;}
    }
    else {
        log.error("  ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
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
bool ArmGroup::MoveL(const PoseEuler &pose, double velocity, double acceleration,
                     int id, ErrorCode &err)
{
    return MoveL(transformPoseEuler2Pose(pose), velocity, acceleration, id, err);
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
// Out:     path(hidden)-> outputs added into planned_path_fifo_ automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveL(const Pose &pose, double velocity, double acceleration, int cnt,
                     const Joint &joint, double v_percent, double a_percent, int cnt_next,
                     int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveL (smooth to MoveJ) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_LINE, err)) {
        log.error("  Cannot plan a path in cartesian space, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!planning_interface_->setAcceleration(acceleration)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    MotionTarget target, next;
    target.type                 = MOTION_LINE;
    target.cnt                  = cnt;
    target.pose_target          = pose;
    target.linear_velocity      = velocity;
    next.type                   = MOTION_JOINT;
    next.cnt                    = cnt_next;
    next.joint_target           = joint;
    next.percent_velocity        = v_percent;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);

    if (err == SUCCESS) {
        log.info("  Cartesian path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() > 0) {return true;}
        else {err = TARGET_REPEATED; return false;}
    }
    else {
        log.error("  ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
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
// Out:     path(hidden)-> outputs added into planned_path_fifo_ automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveL(const PoseEuler &pose_target, double v_max, double a_max, int cnt_target,
                     const Joint &joint_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err)
{
    return MoveL(transformPoseEuler2Pose(pose_target), v_max, a_max, cnt_target,
                 joint_next, v_next, a_next, cnt_next, id, err);
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
bool ArmGroup::MoveL(const Pose &pose, double velocity, double acceleration, int cnt,
                     const Pose &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveL (smooth to MoveL) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_LINE, err)) {
        log.error("  Cannot plan a path in cartesian space, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!planning_interface_->setAcceleration(acceleration)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    MotionTarget target, next;
    target.type                 = MOTION_LINE;
    target.cnt                  = cnt;
    target.pose_target          = pose;
    target.linear_velocity      = velocity;
    next.type                   = MOTION_LINE;
    next.cnt                    = cnt_next;
    next.pose_target            = pose_next;
    next.linear_velocity        = v_next;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);
    
    if (err == SUCCESS) {
        log.info("  Cartesian path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() > 0) {return true;}
        else {err = TARGET_REPEATED; return false;}
    }
    else {
        log.error("  ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
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
bool ArmGroup::MoveL(const PoseEuler &pose, double velocity, double acceleration, int cnt,
                     const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err)
{
    return MoveL(transformPoseEuler2Pose(pose), velocity, acceleration, cnt,
                 transformPoseEuler2Pose(pose_next), v_next, a_next, cnt_next,
                 id, err);
}

//------------------------------------------------------------------------------
// Function:    MoveL (smooth to MoveC)
// Summary: To plan a linear path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveL(const Pose &pose, double velocity, double acceleration, int cnt,
                     const Pose &pose1_circle, const Pose &pose2_circle,
                     double v_circle, double a_circle, int cnt_circle,
                     int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveL (smooth to MoveC) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_LINE, err)) {
        log.error("  Cannot plan a path in cartesian space, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!planning_interface_->setAcceleration(acceleration)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    MotionTarget target, next;
    target.type                 = MOTION_LINE;
    target.cnt                  = cnt;
    target.pose_target          = pose;
    target.linear_velocity      = velocity;
    next.type                   = MOTION_CIRCLE;
    next.cnt                    = cnt_circle;
    next.circle_target.pose1    = pose1_circle;
    next.circle_target.pose2    = pose2_circle;
    next.linear_velocity        = v_circle;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);
    
    if (err == SUCCESS) {
        log.info("  Cartesian path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() > 0) {return true;}
        else {err = TARGET_REPEATED; return false;}
    }
    else {
        log.error("  ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    MoveL (smooth to MoveC)
// Summary: To plan a linear path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveL(const PoseEuler &pose, double velocity, double acceleration, int cnt,
                     const PoseEuler &pose1_circle, const PoseEuler &pose2_circle,
                     double v_circle, double a_circle, int cnt_circle,
                     int id, ErrorCode &err)
{
    MoveL(transformPoseEuler2Pose(pose), velocity, acceleration, cnt,
          transformPoseEuler2Pose(pose1_circle), transformPoseEuler2Pose(pose2_circle),
          v_circle, a_circle, cnt_circle, id, err);
}

//------------------------------------------------------------------------------
// Function:    MoveC (without smooth)
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const Pose pose1, const Pose pose2, double velocity, double acceleration, int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveC (without smooth) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_CIRCLE, err)) {
        log.error("  Cannot plan a path in cartesian space, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!planning_interface_->setAcceleration(acceleration)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    MotionTarget target, next;
    target.type                 = MOTION_CIRCLE;
    target.cnt                  = 0;
    target.circle_target.pose1  = pose1;
    target.circle_target.pose2  = pose2;
    target.linear_velocity      = velocity;
    next.type                   = MOTION_NONE;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);
    
    if (err == SUCCESS) {
        log.info("  Cartesian path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() > 0) {return true;}
        else {err = TARGET_REPEATED; return false;}
    }
    else {
        log.error("  ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
        return false;
    }
}


//------------------------------------------------------------------------------
// Function:    MoveC (without smooth)
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const PoseEuler pose1, const PoseEuler pose2,
                     double velocity, double acceleration,
                     int id, ErrorCode &err)
{
    return MoveC(transformPoseEuler2Pose(pose1), transformPoseEuler2Pose(pose2),
                 velocity, acceleration, id, err);
}


//------------------------------------------------------------------------------
// Function:    MoveC (smooth to MoveJ)
// Summary: To plan a circle path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const Pose &pose1, const Pose &pose2,
                     double velocity, double acceleration, int cnt,
                     const Joint &joint, double v_percent, double a_percent, int cnt_next,
                     int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveC (smooth to MoveJ) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_CIRCLE, err)) {
        log.error("  Cannot plan a path in cartesian space, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!planning_interface_->setAcceleration(acceleration)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    MotionTarget target, next;
    target.type                 = MOTION_CIRCLE;
    target.cnt                  = cnt;
    target.circle_target.pose1  = pose1;
    target.circle_target.pose2  = pose2;
    target.linear_velocity      = velocity;
    next.type                   = MOTION_JOINT;
    next.cnt                    = cnt_next;
    next.joint_target           = joint;
    next.percent_velocity       = v_percent;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);
    
    if (err == SUCCESS) {
        log.info("  Cartesian path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() > 0) {return true;}
        else {err = TARGET_REPEATED; return false;}
    }
    else {
        log.error("  ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    MoveC (smooth to MoveJ)
// Summary: To plan a circle path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const PoseEuler &pose1, const PoseEuler &pose2,
                     double velocity, double acceleration, int cnt,
                     const Joint &joint, double v_percent, double a_percent, int cnt_next,
                     int id, ErrorCode &err)
{
    MoveC(transformPoseEuler2Pose(pose1), transformPoseEuler2Pose(pose2),
          velocity, acceleration, cnt,
          joint, v_percent, a_percent, cnt_next, id, err);
}

//------------------------------------------------------------------------------
// Function:    MoveC (smooth to MoveL)
// Summary: To plan a circle path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const Pose &pose1, const Pose &pose2,
                     double velocity, double acceleration, int cnt,
                     const Pose &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveC (smooth to MoveL) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_CIRCLE, err)) {
        log.error("  Cannot plan a path in cartesian space, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!planning_interface_->setAcceleration(acceleration)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    MotionTarget target, next;
    target.type                 = MOTION_CIRCLE;
    target.cnt                  = cnt;
    target.circle_target.pose1  = pose1;
    target.circle_target.pose2  = pose2;
    target.linear_velocity      = velocity;
    next.type                   = MOTION_LINE;
    next.cnt                    = cnt_next;
    next.pose_target            = pose_next;
    next.linear_velocity        = v_next;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);
    
    if (err == SUCCESS) {
        log.info("  Cartesian path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() > 0) {return true;}
        else {err = TARGET_REPEATED; return false;}
    }
    else {
        log.error("  ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    MoveC (smooth to MoveL)
// Summary: To plan a circle path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const PoseEuler &pose1, const PoseEuler &pose2,
                     double velocity, double acceleration, int cnt,
                     const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err)
{
    MoveC(transformPoseEuler2Pose(pose1), transformPoseEuler2Pose(pose2),
          velocity, acceleration, cnt,
          transformPoseEuler2Pose(pose_next), v_next, a_next, cnt_next,
          id, err);
}

//------------------------------------------------------------------------------
// Function:    MoveC (smooth to MoveC)
// Summary: To plan a circle path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const Pose &pose1, const Pose &pose2,
                     double velocity, double acceleration, int cnt,
                     const Pose &pose3, const Pose &pose4,
                     double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err)
{
    err = SUCCESS;
    if (current_state_ != INITIALIZED) {err = NEED_INITIALIZATION; return false;}

    lockArmGroup();
    log.info("MoveC (smooth to MoveC) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_CIRCLE, err)) {
        log.error("  Cannot plan a path in cartesian space, error_code=0x%llx, planning abort", err);
        unlockArmGroup();
        return false;
    }

    if (!setMaxAcceleration(acceleration)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid acceleration, planning abort");
        unlockArmGroup();
        return false;
    }

    MotionTarget target, next;
    target.type                 = MOTION_CIRCLE;
    target.cnt                  = cnt;
    target.circle_target.pose1  = pose1;
    target.circle_target.pose2  = pose2;
    target.linear_velocity      = velocity;
    next.type                   = MOTION_CIRCLE;
    next.cnt                    = cnt_next;
    next.circle_target.pose1    = pose3;
    next.circle_target.pose2    = pose4;
    next.linear_velocity        = v_next;
    MoveCommand *motion = planning_interface_->createMotionCommand(id, target, next, err);
    
    if (err == SUCCESS) {
        log.info("  Cartesian path generated successfully with %zd points.", motion->getTrajLength());
        if (trajectory_fifo_.size() < trajectory_fifo_dynamic_length_) {
            if (!suspend_state_.is_suspended) {
                convertPath2Trajectory(20, err);
            }
        }
        unlockArmGroup();
        if (motion->getTrajLength() > 0) {return true;}
        else {err = TARGET_REPEATED; return false;}
    }
    else {
        log.error("  ERROR occurred during generating cartesian path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
        unlockArmGroup();
        return false;
    }
}

//------------------------------------------------------------------------------
// Function:    MoveC (smooth to MoveC)
// Summary: To plan a circle path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const PoseEuler &pose1, const PoseEuler &pose2,
                     double velocity, double acceleration, int cnt,
                     const PoseEuler &pose3, const PoseEuler &pose4,
                     double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    MoveC(transformPoseEuler2Pose(pose1), transformPoseEuler2Pose(pose2),
          velocity, acceleration, cnt,
          transformPoseEuler2Pose(pose3), transformPoseEuler2Pose(pose4),
          v_next, a_next, cnt_next,
          id, err);
}


}
