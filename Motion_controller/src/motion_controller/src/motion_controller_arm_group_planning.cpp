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
// Function:    MoveJ
// Summary: To plan a path in joint space to touch target pose, without smooth.
// In:      joint_target-> target in joint space
//          v_max   -> max velocity
//          a_max   -> max acceleration
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_JOINT && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }

    lockArmGroup();
    log.info("MoveJ (without smooth) request accepted, planning joint path...");
    if (!isMotionExecutable(MOTION_JOINT)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan a path in joint space, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!checkJointBoundary(joint_target)) {
        err = TARGET_OUT_OF_CONSTRAINT;
        log.error("  Target point out of joint constraints, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<JointValues> planned_path;
    JointPoint jp_target;
    jp_target.joints = joint_target;
    memset(&jp_target.omegas, 0, sizeof(JointOmegas));

    bool res = planning_interface_->MoveJ2J(m_joint_start,
                                            jp_target,
                                            jp_target,
                                            v_max,
                                            0,
                                            planned_path,
                                            err);

    if (res) {
        if (0 == planned_path.size()) {
            log.warn("  Joint path generated with 0 point, are we standing on the target point already?");
            unlockArmGroup();
            return true;
        }

        if (setLatestIKReference(planned_path.back(), err)) {
            PathPoint pp;
            pp.type = MOTION_JOINT;

            if (last_motion_.smooth_type == SMOOTH_NONE) {
                pp.id = (id << 2) + POINT_START;
                pp.joints = planned_path.front();
                m_planned_path_fifo.push_back(pp);
                pp.id = (id << 2) + POINT_MIDDLE;
                
                std::vector<JointValues>::iterator itr = planned_path.begin() + 1;
                for ( ; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
                m_planned_path_fifo.back().id = (id << 2) + POINT_ENDING;
            }
            else {
                pp.id = (id << 2) + POINT_MIDDLE;
                
                std::vector<JointValues>::iterator itr = planned_path.begin();
                for ( ; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
                m_planned_path_fifo.back().id = (id << 2) + POINT_ENDING;
            }

            allowed_motion_type_ = MOTION_UNDEFINED;

            computeFK(planned_path.back(), m_pose_start, err);
            m_pose_previous = m_pose_start;
            m_vu_start = 0;
            m_v_start = 0;

            log.info("  Joint path generated successfully with %zd points, and added into planned_path_FIFO.",
                     planned_path.size());

            last_motion_.id = id;
            last_motion_.addition_smooth = false;
            last_motion_.smooth_type = SMOOTH_NONE;
            last_motion_.motion_type = MOTION_JOINT;

            unlockArmGroup();
            return true;
        }
        else {
            log.error("  Cannot set new IK reference, error code=0x%llx, planning abort", err);
            unlockArmGroup();
            return false;
        }
    }
    else if (err == TARGET_REPEATED) {
        unlockArmGroup();
        log.warn("  Target repeated, waiting for next target, error code=0x%llx", err);
        return false;
    }
    else {
        unlockArmGroup();
        log.error("  ERROR occurred during generating joint path, error code:0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
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
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
                     const JointValues &joint_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_JOINT && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }
    if (cnt < 0 || cnt > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    lockArmGroup();
    log.info("MoveJ (smooth to MoveJ) request accepted, planning joint path...");
    if (!isMotionExecutable(MOTION_JOINT)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan a path in joint space, planning abort");
        unlockArmGroup();
        return false;
    }
    
    if (!checkJointBoundary(joint_target) || !checkJointBoundary(joint_next)) {
        err = TARGET_OUT_OF_CONSTRAINT;
        log.error("  Target point out of joint constraints, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<JointValues> planned_path;
    JointPoint jp_target, jp_next;
    jp_target.joints = joint_target;
    jp_next.joints   = joint_next;
    memset(&jp_target.omegas, 0, sizeof(JointOmegas));
    memset(&jp_next.omegas,   0, sizeof(JointOmegas));

    bool res = planning_interface_->MoveJ2J(m_joint_start,
                                            jp_target,
                                            jp_next,
                                            v_max,
                                            cnt,
                                            planned_path,
                                            err);

    if (res) {
        if (0 == planned_path.size()) {
            log.warn("  Joint path generated with 0 point, are we standing on the target point already?");
            unlockArmGroup();
            return true;
        }

        if (setLatestIKReference(planned_path.back(), err)) {
            PathPoint pp;
            pp.type = MOTION_JOINT;

            if (last_motion_.smooth_type == SMOOTH_NONE) {
                pp.id = (id << 2) + POINT_START;
                pp.joints = planned_path.front();
                m_planned_path_fifo.push_back(pp);
                pp.id = (id << 2) + POINT_MIDDLE;

                std::vector<JointValues>::iterator itr = planned_path.begin() + 1;
                for( ; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }
            else {
                pp.id = (id << 2) + POINT_MIDDLE;
                
                std::vector<JointValues>::iterator itr = planned_path.begin();
                for ( ; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }

            allowed_motion_type_ = MOTION_JOINT;

            log.info("  Joint path generated successfully with %zd points, and added into planned_path_FIFO.",
                     planned_path.size());

            last_motion_.id = id;
            last_motion_.addition_smooth = false;
            last_motion_.smooth_type = SMOOTH_J2J;
            last_motion_.motion_type = MOTION_JOINT;
            
            unlockArmGroup();
            return true;
        }
        else {
            log.error("  Cannot set new IK reference, error code=0x%llx, planning abort", err);
            unlockArmGroup();
            return false;
        }
    }
    else if (err == TARGET_REPEATED) {
        log.warn("  Target repeated, waiting for next target, error code=0x%llx", err);
        unlockArmGroup();
        return false;
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
//          cnt_next    -> smooth degree in the next path
//          id      -> command id
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
                     const Pose &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_JOINT && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }
    if (cnt < 0 || cnt > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    lockArmGroup();
    log.info("MoveJ (smooth to MoveL) request accepted, planning joint path...");
    if (!isMotionExecutable(MOTION_JOINT)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan a path in joint space, planning abort");
        unlockArmGroup();
        return false;
    }
    if (!checkJointBoundary(joint_target)) {
        err = TARGET_OUT_OF_CONSTRAINT;
        log.error("  Target point out of joint constraints, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<JointValues> planned_path;
    JointValues j_target = joint_target;

    bool res = planning_interface_->MoveJ2L(m_joint_start,
                                            j_target,
                                            v_max,
                                            cnt,
                                            pose_next,
                                            v_next,
                                            cnt_next,
                                            planned_path,
                                            m_pose_start,
                                            m_pose_previous,
                                            m_v_start,
                                            m_vu_start,
                                            err);

    if (res) {
        if (0 == planned_path.size()) {
            log.warn("  Joint path generated with 0 point, are we standing on the target point already?");
            unlockArmGroup();
            return true;
        }

        if (setLatestIKReference(planned_path.back(), err)) {
            PathPoint pp;
            pp.type = MOTION_JOINT;

            if (last_motion_.smooth_type == SMOOTH_NONE) {
                pp.id = (id << 2) + POINT_START;
                pp.joints = planned_path.front();
                m_planned_path_fifo.push_back(pp);
                pp.id = (id << 2) + POINT_MIDDLE;

                vector<JointValues>::iterator itr = planned_path.begin() + 1;
                for ( ; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }
            else {
                pp.id = (id << 2) + POINT_MIDDLE;

                vector<JointValues>::iterator itr = planned_path.begin();
                for ( ; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }

            allowed_motion_type_ = MOTION_CIRCLE;

            log.info("  Joint path generated successfully with %zd points, and added into planned_path_FIFO.",
                     planned_path.size());

            last_motion_.id = id;
            last_motion_.addition_smooth = false;
            last_motion_.smooth_type = SMOOTH_J2L;
            last_motion_.motion_type = MOTION_JOINT;
            
            unlockArmGroup();
            return true;
        }
        else {
            log.error("  Cannot set new IK reference, error code=0x%llx, planning abort", err);
            unlockArmGroup();
            return false;
        }
    }
    else if (err == TARGET_REPEATED) {
        unlockArmGroup();
        log.warn("  Target repeated, waiting for next target, error code=0x%llx", err);
        return false;
    }
    else {
        unlockArmGroup();
        log.error("  ERROR occurred during generating joint path, error code=0x%llx", err);
        log.error("  Invalid path obtained and dropped.");
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
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
                     const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    return MoveJ(joint_target, v_max, a_max, cnt,
                 transformPoseEuler2Pose(pose_next), v_next, a_next, cnt_next,
                 id, err);
}

//------------------------------------------------------------------------------
// Function:    MoveJ (smooth to MoveC)
// Summary: To plan a path in joint space to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
                     const Pose &pose2_circle, const Pose &pose3_circle,
                     double v_circle, double a_circle, int cnt_circle,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_JOINT && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }
    if (cnt < 0 || cnt > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    lockArmGroup();
    log.info("MoveJ (smooth to MoveC) request accepted, planning joint path...");
    if (!isMotionExecutable(MOTION_JOINT)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan a path in joint space, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!checkJointBoundary(joint_target)) {
        err = TARGET_OUT_OF_CONSTRAINT;
        log.error("  Target point out of joint constraints, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<JointValues> planned_path;

    bool res = planning_interface_->MoveJ2C(m_joint_start,
                                            joint_target,
                                            v_max,
                                            cnt,
                                            pose2_circle,
                                            pose3_circle,
                                            v_circle,
                                            planned_path,
                                            m_pose_start,
                                            m_v_start,
                                            err);

    if (res) {
        if (0 == planned_path.size()) {
            log.warn("  Joint path generated with 0 point, are we standing on the target point already?");
            unlockArmGroup();
            return true;
        }

        if (setLatestIKReference(planned_path.back(), err)) {
            PathPoint pp;
            pp.type = MOTION_JOINT;

            if (last_motion_.smooth_type == SMOOTH_NONE) {
                pp.id = (id << 2) + POINT_START;
                pp.joints = planned_path.front();
                m_planned_path_fifo.push_back(pp);
                pp.id = (id << 2) + POINT_MIDDLE;

                vector<JointValues>::iterator itr = planned_path.begin() + 1;
                for ( ; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }
            else {
                pp.id = (id << 2) + POINT_MIDDLE;

                vector<JointValues>::iterator itr = planned_path.begin();
                for ( ; itr != planned_path.end(); ++itr) {
                    pp.joints = *itr;
                    m_planned_path_fifo.push_back(pp);
                }
            }

            allowed_motion_type_ = MOTION_CIRCLE;

            log.info("  Joint path generated successfully with %zd points, and added into planned_path_FIFO.",
                     planned_path.size());

            last_motion_.id = id;
            last_motion_.addition_smooth = false;
            last_motion_.smooth_type = SMOOTH_J2C;
            last_motion_.motion_type = MOTION_JOINT;
            
            unlockArmGroup();
            return true;
        }
        else {
            log.error("  Cannot set new IK reference, error code=0x%llx, planning abort", err);
            unlockArmGroup();
            return false;
        }
    }
    else if (err == TARGET_REPEATED) {
        log.warn("  Target repeated, waiting for next target, error code=0x%llx", err);
        unlockArmGroup();
        return false;
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
bool ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
                     const PoseEuler &pose2_circle, const PoseEuler &pose3_circle,
                     double v_circle, double a_circle, int cnt_circle,
                     int id, ErrorCode &err) {
    MoveJ(joint_target, v_max, a_max, cnt,
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
bool ArmGroup::MoveL(const Pose &pose_target, double v_max, double a_max,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_LINE && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }

    lockArmGroup();
    log.info("MoveL (without smooth) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_LINE)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan additional smooth path, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<Pose> planned_path;
    bool res = planning_interface_->MoveL2J(m_pose_start,
                                            m_v_start,
                                            m_vu_start,
                                            pose_target,
                                            v_max,
                                            0,
                                            m_pose_previous,
                                            planned_path,
                                            err);

    if (res) {
        PathPoint pp;
        pp.type = MOTION_LINE;

        if (last_motion_.smooth_type == SMOOTH_NONE) {
            pp.id = (id << 2) + POINT_START;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin() + 1;
            for ( ; itr != planned_path.end(); ++itr) {
		        /*
		        // ------------------- FOR TEST ONLY --------------------
		        ROS_INFO("Pose :%f,%f,%f,%f,%f,%f,%f",
                         itr->position.x,
                         itr->position.y,
                         itr->position.z,
                         itr->orientation.w,
                         itr->orientation.x,
                         itr->orientation.y,
                         itr->orientation.z);
		        // ^^^^^^^^^^^^^^^^^^^ FOR TEST ONLY ^^^^^^^^^^^^^^^^^^^^
		        */
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin();
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }

        m_planned_path_fifo.back().id = (id << 2) + POINT_ENDING;
        allowed_motion_type_ = MOTION_UNDEFINED;

        log.info("  Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());

        last_motion_.id = id;
        last_motion_.addition_smooth = false;
        last_motion_.smooth_type = SMOOTH_NONE;
        last_motion_.motion_type = MOTION_LINE;
            
        unlockArmGroup();
        return true;
    }
    else if (err == TARGET_REPEATED) {
        log.warn("  Target repeated, waiting for next target, error code=0x%llx", err);
        unlockArmGroup();
        return false;
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
bool ArmGroup::MoveL(const PoseEuler &pose_target, double v_max, double a_max,
                     int id, ErrorCode &err) {
    return MoveL(transformPoseEuler2Pose(pose_target), v_max, a_max, id, err);
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
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveL(const Pose &pose_target, double v_max, double a_max, int cnt_target,
                     const JointValues &joint_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_LINE && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }

    // if (cnt_target <= 0 || cnt_target > 100) {
    if (cnt_target < 0 || cnt_target > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    lockArmGroup();
    log.info("MoveL (smooth to MoveJ) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_LINE)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan additional smooth path, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<Pose> planned_path;
    bool res = planning_interface_->MoveL2J(m_pose_start,
                                            m_v_start,
                                            m_vu_start,
                                            pose_target,
                                            v_max,
                                            cnt_target,
                                            m_pose_previous,
                                            planned_path,
                                            err);

    if (res) {
        fst_controller::PathPoint pp;
        pp.type = MOTION_LINE;

        if (last_motion_.smooth_type == SMOOTH_NONE) {
            pp.id = (id << 2) + POINT_START;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin() + 1;
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin();
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }

        allowed_motion_type_ = MOTION_JOINT;

        log.info("  Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());
        
        last_motion_.id = id;
        last_motion_.addition_smooth = false;
        last_motion_.smooth_type = SMOOTH_L2J;
        last_motion_.motion_type = MOTION_LINE;
            
        unlockArmGroup();
        return true;
    }
    else if (err == TARGET_REPEATED) {
        log.warn("  Target repeated, waiting for next target, error code=0x%llx", err);
        unlockArmGroup();
        return false;
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
// Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//          error_code  -> error code
// Return:  true    -> plan successfully
//------------------------------------------------------------------------------
bool ArmGroup::MoveL(const PoseEuler &pose_target, double v_max, double a_max, int cnt_target,
                     const JointValues &joint_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
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
bool ArmGroup::MoveL(const Pose &pose_target, double v_max, double a_max, int cnt_target,
                     const Pose &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_LINE && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }

    // if (cnt_target <= 0 || cnt_target > 100) {
    if (cnt_target < 0 || cnt_target > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    lockArmGroup();
    log.info("MoveL (smooth to MoveL) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_LINE)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan additional smooth path, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<Pose> planned_path;
    bool res = planning_interface_->MoveL2L(m_pose_start,
                                            m_v_start,
                                            m_vu_start,
                                            pose_target,
                                            v_max,
                                            cnt_target,
                                            pose_next,
                                            v_next,
                                            m_pose_previous,
                                            planned_path,
                                            err);
    
    if (res) {
        PathPoint pp;
        pp.type = MOTION_LINE;

        if (last_motion_.smooth_type == SMOOTH_NONE) {
            pp.id = (id << 2) + POINT_START;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin() + 1;
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin();
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }

        allowed_motion_type_ = MOTION_LINE;

        log.info("  Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());

        last_motion_.id = id;
        last_motion_.addition_smooth = false;
        last_motion_.smooth_type = SMOOTH_L2L;
        last_motion_.motion_type = MOTION_LINE;
            
        unlockArmGroup();
        return true;
    }
    else if (err == TARGET_REPEATED) {
        log.warn("  Target repeated, waiting for next target, error code=0x%llx", err);
        unlockArmGroup();
        return false;
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
bool ArmGroup::MoveL(const PoseEuler &pose_target, double v_max, double a_max, int cnt_target,
                     const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    return MoveL(transformPoseEuler2Pose(pose_target), v_max, a_max, cnt_target,
                 transformPoseEuler2Pose(pose_next), v_next, a_next, cnt_next,
                 id, err);
}


//------------------------------------------------------------------------------
// Function:    MoveL (smooth to MoveC)
// Summary: To plan a linear path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveL(const Pose &pose_target, double v_max, double a_max, int cnt_target,
                     const Pose &pose2_circle, const Pose &pose3_circle,
                     double v_circle, double a_circle, int cnt_circle,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_LINE && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }
    if (cnt_target < 0 || cnt_target > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    lockArmGroup();
    log.info("MoveL (smooth to MoveC) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_LINE)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan additional smooth path, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<Pose> planned_path;
    bool res = planning_interface_->MoveL2C(m_pose_start,
                                            m_v_start,
                                            m_vu_start,
                                            pose_target,
                                            v_max,
                                            cnt_target,
                                            pose2_circle,
                                            pose3_circle,
                                            v_circle,
                                            m_pose_previous,
                                            m_pose_start_past,
                                            planned_path,
                                            err);

    if (res) {
        PathPoint pp;
        pp.type = MOTION_LINE;

        if (last_motion_.smooth_type == SMOOTH_NONE) {
            pp.id = (id << 2) + POINT_START;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin() + 1;
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin();
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }

        allowed_motion_type_ = MOTION_CIRCLE;

        log.info("  Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());

        last_motion_.id = id;
        last_motion_.addition_smooth = true;
        last_motion_.smooth_type = SMOOTH_L2C;
        last_motion_.motion_type = MOTION_LINE;
            
        unlockArmGroup();
        return true;
    }
    else if (err == TARGET_REPEATED) {
        log.warn("  Target repeated, waiting for next target, error code=0x%llx", err);
        unlockArmGroup();
        return false;
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
bool ArmGroup::MoveL(const PoseEuler &pose_target, double v_max, double a_max, int cnt_target,
                     const PoseEuler &pose2_circle, const PoseEuler &pose3_circle,
                     double v_circle, double a_circle, int cnt_circle,
                     int id, ErrorCode &err) {
    MoveL(transformPoseEuler2Pose(pose_target), v_max, a_max, cnt_target,
          transformPoseEuler2Pose(pose2_circle), transformPoseEuler2Pose(pose3_circle),
          v_circle, a_circle, cnt_circle, id, err);
}

//------------------------------------------------------------------------------
// Function:    MoveC (without smooth)
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const Pose pose_2nd, const Pose pose_3rd,
                     double v_target, double a_target,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_CIRCLE && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }

    lockArmGroup();
    log.info("MoveC (without smooth) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_CIRCLE)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan additional smooth path, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_target) || !setMaxAcceleration(a_target)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<Pose> planned_path;
    bool res = planning_interface_->MoveC2J(m_pose_start,
                                            m_v_start,
                                            pose_2nd,
                                            pose_3rd,
                                            v_target,
                                            0,
                                            planned_path,
                                            err);

    if (res) {
        PathPoint pp;
        pp.type = MOTION_CIRCLE;

        if (last_motion_.smooth_type == SMOOTH_NONE) {
            pp.id = (id << 2) + POINT_START;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin() + 1;
            for ( ; itr != planned_path.end(); ++itr) {
		        /*
		        // ------------------- FOR TEST ONLY --------------------
		        ROS_INFO("Pose :%f,%f,%f,%f,%f,%f,%f",
                         itr->position.x,
                         itr->position.y,
                         itr->position.z,
                         itr->orientation.w,
                         itr->orientation.x,
                         itr->orientation.y,
                         itr->orientation.z);
		        // ^^^^^^^^^^^^^^^^^^^ FOR TEST ONLY ^^^^^^^^^^^^^^^^^^^^
		        */
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin();
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }

        m_planned_path_fifo.back().id = (id << 2) + POINT_ENDING;
        allowed_motion_type_ = MOTION_UNDEFINED;

        log.info("  Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());
        
        last_motion_.id = id;
        last_motion_.addition_smooth = false;
        last_motion_.smooth_type = SMOOTH_NONE;
        last_motion_.motion_type = MOTION_CIRCLE;
            
        unlockArmGroup();
        return true;                                            
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
bool ArmGroup::MoveC(const PoseEuler pose_2nd, const PoseEuler pose_3rd,
                     double v_target, double a_target,
                     int id, ErrorCode &err) {
    return MoveC(transformPoseEuler2Pose(pose_2nd), transformPoseEuler2Pose(pose_3rd),
                 v_target, a_target, id, err);
}


//------------------------------------------------------------------------------
// Function:    MoveC (smooth to MoveJ)
// Summary: To plan a circle path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const Pose &pose2_circle, const Pose &pose3_circle,
                     double v_max, double a_max, int cnt_target,
                     const JointValues &joint_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_CIRCLE && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }
    if (cnt_target < 0 || cnt_target > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    lockArmGroup();
    log.info("MoveC (smooth to MoveJ) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_CIRCLE)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan additional smooth path, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<Pose> planned_path;
    bool res = planning_interface_->MoveC2J(m_pose_start,
                                            m_v_start,
                                            pose2_circle,
                                            pose3_circle,
                                            v_max,
                                            cnt_target,
                                            planned_path,
                                            err);

    if (res) {
        fst_controller::PathPoint pp;
        pp.type = MOTION_CIRCLE;

        if (last_motion_.smooth_type == SMOOTH_NONE) {
            pp.id = (id << 2) + POINT_START;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin() + 1;
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin();
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }

        allowed_motion_type_ = MOTION_JOINT;

        log.info("  Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());
        
        last_motion_.id = id;
        last_motion_.addition_smooth = false;
        last_motion_.smooth_type = SMOOTH_C2J;
        last_motion_.motion_type = MOTION_CIRCLE;
            
        unlockArmGroup();
        return true;
    }
    else if (err == TARGET_REPEATED) {
        log.warn("  Target repeated, waiting for next target, error code=0x%llx", err);
        unlockArmGroup();
        return false;
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
bool ArmGroup::MoveC(const PoseEuler &pose2_circle, const PoseEuler &pose3_circle,
                     double v_max, double a_max, int cnt_target,
                     const JointValues &joint_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    MoveC(transformPoseEuler2Pose(pose2_circle), transformPoseEuler2Pose(pose3_circle),
          v_max, a_max, cnt_target,
          joint_next, v_next, a_next, cnt_next, id, err);
}

//------------------------------------------------------------------------------
// Function:    MoveC (smooth to MoveL)
// Summary: To plan a circle path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const Pose &pose2_circle, const Pose &pose3_circle,
                     double v_max, double a_max, int cnt_target,
                     const Pose &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_CIRCLE && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }
    if (cnt_target < 0 || cnt_target > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    lockArmGroup();
    log.info("MoveC (smooth to MoveL) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_CIRCLE)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan additional smooth path, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<Pose> planned_path;
    bool res = planning_interface_->MoveC2L(m_pose_start,
                                            m_v_start,
                                            pose2_circle,
                                            pose3_circle,
                                            v_max,
                                            cnt_target,
                                            pose_next,
                                            v_next,
                                            m_pose_start_past,
                                            m_pose_previous,
                                            m_vu_start,
                                            planned_path,
                                            err);

    if (res) {
        PathPoint pp;
        pp.type = MOTION_CIRCLE;

        if (last_motion_.smooth_type == SMOOTH_NONE) {
            pp.id = (id << 2) + POINT_START;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin() + 1;
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin();
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }

        allowed_motion_type_ = MOTION_LINE;

        log.info("  Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());

        last_motion_.id = id;
        last_motion_.addition_smooth = true;
        last_motion_.smooth_type = SMOOTH_C2L;
        last_motion_.motion_type = MOTION_CIRCLE;
            
        unlockArmGroup();
        return true;
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
bool ArmGroup::MoveC(const PoseEuler &pose2_circle, const PoseEuler &pose3_circle,
                     double v_max, double a_max, int cnt_target,
                     const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    MoveC(transformPoseEuler2Pose(pose2_circle), transformPoseEuler2Pose(pose3_circle),
          v_max, a_max, cnt_target,
          transformPoseEuler2Pose(pose_next), v_next, a_next, cnt_next,
          id, err);
}

//------------------------------------------------------------------------------
// Function:    MoveC (smooth to MoveC)
// Summary: To plan a circle path to touch target pose, with smooth.
//------------------------------------------------------------------------------
bool ArmGroup::MoveC(const Pose &pose2_circle, const Pose &pose3_circle,
                     double v_max, double a_max, int cnt_target,
                     const Pose &pose4_circle, const Pose &pose5_circle,
                     double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    err = SUCCESS;

    if (current_state_ != INITIALIZED) {
        err = NEED_INITIALIZATION;
        return false;
    }
    if (enable_calibration_ == true && calibrator_->getCurrentState() < CALIBRATED) {
        err = NEED_CALIBRATION;
        return false;
    }
    if (allowed_motion_type_ != MOTION_CIRCLE && allowed_motion_type_ != MOTION_UNDEFINED) {
        err = INVALID_SEQUENCE;
        return false;
    }
    if (cnt_target < 0 || cnt_target > 100) {
        err = INVALID_PARAMETER;
        return false;
    }

    lockArmGroup();
    log.info("MoveC (smooth to MoveC) request accepted, planning cartesian path...");
    if (!isMotionExecutable(MOTION_CIRCLE)) {
        err = CARTESIAN_PATH_EXIST;
        log.error("  Cartesian points exist, cannot plan additional smooth path, planning abort");
        unlockArmGroup();
        return false;
    }

    if (!setMaxVelocity(v_max) || !setMaxAcceleration(a_max)) {
        err = INVALID_PARAMETER;
        log.error("  Get invalid parameter, planning abort");
        unlockArmGroup();
        return false;
    }

    std::vector<Pose> planned_path;
    bool res = planning_interface_->MoveC2C(m_pose_start,
                                            m_v_start,
                                            pose2_circle,
                                            pose3_circle,
                                            v_max,
                                            cnt_target,
                                            pose4_circle,
                                            pose5_circle,
                                            v_next,
                                            m_pose_start_past,
                                            planned_path,
                                            err);

    if (res) {
        PathPoint pp;
        pp.type = MOTION_CIRCLE;

        if (last_motion_.smooth_type == SMOOTH_NONE) {
            pp.id = (id << 2) + POINT_START;
            pp.pose = planned_path.front();
            m_planned_path_fifo.push_back(pp);
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin() + 1;
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }
        else {
            pp.id = (id << 2) + POINT_MIDDLE;

            vector<Pose>::iterator itr = planned_path.begin();
            for ( ; itr != planned_path.end(); ++itr) {
                pp.pose = *itr;
                m_planned_path_fifo.push_back(pp);
            }
        }

        allowed_motion_type_ = MOTION_CIRCLE;

        log.info("  Cartesian path generated successfully with %zd points, and added into planned_path_FIFO.",
                 planned_path.size());

        last_motion_.id = id;
        last_motion_.addition_smooth = true;
        last_motion_.smooth_type = SMOOTH_C2C;
        last_motion_.motion_type = MOTION_CIRCLE;
            
        unlockArmGroup();
        return true;
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
bool ArmGroup::MoveC(const PoseEuler &pose2_circle, const PoseEuler &pose3_circle,
                     double v_max, double a_max, int cnt_target,
                     const PoseEuler &pose4_circle, const PoseEuler &pose5_circle,
                     double v_next, double a_next, int cnt_next,
                     int id, ErrorCode &err) {
    MoveC(transformPoseEuler2Pose(pose2_circle), transformPoseEuler2Pose(pose3_circle),
          v_max, a_max, cnt_target,
          transformPoseEuler2Pose(pose4_circle), transformPoseEuler2Pose(pose5_circle),
          v_next, a_next, cnt_next,
          id, err);
}


}
