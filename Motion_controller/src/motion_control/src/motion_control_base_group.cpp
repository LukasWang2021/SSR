/*************************************************************************
	> File Name: motion_control_base_group.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 11时25分56秒
 ************************************************************************/

#include <unistd.h>
#include <string.h>
#include <vector>

#include <motion_control_base_group.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <trajectory_alg.h>
#include <common_file_path.h>
#include <basic_alg.h>

using namespace std;
using namespace fst_base;
using namespace basic_alg;
using namespace fst_parameter;

namespace fst_mc
{


BaseGroup::BaseGroup(fst_log::Logger* plog)
{
    group_state_ = STANDBY;
    log_ptr_ = plog;
    auto_time_ = 0;
    manual_time_ = 0;
    manual_frame_ = JOINT;

    auto_pick_ptr_ = NULL;
    auto_cache_ptr_ = NULL;
    auto_pick_segment_ = 0;

    vel_ratio_ = 0;
    acc_ratio_ = 0;

    waiting_fine_ = false;
}

BaseGroup::~BaseGroup()
{
    if (auto_cache_ptr_ != NULL)
    {
        delete [] auto_cache_ptr_;
        auto_cache_ptr_ = NULL;
    }
}

void BaseGroup::reportError(const ErrorCode &error)
{
    error_monitor_ptr_->add(error);
}

ErrorCode BaseGroup::resetGroup(void)
{
    return bare_core_.resetBareCore() == true ? SUCCESS : BARE_CORE_TIMEOUT;
}

ErrorCode BaseGroup::stopGroup(void)
{
    if (group_state_ == MANUAL || (group_state_ & 0xF) == MANUAL || ((group_state_ >> 4) & 0xF) == MANUAL)
    {
        group_state_ = STANDBY;
        manual_time_ = 0;
        memset(&manual_traj_, 0, sizeof(manual_traj_));

        if (!bare_core_.isPointCacheEmpty())
        {
            bare_core_.clearPointCache();
        }
    }

    return bare_core_.stopBareCore() == true ? SUCCESS : BARE_CORE_TIMEOUT;
}

ErrorCode BaseGroup::clearGroup(void)
{
    FST_INFO("Clear group, current group state = %d", group_state_);

    group_state_ = STANDBY;
    manual_time_ = 0;
    memset(&manual_traj_, 0, sizeof(manual_traj_));

    if (!bare_core_.isPointCacheEmpty())
    {
        bare_core_.clearPointCache();
    }

    return SUCCESS;
}



ManualFrame BaseGroup::getManualFrame(void)
{
    //FST_INFO("Get manual frame = %d", manual_frame_);
    return manual_frame_;
}

ErrorCode BaseGroup::setManualFrame(ManualFrame frame)
{
    FST_INFO("Set manual frame = %d, current frame is %d", frame, manual_frame_);

    if (frame != manual_frame_)
    {
        if (group_state_ == STANDBY)
        {
            manual_frame_ = frame;
            FST_INFO("Done.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Cannot set frame in current state = %d", group_state_);
            return INVALID_SEQUENCE;
        }
    }
    else
    {
        return SUCCESS;
    }
}

double BaseGroup::getManualStepAxis(void)
{
    double step = manual_teach_.getManualStepAxis();
    FST_INFO("Get manual step axis = %.6f", step);
    return step;
}

double BaseGroup::getManualStepPosition(void)
{
    double step = manual_teach_.getManualStepPosition();
    FST_INFO("Get manual step position = %.4f", step);
    return step;
}

double BaseGroup::getManualStepOrientation(void)
{
    double step = manual_teach_.getManualStepOrientation();
    FST_INFO("Get manual step orientation = %.6f", step);
    return step;
}

ErrorCode BaseGroup::setManualStepAxis(double step)
{
    FST_INFO("Set manual step axis = %.6f", step);
    return manual_teach_.setManualStepAxis(step);
}

ErrorCode BaseGroup::setManualStepPosition(double step)
{
    FST_INFO("Set manual step position = %.4f", step);
    return manual_teach_.setManualStepPosition(step);
}

ErrorCode BaseGroup::setManualStepOrientation(double step)
{
    FST_INFO("Set manual step orientation = %.6f", step);
    return manual_teach_.setManualStepOrientation(step);
}

ErrorCode BaseGroup::manualMoveToPoint(const Joint &joint)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual to target joint: %s", printDBLine(&joint[0], buffer, LOG_TEXT_SIZE));

    if (group_state_ != STANDBY)
    {
        FST_ERROR("Cannot manual to target in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    if (manual_frame_ != JOINT)
    {
        FST_ERROR("Cannot manual to target in current frame = %d", manual_frame_);
        return INVALID_SEQUENCE;
    }

    getLatestJoint(manual_traj_.joint_start);
    FST_ERROR("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        FST_ERROR("start-joint is out of soft constraint, manual-mode-apoint is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (!soft_constraint_.isJointInConstraint(joint))
    {
        FST_ERROR("target-joint out of constraint: %s", printDBLine(&joint[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    manual_time_ = 0;
    manual_traj_.mode = APOINT;
    manual_traj_.frame = JOINT;
    ErrorCode err = manual_teach_.manualByTarget(joint, manual_time_, manual_traj_);

    if (err == SUCCESS)
    {
        group_state_ = MANUAL;
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        return err;
    }
}

ErrorCode BaseGroup::manualMoveToPoint(const PoseEuler &pose)
{
    ErrorCode err;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual to target pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f",
             pose.position.x, pose.position.y, pose.position.z, pose.orientation.a, pose.orientation.b, pose.orientation.c);

    if (group_state_ != STANDBY)
    {
        FST_ERROR("Cannot manual to target in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    if (manual_frame_ != BASE && manual_frame_ != USER && manual_frame_ != WORLD)
    {
        FST_ERROR("Cannot manual to target in current frame = %d", manual_frame_);
        return INVALID_SEQUENCE;
    }

    getLatestJoint(manual_traj_.joint_start);
    FST_ERROR("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        FST_ERROR("start-joint is out of soft constraint, manual-mode-apoint is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    Joint res_joint;
    Joint ref_joint = getLatestJoint();

    switch (manual_frame_)
    {
        case BASE:
            err = kinematics_ptr_->inverseKinematicsInBase(pose, ref_joint, res_joint);
            break;
        case USER:
            err = kinematics_ptr_->inverseKinematicsInUser(pose, ref_joint, res_joint);
            break;
        case WORLD:
            err = kinematics_ptr_->inverseKinematicsInWorld(pose, ref_joint, res_joint);
            break;
        default:
            FST_ERROR("Invalid manual frame = %d in manual to pose mode", manual_frame_);
            return MOTION_INTERNAL_FAULT;
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to get inverse kinematics of given target, code = 0x%llx", err);
        return err;
    }

    if (!soft_constraint_.isJointInConstraint(res_joint))
    {
        FST_ERROR("target-joint is out of soft constraint: %s", printDBLine(&res_joint[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    manual_time_ = 0;
    manual_traj_.mode = APOINT;
    manual_traj_.frame = manual_frame_;
    err = manual_teach_.manualByTarget(res_joint, manual_time_, manual_traj_);

    if (err == SUCCESS)
    {
        group_state_ = MANUAL;
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        return err;
    }
}

ErrorCode BaseGroup::manualMoveStep(const ManualDirection *direction)
{
    PoseEuler pose;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual step frame=%d by direction.", manual_frame_);

    if (group_state_ != STANDBY)
    {
        FST_ERROR("Cannot manual step in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    getLatestJoint(manual_traj_.joint_start);
    FST_INFO("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        if (manual_frame_ == JOINT)
        {
            for (size_t i = 0; i < getNumberOfJoint(); i++)
            {
                if (manual_traj_.joint_start[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
                {
                    FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                              i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                    return INVALID_PARAMETER;
                }
                else if (manual_traj_.joint_start[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
                {
                    FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                              i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                    return INVALID_PARAMETER;
                }
            }
        }
        else
        {
            FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
            return INVALID_SEQUENCE;
        }
    }

    switch (manual_frame_)
    {
        case JOINT:
            break;
        case BASE:
            kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, manual_traj_.cart_start);
        case USER:
            kinematics_ptr_->forwardKinematicsInUser(manual_traj_.joint_start, manual_traj_.cart_start);
            break;
        case WORLD:
            kinematics_ptr_->forwardKinematicsInWorld(manual_traj_.joint_start, manual_traj_.cart_start);
            break;
        case TOOL:
            kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, pose);
            manual_traj_.tool_coordinate = pose;
            memset(&manual_traj_.cart_start, 0, sizeof(manual_traj_.cart_start));
            break;
        default:
            FST_ERROR("Unsupported manual frame: %d", manual_frame_);
            return MOTION_INTERNAL_FAULT;
    }

    manual_time_ = 0;
    manual_traj_.mode = STEP;
    manual_traj_.frame = manual_frame_;
    ErrorCode err = manual_teach_.manualStepByDirect(direction, manual_time_, manual_traj_);

    if (err == SUCCESS)
    {
        group_state_ = MANUAL;
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        return err;
    }
}

ErrorCode BaseGroup::manualMoveContinuous(const ManualDirection *direction)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual continuous frame=%d by direction.", manual_frame_);

    if (group_state_ != STANDBY && group_state_ != MANUAL)
    {
        FST_ERROR("Cannot manual continuous in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    if (group_state_ == STANDBY)
    {
        PoseEuler pose;
        getLatestJoint(manual_traj_.joint_start);
        FST_INFO("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

        if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
        {
            if (manual_frame_ == JOINT)
            {
                for (size_t i = 0; i < getNumberOfJoint(); i++)
                {
                    if (manual_traj_.joint_start[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
                    {
                        FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                                  i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                        return INVALID_PARAMETER;
                    }
                    else if (manual_traj_.joint_start[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
                    {
                        FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                                  i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                        return INVALID_PARAMETER;
                    }
                }
            }
            else
            {
                FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
                return INVALID_SEQUENCE;
            }
        }

        switch (manual_frame_)
        {
            case JOINT:
                break;
            case BASE:
                kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, manual_traj_.cart_start);
            case USER:
                kinematics_ptr_->forwardKinematicsInUser(manual_traj_.joint_start, manual_traj_.cart_start);
                break;
            case WORLD:
                kinematics_ptr_->forwardKinematicsInWorld(manual_traj_.joint_start, manual_traj_.cart_start);
                break;
            case TOOL:
                kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, pose);
                manual_traj_.tool_coordinate = pose;
                memset(&manual_traj_.cart_start, 0, sizeof(manual_traj_.cart_start));
                break;
            default:
                FST_ERROR("Unsupported manual frame: %d", manual_frame_);
                return MOTION_INTERNAL_FAULT;
        }

        manual_time_ = 0;
        manual_traj_.mode = CONTINUOUS;
        manual_traj_.frame = manual_frame_;
        ErrorCode err = manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);

        if (err == SUCCESS)
        {
            group_state_ = MANUAL;
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
            memset(&manual_traj_, 0, sizeof(ManualTrajectory));
            return err;
        }
    }
    else if (group_state_ == MANUAL)
    {
        for (size_t i = 0; i < getNumberOfJoint(); i++)
        {
            if (manual_traj_.direction[i] != direction[i])
            {
                return manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);
            }
            else
            {
                continue;
            }
        }

        FST_INFO("Given directions same as current motion, do not need replan.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Cannot manual now, current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }
}

ErrorCode BaseGroup::manualStop(void)
{
    FST_INFO("Manual stop request received.");

    if (group_state_ == MANUAL)
    {
        FST_INFO("Manual mode = %d, frame = %d", manual_traj_.mode, manual_traj_.frame);

        if (manual_traj_.mode == CONTINUOUS)
        {
            ManualDirection direction[NUM_OF_JOINT] = {STANDING};
            ErrorCode err = manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);

            if (err == SUCCESS)
            {
                FST_INFO("Success, the group will stop in %.4fs", manual_traj_.duration - manual_time_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to stop current manual motion, error-code = 0x%llx", err);
                return err;
            }
        }
        else if (manual_traj_.mode == APOINT)
        {
            ErrorCode err = manual_teach_.manualStop(manual_time_, manual_traj_);

            if (err == SUCCESS)
            {
                FST_INFO("Success, the group will stop in %.4fs", manual_traj_.duration - manual_time_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to stop current manual motion, error-code = 0x%llx", err);
                return err;
            }
        }
        else
        {
            FST_INFO("Cannot stop manual motion in step mode, the group will stop in %.4fs", manual_traj_.duration - manual_time_);
            return INVALID_SEQUENCE;
        }
    }
    else
    {
        FST_INFO("The group is not in manual state, current-state = %d", group_state_);
        return SUCCESS;
    }
}


ErrorCode BaseGroup::autoMove(int id, const MotionTarget &target)
{
    FST_INFO("Auto move request received, motion ID = %d, type = %d", id, target.type);
    ErrorCode err;

    switch (target.type)
    {
        case MOTION_JOINT:
            err = autoJoint(target.joint_target, target.vel, target.cnt, id);
            break;
        case MOTION_LINE:
            err = autoLine(target.pose_target, target.vel, target.cnt, id);
            break;
        case MOTION_CIRCLE:
            err = autoCircle(target.circle_target.pose1, target.circle_target.pose2, target.vel, target.cnt, id);
            break;
        default:
            FST_ERROR("Invalid motion type, auto move aborted.");
            return INVALID_PARAMETER;
    }

    if (err == SUCCESS)
    {
        if (group_state_ == STANDBY)
        {
            size_t cnt = 0;

            while (!auto_pick_ptr_->valid && cnt < AUTO_CACHE_SIZE)
            {
                auto_pick_ptr_ = auto_pick_ptr_->next;
            }

            if (auto_pick_ptr_->valid)
            {
                auto_pick_ptr_->cache[0].time_from_start = 0;
                auto_pick_segment_ = 1;
                auto_time_ = 0;
                group_state_ = AUTO;
                FST_INFO("Start motion, pick-cache = %p, pick-seg = %d, pick-time = %.4f", auto_pick_ptr_, auto_pick_segment_, auto_time_);
            }
            else
            {
                FST_INFO("Trajectory cache not ready.");
                return MOTION_INTERNAL_FAULT;
            }
        }

        if (target.cnt < 0)
        {
            FST_INFO("Move with 'FINE', group will waiting for sand stable");
            // move by 'FINE'
            if (target.type == MOTION_JOINT)
            {
                waiting_motion_type_ = MOTION_JOINT;
                waiting_joint_ = target.joint_target;
                start_waiting_cnt_ = 20;
                waiting_fine_ = true;
            }
            else if (target.type == MOTION_LINE)
            {
                waiting_motion_type_ = MOTION_LINE;
                waiting_pose_ = target.pose_target;
                start_waiting_cnt_ = 20;
                waiting_fine_ = true;
            }
            else
            {
                FST_ERROR("Move by 'FINE', motion type = %d invalid", target.type);
                return MOTION_INTERNAL_FAULT;
            }
        }
    }
    else
    {
        FST_ERROR("autoMove: failed, code = 0x%llx", err);
        return err;
    }

    FST_INFO("autoMove: success!");
    return SUCCESS;
}

ErrorCode BaseGroup::autoJoint(const Joint &target, double vel, double cnt, int id)
{
    char buffer[LOG_TEXT_SIZE];
    Joint start_joint;
    FST_INFO("autoJoint: vel = %.4f, cnt = %.4f", vel, cnt);
    FST_INFO("  target = %s", printDBLine(&target[0], buffer, LOG_TEXT_SIZE));

    if (vel < MINIMUM_E6 || vel > 1 + MINIMUM_E6 || fabs(cnt) > 1 + MINIMUM_E6)
    {
        FST_ERROR("Invalid vel or CNT.");
        return INVALID_PARAMETER;
    }

    if (!soft_constraint_.isJointInConstraint(target))
    {
        FST_ERROR("Target out of soft constraint.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (group_state_ == STANDBY)
    {
        getLatestJoint(start_joint);
    }
    else if (group_state_ == AUTO)
    {
        start_joint = start_joint_;
    }
    else
    {
        FST_ERROR("Cannot start auto motion in current group state: %d", group_state_);
        return INVALID_SEQUENCE;
    }

    FST_INFO("  start  = %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(start_joint))
    {
        FST_ERROR("Start joint out of soft constraint.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    ErrorCode err;
    size_t  index, length;
    double  precision = 0.01;
    Joint   path[MAX_PATH_SIZE];
    planJointPath(start_joint, target, precision, index, path, length);
    TrajectoryCache *p_cache = auto_pick_ptr_->valid == false ? auto_pick_ptr_ : auto_pick_ptr_->next;

    for (size_t i = 0; i < length; i++)
    {
        p_cache->cache[i].path_point.id     = id;
        p_cache->cache[i].path_point.stamp  = i;
        p_cache->cache[i].path_point.type   = MOTION_JOINT;
        p_cache->cache[i].path_point.joint  = path[i];
        p_cache->cache[i].time_from_start   = -1;
        //p_cache->cache[i].command_duration  = expect_duration;
        p_cache->cache[i].forward_duration  = -1;
        p_cache->cache[i].backward_duration = -1;
    }

    p_cache->head = 0;
    p_cache->tail = length;
    p_cache->expect_duration = precision / (axis_vel_[index] * vel * vel_ratio_);
    p_cache->deadline = 999999999.999;

    FST_INFO("Prepare trajectory cache ...");
    err = prepareCache(*p_cache);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to prepare trajectory, code = 0x%llx", err);
        return err;
    }

    FST_INFO("Preplan trajectory cache ...");
    err = preplanCache(*p_cache, cnt);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to preplan trajectory, code = 0x%llx", err);
        return err;
    }

    p_cache->valid = true;
    start_joint_ = p_cache->cache[p_cache->tail - 1].ending_state.angle;
    FST_INFO("Success.");
    return SUCCESS;
}

ErrorCode BaseGroup::autoLine(const PoseEuler &target, double vel, double cnt, int id)
{
    /*
    char buffer[LOG_TEXT_SIZE];
    Joint start_joint;
    FST_INFO("autoLine: vel = %.4f, cnt = %.4f", vel, cnt);
    FST_INFO("  target = %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", target);

    if (vel < MINIMUM_E6 || vel > 1 + MINIMUM_E6 || fabs(cnt) > 1 + MINIMUM_E6)
    {
        FST_ERROR("Invalid vel or CNT.");
        return INVALID_PARAMETER;
    }

    if (!soft_constraint_.isJointInConstraint(target))
    {
        FST_ERROR("Target out of soft constraint.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (group_state_ == STANDBY)
    {
        getLatestJoint(start_joint);
    }
    else if (group_state_ == AUTO)
    {
        start_joint = start_joint_;
    }
    else
    {
        FST_ERROR("Cannot start auto motion in current group state: %d", group_state_);
        return INVALID_SEQUENCE;
    }

    FST_INFO("  start  = %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(start_joint))
    {
        FST_ERROR("Start joint out of soft constraint.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    ErrorCode err;
    size_t  index, length;
    double  precision = 0.01;
    Joint   path[MAX_PATH_SIZE];
    planJointPath(start_joint, target, precision, index, path, length);
    TrajectoryCache *p_cache = auto_pick_ptr_->valid == false ? auto_pick_ptr_ : auto_pick_ptr_->next;

    for (size_t i = 0; i < length; i++)
    {
        p_cache->cache[i].path_point.id     = id;
        p_cache->cache[i].path_point.stamp  = i;
        p_cache->cache[i].path_point.type   = MOTION_JOINT;
        p_cache->cache[i].path_point.joint  = path[i];
        p_cache->cache[i].time_from_start   = -1;
        //p_cache->cache[i].command_duration  = expect_duration;
        p_cache->cache[i].forward_duration  = -1;
        p_cache->cache[i].backward_duration = -1;
    }

    p_cache->head = 0;
    p_cache->tail = length;
    p_cache->expect_duration = precision / (axis_vel_[index] * vel * vel_ratio_);

    FST_INFO("Prepare trajectory cache ...");
    err = prepareCache(*p_cache);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to prepare trajectory, code = 0x%llx", err);
        return err;
    }

    FST_INFO("Preplan trajectory cache ...");
    err = preplanCache(*p_cache);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to preplan trajectory, code = 0x%llx", err);
        return err;
    }

    p_cache->valid = true;
    start_joint_ = p_cache->cache[p_cache->tail - 1].ending_state.angle;
    FST_INFO("Success.");
     */
    return SUCCESS;
}

ErrorCode BaseGroup::autoCircle(const PoseEuler &target1, const PoseEuler &target2, double vel, double cnt, int id)
{
    return SUCCESS;
}

ErrorCode BaseGroup::prepareCache(TrajectoryCache &cache)
{
    ErrorCode err = SUCCESS;

    if (cache.tail > cache.head)
    {
        MotionType mtype = cache.cache[cache.head].path_point.type;

        if (mtype == MOTION_JOINT)
        {
            for (size_t i = cache.head; i < cache.tail; i++)
            {
                cache.cache[i].ending_state.angle = cache.cache[i].path_point.joint;
                cache.cache[i + 1].start_state.angle = cache.cache[i].path_point.joint;
            }
        }
        else if (mtype == MOTION_LINE || mtype == MOTION_CIRCLE)
        {
            Joint ref = getLatestJoint();

            for (size_t i = cache.head; i < cache.tail; i++)
            {
                err = kinematics_ptr_->inverseKinematicsInUser(cache.cache[i].path_point.pose, ref, cache.cache[i].ending_state.angle);

                if (err == SUCCESS)
                {
                    ref = cache.cache[i].ending_state.angle;
                    cache.cache[i + 1].start_state.angle = cache.cache[i].ending_state.angle;
                    continue;
                }
                else
                {
                    FST_ERROR("Fail to get IK result, code = 0x%llx", err);
                    return err;
                }
            }
        }
        else
        {
            FST_ERROR("Invalid motion type of point in cache.");
            return MOTION_INTERNAL_FAULT;
        }
    }
    else
    {
        FST_ERROR("prepareCache: cache is empty.");
        return MOTION_INTERNAL_FAULT;
    }

    TrajectorySegment &seg0 = cache.cache[cache.head];
    TrajectorySegment &seg1 = cache.cache[cache.head + 1];
    TrajectorySegment &segn  = cache.cache[cache.tail - 1];

    seg0.start_state.angle = seg0.ending_state.angle;
    memset(&seg0.start_state.omega, 0, sizeof(Joint));
    memset(&seg0.start_state.alpha, 0, sizeof(Joint));
    memset(&seg0.ending_state.omega, 0, sizeof(Joint));
    memset(&seg0.ending_state.alpha, 0, sizeof(Joint));
    memset(&seg1.start_state.omega, 0, sizeof(Joint));
    memset(&seg1.start_state.alpha, 0, sizeof(Joint));

    memset(&segn.ending_state.omega, 0, sizeof(Joint));
    memset(&segn.ending_state.alpha, 0, sizeof(Joint));

    char buffer[LOG_TEXT_SIZE];

    for (size_t i = cache.head; i < cache.tail; i++)
    {
        FST_INFO("Point-%d: %s", i, printDBLine(&cache.cache[i].ending_state.angle[0], buffer, LOG_TEXT_SIZE));
    }

    return SUCCESS;
}

ErrorCode BaseGroup::preplanCache(TrajectoryCache &cache, double cnt)
{
    char buffer[LOG_TEXT_SIZE];
    ErrorCode err = SUCCESS;
    double  this_duration = 99.99;
    double  last_duration = 99.99;
    size_t  index = cache.tail - 1;
    TrajectorySegment *pseg;
    TrajectorySegment &seg1 = cache.cache[1];
    TrajectorySegment &segn = cache.cache[index];


    Joint   forward_alpha_upper;
    Joint   forward_alpha_lower;
    Joint   backward_alpha_upper;
    Joint   backward_alpha_lower;
    DynamicsProduct forward_dynamics_product, backward_dynamics_product;

    computeDynamics(seg1.start_state.angle, seg1.start_state.omega, forward_alpha_upper, forward_alpha_lower, forward_dynamics_product);
    computeDynamics(segn.ending_state.angle, segn.ending_state.omega, backward_alpha_upper, backward_alpha_lower, backward_dynamics_product);

    FST_INFO("forward angle: %s", printDBLine(&seg1.start_state.angle[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("forward omega: %s", printDBLine(&seg1.start_state.omega[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("forward alpha-upper: %s", printDBLine(&forward_alpha_upper[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("forward alpha-lower: %s", printDBLine(&forward_alpha_lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("backward angle: %s", printDBLine(&segn.ending_state.angle[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("backward omega: %s", printDBLine(&segn.ending_state.omega[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("backward alpha-upper: %s", printDBLine(&backward_alpha_upper[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("backward alpha-lower: %s", printDBLine(&backward_alpha_lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));

    while (this_duration > cache.expect_duration + MINIMUM_E6 && index > 0)
    {
        pseg = &cache.cache[index];
        err = backwardCycle(pseg->start_state.angle, pseg->ending_state, cache.expect_duration, backward_alpha_upper, backward_alpha_lower, jerk_, pseg->backward_coeff);
        //err = backwardCycleTest(pseg->start_state.angle, pseg->ending_state, 0.01, backward_alpha_upper, backward_alpha_lower, jerk_, pseg->backward_coeff);

        if (err == SUCCESS)
        {
            FST_WARN("back-cycle: %d, exp-duration: %.4f", index, cache.expect_duration);
            FST_INFO("  start-angle: %s", printDBLine(&pseg->start_state.angle[0], buffer, LOG_TEXT_SIZE));
            FST_INFO("  ending-angle: %s", printDBLine(&pseg->ending_state.angle[0], buffer, LOG_TEXT_SIZE));
            FST_INFO("  ending-omega: %s", printDBLine(&pseg->ending_state.omega[0], buffer, LOG_TEXT_SIZE));
            FST_INFO("  ending-alpha: %s", printDBLine(&pseg->ending_state.alpha[0], buffer, LOG_TEXT_SIZE));

            for (size_t i = 0; i < 6; i++)
            {
                FST_INFO("  duration = %.4f, %.4f, %.4f, %.4f", pseg->backward_coeff[i].duration[0], pseg->backward_coeff[i].duration[1], pseg->backward_coeff[i].duration[2], pseg->backward_coeff[i].duration[3]);
                FST_INFO("  coeff = %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[0][3], pseg->backward_coeff[i].coeff[0][2], pseg->backward_coeff[i].coeff[0][1], pseg->backward_coeff[i].coeff[0][0]);
                FST_INFO("          %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[1][3], pseg->backward_coeff[i].coeff[1][2], pseg->backward_coeff[i].coeff[1][1], pseg->backward_coeff[i].coeff[1][0]);
                FST_INFO("          %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[2][3], pseg->backward_coeff[i].coeff[2][2], pseg->backward_coeff[i].coeff[2][1], pseg->backward_coeff[i].coeff[2][0]);
                FST_INFO("          %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[3][3], pseg->backward_coeff[i].coeff[3][2], pseg->backward_coeff[i].coeff[3][1], pseg->backward_coeff[i].coeff[3][0]);
            }

            pseg->backward_duration = pseg->backward_coeff[0].duration[0] + pseg->backward_coeff[0].duration[1] + pseg->backward_coeff[0].duration[2] + pseg->backward_coeff[0].duration[3];
            pseg->dynamics_product  = backward_dynamics_product;
            this_duration = pseg->backward_duration;
            Joint tmp;
            sampleTrajectorySegment(pseg->backward_coeff, 0, tmp, pseg->start_state.omega, pseg->start_state.alpha);
            cache.cache[index - 1].ending_state = pseg->start_state;
            FST_WARN("this duration: %.8f, exp duration: %.8f, last duration: %.8f", this_duration, cache.expect_duration, last_duration);

            if (this_duration + MINIMUM_E6 > last_duration)
            {
                break;
            }
            else
            {
                last_duration = this_duration;
            }

            if (this_duration > cache.expect_duration + MINIMUM_E6)
            {
                index --;
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Fail in back cycle, err = 0x%llx", err);
        return err;
    }

    index = 1;
    this_duration = 99.99;
    last_duration = 99.99;

    while (this_duration > cache.expect_duration + MINIMUM_E6 && index < cache.tail)
    {
        pseg = &cache.cache[index];
        err = forwardCycle(pseg->start_state, pseg->ending_state.angle, cache.expect_duration, forward_alpha_upper, forward_alpha_lower, jerk_, pseg->forward_coeff);
        //err = forwardCycleTest(pseg->start_state, pseg->ending_state.angle, 0.01, forward_alpha_upper, forward_alpha_lower, jerk_, pseg->forward_coeff);

        if (err == SUCCESS)
        {
            FST_WARN("fore-cycle: %d, exp-duration: %.4f", index, cache.expect_duration);
            FST_INFO("  start-angle: %s", printDBLine(&pseg->start_state.angle[0], buffer, LOG_TEXT_SIZE));
            FST_INFO("  start-omega: %s", printDBLine(&pseg->start_state.omega[0], buffer, LOG_TEXT_SIZE));
            FST_INFO("  start-alpha: %s", printDBLine(&pseg->start_state.alpha[0], buffer, LOG_TEXT_SIZE));
            FST_INFO("  ending-angle: %s", printDBLine(&pseg->ending_state.angle[0], buffer, LOG_TEXT_SIZE));

            for (size_t i = 0; i < 6; i++)
            {
                FST_INFO("  duration = %.4f, %.4f, %.4f, %.4f", pseg->forward_coeff[i].duration[0], pseg->forward_coeff[i].duration[1], pseg->forward_coeff[i].duration[2], pseg->forward_coeff[i].duration[3]);
                FST_INFO("  coeff = %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[0][3], pseg->forward_coeff[i].coeff[0][2], pseg->forward_coeff[i].coeff[0][1], pseg->forward_coeff[i].coeff[0][0]);
                FST_INFO("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[1][3], pseg->forward_coeff[i].coeff[1][2], pseg->forward_coeff[i].coeff[1][1], pseg->forward_coeff[i].coeff[1][0]);
                FST_INFO("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[2][3], pseg->forward_coeff[i].coeff[2][2], pseg->forward_coeff[i].coeff[2][1], pseg->forward_coeff[i].coeff[2][0]);
                FST_INFO("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[3][3], pseg->forward_coeff[i].coeff[3][2], pseg->forward_coeff[i].coeff[3][1], pseg->forward_coeff[i].coeff[3][0]);
            }

            pseg->forward_duration = pseg->forward_coeff[0].duration[0] + pseg->forward_coeff[0].duration[1] +
                                     pseg->forward_coeff[0].duration[2] + pseg->forward_coeff[0].duration[3];
            pseg->dynamics_product = forward_dynamics_product;
            this_duration = pseg->forward_duration;
            Joint tmp;
            sampleTrajectorySegment(pseg->forward_coeff, pseg->forward_duration, tmp, pseg->ending_state.omega, pseg->ending_state.alpha);
            cache.cache[index + 1].start_state = pseg->ending_state;
            FST_WARN("this duration: %.8f, exp duration: %.8f, last duration: %.8f", this_duration, cache.expect_duration, last_duration);
            if (this_duration + MINIMUM_E6 > last_duration)
            {
                break;
            }
            else
            {
                last_duration = this_duration;
            }

            if (this_duration > cache.expect_duration + MINIMUM_E6)
            {
                index ++;
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Fail in fore cycle, err = 0x%llx", err);
        return err;
    }

    size_t forward_index = cache.head + 1;
    size_t backward_index = cache.tail - 1;

    while (forward_index != backward_index)
    {
        //FST_LOG("ind-f=%d, ind-b=%d, duration-f=%.6f, duration-nf=%.6f, duration-b=%.6f, duration-nb=%.6f",
        //        ind_f, ind_b, duration_f, path[ind_f + 1].forward_duration, duration_b, path[ind_b - 1].backward_duration);

        if (cache.cache[forward_index].forward_duration > cache.cache[backward_index].backward_duration)
        {
            if (cache.cache[forward_index + 1].forward_duration > 0)
            {
                forward_index ++;
            }
            else
            {
                break;
            }
        }
        else
        {
            if (cache.cache[backward_index - 1].backward_duration > 0)
            {
                backward_index --;
            }
            else
            {
                break;
            }
        }
    }

    if (forward_index == backward_index)
    {
        if (cache.cache[forward_index].forward_duration < cache.cache[backward_index].backward_duration)
        {
            forward_index --;
        }
        else
        {
            backward_index ++;
        }
    }

    FST_INFO("spd-up-index=%d, spd-down-index=%d", forward_index, backward_index);
    size_t fore_index = forward_index;
    size_t back_index = backward_index;
    forward_index ++;
    backward_index --;

    while (cache.cache[forward_index].forward_duration > 0 && forward_index < cache.tail)
    {
        cache.cache[forward_index ++].forward_duration = -1;
    }

    while (cache.cache[backward_index].backward_duration > 0 && backward_index > cache.head)
    {
        cache.cache[backward_index --].backward_duration = -1;
    }

    for (size_t i = cache.head; i < cache.tail; i++)
    {
        FST_INFO("stamp=%d, duration_f=%.6f, duration_b=%.6f, cmd_duration=%.6f",
                 cache.cache[i].path_point.stamp, cache.cache[i].forward_duration,
                 cache.cache[i].backward_duration, cache.expect_duration);
    }

    if (cnt > MINIMUM_E3)
    {
        double smooth_duration = cache.expect_duration / cnt;

        for (forward_index = cache.head + 1; forward_index < cache.tail; forward_index++)
        {
            if (cache.cache[forward_index].forward_duration > smooth_duration && forward_index + 1 < cache.tail && cache.cache[forward_index + 1].forward_duration > 0)
            {
                forward_index ++;
            }
            else
            {
                break;
            }
        }

        cache.smooth_in_stamp = forward_index;

        for (backward_index = cache.tail - 1; backward_index > cache.head; backward_index--)
        {
            if (cache.cache[backward_index].backward_duration > smooth_duration && backward_index - 1 > cache.head && cache.cache[backward_index - 1].backward_duration > 0)
            {
                backward_index --;
            }
            else
            {
                break;
            }
        }

        cache.smooth_out_stamp = backward_index;
    }
    else
    {
        cache.smooth_in_stamp = fore_index;
        cache.smooth_out_stamp = back_index;
    }

    return SUCCESS;
}

bool BaseGroup::nextMovePermitted(void)
{
    if (waiting_fine_)
        return false;

    if (group_state_ == AUTO || group_state_ == AUTO_TO_STANDBY || group_state_ == STANDBY_TO_AUTO)
    {
        if (auto_pick_ptr_->valid && auto_time_ + 0.1 < auto_pick_ptr_->deadline)
        {
            return false;
        }
    }

    return true;
}

#define  PRINT_COEFFS

ErrorCode BaseGroup::createTrajectory(void)
{
    if (group_state_ == AUTO)
    {
        ErrorCode err = SUCCESS;
        size_t  joint_num = getNumberOfJoint();
        Joint   tmp_joint;
        Joint   alpha_upper;
        Joint   alpha_lower;
        TrajectoryItem traj_item;
        TrajectorySegment *pseg;

        while ((traj_fifo_.duration() < 0.1 && !traj_fifo_.full()) || traj_fifo_.size() < 3)
        {
            if (auto_pick_segment_ >= auto_pick_ptr_->tail)
            {
                auto_pick_ptr_->valid = false;

                if (auto_pick_ptr_->next->valid)
                {
                    auto_pick_ptr_ = auto_pick_ptr_->next;
                    auto_pick_segment_ = 1;
                    auto_pick_ptr_->cache[0].time_from_start = traj_fifo_.back().time_from_start;
                    FST_INFO("auto-pick-ptr switch, seg = %d, time-from-start = %.4f", auto_pick_segment_, auto_pick_ptr_->cache[0].time_from_start);
                }
                else
                {
                    break;
                }
            }

#ifdef PRINT_COEFFS
            FST_INFO("seg-%d, fore-duration = %.4f, back-duration = %.4f", auto_pick_segment_, auto_pick_ptr_->cache[auto_pick_segment_].forward_duration, auto_pick_ptr_->cache[auto_pick_segment_].backward_duration);
            FST_INFO("next-seg = %d, traj-tail = %d, fore-duration = %.4f, back-duration = %.4f", auto_pick_segment_ + 1, auto_pick_ptr_->tail, auto_pick_ptr_->cache[auto_pick_segment_ + 1].forward_duration, auto_pick_ptr_->cache[auto_pick_segment_ + 1].backward_duration);
#endif
            if (auto_pick_segment_ <= auto_pick_ptr_->smooth_out_stamp)
            {
                pseg = &auto_pick_ptr_->cache[auto_pick_segment_];

                if (pseg->forward_duration > 0)
                {
                    traj_item.id = pseg->path_point.id;
                    traj_item.duration = pseg->forward_duration;
                    traj_item.time_from_start = traj_fifo_.timeFromStart() + traj_item.duration;
                    traj_item.dynamics_product = pseg->dynamics_product;
                    memcpy(traj_item.traj_coeff,  pseg->forward_coeff, sizeof(traj_item.traj_coeff));
                    traj_fifo_.push(traj_item);
                    sampleTrajectorySegment(traj_item.traj_coeff, traj_item.duration, tmp_joint, pseg->ending_state.omega, pseg->ending_state.alpha);
                    auto_pick_ptr_->cache[auto_pick_segment_ + 1].start_state = pseg->ending_state;
#ifdef PRINT_COEFFS
                    for (size_t i = 0; i < 6; i++)
                    {
                        FST_INFO("  duration = %.4f, %.4f, %.4f, %.4f", traj_item.traj_coeff[i].duration[0], traj_item.traj_coeff[i].duration[1], traj_item.traj_coeff[i].duration[2], traj_item.traj_coeff[i].duration[3]);
                        FST_INFO("  coeff = %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[0][3], traj_item.traj_coeff[i].coeff[0][2], traj_item.traj_coeff[i].coeff[0][1], traj_item.traj_coeff[i].coeff[0][0]);
                        FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[1][3], traj_item.traj_coeff[i].coeff[1][2], traj_item.traj_coeff[i].coeff[1][1], traj_item.traj_coeff[i].coeff[1][0]);
                        FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[2][3], traj_item.traj_coeff[i].coeff[2][2], traj_item.traj_coeff[i].coeff[2][1], traj_item.traj_coeff[i].coeff[2][0]);
                        FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[3][3], traj_item.traj_coeff[i].coeff[3][2], traj_item.traj_coeff[i].coeff[3][1], traj_item.traj_coeff[i].coeff[3][0]);
                    }
#endif
                }
                else
                {
                    if (pseg->backward_duration < 0 && auto_pick_segment_ + 1 < auto_pick_ptr_->tail && (pseg + 1)->backward_duration < 0)
                    {
                        computeDynamics(pseg->start_state.angle, pseg->start_state.omega, alpha_upper, alpha_lower, pseg->dynamics_product);
                        err = forwardCycle(pseg->start_state, pseg->ending_state.angle, auto_pick_ptr_->expect_duration, alpha_upper, alpha_lower, jerk_, pseg->forward_coeff);
                        //err = forwardCycleTest(pseg->start_state, pseg->ending_state.angle, 0.01, alpha_upper, alpha_lower, jerk_, pseg->forward_coeff);

                        if (err == SUCCESS)
                        {
                            for (size_t i = 0; i < joint_num; i++)
                            {
                                pseg->forward_coeff[i].duration[1] += pseg->forward_coeff[i].duration[0];
                                pseg->forward_coeff[i].duration[2] += pseg->forward_coeff[i].duration[1];
                                pseg->forward_coeff[i].duration[3] += pseg->forward_coeff[i].duration[2];
                            }

                            pseg->forward_duration = pseg->forward_coeff[0].duration[0] + pseg->forward_coeff[0].duration[1] +
                                                     pseg->forward_coeff[0].duration[2] + pseg->forward_coeff[0].duration[3];
                            traj_item.id = pseg->path_point.id;
                            traj_item.duration = pseg->forward_duration;
                            traj_item.time_from_start = traj_fifo_.timeFromStart() + traj_item.duration;
                            traj_item.dynamics_product = pseg->dynamics_product;
                            memcpy(traj_item.traj_coeff,  pseg->forward_coeff, sizeof(traj_item.traj_coeff));
                            traj_fifo_.push(traj_item);
                            sampleTrajectorySegment(traj_item.traj_coeff, traj_item.duration, tmp_joint, pseg->ending_state.omega, pseg->ending_state.alpha);
                            auto_pick_ptr_->cache[auto_pick_segment_ + 1].start_state = pseg->ending_state;
                        }
                        else
                        {
                            FST_ERROR("Fail to create trajectory from path, code = 0x%llx", err);
                            return err;
                        }
#ifdef PRINT_COEFFS
                        for (size_t i = 0; i < 6; i++)
                        {
                            FST_INFO("  duration = %.4f, %.4f, %.4f, %.4f", traj_item.traj_coeff[i].duration[0], traj_item.traj_coeff[i].duration[1], traj_item.traj_coeff[i].duration[2], traj_item.traj_coeff[i].duration[3]);
                            FST_INFO("  coeff = %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[0][3], traj_item.traj_coeff[i].coeff[0][2], traj_item.traj_coeff[i].coeff[0][1], traj_item.traj_coeff[i].coeff[0][0]);
                            FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[1][3], traj_item.traj_coeff[i].coeff[1][2], traj_item.traj_coeff[i].coeff[1][1], traj_item.traj_coeff[i].coeff[1][0]);
                            FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[2][3], traj_item.traj_coeff[i].coeff[2][2], traj_item.traj_coeff[i].coeff[2][1], traj_item.traj_coeff[i].coeff[2][0]);
                            FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[3][3], traj_item.traj_coeff[i].coeff[3][2], traj_item.traj_coeff[i].coeff[3][1], traj_item.traj_coeff[i].coeff[3][0]);
                        }
#endif
                    }
                    else if (pseg->backward_duration > 0 && (auto_pick_segment_ + 1 == auto_pick_ptr_->tail || (pseg + 1)->backward_duration > 0))
                    {
                        traj_item.id = pseg->path_point.id;
                        traj_item.duration = pseg->backward_duration;
                        traj_item.time_from_start = traj_fifo_.timeFromStart() + traj_item.duration;
                        traj_item.dynamics_product = pseg->dynamics_product;
                        memcpy(traj_item.traj_coeff,  pseg->backward_coeff, sizeof(traj_item.traj_coeff));
                        traj_fifo_.push(traj_item);
                        //sampleTrajectorySegment(traj_item.traj_coeff, 0, tmp_joint, pseg->start_state.omega, pseg->start_state.alpha);
                        //auto_pick_ptr_->cache[auto_pick_segment_ + 1].start_state = pseg->ending_state;
#ifdef PRINT_COEFFS
                        for (size_t i = 0; i < 6; i++)
                        {
                            FST_INFO("  duration = %.4f, %.4f, %.4f, %.4f", traj_item.traj_coeff[i].duration[0], traj_item.traj_coeff[i].duration[1], traj_item.traj_coeff[i].duration[2], traj_item.traj_coeff[i].duration[3]);
                            FST_INFO("  coeff = %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[0][3], traj_item.traj_coeff[i].coeff[0][2], traj_item.traj_coeff[i].coeff[0][1], traj_item.traj_coeff[i].coeff[0][0]);
                            FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[1][3], traj_item.traj_coeff[i].coeff[1][2], traj_item.traj_coeff[i].coeff[1][1], traj_item.traj_coeff[i].coeff[1][0]);
                            FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[2][3], traj_item.traj_coeff[i].coeff[2][2], traj_item.traj_coeff[i].coeff[2][1], traj_item.traj_coeff[i].coeff[2][0]);
                            FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[3][3], traj_item.traj_coeff[i].coeff[3][2], traj_item.traj_coeff[i].coeff[3][1], traj_item.traj_coeff[i].coeff[3][0]);
                        }
#endif
                    }
                    else if (pseg->backward_duration < 0 && auto_pick_segment_ + 1 < auto_pick_ptr_->tail && (pseg + 1)->backward_duration > 0)
                    {
                        //smooth
                        computeDynamics(pseg->start_state.angle, pseg->start_state.omega, alpha_upper, alpha_lower, pseg->dynamics_product);
                        traj_item.id = pseg->path_point.id;
                        traj_item.dynamics_product = pseg->dynamics_product;
                        smoothPoint2Point(pseg->start_state, (pseg + 1)->start_state, auto_cache_ptr_->expect_duration, alpha_upper, alpha_lower, jerk_, traj_item.traj_coeff);
                        traj_item.duration = traj_item.traj_coeff[0].duration[0] + traj_item.traj_coeff[0].duration[1] + traj_item.traj_coeff[0].duration[2] + traj_item.traj_coeff[0].duration[3];
                        traj_item.time_from_start = traj_fifo_.timeFromStart() + traj_item.duration;
                        traj_fifo_.push(traj_item);
#ifdef PRINT_COEFFS
                        for (size_t i = 0; i < 6; i++)
                        {
                            FST_INFO("  duration = %.4f, %.4f, %.4f, %.4f", traj_item.traj_coeff[i].duration[0], traj_item.traj_coeff[i].duration[1], traj_item.traj_coeff[i].duration[2], traj_item.traj_coeff[i].duration[3]);
                            FST_INFO("  coeff = %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[0][3], traj_item.traj_coeff[i].coeff[0][2], traj_item.traj_coeff[i].coeff[0][1], traj_item.traj_coeff[i].coeff[0][0]);
                            FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[1][3], traj_item.traj_coeff[i].coeff[1][2], traj_item.traj_coeff[i].coeff[1][1], traj_item.traj_coeff[i].coeff[1][0]);
                            FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[2][3], traj_item.traj_coeff[i].coeff[2][2], traj_item.traj_coeff[i].coeff[2][1], traj_item.traj_coeff[i].coeff[2][0]);
                            FST_INFO("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[3][3], traj_item.traj_coeff[i].coeff[3][2], traj_item.traj_coeff[i].coeff[3][1], traj_item.traj_coeff[i].coeff[3][0]);
                        }
#endif
                    }
                    else
                    {
                        FST_ERROR("createTrajectory: internal fault !!!");
                        return MOTION_INTERNAL_FAULT;
                    }
                }

                if (auto_pick_segment_ == auto_pick_ptr_->smooth_out_stamp)
                {
                    auto_pick_ptr_->deadline = traj_fifo_.timeFromStart();
                }

                auto_pick_segment_ ++;
            }
            else
            {

            }
        }
    }
    return SUCCESS;
}

void BaseGroup::getLatestJoint(Joint &joint)
{
    pthread_mutex_lock(&servo_mutex_);
    joint = current_joint_;
    pthread_mutex_unlock(&servo_mutex_);
}

Joint BaseGroup::getLatestJoint(void)
{
    pthread_mutex_lock(&servo_mutex_);
    Joint joint(current_joint_);
    pthread_mutex_unlock(&servo_mutex_);
    return joint;
}

void BaseGroup::getServoState(ServoState &state)
{
    pthread_mutex_lock(&servo_mutex_);
    state = servo_state_;
    pthread_mutex_unlock(&servo_mutex_);
}

ServoState BaseGroup::getServoState(void)
{
    pthread_mutex_lock(&servo_mutex_);
    ServoState state(servo_state_);
    pthread_mutex_unlock(&servo_mutex_);
    return state;
}

void BaseGroup::getGroupState(GroupState &state)
{
    state = group_state_;
}

GroupState BaseGroup::getGroupState(void)
{
    return group_state_;
}

/*
ErrorCode BaseGroup::getJointFromPose(const PoseEuler &pose, Joint &joint)
{
    ErrorCode err;

    switch (manual_frame_)
    {
        case BASE:
            err = kinematics_ptr_->inverseKinematicsInBase(pose, getLatestJoint(), joint);
            break;
        case USER:
            err = kinematics_ptr_->inverseKinematicsInUser(pose, getLatestJoint(), joint);
            break;
        case WORLD:
            err = kinematics_ptr_->inverseKinematicsInUser(pose, getLatestJoint(), joint);
            break;
        default:
            FST_ERROR("getJointFromPose: motion-frame is invalid: %d", manual_frame_);
            err = INVALID_SEQUENCE;
            break;
    }

    return err;
}

ErrorCode BaseGroup::getPoseFromJoint(const Joint &joint, PoseEuler &pose)
{
    switch (manual_frame_)
    {
        case BASE:
            kinematics_ptr_->forwardKinematicsInBase(joint, pose);
            return SUCCESS;
        case USER:
            kinematics_ptr_->forwardKinematicsInUser(joint, pose);
            return SUCCESS;
        case WORLD:
            kinematics_ptr_->forwardKinematicsInWorld(joint, pose);
            return SUCCESS;
        default:
            FST_ERROR("getPoseFromJoint: motion-frame is invalid: %d", manual_frame_);
            return INVALID_SEQUENCE;
    }
}
*/

/*
ErrorCode BaseGroup::getPoseFromJointInBase(const Joint &joint, PoseEuler &pose)
{
    kinematics_ptr_->forwardKinematicsInBase(joint, pose);
    return SUCCESS;
}

ErrorCode BaseGroup::getPoseFromJointInUser(const Joint &joint, PoseEuler &pose)
{
    kinematics_ptr_->forwardKinematicsInUser(joint, pose);
    return SUCCESS;
}

ErrorCode BaseGroup::getPoseFromJointInWorld(const Joint &joint, PoseEuler &pose)
{
    kinematics_ptr_->forwardKinematicsInWorld(joint, pose);
    return SUCCESS;
}
*/

ErrorCode BaseGroup::setGlobalVelRatio(double ratio)
{
    FST_INFO("Set global velocity ratio: %.4f", ratio);

    if (ratio < 0 || ratio > 1)
    {
        FST_ERROR("Given ratio out of range (0, 1)");
        return INVALID_PARAMETER;
    }
    else
    {
        vel_ratio_ = ratio;
        manual_teach_.setGlobalVelRatio(vel_ratio_);
        return SUCCESS;
    }
}

ErrorCode BaseGroup::setGlobalAccRatio(double ratio)
{
    FST_INFO("Set global acceleration ratio: %.4f", ratio);

    if (ratio < 0 || ratio > 1)
    {
        FST_ERROR("Given ratio out of range (0, 1)");
        return INVALID_PARAMETER;
    }
    else
    {
        acc_ratio_ = ratio;
        manual_teach_.setGlobalAccRatio(acc_ratio_);
        return SUCCESS;
    }
}

double BaseGroup::getGlobalVelRatio(void)
{
    return vel_ratio_;
}

double BaseGroup::getGlobalAccRatio(void)
{
    return acc_ratio_;
}

ErrorCode BaseGroup::setToolFrame(const PoseEuler &tf)
{
    return kinematics_ptr_->setToolFrame(tf);
}

ErrorCode BaseGroup::setUserFrame(const PoseEuler &uf)
{
    return kinematics_ptr_->setUserFrame(uf);
}

ErrorCode BaseGroup::setWorldFrame(const PoseEuler &wf)
{
    return kinematics_ptr_->setUserFrame(wf);
}

ErrorCode BaseGroup::sendPoint(void)
{
    if (group_state_ == MANUAL || group_state_ == MANUAL_TO_STANDBY)
    {
        if (bare_core_.isPointCacheEmpty())
        {
            size_t length = 10;
            TrajectoryPoint point[10];
            pickFromManual(point, length);
            bare_core_.fillPointCache(point, length, POINT_POS);
        }

        return bare_core_.sendPoint() ? SUCCESS : BARE_CORE_TIMEOUT;
    }
    else if (group_state_ == AUTO || group_state_ == AUTO_TO_STANDBY || group_state_ == AUTO_TO_PAUSE)
    {
        if (bare_core_.isPointCacheEmpty())
        {
            size_t length = 10;
            TrajectoryPoint point[10];
            pickFromAuto(point, length);
            bare_core_.fillPointCache(point, length, POINT_POS_VEL_ACC_EFF);
        }

        return bare_core_.sendPoint() ? SUCCESS : BARE_CORE_TIMEOUT;
        return SUCCESS;
    }
    else
    {
        return INVALID_SEQUENCE;
    }
}

ErrorCode BaseGroup::pickFromManual(TrajectoryPoint *point, size_t &length)
{
    return (manual_traj_.frame == JOINT || manual_traj_.mode == APOINT) ? pickFromManualJoint(point, length) : pickFromManualCartesian(point, length);
}

ErrorCode BaseGroup::pickFromManualJoint(TrajectoryPoint *point, size_t &length)
{
    size_t cnt = 0;
    double *angle, *start, *target;
    double tm, omega;

    //FST_INFO("Pick from manual joint");
    //FST_INFO("manual-time=%.4f", manual_time_);

    for (size_t i = 0 ; i < length; i++)
    {
        point[i].level = manual_time_ > MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
        memset(&point[i].omega, 0, sizeof(Joint));
        memset(&point[i].alpha, 0, sizeof(Joint));
        //memset(&point[i].inertia, 0, sizeof(Joint));
        //memset(&point[i].gravity, 0, sizeof(Joint));
        memset(&point[i].ma_cv_g, 0, sizeof(Joint));

        angle  = (double*)&point[i].angle;
        start  = (double*)&manual_traj_.joint_start;
        target = (double*)&manual_traj_.joint_ending;

        manual_time_ += cycle_time_;

        for (size_t jnt = 0; jnt < JOINT_OF_ARM; jnt++)
        {
            if (manual_time_ < manual_traj_.coeff[jnt].start_time)
            {
                *angle = *start;
            }
            else if (manual_time_ < manual_traj_.coeff[jnt].stable_time)
            {
                tm = manual_time_ - manual_traj_.coeff[jnt].start_time;
                *angle = *start + manual_traj_.coeff[jnt].start_alpha * tm * tm / 2;
            }
            else if (manual_time_ < manual_traj_.coeff[jnt].brake_time)
            {
                tm = manual_traj_.coeff[jnt].stable_time - manual_traj_.coeff[jnt].start_time;
                omega = manual_traj_.coeff[jnt].start_alpha * tm;
                *angle = *start + omega * tm / 2;
                tm = manual_time_ - manual_traj_.coeff[jnt].stable_time;
                *angle = *angle + omega * tm;
            }
            else if (manual_time_ < manual_traj_.coeff[jnt].stop_time)
            {
                tm = manual_traj_.coeff[jnt].stable_time - manual_traj_.coeff[jnt].start_time;
                omega = manual_traj_.coeff[jnt].start_alpha * tm;
                *angle = *start + omega * tm / 2;
                tm = manual_traj_.coeff[jnt].brake_time - manual_traj_.coeff[jnt].stable_time;
                *angle = *angle + omega * tm;
                tm = manual_time_ - manual_traj_.coeff[jnt].brake_time;
                *angle = *angle + omega * tm + manual_traj_.coeff[jnt].brake_alpha * tm * tm / 2;
            }
            else
            {
                *angle = *target;
            }

            ++ angle;
            ++ start;
            ++ target;
        }

        cnt ++;

        if (manual_time_ >= manual_traj_.duration)
        {
            point[i].level = POINT_ENDING;
            FST_INFO("%d - %.4f - %.6f %.6f %.6f %.6f %.6f %.6f", point[i].level, manual_time_,
                     point[i].angle.j1, point[i].angle.j2,  point[i].angle.j3,
                     point[i].angle.j4, point[i].angle.j5,  point[i].angle.j6);

            manual_traj_.direction[0] = STANDING;
            manual_traj_.direction[1] = STANDING;
            manual_traj_.direction[2] = STANDING;
            manual_traj_.direction[3] = STANDING;
            manual_traj_.direction[4] = STANDING;
            manual_traj_.direction[5] = STANDING;
            manual_traj_.duration = 0;
            memset(manual_traj_.coeff, 0, JOINT_OF_ARM * sizeof(ManualCoef));
            start_joint_ = manual_traj_.joint_ending;
            manual_time_ = 0;
            group_state_ = MANUAL_TO_STANDBY;
            break;
        }
        else
        {
            //FST_INFO("%d - %.3f - %.6f %.6f %.6f %.6f %.6f %.6f", point[i].level, manual_time_,
            //              point[i].angle.j1, point[i].angle.j2, point[i].angle.j3,
            //              point[i].angle.j4, point[i].angle.j5, point[i].angle.j6);
            continue;
        }
    }

    length = cnt;
    return SUCCESS;
}

ErrorCode BaseGroup::pickFromManualCartesian(TrajectoryPoint *point, size_t &length)
{
    ErrorCode err = SUCCESS;
    PoseEuler pose;
    double tim, vel;
    double *axis, *start, *target;
    size_t cnt = 0;

    FST_INFO("Pick from manual cartesian");
    FST_INFO("manual-time=%.4f", manual_time_);

    for (size_t i = 0; i < length; i++)
    {
        point[i].level = manual_time_ > MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
        memset(&point[i].omega, 0, sizeof(Joint));
        memset(&point[i].alpha, 0, sizeof(Joint));
        memset(&point[i].ma_cv_g, 0, sizeof(Joint));
        //memset(&point[i].inertia, 0, sizeof(Joint));
        //memset(&point[i].gravity, 0, sizeof(Joint));

        axis = (double *) &pose.position.x;
        start = (double *) &manual_traj_.cart_start.position.x;
        target = (double *) &manual_traj_.cart_ending.position.x;

        manual_time_ += cycle_time_;

        for (size_t i = 0; i < 6; i++)
        {
            if (manual_time_ < manual_traj_.coeff[i].start_time)
            {
                *axis = *start;
            }
            else if (manual_time_ < manual_traj_.coeff[i].stable_time)
            {
                tim = manual_time_ - manual_traj_.coeff[i].start_time;
                *axis = *start + manual_traj_.coeff[i].start_alpha * tim * tim / 2;
            }
            else if (manual_time_ < manual_traj_.coeff[i].brake_time)
            {
                tim = manual_traj_.coeff[i].stable_time - manual_traj_.coeff[i].start_time;
                vel = manual_traj_.coeff[i].start_alpha * tim;
                *axis = *start + vel * tim / 2;
                tim = manual_time_ - manual_traj_.coeff[i].stable_time;
                *axis = *axis + vel * tim;
            }
            else if (manual_time_ < manual_traj_.coeff[i].stop_time)
            {
                tim = manual_traj_.coeff[i].stable_time - manual_traj_.coeff[i].start_time;
                vel = manual_traj_.coeff[i].start_alpha * tim;
                *axis = *start + vel * tim / 2;
                tim = manual_traj_.coeff[i].brake_time - manual_traj_.coeff[i].stable_time;
                *axis = *axis + vel * tim;
                tim = manual_time_ - manual_traj_.coeff[i].brake_time;
                *axis = *axis + vel * tim + manual_traj_.coeff[i].brake_alpha * tim * tim / 2;
            }
            else
            {
                *axis = *target;
            }

            ++axis;
            ++start;
            ++target;
        }

        Joint ref_joint = getLatestJoint();

        switch (manual_traj_.frame)
        {
            case BASE:
                err = kinematics_ptr_->inverseKinematicsInBase(pose, ref_joint, point[i].angle);
                break;
            case USER:
                err = kinematics_ptr_->inverseKinematicsInUser(pose, ref_joint, point[i].angle);
                break;
            case WORLD:
                err = kinematics_ptr_->inverseKinematicsInWorld(pose, ref_joint, point[i].angle);
                break;
            case TOOL:
                err = kinematics_ptr_->inverseKinematicsInTool(manual_traj_.tool_coordinate, pose, ref_joint, point[i].angle);
                break;
            case JOINT:
            default:
                err = MOTION_INTERNAL_FAULT;
                break;
        }

        if (err == SUCCESS && soft_constraint_.isJointInConstraint(point[i].angle))
        {
            cnt++;

            if (manual_time_ >= manual_traj_.duration)
            {
                point[i].level = POINT_ENDING;
                FST_INFO("%d - %.4f - %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", point[i].level, manual_time_,
                         point[i].angle.j1, point[i].angle.j2, point[i].angle.j3, point[i].angle.j4, point[i].angle.j5, point[i].angle.j6);

                manual_traj_.direction[0] = STANDING;
                manual_traj_.direction[1] = STANDING;
                manual_traj_.direction[2] = STANDING;
                manual_traj_.direction[3] = STANDING;
                manual_traj_.direction[4] = STANDING;
                manual_traj_.direction[5] = STANDING;
                manual_traj_.duration = 0;
                memset(manual_traj_.coeff, 0, 6 * sizeof(ManualCoef));
                start_joint_ = point[i].angle;
                manual_time_ = 0;
                group_state_ = MANUAL_TO_STANDBY;
                break;
            }
            else
            {
                //FST_INFO("%d - %.4f - %.6f %.6f %.6f %.6f %.6f %.6f", point[i].level, manual_time_,
                //         point[i].angle.j1, point[i].angle.j2, point[i].angle.j3,
                //         point[i].angle.j4, point[i].angle.j5, point[i].angle.j6);
                continue;
            }
        }
        else
        {
            if (err != SUCCESS)
            {
                FST_ERROR("pickFromManualCartesian: IK failed.");
                break;
            }
            else
            {
                FST_ERROR("pickFromManualCartesian: IK result out of soft constraint.");
                err = JOINT_OUT_OF_CONSTRAINT;
                break;
            }
        }
    }

    length = cnt;
    return err;
}


ErrorCode BaseGroup::pickFromAuto(TrajectoryPoint *point, size_t &length)
{
    size_t pick_num = 0;
    size_t joint_num = getNumberOfJoint();
    MotionTime seg_tm;

    FST_INFO("Pick from auto move");
    FST_INFO("auto-time=%.4f", auto_time_);

    for (size_t i = 0; i < length; i++)
    {
        auto_time_ += cycle_time_;

        while (!traj_fifo_.empty() && auto_time_ > traj_fifo_.front().time_from_start)
        {
            traj_fifo_.dropFront();
        }

        createTrajectory();

        if (!traj_fifo_.empty())
        {
            if (auto_time_ < traj_fifo_.front().time_from_start)
            {
                point[i].level = auto_time_ > cycle_time_ + MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
                seg_tm = auto_time_ - (traj_fifo_.front().time_from_start - traj_fifo_.front().duration);

                ErrorCode err = sampleTrajectorySegment(traj_fifo_.front().traj_coeff, seg_tm, point[i].angle, point[i].omega, point[i].alpha);

                FST_INFO("%.4f: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", auto_time_,
                         point[i].angle[0], point[i].angle[1], point[i].angle[2], point[i].angle[3], point[i].angle[4], point[i].angle[5],
                         point[i].omega[0], point[i].omega[1], point[i].omega[2], point[i].omega[3], point[i].omega[4], point[i].omega[5],
                         point[i].alpha[0], point[i].alpha[1], point[i].alpha[2], point[i].alpha[3], point[i].alpha[4], point[i].alpha[5]);

                if (err == SUCCESS)
                {
                    computeCompensate(traj_fifo_.front().dynamics_product, point[i].omega, point[i].alpha, point[i].ma_cv_g);
                    pick_num ++;
                }
                else
                {
                    FST_ERROR("pickFromAuto: sampling error! auto-time = %.4f, time-from-start = %.4f, duration = %.4f",
                              auto_time_, traj_fifo_.front().time_from_start, traj_fifo_.front().duration);
                    length = pick_num;
                    return err;
                }
            }
            else
            {
                FST_ERROR("pickFromAuto: auto-time error! auto-time = %.4f, time-from-start = %.4f, duration = %.4f",
                          auto_time_, traj_fifo_.front().time_from_start, traj_fifo_.front().duration);
                length = pick_num;
                return MOTION_INTERNAL_FAULT;
            }
        }
        else
        {
            ErrorCode err = sampleTrajectorySegment(traj_fifo_.front().traj_coeff, traj_fifo_.front().duration, point[i].angle, point[i].omega, point[i].alpha);

            /*
            FST_INFO("duration = %.4f, %.4f, %.4f, %.4f", traj_fifo_.front().traj_coeff[0].duration[0], traj_fifo_.front().traj_coeff[0].duration[1], traj_fifo_.front().traj_coeff[0].duration[2], traj_fifo_.front().traj_coeff[0].duration[3]);
            FST_INFO("coeff = %.6f, %.6f, %.6f, %.6f", traj_fifo_.front().traj_coeff[0].coeff[0][3], traj_fifo_.front().traj_coeff[0].coeff[0][2], traj_fifo_.front().traj_coeff[0].coeff[0][1], traj_fifo_.front().traj_coeff[0].coeff[0][0]);
            FST_INFO("        %.6f, %.6f, %.6f, %.6f", traj_fifo_.front().traj_coeff[0].coeff[1][3], traj_fifo_.front().traj_coeff[0].coeff[1][2], traj_fifo_.front().traj_coeff[0].coeff[1][1], traj_fifo_.front().traj_coeff[0].coeff[1][0]);
            FST_INFO("        %.6f, %.6f, %.6f, %.6f", traj_fifo_.front().traj_coeff[0].coeff[2][3], traj_fifo_.front().traj_coeff[0].coeff[2][2], traj_fifo_.front().traj_coeff[0].coeff[2][1], traj_fifo_.front().traj_coeff[0].coeff[2][0]);
            FST_INFO("        %.6f, %.6f, %.6f, %.6f", traj_fifo_.front().traj_coeff[0].coeff[3][3], traj_fifo_.front().traj_coeff[0].coeff[3][2], traj_fifo_.front().traj_coeff[0].coeff[3][1], traj_fifo_.front().traj_coeff[0].coeff[3][0]);
            */
            FST_WARN("%.4f: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", auto_time_,
                     point[i].angle[0], point[i].angle[1], point[i].angle[2], point[i].angle[3], point[i].angle[4], point[i].angle[5],
                     point[i].omega[0], point[i].omega[1], point[i].omega[2], point[i].omega[3], point[i].omega[4], point[i].omega[5],
                     point[i].alpha[0], point[i].alpha[1], point[i].alpha[2], point[i].alpha[3], point[i].alpha[4], point[i].alpha[5]);

            if (err == SUCCESS)
            {
                for (size_t j = 0; j < joint_num; j++)
                {
                    point[i].omega[j] = 0;
                    point[i].alpha[j] = 0;
                    //point[i].torque[j] = traj_fifo_.front().dynamics_product[j].torque;
                    //point[i].inertia[j] = traj_fifo_.front().dynamics_product[j].inertia;
                    //point[i].gravity[j] = traj_fifo_.front().dynamics_product[j].gravity;
                }

                point[i].level = POINT_ENDING;
                pick_num ++;
            }
            else
            {
                FST_ERROR("pickFromAuto: sampling error! auto-time = %.4f, time-from-start = %.4f, duration = %.4f",
                          auto_time_, traj_fifo_.front().time_from_start, traj_fifo_.front().duration);
                length = pick_num;
                return err;
            }

            /*
            tos << pick_time_ << ","
                << jout.joint.j1 << "," << jout.joint.j2 << "," << jout.joint.j3 << ","
                << jout.joint.j4 << "," << jout.joint.j5 << "," << jout.joint.j6 << ","
                << jout.omega.j1 << "," << jout.omega.j2 << "," << jout.omega.j3 << ","
                << jout.omega.j4 << "," << jout.omega.j5 << "," << jout.omega.j6 << endl;
            */

            char buffer[LOG_TEXT_SIZE];
            FST_INFO("%d - %.4f - %s", point[i].level, auto_time_, printDBLine(&point[i].angle.j1, buffer, LOG_TEXT_SIZE));
            FST_INFO("Command ID = %d finished", traj_fifo_.front().id);
            //auto_running_ = false;

            if (group_state_ == AUTO)
            {
                group_state_ = AUTO_TO_STANDBY;
            }

            traj_fifo_.clear();

            break;
        }
    }

    length = pick_num;
    return SUCCESS;
}

ErrorCode BaseGroup::computeCompensate(const DynamicsProduct &product, const Joint &omega, const Joint &alpha, Joint &ma_cv_g)
{
    double ma, cv;
    size_t joint_num = getNumberOfJoint();

    for (size_t i = 0; i < joint_num; i++)
    {
        ma = 0;
        cv = 0;

        for (size_t j = 0; j < joint_num; j++)
        {
            ma += product.m[i][j] * alpha[j];
            cv += product.c[i][j] * omega[j];
        }

        ma_cv_g[i] = ma + cv + product.g[i];
    }

    return SUCCESS;
}


ErrorCode BaseGroup::sampleTrajectorySegment(const TrajSegment (&segment)[NUM_OF_JOINT], double time, Joint &angle, Joint &omega, Joint &alpha)
{
    size_t ind;
    size_t joint_num = getNumberOfJoint();

    for (size_t j = 0; j < joint_num; j++)
    {
        double tm = time;
        const double (&coeff)[4][4] = segment[j].coeff;
        const double (&duration)[4] = segment[j].duration;

        if (tm < duration[0]) { ind = 0; goto sample_by_time; }
        tm -= duration[0];
        if (tm < duration[1]) { ind = 1; goto sample_by_time; }
        tm -= duration[1];
        if (tm < duration[2]) { ind = 2; goto sample_by_time; }
        tm -= duration[2];
        if (tm < duration[3] + MINIMUM_E9) { ind = 3; goto sample_by_time; }
        else
        {
            FST_ERROR("Time error! time = %.4f, duration0 = %.4f, duration1 = %.4f, duration2 = %.4f, duration3 = %.4f",
                      time, duration[0], duration[1], duration[2], duration[3]);
            return MOTION_INTERNAL_FAULT;
        }

sample_by_time:
        MotionTime tm_array[4] = {1.0, tm, tm * tm, tm * tm * tm};

        angle[j] = coeff[ind][3] * tm_array[3] + coeff[ind][2] * tm_array[2] + coeff[ind][1] * tm_array[1] + coeff[ind][0];
        omega[j] = coeff[ind][3] * tm_array[2] * 3 + coeff[ind][2] * tm_array[1] * 2 + coeff[ind][1];
        alpha[j] = coeff[ind][3] * tm_array[1] * 6 + coeff[ind][2] * 2;
    }

    return SUCCESS;
}



void BaseGroup::realtimeTask(void)
{
    ErrorCode   err;
    ServoState  barecore_state;
    Joint   barecore_joint;
    Pose    barecore_pose;
    size_t  send_fail_cnt = 0;
    size_t  stable_cnt = 0;

    FST_WARN("Realtime task start.");
    memset(&barecore_joint, 0, sizeof(barecore_joint));

    while (rt_task_active_)
    {
        if (bare_core_.getLatestJoint(barecore_joint, barecore_state))
        {
            pthread_mutex_lock(&servo_mutex_);
            servo_state_ = barecore_state;
            current_joint_ = barecore_joint;
            pthread_mutex_unlock(&servo_mutex_);
        }
        else
        {
            FST_ERROR("Lost communication with bare core.");
            reportError(BARE_CORE_TIMEOUT);
        }

        if (waiting_fine_)
        {
            if (start_waiting_cnt_ > 0)
            {
                start_waiting_cnt_ --;
            }
            else
            {
                if (group_state_ == STANDBY && servo_state_ == SERVO_IDLE)
                {
                    if (waiting_motion_type_ == MOTION_JOINT)
                    {
                        if (isSameJoint(waiting_joint_, barecore_joint))
                        {
                            stable_cnt ++;
                        }
                        else
                        {
                            stable_cnt = 0;
                        }
                    }
                    else
                    {
                        kinematics_ptr_->forwardKinematicsInUser(barecore_joint, barecore_pose);

                        if (getDistance(barecore_pose.position, waiting_pose_.position) < 0.05)
                        {
                            stable_cnt ++;
                        }
                        else
                        {
                            stable_cnt = 0;
                        }
                    }

                    if (stable_cnt > 5)
                    {
                        FST_WARN("Waiting-fine: group is stable.");
                        waiting_fine_ = false;
                    }
                }
            }
        }

        if ((servo_state_ == SERVO_IDLE || servo_state_ == SERVO_RUNNING) &&
            (group_state_ == MANUAL || group_state_ == MANUAL_TO_STANDBY || group_state_ == AUTO || group_state_ == AUTO_TO_STANDBY))
        {
            err = sendPoint();

            if (err == SUCCESS)
            {
                send_fail_cnt = 0;

                if (group_state_ == MANUAL_TO_STANDBY)
                {
                    group_state_ = STANDBY;
                }
                else if (group_state_ == AUTO_TO_STANDBY)
                {
                    group_state_ = STANDBY;
                }
            }
            else
            {
                send_fail_cnt++;

                if (send_fail_cnt > 10)
                {
                    send_fail_cnt = 0;
                    FST_ERROR("Cannot send point to bare core");
                    reportError(err);
                }
            }
        }

        /*
        cnt ++;
        if (cnt > 200)
        {
            cnt = 0;
            FST_INFO("servo-state=%d, joint=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", servo_state_,
                     current_joint_[0], current_joint_[1], current_joint_[2], current_joint_[3], current_joint_[4], current_joint_[5]);
        }
         */

        usleep(5000);
    }

    FST_WARN("Realtime task quit.");
}

bool BaseGroup::isSameJoint(const Joint &joint1, const Joint &joint2)
{
    size_t  joint_num = getNumberOfJoint();

    for (size_t i = 0; i < joint_num; i++)
    {
        if (fabs(joint1[i] - joint2[i]) > MINIMUM_E6)
        {
            return false;
        }
    }

    return true;
}

void BaseGroup::inactiveRealtimeTask(void)
{
    rt_task_active_ = false;
}

void BaseGroup::activeRealtimeTask(void)
{
    rt_task_active_ = true;
}

BaseKinematics* BaseGroup::getKinematicsPtr(void)
{
    return kinematics_ptr_;
}

Calibrator* BaseGroup::getCalibratorPtr(void)
{
    return &calibrator_;
}

Constraint* BaseGroup::getSoftConstraintPtr(void)
{
    return &soft_constraint_;
}

ErrorCode BaseGroup::getSoftConstraint(JointConstraint &soft_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    soft_constraint_.getConstraint(soft_constraint);
    FST_INFO("Get soft constraint:");
    FST_INFO("  lower = %s", printDBLine(&soft_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&soft_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::getFirmConstraint(JointConstraint &firm_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    firm_constraint_.getConstraint(firm_constraint);
    FST_INFO("Get firm constraint.");
    FST_INFO("  lower = %s", printDBLine(&firm_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&firm_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::getHardConstraint(JointConstraint &hard_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    hard_constraint_.getConstraint(hard_constraint);
    FST_INFO("Get hard constraint.");
    FST_INFO("  lower = %s", printDBLine(&hard_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&hard_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::setSoftConstraint(const JointConstraint &soft_constraint)
{
    Joint lower, upper;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set soft constraint.");

    soft_constraint_.getConstraint(lower, upper);
    FST_INFO("Origin soft constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    FST_INFO("Given soft constraint:");
    FST_INFO("  lower = %s", printDBLine(&soft_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&soft_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    firm_constraint_.getConstraint(lower, upper);
    FST_INFO("Firm constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    if (firm_constraint_.isCoverConstaint(soft_constraint))
    {
        ParamGroup param;
        vector<double> v_upper(&soft_constraint.upper[0], &soft_constraint.upper[0] + NUM_OF_JOINT);
        vector<double> v_lower(&soft_constraint.lower[0], &soft_constraint.lower[0] + NUM_OF_JOINT);

        for (size_t i = getNumberOfJoint(); i < NUM_OF_JOINT; i++)
        {
            v_upper[i] = 0;
            v_lower[i] = 0;
        }

        if (param.loadParamFile(AXIS_GROUP_DIR"soft_constraint.yaml") &&
            param.setParam("soft_constraint/upper", v_upper) &&
            param.setParam("soft_constraint/lower", v_lower) &&
            param.dumpParamFile())
        {
            soft_constraint_.setConstraint(soft_constraint);
            FST_INFO("Soft constraint updated successfully.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail dumping soft constraint to config file");
            return param.getLastError();
        }
    }
    else
    {
        FST_ERROR("Given soft constraint out of firm constraint.");
        return INVALID_PARAMETER;
    }
}


ErrorCode BaseGroup::setFirmConstraint(const JointConstraint &firm_constraint)
{
    Joint lower, upper;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set firm constraint.");

    firm_constraint_.getConstraint(lower, upper);
    FST_INFO("Origin firm constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    FST_INFO("Given firm constraint:");
    FST_INFO("  lower = %s", printDBLine(&firm_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&firm_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    hard_constraint_.getConstraint(lower, upper);
    FST_INFO("Hard constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    if (hard_constraint_.isCoverConstaint(firm_constraint_))
    {
        ParamGroup param;
        vector<double> v_upper(&firm_constraint.upper[0], &firm_constraint.upper[0] + NUM_OF_JOINT);
        vector<double> v_lower(&firm_constraint.lower[0], &firm_constraint.lower[0] + NUM_OF_JOINT);

        for (size_t i = getNumberOfJoint(); i < NUM_OF_JOINT; i++)
        {
            v_upper[i] = 0;
            v_lower[i] = 0;
        }

        if (param.loadParamFile(AXIS_GROUP_DIR"firm_constraint.yaml") &&
            param.setParam("firm_constraint/upper", v_upper) &&
            param.setParam("firm_constraint/lower", v_lower) &&
            param.dumpParamFile())
        {
            firm_constraint_.setConstraint(firm_constraint);
            FST_INFO("Firm constraint updated successfully.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail dumping firm constraint to config file");
            return param.getLastError();
        }
    }
    else
    {
        FST_ERROR("Given firm constraint out of hard constraint.");
        return INVALID_PARAMETER;
    }
}


ErrorCode BaseGroup::setHardConstraint(const JointConstraint &hard_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set hard constraint.");
    FST_INFO("Origin hard constraint:");
    FST_INFO("  lower = %s", printDBLine(&hard_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&hard_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));

    FST_INFO("Given hard constraint:");
    FST_INFO("  lower = %s", printDBLine(&hard_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&hard_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    ParamGroup param;
    vector<double> v_upper(&hard_constraint.upper[0], &hard_constraint.upper[0] + NUM_OF_JOINT);
    vector<double> v_lower(&hard_constraint.lower[0], &hard_constraint.lower[0] + NUM_OF_JOINT);

    for (size_t i = getNumberOfJoint(); i < NUM_OF_JOINT; i++)
    {
        v_upper[i] = 0;
        v_lower[i] = 0;
    }

    if (param.loadParamFile(AXIS_GROUP_DIR"hard_constraint.yaml") &&
        param.setParam("hard_constraint/upper", v_upper) &&
        param.setParam("hard_constraint/lower", v_lower) &&
        param.dumpParamFile())
    {
        hard_constraint_.setConstraint(hard_constraint);
        FST_INFO("Hard constraint updated successfully.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail dumping hard constraint to config file");
        return param.getLastError();
    }
}



}

