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
#include <common_file_path.h>

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

    auto_cache_[0].deadline = 0;
    auto_cache_[0].valid = false;
    auto_cache_[0].head = 0;
    auto_cache_[0].tail = 0;
    auto_cache_[0].smooth_in_stamp = 0;
    auto_cache_[0].smooth_out_stamp = 0;
    auto_cache_[0].next = &auto_cache_[1];
    auto_cache_[0].prev = &auto_cache_[1];

    auto_cache_[1].deadline = 0;
    auto_cache_[1].valid = false;
    auto_cache_[1].head = 0;
    auto_cache_[1].tail = 0;
    auto_cache_[1].smooth_in_stamp = 0;
    auto_cache_[1].smooth_out_stamp = 0;
    auto_cache_[1].next = &auto_cache_[0];
    auto_cache_[1].prev = &auto_cache_[0];

    auto_cache_ptr_ = &auto_cache_[0];

    vel_ratio_ = 0;
    acc_ratio_ = 0;
}

BaseGroup::~BaseGroup()
{}

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
    return bare_core_.stopBareCore() == true ? SUCCESS : BARE_CORE_TIMEOUT;
}

ErrorCode BaseGroup::clearGroup(void)
{
    FST_INFO("Clear group, current group state = %d", group_state_);

    group_state_ = STANDBY;
    manual_time_ = 0;
    memset(&manual_traj_, 0, sizeof(manual_traj_));
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

    if (soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
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

    if (soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
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

    if (soft_constraint_.isJointInConstraint(res_joint))
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
            err = autoJoint(id, target);
            break;
        case MOTION_LINE:
            err = autoLine(id, target);
            break;
        case MOTION_CIRCLE:
            err = autoCircle(id, target);
            break;
        default:
            FST_ERROR("Invalid motion type, auto move aborted.");
            return INVALID_PARAMETER;
    }

    return err;
}

ErrorCode BaseGroup::autoJoint(int id, const MotionTarget &target)
{
    char buffer[LOG_TEXT_SIZE];
    Joint start_joint;
    FST_INFO("autoJoint: vel = %.4f, cnt = %.4f", target.vel, target.cnt);
    FST_INFO("  target = %s", printDBLine(&target.joint_target[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(target.joint_target))
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

    size_t  index, length;
    double  precision = 0.01;
    Joint   path[MAX_PATH_SIZE];
    planJointPath(start_joint, target.joint_target, precision, index, path, length);
    double  expect_duration = precision / axis_vel_[index] / vel_ratio_;
    TrajectoryCache *p_cache = auto_cache_ptr_->valid == false ? auto_cache_ptr_ : auto_cache_ptr_->next;

    for (size_t i = 0; i < length; i++)
    {
        p_cache->cache[i].path_point.id = id;
        p_cache->cache[i].path_point.stamp = i;
        p_cache->cache[i].path_point.type  = MOTION_JOINT;
        p_cache->cache[i].path_point.joint = path[i];
        p_cache->cache[i].command_duration = expect_duration;

        p_cache->cache[i].time_from_start = -1;
        p_cache->cache[i].forward_duration = -1;
        p_cache->cache[i].backward_duration = -1;
    }

    p_cache->head = 0;
    p_cache->tail = length - 1;

    ErrorCode err = planFirstStageTraj(*p_cache);

    FST_INFO("SUCCESS.");
    return SUCCESS;
}

ErrorCode BaseGroup::planFirstStageTraj(TrajectoryCache &cache)
{

}

ErrorCode BaseGroup::autoLine(int id, const MotionTarget &target)
{
    return SUCCESS;
}

ErrorCode BaseGroup::autoCircle(int id, const MotionTarget &target)
{
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
    if (group_state_ == MANUAL || group_state_ == MANUAL_TO_STANDBY )
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
    else if (group_state_ == AUTO)
    {
        // TODO
        return SUCCESS;
    }
    else
    {
        return INVALID_SEQUENCE;
    }
}


void BaseGroup::realtimeTask(void)
{
    ErrorCode  err;
    Joint barecore_joint;
    ServoState barecore_state;
    size_t send_fail_cnt = 0;
    memset(&barecore_joint, 0, sizeof(barecore_joint));
    FST_WARN("Realtime task start.");

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

        if ((servo_state_ == SERVO_IDLE || servo_state_ == SERVO_RUNNING) &&
            (group_state_ == MANUAL || group_state_ == MANUAL_TO_STANDBY || group_state_ == AUTO))
        {
            err = sendPoint();

            if (err == SUCCESS)
            {
                send_fail_cnt = 0;

                if (group_state_ == MANUAL_TO_STANDBY)
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

