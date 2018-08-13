/*************************************************************************
	> File Name: motion_control_arm_group.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 14时34分07秒
 ************************************************************************/

#include <motion_control_arm_group.h>

#define FST_LOG(fmt, ...)       log_ptr_->log(fmt, ##__VA_ARGS__)
#define FST_INFO(fmt, ...)      log_ptr_->info(fmt, ##__VA_ARGS__)
#define FST_WARN(fmt, ...)      log_ptr_->warn(fmt, ##__VA_ARGS__)
#define FST_ERROR(fmt, ...)     log_ptr_->error(fmt, ##__VA_ARGS__)

namespace fst_mc
{

ErrorCode ArmGroup::initGroup(void)
{
    cycle_time_ = 0.001;

    soft_constraint_.upper[0] = 2.8;
    soft_constraint_.upper[1] = 1.6;
    soft_constraint_.upper[2] = 3.3;
    soft_constraint_.upper[3] = 3.1;
    soft_constraint_.upper[4] = 1.8;
    soft_constraint_.upper[5] = 6.0;
    soft_constraint_.lower[0] = -2.8;
    soft_constraint_.lower[1] = -2.2;
    soft_constraint_.lower[2] = -1.0;
    soft_constraint_.lower[3] = -3.1;
    soft_constraint_.lower[4] = -1.8;
    soft_constraint_.lower[5] = -6.0;

    memset(&manual_traj_, 0, sizeof(ManualTrajectory));
    return SUCCESS;
}

ErrorCode ArmGroup::setManualMode(ManualMode mode)
{
    FST_INFO("Set manual mode = %d", mode);

    if (group_state_ == STANDBY)
    {
        manual_traj_.mode = mode;
        FST_INFO("Done.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Cannot set mode in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }
}

ErrorCode ArmGroup::setManualFrame(ManualFrame frame)
{
    FST_INFO("Set manual frame = %d", frame);

    if (group_state_ == STANDBY)
    {
        manual_traj_.frame = frame;
        FST_INFO("Done.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Cannot set frame in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }
}

ErrorCode ArmGroup::manualMove(const Joint &joint)
{
    FST_INFO("Manual (mode=%d, frame=%d) by target: %.6f %.6f %.6f %.6f %.6f %.6f",
             manual_traj_.mode, manual_traj_.frame, joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);

    if (manual_traj_.mode != APOINT || manual_traj_.frame != JOINT)
    {
        FST_ERROR("Cannot manual to target in current mode = %d", manual_traj_.mode);
        return INVALID_SEQUENCE;
    }

    if (group_state_ != STANDBY)
    {
        FST_ERROR("Cannot manual to target in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    bare_core_.getLatestJoint(manual_traj_.joint_start);

    if (!isJointInConstraint(manual_traj_.joint_start, soft_constraint_))
    {
        FST_ERROR("start-joint = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                  manual_traj_.joint_start[0], manual_traj_.joint_start[1], manual_traj_.joint_start[2],
                  manual_traj_.joint_start[3], manual_traj_.joint_start[4], manual_traj_.joint_start[5]);
        FST_ERROR("start-joint is out of soft constraint, manual-mode-apoint is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (!isJointInConstraint(joint, soft_constraint_))
    {
        FST_ERROR("target-joint out of constraint: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                  joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6);
        return JOINT_OUT_OF_CONSTRAINT;
    }

    manual_time_ = 0;
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

ErrorCode ArmGroup::manualMove(const ManualDirection *direction)
{
    FST_INFO("Manual (mode=%d, frame=%d) by direction: %d %d %d %d %d %d",
             manual_traj_.mode, manual_traj_.frame,
             direction[0], direction[1], direction[2], direction[3], direction[4], direction[5]);

    if (manual_traj_.mode == STEP && group_state_ != STANDBY)
    {
        FST_ERROR("Cannot manual step in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    if (manual_traj_.mode == APOINT)
    {
        FST_ERROR("Given directions in manual-to-target mode");
        return INVALID_PARAMETER;
    }

    if (group_state_ == STANDBY)
    {
        bare_core_.getLatestJoint(manual_traj_.joint_start);
        FST_INFO("start joint: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                manual_traj_.joint_start[0], manual_traj_.joint_start[1], manual_traj_.joint_start[2], 
                manual_traj_.joint_start[3], manual_traj_.joint_start[4], manual_traj_.joint_start[5]);

        if (!isJointInConstraint(manual_traj_.joint_start, soft_constraint_))
        {
            if (manual_traj_.frame != JOINT)
            {
                FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
                return INVALID_SEQUENCE;
            }
            if (manual_traj_.mode == APOINT)
            {
                FST_ERROR("start-joint is out of soft constraint, manual-mode-apoint is disabled.");
                return INVALID_SEQUENCE;
            }
            
            for (size_t i = 0; i < JOINT_OF_ARM; i++)
            {
                if (manual_traj_.joint_start[i] > soft_constraint_.upper[i] + MINIMUM_E9 && direction[i] == INCREASE)
                {
                    FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                              i + 1, manual_traj_.joint_start[i], soft_constraint_.lower[i], soft_constraint_.upper[i]);
                    return INVALID_PARAMETER;
                }
                else if (manual_traj_.joint_start[i] < soft_constraint_.lower[i] - MINIMUM_E9 && direction[i] == DECREASE)
                {
                    FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                              i + 1, manual_traj_.joint_start[i], soft_constraint_.lower[i], soft_constraint_.upper[i]);
                    return INVALID_PARAMETER;
                }
            }
        }

        manual_time_ = 0;
        ErrorCode err = manual_teach_.manualByDirect(direction, manual_time_, manual_traj_);
        
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
        for (size_t i = 0; i < JOINT_OF_ARM; i++)
        {
            if (manual_traj_.direction[i] != direction[i])
            {
                return manual_teach_.manualByDirect(direction, manual_time_, manual_traj_);
            }
            else
            {
                continue;
            }
        }

        return SUCCESS;
    }

    FST_ERROR("Cannot manual now, current state = %d", group_state_);
    return INVALID_SEQUENCE;
}

ErrorCode ArmGroup::manualStop(void)
{
    FST_INFO("Manual stop request received.");

    if (group_state_ == MANUAL)
    {
        FST_INFO("Manual mode = %d, frame = %d", manual_traj_.mode, manual_traj_.frame);
        
        if (manual_traj_.mode == CONTINUOUS)
        {
            ManualDirection direction[JOINT_OF_ARM] = {STANDING};
            ErrorCode err = manual_teach_.manualByDirect(direction, manual_time_, manual_traj_);
            
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

ErrorCode ArmGroup::sendPoint(void)
{
    if (group_state_ == MANUAL)
    {
        if (bare_core_.isPointCacheEmpty())
        {
            size_t length = 10;
            TrajectoryPoint point[10];
            pickFromManual(point, length);
            bare_core_.fillPointCache(point, length, POINT_POS);
        }

        bare_core_.sendPoint();
        return SUCCESS;
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

ErrorCode ArmGroup::pickFromManual(TrajectoryPoint *point, size_t &length)
{
    return manual_traj_.frame == JOINT ? pickFromManualJoint(point, length) : pickFromManualCartesian(point, length);
}

ErrorCode ArmGroup::pickFromManualJoint(TrajectoryPoint *point, size_t &length)
{
    size_t cnt = 0;
    double *angle, *start, *target;
    double tm, omega;

    FST_INFO("Pick from manual joint");
    FST_INFO("manual-time=%.4f", manual_time_);

    for (size_t i = 0 ; i < length; i++)
    {
        point[i].level = manual_time_ > MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
        memset(&point[i].omega, 0, sizeof(Joint));
        memset(&point[i].alpha, 0, sizeof(Joint));
        memset(&point[i].torque, 0, sizeof(Joint));
        memset(&point[i].inertia, 0, sizeof(Joint));
        memset(&point[i].gravity, 0, sizeof(Joint));
        
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
            FST_INFO("%d - %.3f - %.6f %.6f %.6f %.6f %.6f %.6f", point[i].level, manual_time_,
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
            group_state_ = STANDBY;
            break;
        }
        else
        {
            FST_INFO("%d - %.3f - %.6f %.6f %.6f %.6f %.6f %.6f", point[i].level, manual_time_,
                          point[i].angle.j1, point[i].angle.j2, point[i].angle.j3,
                          point[i].angle.j4, point[i].angle.j5, point[i].angle.j6);
            continue;
        }
    }

    length = cnt;
    return SUCCESS;
}

ErrorCode ArmGroup::pickFromManualCartesian(TrajectoryPoint *point, size_t &length)
{
    // TODO
    return SUCCESS;
}



ErrorCode ArmGroup::autoMove(void)
{
    return SUCCESS;
}


bool ArmGroup::isJointInConstraint(Joint joint, JointConstraint constraint)
{
    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        if (joint[i] > constraint.upper[i] + MINIMUM_E9 || joint[i] < constraint.lower[i] - MINIMUM_E9)
        {
            return false;
        }
    }

    return true;
}






}




