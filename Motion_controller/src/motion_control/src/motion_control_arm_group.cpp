/*************************************************************************
	> File Name: motion_control_arm_group.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 14时34分07秒
 ************************************************************************/

#include <math.h>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <motion_control_arm_group.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include "../../parameter_manager/include/parameter_manager/parameter_manager_param_group.h"
#include "../../error_monitor/include/error_monitor.h"


using namespace std;
using namespace fst_base;
using namespace fst_parameter;

namespace fst_mc
{

ErrorCode ArmGroup::initGroup(ErrorMonitor *error_monitor_ptr)
{
    log_ptr_->initLogger("ArmGroup");
    cycle_time_ = 0.001;
    memset(&manual_traj_, 0, sizeof(ManualTrajectory));

    if (error_monitor_ptr == NULL)
    {
        FST_ERROR("Invalid pointer of error-monitor.");
        return MOTION_INTERNAL_FAULT;
    }
    else
    {
        error_monitor_ptr_ = error_monitor_ptr;
    }

    if (pthread_mutex_init(&auto_mutex_, NULL) != 0 ||
        pthread_mutex_init(&manual_mutex_, NULL) != 0 ||
        pthread_mutex_init(&servo_mutex_, NULL) != 0)
    {
        FST_ERROR("Fail to initialize mutex.");
        return MOTION_INTERNAL_FAULT;
    }

    ParamGroup param;
    vector<double> upper;
    vector<double> lower;

    if (param.loadParamFile("hard_constraint.yaml") &&
        param.getParam("hard_constraint/upper", upper) &&
        param.getParam("hard_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            for (size_t i = 0; i < JOINT_OF_ARM; i++)
            {
                hard_constraint_.upper()[i] = upper[i];
                hard_constraint_.lower()[i] = lower[i];
            }
        }
        else
        {
            FST_ERROR("Invalid array size of hard constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading hard constraint from config file");
        return param.getLastError();
    }

    param.reset();
    upper.clear();
    lower.clear();

    if (param.loadParamFile("firm_constraint.yaml") &&
        param.getParam("firm_constraint/upper", upper) &&
        param.getParam("firm_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            for (size_t i = 0; i < JOINT_OF_ARM; i++)
            {
                firm_constraint_.upper()[i] = upper[i];
                firm_constraint_.lower()[i] = lower[i];
            }
        }
        else
        {
            FST_ERROR("Invalid array size of firm constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading firm constraint from config file");
        return param.getLastError();
    }

    param.reset();
    upper.clear();
    lower.clear();

    if (param.loadParamFile("soft_constraint.yaml") &&
        param.getParam("soft_constraint/upper", upper) &&
        param.getParam("soft_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            for (size_t i = 0; i < JOINT_OF_ARM; i++)
            {
                soft_constraint_.upper()[i] = upper[i];
                soft_constraint_.lower()[i] = lower[i];
            }
        }
        else
        {
            FST_ERROR("Invalid array size of soft constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading soft constraint from config file");
        return param.getLastError();
    }

    if (!bare_core_.initInterface())
    {
        FST_ERROR("Fail to create communication with bare core.");
        return BARE_CORE_TIMEOUT;
    }

    ErrorCode err = calibrator_.initCalibrator();

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to initialize calibrator, code = 0x%llx", err);
        return err;
    }

    return SUCCESS;
}


Calibrator* ArmGroup::getCalibratorPtr(void)
{
    return &calibrator_;
}


ErrorCode ArmGroup::getSoftConstraint(JointConstraint &soft_constraint)
{
    FST_INFO("Get soft constraint.");
    soft_constraint_.getConstraint(soft_constraint);
    FST_INFO("  lower = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             soft_constraint.lower[0], soft_constraint.lower[1], soft_constraint.lower[2],
             soft_constraint.lower[3], soft_constraint.lower[4], soft_constraint.lower[5]);
    FST_INFO("  upper = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             soft_constraint.upper[0], soft_constraint.upper[1], soft_constraint.upper[2],
             soft_constraint.upper[3], soft_constraint.upper[4], soft_constraint.upper[5]);
    return SUCCESS;
}


ErrorCode ArmGroup::getFirmConstraint(JointConstraint &firm_constraint)
{
    FST_INFO("Get firm constraint.");
    firm_constraint_.getConstraint(firm_constraint);
    FST_INFO("  lower = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             firm_constraint.lower[0], firm_constraint.lower[1], firm_constraint.lower[2],
             firm_constraint.lower[3], firm_constraint.lower[4], firm_constraint.lower[5]);
    FST_INFO("  upper = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             firm_constraint.upper[0], firm_constraint.upper[1], firm_constraint.upper[2],
             firm_constraint.upper[3], firm_constraint.upper[4], firm_constraint.upper[5]);
    return SUCCESS;
}


ErrorCode ArmGroup::getHardConstraint(JointConstraint &hard_constraint)
{
    FST_INFO("Get hard constraint.");
    hard_constraint_.getConstraint(hard_constraint);
    FST_INFO("  lower = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             hard_constraint.lower[0], hard_constraint.lower[1], hard_constraint.lower[2],
             hard_constraint.lower[3], hard_constraint.lower[4], hard_constraint.lower[5]);
    FST_INFO("  upper = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             hard_constraint.upper[0], hard_constraint.upper[1], hard_constraint.upper[2],
             hard_constraint.upper[3], hard_constraint.upper[4], hard_constraint.upper[5]);
    return SUCCESS;
}


ErrorCode ArmGroup::setSoftConstraint(const JointConstraint &soft_constraint)
{
    Joint lower, upper;
    FST_INFO("Set soft constraint.");

    soft_constraint_.getConstraint(lower, upper);
    FST_INFO("Origin soft constraint:");
    FST_INFO("  lower = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", lower[0], lower[1], lower[2], lower[3], lower[4], lower[5]);
    FST_INFO("  upper = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", upper[0], upper[1], upper[2], upper[3], upper[4], upper[5]);

    FST_INFO("Given soft constraint:");
    FST_INFO("  lower = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             soft_constraint.lower[0], soft_constraint.lower[1], soft_constraint.lower[2],
             soft_constraint.lower[3], soft_constraint.lower[4], soft_constraint.lower[5]);
    FST_INFO("  upper = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             soft_constraint.upper[0], soft_constraint.upper[1], soft_constraint.upper[2],
             soft_constraint.upper[3], soft_constraint.upper[4], soft_constraint.upper[5]);

    firm_constraint_.getConstraint(lower, upper);
    FST_INFO("Firm constraint:");
    FST_INFO("  lower = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", lower[0], lower[1], lower[2], lower[3], lower[4], lower[5]);
    FST_INFO("  upper = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", upper[0], upper[1], upper[2], upper[3], upper[4], upper[5]);

    if (firm_constraint_.isCoverConstaint(soft_constraint))
    {
        ParamGroup param;
        vector<double> v_upper(&soft_constraint.upper[0], &soft_constraint.upper[0] + NUM_OF_JOINT);
        vector<double> v_lower(&soft_constraint.lower[0], &soft_constraint.lower[0] + NUM_OF_JOINT);

        for (size_t i = JOINT_OF_ARM; i < NUM_OF_JOINT; i++)
        {
            v_upper[i] = 0;
            v_lower[i] = 0;
        }

        if (param.loadParamFile("soft_constraint.yaml") &&
            param.setParam("soft_constraint/upper", v_upper) &&
            param.setParam("soft_constraint/lower", v_lower) &&
            param.dumpParamFile("soft_constraint.yaml"))
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


ErrorCode ArmGroup::setFirmConstraint(const JointConstraint &firm_constraint)
{
    Joint lower, upper;
    FST_INFO("Set firm constraint.");

    firm_constraint_.getConstraint(lower, upper);
    FST_INFO("Origin firm constraint:");
    FST_INFO("  lower = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", lower[0], lower[1], lower[2], lower[3], lower[4], lower[5]);
    FST_INFO("  upper = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", upper[0], upper[1], upper[2], upper[3], upper[4], upper[5]);

    FST_INFO("Given firm constraint:");
    FST_INFO("  lower = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             firm_constraint.lower[0], firm_constraint.lower[1], firm_constraint.lower[2],
             firm_constraint.lower[3], firm_constraint.lower[4], firm_constraint.lower[5]);
    FST_INFO("  upper = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             firm_constraint.upper[0], firm_constraint.upper[1], firm_constraint.upper[2],
             firm_constraint.upper[3], firm_constraint.upper[4], firm_constraint.upper[5]);

    hard_constraint_.getConstraint(lower, upper);
    FST_INFO("Hard constraint:");
    FST_INFO("  lower = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", lower[0], lower[1], lower[2], lower[3], lower[4], lower[5]);
    FST_INFO("  upper = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", upper[0], upper[1], upper[2], upper[3], upper[4], upper[5]);

    if (hard_constraint_.isCoverConstaint(firm_constraint_))
    {
        ParamGroup param;
        vector<double> v_upper(&firm_constraint.upper[0], &firm_constraint.upper[0] + NUM_OF_JOINT);
        vector<double> v_lower(&firm_constraint.lower[0], &firm_constraint.lower[0] + NUM_OF_JOINT);

        for (size_t i = JOINT_OF_ARM; i < NUM_OF_JOINT; i++)
        {
            v_upper[i] = 0;
            v_lower[i] = 0;
        }

        if (param.loadParamFile("firm_constraint.yaml") &&
            param.setParam("firm_constraint/upper", v_upper) &&
            param.setParam("firm_constraint/lower", v_lower) &&
            param.dumpParamFile("firm_constraint.yaml"))
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


ErrorCode ArmGroup::setHardConstraint(const JointConstraint &hard_constraint)
{
    FST_INFO("Set hard constraint.");

    FST_INFO("Origin hard constraint:");
    FST_INFO("  lower = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             hard_constraint_.lower()[0], hard_constraint_.lower()[1], hard_constraint_.lower()[2],
             hard_constraint_.lower()[3], hard_constraint_.lower()[4], hard_constraint_.lower()[5]);
    FST_INFO("  upper = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             hard_constraint_.upper()[0], hard_constraint_.upper()[1], hard_constraint_.upper()[2],
             hard_constraint_.upper()[3], hard_constraint_.upper()[4], hard_constraint_.upper()[5]);

    FST_INFO("Given hard constraint:");
    FST_INFO("  lower = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             hard_constraint.lower[0], hard_constraint.lower[1], hard_constraint.lower[2],
             hard_constraint.lower[3], hard_constraint.lower[4], hard_constraint.lower[5]);
    FST_INFO("  upper = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             hard_constraint.upper[0], hard_constraint.upper[1], hard_constraint.upper[2],
             hard_constraint.upper[3], hard_constraint.upper[4], hard_constraint.upper[5]);

    ParamGroup param;
    vector<double> v_upper(&hard_constraint.upper[0], &hard_constraint.upper[0] + NUM_OF_JOINT);
    vector<double> v_lower(&hard_constraint.lower[0], &hard_constraint.lower[0] + NUM_OF_JOINT);

    for (size_t i = JOINT_OF_ARM; i < NUM_OF_JOINT; i++)
    {
        v_upper[i] = 0;
        v_lower[i] = 0;
    }

    if (param.loadParamFile("hard_constraint.yaml") &&
        param.setParam("hard_constraint/upper", v_upper) &&
        param.setParam("hard_constraint/lower", v_lower) &&
        param.dumpParamFile("hard_constraint.yaml"))
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

ErrorCode ArmGroup::manualMoveToPoint(const Joint &joint)
{
    FST_INFO("Manual to pose, frame = %d by target: %.6f %.6f %.6f %.6f %.6f %.6f",
             manual_traj_.frame, joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);

    if (group_state_ != STANDBY)
    {
        FST_ERROR("Cannot manual to target in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    getLatestJoint(manual_traj_.joint_start);

    if (soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        FST_ERROR("start-joint = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                  manual_traj_.joint_start[0], manual_traj_.joint_start[1], manual_traj_.joint_start[2],
                  manual_traj_.joint_start[3], manual_traj_.joint_start[4], manual_traj_.joint_start[5]);
        FST_ERROR("start-joint is out of soft constraint, manual-mode-apoint is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (!soft_constraint_.isJointInConstraint(joint))
    {
        FST_ERROR("target-joint out of constraint: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                  joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6);
        return JOINT_OUT_OF_CONSTRAINT;
    }

    manual_time_ = 0;
    manual_traj_.mode = APOINT;
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

ErrorCode ArmGroup::manualMoveStep(const ManualDirection *direction)
{
    FST_INFO("Manual step frame=%d by direction.", manual_traj_.frame);

    if (group_state_ != STANDBY)
    {
        FST_ERROR("Cannot manual step in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    getLatestJoint(manual_traj_.joint_start);

    FST_INFO("start joint: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
            manual_traj_.joint_start[0], manual_traj_.joint_start[1], manual_traj_.joint_start[2],
            manual_traj_.joint_start[3], manual_traj_.joint_start[4], manual_traj_.joint_start[5]);

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        if (manual_traj_.frame != JOINT)
        {
            FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
            return INVALID_SEQUENCE;
        }

        for (size_t i = 0; i < JOINT_OF_ARM; i++)
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

    manual_time_ = 0;
    manual_traj_.mode = STEP;
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

ErrorCode ArmGroup::manualMoveContinuous(const ManualDirection *direction)
{
    FST_INFO("Manual continuous frame=%d by direction.", manual_traj_.frame);

    if (group_state_ != STANDBY && group_state_ != MANUAL)
    {
        FST_ERROR("Cannot manual continuous in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    if (group_state_ == STANDBY)
    {
        getLatestJoint(manual_traj_.joint_start);
        FST_INFO("start joint: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 manual_traj_.joint_start[0], manual_traj_.joint_start[1], manual_traj_.joint_start[2],
                 manual_traj_.joint_start[3], manual_traj_.joint_start[4], manual_traj_.joint_start[5]);

        if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
        {
            if (manual_traj_.frame != JOINT)
            {
                FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
                return INVALID_SEQUENCE;
            }

            for (size_t i = 0; i < JOINT_OF_ARM; i++)
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

        manual_time_ = 0;
        manual_traj_.mode = CONTINUOUS;
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
        for (size_t i = 0; i < JOINT_OF_ARM; i++)
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


size_t ArmGroup::getFIFOLength(void)
{
    if (group_state_ == MANUAL)
    {
        return ceil((manual_traj_.duration - manual_time_) * 1000);
    }
    else if (group_state_ == AUTO)
    {
        return 0;
    }
    else
    {
        return 0;
    }
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




