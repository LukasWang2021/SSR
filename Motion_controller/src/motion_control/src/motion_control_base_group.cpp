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
#include "parameter_manager/parameter_manager_param_group.h"

using namespace std;
using namespace fst_base;
using namespace fst_parameter;

namespace fst_mc
{


BaseGroup::BaseGroup(fst_log::Logger* plog)
{
    group_state_ = STANDBY;
    log_ptr_ = plog;
    auto_cache_ = NULL;
    manual_cache_ = NULL;
    auto_time_ = 0;
    manual_time_ = 0;
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

ErrorCode BaseGroup::sendPoint(void)
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


void BaseGroup::realtimeTask(void)
{
    ErrorCode  err;
    Joint barecore_joint;
    ServoState barecore_state;
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
            (group_state_ == MANUAL || group_state_ == AUTO))
        {
            err = sendPoint();

            if (err != SUCCESS)
            {
                FST_ERROR("Cannot send point to bare core");
                reportError(err);
            }
        }

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



}

