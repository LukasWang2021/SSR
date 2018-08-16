/*************************************************************************
	> File Name: motion_control_base_group.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 11时25分56秒
 ************************************************************************/

#include <unistd.h>
#include <string.h>

#include <motion_control_base_group.h>

using namespace fst_base;

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

void BaseGroup::getServoState(ServoStatus &state)
{
    pthread_mutex_lock(&servo_mutex_);
    state = servo_state_;
    pthread_mutex_unlock(&servo_mutex_);
}

ServoStatus BaseGroup::getServoState(void)
{
    pthread_mutex_lock(&servo_mutex_);
    ServoStatus state(servo_state_);
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


ErrorCode BaseGroup::realtimeTask(void)
{
    ErrorCode  err;
    Joint barecore_joint;
    ServoStatus barecore_state;

    while (true)
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

    return SUCCESS;
}



}

