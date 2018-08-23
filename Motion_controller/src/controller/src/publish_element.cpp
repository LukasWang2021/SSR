#include "controller_publish.h"

using namespace fst_ctrl;


void* ControllerPublish::getUserOpModePtr()
{
    return (void*)state_machine_ptr_->getUserOpModePtr();
}

void* ControllerPublish::getRunningStatePtr()
{
    return (void*)state_machine_ptr_->getRunningStatePtr();
}

void* ControllerPublish::getInterpreterStatePtr()
{
    return (void*)state_machine_ptr_->getInterpreterStatePtr();
}

void* ControllerPublish::getRobotStatePtr()
{
    return (void*)state_machine_ptr_->getRobotStatePtr();
}

void* ControllerPublish::getCtrlStatePtr()
{
    return (void*)state_machine_ptr_->getCtrlStatePtr();
}

void* ControllerPublish::getServoStatePtr()
{
    return (void*)state_machine_ptr_->getServoStatePtr();
}

void* ControllerPublish::getSafetyAlarmPtr()
{
    return (void*)state_machine_ptr_->getSafetyAlarmPtr();
}

void* ControllerPublish::getAxisGroupJointFeedbackPtr()
{
    return (void*)&joint_feedback_;
}

void* ControllerPublish::getAxisGroupTcpWorldCartesianPtr()
{
    return NULL;
}

void* ControllerPublish::getAxisGroupTcpBaseCartesianPtr()
{
    return NULL;
}

void* ControllerPublish::getAxisGroupTcpCurrentCartesianPtr()
{
    return NULL;
}

void* ControllerPublish::getAxisGroupCurrentCoordinatePtr()
{
    return NULL;
}

void* ControllerPublish::getAxisGroupCurrentToolPtr()
{
    return NULL;
}

void* ControllerPublish::getGlobalVelRatioPtr()
{
    return NULL;
}

void* ControllerPublish::getGlobalAccRatioPtr()
{
    return NULL;
}

void* ControllerPublish::getProgramStatusPtr()
{
    return NULL;
}


