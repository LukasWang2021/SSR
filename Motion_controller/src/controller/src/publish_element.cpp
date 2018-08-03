#include "controller_publish.h"

using namespace fst_ctrl;


void* ControllerPublish::getUserOpModePtr()
{
    return (void*)state_machine_ptr_->getUserOpModePtr();
}

void* ControllerPublish::getRunningStatusPtr()
{
    return (void*)state_machine_ptr_->getRunningStatusPtr();
}

void* ControllerPublish::getInterpreterStatusPtr()
{
    return (void*)state_machine_ptr_->getInterpreterStatusPtr();
}

void* ControllerPublish::getRobotStatusPtr()
{
    return (void*)state_machine_ptr_->getRobotStatusPtr();
}

void* ControllerPublish::getCtrlStatusPtr()
{
    return (void*)state_machine_ptr_->getCtrlStatusPtr();
}

void* ControllerPublish::getServoStatusPtr()
{
    return (void*)state_machine_ptr_->getServoStatusPtr();
}

void* ControllerPublish::getSafetyAlarmPtr()
{
    return (void*)state_machine_ptr_->getSafetyAlarmPtr();
}


