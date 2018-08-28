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
    return (void*)&tcp_world_cartesian_;
}

void* ControllerPublish::getAxisGroupTcpBaseCartesianPtr()
{
    return (void*)&tcp_base_cartesian_;
}

void* ControllerPublish::getAxisGroupTcpCurrentCartesianPtr()
{
    return (void*)&tcp_current_cartesian_;
}

void* ControllerPublish::getAxisGroupCurrentCoordinatePtr()
{
    return (void*)&current_coordinate_;
}

void* ControllerPublish::getAxisGroupCurrentToolPtr()
{
    return (void*)&current_tool_;
}

void* ControllerPublish::getGlobalVelRatioPtr()
{
    return (void*)&global_vel_ratio_;
}

void* ControllerPublish::getGlobalAccRatioPtr()
{
    return (void*)&global_acc_ratio_;
}

void* ControllerPublish::getProgramStatusPtr()
{
    return (void*)&program_status_;
}


