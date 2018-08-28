#include "controller_publish.h"

using namespace fst_ctrl;
using namespace fst_mc;

void ControllerPublish::updateAxisGroupJointFeedback()
{
    Joint joint_feedback = motion_control_ptr_->getServoJoint();
    joint_feedback_.data1.data = 1;
    joint_feedback_.data2.data_count = 9;
    joint_feedback_.data2.data[0] = joint_feedback.j1;
    joint_feedback_.data2.data[1] = joint_feedback.j2;
    joint_feedback_.data2.data[2] = joint_feedback.j3;
    joint_feedback_.data2.data[3] = joint_feedback.j4;
    joint_feedback_.data2.data[4] = joint_feedback.j5;
    joint_feedback_.data2.data[5] = joint_feedback.j6;
    joint_feedback_.data2.data[6] = joint_feedback.j7;
    joint_feedback_.data2.data[7] = joint_feedback.j8;
    joint_feedback_.data2.data[8] = joint_feedback.j9;
}

void ControllerPublish::updateAxisGroupTcpWorldCartesian()
{

}

void ControllerPublish::updateAxisGroupTcpBaseCartesian()
{

}

void ControllerPublish::updateAxisGroupTcpCurrentCartesian()
{

}

void ControllerPublish::updateAxisGroupCurrentCoordinate()
{

}

void ControllerPublish::updateAxisGroupCurrentTool()
{

}

void ControllerPublish::updateGlobalVelRatio()
{

}

void ControllerPublish::updateGlobalAccRatio()
{

}

void ControllerPublish::updateProgramStatus()
{

}

