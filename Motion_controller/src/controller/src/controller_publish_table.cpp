#include "controller_publish.h"


using namespace fst_ctrl;

void ControllerPublish::initPublishTable()
{
    PublishService publish_service;
    publish_service = {"/publish/controller/UserOpMode", 0x00015255, &ControllerPublish::getUserOpModePtr, NULL}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/RunningStatus", 0x00001F33, &ControllerPublish::getRunningStatePtr, NULL}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/InterpreterStatus", 0x00003203, &ControllerPublish::getInterpreterStatePtr, NULL}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/RobotStatus", 0x00012943, &ControllerPublish::getRobotStatePtr, NULL}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/CtrlStatus", 0x0000E8E3, &ControllerPublish::getCtrlStatePtr, NULL}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/ServoStatus", 0x00002053, &ControllerPublish::getServoStatePtr, NULL}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/SafetyAlarm", 0x0000D0AD, &ControllerPublish::getSafetyAlarmPtr, NULL}; publish_table_.push_back(publish_service);   
    publish_service = {"/publish/motion_control/axis_group/feedback/joints", 0x000161F3, &ControllerPublish::getAxisGroupJointFeedbackPtr, &ControllerPublish::updateAxisGroupJointFeedback}; publish_table_.push_back(publish_service); 
    publish_service = {"/publish/motion_control/axis_group/feedback/tcp_world_cartesian", 0x00009D8E, &ControllerPublish::getAxisGroupTcpWorldCartesianPtr, &ControllerPublish::updateAxisGroupTcpWorldCartesian}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/motion_control/axis_group/feedback/tcp_base_cartesian", 0x00002D5E, &ControllerPublish::getAxisGroupTcpBaseCartesianPtr, &ControllerPublish::updateAxisGroupTcpBaseCartesian}; publish_table_.push_back(publish_service); 
    publish_service = {"/publish/motion_control/axis_group/feedback/tcp_current_cartesian", 0x0000352E, &ControllerPublish::getAxisGroupTcpCurrentCartesianPtr, &ControllerPublish::updateAxisGroupTcpCurrentCartesian}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/motion_control/axis_group/feedback/current_coordinate", 0x00012C55, &ControllerPublish::getAxisGroupCurrentCoordinatePtr, &ControllerPublish::updateAxisGroupCurrentCoordinate}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/motion_control/axis_group/feedback/current_tool", 0x00004BEC, &ControllerPublish::getAxisGroupCurrentToolPtr, &ControllerPublish::updateAxisGroupCurrentTool}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/motion_control/global_vel_ratio", 0x00012A4F, &ControllerPublish::getGlobalVelRatioPtr, &ControllerPublish::updateGlobalVelRatio}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/motion_control/global_acc_ratio", 0x0001517F, &ControllerPublish::getGlobalAccRatioPtr, &ControllerPublish::updateGlobalAccRatio}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/interpreter/program_status", 0x00001AF3, &ControllerPublish::getProgramStatusPtr, &ControllerPublish::updateProgramStatus}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/interpreter/tp_program_status", 0x000042B3, &ControllerPublish::getTpProgramStatusPtr, &ControllerPublish::updateTpProgramStatus}; publish_table_.push_back(publish_service);
}

