#include "controller_publish.h"


using namespace fst_ctrl;

void ControllerPublish::initPublishTable()
{
    PublishService publish_service;
    publish_service = {"/publish/controller/UserOpMode", 0x00015255, &ControllerPublish::getUserOpModePtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/RunningStatus", 0x00001F33, &ControllerPublish::getRunningStatePtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/InterpreterStatus", 0x00003203, &ControllerPublish::getInterpreterStatePtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/RobotStatus", 0x00012943, &ControllerPublish::getRobotStatePtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/CtrlStatus", 0x0000E8E3, &ControllerPublish::getCtrlStatePtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/ServoStatus", 0x00002053, &ControllerPublish::getServoStatePtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/SafetyAlarm", 0x0000D0AD, &ControllerPublish::getSafetyAlarmPtr}; publish_table_.push_back(publish_service);   
    publish_service = {"/publish/motion_control/axis_group/feedback/joints", 0x00013643, &ControllerPublish::getAxisGroupJointFeedbackPtr}; publish_table_.push_back(publish_service); 
}

