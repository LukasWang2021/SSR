#include "controller_publish.h"


using namespace fst_ctrl;

void ControllerPublish::initPublishTable()
{
    PublishService publish_service;
    publish_service = {"/publish/controller/UserOpMode", 0x00015255, &ControllerPublish::getUserOpModePtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/RunningStatus", 0x00001F33, &ControllerPublish::getRunningStatusPtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/InterpreterStatus", 0x00003203, &ControllerPublish::getInterpreterStatusPtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/RobotStatus", 0x00012943, &ControllerPublish::getRobotStatusPtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/CtrlStatus", 0x0000E8E3, &ControllerPublish::getCtrlStatusPtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/ServoStatus", 0x00002053, &ControllerPublish::getServoStatusPtr}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/controller/SafetyAlarm", 0x0000D0AD, &ControllerPublish::getSafetyAlarmPtr}; publish_table_.push_back(publish_service);   
}

