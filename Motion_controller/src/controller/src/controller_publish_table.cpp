#include "controller_publish.h"


using namespace user_space;

void ControllerPublish::initPublishTable()
{
    PublishService publish_service;

    publish_service = {"/publish/axes_feedback", 0x0001715B, &ControllerPublish::getAxisFdbPtr, &ControllerPublish::updateAxisFdb}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/servo1001/servos_feedback", 0x0001128B, &ControllerPublish::getServo1001ServoFdbPtr, &ControllerPublish::updateServo1001ServoFdb}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/servo1001/cpu_feedback", 0x00012FFB, &ControllerPublish::getServo1001CpuFdbPtr, &ControllerPublish::updateServo1001CpuFdb}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/io1000/io_feedback", 0x00013C8B, &ControllerPublish::getIODigitalFdbPtr, &ControllerPublish::updateIODigitalFdb}; publish_table_.push_back(publish_service);
    publish_service = {"/publish/iosafety/safety_feedback",	0x0001472B, &ControllerPublish::getIOSafetyFdbPtr, &ControllerPublish::updateIOSafetyFdb}; publish_table_.push_back(publish_service); 
	publish_service = {"/publish/torque_feedback", 0x0000AEAB, &ControllerPublish::getTorqueFdbPtr, &ControllerPublish::updateTorqueFdb}; publish_table_.push_back(publish_service);
}

