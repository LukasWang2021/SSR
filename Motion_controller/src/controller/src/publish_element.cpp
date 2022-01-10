#include "controller_publish.h"

using namespace user_space;


void* ControllerPublish::getAxisFdbPtr()
{
    return (void*)&axis_fdb_;
}

void* ControllerPublish::getServo1001ServoFdbPtr()
{
    return (void*)&servo1001_servo_fdb_;
}

void* ControllerPublish::getServo1001CpuFdbPtr()
{
    return (void*)&servo1001_cpu_fdb_;
}

void* ControllerPublish::getIODigitalFdbPtr()
{
    return (void*)&io_digital_fdb_;
}

void* ControllerPublish::getIOSafetyFdbPtr()
{
    return (void*)&io_safety_fdb_;
}
