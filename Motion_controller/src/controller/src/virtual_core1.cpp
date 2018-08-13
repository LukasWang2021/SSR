#include "virtual_core1.h"
#include <cstring>
#include <unistd.h>
#include <iostream>


using namespace fst_ctrl;
using namespace fst_mc;
using namespace std;

VirtualCore1::VirtualCore1():
    log_(NULL),
    is_estop(false), is_reset(false),
    servo_status_(1), arm_status_(1)
{
    memset(&joint_cmd_, 0, sizeof(joint_cmd_));
    memset(&joint_feedback_, 0, sizeof(joint_feedback_));
}

VirtualCore1::~VirtualCore1()
{

}

void VirtualCore1::init(fst_log::Logger* log)
{
    log_ = log;
    thread_.run(&virtualCore1ThreadFunc, this, 50);
}

int VirtualCore1::getServoStatus()
{
    return servo_status_;
}

int VirtualCore1::getArmStatus()
{
    return arm_status_;
}

int VirtualCore1::getSafetyAlarm()
{
    return 0;
}

void VirtualCore1::doEstop()
{
    is_estop = true;
}

void VirtualCore1::doReset()
{
    is_reset = false;
}

void VirtualCore1::threadFunc()
{
    if(is_estop)
    {
        is_estop = false;
        if(servo_status_ == 2)
        {
            usleep(500000);
            servo_status_ = 1;
        }
    }
    else if(is_reset)
    {
        is_reset = false;
        usleep(1000000);
        servo_status_ = 1;
        arm_status_ = 1;
    }
    else{}

    usleep(100000);
}

void virtualCore1ThreadFunc(void* arg)
{
    std::cout<<"---virtualCore1ThreadFunc running"<<std::endl;
    VirtualCore1* virtual_core1_ptr = static_cast<VirtualCore1*>(arg);
    while(1)
    {
        virtual_core1_ptr->threadFunc();
    }
}


