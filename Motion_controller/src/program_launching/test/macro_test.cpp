/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       macro_test.cpp
Author:     Feng.Wu 
Create:     03-Dec-2018
Modify:     03-Dec-2018
Summary:    test process
**********************************************/
#ifndef PROGRAM_LAUNCHING_MACRO_TEST_CPP_
#define PROGRAM_LAUNCHING_MACRO_TEST_CPP_

#include "program_launching.h"
#include "io_manager.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
using namespace fst_ctrl;

int main(int argc, char** argv)
{
    fst_hal::DeviceManager device_manager;
    device_manager.init();
    fst_hal::IoManager* io_manager_ptr = fst_hal::IoManager::getInstance(&device_manager);
    io_manager_ptr->init();

    fst_ctrl::IoMapping* map_ptr = new fst_ctrl::IoMapping();
    int ret = map_ptr->init(io_manager_ptr);
    if (ret != 0) 
    {
        printf("failed init io mapping\n");
        return -1;
    }

    fst_base::ProcessComm* process_comm_ptr = fst_base::ProcessComm::getInstance();
    ret = fst_base::ProcessComm::getInitErrorCode();
    if (ret != 0) 
    {
        printf("failed init process comm\n");
        return -1;
    }

    ret = process_comm_ptr->getControllerServerPtr()->init();
    if (ret != 0) 
    {
        printf("failed init process comm get server ptr\n");
        return -1;
    }

    ret = process_comm_ptr->getControllerClientPtr()->init(process_comm_ptr->getControllerServerPtr());
    if (ret != 0) 
    {
        printf("failed init process comm get client ptr\n");
        return -1;
    }

    ProgramLaunching program_launching;
    ret = program_launching.init(map_ptr, process_comm_ptr->getControllerClientPtr());
    if (ret != 0) 
    {
        printf("failed init program launching\n");
        return -1;
    }

    while(true)
    {
        program_launching.processMacro(1);
        usleep(10*1000);
        map_ptr->setDIByBit(1,  1);
    }



    return 0;
}


#endif 
