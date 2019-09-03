/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       backup_test.cpp
Author:     Feng.Wu 
Create:     1-Mar-2019
Modify:     1-Mar-2017
Summary:    test process
**********************************************/
#ifndef SYSTEM_MANAGER_ONE_COMMAND_CPP_
#define SYSTEM_MANAGER_ONE_COMMAND_CPP_
   
#include "system_manager.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

int main(int argc, char** argv)
{            
    std::cout<<"argc = "<<argc<<std::endl;
    std::cout<<"argv[0]="<<argv[0]<<std::endl;
    std::cout<<"argv[1]="<<argv[1]<<std::endl;

    if (argc != 2)
    {
        std::cout<<"there should be 1 argument."<<std::endl;
        return 0;
    }

    /* construct the execute command request. */
    fst_ctrl::SystemManager sys;
    sys.init();
    if (strcmp(argv[1], "1") == 0)
    {
        sys.controllerBackup();
    }
    else if (strcmp(argv[1], "2") == 0)
    {
        sys.controllerRestore();
    }

    

    std::cout<<"exit..."<<std::endl;
}


#endif
