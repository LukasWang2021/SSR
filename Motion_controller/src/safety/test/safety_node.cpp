/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       safety_fake.cpp
Author:     Feng.Wu 
Create:     21-Sep-2017
Modify:     21-Sep-2017
Summary:    test process
**********************************************/
#ifndef SAFETY_SAFETY_NODE_CPP_
#define SAFETY_SAFETY_NODE_CPP_

#include "safety/safety.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

int main(int argc, char** argv)
{  
    if (argc != 2)
    {
        std::cout<<"There should be 1 argument."<<std::endl;
        return 0;
    }
    int cmd;
    if (strcmp(argv[1], "BIT_0") == 0)
        cmd = 0x01;
    else if (strcmp(argv[1], "BIT_1") == 0)
        cmd = 0x02;
    else if (strcmp(argv[1], "BIT_2") == 0)
        cmd = 0x04;
    else if (strcmp(argv[1], "BIT_3") == 0)
        cmd = 0x08;
    else if (strcmp(argv[1], "BIT_4") == 0)
        cmd = 0x10;
    else if (strcmp(argv[1], "BIT_5") == 0)
        cmd = 0x20;
    else if (strcmp(argv[1], "RESET") == 0)
        cmd = 0;

    openSafety();

    unsigned long long int ret = 0;

    // --------------------set value.-----------------------//
    ret = setSafety(cmd, SAFETY_OUTPUT_SECONDFRAME);
    if (ret != 0)
        printf("set safety error.\n");

    // --------------------get value.-----------------------//
    while(true)
//    for (int i = 0; i < 10; i++)
    {
        ret = autorunSafetyData();
        if (ret == 0)
        {
            int data = getSafety(SAFETY_INPUT_FIRSTFRAME, &ret);
            //if (data != 0)
                printf("recv data[1] = %x, error = %llx\n", data, ret);

            data = getSafety(SAFETY_INPUT_SECONDFRAME, &ret);
            //if (data != 0)
                printf("recv data[2] = %x, error = %llx\n", data, ret);
        }
        else
        {
            printf("|---------------autorun error = %llx---------------------|\n", ret);
            break;
        }
        usleep(5*1000);
    }

    printf("end.\n");
    closeSafety();
    return 0;
}


#endif 
