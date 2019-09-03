/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       safety_fake.cpp
Author:     Feng.Wu 
Create:     21-Sep-2017
Modify:     28-Sep-2017
Summary:    test process
**********************************************/
#ifndef SAFETY_SAFETY_NODE_CPP_
#define SAFETY_SAFETY_NODE_CPP_

#include "fst_safety_mem.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

int main(int argc, char** argv)
{  
    int cmd = 0;
    unsigned long long int ret = 0;
    int data = 0;
    int count = 0;
    /*
    if (argc != 2){
        std::cout<<"There should be one argument."<<std::endl;
        return 0;
    }
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
*/

    ret = openSafety();
//    ret = fake_init(); //for fake.
	if(ret != 0){ 
       std::cout<< "open safety return  " << ret << std::endl;
	   return 1;
    }
    else{
       std::cout<< "open safety OK. " << std::endl;
    }

    // simulate core1 to reset.
/*    ret = setSafety(0x00000008, SAFETY_OUTPUT_SECONDFRAME);
        if (ret != 0)
            std::cout<< "reset safety error. " << std::endl;
*/
    while(true)
    {
    // -------------------set value.-----------------------//
        count++;

        if (count >= 20)
            count = 0;

        if (count == 0){
            cmd++;
            if (cmd >= 0x3F)
                cmd = 0;
            printf("\nwrite data = 0x%x.\n", cmd);
        }
        ret = setSafety(cmd, SAFETY_OUTPUT_FIRSTFRAME);
        if (ret != 0)
            std::cout<< "set safety error. " << std::endl;

    // --------------------get value.-----------------------//
        ret = autorunSafetyData();
        if (ret == 0){
            data = getSafety(SAFETY_OUTPUT_FIRSTFRAME, &ret);
            if (count == 0)
                printf("Writing data[0] = 0x%x, error = 0x%llx.\n", data, ret);

            //data = getSafety(SAFETY_OUTPUT_SECONDFRAME, &ret);// simulate core1 reset
            //if (count == 0)
            //    printf("Writing data[1] = 0x%x, error = 0x%llx.\n", data, ret);

            data = getSafety(SAFETY_INPUT_FIRSTFRAME, &ret);
            if (count == 0)
                printf("recv data[0] = 0x%x, error = 0x%llx\n", data, ret);

            data = getSafety(SAFETY_INPUT_SECONDFRAME, &ret);
            if (count == 0)
                printf("recv data[1] = 0x%x, error = 0x%llx\n", data, ret);
        }else{
            printf("|---------------autorun error = %llx---------------------|\n", ret);
            break;
        }
        usleep(50*1000);
    }

    printf("end.\n");
    closeSafety();
    return 0;
}


#endif 
