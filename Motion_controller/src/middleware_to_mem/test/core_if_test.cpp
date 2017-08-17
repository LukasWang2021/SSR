/**********************************************
File: test.c
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: test the service functions
Author: Feng.Wu 10-oct-2016
Modifier:
**********************************************/

#ifndef TEST_C_
#define TEST_C_

#include <iostream>
#include <string.h>
#include "comm_interface/core_interface.h"
#include "struct_to_mem/struct_joint_command.h"
#include "struct_to_mem/struct_feedback_joint_states.h"
#include "error_code/error_code.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <netinet/in.h>  /* For htonl and ntohl */
#include <sys/mman.h>

int main(int argc, char *argv[]) {

    int i, j, k;
    fst_core_interface::CoreInterface comm;
    ERROR_CODE_TYPE init = comm.init();
    std::cout<<"init "<<init<<std::endl;

    for (k = 1; k < 5; ++k)
    {

        //Writing Joint Command
        JointCommand jc;
        for(i = 0; i < TS_POINT_NUM; i++)
        {   
            for(j = 0; j < JOINT_NUM; j++)
            {
                jc.points[i].positions[j] = k * 0.01;
            }
        }
        jc.points[0].point_position = START_POINT;
        jc.points[9].point_position = END_POINT;

        jc.total_points = TS_POINT_NUM;
    
        ERROR_CODE_TYPE result = comm.sendBareCore(jc);
        if (result != 0)
        {
            printf("====Fail to send====\n");
        } else if(result == 0)
        {
            printf("==suceed to send==\n");
        }
        usleep(100000);
        FeedbackJointState fbjs;
        ERROR_CODE_TYPE rc = comm.recvBareCore(fbjs);
        if (rc == 0)
        {
            for(i=0;i<JOINT_NUM; i++)
            {
                printf("fbjs.position[%d] = %f \n", i, fbjs.position[i]);
            }
        } else 
        {
            std::cout<<"fbjs result = "<<rc<<std::endl;
        }
        printf("\n");

    }
    usleep(1000);
    return 0;
}

#endif

