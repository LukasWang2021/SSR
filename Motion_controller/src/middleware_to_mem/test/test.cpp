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
#include <vector>
#include <string.h>
#include <unistd.h>

#include "error_code/error_code.h"
#include "comm_monitor/service_wrapper.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"

/*
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <time.h>
#include <sched.h>
#include <sys/time.h>
#include <netinet/in.h>  // For htonl and ntohl
#include <sys/mman.h>
*/
int main(int argc, char *argv[]) {

    fst_service_wrapper::ServiceWrapper comm;
    ERROR_CODE_TYPE result = comm.init();

    if (result != 0)
    {
        std::cout<<"fail when init."<<std::endl;
    }

    ServiceResponse resp;
    int i=0;
    while (i<5)
    {
        sleep(1);
        if (comm.sendResetRequest() != 0)
        {
            std::cout<<"send reset fail"<<std::endl;
            return 1;
        }

        if (comm.sendHeartbeatRequest(resp) != 0)
        {
            std::cout<<"recv heartbeat fail"<<std::endl;
            return 1;
        }

        else
        {
            unsigned int size = *(int*)(&resp.res_buff);
            if (size > 0)
                printf("recv heartbeat:id = %d, %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n", resp.res_id, (unsigned char)resp.res_buff[7+8],(unsigned char)resp.res_buff[6+8],(unsigned char)resp.res_buff[5+8],(unsigned char)resp.res_buff[4+8],(unsigned char)resp.res_buff[3+8],(unsigned char)resp.res_buff[2+8],(unsigned char)resp.res_buff[1+8],(unsigned char)resp.res_buff[0+8]); 
        }

        if (comm.sendStopRequest() != 0)
        {
            std::cout<<"send stop fail"<<std::endl;
            return 1;
        }


        ++i;

    }

    return 0;
}

#endif

