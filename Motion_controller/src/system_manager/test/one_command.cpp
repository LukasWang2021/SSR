/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       system_client.cpp
Author:     Feng.Wu 
Create:     31-July-2017
Modify:     31-July-2017
Summary:    test process
**********************************************/
#ifndef SYSTEM_MANAGER_ONE_COMMAND_CPP_
#define SYSTEM_MANAGER_ONE_COMMAND_CPP_
   
#include "system_manager/system_manager.h"
#include "comm_interface/comm_interface.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

int main(int argc, char** argv)
{    
    /* initialize the communicatoin channel. */
    fst_comm_interface::CommInterface comm_system_;
    U64 result = comm_system_.createChannel(COMM_REQ, COMM_IPC, "system");
    if (result != FST_SUCCESS)
    {
        std::cout<<"Error when creating channel."<<std::endl;
        return 0;
    }
        
    std::cout<<"argc = "<<argc<<std::endl;
    std::cout<<"argv[0]="<<argv[0]<<std::endl;
    std::cout<<"argv[1]="<<argv[1]<<std::endl;

    if (argc != 2)
    {
        std::cout<<"there should be 1 argument."<<std::endl;
        return 0;
    }

    /* construct the execute command request. */
    ServiceRequest request;
    if (strcmp(argv[1], "FTP_ON") == 0)
        request.req_id = 0xB2;
    else if (strcmp(argv[1], "FTP_OFF") == 0)
        request.req_id = 0xB3;
    else if (strcmp(argv[1], "CONFIG_BACKUP") == 0)
        request.req_id = 0xB4;
    else if (strcmp(argv[1], "RESTORE") == 0)
        request.req_id = 0xB5;
    else if (strcmp(argv[1], "ALL_UPGRADE") == 0)
        request.req_id = 0xB6;
    else if (strcmp(argv[1], "ALL_BACKUP") == 0)
        request.req_id = 0xB7;

    /* send the execute command. */
    result = comm_system_.send(&request, sizeof(request), COMM_DONTWAIT);
    if (result != FST_SUCCESS)
        return 0;
    std::cout<<"send the command request."<<std::endl;
       
    /* receive the execute command response.*/
    ServiceResponse response;
    do
    {
        usleep(100*1000);
        result = comm_system_.recv(&response, sizeof(response), COMM_DONTWAIT);
    } while (result != FST_SUCCESS);
    std::cout<<"got the command reply."<<std::endl;

    /* construct the check-status request.*/
    request.req_id = 0xB1;
    while (true)
    {
        usleep(100*1000);
        /* send the check-status request. */
        result = comm_system_.send(&request, sizeof(request), COMM_DONTWAIT);
        if (result != FST_SUCCESS)
            return 0;

        /* receive the check-status response. */
        do
        {
            usleep(100*1000);
            result = comm_system_.recv(&response, sizeof(response), COMM_DONTWAIT);
        } while (result != FST_SUCCESS);
        /* parse the response value. */
        if (result == FST_SUCCESS) 
        {
            long long int error;
            memcpy(&error, &(response.res_buff[8]), sizeof(error));
            if (error == FST_SUCCESS)
                break;
            if (error != SYS_OPS_UNFINISHED)
                break;
        }
    }
    std::cout<<"exit..."<<std::endl;
}


#endif
