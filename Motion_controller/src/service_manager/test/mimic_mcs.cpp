/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       system_client.cpp
Author:     Feng.Wu 
Create:     31-July-2017
Modify:     31-July-2017
Summary:    test process
**********************************************/
#ifndef SERVICE_MANAGER_MIMIC_MCS_CPP_
#define SERVICE_MANAGER_MIMIC_MCS_CPP_
   
#include "service_manager/service_manager.h"
#include "comm_interface/comm_interface.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "error_code.h"

int main(int argc, char** argv)
{    
    /* initialize the communicatoin channel. */
    fst_comm_interface::CommInterface comm_mcs;
    ErrorCode result = comm_mcs.createChannel(COMM_REQ, COMM_IPC, "mcs");
    if (result == CREATE_CHANNEL_FAIL)
    {
        std::cout<<"Error when creating channel."<<std::endl;
        return 0;
    }

    ServiceRequest request = {MONITOR_HEARTBEAT_SID, ""};
    while (true)
    {
        usleep(50*1000);
        ErrorCode result = comm_mcs.send(&request, sizeof(request), COMM_DONTWAIT);
        if (result == SUCCESS)
        {
//               std::cout<<"send heartbeat req ok."<<std::endl;
        }

        ServiceResponse resp;
        ErrorCode ret = comm_mcs.recv(&resp, sizeof(resp), COMM_WAIT);
        if (ret == SUCCESS)
        {
            size_t size;
            memcpy(&size, &resp.res_buff[0], sizeof(size));
//          printf("recv heartbeat:id = %d, %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n", resp.res_id, (unsigned char)resp.res_buff[7+8],(unsigned char)resp.res_buff[6+8],(unsigned char)resp.res_buff[5+8],(unsigned char)resp.res_buff[4+8],(unsigned char)resp.res_buff[3+8],(unsigned char)resp.res_buff[2+8],(unsigned char)resp.res_buff[1+8],(unsigned char)resp.res_buff[0+8]);
        }
    }
 
    std::cout<<"exit..."<<std::endl;
}


#endif
