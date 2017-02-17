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
#include "error_code/error_code.h"
#include "comm_interface/comm_interface.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <sys/time.h>
#include <netinet/in.h>  /* For htonl and ntohl */
#include <sys/mman.h>
int main(int argc, char *argv[]) {

    int n = 10;
std::cout<<"hello0"<<std::endl;

    if (!fork()) { /* child */
        //create communication channel.
        fst_comm_interface::CommInterface comm;
        ERROR_CODE_TYPE fd = comm.createChannel(COMM_REQ, COMM_IPC, "testIPC");      
        if (fd == CREATE_CHANNEL_FAIL)
        {
            printf("Error when client createChannel.\n");
            return -1;
        }
        //set the request data

        ServiceRequest req = {0x31, "hello this client request."};
        ServiceResponse resp;
        usleep(100000);
        for (int i = 0; i < n; ++i)
        {
            ERROR_CODE_TYPE send = comm.send(&req, sizeof(req), COMM_DONTWAIT);
            if (send == SEND_MSG_FAIL)
            {
                printf("Error when client send.\n");
                return -1;
            }
            usleep(1000000);
            //receiving in BLOCK mode using string
            int rc = comm.recv(&resp, sizeof(resp), COMM_WAIT);
            if (rc == -1)
            {
                printf("Error when client recv.\n");
            }
            
            std::cout<<i<<" Client: receive rep.id = "<<resp.res_id<<". rep_info = "<<resp.res_buff<<std::endl;

        }
        sleep(1);
        printf("Client: done\n");
        return 0;
          
    } else { /* parent */
        //create communication channel.
        fst_comm_interface::CommInterface comm;
        ERROR_CODE_TYPE fd = comm.createChannel(COMM_REP,  COMM_IPC, "testIPC");
        if (fd == CREATE_CHANNEL_FAIL)
        {
            printf("Error when server createChannel.\n");
            return -1;
        }
        ServiceRequest req = {0, ""};
        ServiceResponse resp = {0x31, "This is response"};

        usleep(100000);
        for (int i = 0; i < n; ++i)
        {
            
            int rc = comm.recv(&req, sizeof(req), COMM_WAIT);
            if (rc == RECV_MSG_FAIL)
            {
                printf("Error when server recv.\n");
            }
//            std::cout<<"sizeof req = "<<sizeof(req)<<std::endl;
//            std::cout<<"req.req_id = 0x"<<std::hex<<req.req_id<<std::dec<<". req.req_buf = "<<req.req_buff<<std::endl;
       //send
            int send = comm.send(&resp, sizeof(resp), COMM_DONTWAIT);
            if (send == -1)
            {
                printf("Error when client send.\n");
                return -1;
            }
        }
    }
    sleep(1);
    printf("Server: done\n");
    return 0;
}

#endif

