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

    int n = 5;
    int max_size = 1024;
 
        //create communication channel.
        fst_comm_interface::CommInterface comm;
//        comm.init(); //for Junlong shared memory.
        ERROR_CODE_TYPE fd = comm.createChannel(IPC_REQ, "mcs");      
        if (fd == CREATE_CHANNEL_FAIL)
        {
            printf("Error when client createChannel.\n");
            return -1;
        }
        //initialize the receive buff.
        char rec_buff[max_size];
        memset(rec_buff, 0, sizeof(rec_buff));
        for (int i = 0; i < n; ++i)
        {
            sleep(1);
            //set the request data

            ServiceRequest req = {0x31, "hello this client request."};
            ERROR_CODE_TYPE send = comm.send(&req, sizeof(req), IPC_DONTWAIT);
            if (send == SEND_MSG_FAIL)
            {
                printf("Error when client send.\n");
                return -1;
            } else
            {
                printf("ok\n");
            }

            usleep(100000);
            ServiceResponse resp = {0, ""};
            int rc = comm.recv(&resp, sizeof(resp), IPC_DONTWAIT);
            if (rc == 0)
            {
                printf("server recv.\n");
            
//            std::cout<<"sizeof resp = "<<sizeof(resp)<<std::endl;
            std::cout<<"resp.req_id = 0x"<<std::hex<<resp.res_id<<std::dec<<". resp.res_buf = "<<resp.res_buff<<std::endl;
            } else
            {
                std::cout<<"client not recv"<<std::endl;
            }



          
        }
        usleep(10000);
        sleep(1);
        printf("Client: done\n");
    return 0;
}

#endif

