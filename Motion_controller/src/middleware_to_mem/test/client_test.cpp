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
  
    sleep(5);
    //create communication channel.
    fst_comm_interface::CommInterface comm;
    ERROR_CODE_TYPE err = comm.createChannel(IPC_REQ, "hello5");      
    if (err == CREATE_CHANNEL_FAIL)
    {
        printf("Error when client createChannel.\n");
        return -1;
    }

    for (int i = 0; i < n; ++i)
    {
        //set the request data
        ServiceRequest req = {0x31, "hello this client request."};
        ERROR_CODE_TYPE send = comm.send(&req, sizeof(req), IPC_DONTWAIT);
        if (send == SEND_MSG_FAIL)
        {
                printf("client not send.\n");
                return -1;
        }
        else if (send == 0)
        {
            std::cout<<"client send ok"<<std::endl;
        }
          
        usleep(10000);
        ServiceResponse resp = {0, ""};
        ERROR_CODE_TYPE rc = comm.recv(&resp, sizeof(resp), IPC_DONTWAIT);
        if (rc == RECV_MSG_FAIL)
        {
            printf("client not recv.\n");
        } 
        else if (rc == 0)
        {
            std::cout<<"client recv ok. resp.res_id = 0x"<<std::hex<<resp.res_id<<std::dec<<". resp.res_buf = "<<resp.res_buff<<std::endl;

        }

    }
    sleep(1);
    comm.closeChannel();

    printf("Client: done\n");
    return 0;
}

#endif

