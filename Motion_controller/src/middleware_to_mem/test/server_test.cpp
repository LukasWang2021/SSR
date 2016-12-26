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

    sleep(1);
    //create communication channel.
    fst_comm_interface::CommInterface comm;

    ERROR_CODE_TYPE err = comm.createChannel(IPC_REP, "hello5");
    if (err == CREATE_CHANNEL_FAIL)
    {
        printf("Error when server createChannel.\n");
        return -1;
    }

    while(true)
    {
             
        usleep(10000);
        ServiceRequest req = {0, ""};
        int rc = comm.recv(&req, sizeof(req), IPC_WAIT);
        if (rc == RECV_MSG_FAIL)
        {
            printf("server not recv.\n");
        }

        ServiceResponse resp = {0, ""};
        ERROR_CODE_TYPE send = comm.send(&resp, sizeof(resp), IPC_DONTWAIT);
        if (send == SEND_MSG_FAIL)
        {
            std::cout<<"server not send."<<std::endl;

        }
            
    }
    sleep(1);
    comm.closeChannel();

    printf("Server: done\n");
    return 0;
}

#endif

