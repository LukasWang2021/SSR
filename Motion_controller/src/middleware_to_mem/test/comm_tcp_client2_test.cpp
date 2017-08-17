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

    int n = 100;
    struct timespec start, stop;
    int64_t delta;
    int64_t sum=0;


    fst_comm_interface::CommInterface comm;

    ERROR_CODE_TYPE fd = comm.createChannel(COMM_REQ, COMM_TCP, "192.168.3.13:5558");      
    if (fd == CREATE_CHANNEL_FAIL)
    {
        printf("Error when client createChannel.\n");
        return -1;
    }
//    comm.setMaxMsgSize(1024*1024*5);
    for (int i = 0; i < n; ++i)
    {
        //set the request data

        ServiceRequest req = {10, "This is client request"};
        ServiceResponse resp;
        if (clock_gettime(CLOCK_MONOTONIC, &start) == -1)
        {
            perror("clock_gettime");
            return -1;
        }

        ERROR_CODE_TYPE send = comm.send(&req, sizeof(req), COMM_DONTWAIT);
        if (send == SEND_MSG_FAIL)
            std::cout<<"Error when client send."<<std::endl;
//        std::cout<<"start sec="<<sec<<". nsec="<<nsec<<std::endl;
          
        //receiving in BLOCK mode using string
        sleep(2);
        int rc = comm.recv(&resp, sizeof(resp), COMM_WAIT);
        if (rc == RECV_MSG_FAIL)
            printf("Error when client recv.\n");
          
        if (clock_gettime(CLOCK_MONOTONIC, &stop) == -1) {
            perror("clock_gettime");
            return 1;
        }
        delta =(stop.tv_sec - start.tv_sec)*1000000 + (stop.tv_nsec - start.tv_nsec)/1000;
//        if (delta > 10000)
//            std::cout<<"|=====delta time = "<<delta<<"us"<<std::endl;

        std::cout<<i<<" Client: receive rep.id = "<<resp.res_id<<". rep_info = "<<resp.res_buff<<std::endl;


        sum += delta;
    }
    std::cout<<"avg delta time = "<<(sum/n)<<std::endl;
    sleep(1);
    printf("Client: done\n");
    return 0;
          
}

#endif

