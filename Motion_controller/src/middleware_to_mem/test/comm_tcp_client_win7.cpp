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

//#include <stdint.h>
#include <stdio.h>
//#include <stdlib.h>
//#include <sys/socket.h>
//#include <time.h>
#include <unistd.h>
//#include <sched.h>
//#include <sys/time.h>
//#include <netinet/in.h>  /* For htonl and ntohl */
//#include <sys/mman.h>

int main(int argc, char *argv[]) {

    int n = 10;
    struct timespec start, stop;
    int64_t delta;
    int64_t sum=0;


    fst_comm_interface::CommInterface comm;

    ERROR_CODE_TYPE fd = comm.createChannel(COMM_REQ, COMM_TCP, "192.168.1.64:5559");      
    if (fd == CREATE_CHANNEL_FAIL)
    {
        printf("Error when client createChannel.\n");
        return -1;
    }
sleep(1);
    int req = 20, resp = 0;
    for (int i = 0; i < n; ++i)
    {
        if (clock_gettime(CLOCK_MONOTONIC, &start) == -1)
        {
            perror("clock_gettime");
            return -1;
        }

        ERROR_CODE_TYPE send = comm.send(&req, sizeof(req), COMM_DONTWAIT);
        if (send == SEND_MSG_FAIL)
            printf("Error when client send.\n");
//        std::cout<<"start sec="<<sec<<". nsec="<<nsec<<std::endl;
          
        int rc = comm.recv(&resp, sizeof(resp), COMM_WAIT);
        if (rc == -1)
            printf("Error when client recv.\n");
          
        if (clock_gettime(CLOCK_MONOTONIC, &stop) == -1) {
            perror("clock_gettime");
            return -1;
        }
        delta =(stop.tv_sec - start.tv_sec)*1000000 + (stop.tv_nsec - start.tv_nsec)/1000;
        std::cout<<"|=====delta time = "<<delta<<"us"<<std::endl;
        std::cout<<" Client: receive = "<<resp<<std::endl;

        sum += delta;
    }
    std::cout<<"avg delta time = "<<(sum/n)<<std::endl;
    sleep(1);
    printf("Client: done\n");
    return 0;
          
}

#endif

