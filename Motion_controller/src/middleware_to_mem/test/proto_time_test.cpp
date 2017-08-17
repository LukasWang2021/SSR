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
#include <string>
#include "comm_interface/comm_interface.h"
#include "proto/comm.pb.h"
#include "nanomsg/nn.h"
#include "nanomsg/reqrep.h"

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
    struct timespec start, stop;
    long int delta, i, n = 100;
    commsg::ProcessResponse rep;
    rep.set_id(10);
    rep.set_affix_info(true);
    rep.set_info("This is heartbeat response from monitor.");
    std::string str;

    fst_comm_interface::CommInterface comm;
    int fd = comm.createChannel(REQ, "client");
    if (fd == -1)
    {
        printf("Error when client createChannel.\n");
        return -1;
    }
    for (int i = 0; i < 10; ++i)
    {
        if (clock_gettime(CLOCK_MONOTONIC, &start) == -1) {
            perror("clock_gettime");
            return 1;
        }
        rep.SerializeToString(&str);       
        if (clock_gettime(CLOCK_MONOTONIC, &stop) == -1) {
            perror("clock_gettime");
            return 1;
        }
//        printf("serialize rep sizeof is %d\n", str.length());
        delta =(stop.tv_sec - start.tv_sec) * 1000000 + (stop.tv_nsec - start.tv_nsec)/1000;

        printf("serial: %li\n", delta);

        if (clock_gettime(CLOCK_MONOTONIC, &start) == -1) {
            perror("clock_gettime");
            return 1;
        }
        rep.ParseFromString(str);
        if (clock_gettime(CLOCK_MONOTONIC, &stop) == -1) {
            perror("clock_gettime");
            return 1;
        }
        delta =(stop.tv_sec - start.tv_sec) * 1000000 + (stop.tv_nsec - start.tv_nsec)/1000;

        printf("parse: %li\n", delta);


        char *send_buff = const_cast<char*>(str.c_str());
        int send = comm.send(fd, send_buff, strlen(send_buff));
        if (send == -1)
        {
            printf("Error when client send.\n");
            return -1;
        }
        usleep(1000);
    }

  return 0;
}

#endif

