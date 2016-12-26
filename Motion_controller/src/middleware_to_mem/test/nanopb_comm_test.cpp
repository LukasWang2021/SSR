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
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

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
    long int delta, i, n = 1;
    int max_size = 1024;
    if (!fork()) { /* child */
        //create communication channel.
        fst_comm_interface::CommInterface comm;
//        comm.init(); //for Junlong shared memory.
        int fd = comm.createChannel(IPC_REQ, "client");      
        if (fd == -1)
        {
            printf("Error when client createChannel.\n");
            return -1;
        }
        uint8_t buffer[128];
        //memset(buffer, 0, 128);

        //initialize the structure
        ProcessResponse rep = ProcessResponse_init_zero;
        //create a stream that will write to our buffer
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        //fill in data
        rep.id = 11;
        rep.affix_info = true;
        strcpy(rep.info, "This is heartbeat response from monitor.");
/*        for (int i = 0; i < n; ++i)
        {
            if (clock_gettime(CLOCK_MONOTONIC, &start) == -1) {
            perror("clock_gettime");
            return 1;
            }
*/            //Serialize the msg
        std::cout<<"req = "<<rep.id<<".  req.affix_info = "<<rep.affix_info<<". info= "<<rep.info<<std::endl;
            bool status = pb_encode(&stream, ProcessResponse_fields, &rep);
            size_t message_length = stream.bytes_written;
            std::cout<<"status = "<<status<<". length = "<<message_length<<std::endl;
            printf("message length = %zu\n",message_length);
            std::cout<<"buffer="<<buffer<<std::endl;
/*            if (clock_gettime(CLOCK_MONOTONIC, &stop) == -1) {
            perror("clock_gettime");
            return 1;
            }
            delta =(stop.tv_sec - start.tv_sec) * 1000000 + (stop.tv_nsec - start.tv_nsec)/1000;

//            printf("serial: %li\n", delta);
*/
            int send = comm.send(fd, buffer, sizeof(buffer));
            if (send == -1)
            {
                printf("Error when client send.\n");
                return -1;
            }
          
            usleep(10000);
//        }
//      comm.close(fd);
        sleep(1);
        printf("Client: done\n");
        return 0;
          
    } else { /* parent */
        //create communication channel.
        fst_comm_interface::CommInterface comm;
//      comm.init(); //for Junlong shared memory.
        int fd = comm.createChannel(IPC_REP, "client");
        if (fd == -1)
        {
            printf("Error when client createChannel.\n");
            return -1;
        }
        uint8_t buffer[128];

        //initialize the structure
        ProcessResponse rep = ProcessResponse_init_zero;
        //create a stream that will write to our buffer
        pb_istream_t stream = pb_istream_from_buffer(buffer, sizeof(buffer));

        for (int i = 0; i < n; ++i)
        {
            usleep(10000);
            int rc = comm.recv(fd, buffer, sizeof(buffer), IPC_BLOCK);


            if (clock_gettime(CLOCK_MONOTONIC, &start) == -1) {
            perror("clock_gettime");
            return 1;
            }
            //Serialize the msg
            bool status = pb_decode(&stream, ProcessResponse_fields, &rep);
            if (clock_gettime(CLOCK_MONOTONIC, &stop) == -1) {
            perror("clock_gettime");
            return 1;
            }
            delta =(stop.tv_sec - start.tv_sec) * 1000000 + (stop.tv_nsec - start.tv_nsec)/1000;

//            printf("parse: %li\n", delta);

//            std::cout<<"sizeof req = "<<sizeof(buffer)<<std::endl;
            std::cout<<"rep.id = "<<rep.id<<"rep.affix_info = "<<rep.affix_info<<"rep.info = "<<rep.info<<std::endl;
        }
//      comm.close (fd);
    }
    sleep(1);
    printf("Server: done\n");
    return 0;
}







#endif

