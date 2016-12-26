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

    int n = 2;
    int max_size = 1024;
    if (!fork()) { /* child */
        //create communication channel.
        fst_comm_interface::CommInterface comm;
//        comm.init(); //for Junlong shared memory.
        ERROR_CODE_TYPE fd = comm.createChannel(IPC_REQ, "client");      
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
            //set the request data

            ServiceRequest req = {0x31, "hello this client request."};
            ERROR_CODE_TYPE send = comm.send(&req, sizeof(req), IPC_DONTWAIT);
            if (send == SEND_MSG_FAIL)
            {
                printf("Error when client send.\n");
                return -1;
            }
          
            usleep(10000);
            //receiving in BLOCK mode using string
/*            std::string str_recv;
            int rc = comm.recv(fd, &str_recv, IPC_WAIT);
            if (rc == -1)
            {
                printf("Error when client recv.\n");
            }
            
            std::cout<<i<<" Client: receive rep.id = "<<rep.id()<<". rep_info = "<<rep.info()<<std::endl;
*/
        }
//      comm.close(fd);
        sleep(1);
        printf("Client: done\n");
        return 0;
          
    } else { /* parent */
        //create communication channel.
        fst_comm_interface::CommInterface comm;
//      comm.init(); //for Junlong shared memory.
        ERROR_CODE_TYPE fd = comm.createChannel(IPC_REP, "client");
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
            usleep(10000);
            ServiceRequest req = {0, ""};
            std::cout<<"test req sizefof req = "<<sizeof(req)<<std::endl;
            int rc = comm.recv(&req, sizeof(req), IPC_DONTWAIT);
            if (rc == RECV_MSG_FAIL)
            {
                printf("Error when server recv.\n");
            }
            std::cout<<"sizeof req = "<<sizeof(req)<<std::endl;
            std::cout<<"req.req_id = 0x"<<std::hex<<req.req_id<<std::dec<<". req.req_buf = "<<req.req_buff<<std::endl;
       //send
/*            int send = comm.send(fd, send_buff, strlen(send_buff) + 1);
            if (send == -1)
            {
                printf("Error when client send.\n");
                return -1;
            }
*/        }
//      comm.close (fd);
    }
    sleep(1);
    printf("Server: done\n");
    return 0;
}

#endif

