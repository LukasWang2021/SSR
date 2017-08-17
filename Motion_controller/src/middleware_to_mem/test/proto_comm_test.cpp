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
#include "comm_interface/comm_interface.h"
#include "proto/comm.pb.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"
//#include "nanomsg/nn.h"
//#include "nanomsg/reqrep.h"

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
        int fd = comm.createChannel(IPC_REQ, "client");      
        if (fd == -1)
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
/*            commsg::ProcessRequest req;
            req.set_id(10);
            req.set_need_reply(true);
            req.set_affix_info(true);
            req.set_info("This is reset resquest from client.");
            //serialize the request to a string
            std::string str;
            req.SerializeToString(&str);
            //send
            int send = comm.send(fd, str);
*/
            ServiceRequest req = {1100, "hello this client request."};
            int send = comm.send(fd, &req, sizeof(req));
            if (send == -1)
            {
                printf("Error when client send.\n");
                return -1;
            }
          
            usleep(10000);
            //receiving in BLOCK mode using string
/*            std::string str_recv;
            int rc = comm.recv(fd, &str_recv, IPC_BLOCK);
            if (rc == -1)
            {
                printf("Error when client recv.\n");
            }
            //parse the string to response data
            commsg::ProcessResponse rep;
            rep.ParseFromString(str_recv);
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
        int fd = comm.createChannel(IPC_REP, "client");
        if (fd == -1)
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
            ServiceRequest req;
            int rc = comm.recv(fd, &req, sizeof(req));
            std::cout<<"sizeof req = "<<sizeof(req)<<std::endl;
            std::cout<<"req.req_id = "<<req.req_id<<"req.req_buf = "<<req.req_buff<<std::endl;
            //receiving in BLOCK mode using array "rec_buff"
/*            int rc = comm.recv(fd, rec_buff, max_size, IPC_BLOCK);
            if (rc == -1)
            {
                printf("Error when server recv.\n");
            }
            //parse the array to request data
            commsg::ProcessRequest req;
            req.ParseFromArray(rec_buff, 1024);
            std::cout<<i<<" Server: receive req.id = "<<req.id()<<". req.info = "<<req.info()<<std::endl;
            commsg::ProcessResponse rep;
            rep.set_id(10);
            rep.set_affix_info(true);
            rep.set_info("server response.");
            //serialize the request to a string
            std::string str;
            rep.SerializeToString(&str);
//            char *send_buff = const_cast<char*> (str.c_str());
            char *send_buff = new char[str.length() + 1];
            strcpy(send_buff, str.c_str());
            //send
            int send = comm.send(fd, send_buff, strlen(send_buff) + 1);
            if (send == -1)
            {
                printf("Error when client send.\n");
                return -1;
            }
            delete[] send_buff;
*/        }
//      comm.close (fd);
    }
    sleep(1);
    printf("Server: done\n");
    return 0;
}

#endif

