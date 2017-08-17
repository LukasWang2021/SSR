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

#define SERVER_PORT "5558"
int main(int argc, char *argv[]) {


    //create communication channel.
    fst_comm_interface::CommInterface comm;
    char *ip, ip_addr[32];
    if (!comm.getLocalIP(&ip)) return false;

    sprintf(ip_addr,"%s:%s", ip, SERVER_PORT);
    ERROR_CODE_TYPE fd = comm.createChannel(COMM_REP, COMM_TCP, ip_addr);
    if (fd == CREATE_CHANNEL_FAIL)
    {
        std::cout<<"Error when server createChannel."<<std::endl;
        return -1;
    }
    comm.setMaxMsgSize(1024*1024*5);
    std::cout<<"server bind ok :"<<ip_addr<<std::endl;

    ServiceRequest req = {0, ""};
    ServiceResponse resp = {0x31, "This is response"};
    while (true)
    {
//        usleep(100000);
        int rc = comm.recv(&req, sizeof(req), COMM_WAIT);
        if (rc == RECV_MSG_FAIL)
            std::cout<<"Error when server recv."<<std::endl;

        std::cout<<"req.req_id = "<<req.req_id<<". req.req_buf = "<<req.req_buff<<std::endl;
        resp.res_id = req.req_id + 1;
       //send
        int send = comm.send(&resp, sizeof(resp), COMM_DONTWAIT);
        if (send == SEND_MSG_FAIL)        
            std::cout<<"Error when client send."<<std::endl;    
    }

    sleep(1);
    std::cout<<"ip2="<<ip<<std::endl;

    printf("Server: done\n");
    return 0;
}

#endif

