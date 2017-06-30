/**********************************************
File: test.c
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: test tcp/ip
Author: Feng.Wu 27-03-2017
Modifier:
**********************************************/

#ifndef TEST_C_
#define TEST_C_

#include <iostream>
#include "error_code/error_code.h"
#include "comm_interface/comm_interface.h"

#define SERVER_PORT "5559"
bool isLittleEndian(void)
{
	union FourByte{
		uint32_t i;
		char j;
	}e;
	e.i = 1;
	if (e.j == 1)
		return true;
	return false;
}

int main(int argc, char *argv[]) {

    printf("System is %s - Endian.\n", isLittleEndian() ? "Little" : "Big");
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
    std::cout<<"server bind ok :"<<ip_addr<<std::endl;

    int req = 0, resp = 110;
    while (true)
    {
        int rc = comm.recv(&req, sizeof(req), COMM_WAIT);
        if (rc == RECV_MSG_FAIL)
            std::cout<<"Error when server recv."<<std::endl;;

         std::cout<<"server recv = "<<req<<std::endl;
        int send = comm.send(&resp, sizeof(resp), COMM_DONTWAIT);
        if (send == SEND_MSG_FAIL)          
            std::cout<<"Error when client send."<<std::endl;    
    }

    std::cout<<"Server: done"<<std::endl;
    return 0;
}

#endif

