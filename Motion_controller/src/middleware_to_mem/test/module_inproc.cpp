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
#include "module_comm_send.h"
#include "module_comm_receive.h"

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


    fst_module_send::ModuleSend moduleSend;
    fst_module_receive::ModuleReceive moduleRecv;
    moduleSend.init();
    moduleRecv.init();
    sleep(1);

    for (int i = 0; i<10; ++i)
    {
        moduleSend.send();
        moduleRecv.recv();
//        usleep(10);
    }
/*
    for (int j = 0; j<10; ++j)
    {
        moduleRecv.recv();
    }
*/
    
    
    return 0;
}

#endif

