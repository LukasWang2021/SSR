/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_test.cpp
Author:     Feng.Wu 
Create:     05-May-2019
Modify:     05-May-2019
Summary:    test process
**********************************************/
#ifndef IODEV_TEST_CPP_
#define IODEV_TEST_CPP_

#include "fst_io_mem.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

int main(int argc, char** argv)
{  
    if (argc != 2){
        std::cout<<"There should be one argument for address."<<std::endl;
        return 0;
    }
    int address = atof(argv[1]);
    int offset = 0;

    int ret = ioInit();
    if(ret != 0)
    {
        printf("io test failed init\n");
        return ret;
    }

    ret = ioWriteId(offset, address);
    if(ret != 0)
    {
        printf("io test failed write id\n");
        return ret;
    }
    //ioWriteId(1, address+1);
    //ioWriteId(2, address+2);
    //ioWriteId(3, address+3);

    usleep(20*1000);

    int version = 0;
    ioBoardVersionFromMem(offset, &version);
    printf("io_board_%d version: %x\n", address, version);

    IODeviceData io;
    io.offset = offset;
    io.output[0] = 0xFF;
    io.output[1] = 0xFF;
    io.output[2] = 0xFF;
    io.output[3] = 0xFF;
    io.output[4] = 0xFF;

    if (ioWriteDownload(&io) != 0)
    {
        printf("ioWriteDownload() failed\n");
        return 0;
    }

    if (ioReadUpload(&io) != 0)
    {
        printf("ioReadUpload() failed\n");
        return 0;
    }
    
    printf("io.offset=%u, io.enable=%u, io.verify=%u, io.model=%u\n", io.offset, io.enable, io.verify, io.model);
    for(int i = 0; i < 5; ++i)
    {
        printf("input[%d] = 0x%x\n", i, io.input[i]);
    }
    for(int i = 0; i < 5; ++i)
    {
        printf("output[%d] = 0x%x\n", i, io.output[i]);
    }

    ioClose();
    return 0;
}


#endif 
