/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       mapping_test.cpp
Author:     Feng.Wu 
Create:     15-Obt-2018
Modify:     15-Obt-2018
Summary:    test process
**********************************************/
#ifndef IO_MAPPING_MAPPING_TEST_CPP_
#define IO_MAPPING_MAPPING_TEST_CPP_

#include "io_mapping.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
using namespace fst_hal;

int main(int argc, char** argv)
{
    fst_hal::FstIoDevice* io_device_ptr = new fst_hal::FstIoDevice(fst_hal::DEVICE_TYPE_FST_IO);
    io_device_ptr->init();

    fst_ctrl::IoMapping map;
    int ret = map.init(io_device_ptr);
    if (ret != 0) {
        printf("failed init io mapping\n");
        return -1;
    }

    uint8_t value = 9;
    for(int i = 1; i <= 6; ++i) {
        map.getDIByBit(i, value);
        printf("getDI[%d]=%d", i, value);
        value = 1;
        map.setDIByBit(i, value);
        printf("\t setDI[%d]=%d", i, value);
        map.getDIByBit(i, value);
        printf("\t getDI[%d]=%d\n", i, value);

        map.getRIByBit(i, value);
        printf("getRI[%d]=%d", i, value);
        value = 0;
        map.setRIByBit(i, value);
        printf("\t setRI[%d]=%d", i, value);
        map.getRIByBit(i, value);
        printf("\t getRI[%d]=%d\n", i, value);
    }

    int map_id = map.getIOPhysicsID("DI[2]");
    printf("test id = 0x%x\n",map_id);

    PhysicsID id;
    id.info.port=8;
    id.info.port_type = 1;
    id.info.address = 3;
    id.info.dev_type =2;
    printf("map id1 = 0x%x\n", id.number);
    id.info.port = 9;
    printf("map id2 = 0x%x\n", id.number);

    delete io_device_ptr;
    return 0;
}


#endif 
