/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_node.cpp
Author:     Feng.Wu 
Create:     05-Dec-2018
Modify:     05-Dec-2018
Summary:    test process
**********************************************/
#ifndef IO_NODE_CPP_
#define IO_NODE_CPP_

#include "fst_io_device.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <parameter_manager/parameter_manager_param_group.h>
#include "base_device.h"
#include "common_log.h"
#include "fst_io_device_param.h"

using namespace fst_hal;

int main(int argc, char** argv)
{
    int num = 0;
    bool ret = false;
    std::vector<fst_hal::IODeviceInfo> vInfo;
    IODevicePortValues values;
    FstIoDevice io(2);

    ret = io.init();
	if(ret == false){
       std::cout<< "io init failed  " << ret << std::endl;
	   return 1;
    }
    else {
        std::cout << "io init OK. " << std::endl;
    }
//---------------getIODevNum()-----------------//
    num = io.getIODevNum();
	printf("io_device num: %d\n", num);

//---------------getIODevices---------------------//
    vInfo = io.getIODeviceList();
    for(int i = 0; i < num; ++i){
        printf("vector id=%d,devtype =%d,DInum=%d,DOnum=%d,RInum=%d,ROnum=%d\n",
               vInfo[i].id, vInfo[i].dev_type, vInfo[i].DI_num, vInfo[i].DO_num, vInfo[i].RI_num, vInfo[i].RO_num);
    }

//---------------getDeviceInfo---------------------//
    for(int i = 0; i < (num-1); ++i){
        io.getDevicePortValues(i, values);
        printf("id = %d,DI[]=0x%x-%x-%x-%x,DO[]=0x%x-%x-%x-%x,RI[]=0x%x,RO[]=0x%x\n",values.id,
               values.DI[3],values.DI[2],values.DI[1],values.DI[0],
               values.DO[3],values.DO[2],values.DO[1],values.DO[0],values.RI[0],values.RO[0]);
    }
    int virtual_id = 16;
    io.getDevicePortValues(virtual_id, values);
    printf("id = %d,DI[]=0x%x-%x-%x-%x,DO[]=0x%x-%x-%x-%x,RI[]=0x%x,RO[]=0x%x\n",values.id,
           values.virtual_DI[3],values.virtual_DI[2],values.virtual_DI[1],values.virtual_DI[0],
           values.virtual_DO[3],values.virtual_DO[2],values.virtual_DO[1],values.virtual_DO[0],values.RI[0],values.RO[0]);

//----------------setDIOByBit(uint32_t physics_id, uint8_t value);-------------------------------//
    // for id=0 device
    int dev = 0;

    PhysicsID id;
    id.info.dev_type = 2;
    id.info.address = virtual_id;//dev or virtual_id
    id.info.port = 1;
    while (true) {
        for (int i = 1; i <= 25; ++i) {
            id.info.port = i;
            id.info.port_type = IO_TYPE_DO;
            io.setDIOByBit(id.number, 1);
            usleep(500 * 1000);
            io.getDevicePortValues(virtual_id, values);
            printf("DI[]=%x-%x-%x-%x,DO[]=%x-%x-%x-%x,RI[]=%x,RO[]=%x\n",
                   values.virtual_DI[3],values.virtual_DI[2],values.virtual_DI[1],values.virtual_DI[0],
                   values.virtual_DO[3],values.virtual_DO[2],values.virtual_DO[1],values.virtual_DO[0],values.RI[0],values.RO[0]);

        }
        for (int i = 1; i <= 6; ++i) {
            id.info.port = i;
            id.info.port_type = IO_TYPE_RO;
            io.setDIOByBit(id.number, 1);
            usleep(500 * 1000);
            io.getDevicePortValues(virtual_id, values);
            printf("DI[]=%x-%x-%x-%x,DO[]=%x-%x-%x-%x,RI[]=%x,RO[]=%x\n",
                   values.DI[3],values.DI[2],values.DI[1],values.DI[0],
                   values.DO[3],values.DO[2],values.DO[1],values.DO[0],values.RI[0],values.RO[0]);
        }
        for (int i = 1; i <= 25; ++i) {
            id.info.port = i;
            id.info.port_type = IO_TYPE_DO;
            io.setDIOByBit(id.number, 0);
            usleep(500 * 1000);
            io.getDevicePortValues(virtual_id, values);
            printf("DI[]=%x-%x-%x-%x,DO[]=%x-%x-%x-%x,RI[]=%x,RO[]=%x\n",
                   values.virtual_DI[3],values.virtual_DI[2],values.virtual_DI[1],values.virtual_DI[0],
                   values.virtual_DO[3],values.virtual_DO[2],values.virtual_DO[1],values.virtual_DO[0],values.RI[0],values.RO[0]);
        }
        for (int i = 1; i <= 6; ++i) {
            id.info.port = i;
            id.info.port_type = IO_TYPE_RO;
            io.setDIOByBit(id.number, 0);
            usleep(500 * 1000);
            io.getDevicePortValues(virtual_id, values);
            printf("DI[]=%x-%x-%x-%x,DO[]=%x-%x-%x-%x,RI[]=%x,RO[]=%x\n",
                   values.DI[3],values.DI[2],values.DI[1],values.DI[0],
                   values.DO[3],values.DO[2],values.DO[1],values.DO[0],values.RI[0],values.RO[0]);
        }
    }

    printf("end.\n");

    return 0;
}


#endif 
