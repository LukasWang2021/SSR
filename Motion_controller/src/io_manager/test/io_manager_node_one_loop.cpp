/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_manager_node.cpp
Author:     Feng.Wu 
Create:     15-Mar-2017
Modify:     15-Mar-2017
Summary:    test process
**********************************************/
#ifndef IO_MANAGER_IO_MANAGER_CPP_
#define IO_MANAGER_IO_MANAGER_CPP_

#include "io_manager/io_manager.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

int main(int argc, char** argv)
{    
    fst_io_manager::IOManager io_manager;
    io_manager.init(1);

    // -----------------get num of devices.----------------------//
    int num = io_manager.getDevicesNum();
    std::cout<<"num = "<<num<<std::endl;

    // ---------------check the thread error.--------------------//
    if (io_manager.getIOError() != FST_SUCCESS)
    {
        std::cout<<"1 thread error"<<std::endl;
//        return -1;
    }

    // ----------------get info of devices.---------------------//
    fst_io_manager::IODeviceInfo info[4];
    U64 result;
    for (int i = 0; i < num; ++i)
    {
        result = io_manager.getDeviceInfo(i, info[i]);
        if(result == FST_SUCCESS)
        {
            std::cout<<"Device: path="<<info[i].path;
            std::cout<<". id="<<info[i].id;
            std::cout<<". comm_type="<<info[i].communication_type;
            std::cout<<". device_number="<<info[i].device_number;
            std::cout<<". device_type="<<info[i].device_type;
            std::cout<<". input="<<info[i].input;
            std::cout<<". output="<<info[i].output<<std::endl;
        }
        else
        {
            return -1;
        }
    }

    // --------------------set port value.-----------------------//
    if (io_manager.getIOError() != FST_SUCCESS)
    {
        std::cout<<"2 thread error"<<std::endl;
//        return -1;
    }
    result = io_manager.setModuleValue(info[0].id, 24, 1);
    io_manager.setModuleValue(info[0].id, 16, 1);
    io_manager.setModuleValue(info[0].id, 8, 1);
    usleep(50*1000);
   
    // -------------------get all port values.---------------------//
    if (io_manager.getIOError() != FST_SUCCESS)
    {
        std::cout<<"3 thread error"<<std::endl;
//        return -1;
    }
    //unsigned char *arr = new unsigned char[72];// don't forget delete []arr;
    //unsigned char *arr = (unsigned char *) malloc(72);//don't forget free(arr);
    for (int i = 0; i < num; ++i)
    {
        unsigned char arr[72] = {0};
        int num2 = 0;
        result = io_manager.getModuleValues(info[i].id, 72, arr, num2);
        if (result == FST_SUCCESS)
        {
            std::cout<<"id = "<<info[i].id<<". all port values = ";
            for(unsigned int j = 0; j < (unsigned int)num2; ++j)
            {
                std::cout<<int(arr[j]);
                if ((j + 1) == info[i].input)
                    std::cout<<" ";
            }
            std::cout<<std::endl;
        }
    }
    // --------------------get one port value.--------------------//
    if (io_manager.getIOError() != FST_SUCCESS)
    {
        std::cout<<"4 thread error"<<std::endl;
//        return -1;
    }
    unsigned char value;
    int port_seq = 8;
    result = io_manager.getModuleValue(info[0].id, IO_OUTPUT, port_seq, value);
    if (result == FST_SUCCESS)
        std::cout<<"id = "<<info[0].id<<". port["<<port_seq<<"] = "<<int(value)<<std::endl;

    return 0;
}


#endif //IO_MANAGER_IO_MANAGER_NODE_CPP_
