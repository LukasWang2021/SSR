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
    int init_result = io_manager.init(1);
    if (init_result != 0)
        return -1;

    // -----------------get num of devices.----------------------//
    int num = io_manager.getDevicesNum();
    std::cout<<"Devices num = "<<num<<std::endl;

    // ---------------check the thread error.--------------------//
    if (io_manager.getIOError() != FST_SUCCESS)
    {
        std::cout<<"init thread error"<<std::endl;
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

    unsigned char arr[4][10] = {0}, temp[4][10] = {0};
    while (true)
    {
        // --------------------set port value.-----------------------//
        U64 error = io_manager.getIOError();
        if (error != FST_SUCCESS)
        {
            std::cout<<"running thread error "<<error<<std::endl;
//          return -1;
        }
   
        // -------------------get all port values.---------------------//
        for (int i = 0; i < num; ++i)
        {
            int num2 = 0;
            for (unsigned int k = 0; k < (info[i].output - 1); ++k)
            {
//                io_manager.setModuleValue(info[i].id, (k + 1), 0);
//                io_manager.setModuleValue(info[i].id, (k + 2), 1);

                usleep(40*1000);

                memcpy(temp[i], arr[i], sizeof(temp[i]));
                result = io_manager.getModuleValues(info[i].id, 10, arr[i], num2);
                bool flag = false;
                if (result == FST_SUCCESS)
                {
                    for (unsigned int j = 0;  j < 10; ++j)
                    {
                        if (temp[i][j] != arr[i][j])
                        {
                            flag = true;
                            break;
                        }
                    }
                    if (flag == true)
                    {
                        std::cout<<"id = "<<info[i].id<<". all port values = ";
                        for(unsigned int j = 0; j < num2; ++j)
                        {
//                            std::cout<<int(arr[i][j])<<" ";
                            std::cout<<std::bitset<8>(arr[i][j])<<" ";
                            if ((j + 1) == 5)
                                std::cout<<" | ";
                        }
                        std::cout<<std::endl;
                    }
                }
            }
        }
    }

    io_manager.setModuleValue(info[0].id, 1, 1);
    usleep(100*1000);
    int num3;
    result = io_manager.getModuleValues(info[0].id, 72, arr[0], num3);
    if (result == FST_SUCCESS)
    {
        for(unsigned int j = 0; j < num3; ++j)
        {
            std::cout<<int(arr[0][j]);
            if ((j + 1) == info[0].input)
                std::cout<<" ";
        }
        std::cout<<std::endl;
    }



    // --------------------get one port value.--------------------//
    unsigned char value;
    int port_seq = 8;
    result = io_manager.getModuleValue(info[0].id, IO_OUTPUT, port_seq, value);
    if (result == FST_SUCCESS)
        std::cout<<"id = "<<info[0].id<<". port["<<port_seq<<"] = "<<int(value)<<std::endl;

    return 0;
}


#endif //IO_MANAGER_IO_MANAGER_NODE_CPP_
