/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       mSend.h
Author:     Feng.Wu 
Create:     05-Jan-2017
Modify:     05-Jan-2017
Summary:    test inproc
**********************************************/
#include <vector>
#include "error_code/error_code.h"

#include <nanomsg/nn.h>  
#include <nanomsg/reqrep.h>
#include <nanomsg/pubsub.h>
#include <sys/time.h>
#include <unistd.h>

namespace fst_module_send
{

class ModuleSend
{
public:
    ModuleSend(){}

    ~ModuleSend(){}

    inline void init(void)
    {
        fd_ = nn_socket(AF_SP, NN_PUB);
        int rc = nn_bind(fd_, "inproc://inproc_test");
        if (rc == -1)
            printf("bind error");
//        sleep(1);

    }

    inline void send(void)
    {
        char data[2024*1024];
        struct timespec start, stop;
        uint64_t nsec = 10;
        clock_gettime(CLOCK_MONOTONIC, &start);
        nsec = start.tv_nsec;
        nn_send(fd_, &nsec, sizeof(uint64_t), 0);
        //nn_send(fd_, data, sizeof(data), 0);
//        clock_gettime(CLOCK_MONOTONIC, &stop);
//        printf("delta =%llu\n", (stop.tv_nsec - nsec)/1000);
    }

private:

    int fd_;
    fst_comm_interface::CommInterface comm_;
};
} 

