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

namespace fst_module_receive
{

class ModuleReceive
{
public:
    ModuleReceive(){}

    ~ModuleReceive(){}

    inline void init(void)
    {
        fd_ = nn_socket(AF_SP, NN_SUB);
        int rc = nn_connect(fd_, "inproc://inproc_test");
        if (rc == -1)
            printf("connect error");
        nn_setsockopt(fd_, NN_SUB, NN_SUB_SUBSCRIBE, "", 0);
//        sleep(1);

    }

    inline void recv(void)
    {
        char data[2024*1024];
        uint64_t nsec = 20;
//        clock_gettime(CLOCK_MONOTONIC, &start);
        //nn_recv(fd_, data, sizeof(data), 1);
        nn_recv(fd_, &nsec, sizeof(uint64_t), 0);
        clock_gettime(CLOCK_MONOTONIC, &stop);
        uint64_t delta = (stop.tv_nsec - nsec)/1000; 
        printf("start=%llu, stop=%llu, delta =%llu\n", nsec/1000,stop.tv_nsec/1000, delta);
//printf("delta =%llu\n", delta);

    }

private:

    int fd_;
    struct timespec stop, start;

};
} 

