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

#include <comm_interface/comm_interface.h>
//#include <nanomsg/nn.h>  
//#include <nanomsg/reqrep.h>
//#include <nanomsg/pubsub.h>
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
        int result = comm_.createChannel(COMM_PUB, COMM_INPROC, "intra");
        if (result == -1)
            printf("send create error\n");
//        sleep(1);

    }

    inline void send(void)
    {
        struct timespec start, stop;
        uint64_t nsec = 10;
        clock_gettime(CLOCK_MONOTONIC, &start);
        nsec = start.tv_nsec;
        comm_.send(&nsec, sizeof(uint64_t), COMM_DONTWAIT);
//        clock_gettime(CLOCK_MONOTONIC, &stop);
//        printf("delta =%llu\n", (stop.tv_nsec - nsec)/1000);
    }

private:

    fst_comm_interface::CommInterface comm_;
};
} 

