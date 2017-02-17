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

namespace fst_module_receive
{

class ModuleReceive
{
public:
    ModuleReceive(){}

    ~ModuleReceive(){}

    inline void init(void)
    {
        int result = comm_.createChannel(COMM_SUB, COMM_INPROC, "intra");
        if (result == -1)
            printf("rec create error\n");
//        sleep(1);

    }

    inline void recv(void)
    {
        uint64_t nsec = 20;
//        clock_gettime(CLOCK_MONOTONIC, &start);
        comm_.recv(&nsec, sizeof(uint64_t), COMM_WAIT);
        clock_gettime(CLOCK_MONOTONIC, &stop);
        uint64_t delta = (stop.tv_nsec - nsec)/1000; 
        printf("start=%llu, stop=%llu, delta =%llu\n", nsec/1000,stop.tv_nsec/1000, delta);
//printf("delta =%llu\n", delta);

    }

private:

    struct timespec stop, start;
    fst_comm_interface::CommInterface comm_;

};
} 

