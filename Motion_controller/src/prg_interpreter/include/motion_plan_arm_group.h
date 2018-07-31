/*************************************************************************
	> File Name: motion_plan_arm_group.h
	> Author: 
	> Mail: 
	> Created Time: 2017年12月11日 星期一 09时14分31秒
 ************************************************************************/

#ifndef _MOTION_PLAN_ARM_GROUP_H
#define _MOTION_PLAN_ARM_GROUP_H

#include <fstream>
#include <string>
#include <vector>
#include <pthread.h>
#include <fst_datatype.h>

#define     PATH_FIFO_CAPACITY              2048        // must be setted to 2~N
#define     MOTION_POOL_CAPACITY            4           // must be setted to 2^N

namespace fst_controller
{

enum MotionState
{
    IDLE = 0x0,
    AUTO_RUNNING = 0x100,
    AUTO_RUNNING_TO_PAUSE = 0x101,
    AUTO_PAUSE = 0x110,
    AUTO_RESUME = 0x120,
    AUTO_RESUME_TO_PAUSE = 0x121,
    AUTO_READY = 0x130,
};

enum SpeedState
{
    SPEED_KEEP = 0,
    SPEED_UP = 1,
    SPEED_DOWN = 2,
};

struct ControlPointCache
{
    bool    valid;
    size_t  head;
    size_t  tail;
    size_t  smooth_in_stamp;
    size_t  smooth_out_stamp;
    double  deadline;

    ControlPoint path[PATH_FIFO_CAPACITY];
    ControlPointCache   *prev;
    ControlPointCache   *next;
};

}

#endif
