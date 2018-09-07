//
// Created by fst on 18-8-15.
//

#ifndef COMMON_ENUM_H
#define COMMON_ENUM_H

namespace fst_mc
{

enum MotionType
{
    MOTION_NONE   = 0,
    MOTION_JOINT  = 1,
    MOTION_LINE   = 2,
    MOTION_CIRCLE = 3,
};

enum ManualFrame
{
    JOINT,
    BASE,
    WORLD,
    USER,
    TOOL,
};

typedef enum 
{
    SERVO_IDLE = 1,
    SERVO_RUNNING = 2,
    SERVO_DISABLE = 3,
    SERVO_WAIT_READY = 4,
    SERVO_WAIT_DOWN = 5,
    SERVO_INIT = 10,
}ServoState;

}

#endif //COMMON_ENUM_H
