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

enum PointProperty
{
    POINT_POS = 1,
    POINT_POS_VEL = 2,
    POINT_POS_VEL_ACC = 4,
    POINT_POS_VEL_ACC_EFF = 8,
};

enum PointLevel
{
    POINT_MIDDLE = 0,
    POINT_START  = 1,
    POINT_ENDING = 2,
};

enum ManualMode
{
    STEP,
    CONTINUOUS,
    APOINT,
};

enum ManualDirection
{
    STANDING = 0,
    INCREASE = 1,
    DECREASE = 2,
};

enum GroupState
{
    STANDBY = 0,
    MANUAL = 1,
    AUTO = 2,
    PAUSE = 3,

    MANUAL_TO_STANDBY = 0xF10,
    STANDBY_TO_MANUAL = 0xF01,
    AUTO_TO_STANDBY = 0xF20,
    STANDBY_TO_AUTO = 0xF02,
    AUTO_TO_PAUSE = 0xF23,
    PAUSE_TO_AUTO = 0xF32,
    PAUSE_TO_STANDBY = 0xF30,
};

}

#endif //COMMON_ENUM_H
