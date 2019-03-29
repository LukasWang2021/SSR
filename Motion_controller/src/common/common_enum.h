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

enum PointType
{
    PATH_POINT = 0,
    TRANSITION_POINT = 1,
};

enum ManualFrame
{
    JOINT,
    BASE,
    WORLD,
    USER,
    TOOL,
};

enum ServoState 
{
    SERVO_IDLE = 1,
    SERVO_RUNNING = 2,
    SERVO_DISABLE = 3,
    SERVO_WAIT_READY = 4,
    SERVO_WAIT_DOWN = 5,
    SERVO_INIT = 10,
};

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

enum AxisType
{
    ROTARY_AXIS = 0,
    LINEAR_AXIS = 1,
};

enum GroupState
{
    UNKNOW = 0x0,
    DISABLE = 0x1,
    STANDBY = 0x2,
    MANUAL = 0x3,
    AUTO = 0x4,
    PAUSE = 0x5,
    PAUSE_RETURN = 0x6,

    DISABLE_TO_STANDBY = 0x12,
    STANDBY_TO_DISABLE = 0x21,
    MANUAL_TO_STANDBY = 0x32,
    STANDBY_TO_MANUAL = 0x23,
    AUTO_TO_STANDBY = 0x42,
    STANDBY_TO_AUTO = 0x24,
    AUTO_TO_PAUSE = 0x45,
    PAUSE_RETURN_TO_STANDBY = 0x62,
    PAUSE_TO_PAUSE_RETURN = 0x56,
};

}

#endif //COMMON_ENUM_H
