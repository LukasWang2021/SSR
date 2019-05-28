//
// Created by fst on 18-8-15.
//

#ifndef COMMON_ENUM_H
#define COMMON_ENUM_H

namespace fst_mc
{

enum ServiceID
{
    JTAC_CMD_SID = 0x01,
    READ_VERSION_SID = 0x10,
    HEARTBEAT_INFO_SID = 0x11,
    READ_SERVO_DATA_BY_ADDR = 0x14,
    READ_DATA_BY_ID = 0x1D,
    WRITE_SERVO_DATA_BY_ADDR = 0x24,
    WRTIE_DATA_BY_ID = 0x2D,
    READ_DTC_SID = 0x30,
    READ_SERVO_DTC_SID = 0x31,
    LOG_CONTROL_SID = 0x40,
    LOG_GETLIST_SID = 0x41,
    SERVO_CMD_SID = 0x60,
    GET_ENCODER_SID  = 0x70,
    GET_ENCODER_ERR_SID = 0x71,
	RESET_ENCODER_ERR_SID = 0x72,
    GET_CONTROL_POS_SID = 0x80,
    MONITOR_HEARTBEAT_SID = 0xA1,
};

enum MotionType
{
    MOTION_NONE   = 0,
    MOTION_JOINT  = 1,
    MOTION_LINE   = 2,
    MOTION_CIRCLE = 3,
    MOTION_XPOS   = 4,
};

enum SmoothType
{
    SMOOTH_NONE = 0,
    SMOOTH_DISTANCE = 1,
    SMOOTH_VELOCITY = 2,
};

enum CoordinateType
{
    COORDINATE_JOINT = 0,
    COORDINATE_CARTESIAN = 1,
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
    UNKNOW  = 0x0,
    DISABLE = 0x1,
    STANDBY = 0x2,
    MANUAL  = 0x3,
    AUTO    = 0x4,
    PAUSE   = 0x5,
    PAUSE_RETURN = 0x6,
    PAUSE_MANUAL = 0x7,
    PAUSING = 0x8,

    DISABLE_TO_STANDBY = 0x12,
    STANDBY_TO_DISABLE = 0x21,
    MANUAL_TO_STANDBY = 0x32,
    STANDBY_TO_MANUAL = 0x23,
    AUTO_TO_STANDBY = 0x42,
    STANDBY_TO_AUTO = 0x24,
    AUTO_TO_PAUSE = 0x45,
    PAUSE_RETURN_TO_STANDBY = 0x62,
    PAUSE_TO_PAUSE_RETURN = 0x56,
    PAUSE_TO_PAUSE_MANUAL = 0x57,
    PAUSE_MANUAL_TO_PAUSE = 0x75,
};

}

#endif //COMMON_ENUM_H
