//
// Created by fst on 18-8-15.
//

#ifndef COMMON_ENUM_H
#define COMMON_ENUM_H

namespace group_space
{

typedef enum
{
    USER_OP_MODE_NONE             = 0,
    USER_OP_MODE_AUTO             = 1,
    USER_OP_MODE_SLOWLY_MANUAL    = 2,
    USER_OP_MODE_MANUAL = 3,
    USER_OP_MODE_ONLINE = 4,
}UserOpMode;

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
    SET_OMEGA_FILTER_SID = 0x90,
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
    SERVO_INIT = 0,
    SERVO_IDLE = 1,
    SERVO_RUNNING = 2,
    SERVO_DISABLE = 3,
    SERVO_WAIT_READY = 4,
    SERVO_WAIT_DOWN = 5, 
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

enum EncoderState
{
    VALID = 0,
    BATTERY_ERROR = 1,
    BATTERY_WARN = 2,
    COMMUNICATION_LOST = 4,
    COMMUNICATION_ERROR = 8,
    INVALID = 16,
};

enum MotionControlState
{
    STANDBY = 0x2,
    MANUAL  = 0x3,
    AUTO    = 0x4,
    PAUSE   = 0x5,
    PAUSE_RETURN = 0x6,
    PAUSE_MANUAL = 0x7,
    PAUSING = 0x8,
    OFFLINE = 0x9,
    RESUME = 0xA,
    PREPARE_RESUME = 0xB,
    ONLINE = 0xC,
    PAUSE_ONLINE = 0x0D,

    MANUAL_TO_STANDBY = 0x32,
    STANDBY_TO_MANUAL = 0x23,
    AUTO_TO_STANDBY = 0x42,
    STANDBY_TO_AUTO = 0x24,
    STANDBY_TO_OFFLINE = 0x29,
    OFFLINE_TO_STANDBY = 0x92,
    AUTO_TO_PAUSING = 0x48,
    PAUSING_TO_PAUSE = 0x85,
    PAUSE_TO_RESUME = 0x5A,
    PAUSE_RETURN_TO_PAUSE = 0x65,
    PAUSE_TO_PAUSE_RETURN = 0x56,
    PAUSE_TO_PAUSE_MANUAL = 0x57,
    PAUSE_MANUAL_TO_PAUSE = 0x75,
};

}

#endif //COMMON_ENUM_H
