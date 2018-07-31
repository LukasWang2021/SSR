/**
 * @file robot.h
 * @brief 
 * @author WangWei
 * @version 1.0
 * @date 2016-09-26
 */
#ifndef TP_INTERFACE_ROBOT_H_
#define TP_INTERFACE_ROBOT_H_

typedef enum _MotionMode
{
	INIT_M				= 0,	
	PROGRAM_EXE_M		= 1,
	TEACH_M     		= 2,
}MotionMode;

typedef enum _MotionModeCmd
{
	GOTO_NONE_E			= -1,
	GOTO_PROGRAM_EXE_E	= 0,
	GOTO_TEACH_E    	= 1,
}MotionModeCmd;

typedef enum _RobotState
{
	INIT_S			    = 0,
	ENGAGED_S		    = 4,
    ESTOP_S             = 7,
    TERMINATED_S        = 8,

    TO_ESTOP_T          = 700,
    RESET_ESTOP_T       = 701,
    TERMINATING_T       = 801,
}RobotState;


typedef enum _WorkStatus
{
    IDLE_W      = 0,
    RUNNING_W   = 1,
    TEACHING_W  = 2,


    IDLE_TO_RUNNING_T   = 101,
    IDLE_TO_TEACHING_T  = 102,
    RUNNING_TO_IDLE_T   = 103,
    TEACHING_TO_IDLE_T  = 104,
}WorkStatus;


typedef enum _RobotStateCmd
{
    EMERGENCY_STOP_E    = 20,
    SAVE_CONFIGURATION_E = 254,
    ACKNOWLEDGE_ERROR   = 255,
    GOTO_TERMINATED     = 300,
}RobotStateCmd;

typedef enum _RobotCtrlCmd
{
    NONE                = -1,
    SHUTDOWN_CMD        = 1,
    
    PAUSE_CMD           = 10,
    CONTINUE_CMD        = 11,
    ABORT_CMD           = 12,

    CALIBRATE_CMD       = 20,
    SETTMPZERO_CMD		= 21
}RobotCtrlCmd;




typedef enum _RunningMode
{
    NORMAL_R        = 0, 
    CALIBRATE_R     = 0x1,
    SOFTLIMITED_R   = 0x2,
}RunningMode;


typedef enum _PointType
{
    JOINT_M,
    CART_M
}PointType;

// XX STATE MACHINE
typedef enum _ServoStatus
{
    SERVO_STATE_INIT = 10,
    SERVO_STATE_READY = 1,
    SERVO_STATE_RUNNING = 2,
    SERVO_STATE_ERROR = 3,
    SERVO_STATE_WAIT_SERVOREADY = 4,
    SERVO_STATE_WAIT_SERVODOWN = 5,
}ServoStatus;


#define MAX_JOINTS				(6)
#define MAX_LINE_SPEED			(4000)      //ms/s
#define MAX_RAD_SPEED           (10)        //rad/s
#define MAX_MANUAL_SPEED        (1000)      //ms/s

#define TRAJ_FLOW_INTERVAL         	(5)    //ms
#define STATE_MACHINE_INTERVAL      (2)
#define SM_INTERVAL_COUNT           (5)  //2ms*5=10ms
#define HEART_BEAT_INTERVAL         (50)    //ms

#define DEFAULT_ACC			    (7000)

#define MIN_ACCURATE_VALUE		(0.00001)

#define MAX_CNT_VAL		(100)

#endif
