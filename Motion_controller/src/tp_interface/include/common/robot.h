/**
 * @file robot.h
 * @brief 
 * @author WangWei
 * @version 1.0
 * @date 2016-09-26
 */
#ifndef TP_INTERFACE_ROBOT_H_
#define TP_INTERFACE_ROBOT_H_

typedef enum _RobotMode
{
	INIT_M					= 0,	
	PAUSE_M					= 1,
	AUTO_RUN_M				= 2,
	MANUAL_JOINT_MODE_M		= 3,
	MANUAL_CART_MODE_M		= 4,
	AUTO_RESET_M			= 5,

	AUTO_RUN_TO_PAUSE_T				= 201,
	PAUSE_TO_AUTO_RUN_T				= 102,
	PAUSE_TO_MANUAL_JOINT_T			= 103,
	PAUSE_TO_MANUAL_CART_T			= 104,
	MANUAL_CART_TO_PAUSE_T			= 401,
	MANUAL_JOINT_TO_PAUSE_T			= 301,
	MANUAL_CART_TO_MANUAL_JOINT_T	= 403,
	MANUAL_JOINT_TO_MANUAL_CART_T	= 304
}RobotMode;

typedef enum _RobotModeCmd
{
	GOTO_NONE_E					= -1,
	GOTO_PAUSE_E				= 1,
	GOTO_AUTO_RUN_E				= 2,
	GOTO_MANUAL_JOINT_MODE_E	= 3,
	GOTO_MANUAL_CART_MODE_E		= 4,
	GOTO_AUTO_RESET_E			= 5
}RobotModeCmd;

typedef enum _RobotState
{
	INIT_S			= 0,
	OFF_S			= 1,
	DISENGAGED_S	= 2,
	RETARCT_S		= 3,
	ENGAGED_S		= 4,
	CALIBRATE_S		= 5,
    REFERENCING_S   = 6,
    ESTOP_S         = 7,

	OFF_TO_DISENGAGED_T		= 102,
	OFF_TO_CALIBRATE_T		= 105,
	DISENGAGED_TO_OFF_T		= 201,
	DISENGAGED_TO_RETRACT_T = 203,
	RETARCT_TO_DISENGAGED_T	= 302,
	DISENGAGED_TO_ENGAGED_T = 204,
	ENGAGED_TO_DISENGAGED_T = 402,
    TO_FORCEDDISENGAGE_T    = 600,
    TO_ESTOP_T              = 700,
    RESET_ESTOP_T           = 701
}RobotState;

typedef enum _RobotStateCmd
{
	DO_NOTHING			= -1,
	GOTO_OFF			= 0,
	GOTO_DISENGAGED_E	= 1,
	GOTO_ENGAGED_E		= 2,
	GOTO_RETRACT_E		= 3,
	GOTO_REFERENCING_E	= 4,
	GOTO_AUTORESET_E	= 5,
    FORCE_DISENGAGE_E   = 10,
    EMERGENCY_STOP_E    = 20,
    SAVE_CONFIGURATION_E = 254,
    ACKNOWLEDGE_ERROR   = 255
}RobotStateCmd;

typedef enum _RobotCtrlCmd
{
    NONE                = -1,
    SHUTDOWN_CMD        = 1
}RobotCtrlCmd;

typedef enum _ManualState
{
	MANUAL_IDLE_S		= 0,
	MANUAL_READY_S		= 1,
	MANUAL_RUNNING_S	= 2,
    MANUAL_SUSPEND_S    = 3
}ManualState;

typedef enum _AutoState
{
    AUTO_IDLE_S         = 0,
    AUTO_READY_S        = 1,
    AUTO_RUNNING_S      = 2,
    AUTO_SUSPEND_S      = 3
}AutoState;


#define MAX_JOINTS				(6)
#define MAX_LINE_SPEED			(4000)
#define MAX_MANUAL_SPEED        (1000)

#define INTERVAL_PROPERTY_UPDATE   	(10)    //ms
#define INTERVAL_REF_START_UPDATE	(3000)  //every 3000ms update start state
#define INTERVAL_PROCESS_UPDATE	    (2)
#define INTERVAL_HEART_BEAT_UPDATE  (50)


#define MIN_ACCURATE_VALUE		(0.00001)

#endif
