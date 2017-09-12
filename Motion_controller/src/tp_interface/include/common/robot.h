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
	AUTO_RUN_M				= 1,
	MANUAL_MODE_M		    = 2,


	AUTO_RUN_TO_MANUAL_T	= 101,
	MANUAL_TO_AUTO_RUN_T	= 102,
}RobotMode;

typedef enum _RobotModeCmd
{
	GOTO_NONE_E				= -1,
	GOTO_AUTO_RUN_E			= 0,
	GOTO_MANUAL_MODE_E	    = 1,
}RobotModeCmd;

typedef enum _RobotState
{
	INIT_S			    = 0,
	OFF_S			    = 1,
	DISENGAGED_S	    = 2,
	RETARCT_S		    = 3,
	ENGAGED_S		    = 4,
	CALIBRATE_S		    = 5,
    REFERENCING_S       = 6,
    ESTOP_S             = 7,

	OFF_TO_DISENGAGED_T		= 102,
	OFF_TO_CALIBRATE_T		= 105,
	DISENGAGED_TO_OFF_T		= 201,
	DISENGAGED_TO_RETRACT_T = 203,
	RETARCT_TO_DISENGAGED_T	= 302,
	DISENGAGED_TO_ENGAGED_T = 204,
	ENGAGED_TO_DISENGAGED_T = 402,
    TO_FORCEDDISENGAGE_T    = 600,
    TO_ESTOP_T              = 700,
    RESET_ESTOP_T           = 701,
}RobotState;


typedef enum _ProgramState
{
    IDLE_R      = 0,
    EXECUTE_R   = 1,
    PAUSED_R    = 2,

    IDLE_TO_EXECUTE_T   = 101,
    EXECUTE_TO_PAUSE_T  = 102,
    PAUSE_TO_IDLE_T     = 103,
    PAUSE_TO_EXECUTE_T  = 104,
}ProgramState;

typedef enum _ProgramStateCmd
{
    GOTO_IDLE_E         = 0,
    GOTO_EXECUTE_E      = 1,
    GOTO_PAUSED_E       = 2
}ProgramStateCmd;

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
    ACKNOWLEDGE_ERROR   = 255,
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



#define MAX_JOINTS				(6)
#define MAX_LINE_SPEED			(4000)      //ms/s
#define MAX_RAD_SPEED           (10)        //rad/s
#define MAX_MANUAL_SPEED        (1000)      //ms/s

#define INTERVAL_PROPERTY_UPDATE   	(10)    //ms
#define INTERVAL_PROCESS_UPDATE	    (6)     //ms
#define INTERVAL_HEART_BEAT_UPDATE  (50)    //ms


#define MIN_ACCURATE_VALUE		(0.00001)

#endif
