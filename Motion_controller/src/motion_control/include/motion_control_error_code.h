/*************************************************************************
	> File Name: motion_control_error_code.h
	> Author: 
	> Mail: 
	> Created Time: 2016年11月30日 星期三 16时59分39秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_ERROR_CODE_H
#define _MOTION_CONTROL_ERROR_CODE_H

typedef unsigned long long int ErrorCode;


#define IK_OUT_OF_WORKSPACE (unsigned long long int)0x00010004006503E9   /*IK failed for Axis 1*/
#define IK_JOINT_OUT_OF_LIMIT (unsigned long long int)0x00010004006503EA   /*IK failed for Axis 2~6*/
#define IK_EXCESSIVE_DISTANCE (unsigned long long int)0x00010004006503EB   /*IK result far away from reference*/
#define FK_JOINT_OUT_OF_LIMIT (unsigned long long int)0x00010004006503F3   /*joint out of limit computing FK*/
#define TARGET_REPEATED (unsigned long long int)0x00000001006603F5   /*Motion Command planned with 0 point*/
#define AXIS_OVERSHOOT (unsigned long long int)0x0001000400650407   /*axis run over target position in moveJ*/
#define AXIS_APPROACHING_LIMIT (unsigned long long int)0x0001000400650408   /*axis approaching and going to crash a limit*/
#define CUBIC_CURVE_PLANNING_FAILED (unsigned long long int)0x0001000400650400   /*planning failed using cubic curve*/
#define MOTION_SPEED_TOO_LOW (unsigned long long int)0x0001000400650401   /*motion speed is too low*/
#define CURVE_PRECISION_TOO_LOW (unsigned long long int)0x000100040065041D   /*get a low precision curve when planning MoveC*/
#define THREE_POINTS_COLINEAR (unsigned long long int)0x000100040065041E   /*given points colinear, cannot define a circle*/
#define PLANNING_MAJOR_ARC (unsigned long long int)0x000100040065041F   /*given circle targets define a major arc*/
#define S_CURVE_PLANNING_FAILED (unsigned long long int)0x0001000400650403   /*failed to plan a S-CURVE*/
#define INVALID_CNT (unsigned long long int)0x0001000400650405   /*given CNT parameter is invalid*/
#define AXIS_INCONSISTENT (unsigned long long int)0x0001000400650409   /*failed to coordinate every axis*/

#define MOTION_INTERNAL_FAULT (unsigned long long int)0x0001000400660001   /*program internal fault*/
#define MOTION_FAIL_IN_INIT (unsigned long long int)0x00010002006603E9   /*initialization failed*/
#define FAIL_LOADING_CONSTRAINT (unsigned long long int)0x00010002006603EA   /*load joint constraint failed*/
#define FAIL_LOADING_PARAMETER (unsigned long long int)0x00010002006603EB   /*load parameter failed*/
#define JOINT_OUT_OF_CONSTRAINT (unsigned long long int)0x00010004006603F3   /*joint out of constraint*/
#define TARGET_OUT_OF_CONSTRAINT (unsigned long long int)0x00010004006603F4   /*target joint out of constraint*/
#define INVALID_PARAMETER (unsigned long long int)0x00010004006603FD   /*APIs called with an invalid parameter*/
#define INVALID_SEQUENCE (unsigned long long int)0x00010004006603FE   /*APIs called in a invalid sequence*/
#define CURRENT_JOINT_OUT_OF_CONSTRAINT (unsigned long long int)0x00000001006603FF   /*current joint is out of constraint when resetArmGroup*/
#define TRAJECTORY_FIFO_FULL (unsigned long long int)0x0000000200660401   /*trajectory FIFO full, cannot fill points into FIFO2*/
#define CARTESIAN_PATH_EXIST (unsigned long long int)0x0001000200660402   /*cartesian point exist, cannot plan joint path*/
#define NO_ENOUGH_POINTS_FIFO1 (unsigned long long int)0x0001000200660407   /*have no enough points in FIFO1*/
#define NO_ENOUGH_POINTS_FIFO2 (unsigned long long int)0x0001000200660408   /*have no enough points in FIFO2*/
#define CALIBRATION_FAULT (unsigned long long int)0x00010002006607D1   /*error while calibrating zero offset*/
#define ZERO_OFFSET_LOST (unsigned long long int)0x00010004006607D2   /*one or more axis lost its zero offset*/
#define ZERO_OFFSET_DEVIATE (unsigned long long int)0x00010004006607D3   /*axis zero offset deviated*/
#define CALIBRATION_FAIL_IN_INIT (unsigned long long int)0x00000002006607DB   /*calibrator initialization failed*/
#define FAIL_GET_FEEDBACK_JOINT (unsigned long long int)0x00010002006607E6   /*fail to get FeedbackJointState*/
#define NEED_INITIALIZATION (unsigned long long int)0x0001000200660411   /*ArmGroup need to initialize */
#define NEED_CALIBRATION (unsigned long long int)0x0001000400660412   /*ArmGroup need to calibrate*/
#define IPC_COMMUNICATION_ERROR (unsigned long long int)0x00010002006607E7   /*fail to communicate with other process*/
#define BARE_CORE_TIMEOUT (unsigned long long int)0x00010004006607E8   /*fail to communicate with bare core*/

#ifndef SUCCESS
#define SUCCESS (unsigned long long int)0x0000000000000000
#endif


#endif
