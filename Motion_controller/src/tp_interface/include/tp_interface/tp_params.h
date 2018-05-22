/**
 * @file tp_params.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-07-20
 */

#ifndef RCS_TP_PARAMS_H_
#define RCS_TP_PARAMS_H_


#include "base_types.pb.h"

BaseTypes_ParamInfo g_param_info[] = 
/*
{												
	{	"root/Logic/curveMode",	32965,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/controllerWorkStatus",	31619,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/interpreterState",	48981,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/state",	40629,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/userOperationMode",	36933,	0,	5,	4,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/stateCommand",	7076,	0,	5,	4,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/actualJointPositionsFiltered",	1284,	0,	258,	8,	6,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/forwardKinematics/toolCoordinates",	77427,	0,	258,	8,	6,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/forwardKinematics/flangeCoordinates",	44979,	0,	258,	8,	6,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/toolFrame",	53909,	0,	0,	0,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/userFrame",	24293,	0,	0,	0,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/register",	80242,	0,	0,	0,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/line_id",	94084,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/startRun",	45710,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/startDebug",	33367,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/jumpLine",	8837,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/step",	99488,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/backward",	4276,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"io/system/dInUser3",	9523,	0,	2,	1,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"io/system/dInUser4",	9524,	0,	2,	1,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/fst/ControlCommand",	79844,	0,	1,	1,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/activeWarnings",	75299,	0,	8,	8,	10,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Config/IO/Info",	97967,	0,	0,	0,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Calculate/ForwordKinematics",	15923,	0,	0,	0,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Calculate/InverseKinematics",	43859,	0,	0,	0,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Params/system/localTime",	53925,	0,	5,	4,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Params/equipment/jointConstraint",	50980,	0,	0,	0,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Params/equipment/dh",	32184,	0,	0,	0,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Params/equipment/hardwareLimit",	57668,	0,	0,	0,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Params/manualCommand",	77076,	0,	0,	0,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/globalVelocity",	17993,	0,	258,	8,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/hostInTeachTarget",	11892,	0,	258,	8,	6,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Safety_IO/inputFrame",	94965,	0,	5,	4,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
};												
	*/
{												
	{	"root/Logic/curveMode",	32965,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/controllerWorkStatus",	31619,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/interpreterState",	48981,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/state",	40629,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/runningMode",	69397,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/servoState",	50277,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/userOperationMode",	36933,	0,	5,	4,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/stateCommand",	7076,	0,	5,	4,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/actualJointPositionsFiltered",	1284,	0,	258,	8,	6,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/forwardKinematics/toolCoordinates",	77427,	0,	258,	8,	6,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/forwardKinematics/flangeCoordinates",	44979,	0,	258,	8,	6,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/register",	80242,	0,	0,	0,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/line_id",	94084,	0,	514,	128,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/startRun",	45710,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/startDebug",	33367,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/jumpLine",	8837,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/step",	99488,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/MotionInterpreter/backward",	4276,	0,	5,	4,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"io/system/dInUser3",	9523,	0,	2,	1,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"io/system/dInUser4",	9524,	0,	2,	1,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/fst/ControlCommand",	79844,	0,	1,	1,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Logic/activeWarnings",	75299,	0,	8,	8,	10,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Config/IO/Info",	97967,	0,	0,	0,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Calculate/ForwordKinematics",	15923,	0,	0,	0,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Calculate/InverseKinematics",	43859,	0,	0,	0,	1,	BaseTypes_ParamType_INPUT_SIGNAL,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Params/system/localTime",	53925,	0,	5,	4,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Params/equipment/jointConstraint",	50980,	0,	1280,	1024,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Params/equipment/jointConstraintLimit",	35940,	0,	1280,	1024,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Params/equipment/dh",	32184,	0,	1280,	1024,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Params/equipment/hardwareLimit",	57668,	0,	1280,	1024,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Params/manualCommand",	77076,	0,	0,	0,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/globalVelocity",	17993,	0,	258,	8,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/hostInTeachTarget",	11892,	0,	258,	8,	6,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Safety_IO/inputFrame",	94965,	0,	5,	4,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/version",	10478,	0,	5,	8,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Register",	87442,	0,	7,	8,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/toolFrame",	53909,	0,	1280,	96,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/userFrame",	24293,	0,	1280,	96,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/activateToolFrame",	49429,	0,	5,	4,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/activateUserFrame",	67525,	0,	5,	4,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/userValidFrameIDList",	65444,	0,	1280,	363,	32,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/toolValidFrameIDList",	37812,	0,	1280,	363,	32,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Register/pose",	34069,	0,	1280,	188,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Register/number",	32882,	0,	1280,	54,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Test/string",	4855,	0,	1280,	128,	1,	BaseTypes_ParamType_OUTPUT_SIGNAL,	BaseTypes_Permission_R,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},
	{	"root/Control/globalAcceleration",	22462,	0,	1280,	9,	1,	BaseTypes_ParamType_PARAMETER,	BaseTypes_Permission_RW,	BaseTypes_UserLevel_user_level_undefined,	BaseTypes_Unit_unit_undefined,	},

};												


										
											

#endif //RCS_TP_PARAMS_H_


