/**
 * @file ctrl_func.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-9-11
 */
#ifndef CTRL_FUNC_H_
#define CTRL_FUNC_H_

#include "controller.h"


typedef struct _CtrlFunctions
{
    void (Controller::*setValue)(void* params, int len);
    void (Controller::*getValue)(void* params);
    void (Controller::*updateValue)(int id);
}CtrlFunctions;

/*
std::map<int, CtrlFunctions> g_ctrl_funcs_mp = 
{								
	{	32965,	{	&Controller::setError,	&Controller::getCurveMode,	&Controller::updateCurveMode	}	},
	{	31619,	{	&Controller::setError,	&Controller::getWorkStatus,	&Controller::updateWorkStatus	}	},
	{	48981,	{	&Controller::setError,	&Controller::getInterpreterState,	&Controller::updateInterpreterState	}	},
	{	40629,	{	&Controller::setError,	&Controller::getCtrlState,	&Controller::updateCtrlState	}	},
	{	36933,	{	&Controller::setUserOpMode,	&Controller::getError,	&Controller::updateDefault	}	},
	{	7076,	{	&Controller::setStateCmd,	&Controller::getError,	&Controller::updateDefault	}	},
	{	1284,	{	&Controller::setError,	&Controller::getCurJoints,	&Controller::updateCurJoints	}	},
	{	77427,	{	&Controller::setError,	&Controller::getTCPPose,	&Controller::updateTCPPose	}	},
	{	44979,	{	&Controller::setError,	&Controller::getFlangePose,	&Controller::updateFlangePose	}	},
	{	53909,	{	&Controller::setToolFrame,	&Controller::getToolFrame,	&Controller::updateDefault	}	},
	{	24293,	{	&Controller::setUserFrame,	&Controller::getUserFrame,	&Controller::updateDefault	}	},
	{	80242,	{	&Controller::setUserRegs,	&Controller::getUserRegs,	&Controller::updateUserRegs	}	},
	{	94084,	{	&Controller::setError,	&Controller::getLineID,	&Controller::updateLineID	}	},
	{	45710,	{	&Controller::startRun,	&Controller::getError,	&Controller::updateDefault	}	},
	{	33367,	{	&Controller::startDebug,	&Controller::getError,	&Controller::updateDefault	}	},
	{	8837,	{	&Controller::jumpLine,	&Controller::getError,	&Controller::updateDefault	}	},
	{	99488,	{	&Controller::step,	&Controller::getError,	&Controller::updateDefault	}	},
	{	4276,	{	&Controller::backward,	&Controller::getError,	&Controller::updateDefault	}	},
	{	9523,	{	&Controller::setError,	&Controller::getSafetyTPManual,	&Controller::updateDefault	}	},
	{	9524,	{	&Controller::setError,	&Controller::getSafetyTPAuto,	&Controller::updateDefault	}	},
	{	79844,	{	&Controller::setCtrlCmd,	&Controller::getError,	&Controller::updateDefault	}	},
	{	75299,	{	&Controller::setError,	&Controller::getWarnings,	&Controller::updateWarnings	}	},
	{	97967,	{	&Controller::setError,	&Controller::getIOInfo,	&Controller::updateDefault	}	},
	{	15923,	{	&Controller::setError,	&Controller::getFK,	&Controller::updateDefault	}	},
	{	43859,	{	&Controller::setError,	&Controller::getIK,	&Controller::updateDefault	}	},
	{	53925,	{	&Controller::setLocalTime,	&Controller::getLocalTime,	&Controller::updateDefault	}	},
	{	50980,	{	&Controller::setJointConstraint,	&Controller::getSoftLimit,	&Controller::updateDefault	}	},
	{	32184,	{	&Controller::setError,	&Controller::getDH,	&Controller::updateDefault	}	},
	{	57668,	{	&Controller::setError,	&Controller::getHardLimit,	&Controller::updateDefault	}	},
	{	77076,	{	&Controller::setManualCmd,	&Controller::getError,	&Controller::updateDefault	}	},
	{	17993,	{	&Controller::setGlobalVel,	&Controller::getGlobalVel,	&Controller::updateDefault	}	},
	{	11892,	{	&Controller::setTeachTarget,	&Controller::getError,	&Controller::updateDefault	}	},
	{	94965,	{	&Controller::setError,	&Controller::getSafetyInFrame,	&Controller::updateSafetyFrame	}	},
};	*/							


std::map<int, CtrlFunctions> g_ctrl_funcs_mp = 
{								
	{	32965,	{	&Controller::setError,	&Controller::getCurveMode,	&Controller::updateCurveMode	}	},
	{	31619,	{	&Controller::setError,	&Controller::getWorkStatus,	&Controller::updateWorkStatus	}	},
	{	48981,	{	&Controller::setError,	&Controller::getInterpreterState,	&Controller::updateInterpreterState	}	},
	{	40629,	{	&Controller::setError,	&Controller::getCtrlState,	&Controller::updateCtrlState	}	},
	{	69397,	{	&Controller::setError,	&Controller::getRunningMode,	&Controller::updateRunningMode	}	},
	{	50277,	{	&Controller::setError,	&Controller::getServoState,	&Controller::updateServoState	}	},
	{	36933,	{	&Controller::setUserOpMode,	&Controller::getUserOpMode,	&Controller::updateDefault	}	},
	{	7076,	{	&Controller::setStateCmd,	&Controller::getError,	&Controller::updateDefault	}	},
	{	1284,	{	&Controller::setError,	&Controller::getCurJoints,	&Controller::updateCurJoints	}	},
	{	77427,	{	&Controller::setError,	&Controller::getTCPPose,	&Controller::updateTCPPose	}	},
	{	44979,	{	&Controller::setError,	&Controller::getFlangePose,	&Controller::updateFlangePose	}	},
	{	53909,	{	&Controller::setToolFrame,	&Controller::getToolFrame,	&Controller::updateDefault	}	},
	{	94084,	{	&Controller::setError,	&Controller::getLineID,	&Controller::updateLineID	}	},
	{	45710,	{	&Controller::startRun,	&Controller::getError,	&Controller::updateDefault	}	},
	{	33367,	{	&Controller::startDebug,	&Controller::getError,	&Controller::updateDefault	}	},
	{	8837,	{	&Controller::jumpLine,	&Controller::getError,	&Controller::updateDefault	}	},
	{	99488,	{	&Controller::step,	&Controller::getError,	&Controller::updateDefault	}	},
	{	4276,	{	&Controller::backward,	&Controller::getError,	&Controller::updateDefault	}	},
	{	9523,	{	&Controller::setError,	&Controller::getSafetyTPManual,	&Controller::updateDefault	}	},
	{	9524,	{	&Controller::setError,	&Controller::getSafetyTPAuto,	&Controller::updateDefault	}	},
	{	79844,	{	&Controller::setCtrlCmd,	&Controller::getError,	&Controller::updateDefault	}	},
	{	75299,	{	&Controller::setError,	&Controller::getWarnings,	&Controller::updateWarnings	}	},
	{	97967,	{	&Controller::setError,	&Controller::getIOInfo,	&Controller::updateDefault	}	},
	{	15923,	{	&Controller::setError,	&Controller::getFK,	&Controller::updateDefault	}	},
	{	43859,	{	&Controller::setError,	&Controller::getIK,	&Controller::updateDefault	}	},
	{	53925,	{	&Controller::setLocalTime,	&Controller::getLocalTime,	&Controller::updateDefault	}	},
	{	50980,	{	&Controller::setJointConstraint,	&Controller::getSoftLimit,	&Controller::updateDefault	}	},
	{	35940,	{	&Controller::setError,	&Controller::getSoftConstraintLimit,	&Controller::updateDefault	}	},
	{	32184,	{	&Controller::setError,	&Controller::getDH,	&Controller::updateDefault	}	},
	{	57668,	{	&Controller::setError,	&Controller::getHardLimit,	&Controller::updateDefault	}	},
	{	77076,	{	&Controller::setManualCmd,	&Controller::getError,	&Controller::updateDefault	}	},
	{	17993,	{	&Controller::setGlobalVel,	&Controller::getGlobalVel,	&Controller::updateDefault	}	},
	{	11892,	{	&Controller::setTeachTarget,	&Controller::getError,	&Controller::updateDefault	}	},
	{	94965,	{	&Controller::setError,	&Controller::getSafetyInFrame,	&Controller::updateSafetyFrame	}	},
	{	10478,	{	&Controller::setError,	&Controller::getVersion,	&Controller::updateDefault	}	},
	{	87442,	{	&Controller::setRegister,	&Controller::getRegister,	&Controller::updateDefault	}	},
	{	53909,	{	&Controller::setToolFrame,	&Controller::getToolFrame,	&Controller::updateDefault	}	},
	{	24293,	{	&Controller::setUserFrame,	&Controller::getUserFrame,	&Controller::updateDefault	}	},
	{	49429,	{	&Controller::setActivateToolFrame,	&Controller::getActivateToolFrame,	&Controller::updateDefault	}	},
	{	67525,	{	&Controller::setActivateUserFrame,	&Controller::getActivateUserFrame,	&Controller::updateDefault	}	},						
	{	34069,	{	&Controller::setPoseRegister,	&Controller::getPoseRegister,	&Controller::updateDefault	}	},
	{	32882,	{	&Controller::setNumberRegister,	&Controller::getNumberRegister,	&Controller::updateDefault	}	},
	{	4855,	{	&Controller::setError,	&Controller::getString,	&Controller::updateString	}	},
	{	22462,	{	&Controller::setGlobalAcc,	&Controller::getGlobalAcc,	&Controller::updateDefault	}	},
		

};

							
#endif

