/*
 * Servo_Sm.h
 *
 *  Created on: 2019年9月10日
 *      Author: qichao.wang
 */

#ifndef SERVO_SM_H_
#define SERVO_SM_H_

#include "Servo_Common_Def.h"
#include "common/servo_interface.h"
#include "common/core_comm_servo_datatype.h"

namespace virtual_servo_device{

typedef struct
{
    ServoCtrl_u ctrl_word;
    ServoCtrl_u ctrl_word_back;
    ServoState_u state_word;
    bool error_exist;
    bool pid_enabled;
    bool quick_stop_enabled;
    runSm run;
}ServoSm_t;

void initServoSm(ServoSm_t* servo_sm_ptr);
void gotoServoSmFaultReactionActive(ServoSm_t* servo_sm_ptr);

void setCtrlShutDown(ServoCtrl_u* ctrl_word_ptr);
void setCtrlSwitchOn(ServoCtrl_u* ctrl_word_ptr);
void setCtrlSwitchOnAndEnableOperation(ServoCtrl_u* ctrl_word_ptr);
void setCtrlDisableVoltage(ServoCtrl_u* ctrl_word_ptr);
void setCtrlQuickStop(ServoCtrl_u* ctrl_word_ptr);
void setCtrlDisableOperation(ServoCtrl_u* ctrl_word_ptr);
void setCtrlEnableOperation(ServoCtrl_u* ctrl_word_ptr);
void setCtrlFaultReset(ServoCtrl_u* ctrl_word_ptr);
void resetCtrlFaultReset(ServoCtrl_u* ctrl_word_ptr);

void setStateFaultReactionLine(ServoSm_t* servo_sm_ptr);
}
#endif /* SERVO_SM_H_ */

