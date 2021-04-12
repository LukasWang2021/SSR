#include "Servo_Sm.h"

using namespace virtual_servo_device;

static void doServoSmStart(void* sm_ptr);
static void doServoSmNotReadyToSwitchOn(void* sm_ptr);
static void doServoSmSwitchOnDisabled(void* sm_ptr);
static void doServoSmReadyToSwitchOn(void* sm_ptr);
static void doServoSmSwitchedOn(void* sm_ptr);
static void doServoSmOperationEnabled(void* sm_ptr);
static void doServoSmQuickStopActive(void* sm_ptr);
static void doServoSmFaultReactionActive(void* sm_ptr);
static void doServoSmFault(void* sm_ptr);

static bool isCtrlShutDown(ServoCtrl_u ctrl_word)
{
    if(ctrl_word.bit.switch_on == 0
        && ctrl_word.bit.enable_voltage == 1
        && ctrl_word.bit.quick_stop == 1
        && ctrl_word.bit.fault_reset == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool isCtrlSwitchOn(ServoCtrl_u ctrl_word)
{
    if(ctrl_word.bit.switch_on == 1
        && ctrl_word.bit.enable_voltage == 1
        && ctrl_word.bit.quick_stop == 1
        && ctrl_word.bit.enable_operation == 0
        && ctrl_word.bit.fault_reset == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool isCtrlSwitchOnAndEnableOperation(ServoCtrl_u ctrl_word)
{
    if(ctrl_word.bit.switch_on == 1
        && ctrl_word.bit.enable_voltage == 1
        && ctrl_word.bit.quick_stop == 1
        && ctrl_word.bit.enable_operation == 1
        && ctrl_word.bit.fault_reset == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool isCtrlDisableVoltage(ServoCtrl_u ctrl_word)
{
    if(ctrl_word.bit.enable_voltage == 0
        && ctrl_word.bit.fault_reset == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool isCtrlQuickStop(ServoCtrl_u ctrl_word)
{
    if(ctrl_word.bit.enable_voltage == 1
        && ctrl_word.bit.quick_stop == 0
        && ctrl_word.bit.fault_reset == 0)
    {
        return true;
    }
    else
    {
        return false;
    }    
} 

static bool isCtrlDisableOperation(ServoCtrl_u ctrl_word)
{
    if(ctrl_word.bit.switch_on == 1
        && ctrl_word.bit.enable_voltage == 1
        && ctrl_word.bit.quick_stop == 1
        && ctrl_word.bit.enable_operation == 0
        && ctrl_word.bit.fault_reset == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool isCtrlEnableOperation(ServoCtrl_u ctrl_word)
{
    if(ctrl_word.bit.switch_on == 1
        && ctrl_word.bit.enable_voltage == 1
        && ctrl_word.bit.quick_stop == 1
        && ctrl_word.bit.enable_operation == 1
        && ctrl_word.bit.fault_reset == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool isCtrlFaultReset(ServoCtrl_u ctrl_word_current, ServoCtrl_u ctrl_word_back)
{
    if(ctrl_word_current.bit.fault_reset == 1 
        && ctrl_word_back.bit.fault_reset == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void virtual_servo_device::setCtrlShutDown(ServoCtrl_u* ctrl_word_ptr)
{
    ctrl_word_ptr->bit.switch_on = 0;
    ctrl_word_ptr->bit.enable_voltage = 1;
    ctrl_word_ptr->bit.quick_stop = 1;
    ctrl_word_ptr->bit.fault_reset = 0;
}

void virtual_servo_device::setCtrlSwitchOn(ServoCtrl_u* ctrl_word_ptr)
{
    ctrl_word_ptr->bit.switch_on = 1;
    ctrl_word_ptr->bit.enable_voltage = 1;
    ctrl_word_ptr->bit.quick_stop = 1;
    ctrl_word_ptr->bit.enable_operation = 0;
    ctrl_word_ptr->bit.fault_reset = 0;
}

void virtual_servo_device::setCtrlSwitchOnAndEnableOperation(ServoCtrl_u* ctrl_word_ptr)
{
    ctrl_word_ptr->bit.switch_on = 1;
    ctrl_word_ptr->bit.enable_voltage = 1;
    ctrl_word_ptr->bit.quick_stop = 1;
    ctrl_word_ptr->bit.enable_operation = 1;
    ctrl_word_ptr->bit.fault_reset = 0;
}

void virtual_servo_device::setCtrlDisableVoltage(ServoCtrl_u* ctrl_word_ptr)
{
    ctrl_word_ptr->bit.enable_voltage = 0;
    ctrl_word_ptr->bit.fault_reset = 0;
}

void virtual_servo_device::setCtrlQuickStop(ServoCtrl_u* ctrl_word_ptr)
{
    ctrl_word_ptr->bit.enable_voltage = 1;
    ctrl_word_ptr->bit.quick_stop = 0;
    ctrl_word_ptr->bit.fault_reset = 0;  
} 

void virtual_servo_device::setCtrlDisableOperation(ServoCtrl_u* ctrl_word_ptr)
{
    ctrl_word_ptr->bit.switch_on = 1;
    ctrl_word_ptr->bit.enable_voltage = 1;
    ctrl_word_ptr->bit.quick_stop = 1;
    ctrl_word_ptr->bit.enable_operation = 0;
    ctrl_word_ptr->bit.fault_reset = 0;
}

void virtual_servo_device::setCtrlEnableOperation(ServoCtrl_u* ctrl_word_ptr)
{
    ctrl_word_ptr->bit.switch_on = 1;
    ctrl_word_ptr->bit.enable_voltage = 1;
    ctrl_word_ptr->bit.quick_stop = 1;
    ctrl_word_ptr->bit.enable_operation = 1;
    ctrl_word_ptr->bit.fault_reset = 0;
}

void virtual_servo_device::setCtrlFaultReset(ServoCtrl_u* ctrl_word_ptr)
{
    ctrl_word_ptr->bit.fault_reset = 1; 
}

void virtual_servo_device::resetCtrlFaultReset(ServoCtrl_u* ctrl_word_ptr)
{
    ctrl_word_ptr->bit.fault_reset = 0; 
}

static void setStateNotReadyToSwitchOn(ServoState_u* state_word_ptr)
{
    state_word_ptr->all = state_word_ptr->all & 0xFFFFFFB0;   // X0XX 0000
}

static void setStateSwitchOnDisabled(ServoState_u* state_word_ptr)
{
    state_word_ptr->bit.ready_to_switch_on = 0;
    state_word_ptr->bit.switched_on = 0;
    state_word_ptr->bit.operation_enabled = 0;
    state_word_ptr->bit.fault = 0;
    state_word_ptr->bit.switch_on_disabled = 1;
}

static void setStateReadyToSwitchOn(ServoState_u* state_word_ptr)
{
    state_word_ptr->bit.ready_to_switch_on = 1;
    state_word_ptr->bit.switched_on = 0;
    state_word_ptr->bit.operation_enabled = 0;
    state_word_ptr->bit.fault = 0;
    state_word_ptr->bit.quick_stop = 1;
    state_word_ptr->bit.switch_on_disabled = 0;
}

static void setStateSwitchedOn(ServoState_u* state_word_ptr)
{
    state_word_ptr->bit.ready_to_switch_on = 1;
    state_word_ptr->bit.switched_on = 1;
    state_word_ptr->bit.operation_enabled = 0;
    state_word_ptr->bit.fault = 0;
    state_word_ptr->bit.quick_stop = 1;
    state_word_ptr->bit.switch_on_disabled = 0;
}

static void setStateOperationEnabled(ServoState_u* state_word_ptr)
{
    state_word_ptr->bit.ready_to_switch_on = 1;
    state_word_ptr->bit.switched_on = 1;
    state_word_ptr->bit.operation_enabled = 1;
    state_word_ptr->bit.fault = 0;
    state_word_ptr->bit.quick_stop = 1;
    state_word_ptr->bit.switch_on_disabled = 0;
}

static void setStateQuickStopActive(ServoState_u* state_word_ptr)
{
    state_word_ptr->bit.ready_to_switch_on = 1;
    state_word_ptr->bit.switched_on = 1;
    state_word_ptr->bit.operation_enabled = 1;
    state_word_ptr->bit.fault = 0;
    state_word_ptr->bit.quick_stop = 0;
    state_word_ptr->bit.switch_on_disabled = 0;
}

static void setStateFaultReactionActive(ServoState_u* state_word_ptr)
{
    state_word_ptr->bit.ready_to_switch_on = 1;
    state_word_ptr->bit.switched_on = 1;
    state_word_ptr->bit.operation_enabled = 1;
    state_word_ptr->bit.fault = 1;
    state_word_ptr->bit.switch_on_disabled = 0;
}
/*设置错误线*/
void virtual_servo_device::setStateFaultReactionLine(ServoSm_t* servo_sm_ptr)
{
	setStateFaultReactionActive(&servo_sm_ptr->state_word);
	servo_sm_ptr->run = &doServoSmFaultReactionActive;
}

static void setStateFault(ServoState_u* state_word_ptr)
{
    state_word_ptr->bit.ready_to_switch_on = 0;
    state_word_ptr->bit.switched_on = 0;
    state_word_ptr->bit.operation_enabled = 0;
    state_word_ptr->bit.fault = 1;
    state_word_ptr->bit.switch_on_disabled = 0;
}

void virtual_servo_device::initServoSm(ServoSm_t* servo_sm_ptr)
{
    servo_sm_ptr->ctrl_word.all = 0;
    servo_sm_ptr->ctrl_word_back.all = 0;
    servo_sm_ptr->state_word.all = 0;
    servo_sm_ptr->error_exist = false;
    servo_sm_ptr->pid_enabled = false;
    servo_sm_ptr->quick_stop_enabled = false;
    servo_sm_ptr->run = &doServoSmStart;
}

void doServoSmStart(void* sm_ptr)
{
    ServoSm_t* servo_sm_ptr = (ServoSm_t*)sm_ptr;
    setStateNotReadyToSwitchOn(&servo_sm_ptr->state_word);
    servo_sm_ptr->run = &doServoSmNotReadyToSwitchOn;
}

void doServoSmNotReadyToSwitchOn(void* sm_ptr)
{
    ServoSm_t* servo_sm_ptr = (ServoSm_t*)sm_ptr;
    setStateSwitchOnDisabled(&servo_sm_ptr->state_word);
    servo_sm_ptr->run = &doServoSmSwitchOnDisabled;
}

void doServoSmSwitchOnDisabled(void* sm_ptr)
{
    ServoSm_t* servo_sm_ptr = (ServoSm_t*)sm_ptr;

    if(isCtrlShutDown(servo_sm_ptr->ctrl_word))
    {
        setStateReadyToSwitchOn(&servo_sm_ptr->state_word);
        servo_sm_ptr->run = &doServoSmReadyToSwitchOn;
    }
}

void doServoSmReadyToSwitchOn(void* sm_ptr)
{
    ServoSm_t* servo_sm_ptr = (ServoSm_t*)sm_ptr;

    if(isCtrlSwitchOn(servo_sm_ptr->ctrl_word)
        || isCtrlSwitchOnAndEnableOperation(servo_sm_ptr->ctrl_word))
    {
        setStateSwitchedOn(&servo_sm_ptr->state_word);
        servo_sm_ptr->run = &doServoSmSwitchedOn;
    }
    else if(isCtrlDisableVoltage(servo_sm_ptr->ctrl_word)
        || isCtrlQuickStop(servo_sm_ptr->ctrl_word))
    {
        setStateSwitchOnDisabled(&servo_sm_ptr->state_word);
        servo_sm_ptr->run = &doServoSmSwitchOnDisabled;
    }
}

void doServoSmSwitchedOn(void* sm_ptr)
{
    ServoSm_t* servo_sm_ptr = (ServoSm_t*)sm_ptr;

    if(isCtrlEnableOperation(servo_sm_ptr->ctrl_word)
        || isCtrlSwitchOnAndEnableOperation(servo_sm_ptr->ctrl_word))
    {
        setStateOperationEnabled(&servo_sm_ptr->state_word);
        servo_sm_ptr->pid_enabled = true;
        servo_sm_ptr->run = &doServoSmOperationEnabled;
    }
    else if(isCtrlDisableVoltage(servo_sm_ptr->ctrl_word)
        || isCtrlQuickStop(servo_sm_ptr->ctrl_word)) 
    {
        setStateSwitchOnDisabled(&servo_sm_ptr->state_word);
        servo_sm_ptr->run = &doServoSmSwitchOnDisabled;
    }
    else if(isCtrlShutDown(servo_sm_ptr->ctrl_word))
    {
        setStateReadyToSwitchOn(&servo_sm_ptr->state_word);
        servo_sm_ptr->run = &doServoSmReadyToSwitchOn;
    }
}

void doServoSmOperationEnabled(void* sm_ptr)
{    
    ServoSm_t* servo_sm_ptr = (ServoSm_t*)sm_ptr;

    if(isCtrlQuickStop(servo_sm_ptr->ctrl_word))
    {
        setStateQuickStopActive(&servo_sm_ptr->state_word);
        servo_sm_ptr->quick_stop_enabled = true;
        servo_sm_ptr->run = &doServoSmQuickStopActive;
    }
    else if(isCtrlDisableOperation(servo_sm_ptr->ctrl_word))
    {
        setStateSwitchedOn(&servo_sm_ptr->state_word);
        servo_sm_ptr->pid_enabled = false;
        servo_sm_ptr->run = &doServoSmSwitchedOn;
    }
    else if(isCtrlShutDown(servo_sm_ptr->ctrl_word))
    {
        setStateReadyToSwitchOn(&servo_sm_ptr->state_word);
        servo_sm_ptr->pid_enabled = false;
        servo_sm_ptr->run = &doServoSmReadyToSwitchOn;
    }
    else if(isCtrlDisableVoltage(servo_sm_ptr->ctrl_word))
    {
        setStateSwitchOnDisabled(&servo_sm_ptr->state_word);
        servo_sm_ptr->pid_enabled = false;
        servo_sm_ptr->run = &doServoSmSwitchOnDisabled;
    }
}

void doServoSmQuickStopActive(void* sm_ptr)
{
    ServoSm_t* servo_sm_ptr = (ServoSm_t*)sm_ptr;

    if(isCtrlDisableVoltage(servo_sm_ptr->ctrl_word))
    {
        setStateSwitchOnDisabled(&servo_sm_ptr->state_word);
        servo_sm_ptr->quick_stop_enabled = false;
        servo_sm_ptr->pid_enabled = false;
        servo_sm_ptr->run = &doServoSmSwitchOnDisabled;
    }
    else if(isCtrlEnableOperation(servo_sm_ptr->ctrl_word))
    {
        setStateOperationEnabled(&servo_sm_ptr->state_word);
        servo_sm_ptr->quick_stop_enabled = false;
        servo_sm_ptr->run = &doServoSmOperationEnabled;
    }
}

void doServoSmFaultReactionActive(void* sm_ptr)
{
    ServoSm_t* servo_sm_ptr = (ServoSm_t*)sm_ptr;

    // error handling...

    setStateFault(&servo_sm_ptr->state_word);
    servo_sm_ptr->run = &doServoSmFault;
}

void doServoSmFault(void* sm_ptr)
{
    ServoSm_t* servo_sm_ptr = (ServoSm_t*)sm_ptr;

    if(isCtrlFaultReset(servo_sm_ptr->ctrl_word, servo_sm_ptr->ctrl_word_back))
    {
        setStateSwitchOnDisabled(&servo_sm_ptr->state_word);
        servo_sm_ptr->run = &doServoSmSwitchOnDisabled;
    }
}

void gotoServoSmFaultReactionActive(ServoSm_t* servo_sm_ptr)
{
    servo_sm_ptr->quick_stop_enabled = false;
    servo_sm_ptr->pid_enabled = false;
    setStateFaultReactionActive(&servo_sm_ptr->state_word);
    servo_sm_ptr->run = &doServoSmFaultReactionActive;
}


