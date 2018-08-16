#ifndef CONTROLLER_SM_H
#define CONTROLLER_SM_H

#include "controller_param.h"
#include "common_log.h"
#include "virtual_core1.h"
#include "common_enum.h"

namespace fst_ctrl
{
typedef enum
{
    USER_OP_MODE_NONE             = 0,
    USER_OP_MODE_AUTO             = 1,
    USER_OP_MODE_SLOWLY_MANUAL    = 2,
    USER_OP_MODE_UNLIMITED_MANUAL = 3,
}UserOpMode;

typedef enum
{
    RUNNING_STATUS_NORMAL    = 0, 
    RUNNING_STATUS_LIMITED   = 1,
}RunningStatus;

typedef enum
{
    INTERPRETER_IDLE      = 0,    
    INTERPRETER_EXECUTE   = 1,
    INTERPRETER_PAUSED    = 2,
    INTERPRETER_IDLE_TO_EXECUTE   = 101,
    INTERPRETER_EXECUTE_TO_PAUSE  = 102,
    INTERPRETER_PAUSE_TO_IDLE     = 103,
    INTERPRETER_PAUSE_TO_EXECUTE  = 104    
}InterpreterStatus;

typedef enum
{
    ROBOT_IDLE      = 0,
    ROBOT_RUNNING   = 1,
    ROBOT_TEACHING  = 2,
    ROBOT_IDLE_TO_RUNNING   = 101,
    ROBOT_IDLE_TO_TEACHING  = 102,
    ROBOT_RUNNING_TO_IDLE   = 103,
    ROBOT_TEACHING_TO_IDLE  = 104,
}RobotStatus;

typedef enum
{
    CTRL_INIT       = 0,
    CTRL_ENGAGED    = 1,
    CTRL_ESTOP      = 2,
    CTRL_TERMINATED = 3,
    CTRL_ANY_TO_ESTOP       = 101,
    CTRL_ESTOP_TO_ENGAGED   = 102,
    CTRL_ESTOP_TO_TERMINATE = 103,
}CtrlStatus;

class ControllerSm
{
public:
    ControllerSm();
    ~ControllerSm();
    void init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, VirtualCore1* virtual_core1_ptr);
    void processStateMachine();

    UserOpMode getUserOpMode();
    RunningStatus getRunningStatus();
    InterpreterStatus getInterpreterStatus();
    RobotStatus getRobotStatus();
    CtrlStatus getCtrlStatus();
    fst_mc::ServoStatus getServoStatus();
    int getSafetyAlarm();
    bool setUserOpMode(UserOpMode mode);
    bool callEstop();
    bool callReset();

    // for publish data
    UserOpMode* getUserOpModePtr();
    RunningStatus* getRunningStatusPtr();
    InterpreterStatus* getInterpreterStatusPtr();
    RobotStatus* getRobotStatusPtr();
    CtrlStatus* getCtrlStatusPtr();
    fst_mc::ServoStatus* getServoStatusPtr();
    int* getSafetyAlarmPtr();    
    
private:
    fst_log::Logger* log_ptr_;
    ControllerParam* param_ptr_;
    VirtualCore1* virtual_core1_ptr_;

    // mode and status
    UserOpMode user_op_mode_;
    RunningStatus running_status_;
    InterpreterStatus interpreter_status_;
    RobotStatus robot_status_;
    CtrlStatus ctrl_status_;
    fst_mc::ServoStatus servo_status_;
    int safety_alarm_;
    int ctrl_reset_count_;

    // error flags
    long long int interpreter_warning_code_;
    int error_level_;
    bool is_error_exist_;
    bool is_init_error_exist;
    
    // state machine transfer
    void processInterpreter();  
    void processError();
    void transferServoStatus();
    void transferCtrlStatus();
    void transferRobotStatus();      
};

}


#endif
