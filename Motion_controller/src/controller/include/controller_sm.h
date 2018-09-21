#ifndef CONTROLLER_SM_H
#define CONTROLLER_SM_H

#include "controller_param.h"
#include "common_log.h"
#include "virtual_core1.h"
#include "common_enum.h"
#include "motion_control.h"
#include "process_comm.h"
#include "base_datatype.h"
#include "serverAlarmApi.h"
#include "fst_safety_device.h"
#include "interpreter_common.h"
#include <string>
#include <sys/time.h>
#include <mutex>

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
}RunningState;

typedef enum
{
    INTERPRETER_IDLE      = 0,    
    INTERPRETER_EXECUTE   = 1,
    INTERPRETER_PAUSED    = 2,
    INTERPRETER_IDLE_TO_EXECUTE   = 101,
    INTERPRETER_EXECUTE_TO_PAUSE  = 102,
    INTERPRETER_PAUSE_TO_IDLE     = 103,
    INTERPRETER_PAUSE_TO_EXECUTE  = 104    
}InterpreterState;

typedef enum
{
    ROBOT_IDLE      = 0,
    ROBOT_RUNNING   = 1,
    ROBOT_TEACHING  = 2,
    ROBOT_IDLE_TO_RUNNING   = 101,
    ROBOT_IDLE_TO_TEACHING  = 102,
    ROBOT_RUNNING_TO_IDLE   = 103,
    ROBOT_TEACHING_TO_IDLE  = 104,
}RobotState;

typedef enum
{
    CTRL_INIT       = 0,
    CTRL_ENGAGED    = 1,
    CTRL_ESTOP      = 2,
    CTRL_TERMINATED = 3,
    CTRL_ANY_TO_ESTOP       = 101,
    CTRL_ESTOP_TO_ENGAGED   = 102,
    CTRL_ESTOP_TO_TERMINATE = 103,
}CtrlState;

class ControllerSm
{
public:
    ControllerSm();
    ~ControllerSm();
    void init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, fst_mc::MotionControl* motion_control_ptr, 
                VirtualCore1* virtual_core1_ptr, fst_base::ControllerClient* controller_client_ptr, 
                fst_hal::FstSafetyDevice* safety_device_ptr);
    ControllerParam* getParam();
    void processStateMachine();
    
    UserOpMode getUserOpMode();
    RunningState getRunningState();
    InterpreterState getInterpreterState();
    RobotState getRobotState();
    CtrlState getCtrlState();
    fst_mc::ServoState getServoState();
    int getSafetyAlarm();
    ErrorCode setUserOpMode(UserOpMode mode);
    ErrorCode callEstop();
    ErrorCode callReset();
    ErrorCode callShutdown();

    void transferRobotStateToTeaching();
    void transferRobotStateToRunning();
    bool updateContinuousManualMoveRpcTime();

    void getNewInstruction(Instruction* data_ptr);
    bool isNextInstructionNeeded();

    // for publish data
    UserOpMode* getUserOpModePtr();
    RunningState* getRunningStatePtr();
    InterpreterState* getInterpreterStatePtr();
    RobotState* getRobotStatePtr();
    CtrlState* getCtrlStatePtr();
    fst_mc::ServoState* getServoStatePtr();
    int* getSafetyAlarmPtr();    
    
private:
    fst_log::Logger* log_ptr_;
    ControllerParam* param_ptr_;
    fst_mc::MotionControl* motion_control_ptr_;
    VirtualCore1* virtual_core1_ptr_;
    fst_base::ControllerClient* controller_client_ptr_;
    fst_hal::FstSafetyDevice* safety_device_ptr_;

    // mode and status
    UserOpMode user_op_mode_;
    RunningState running_state_;
    InterpreterState interpreter_state_;
    RobotState robot_state_;
    CtrlState ctrl_state_;
    fst_mc::ServoState servo_state_;
    int safety_alarm_;
    int ctrl_reset_count_;

    // manual rpc related
    bool is_continuous_manual_move_timeout_;
    struct timeval last_continuous_manual_move_rpc_time_;    
    std::mutex manual_rpc_mutex_;

    // user op mode related
    struct timeval last_unknown_user_op_mode_time_;
    bool is_unknown_user_op_mode_exist_;

    // interpreter instruction
    Instruction instruction_;
    bool is_instruction_available_;

    // error flags
    long long int interpreter_warning_code_;
    int error_level_;
    bool is_error_exist_;
    
    // state machine transfer
    void processInterpreter();
    void processSafety();
    void processError();
    void transferServoState();
    void transferCtrlState();
    void transferRobotState();
    void shutdown();

    // manual rpc related
    long long computeTimeElapse(struct timeval &current_time, struct timeval &last_time);
    void handleContinuousManualRpcTimeout();

    // interpreter instruction
    void clearInstruction();

    // log service
    void recordLog(std::string log_str);
    void recordLog(ErrorCode error_code);
};

}


#endif
