#ifndef CONTROLLER_SM_H
#define CONTROLLER_SM_H

#include "controller_param.h"
#include "common_log.h"
#include "virtual_core1.h"
#include "common_enum.h"
#include "motion_control.h"
#include "process_comm.h"
#include "serverAlarmApi.h"
#include "fst_safety_device.h"
#include "modbus_manager.h"
#include "io_mapping.h"
#include "program_launching.h"
#include "interpreter_common.h"
#include <string>
#include <sys/time.h>
#include <mutex>

namespace fst_ctrl
{

//define move to fst_safety_device
//typedef enum
//{
//    USER_OP_MODE_NONE             = 0,
//    USER_OP_MODE_AUTO             = 1,
//    USER_OP_MODE_SLOWLY_MANUAL    = 2,
//    USER_OP_MODE_MANUAL = 3,
//}UserOpMode;

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

typedef enum
{
    UI_SERVO_ENABLE        = 1,
    UI_PAUSE_REQUEST       = 2,
    UI_RESET               = 3,
    UI_START               = 4,
    UI_ABORT_PROGRAM       = 5,
    UI_SELECTION_STROBE    = 6,
    UI_MPLCS_START         = 7,
    UI_PROGRAM_SELECTION_1 = 8,
    UI_PROGRAM_SELECTION_2 = 9,
    UI_PROGRAM_SELECTION_3 = 10,
    UI_PROGRAM_SELECTION_4 = 11,
    UI_PROGRAM_SELECTION_5 = 12,
}UICommand;

typedef enum
{
    UO_CMD_ENABLE              = 1,
    UO_PAUSED                  = 2,
    UO_FAULT                   = 3,
    UO_PROGRAM_RUNNING         = 4,
    UO_SERVO_STATUS            = 5,
    UO_SELECTION_CHECK_REQUEST = 6,
    UO_MPLCS_START_DONE        = 7,
    UO_PROGRAM_CONFIRM_1       = 8,
    UO_PROGRAM_CONFIRM_2       = 9,
    UO_PROGRAM_CONFIRM_3       = 10,
    UO_PROGRAM_CONFIRM_4       = 11,
    UO_PROGRAM_CONFIRM_5       = 12,
}UOCommand;


class ControllerSm
{
public:
    ControllerSm();
    ~ControllerSm();
    void init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, fst_mc::MotionControl* motion_control_ptr, 
                VirtualCore1* virtual_core1_ptr, fst_base::ControllerClient* controller_client_ptr, 
                fst_hal::DeviceManager* device_manager_ptr, fst_ctrl::IoMapping* io_mapping_ptr,
                ProgramLaunching* program_launching);
    ControllerParam* getParam();
    void processStateMachine();
    
    fst_hal::UserOpMode getUserOpMode();
    RunningState getRunningState();
    InterpreterState getInterpreterState();
    RobotState getRobotState();
    CtrlState getCtrlState();
    fst_mc::ServoState getServoState();
    int getSafetyAlarm();  

    ErrorCode setUserOpMode(fst_hal::UserOpMode mode);
    ErrorCode checkOffsetState();
    ErrorCode callEstop();
    ErrorCode callReset();
    ErrorCode callShutdown();
    void setSafetyStop(ErrorCode error_code);

    void transferRobotStateToTeaching();
    void transferRobotStateToRunning();
    bool updateContinuousManualMoveRpcTime();

    void getNewInstruction(Instruction* data_ptr);
    bool isNextInstructionNeeded();

    // for publish data
    fst_hal::UserOpMode* getUserOpModePtr();
    RunningState* getRunningStatePtr();
    InterpreterState* getInterpreterStatePtr();
    RobotState* getRobotStatePtr();
    CtrlState* getCtrlStatePtr();
    fst_mc::ServoState* getServoStatePtr();
    int* getSafetyAlarmPtr();  


    void setState(bool state);
    bool getState();  
    
private:
    fst_log::Logger* log_ptr_;
    ControllerParam* param_ptr_;
    fst_mc::MotionControl* motion_control_ptr_;
    VirtualCore1* virtual_core1_ptr_;
    fst_base::ControllerClient* controller_client_ptr_;
    fst_hal::DeviceManager* device_manager_ptr_;
    fst_hal::FstSafetyDevice* safety_device_ptr_;    
    fst_hal::ModbusManager* modbus_manager_ptr_; 
    fst_ctrl::IoMapping* io_mapping_ptr_;
    ProgramLaunching* program_launching_ptr_;

    // mode and status
    fst_hal::UserOpMode user_op_mode_;
    fst_hal::UserOpMode pre_user_op_mode_;
    RunningState running_state_;
    InterpreterState interpreter_state_;
    RobotState robot_state_;
    CtrlState ctrl_state_;
    fst_mc::ServoState servo_state_;
    int safety_alarm_;
    int ctrl_reset_count_;
    int robot_state_timeout_count_;
    bool valid_state_;
    int program_code_; //program launching code

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
    void processMacroLaunching();//to set the enable value of macro launching
    void processModbusClientList();
    void processUIUO();
    
    // setUO
    void setUoEnableOn(void);
    void setUoEnableOff(void);
    void setUoPausedOn(void);
    void setUoPausedOff(void);
    void setUoFaultOn(void);
    void setUoFaultOff(void);
    void setUoProgramRunOn(void);
    void setUoProgramRunOff(void);
    void setUoServoOn(void);
    void setUoServoOff(void);

    // UI check if there is falling/rising edge.
    bool isFallingEdgeStart(uint32_t user_port);
    bool isFallingEdgeAbort(uint32_t user_port);
    bool isRisingEdge(uint32_t user_port);
    // UI get and UO set program launching code.
    int getSetProgramCode();
    // getUI, setUO
    bool getUI(uint32_t user_port, bool &level);
    void setUO(uint32_t user_port, bool level);

    // manual rpc related
    long long computeTimeElapse(struct timeval &current_time, struct timeval &last_time);
    void handleContinuousManualRpcTimeout();
    
    // interpreter instruction
    void clearInstruction();

    // log service
    void recordLog(std::string log_str);
    void recordLog(ErrorCode error_code);
    void recordLog(ErrorCode error_code, std::string log_str);
};

}


#endif
