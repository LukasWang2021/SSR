#include "controller_sm.h"
#include "error_monitor.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string>
#include <sstream>
#include "basic_alg_datatype.h"
#include "basic_constants.h"


using namespace fst_ctrl;
using namespace fst_base;
using namespace fst_mc;
using namespace fst_hal;
using namespace basic_alg;

ControllerSm::ControllerSm():
    log_ptr_(NULL),
    param_ptr_(NULL),
    virtual_core1_ptr_(NULL),
    controller_client_ptr_(NULL),
    device_manager_ptr_(NULL),
    safety_device_ptr_(NULL),
    modbus_manager_ptr_(NULL),
    io_mapping_ptr_(NULL),
    program_launching_ptr_(NULL),
    user_op_mode_(USER_OP_MODE_AUTO),
    pre_user_op_mode_(USER_OP_MODE_AUTO),
    running_state_(RUNNING_STATUS_LIMITED),
    interpreter_state_(INTERPRETER_IDLE),
    robot_state_(ROBOT_IDLE),
    ctrl_state_(CTRL_INIT),
    servo_state_(SERVO_INIT),
    safety_alarm_(0),
    ctrl_reset_count_(0),
    robot_state_timeout_count_(100000),
    valid_state_(false),
    program_code_(0),
    interpreter_warning_code_(0),
    error_level_(0),
    is_error_exist_(false),
    is_continuous_manual_move_timeout_(false),
    is_unknown_user_op_mode_exist_(false),
    is_instruction_available_(false)
{
    memset(&last_continuous_manual_move_rpc_time_, 0, sizeof(struct timeval));
    memset(&last_unknown_user_op_mode_time_, 0, sizeof(struct timeval));
    memset(&instruction_, 0, sizeof(Instruction));
}

ControllerSm::~ControllerSm()
{

}

void ControllerSm::init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, fst_mc::MotionControl* motion_control_ptr, 
                            VirtualCore1* virtual_core1_ptr, ControllerClient* controller_client_ptr, 
                            fst_hal::DeviceManager* device_manager_ptr, fst_ctrl::IoMapping* io_mapping_ptr,
                            ProgramLaunching* program_launching_ptr)
{
    log_ptr_ = log_ptr;
    param_ptr_ = param_ptr;
    motion_control_ptr_ = motion_control_ptr;
    virtual_core1_ptr_ = virtual_core1_ptr;
    controller_client_ptr_ = controller_client_ptr;
    device_manager_ptr_ =device_manager_ptr;
    io_mapping_ptr_ = io_mapping_ptr;
    program_launching_ptr_ = program_launching_ptr;

    // get the safety device ptr.
    std::vector<fst_hal::DeviceInfo> device_list = device_manager_ptr_->getDeviceList();
    for(unsigned int i = 0; i < device_list.size(); ++i)
    {
        if (device_list[i].type == DEVICE_TYPE_FST_SAFETY)
        {
            BaseDevice* device_ptr = device_manager_ptr_->getDevicePtrByDeviceIndex(device_list[i].index);
            if (device_ptr != NULL || safety_device_ptr_ == NULL)
            {
                safety_device_ptr_ = static_cast<FstSafetyDevice*>(device_ptr);
            }  
        }
        else if (device_list[i].type == DEVICE_TYPE_MODBUS)
        {
            BaseDevice* device_ptr = device_manager_ptr_->getDevicePtrByDeviceIndex(device_list[i].index);
            if (device_ptr != NULL || safety_device_ptr_ == NULL)
            {
                modbus_manager_ptr_ = static_cast<ModbusManager*>(device_ptr);
            }     
        }
    }

    // init the timeout param of robot_state_
    robot_state_timeout_count_ = param_ptr_->robot_state_timeout_ / param_ptr_->routine_cycle_time_;
}

ControllerParam* ControllerSm::getParam()
{
    return param_ptr_;
}

void ControllerSm::processStateMachine()
{
    processInterpreter();
    processSafety();
    processError();
    transferServoState();
    transferCtrlState();
    transferRobotState();
    processMacroLaunching();
    processModbusClientList();
    processUIUO();
}

fst_hal::UserOpMode ControllerSm::getUserOpMode()
{
    return user_op_mode_;
}

RunningState ControllerSm::getRunningState()
{
    return running_state_;
}

fst_ctrl::InterpreterState ControllerSm::getInterpreterState()
{
    return interpreter_state_;
}

fst_ctrl::RobotState ControllerSm::getRobotState()
{
    return robot_state_;
}

fst_ctrl::CtrlState ControllerSm::getCtrlState()
{
    return ctrl_state_;
}

fst_mc::ServoState ControllerSm::getServoState()
{
    return servo_state_;
}

int ControllerSm::getSafetyAlarm()
{
    return safety_alarm_;
}

ErrorCode ControllerSm::setUserOpMode(fst_hal::UserOpMode mode)
{
    // the function should be called when Safety_Board is not existed and for TP_simulation
    if (safety_device_ptr_->isValid())
    {
        return CONTROLLER_INVALID_OPERATION;
    }

    if(mode == USER_OP_MODE_AUTO
        || mode == USER_OP_MODE_SLOWLY_MANUAL
        || mode == USER_OP_MODE_MANUAL)
    {
        user_op_mode_ = mode;
        return SUCCESS;
    }
    else
    {
        return CONTROLLER_INVALID_ARG;
    }
}

ErrorCode ControllerSm::checkOffsetState()
{
    CalibrateState calib_state;
    OffsetState offset_state[NUM_OF_JOINT];

    ErrorCode error_code = motion_control_ptr_->checkOffset(calib_state, offset_state);
    if (error_code != SUCCESS)
    {
        return error_code;
    }

    if(calib_state != MOTION_NORMAL)
    {
        for(int i=0; i<NUM_OF_JOINT; ++i)
        {
            if(offset_state[i] == OFFSET_LOST)
            {
                std::string log_str("offset lost: joint ");
                log_str.append(std::to_string(i+1));
                recordLog(ZERO_OFFSET_LOST, log_str);
            }
            else if(offset_state[i] == OFFSET_DEVIATE)
            {
                std::string log_str("offset deviate: joint ");
                log_str.append(std::to_string(i+1));
                recordLog(ZERO_OFFSET_DEVIATE, log_str);
            }
        }
    
        if(calib_state == MOTION_FORBIDDEN)
        {
            return CONTROLLER_OFFSET_NEED_CALIBRATE;
        }
    }
    
    return SUCCESS;
}

ErrorCode ControllerSm::callEstop()
{
    if(ctrl_state_ == CTRL_ENGAGED
        || ctrl_state_ == CTRL_ESTOP_TO_ENGAGED
        || ctrl_state_ == CTRL_INIT)
    {
        safety_device_ptr_->setDOType0Stop(1);
        controller_client_ptr_->abort();
        motion_control_ptr_->stopGroup();
        motion_control_ptr_->abortMove();
        recordLog("Controller transfer to ANY_TO_ESTOP by rpc-callEstop");
        ctrl_state_ = CTRL_ANY_TO_ESTOP;
        return SUCCESS;
    } 
    return CONTROLLER_INVALID_OPERATION;
}

ErrorCode ControllerSm::callReset()
{
    if(valid_state_ == false)
        return CONTROLLER_INVALID_OPERATION_RESET;

    if (ctrl_state_ == CTRL_ENGAGED)
    {
        ErrorMonitor::instance()->add(INFO_RESET_SUCCESS);
        return SUCCESS;
    }

    if(ctrl_state_ == CTRL_ESTOP
        || ctrl_state_ == CTRL_INIT)
    {
        is_unknown_user_op_mode_exist_ = false;
        ErrorMonitor::instance()->clear();
        clearInstruction();
        ErrorCode error_code = checkOffsetState();
        if(error_code != SUCCESS)
        {
            controller_client_ptr_->abort();
            motion_control_ptr_->stopGroup();
            motion_control_ptr_->abortMove();
            recordLog(error_code, "Controller transfer to ANY_TO_ESTOP for offset failure when reset");
            ctrl_state_ = CTRL_ANY_TO_ESTOP;
            return error_code;
        }

        //check safety_board status
        safety_device_ptr_->reset();
        if (safety_device_ptr_->checkSafetyBoardAlarm())
        {
            return CONTROLLER_SAFETY_NOT_READY;
        }
        error_code = safety_device_ptr_->checkDeadmanNormal();
        if (error_code != SUCCESS)
        {
            return error_code;
        }

        //check io_board
        std::vector<fst_hal::DeviceInfo> device_list = device_manager_ptr_->getDeviceList();
        for(unsigned int i = 0; i < device_list.size(); ++i)
        {
            if (device_list[i].type == DEVICE_TYPE_FST_IO)
            {
                BaseDevice* device_ptr = device_manager_ptr_->getDevicePtrByDeviceIndex(device_list[i].index);
                if (device_ptr == NULL) continue;
                FstIoDevice* io_device_ptr = static_cast<FstIoDevice*>(device_ptr);
                bool ret = io_device_ptr->isValid();
                if (ret != true)
                {
                    ErrorMonitor::instance()->add(IO_DEVICE_UNFOUND);
                    return IO_DEVICE_UNFOUND;
                }                
            }
        }

        recordLog("Controller transfer to ESTOP_TO_ENGAGED by rpc-callReset");
        ErrorMonitor::instance()->add(INFO_RESET_SUCCESS);
        ctrl_state_ = CTRL_ESTOP_TO_ENGAGED;  
        usleep(10000);//reset safety_board bit before sending RESET to bare_core.
        motion_control_ptr_->resetGroup();  
        ctrl_reset_count_ =  param_ptr_->reset_max_time_ / param_ptr_->routine_cycle_time_;  
        return SUCCESS;  
    }
    return CONTROLLER_INVALID_OPERATION_RESET;
}

ErrorCode ControllerSm::callShutdown()
{
    if(ctrl_state_ == CTRL_ESTOP)
    {
        ctrl_state_ = CTRL_ESTOP_TO_TERMINATE;
        return SUCCESS;
    }
    else
    {
        return CONTROLLER_INVALID_OPERATION;
    }
}

void ControllerSm::transferRobotStateToTeaching()
{
    if(robot_state_ == ROBOT_IDLE)
    {
        recordLog("Robot transfer from IDLE to IDLE_TO_TEACHING");
        robot_state_ = ROBOT_IDLE_TO_TEACHING;
    }
}

void ControllerSm::transferRobotStateToRunning()
{
    if(robot_state_ == ROBOT_IDLE)
    {
        recordLog("Robot transfer from IDLE to IDLE_TO_RUNNING");
        robot_state_ = ROBOT_IDLE_TO_RUNNING;
    }
}

bool ControllerSm::updateContinuousManualMoveRpcTime()
{
    manual_rpc_mutex_.lock();
    if(is_continuous_manual_move_timeout_)
    {
        manual_rpc_mutex_.unlock();
        return false;
    }
    
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    last_continuous_manual_move_rpc_time_.tv_sec = current_time.tv_sec;
    last_continuous_manual_move_rpc_time_.tv_usec = current_time.tv_usec;
    manual_rpc_mutex_.unlock();
    return true;
}

void ControllerSm::getNewInstruction(Instruction* data_ptr)
{
    if(data_ptr != NULL)
    {
        memcpy(&instruction_, data_ptr, sizeof(Instruction));
        is_instruction_available_ = true;
    }
}

bool ControllerSm::isNextInstructionNeeded()
{
    if(!is_instruction_available_)
    {
        if(instruction_.type == MOTION)
        {
            return motion_control_ptr_->nextMovePermitted();
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

fst_hal::UserOpMode* ControllerSm::getUserOpModePtr()
{
    return &user_op_mode_;
}

fst_ctrl::RunningState* ControllerSm::getRunningStatePtr()
{
    return &running_state_;
}

fst_ctrl::InterpreterState* ControllerSm::getInterpreterStatePtr()
{
    return &interpreter_state_;
}

fst_ctrl::RobotState* ControllerSm::getRobotStatePtr()
{
    return &robot_state_;
}

fst_ctrl::CtrlState* ControllerSm::getCtrlStatePtr()
{
    return &ctrl_state_;
}

fst_mc::ServoState* ControllerSm::getServoStatePtr()
{
    return &servo_state_;
}

int* ControllerSm::getSafetyAlarmPtr()
{
    return &safety_alarm_;
}


void ControllerSm::processInterpreter()
{
    interpreter_state_ = (fst_ctrl::InterpreterState)controller_client_ptr_->getInterpreterPublishPtr()->status;

    if(interpreter_state_ == INTERPRETER_EXECUTE)
    {
        if(is_instruction_available_)
        {
            ErrorCode error_code = SUCCESS;
            switch(instruction_.type)
            {
                case MOTION:
                {//FST_ERROR("---Instruction Motion");
                    error_code = motion_control_ptr_->autoMove(0, instruction_.target);
                    break;
                }
                case SET_UF:
                {//FST_ERROR("---Instruction SetUF");
                    error_code = motion_control_ptr_->setUserFrame(instruction_.current_uf);
                    break;
                }
                case SET_TF:
                {//FST_ERROR("---Instruction SetTf");
                    error_code = motion_control_ptr_->setToolFrame(instruction_.current_tf);
                    break;
                }
                case SET_OVC:
                {//FST_ERROR("---Instruction SetOvc");
                    if(user_op_mode_ == USER_OP_MODE_SLOWLY_MANUAL
                        && instruction_.current_ovc > param_ptr_->max_limited_global_vel_ratio_)
                    {
                        error_code = motion_control_ptr_->setGlobalVelRatio(param_ptr_->max_limited_global_vel_ratio_);
                    }
                    else
                    {
                        error_code = motion_control_ptr_->setGlobalVelRatio(instruction_.current_ovc);
                    }
                    break;
                }
                case SET_OAC:
                {//FST_ERROR("---Instruction SetOac");
                    if(user_op_mode_ == USER_OP_MODE_SLOWLY_MANUAL
                        && instruction_.current_oac > param_ptr_->max_limited_global_acc_ratio_)
                    {
                        error_code = motion_control_ptr_->setGlobalAccRatio(param_ptr_->max_limited_global_acc_ratio_);
                    }
                    else
                    {
                        error_code = motion_control_ptr_->setGlobalAccRatio(instruction_.current_oac);
                    }
                    break;
                }
                default:
                    ;
            }
            
            if(error_code != SUCCESS)
            {
                ErrorMonitor::instance()->add(error_code);
            }
            is_instruction_available_ = false;
        }
    }    
}

void ControllerSm::processSafety()
{
    // safety signal process 
    if(safety_device_ptr_->isValid())
    {
        struct timeval current_time;
        gettimeofday(&current_time, NULL);
        user_op_mode_ = safety_device_ptr_->getDITPUserMode();
        if(!is_unknown_user_op_mode_exist_
            && user_op_mode_ == USER_OP_MODE_NONE
            && last_unknown_user_op_mode_time_.tv_sec != 0)
        {
            long long time_elapse = computeTimeElapse(current_time, last_unknown_user_op_mode_time_);
            if(time_elapse >= param_ptr_->max_unknown_user_op_mode_timeout_)
            {
                is_unknown_user_op_mode_exist_ = true;
                ErrorMonitor::instance()->add(CONTROLLER_UNKNOWN_USER_OP_MODE);
            }
        }
        else
        {
            last_unknown_user_op_mode_time_.tv_sec = current_time.tv_sec;
            last_unknown_user_op_mode_time_.tv_usec = current_time.tv_usec;
        }
        
        //check safety_board status
        safety_device_ptr_->checkSafetyBoardAlarm();
        //check deadman normal and key switch under engage.
        if (ctrl_state_ == CTRL_ENGAGED)
        {
            if (pre_user_op_mode_ != user_op_mode_)
            {
                ErrorMonitor::instance()->add(SAFETY_BOARD_KEY_SWITCH_UNDER_ENGAGED); 
            }
            ErrorCode result = safety_device_ptr_->checkDeadmanNormal();
            if (result != SUCCESS && pre_user_op_mode_ == user_op_mode_)
            {
                ErrorMonitor::instance()->add(result);
            }
        }
        pre_user_op_mode_ = user_op_mode_;

        //get the safety_board alarm for TP
        safety_alarm_ = safety_device_ptr_->getExcitorStop();
        //get the cabinet reset
        if (safety_device_ptr_->isCabinetResetRequest())
        {
            if (callReset() == SUCCESS)
            {
                ErrorMonitor::instance()->add(INFO_RESET_SUCCESS);//info tp to clear its errors.
            }
            ErrorMonitor::instance()->add(SAFETY_BOARD_CABINET_RESET);
        }

        //DO output according to the user mode.
        uint32_t port_offset = 0;
        uint8_t value = 0;
        bool ret = safety_device_ptr_->getAutoModeDo(port_offset, value);
        if (ret == true)
        {
            io_mapping_ptr_->setDOByBit(port_offset, value);
        }
        ret = safety_device_ptr_->getLimitedManualModeDo(port_offset, value);
        if (ret == true)
        {
            io_mapping_ptr_->setDOByBit(port_offset, value);
        }
        ret = safety_device_ptr_->getManualModeDo(port_offset, value);
        if (ret == true)
        {
            io_mapping_ptr_->setDOByBit(port_offset, value);
        }

    }
    else
    {
        safety_alarm_ = 0;
    }

    // business logic process
    if(user_op_mode_ == USER_OP_MODE_SLOWLY_MANUAL)
    {
        ErrorCode error_code;
        if(motion_control_ptr_->getGlobalVelRatio() > param_ptr_->max_limited_global_vel_ratio_)
        {
            error_code = motion_control_ptr_->setGlobalVelRatio(param_ptr_->max_limited_global_vel_ratio_);
            if(error_code != SUCCESS)
            {
                ErrorMonitor::instance()->add(error_code);
            }
        }
        if(motion_control_ptr_->getGlobalAccRatio() > param_ptr_->max_limited_global_acc_ratio_)
        {
            error_code = motion_control_ptr_->setGlobalAccRatio(param_ptr_->max_limited_global_acc_ratio_);
            if(error_code != SUCCESS)
            {
                ErrorMonitor::instance()->add(error_code);
            }
        }            
    }    
}

void ControllerSm::processError()
{
    error_level_ = ErrorMonitor::instance()->getWarningLevel();
    if(error_level_ >= 4)
    {
        is_error_exist_ = true;
    }
    else
    {
        is_error_exist_ = false;
    }

    ErrorCode error_code;
    while(ErrorMonitor::instance()->pop(error_code))
    {
        recordLog(error_code);
    
        //STOP process: write bit to the safety_board.
        if (ErrorMonitor::instance()->isCore0Error(error_code))
        {
            int level = ErrorMonitor::instance()->getErrorLevel(error_code);
            // define error type to stop(0,1,2)
            if (level == 7 || level == 10 || level == 11)
            {
                safety_device_ptr_->setDOType0Stop(1);
            } 
            else if (level >= 4)
            {
                safety_device_ptr_->setDOType1Stop(1);
            }
            else
            {
            }
        }

    }
}

void ControllerSm::transferServoState()
{
    servo_state_ = motion_control_ptr_->getServoState();
    //servo_state_ = (ServoState)virtual_core1_ptr_->getServoState();
    if(ctrl_state_ == CTRL_ENGAGED
        && servo_state_ != SERVO_IDLE
        && servo_state_ != SERVO_RUNNING)
    {
        // this is ugly, because of lacking status in ctrl status definition
        if(robot_state_ == ROBOT_TEACHING
            || robot_state_ == ROBOT_IDLE_TO_TEACHING
            || robot_state_ == ROBOT_TEACHING_TO_IDLE)
        {
            motion_control_ptr_->clearGroup();
        }
        //RobotCtrlCmd cmd = ABORT_CMD;
        //XXsetCtrlCmd(&cmd, 0);
        controller_client_ptr_->abort();
        motion_control_ptr_->stopGroup();
        motion_control_ptr_->abortMove();
        recordLog("Servo state is abnormal");
        ctrl_state_ = CTRL_ANY_TO_ESTOP;
    }      
}

void ControllerSm::transferCtrlState()
{
    switch(ctrl_state_)
    {
        case CTRL_ANY_TO_ESTOP:
            if(robot_state_ == ROBOT_IDLE
                && servo_state_ != SERVO_RUNNING)
            {
                //motion_control_ptr_->saveJoint();
                recordLog("Controller transfer from ANY_TO_ESTOP to ESTOP");
                ctrl_state_ = CTRL_ESTOP;
            }
            break;
        case CTRL_ESTOP_TO_ENGAGED:
            if(robot_state_ == ROBOT_IDLE
                && servo_state_ == SERVO_IDLE
                && !is_error_exist_)
            {
                recordLog("Controller transfer from ESTOP_TO_ENGAGED to ENGAGED");
                ctrl_state_ = CTRL_ENGAGED;
            }
            else if((--ctrl_reset_count_) < 0)
            {
                recordLog("Controller transfer from ESTOP_TO_ENGAGED to ESTOP for reset timeout");
                ctrl_state_ = CTRL_ESTOP;
            }
            break;
        case CTRL_ESTOP_TO_TERMINATE:
            if(robot_state_ == ROBOT_IDLE)
            {
                //motion_control_ptr_->saveJoint();
                recordLog("Controller transfer from ESTOP_TO_TERMINATE to TERMINATED");
                ctrl_state_ = CTRL_TERMINATED;
                shutdown();
            }
            break;
        case CTRL_ENGAGED:
            if(is_error_exist_)
            {
                controller_client_ptr_->abort();
                motion_control_ptr_->stopGroup();
                motion_control_ptr_->abortMove();
                recordLog("Controller transfer from ENGAGED to CTRL_ANY_TO_ESTOP");
                ctrl_state_ = CTRL_ANY_TO_ESTOP;
            }
            break;
        case CTRL_INIT:
            ctrl_state_ = CTRL_ESTOP;
            break;
        default:
            ;
    }
}

void ControllerSm::transferRobotState()
{    
    switch(robot_state_)
    {
        case ROBOT_IDLE_TO_RUNNING:
            if(is_error_exist_)
            {
                recordLog("Robot transfer from IDLE_TO_RUNNING to RUNNING_TO_IDLE for error");
                robot_state_ = ROBOT_RUNNING_TO_IDLE;
            }
            else if(interpreter_state_ == INTERPRETER_EXECUTE)
            {
                recordLog("Robot transfer from IDLE_TO_RUNNING to RUNNING");
                robot_state_ = ROBOT_RUNNING;
            }
            else if((--robot_state_timeout_count_) < 0)
            {
                recordLog("Robot transfer from IDLE_TO_RUNNING to RUNNING_TO_IDLE for timeout");
                robot_state_ = ROBOT_RUNNING_TO_IDLE;
            }
            break;
        case ROBOT_IDLE_TO_TEACHING:
            recordLog("Robot transfer from IDLE_TO_TEACHING to TEACHING");
            robot_state_ = ROBOT_TEACHING;
            break;
        case ROBOT_RUNNING_TO_IDLE:
            if(interpreter_state_ != INTERPRETER_EXECUTE
                && servo_state_ != SERVO_RUNNING)
            {
                robot_state_timeout_count_ = param_ptr_->robot_state_timeout_ / param_ptr_->routine_cycle_time_;
                recordLog("Robot transfer from RUNNING_TO_IDLE to IDLE");
                robot_state_ = ROBOT_IDLE;
            }
            break;
        case ROBOT_TEACHING_TO_IDLE:
            if(servo_state_ != SERVO_RUNNING)
            {
                is_continuous_manual_move_timeout_ = false;
                memset(&last_continuous_manual_move_rpc_time_, 0, sizeof(struct timeval));
                robot_state_timeout_count_ = param_ptr_->robot_state_timeout_ / param_ptr_->routine_cycle_time_;
                recordLog("Robot transfer from TEACHING_TO_IDLE to IDLE");
                robot_state_ = ROBOT_IDLE;
            }
            break;
        case ROBOT_RUNNING:
            if(interpreter_state_ != INTERPRETER_EXECUTE)
            {
                recordLog("Robot transfer from RUNNING to RUNNING_TO_IDLE");
                robot_state_ = ROBOT_RUNNING_TO_IDLE;
            }
            break;
        case ROBOT_TEACHING:
            {
                handleContinuousManualRpcTimeout();
                fst_mc::GroupState group_state = motion_control_ptr_->getGroupState();
                if (group_state == fst_mc::STANDBY || group_state == fst_mc::DISABLE)
                //if(virtual_core1_ptr_->getArmState() == 1)
                {
                    recordLog("Robot transfer from TEACHING to TEACHING_TO_IDLE");
                    robot_state_ = ROBOT_TEACHING_TO_IDLE;
                }
                break;
            }
        default:
            ;
    }
}

void ControllerSm::shutdown()
{
	// flush the cache
	// int flushshutdown;
	// flushshutdown = fsync();

	// sent a message outside to shutdown
	int fdshutdown;
	fdshutdown = open("/dev/mem", O_RDWR);
	if (fdshutdown == -1)
		printf("The shutdown-message cann't be sent. fd = %d\n", fdshutdown);
	enum msgshutdown {
		SHUTDOWN_BASE = 0xff300000,
		SHUTDOWN_OFFSET = 0x0020,
		SHUTDOWN_LEN = 0x1000,
	};
	void *ptrshutdown;
	ptrshutdown = mmap(NULL, SHUTDOWN_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fdshutdown, SHUTDOWN_BASE);
	if (ptrshutdown == MAP_FAILED)
	{
		printf("The shutdown-message cann't be sent. mmap = %d\n", (void *)ptrshutdown);
	}
	else
	{
		int *pshutdown;
		pshutdown = (int *)((uint8_t*)ptrshutdown + 0x0020);
		*pshutdown = 1;
	}

	system("shutdown -h now");
}

void ControllerSm::processMacroLaunching()
{
    int enable_macro_launching = false;
    if (user_op_mode_ == USER_OP_MODE_AUTO
        && interpreter_state_ == INTERPRETER_IDLE
        && ctrl_state_ == CTRL_ENGAGED
        && robot_state_ == ROBOT_IDLE)
    {
        if(program_launching_ptr_->processMacro())
        {
            transferRobotStateToRunning();
        }
    }
    else
    {
        return;
    }
}

long long ControllerSm::computeTimeElapse(struct timeval &current_time, struct timeval &last_time)
{
    long long delta_tv_sec = current_time.tv_sec - last_time.tv_sec;
    long long delta_tv_usec = current_time.tv_usec - last_time.tv_usec;
    return (delta_tv_sec * 1000000 + delta_tv_usec);
}

void ControllerSm::handleContinuousManualRpcTimeout()
{
    if(is_continuous_manual_move_timeout_)
    {
        return;
    }

    manual_rpc_mutex_.lock();
    if(last_continuous_manual_move_rpc_time_.tv_sec == 0
        && last_continuous_manual_move_rpc_time_.tv_usec == 0)
    {
        manual_rpc_mutex_.unlock();
        return;
    }
    
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    long long time_elapse = computeTimeElapse(current_time, last_continuous_manual_move_rpc_time_);
    manual_rpc_mutex_.unlock();
    if(time_elapse > param_ptr_->max_continuous_manual_move_timeout_)
    {
        is_continuous_manual_move_timeout_ = true;
        GroupDirection direction;
        direction.axis1 = fst_mc::STANDING;
        direction.axis2 = fst_mc::STANDING;
        direction.axis3 = fst_mc::STANDING;
        direction.axis4 = fst_mc::STANDING;
        direction.axis5 = fst_mc::STANDING;
        direction.axis6 = fst_mc::STANDING;
        direction.axis7 = fst_mc::STANDING;
        direction.axis8 = fst_mc::STANDING;
        direction.axis9 = fst_mc::STANDING;        
        ErrorCode error_code = motion_control_ptr_->doContinuousManualMove(direction);
        if(error_code != SUCCESS)
        {
            ErrorMonitor::instance()->add(error_code);
        }
    }
}

void ControllerSm::processModbusClientList()
{
    ErrorCode error_code = modbus_manager_ptr_->scanAllClientDataArea();
    if (error_code != SUCCESS)
    {
        ErrorMonitor::instance()->add(error_code);
    }
}

void ControllerSm::clearInstruction()
{
    is_instruction_available_ = false;
    memset(&instruction_, 0, sizeof(Instruction));
}

void ControllerSm::recordLog(std::string log_str)
{
    std::stringstream stream;
    stream<<"Log_Code: 0x"<<std::hex<<CONTROLLER_LOG<<" : "<<log_str;
    FST_INFO(stream.str().c_str());

    ServerAlarmApi::GetInstance()->sendOneAlarm(CONTROLLER_LOG, log_str);
}

void ControllerSm::recordLog(ErrorCode error_code)
{
    std::stringstream stream;
    stream<<"Log_Code: 0x"<<std::hex<<error_code;
    if (error_code != INFO_RESET_SUCCESS)
        FST_ERROR(stream.str().c_str());

    ServerAlarmApi::GetInstance()->sendOneAlarm(error_code);
}

void ControllerSm::recordLog(ErrorCode error_code, std::string log_str)
{
    std::stringstream stream;
    stream<<"Log_Code: 0x"<<std::hex<<error_code<<" : "<<log_str;
    FST_ERROR(stream.str().c_str());

    ServerAlarmApi::GetInstance()->sendOneAlarm(error_code, log_str);
}

void ControllerSm::setState(bool state)
{
    valid_state_ = state;
}

bool ControllerSm::getState()
{
    return valid_state_;
} 


