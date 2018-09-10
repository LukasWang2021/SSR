#include "controller_sm.h"
#include "error_monitor.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>


using namespace fst_ctrl;
using namespace fst_base;
using namespace fst_mc;

ControllerSm::ControllerSm():
    log_ptr_(NULL),
    param_ptr_(NULL),
    virtual_core1_ptr_(NULL),
    controller_client_ptr_(NULL),
    user_op_mode_(USER_OP_MODE_AUTO),
    running_state_(RUNNING_STATUS_LIMITED),
    interpreter_state_(INTERPRETER_IDLE),
    robot_state_(ROBOT_IDLE),
    ctrl_state_(CTRL_INIT),
    servo_state_(SERVO_INIT),
    safety_alarm_(0),
    ctrl_reset_count_(0),
    interpreter_warning_code_(0),
    error_level_(0),
    is_error_exist_(false),
    is_init_error_exist(false)
{

}

ControllerSm::~ControllerSm()
{

}

void ControllerSm::init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, fst_mc::MotionControl* motion_control_ptr, 
                            VirtualCore1* virtual_core1_ptr, ControllerClient* controller_client_ptr)
{
    log_ptr_ = log_ptr;
    param_ptr_ = param_ptr;
    motion_control_ptr_ = motion_control_ptr;
    virtual_core1_ptr_ = virtual_core1_ptr;
    controller_client_ptr_ = controller_client_ptr;
}

void ControllerSm::processStateMachine()
{
    processInterpreter();
    processError();
    transferServoState();
    transferCtrlState();
    transferRobotState();
}

fst_ctrl::UserOpMode ControllerSm::getUserOpMode()
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

ErrorCode ControllerSm::setUserOpMode(fst_ctrl::UserOpMode mode)
{
    if(mode == USER_OP_MODE_AUTO
        || mode == USER_OP_MODE_SLOWLY_MANUAL
        || mode == USER_OP_MODE_UNLIMITED_MANUAL)
    {
        user_op_mode_ = mode;
        return SUCCESS;
    }
    else
    {
        return CONTROLLER_INVALID_ARG;
    }
}

ErrorCode ControllerSm::callEstop()
{
    if(ctrl_state_ == CTRL_ENGAGED
        || ctrl_state_ == CTRL_ESTOP_TO_ENGAGED
        || ctrl_state_ == CTRL_INIT)
    {
        motion_control_ptr_->stopGroup();
        //RobotCtrlCmd cmd = ABORT_CMD;
        //XXsetCtrlCmd(&cmd, 0);
        FST_INFO("---callEstop: ctrl_state-->CTRL_ANY_TO_ESTOP");
        ctrl_state_ = CTRL_ANY_TO_ESTOP;
    } 
    return SUCCESS;
}

ErrorCode ControllerSm::callReset()
{
    if(ctrl_state_ == CTRL_ESTOP
        || ctrl_state_ == CTRL_INIT)
    {
        ErrorMonitor::instance()->clear();
        motion_control_ptr_->resetGroup();

        //safety_interface_.reset();
        ctrl_reset_count_ =  param_ptr_->reset_max_time_ / param_ptr_->routine_cycle_time_;
        FST_INFO("---callReset: ctrl_state-->CTRL_ESTOP_TO_ENGAGED");
        ctrl_state_ = CTRL_ESTOP_TO_ENGAGED;
    }
    return SUCCESS;
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

fst_ctrl::UserOpMode* ControllerSm::getUserOpModePtr()
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
    interpreter_state_ = controller_client_ptr_->getInterpreterPublishPtr()->status;
#if 0
    U64 result = SUCCESS; 
    Instruction inst;

    xx_intrp_status_ = ShareMem::instance()->getIntprtState();
    if (xx_intrp_status_ == EXECUTE_R)
    {
        if (ShareMem::instance()->getInstruction(inst))
        {
            if(strlen(inst.line) > 0)
            {
                result = auto_motion_->moveTarget(inst.target);
                if(result != SUCCESS)
                {
                    rcs::Error::instance()->add(result);
                    RobotStateCmd cmd = EMERGENCY_STOP_E;
                    XXsetStateCmd(&cmd, 0);
                    
                    InterpreterControl ctrl;
                    ctrl.cmd = ABORT;
                    ShareMem::instance()->intprtControl(ctrl);
                    FST_INFO("---XXprocessInterp: moveTarget Failed: intrp_status-->ABORT");
                }
                else
                {
                    current_cnt_ = inst.target.cnt;
                    calcMotionDst(inst.target);
                }
            }
        }

        xx_intrp_warn_ = ShareMem::instance()->getWarning();
        if (xx_intrp_warn_ >= FAIL_INTERPRETER_BASE)
        {
            rcs::Error::instance()->add(xx_intrp_warn_);
            ShareMem::instance()->setWarning(0);
            FST_INFO("---XXprocessInterp: report intrp error code = %ld", xx_intrp_warn_);
        }
    }

    double left_time =  arm_group_->timeBeforeDeadline();
    if(left_time <= 0.001)
    {
        if(current_cnt_ < 0 
            && xx_servo_status_ != SERVO_STATE_READY)
        {
            return;
        }

        if(!ShareMem::instance()->getIntprtSendFlag())
        {
            ShareMem::instance()->sendingPermitted();
            ShareMem::instance()->setIntprtSendFlag(true);
        }
    }
#endif    
}

void ControllerSm::processError()
{
    error_level_ = ErrorMonitor::instance()->getWarningLevel();
    is_init_error_exist = ErrorMonitor::instance()->isInitError();
    //safety_status_ = safety_interface_.getDIAlarm();
    safety_alarm_ = virtual_core1_ptr_->getSafetyAlarm();
    if(error_level_ > 4
        || is_init_error_exist
        || safety_alarm_ != 0)
    {
        is_error_exist_ = true;
    }
    else
    {
        is_error_exist_ = false;
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
        recordLog("Servo state is abnormal");
        ctrl_state_ = CTRL_ANY_TO_ESTOP;
    }      
}

void ControllerSm::transferCtrlState()
{
    switch(ctrl_state_)
    {
        case CTRL_ANY_TO_ESTOP:
            if(robot_state_ == ROBOT_IDLE)
            {
                motion_control_ptr_->saveJoint();
                recordLog("Controller transfer to ESTOP");
                ctrl_state_ = CTRL_ESTOP;
            }
            break;
        case CTRL_ESTOP_TO_ENGAGED:
            if(robot_state_ == ROBOT_IDLE
                && servo_state_ == SERVO_IDLE
                && !is_error_exist_)
            {
                recordLog("Controller transfer to ENGAGED");
                ctrl_state_ = CTRL_ENGAGED;
            }
            else if((--ctrl_reset_count_) < 0)
            {
                recordLog("Controller transfer to ESTOP");
                ctrl_state_ = CTRL_ESTOP;
            }
            break;
        case CTRL_ESTOP_TO_TERMINATE:
            if(robot_state_ == ROBOT_IDLE)
            {
                motion_control_ptr_->saveJoint();
                recordLog("Controller transfer to TERMINATED");
                ctrl_state_ = CTRL_TERMINATED;
                shutdown();
            }
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
            if(interpreter_state_ == INTERPRETER_EXECUTE)
            {
                recordLog("Robot transfer to RUNNING");
                robot_state_ = ROBOT_RUNNING;
            }
            break;
        case ROBOT_IDLE_TO_TEACHING:
            recordLog("Robot transfer to TEACHING");
            robot_state_ = ROBOT_TEACHING;
            break;
        case ROBOT_RUNNING_TO_IDLE:
            if(interpreter_state_ != INTERPRETER_EXECUTE
                && servo_state_ != SERVO_RUNNING)
            {
                recordLog("Robot transfer to IDLE");
                robot_state_ = ROBOT_IDLE;
            }
            break;
        case ROBOT_TEACHING_TO_IDLE:
            if(servo_state_ != SERVO_RUNNING)
            {
                recordLog("Robot transfer to IDLE");
                robot_state_ = ROBOT_IDLE;
            }
            break;
        case ROBOT_RUNNING:
            if(interpreter_state_ != INTERPRETER_EXECUTE)
            {
                robot_state_ = ROBOT_RUNNING_TO_IDLE;
            }
            break;
        case ROBOT_TEACHING:
            if (motion_control_ptr_->getGroupState() == fst_mc::STANDBY)
            //if(virtual_core1_ptr_->getArmState() == 1)
            {
                robot_state_ = ROBOT_TEACHING_TO_IDLE;
            }
            break;
        case ROBOT_IDLE:
            if(interpreter_state_ == INTERPRETER_EXECUTE)
            {
                robot_state_ = ROBOT_IDLE_TO_RUNNING;
            }
            break;
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

void ControllerSm::recordLog(std::string log_str)
{
    ServerAlarmApi::GetInstance()->sendOneAlarm(CONTROLLER_LOG, log_str);
}


