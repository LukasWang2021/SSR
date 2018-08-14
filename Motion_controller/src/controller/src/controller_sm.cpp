#include "controller_sm.h"
#include "error_monitor1.h"
#include <unistd.h>


using namespace fst_ctrl;
using namespace fst_base;

ControllerSm::ControllerSm():
    log_ptr_(NULL),
    param_ptr_(NULL),
    virtual_core1_ptr_(NULL),
    user_op_mode_(USER_OP_MODE_AUTO),
    running_status_(RUNNING_STATUS_LIMITED),
    interpreter_status_(INTERPRETER_IDLE),
    robot_status_(ROBOT_IDLE),
    ctrl_status_(CTRL_INIT),
    servo_status_(SERVO_INIT),
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

void ControllerSm::init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, VirtualCore1* virtual_core1_ptr)
{
    log_ptr_ = log_ptr;
    param_ptr_ = param_ptr;
    virtual_core1_ptr_ = virtual_core1_ptr;
}

void ControllerSm::processStateMachine()
{
    processInterpreter();
    processError();
    transferServoStatus();
    transferCtrlStatus();
    transferRobotStatus();
    usleep(param_ptr_->routine_cycle_time_);
}

UserOpMode ControllerSm::getUserOpMode()
{
    return user_op_mode_;
}

RunningStatus ControllerSm::getRunningStatus()
{
    return running_status_;
}

InterpreterStatus ControllerSm::getInterpreterStatus()
{
    return interpreter_status_;
}

RobotStatus ControllerSm::getRobotStatus()
{
    return robot_status_;
}

CtrlStatus ControllerSm::getCtrlStatus()
{
    return ctrl_status_;
}

ServoStatus ControllerSm::getServoStatus()
{
    return servo_status_;
}

int ControllerSm::getSafetyAlarm()
{
    return safety_alarm_;
}

bool ControllerSm::setUserOpMode(UserOpMode mode)
{
    if(mode == USER_OP_MODE_AUTO
        || mode == USER_OP_MODE_SLOWLY_MANUAL
        || mode == USER_OP_MODE_UNLIMITED_MANUAL)
    {
        user_op_mode_ = mode;
        return true;
    }
    else
    {
        return false;
    }
}

bool ControllerSm::callEstop()
{
    if(ctrl_status_ == CTRL_ENGAGED
        || ctrl_status_ == CTRL_ESTOP_TO_ENGAGED)
    {
        //serv_jtac_.stopBareMetal();
        //RobotCtrlCmd cmd = ABORT_CMD;
        //XXsetCtrlCmd(&cmd, 0);
        FST_INFO("---callEstop: ctrl_status-->CTRL_ANY_TO_ESTOP");
        ctrl_status_ = CTRL_ANY_TO_ESTOP;
    } 
    return true;
}

bool ControllerSm::callReset()
{
    if(ctrl_status_ == CTRL_ESTOP
        || ctrl_status_ == CTRL_INIT)
    {
        ErrorMonitor::instance()->clear();
        //serv_jtac_.resetBareMetal();
        //safety_interface_.reset();
        ctrl_reset_count_ =  param_ptr_->reset_max_time_ / param_ptr_->routine_cycle_time_;
        FST_INFO("---callReset: ctrl_status-->CTRL_ESTOP_TO_ENGAGED");
        ctrl_status_ = CTRL_ESTOP_TO_ENGAGED;
    }
    return true;
}

UserOpMode* ControllerSm::getUserOpModePtr()
{
    return &user_op_mode_;
}

RunningStatus* ControllerSm::getRunningStatusPtr()
{
    return &running_status_;
}

InterpreterStatus* ControllerSm::getInterpreterStatusPtr()
{
    return &interpreter_status_;
}

RobotStatus* ControllerSm::getRobotStatusPtr()
{
    return &robot_status_;
}

CtrlStatus* ControllerSm::getCtrlStatusPtr()
{
    return &ctrl_status_;
}

ServoStatus* ControllerSm::getServoStatusPtr()
{
    return &servo_status_;
}

int* ControllerSm::getSafetyAlarmPtr()
{
    return &safety_alarm_;
}

void ControllerSm::processInterpreter()
{
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

void ControllerSm::transferServoStatus()
{
    //servo_status_ = (ServoStatus)ShareMem::instance()->getServoState();
    servo_status_ = (ServoStatus)virtual_core1_ptr_->getServoStatus();
    if(ctrl_status_ == CTRL_ENGAGED
        && servo_status_ != SERVO_READY
        && servo_status_ != SERVO_RUNNING)
    {
        // this is ugly, because of lacking status in ctrl status definition
        if(robot_status_ == ROBOT_TEACHING
            || robot_status_ == ROBOT_IDLE_TO_TEACHING
            || robot_status_ == ROBOT_TEACHING_TO_IDLE)
        {
            ;//FST_INFO("---transferServoStatus: in teaching mode: clearArmGroup");
            //arm_group_->clearArmGroup();
        }
        //RobotCtrlCmd cmd = ABORT_CMD;
        //XXsetCtrlCmd(&cmd, 0);
        FST_INFO("---transferServoStatus: servo in error: ctrl_status-->CTRL_ANY_TO_ESTOP");
        ctrl_status_ = CTRL_ANY_TO_ESTOP;
    }      
}

void ControllerSm::transferCtrlStatus()
{
    switch(ctrl_status_)
    {
        case CTRL_ANY_TO_ESTOP:
            if(robot_status_ == ROBOT_IDLE)
            {
                //recordJoints();
                FST_INFO("---transferCtrlStatus: ctrl_status-->CTRL_ESTOP");
                ctrl_status_ = CTRL_ESTOP;
            }
            break;
        case CTRL_ESTOP_TO_ENGAGED:
            if(robot_status_ == ROBOT_IDLE
                && servo_status_ == SERVO_READY
                && !is_error_exist_)
            {
                FST_INFO("---transferCtrlStatus: ctrl_status-->CTRL_ENGAGED");
                ctrl_status_ = CTRL_ENGAGED;
            }
            else if((--ctrl_reset_count_) < 0)
            {
                FST_INFO("---transferCtrlStatus: ctrl_status-->CTRL_ESTOP");
                ctrl_status_ = CTRL_ESTOP;
            }
            break;
        case CTRL_ESTOP_TO_TERMINATE:
            if(robot_status_ == ROBOT_IDLE)
            {
                //recordJoints();
                FST_INFO("---transferCtrlStatus: ctrl_status-->CTRL_TERMINATED");
                ctrl_status_ = CTRL_TERMINATED;
            }
            break;
        default:
            ;
    }
}

void ControllerSm::transferRobotStatus()
{    
    switch(robot_status_)
    {
        case ROBOT_IDLE_TO_RUNNING:
            if(interpreter_status_ == INTERPRETER_EXECUTE)
            {
                FST_INFO("---transferRobotStatus: robot_status-->ROBOT_RUNNING");
                robot_status_ = ROBOT_RUNNING;
            }
            break;
        case ROBOT_IDLE_TO_TEACHING:
            FST_INFO("---transferRobotStatus: robot_status-->ROBOT_TEACHING");
            robot_status_ = ROBOT_TEACHING;
            break;
        case ROBOT_RUNNING_TO_IDLE:
            if(interpreter_status_ != INTERPRETER_EXECUTE
                && servo_status_ != SERVO_RUNNING)
            {
                FST_INFO("---transferRobotStatus: robot_status-->ROBOT_IDLE");
                robot_status_ = ROBOT_IDLE;
            }
            break;
        case ROBOT_TEACHING_TO_IDLE:
            if(servo_status_ != SERVO_RUNNING)
            {
                FST_INFO("---transferRobotStatus: robot_status-->ROBOT_IDLE");
                robot_status_ = ROBOT_IDLE;
            }
            break;
        case ROBOT_RUNNING:
            if(interpreter_status_ != INTERPRETER_EXECUTE)
            {
                FST_INFO("---transferRobotStatus: robot_status-->ROBOT_RUNNING_TO_IDLE");
                robot_status_ = ROBOT_RUNNING_TO_IDLE;
            }
            break;
        case ROBOT_TEACHING:
            //if (arm_group_->getFIFOLength() == 0)
            if(virtual_core1_ptr_->getArmStatus() == 1)
            {
                FST_INFO("---transferRobotStatus: robot_status-->ROBOT_TEACHING_TO_IDLE");
                robot_status_ = ROBOT_TEACHING_TO_IDLE;
            }
            break;
        case ROBOT_IDLE:
            if(interpreter_status_ == INTERPRETER_EXECUTE)
            {
                FST_INFO("---transferRobotStatus: robot_status-->ROBOT_IDLE_TO_RUNNING");
                robot_status_ = ROBOT_IDLE_TO_RUNNING;
            }
            break;
        default:
            ;
    }
}

