#include "controller_sm.h"
#include "error_monitor.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string>
#include "forsight_inter_control.h"


using namespace fst_ctrl;
using namespace fst_base;
using namespace fst_mc;
using namespace fst_hal;

void ControllerSm::processUIUO()
{
    // request from specific case. DO is on when being in the specific home position.    
    basic_alg::Joint joint = motion_control_ptr_->getServoJoint(); 
    double joint_1 = param_ptr_->hp_joints_[0] * M_PI / 180;//-22.755 * M_PI / 180;   
    double joint_2 = param_ptr_->hp_joints_[1] * M_PI / 180;//-41.682 * M_PI / 180; 
    double joint_3 = param_ptr_->hp_joints_[2] * M_PI / 180;//-7.926 * M_PI / 180; 
    double joint_4 = param_ptr_->hp_joints_[3] * M_PI / 180;//-3.465 * M_PI / 180; 
    double joint_5 = param_ptr_->hp_joints_[4] * M_PI / 180;//-39.473 * M_PI / 180; 
    double joint_6 = param_ptr_->hp_joints_[5] * M_PI / 180;//-69.684 * M_PI / 180;   
    double threhold = M_PI * 2 / 180;    
    uint32_t port_offset = 103;    
    if ((fabs(joint.j1_ - joint_1) < threhold) && (fabs(joint.j2_ - joint_2) < threhold) && (fabs(joint.j3_ - joint_3) < threhold)         
    && (fabs(joint.j4_ - joint_4) < threhold) && (fabs(joint.j5_ - joint_5) < threhold) && (fabs(joint.j6_ - joint_6) < threhold))    
    {        
        io_mapping_ptr_->setDOByBit(port_offset, 1);    
    }    
    else    
    {        
        io_mapping_ptr_->setDOByBit(port_offset, 0);    
    }

    //UO[1]CMDENBLE singals whether enable to start
    if((getUserOpMode() == USER_OP_MODE_AUTO) 
       && (getCtrlState() == CTRL_ENGAGED)
       && (getInterpreterState() == INTERPRETER_IDLE))
    {
        setUoEnableOn();
    }else
    {
        setUoEnableOff();
    }

    //UO[2] PAUSED singal
    if(getInterpreterState() == INTERPRETER_PAUSED)
    {
        setUoPausedOn();
    }else
    {
        setUoPausedOff();
    }

    //UO[3]FAULT is on if error is exist
    if(is_error_exist_.data)
    {
        setUoFaultOn();
    }

    bool level = false;
    //if UI[1]ServoEnable is OFF, 0-stop.
    if (getUI(static_cast<uint32_t>(UI_SERVO_ENABLE), level))
    {
        if(level == false)
        {
            if(callEstop() == SUCCESS)
            {
                FST_INFO("UI[1]ServoEnable call ----> Estop success.");
                ui_servo_enable_ = false;
                recordLog(UI_SERVO_ENABLE_OFF);
            }
        }else
        {
            ui_servo_enable_ = true;
        }
    }

    //if UI[2]Pause is OFF, pause
    if (getUI(static_cast<uint32_t>(UI_PAUSE_REQUEST), level))
    {
        if((level == false) && (getInterpreterState() == INTERPRETER_EXECUTE) && (getCtrlState() == CTRL_ENGAGED))
        {
            FST_INFO("UI[2]Pause call ----> pause.");
            if(motion_control_ptr_->pauseMove() == SUCCESS)
            {
                FST_INFO("UI[2]Pause call ----> pause success.");
                // controller_client_ptr_->pause();
				InterpreterControl intprt_ctrl ;
				intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_PAUSE ;
				parseCtrlComand(intprt_ctrl, "");
		
                setUoPausedOn();
                setUoProgramRunOff();
                recordLog(UI_PAUSE_INFO);
            }  
        }   
    }

    //if UI[3]Reset is ON, reset
    if (getUI(static_cast<uint32_t>(UI_RESET), level))
    {
        if((level == true) && (getCtrlState() != CTRL_ENGAGED))
        {
            if (callReset() == SUCCESS)
            {
                FST_INFO("UI[3]Reset call ----> reset success.");
                recordLog(UI_RESET_INFO);
            }
        }   
        else if((level == true) && (getCtrlState() == CTRL_ENGAGED)) 
        {
            clearPreError();
            setUoFaultOff();//UO[3]=off
            recordLog(INFO_RESET_SUCCESS);
        }
    }

    //if UI[4]Start&Resume is pulse down, start&resume
    if (program_launching_ptr_->getLaunchMode() != PROGRAM_LAUNCH_CODE_SIMPLE
        && isFallingEdgeStart(static_cast<uint32_t>(UI_START)))
    {
        if((getInterpreterState() == INTERPRETER_PAUSED) && (getCtrlState() == CTRL_ENGAGED) && (getRobotState() == ROBOT_IDLE))
        {
            FST_INFO("UI[4] call ----> resume.");
            if(motion_control_ptr_->restartMove() == SUCCESS)
            {
                FST_INFO("UI[4] call ----> resume success.");
                // controller_client_ptr_->resume();
				InterpreterControl intprt_ctrl ;
				intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_RESUME;
				parseCtrlComand(intprt_ctrl, "");
				
                transferRobotStateToRunning();
                setUoPausedOff();
                setUoProgramRunOn();
                recordLog(UI_RESUME_INFO);
            } 
        } 
        else if((getInterpreterState() == INTERPRETER_IDLE) && (getCtrlState() == CTRL_ENGAGED))
        {
            FST_INFO("UI[4] call ----> start program: %s", program_name_.c_str());
            // controller_client_ptr_->start(program_name_);
            InterpreterControl intprt_ctrl ;
            intprt_ctrl.autoMode = LAUNCH_CODE_U;
			intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_START ;
            parseCtrlComand(intprt_ctrl, program_name_.c_str());

            transferRobotStateToRunning();
            setUoProgramRunOn();
            recordLog(UI_START_INFO);
        }     
    }
    
    //if UI[5]Abort is OFF, abort
    if (getUI(static_cast<uint32_t>(UI_ABORT_PROGRAM), level))
    {
        if((level == false) && (getInterpreterState() != INTERPRETER_IDLE))
        {
            FST_INFO("UI[5] call ----> abort.");
            if (motion_control_ptr_->abortMove() == SUCCESS)
            {
                FST_INFO("UI[5] call ----> abort success.");
                // controller_client_ptr_->abort(); 
		        InterpreterControl intprt_ctrl ;
				intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_ABORT ;
		        parseCtrlComand(intprt_ctrl, "");
		
                setUoPausedOff();
                setUoProgramRunOff();
                recordLog(UI_ABORT_INFO);
            }
        }
    }
    
    //below can only be controlled under "MPLCS"" mode, use UI[4]
    if(program_launching_ptr_->getLaunchMode() == PROGRAM_LAUNCH_CODE_SIMPLE)
    {
        if (isFallingEdgeStart(static_cast<uint32_t>(UI_START)))
        {
            if((getUserOpMode() == USER_OP_MODE_AUTO)
                && (getInterpreterState() == INTERPRETER_IDLE)
                && (getCtrlState() == CTRL_ENGAGED)
                && (getRobotState() == ROBOT_IDLE))
            {
                program_code_ = getProgramCode();
                if (program_code_ != -1)
                {                
                    //send prg code.
                    FST_INFO("UI[4] simple MPLCS_Start call ----> start program code: %d", program_code_);
                    InterpreterControl intprt_ctrl ;
                    intprt_ctrl.autoMode = LAUNCH_CODE_U;
                    intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_CODE_START ;
                    char cTemp[16];
                    memset(cTemp, 0x00, 16);
                    sprintf(cTemp, "%d", program_code_);
                    parseCtrlComand(intprt_ctrl, cTemp);
                    
                    transferRobotStateToRunning();
                    setUoProgramRunOn();
                    recordLog(UI_CODE_PROGRAM_START_INFO);
                }
            }
            else if((getUserOpMode() == USER_OP_MODE_AUTO)
                && (getInterpreterState() == INTERPRETER_PAUSED)
                && (getCtrlState() == CTRL_ENGAGED)
                && (getRobotState() == ROBOT_IDLE))
            {
                FST_INFO("UI[4] simple MPLCS_Start call ----> resume.");
                if(motion_control_ptr_->restartMove() == SUCCESS)
                {
                    FST_INFO("UI[4] simple MPLCS_Start call ----> resume success.");
                    InterpreterControl intprt_ctrl ;
                    intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_RESUME;
                    parseCtrlComand(intprt_ctrl, "");
                    
                    transferRobotStateToRunning();
                    setUoPausedOff();
                    setUoProgramRunOn();
                    recordLog(UI_RESUME_INFO);
                } 
            }
            else
            {
                FST_WARN("UI[4] simple MPLCS_Start call under invalid status: user_op_mode=%d, rpg=%d", getUserOpMode(), getInterpreterState());
                recordLog(CONTROLLER_INVALID_OPERATION_START);
            }
        }
    }

    if(program_launching_ptr_->getLaunchMode() == PROGRAM_LAUNCH_CODE_FULL)
    {
        if (uo_cmd_enable_ == false)
            return;
        //if UI[6]Selection is ON, read UI[8-13] to get and set progarm code.
        if (getUI(static_cast<uint32_t>(UI_SELECTION_STROBE), level)
            && getInterpreterState() == INTERPRETER_IDLE)
        {
            if(level == true)
            {
                program_code_ = getSetProgramCode();
                //FST_INFO("UI[6] call ----> select program code:%d", program_code_);
                if (program_code_ != -1)
                {
                    setUO(static_cast<uint32_t>(UO_SELECTION_CHECK_REQUEST), true);//code reading finished
                }

            }
            else if(level == false)
            {
                setUO(static_cast<uint32_t>(UO_SELECTION_CHECK_REQUEST), false);//UO[6] reset
                setUO(static_cast<uint32_t>(UO_MPLCS_START_DONE), false);//UO[7] reset
                setUO(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_1), false);//UO[8]
                setUO(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_2), false);//UO[9]
                setUO(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_3), false);//UO[10]
                setUO(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_4), false);//UO[11]
                setUO(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_5), false);//UO[12]
                setUO(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_6), false);//UO[13]
            }
        }

        //if UI[7]MPLCS_Start is ON, call prg to start program running.
        if (isFallingEdgeCodeStart(static_cast<uint32_t>(UI_MPLCS_START)))
        {
            if((getUserOpMode() == USER_OP_MODE_AUTO)
                && (getInterpreterState() == INTERPRETER_IDLE)
                && (getCtrlState() == CTRL_ENGAGED)
                && (getRobotState() == ROBOT_IDLE))
            {
                //send prg code.
                FST_INFO("UI[7] MPLCS_Start call ----> start program code: %d", program_code_);
                // controller_client_ptr_->codeStart(program_code_);
                InterpreterControl intprt_ctrl ;
                intprt_ctrl.autoMode = LAUNCH_CODE_U;
                intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_CODE_START ;
                char cTemp[16];
                memset(cTemp, 0x00, 16);
                sprintf(cTemp, "%d", program_code_);
                parseCtrlComand(intprt_ctrl, cTemp);
                
                transferRobotStateToRunning();
                setUoProgramRunOn();
                setUO(static_cast<uint32_t>(UO_MPLCS_START_DONE), true);//UO[7]=true signals sending code to prg.
                recordLog(UI_CODE_PROGRAM_START_INFO);
            }
            else
            {
                FST_WARN("UI[7] MPLCS_Start call under invalid status: user_op_mode=%d, rpg=%d", getUserOpMode(), getInterpreterState());
                recordLog(CONTROLLER_INVALID_OPERATION_START);
            }
        }
        
    }//end if
}

// UI check if there is falling edge.
bool ControllerSm::isFallingEdgeStart(uint32_t user_port)
{
    static uint8_t pre_value = 0;
    uint8_t current_value = 0;
    if (io_mapping_ptr_->getUIByBit(user_port, current_value) == SUCCESS)
    {
        if (pre_value == 1 && current_value == 0)
        {
            pre_value = current_value;
            return true;
        }
        pre_value = current_value;
        return false;
    }
    return false;
}

bool ControllerSm::isFallingEdgeAbort(uint32_t user_port)
{
    static uint8_t pre_value = 0;
    uint8_t current_value = 0;
    if (io_mapping_ptr_->getUIByBit(user_port, current_value) == SUCCESS)
    {
        if (pre_value == 1 && current_value == 0)
        {
            pre_value = current_value;
            return true;
        }
        pre_value = current_value;
        return false;
    }
    return false;
}

// check if there is falling edge.
bool ControllerSm::isFallingEdgeCodeStart(uint32_t user_port)
{
    static uint8_t pre_value = 0;
    uint8_t current_value = 0;
    if (io_mapping_ptr_->getUIByBit(user_port, current_value) == SUCCESS)
    {
        if (pre_value == 1 && current_value == 0)
        {
            pre_value = current_value;
            return true;
        }
        pre_value = current_value;
        return false;
    }
    return false;
}

int ControllerSm::getProgramCode()
{
    uint8_t value = 0;
    uint8_t code = 0;

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_1), value) != SUCCESS)
        return -1;
    code += value;

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_2), value) != SUCCESS)
        return -1;
    code += (value<<1);

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_3), value) != SUCCESS)
        return -1;
    code += (value<<2);

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_4), value) != SUCCESS)
        return -1;
    code += (value<<3);

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_5), value) != SUCCESS)
        return -1;
    code += (value<<4);

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_6), value) != SUCCESS)
        return -1;
    code += (value<<5);

    return code;
}

int ControllerSm::getSetProgramCode()
{
    uint8_t value = 0;
    uint8_t code = 0;

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_1), value) != SUCCESS)
        return -1;
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_1), value);
    code += value;

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_2), value) != SUCCESS)
        return -1;
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_2), value);
    code += (value<<1);

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_3), value) != SUCCESS)
        return -1;
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_3), value);
    code += (value<<2);

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_4), value) != SUCCESS)
        return -1;
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_4), value);
    code += (value<<3);

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_5), value) != SUCCESS)
        return -1;
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_5), value);
    code += (value<<4);

    if (io_mapping_ptr_->getUIByBit(static_cast<uint32_t>(UI_PROGRAM_SELECTION_6), value) != SUCCESS)
        return -1;
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_6), value);
    code += (value<<5);

    return code;
}


bool ControllerSm::getUI(uint32_t user_port, bool &level)
{
    uint8_t value = 0;
    ErrorCode ret = io_mapping_ptr_->getUIByBit(user_port, value);
    if(ret == SUCCESS)
    {
        level = value;
        return true;
    }
    return false;
}

void ControllerSm::setUO(uint32_t user_port, bool level)
{
    uint8_t value = level;
    io_mapping_ptr_->setUOByBit(user_port, value);
}


void ControllerSm::setUoEnableOn(void)
{
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_CMD_ENABLE), 1);
    uo_cmd_enable_ = true;
}

void ControllerSm::setUoEnableOff(void)
{
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_CMD_ENABLE), 0);
    uo_cmd_enable_ = false;
}

void ControllerSm::setUoPausedOn(void)
{
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_PAUSED), 1);
}

void ControllerSm::setUoPausedOff(void)
{
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_PAUSED), 0);
}

void ControllerSm::setUoFaultOn(void)
{
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_FAULT), 1);
}

void ControllerSm::setUoFaultOff(void)
{
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_FAULT), 0);
}

void ControllerSm::setUoProgramRunOn(void)
{
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_PROGRAM_RUNNING), 1);
}
void ControllerSm::setUoProgramRunOff(void)
{
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_PROGRAM_RUNNING), 0);
}

void ControllerSm::setUoServoOn(void)
{
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_SERVO_STATUS), 1);
}
void ControllerSm::setUoServoOff(void)
{
    io_mapping_ptr_->setUOByBit(static_cast<uint32_t>(UO_SERVO_STATUS), 0);
}

void ControllerSm::setUoAllOff(void)
{
    setUoEnableOff();
    setUoPausedOff();
    setUoFaultOff();
    setUoProgramRunOff();
    setUoServoOff();
}
