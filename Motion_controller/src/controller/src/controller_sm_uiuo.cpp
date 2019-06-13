#include "controller_sm.h"
#include "error_monitor.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string>


using namespace fst_ctrl;
using namespace fst_base;
using namespace fst_mc;
using namespace fst_hal;

void ControllerSm::processUIUO()
{
    //UO[1] singles whether enable to start
    if((getUserOpMode() == USER_OP_MODE_AUTO) 
       && (getCtrlState() == CTRL_ENGAGED)
       && (getInterpreterState() == INTERPRETER_IDLE))
    {
        setUoEnableOn();//UO[1]=on
    }else
    {
        setUoEnableOff();//UO[1]=off
    }

    //UO[3] is on if error is exist
    if(is_error_exist_)
    {
        setUoFaultOn();//UO[3] signal fault
    }

    bool level = false;
    //if UI[1] is OFF, 0-stop.
    if (getUI(static_cast<uint32_t>(UI_SERVO_ENABLE), level))
    {
        if(level == false)
        {
            if(callEstop() == SUCCESS)
            {
                ui_servo_enable_ = false;
                //recordLog(UI_SERVO_ENABLE_OFF);
            }
        }else
        {
            ui_servo_enable_ = true;
        }
    }

    //if UI[2] is OFF, pause
    if (getUI(static_cast<uint32_t>(UI_PAUSE_REQUEST), level))
    {
        if((level == false) && (getInterpreterState() == INTERPRETER_EXECUTE) && (getCtrlState() == CTRL_ENGAGED))
        {
            FST_INFO("----UI call pause.");
            if(motion_control_ptr_->pauseMove() == SUCCESS)
            {
                FST_INFO("----UI call pause success.");
                controller_client_ptr_->pause();
                setUoPausedOn();//UO[2]=on
                setUoProgramRunOff();//UO[4]=off
                //recordLog(UI_Pause_INFO);
            }  
        }   
    }

    //if UI[3] is ON, reset
    if (getUI(static_cast<uint32_t>(UI_RESET), level))
    {
        if(level == true)
        {
            callReset();
        }     
    }

    //if UI[4] is pulse down, start&restart (resume)
    if (isFallingEdgeStart(static_cast<uint32_t>(UI_START)))
    {
        if((getInterpreterState() == INTERPRETER_PAUSED) && (getCtrlState() == CTRL_ENGAGED) && (getRobotState() == ROBOT_IDLE))
        {
            FST_INFO("----UI call resume.");
            if(motion_control_ptr_->restartMove() == SUCCESS)
            {
                FST_INFO("----UI call resume success.");
                controller_client_ptr_->resume();
                transferRobotStateToRunning();
                setUoPausedOff();//UO[2]=off
                setUoProgramRunOn();//UO[4]=on
            } 
        } 
        else if((getInterpreterState() == INTERPRETER_IDLE) && (getCtrlState() == CTRL_ENGAGED))
        {
            InterpreterPublish* data_ptr = controller_client_ptr_->getInterpreterPublishPtr();
            std:string program_name = data_ptr->program_name;
            FST_INFO("----UI[4] call to start program: %s --- %s", program_name.c_str(), data_ptr->program_name);
            controller_client_ptr_->start(program_name);
            transferRobotStateToRunning();
            setUoPausedOff();//UO[2]=off
            setUoProgramRunOn();//UO[4]=on
        }     
    }
    
    //if UI[5] is OFF, abort
    if (isFallingEdgeAbort(static_cast<uint32_t>(UI_ABORT_PROGRAM)))
    {
        if(getInterpreterState() != INTERPRETER_IDLE)
        {
            FST_INFO("----UI call Abort.");
            if (motion_control_ptr_->abortMove() == SUCCESS)
            {
                FST_INFO("----UI call Abort success.");
                controller_client_ptr_->abort(); 
                setUoPausedOff();//UO[2]=off
                setUoProgramRunOff();//UO[4]=off
            }
        }
    }
    
    //below can only be controlled under "MPLCS"" mode 
    if(program_launching_ptr_->getLaunchMode() != PROGRAM_LAUNCH_CODE_SELECT)
    {
        return;
    }

    if (uo_cmd_enable_ == false)
    {
        return;
    }

    //if UI[6] is ON, read UI[8-13] to get and set progarm code.
    if (getUI(static_cast<uint32_t>(UI_SELECTION_STROBE), level)
        && getInterpreterState() == INTERPRETER_IDLE)
    {
        if(level == true)
        {
            program_code_ = getSetProgramCode();
            //FST_INFO("----UI call to select program:%d", program_code_);//todo comment
            if (program_code_ != -1)
            {
                setUO(static_cast<uint32_t>(UO_SELECTION_CHECK_REQUEST), true);//UO[6]=true signals code reading is finished
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

    //if UI[7] is ON, call prg to start program running.
    if (isRisingEdge(static_cast<uint32_t>(UI_MPLCS_START)))
    {
        if((getUserOpMode() == USER_OP_MODE_AUTO)
            && (getInterpreterState() == INTERPRETER_IDLE)
            && (getCtrlState() == CTRL_ENGAGED)
            && (getRobotState() == ROBOT_IDLE))
        {
            //send prg code.
            FST_INFO("----UI call to start program.");
            controller_client_ptr_->codeStart(program_code_);
            transferRobotStateToRunning();
            
            setUoProgramRunOn();//UO[4]=on//setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), true);
            setUO(static_cast<uint32_t>(UO_MPLCS_START_DONE), true);//UO[7]=true signals sending code to prg.
        }
    }

}

/*
void ControllerSm::processUIUO()
{     
    if(program_launching_ptr_->getLaunchMode() != PROGRAM_LAUNCH_CODE_SELECT)
    {
        return;
    }

    bool level = false;

    if((getUserOpMode() != USER_OP_MODE_AUTO) || (getCtrlState() != CTRL_ENGAGED))
    {
        setUO(static_cast<uint32_t>(UO_CMD_ENABLE), false);//UO[1]=false not enable UIUO function
        setUO(static_cast<uint32_t>(UO_PAUSED), false);//UO[2]=false signal unpaused
        setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), false);//UO[4]=false signal no program running
        setUO(static_cast<uint32_t>(UO_SERVO_STATUS), false);//UO[5]=false signal servo_off
        setUO(static_cast<uint32_t>(UO_SELECTION_CHECK_REQUEST), false);//UO[6] reset
        setUO(static_cast<uint32_t>(UO_MPLCS_START_DONE), false);//UO[7] reset
        setUO(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_1), false);//UO[8]
        setUO(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_2), false);//UO[9]
        setUO(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_3), false);//UO[10]
        setUO(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_4), false);//UO[11]
        setUO(static_cast<uint32_t>(UO_PROGRAM_CONFIRM_5), false);//UO[12]

        //UO[3] is on if error is exist
        if(is_error_exist_)
        {
            setUO(static_cast<uint32_t>(UO_FAULT), true);//UO[3] signal fault
        }

        //if UI[3] is ON, reset
        bool enable_level = false;
        if (getUI(static_cast<uint32_t>(UI_RESET), level) && getUI(static_cast<uint32_t>(UI_SERVO_ENABLE), enable_level))
        {
            if(level == true && enable_level == true)
            {
                if (callReset() == SUCCESS)
                {
                    setUO(static_cast<uint32_t>(UO_FAULT), false);//UO[3]=false signal no_fault 
                }
            }     
        }
        return;
    }
    else
    {
        setUO(static_cast<uint32_t>(UO_CMD_ENABLE), true);//UO[1]=true enable UIUO function
    }

    //if UI[1] is OFF, 0-stop.
    if (getUI(static_cast<uint32_t>(UI_SERVO_ENABLE), level))
    {
        if(level == false)
        {
            callEstop();
            setUO(static_cast<uint32_t>(UO_PAUSED), false);//UO[2]=false signal unpaused
            setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), false);//UO[4]=false signal no program running
        }
    }

    //if UI[2] is OFF, pause
    if (getUI(static_cast<uint32_t>(UI_PAUSE_REQUEST), level))
    {
        if((level == false) && (getInterpreterState() == INTERPRETER_EXECUTE) && (getCtrlState() == CTRL_ENGAGED))
        {
            FST_INFO("----UI call pause.");
            if(motion_control_ptr_->pauseMove() == SUCCESS)
            {
                FST_INFO("----UI call pause success.");
                controller_client_ptr_->pause();
                setUO(static_cast<uint32_t>(UO_PAUSED), true);//UO[2]=true Paused signal
                setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), false);//UO[4]=false signal no program running
            }  
        }   
    }
    
    //if UI[4] is pulse down, start&restart (resume)
    if (isFallingEdgeStart(static_cast<uint32_t>(UI_START)))
    {
        if((getInterpreterState() == INTERPRETER_PAUSED) && (getCtrlState() == CTRL_ENGAGED) && (getRobotState() == ROBOT_IDLE))
        {
            FST_INFO("----UI call resume.");
            if(motion_control_ptr_->restartMove() == SUCCESS)
            {
                FST_INFO("----UI call resume success.");
                controller_client_ptr_->resume();
                transferRobotStateToRunning();
                setUO(static_cast<uint32_t>(UO_PAUSED), false);//UO[2]=false signal unpaused
                setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), true);//UO[4]=true signal program running 
            } 
        }      
    }

    //if UI[5] is OFF, abort
    if (isFallingEdgeAbort(static_cast<uint32_t>(UI_ABORT_PROGRAM)))
    {
        if(getInterpreterState() != INTERPRETER_IDLE)
        {
            FST_INFO("----UI call Abort.");
            if (motion_control_ptr_->abortMove() == SUCCESS)
            {
                FST_INFO("----UI call Abort success.");
                controller_client_ptr_->abort(); 
                setUO(static_cast<uint32_t>(UO_PAUSED), false);//UO[2]=false signal unpaused
                setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), false);//UO[4]=false signal no program running
            }
        }
    }

    //if UI[6] is ON, read UI[8-12] to get progarm code.
    if (program_launching_ptr_->getLaunchMode() == PROGRAM_LAUNCH_CODE_SELECT
        && getUI(static_cast<uint32_t>(UI_SELECTION_STROBE), level))
    {
        if(level == true)
        {
            program_code_ = getSetProgramCode();
            //FST_INFO("----UI call to select program:%d", program_code_);//todo comment
            if (program_code_ != -1)
            {
                setUO(static_cast<uint32_t>(UO_SELECTION_CHECK_REQUEST), true);//UO[6]=true signals code reading is finished
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
        }
    }

    //if UI[7] is ON, call prg to start program running.
    if (isRisingEdge(static_cast<uint32_t>(UI_MPLCS_START)))
    {
        if((getUserOpMode() == USER_OP_MODE_AUTO)
            && (getInterpreterState() == INTERPRETER_IDLE)
            && (getCtrlState() == CTRL_ENGAGED)
            && (getRobotState() == ROBOT_IDLE))
        {
            //send prg code.
            FST_INFO("----UI call to start program.");
            controller_client_ptr_->codeStart(program_code_);
            transferRobotStateToRunning();
            
            setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), true);//UO[4]=true signal program running
            setUO(static_cast<uint32_t>(UO_MPLCS_START_DONE), true);//UO[7]=true signals sending code to prg.
        }
    }

    //UO[3] is on if error is exist
    if(is_error_exist_)
    {
        setUO(static_cast<uint32_t>(UO_FAULT), true);//UO[3] signal fault
    }

    //UO[4] is off if idle
    if((getInterpreterState() != INTERPRETER_EXECUTE) && (getRobotState() == ROBOT_IDLE))
    {
        setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), false);//UO[4]=false signal no program running
    }
     
    //UO[5] is off if servo_off
    if(getServoState() == SERVO_DISABLE)
    {
        setUO(static_cast<uint32_t>(UO_SERVO_STATUS), false);//UO[5] signal servo_off
    }
    else if(getServoState() == SERVO_IDLE)
    {
        setUO(static_cast<uint32_t>(UO_SERVO_STATUS), true);//UO[5] signal servo_on
    }

}
*/
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

// check if there is rising edge.
bool ControllerSm::isRisingEdge(uint32_t user_port)
{
    static uint8_t pre_value = 1;
    uint8_t current_value = 1;
    if (io_mapping_ptr_->getUIByBit(user_port, current_value) == SUCCESS)
    {
        if (pre_value == 0 && current_value == 1)
        {
            pre_value = current_value;
            return true;
        }
        pre_value = current_value;
        return false;
    }
    return false;
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
