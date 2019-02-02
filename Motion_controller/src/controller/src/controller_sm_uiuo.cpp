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


void ControllerSm::processUIUO()
{
    if((program_launching_ptr_->getLaunchMode() != PROGRAM_LAUNCH_CODE_SELECT)
        || (getUserOpMode() != USER_OP_MODE_AUTO)
        || (getCtrlState() != CTRL_ENGAGED))
    {
        setUO(static_cast<uint32_t>(UO_CMD_ENABLE), false);//UO[1] not enable UIUO function
        return;
    }
    else
    {
        setUO(static_cast<uint32_t>(UO_CMD_ENABLE), true);//UO[1] enable UIUO function
    }
    

    uint8_t value = 0;
    bool level = false;

    //if UI[1] is OFF, 0-stop.
    if (getUI(static_cast<uint32_t>(UI_SERVO_ENABLE), level))
    {
        if(level == false)
        {
            callEstop();
            setUO(static_cast<uint32_t>(UO_PAUSED), false);//UO[2] signal not paused
            setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), false);//UO[4] signal no program running
        }
    }

    //if UI[2] is OFF, pause
    if (getUI(static_cast<uint32_t>(UI_PAUSE_REQUEST), level))
    {
        if((getInterpreterState() != INTERPRETER_EXECUTE) || (getCtrlState() != CTRL_ENGAGED))
        {}
        else if(level == false)
        {
            FST_INFO("----UI call pause.");
            controller_client_ptr_->pause();
            setUO(static_cast<uint32_t>(UO_PAUSED), true);//UO[2] Paused signal
            setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), false);//UO[4] signal no program running
        }   

    }

    //if UI[3] is ON, reset
    if (getUI(static_cast<uint32_t>(UI_RESET), level))
    {
        if(level == true)
        {
            callReset();
            setUO(static_cast<uint32_t>(UO_PAUSED), false);//UO[2] signal not paused
            setUO(static_cast<uint32_t>(UO_FAULT), false);//UO[3] signal no_fault 
            setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), false);//UO[4] signal no program running
        }     
    }

    //if UI[4] is pulse down, start&restart (resume?)
    if (isFallingEdge(static_cast<uint32_t>(UI_PAUSE_REQUEST)))
    {
        if((getInterpreterState() != INTERPRETER_PAUSED) && (getCtrlState() != CTRL_ENGAGED))
        {}
        else
        {
            FST_INFO("----UI call resume.");
            controller_client_ptr_->resume();//todo start(program_name)?
            setUO(static_cast<uint32_t>(UO_PAUSED), false);//UO[2] signal not paused
            setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), true);//UO[4] signal program running 
        }      
    }

    //if UI[5] is OFF, abort
    if (getUI(static_cast<uint32_t>(UI_ABORT_PROGRAM), level))
    {
        if((level == false) && (getInterpreterState() != INTERPRETER_IDLE))
        {
            FST_INFO("----UI call Abort.");
            controller_client_ptr_->abort(); 
            motion_control_ptr_->abortMove();
            setUO(static_cast<uint32_t>(UO_PROGRAM_RUNNING), false);//UO[4] signal no program running
        }
    }

    //if UI[6] is ON, read UI[8-12] to get progarm code.
    if (program_launching_ptr_->getLaunchMode() == PROGRAM_LAUNCH_CODE_SELECT
        && getUI(static_cast<uint32_t>(UI_SELECTION_STROBE), level))
    {
        if(level == true)
        {
            program_code_ = getSetProgramCode();
            FST_INFO("----UI call to select program:%d", program_code_);
            if (program_code_ != -1)
            {
                setUO(static_cast<uint32_t>(UO_SELECTION_CHECK_REQUEST), true);//UO[6] signals code reading is finished
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
    if (getUI(static_cast<uint32_t>(UI_MPLCS_START), level))
    {
        if((level == true) && (getUserOpMode() == USER_OP_MODE_AUTO)
                        && (getInterpreterState() == INTERPRETER_IDLE)
                        && (getCtrlState() == CTRL_ENGAGED)
                        && (getRobotState() == ROBOT_IDLE))
        {
            //send prg code.
            FST_INFO("----UI call to start program.");
            controller_client_ptr_->codeStart(program_code_);

            //UO[7] true after sending code to prg.
            setUO(static_cast<uint32_t>(UO_MPLCS_START_DONE), true);//UO[7] signals program is started.
        }
    }

    //UO[3] is on if error is exist
    if(is_error_exist_)
    {
        setUO(static_cast<uint32_t>(UO_FAULT), true);//UO[3] signal fault
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

// UI check if there is falling edge.
bool ControllerSm::isFallingEdge(uint32_t user_port)
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
