#include <unistd.h>
#include <string.h>  
#include "axis1001.h"

using namespace axis_space;
using namespace log_space;
using namespace base_space;
using namespace servo_comm_space;
using namespace system_model_space;


Axis1001::Axis1001(int32_t id):
	Axis(id, AXIS_TYPE_STEPPER)
{    
}

Axis1001::~Axis1001(void)
{
}

bool Axis1001::initApplication(void)
{  
    return true;
}

bool Axis1001::reloadSystemModel(void)
{
    return true;
}

ErrorCode Axis1001::mcPower(bool enable)
{
    bool ret = false;
    if (enable)
    {
        clearBQ();

        // to update application params
        if (!reloadSystemModel())
        {
            LogProducer::error("Axis1001", "Axis[%d] reload parameters failed", getID());
            return AXIS_ALG_NOT_DEFINED;
        }
		        
		ret = servo_comm_ptr_->emitServoCmdShutDown();
	    if (!ret)
	    {
	    	LogProducer::warn("Axis1001", "Axis[%d] stepperPower called failed when emitServoCmdShutDown", getID());
		    return AXIS_SEND_CORE_POWER_FAILED;
	    }
		usleep(10*1000);
    
        ret = servo_comm_ptr_->emitServoCmdEnableOperation();
        if (!ret)
	    {
	        LogProducer::warn("Axis1001", "Axis[%d] stepperPower called failed when emitServoCmdSwitchOnAndEnableOperation", getID());
		    return AXIS_SEND_CORE_POWER_FAILED;
	    }
    }
	else
	{   
		ret = servo_comm_ptr_->emitServoCmdDisableOperation();
		if (!ret)
		{
			LogProducer::warn("Axis1001", "Axis[%d] stepperPower called failed when emitServoCmdSwitchOn", getID());
			return AXIS_SEND_CORE_POWER_FAILED;
		}
	}
	LogProducer::info("Axis1001", "Axis[%d] stepperPower(%d) called", getID(), enable);
	return SUCCESS;
}

bool Axis1001::pushBackFB(void* fb_ptr)
{
    return false;
}

FBQueueStatus_e Axis1001::getFBQStatus()
{
        return FBQ_STATUS_FULL;
}

void Axis1001::processFBQ()
{
    return;
}

void Axis1001::processTBQ()
{
    return;
}

void Axis1001::clearBQ() 
{
    return;
}

