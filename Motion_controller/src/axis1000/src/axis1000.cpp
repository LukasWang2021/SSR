#include <unistd.h>
#include <string.h>  
#include "axis1000.h"

using namespace axis_space;
using namespace log_space;
using namespace base_space;
using namespace servo_comm_space;
using namespace system_model_space;


Axis1000::Axis1000(int32_t id):
	Axis(id, AXIS_TYPE_SERVO)
{
}

Axis1000::~Axis1000(void)
{
}

bool Axis1000::initApplication(void)
{
    return true;
}

bool Axis1000::reloadSystemModel(void)
{
	if (!db_ptr_->application_ptr->get(AxisApplication1001__app_max_vel, &max_vel_))
		return false;
	LogProducer::info("Axis1000", "Axis[%d] max velocity = %d", getID(), max_vel_);
    return true;
}

ErrorCode Axis1000::mcPower(bool enable)
{
    bool ret = false;
    if (enable)
    {
        clearBQ();

        // to update application params
        if (!reloadSystemModel())
        {
            LogProducer::error("Axis1000", "Axis[%d] reload parameters failed", getID());
            return AXIS_ALG_NOT_DEFINED;
        }
    
        ret = servo_comm_ptr_->emitServoCmdShutDown();
	    if (!ret)
	    {
	    	LogProducer::warn("Axis1000", "Axis[%d] servoPower called failed when emitServoCmdShutDown", getID());
		    return AXIS_SEND_CORE_POWER_FAILED;
	    }
		usleep(10*1000);

        ret = servo_comm_ptr_->emitServoCmdSwitchOnAndEnableOperation();
        if (!ret)
	    {
	        LogProducer::warn("Axis1000", "Axis[%d] servoPower called failed when emitServoCmdSwitchOnAndEnableOperation", getID());
		    return AXIS_SEND_CORE_POWER_FAILED;
	    }
    }
	else
	{   
		ret = servo_comm_ptr_->emitServoCmdDisableVoltage();
		if (!ret)
		{
			LogProducer::warn("Axis1000", "Axis[%d] servoPower called failed when emitServoCmdDisableVoltage", getID());
			return AXIS_SEND_CORE_POWER_FAILED;
		}
	}
	LogProducer::info("Axis1000", "Axis[%d] servoPower(%d) called", getID(), enable);
	return SUCCESS;
}

bool Axis1000::pushBackFB(void* fb_ptr)
{
    return false;
}

FBQueueStatus_e Axis1000::getFBQStatus()
{
    return FBQ_STATUS_FULL;
}

void Axis1000::processFBQ()
{
    return;
}

void Axis1000::processTBQ()
{
    return;
}

void Axis1000::clearBQ() 
{
    return;
}

