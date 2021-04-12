#include <string.h>  
#include "axis_sm.h"

using namespace axis_space;
using namespace log_space;
using namespace base_space;
using namespace system_model_space;
using namespace servo_comm_space;



AxisSm::AxisSm(void):
	axis_state_(AXIS_STATUS_UNKNOWN),
	fdb_ptr_(NULL),
	servo_comm_ptr_(NULL),
	is_err_exist_(false),
    target_reached_count_(0),
	target_reached_max_(2),
	ctrl_pdo_enble_(false),
	is_valid_(true)
{
    
}

AxisSm::~AxisSm(void)
{

}

bool AxisSm::init(int32_t id, AxisFdb* fdb_ptr, ServoCommBase* servo_comm_ptr, AxisModel_t* db_ptr)
{
    id_ = id;
	if (fdb_ptr == NULL || servo_comm_ptr == NULL ||db_ptr == NULL)
		return false;
    fdb_ptr_ = fdb_ptr;
    servo_comm_ptr_ = servo_comm_ptr;

	return true;
}


void AxisSm::processStatemachine()
{
    if(!is_valid_)
        return;
    processError();
	processAxisState();
}

bool AxisSm::transferStateToDiscreteMotion(void)
{
    if(!is_valid_)
        return true;

    if (axis_state_ == AXIS_STATUS_STANDSTILL || axis_state_ == AXIS_STATUS_CONTINUOUS_MOTION 
		|| axis_state_ == AXIS_STATUS_SYNCHRONIZED_MOTION || axis_state_ == AXIS_STATUS_DISCRETE_MOTION)
    {
        LogProducer::warn("AxisSm", "Axis[%d] transfer from %s to DISCRETE_MOTION success", id_, getAxisStateString(axis_state_).c_str());
    	axis_state_ = AXIS_STATUS_DISCRETE_MOTION;	
        target_reached_count_ = 0;
		return true;
    }
	LogProducer::warn("AxisSm", "Axis[%d] transfer from %s to DISCRETE_MOTION failed", id_, getAxisStateString(axis_state_).c_str());
	return false;
}

bool AxisSm::transferStateToContinuousMotion(void)
{
    if(!is_valid_)
        return true;

    if (axis_state_ == AXIS_STATUS_STANDSTILL || axis_state_ == AXIS_STATUS_CONTINUOUS_MOTION 
		|| axis_state_ == AXIS_STATUS_SYNCHRONIZED_MOTION || axis_state_ == AXIS_STATUS_DISCRETE_MOTION)

    {
        LogProducer::warn("AxisSm", "Axis[%d] transfer from %s to CONTINUOUS_MOTION success", id_, getAxisStateString(axis_state_).c_str());
	    axis_state_ = AXIS_STATUS_CONTINUOUS_MOTION;	
        target_reached_count_ = 0;
	    return true;
    }
	LogProducer::warn("AxisSm", "Axis[%d] transfer from %s to CONTINUOUS_MOTION failed", id_, getAxisStateString(axis_state_).c_str());
    return false;
}

bool AxisSm::transferStateToHoming(void)
{
    if(!is_valid_)
        return true;

    if (axis_state_ == AXIS_STATUS_STANDSTILL)
    {
        LogProducer::warn("AxisSm", "Axis[%d] transfer from %s to HOMING success", id_, getAxisStateString(axis_state_).c_str());
	    axis_state_ = AXIS_STATUS_HOMING;
        target_reached_count_ = 0;
	    return true;
    }
	LogProducer::warn("AxisSm", "Axis[%d] transfer from %s to HOMING failed", id_, getAxisStateString(axis_state_).c_str());
    return false;
}

void AxisSm::setValid(bool is_valid)
{
    is_valid_ = is_valid;
}

void AxisSm::setError(void)
{
    is_err_exist_ = true;
}

void AxisSm::clearError(void)
{
    is_err_exist_ = false;
}


AxisStatus_e AxisSm::getAxisStatus(void)
{
    return axis_state_;
}

std::string AxisSm::getAxisStateString(AxisStatus_e axis_status)
{
    switch(axis_status)
    {
        case AXIS_STATUS_UNKNOWN:                return std::string("UNKNOWN");
        case AXIS_STATUS_ERRORSTOP:              return std::string("ERRORSTOP");
        case AXIS_STATUS_DISABLED:               return std::string("DISABLED");
        case AXIS_STATUS_STANDSTILL:             return std::string("STANDSTILL");
        case AXIS_STATUS_STOPPING:               return std::string("STOPPING");
        case AXIS_STATUS_HOMING:                 return std::string("HOMING");
        case AXIS_STATUS_DISCRETE_MOTION:        return std::string("DISCRETE_MOTION");
        case AXIS_STATUS_CONTINUOUS_MOTION:      return std::string("CONTINUOUS_MOTION");
        case AXIS_STATUS_SYNCHRONIZED_MOTION:    return std::string("SYNCHRONIZED_MOTION");
        default:                                return std::string("Unknown");
    }
}

bool AxisSm::isCtrlPdoEnable(void)
{
    return ctrl_pdo_enble_;
}



//private functions
void AxisSm::processError(void)
{
    if (true == is_err_exist_ || SERVO_SM_FAULT == fdb_ptr_->getServoState())
    {
        if (axis_state_ != AXIS_STATUS_ERRORSTOP)
        {
            //upload servo error
            int32_t error = 0; 
            servo_comm_ptr_->doServoCmdReadParameter(SERVO_PARAM_CURRENT_ERROR, &error);
            ErrorQueue::instance().push(error);
            
            LogProducer::warn("AxisSm", "Axis[%d] transfer from %s to ERRORSTOP success", id_, getAxisStateString(axis_state_).c_str());
    	    axis_state_ = AXIS_STATUS_ERRORSTOP;
            target_reached_count_ = 0;
            ctrl_pdo_enble_ = false;
        }
    }
}

void AxisSm::processAxisState(void)
{
    switch(axis_state_)
    {
        case AXIS_STATUS_ERRORSTOP:
			if (false == is_err_exist_ && SERVO_SM_SWITCH_ON_DISABLED == fdb_ptr_->getServoState())
			{
			    axis_state_ = AXIS_STATUS_DISABLED;
				LogProducer::warn("AxisSm", "Axis[%d] transfer from ERRORSTOP to DISABLED success", id_);
			}
			else if (false == is_err_exist_ && SERVO_SM_OPERATION_ENABLED == fdb_ptr_->getServoState())
			{
			    axis_state_ = AXIS_STATUS_STANDSTILL;
                ctrl_pdo_enble_ = true;
				LogProducer::warn("AxisSm", "Axis[%d] transfer from ERRORSTOP to STANDSTILL success", id_);
			}
			break;
		case AXIS_STATUS_DISABLED:
			if (SERVO_SM_OPERATION_ENABLED == fdb_ptr_->getServoState())
			{
			    axis_state_ = AXIS_STATUS_STANDSTILL;
                ctrl_pdo_enble_ = true;
				LogProducer::warn("AxisSm", "Axis[%d] transfer from DISABLED to STANDSTILL success", id_);
			}
			break;
		case AXIS_STATUS_STANDSTILL:
            if (SERVO_SM_QUICK_STOP_ACTIVE == fdb_ptr_->getServoState())
            {
                axis_state_ = AXIS_STATUS_STOPPING;
                target_reached_count_ = 0;
                ctrl_pdo_enble_ = false;
                LogProducer::warn("AxisSm", "Axis[%d] transfer from STANDSTILL to STOPPING success", id_);
            }
            else if (fdb_ptr_->getServoOpMode() == SERVO_OP_MODE_HOMING_MODE && fdb_ptr_->isHoming())
            {
                axis_state_ = AXIS_STATUS_HOMING;
                target_reached_count_ = 0;
                LogProducer::warn("AxisSm", "Axis[%d] transfer from STANDSTILL to HOMING success", id_);
            }
		    break;
		case AXIS_STATUS_STOPPING:
			if (fdb_ptr_->isTargetReached() && SERVO_SM_QUICK_STOP_ACTIVE != fdb_ptr_->getServoState())
			{
			    ++target_reached_count_;
                if (target_reached_count_ > target_reached_max_)
                {
                    if (servo_comm_ptr_->emitServoCmdEnableOperation())
                    {
                        target_reached_count_ = 0;
                        ctrl_pdo_enble_ = true;
				        LogProducer::warn("AxisSm", "Axis[%d] transfer from STOPPING to STANDSTILL success", id_);
                        axis_state_ = AXIS_STATUS_STANDSTILL;
                    }
                    else
                    {
                        setError();
                        ErrorQueue::instance().push(AXIS_STATE_TRANSFER_INVALID);
                    }
                }
			}
		    break;
		case AXIS_STATUS_HOMING:
            if (!fdb_ptr_->isHoming())
            {
                axis_state_ = AXIS_STATUS_STANDSTILL;
                LogProducer::warn("AxisSm", "Axis[%d] transfer from HOMING to STANDSTILL success", id_);
            }
            else if (SERVO_SM_QUICK_STOP_ACTIVE == fdb_ptr_->getServoState())
            {
                axis_state_ = AXIS_STATUS_STOPPING;
                target_reached_count_ = 0;
                ctrl_pdo_enble_ = false;
                LogProducer::warn("AxisSm", "Axis[%d] transfer from HOMING to STOPPING success", id_);
            }
		    break;
		case AXIS_STATUS_DISCRETE_MOTION:
			if (fdb_ptr_->isTargetReached())
			{
			    ++target_reached_count_;
			    if (target_reached_count_ > target_reached_max_)
                {
                    target_reached_count_ = 0;
                    LogProducer::warn("AxisSm", "Axis[%d] transfer from DISCRETE_MOTION to STANDSTILL success", id_);
                    axis_state_ = AXIS_STATUS_STANDSTILL;
                }
			}
            if (SERVO_SM_QUICK_STOP_ACTIVE == fdb_ptr_->getServoState())
            {
                axis_state_ = AXIS_STATUS_STOPPING;
                target_reached_count_ = 0;
                ctrl_pdo_enble_ = false;
                LogProducer::warn("AxisSm", "Axis[%d] transfer from DISCRETE_MOTION to STOPPING success", id_);
            }
		    break;
		case AXIS_STATUS_CONTINUOUS_MOTION:
		    break;
		case AXIS_STATUS_SYNCHRONIZED_MOTION:
		    break;
		case AXIS_STATUS_UNKNOWN:
            axis_state_ = AXIS_STATUS_DISABLED;
            break;
        default:
            axis_state_ = AXIS_STATUS_UNKNOWN;
			break;
    }
	if (false == is_err_exist_ 
    && (SERVO_SM_SWITCH_ON_DISABLED == fdb_ptr_->getServoState() || SERVO_SM_SWITCHED_ON == fdb_ptr_->getServoState()))
	{
        if (axis_state_ != AXIS_STATUS_DISABLED)
        {
            LogProducer::warn("AxisSm", "Axis[%d] transfer from %s to DISABLED success", id_, getAxisStateString(axis_state_).c_str());
    	    axis_state_ = AXIS_STATUS_DISABLED;
            target_reached_count_ = 0;
            ctrl_pdo_enble_ = false;
        }
	}
}




