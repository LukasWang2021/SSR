#include "group_fdb.h"

using namespace group_space;
using namespace axis_space;
using namespace log_space;

GroupFdb::GroupFdb(void):
    servos_status_(SERVO_SM_SWITCH_ON_DISABLED),
    axis_error_(0)
{
    
}

GroupFdb::~GroupFdb(void)
{
}

bool GroupFdb::init(int32_t id, std::map<int, axis_space::Axis*>* axis_group_ptr)
{
    id_ = id;
    axis_group_ptr_ = axis_group_ptr;
	
	return true;
}

void GroupFdb::processAxesFdb()
{
    if (axis_group_ptr_->size() == 0)
    {
        servos_status_ = SERVO_SM_SWITCH_ON_DISABLED;
        return;
    }

    std::map<int, Axis*>::reverse_iterator it;
    
    uint32_t time_stamp = 0;    
    for (it = axis_group_ptr_->rbegin(); it != axis_group_ptr_->rend(); ++it)
    {        
        if (it == axis_group_ptr_->rbegin())
        {
            it->second->processFdbPdoCurrent(&time_stamp);
        }
        else
        {
            it->second->processFdbPdoSync(time_stamp);
        }
    }
    
    ServoSm_e status = SERVO_SM_SWITCH_ON_DISABLED;
    int32_t error = 0;
    for (it = axis_group_ptr_->rbegin(); it != axis_group_ptr_->rend(); ++it)
    {
        status = it->second->readServoStatus();
        switch (status)
        {
            case SERVO_SM_FAULT:
                it->second->mcReadAxisError(error);
                axis_error_ = error;
                servos_status_ = SERVO_SM_FAULT;
                return;
            case SERVO_SM_SWITCH_ON_DISABLED:
                servos_status_ = SERVO_SM_SWITCH_ON_DISABLED;
                return;
            case SERVO_SM_READY_TO_SWITCH_ON:
                servos_status_ = SERVO_SM_READY_TO_SWITCH_ON;
                return;
            case SERVO_SM_SWITCHED_ON:
                servos_status_ = SERVO_SM_SWITCHED_ON;
                return;
            case SERVO_SM_QUICK_STOP_ACTIVE:
                servos_status_ = SERVO_SM_QUICK_STOP_ACTIVE;
                return;
            default: break;
        }
    }
    servos_status_ = SERVO_SM_OPERATION_ENABLED;
}

ServoSm_e GroupFdb::getServosStatus(void)
{
    return servos_status_;
}

ErrorCode GroupFdb::getAxisError(void)
{
    return axis_error_;
}

bool GroupFdb::isTargetReached(void)
{
    std::map<int, Axis*>::iterator it;
    for (it = axis_group_ptr_->begin(); it != axis_group_ptr_->end(); ++it)
    {            
        if (!it->second->rtmIsTargetReached())
        {
            return false;
        }
    }
    return true;
}



