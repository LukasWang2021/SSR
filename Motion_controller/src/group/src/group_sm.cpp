#include <string.h>  
#include "group_sm.h"

using namespace group_space;
using namespace axis_space;
using namespace log_space;
using namespace base_space;
using namespace system_model_space;

GroupSm::GroupSm(void):
	group_state_(GROUP_STATUS_UNKNOWN),
	is_err_exist_(false),
	id_(0),
	target_reached_count_(0),
	target_reached_max_(0),
	ctrl_pdo_enble_(false)
{
    
}

GroupSm::~GroupSm(void)
{

}

bool GroupSm::init(int32_t id, GroupFdb* fdb_ptr, GroupModel_t* db_ptr, std::map<int, axis_space::Axis*>* axis_group_ptr)
{
    if (fdb_ptr == NULL || db_ptr == NULL)
        return false;
    
    id_ = id;
    fdb_ptr_ = fdb_ptr;
    db_ptr_ = db_ptr;
    axis_group_ptr_ = axis_group_ptr;

    if (!db_ptr->application_ptr->get(GroupApplication1000__target_reached_count_max, &target_reached_max_))
        return false;
    LogProducer::info("GroupSm", "target_reached_count_max is %d", target_reached_max_);

	return true;
}


void GroupSm::processStatemachine()
{
    processError();
	processGroupState();
}

bool GroupSm::transferStateToGroupStopping(void)
{
    if (group_state_ == GROUP_STATUS_ERROR_STOP && group_state_ == GROUP_STATUS_DISABLED)
    {
        LogProducer::warn("GroupSm", "Group[%d] transfer from %s to GROUP_STOPPING failed", id_, getGroupStatusString(group_state_).c_str());
		return false;
    }
    
    LogProducer::warn("GroupSm", "Group[%d] transfer from %s to GROUP_STOPPING success", id_, getGroupStatusString(group_state_).c_str());
	group_state_ = GROUP_STATUS_STOPPING;
    target_reached_count_ = 0;
    ctrl_pdo_enble_ = false;
    return true;
}

bool GroupSm::transferStateToGroupHoming(void)
{
    if (group_state_ == GROUP_STATUS_STANDBY)
    {
		LogProducer::warn("GroupSm", "Group[%d] transfer from %s to GROUP_HOMING success", id_, getGroupStatusString(group_state_).c_str());
    	group_state_ = GROUP_STATUS_HOMING;
        target_reached_count_ = 0;
		return true;
    }
	LogProducer::warn("GroupSm", "Group[%d] transfer from %s to GROUP_HOMING failed", id_, getGroupStatusString(group_state_).c_str());
	return false;
}

bool GroupSm::transferStateToGroupMoving(void)
{
    if (group_state_ == GROUP_STATUS_STANDBY || group_state_ == GROUP_STATUS_MOVING)
    {
		LogProducer::warn("GroupSm", "Group[%d] transfer from %s to GROUP_MOVING success", id_, getGroupStatusString(group_state_).c_str());
	    group_state_ = GROUP_STATUS_MOVING;
        target_reached_count_ = 0;
        return true;
    }
	LogProducer::warn("GroupSm", "Group[%d] transfer from %s to GROUP_MOVING failed", id_, getGroupStatusString(group_state_).c_str());
    return false;
}

bool GroupSm::transferStateToGroupStandby(void)
{
    if (group_state_ == GROUP_STATUS_STANDBY)
    {
        LogProducer::warn("GroupSm", "Group[%d] transfer from %s to GROUP_STANDBY success", id_, getGroupStatusString(group_state_).c_str());
        group_state_ = GROUP_STATUS_STANDBY;
        target_reached_count_ = 0;
        return true;
    }

    if (group_state_ == GROUP_STATUS_MOVING)
    {
		LogProducer::warn("GroupSm", "Group[%d] transfer from %s to GROUP_STANDBY success", id_, getGroupStatusString(group_state_).c_str());
	    group_state_ = GROUP_STATUS_STANDBY;
        target_reached_count_ = 0;
        return true;
    }
	LogProducer::warn("GroupSm", "Group[%d] transfer from %s to GROUP_STANDBY failed", id_, getGroupStatusString(group_state_).c_str());
    return false;
}

void GroupSm::setError(void)
{
    is_err_exist_ = true;
}

void GroupSm::clearError(void)
{
    is_err_exist_ = false;
}


GroupStatus_e GroupSm::getGroupStatus(void)
{
    return group_state_;
}

std::string GroupSm::getGroupStatusString(GroupStatus_e group_status)
{
    switch(group_status)
    {
        case GROUP_STATUS_UNKNOWN:                 return std::string("UNKNOWN");
        case GROUP_STATUS_ERROR_STOP:              return std::string("GROUP_ERROR_STOP");
        case GROUP_STATUS_DISABLED:                return std::string("GROUP_DISABLED");
        case GROUP_STATUS_STANDBY:                 return std::string("GROUP_STANDBY");
        case GROUP_STATUS_STOPPING:                return std::string("GROUP_STOPPING");
        case GROUP_STATUS_HOMING:                  return std::string("GROUP_HOMING");
        case GROUP_STATUS_MOVING:                  return std::string("GROUP_MOVING");
        default:                                   return std::string("Unknown");
    }
}

bool GroupSm::isCtrlPdoEnable(void)
{
    return ctrl_pdo_enble_;
}


//private functions
void GroupSm::processError(void)
{
    if (true == is_err_exist_ || SERVO_SM_FAULT == fdb_ptr_->getServosStatus())
    {
        if (group_state_ != GROUP_STATUS_ERROR_STOP)
        {
            //upload servo error
            ErrorCode error = fdb_ptr_->getAxisError();
            ErrorQueue::instance().push(error);
            
            LogProducer::warn("GroupSm", "Group[%d] transfer from %s to ERRORSTOP success", id_, getGroupStatusString(group_state_).c_str());
            group_state_ = GROUP_STATUS_ERROR_STOP;
            target_reached_count_ = 0;
            ctrl_pdo_enble_ = false;
        }
    }
}

void GroupSm::processGroupState(void)
{
    switch(group_state_)
    {
        case GROUP_STATUS_ERROR_STOP:
			if (false == is_err_exist_ && SERVO_SM_OPERATION_ENABLED == fdb_ptr_->getServosStatus())
			{
			    group_state_ = GROUP_STATUS_STANDBY;
                ctrl_pdo_enble_ = true;
				LogProducer::warn("GroupSm", "Group[%d] transfer from GROUP_ERROR_STOP to GROUP_STANDBY success", id_);
			}
			break;
		case GROUP_STATUS_DISABLED:
			if (SERVO_SM_OPERATION_ENABLED == fdb_ptr_->getServosStatus())
			{
			    group_state_ = GROUP_STATUS_STANDBY;
                ctrl_pdo_enble_ = true;
				LogProducer::warn("GroupSm", "Group[%d] transfer from GROUP_DISABLED to GROUP_STANDBY success", id_);
			}
			break;
		case GROUP_STATUS_STANDBY:
            if (axis_group_ptr_->size() == 0 || SERVO_SM_SWITCH_ON_DISABLED == fdb_ptr_->getServosStatus())
            {
                group_state_ = GROUP_STATUS_DISABLED;
                ctrl_pdo_enble_ = false;
                LogProducer::warn("GroupSm", "Group[%d] transfer from GROUP_STANDBY to GROUP_DISABLED success", id_);
            }
		    break;
		case GROUP_STATUS_STOPPING:
            if (fdb_ptr_->isTargetReached() && SERVO_SM_QUICK_STOP_ACTIVE != fdb_ptr_->getServosStatus())
            {
                ++target_reached_count_;
			    if (target_reached_count_ > target_reached_max_)
                {
                    std::map<int, Axis*>::iterator it;
                    for (it = axis_group_ptr_->begin(); it != axis_group_ptr_->end(); ++it)
                    {
                        it->second->getServoCommPtr()->emitServoCmdEnableOperation();
                    }
                    if (SERVO_SM_OPERATION_ENABLED == fdb_ptr_->getServosStatus())
                    {
                        target_reached_count_ = 0;
				        LogProducer::warn("GroupSm", "Group[%d] transfer from GROUP_STOPPING to GROUP_STANDBY success", id_);                   
                        group_state_ = GROUP_STATUS_STANDBY;
                    }
                }
            }
            break;
		case GROUP_STATUS_HOMING:
		case GROUP_STATUS_MOVING:
			if (fdb_ptr_->isTargetReached())
			{
			    ++target_reached_count_;
			    if (target_reached_count_ > target_reached_max_)
                {
                    target_reached_count_ = 0;
				    LogProducer::warn("GroupSm", "Group[%d] transfer from GROUP_MOVING to GROUP_STANDBY success", id_);                   
                    group_state_ = GROUP_STATUS_STANDBY;
                    ctrl_pdo_enble_ = true;
                }                            
			}
		    break;
        case GROUP_STATUS_UNKNOWN:
            group_state_ = GROUP_STATUS_DISABLED;
            break;
		default:
		    group_state_ = GROUP_STATUS_UNKNOWN;
			break;
    }
}




