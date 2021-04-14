#include <unistd.h>
#include <string.h>  
#include "group.h"

using namespace group_space;
using namespace axis_space;
using namespace system_model_space;
using namespace log_space;
using namespace base_space;

Group::Group(int32_t id):
    id_(id),
    group_error_(0)
{
}

Group::~Group(void)
{

}

bool Group::init(servo_comm_space::ServoCpuCommBase* cpu_comm_ptr, system_model_space::GroupModel_t* db_ptr,
    system_model_space::GroupConfig_t* group_config_ptr)
{
	//init ptr;
	if (cpu_comm_ptr == NULL || db_ptr == NULL || group_config_ptr == NULL)
		return false;
    cpu_comm_ptr_ = cpu_comm_ptr;
	db_ptr_ = db_ptr;
    group_config_ptr_ = group_config_ptr;

	//init fdb
	if (!fdb_.init(id_, &axis_group_))
	{
	    LogProducer::error("Group","Group[%d] failed to init group feedback", id_);
		return false;
	}

    // init state machine.
    if (!sm_.init(id_, &fdb_, db_ptr_, &axis_group_))
    {
    	LogProducer::error("Group","Group[%d] failed to init group state machine", id_);
		return false;
    }

    LogProducer::warn("Group", "Group[%d] init success", id_);
	return true;	
}

bool Group::reloadAlgorithms(void)
{

    return true;
}

ErrorCode Group::mcAddAxisToGroup(int32_t id_in_group, axis_space::Axis &axis_ref)
{
    GroupStatus_e status = sm_.getGroupStatus();
    if (status != GROUP_STATUS_UNKNOWN && status != GROUP_STATUS_DISABLED 
        && status != GROUP_STATUS_STANDBY && status != GROUP_STATUS_ERROR_STOP)
    {
        LogProducer::warn("Group", "Group[%d] mcAddAxisToGroup called failed when group_status is %s", id_, sm_.getGroupStatusString(status).c_str());
        return GROUP_STATE_EXE_INVALID;
    }

    std::pair<std::map<int32_t, Axis*>::iterator, bool> ret = axis_group_.insert(std::pair<int, Axis*>(id_in_group, &axis_ref));
    if (!ret.second)
    {
        LogProducer::error("Group", "Group[%d] mcAddAxisToGroup failed to add existed axis, id_in_group is %d", id_, id_in_group);
        return GROUP_ADD_AXIS_FAILED;
    }
    axis_ref.setAxisInGroup(true);
    LogProducer::info("Group", "Group[%d] mcAddAxisToGroup success, id_in_group is %d, axis_id is %d", id_, id_in_group, axis_ref.getID());
    return SUCCESS;
}

ErrorCode Group::mcRemoveAxisFromGroup(int32_t id_in_group)
{
    GroupStatus_e status = sm_.getGroupStatus();
    if (status != GROUP_STATUS_DISABLED && status != GROUP_STATUS_STANDBY && status != GROUP_STATUS_ERROR_STOP)
    {
        LogProducer::warn("Group", "Group[%d] mcRemoveAxisFromGroup called failed when group_status is %s", id_, sm_.getGroupStatusString(status).c_str());
        return GROUP_STATE_EXE_INVALID;
    }
    std::map<int32_t, Axis*>::iterator it;
    it = axis_group_.find(id_in_group);
    if (it != axis_group_.end())
    {      
        it->second->setAxisInGroup(false);
        LogProducer::info("Group", "Group[%d] mcRemoveAxisFromGroup success, id_in_group is %d", id_, id_in_group);
        axis_group_.erase(it);
        return SUCCESS;
    }
    LogProducer::error("Group", "Group[%d] mcRemoveAxisFromGroup failed to remove none-existed axis, id_in_group is %d", id_, id_in_group);
    return GROUP_REMOVE_AXIS_FAILED;
}

ErrorCode Group::mcUngroupAllAxes(void)
{
    GroupStatus_e status = sm_.getGroupStatus();
    if (status != GROUP_STATUS_DISABLED && status != GROUP_STATUS_STANDBY && status != GROUP_STATUS_ERROR_STOP)
    {
        LogProducer::warn("Group", "Group[%d] mcUngroupAllAxes called failed when group_status is %s", id_, sm_.getGroupStatusString(status).c_str());
        return GROUP_STATE_EXE_INVALID;
    }

    std::map<int32_t, Axis*>::iterator it;
    for (it = axis_group_.begin(); it != axis_group_.end(); ++it)
    {        
        it->second->setAxisInGroup(false);
    }
    axis_group_.clear();
    LogProducer::info("Group", "Group[%d] mcUngroupAllAxes success", id_);
    return SUCCESS;
}

ErrorCode Group::mcGroupReset(void)
{
    clearBQ();

    // to update application params
    if (!reloadSystemModel())
    {
        LogProducer::error("Group", "group[%d] load parameters failed", id_);
        return GROUP_ALG_NOT_DEFINED;
    }
            
    ErrorCode ret = SUCCESS;
    std::map<int32_t, Axis*>::iterator it;
    for (it = axis_group_.begin(); it != axis_group_.end(); ++it)
    {
        ErrorCode err = it->second->mcReset();
        if (err != SUCCESS)
            ret = err;
    }
    clearError();

    if (ret == SUCCESS)
    {
        LogProducer::info("Group", "Group[%d] mcGroupReset success", id_);
    }
    else
    {
        LogProducer::error("Group", "Group[%d] mcGroupReset failed, err = 0x%x", id_, ret);
    }
    return ret;
}

ErrorCode Group::mcGroupEnable(void)
{
    clearBQ();

    // to update application params
    if (!reloadSystemModel())
    {
        LogProducer::error("Group", "group[%d] load parameters failed", id_);
        return GROUP_ALG_NOT_DEFINED;
    }
    
    ErrorCode ret = SUCCESS;
    std::map<int32_t, Axis*>::iterator it;
    for (it = axis_group_.begin(); it != axis_group_.end(); ++it)
    {
        ErrorCode err = it->second->mcPower(true);
        if (err != SUCCESS)
            ret = err;
    }

    if (ret == SUCCESS)
    {
        LogProducer::info("Group", "Group[%d] mcGroupEnable success", id_);
    }
    else
    {
        LogProducer::error("Group", "Group[%d] mcGroupEnable failed, err = 0x%x", id_, ret);
    }    
    return ret;
}
ErrorCode Group::mcGroupDisable(void)
{
    ErrorCode ret = SUCCESS;
    std::map<int32_t, Axis*>::iterator it;
    for (it = axis_group_.begin(); it != axis_group_.end(); ++it)
    {
        ErrorCode err = it->second->mcPower(false);
        if (err != SUCCESS)
            ret = err;
    }

    if (ret == SUCCESS)
    {
        LogProducer::info("Group", "Group[%d] mcGroupDisable success", id_);
    }
    else
    {
        LogProducer::error("Group", "Group[%d] mcGroupDisable failed, err = 0x%x", id_, ret);
    }    
    return ret;
}

ErrorCode Group::mcGroupStop(double dec, double jerk)
{
    GroupStatus_e status = sm_.getGroupStatus();
    if (status == GROUP_STATUS_DISABLED || status == GROUP_STATUS_ERROR_STOP)
    {
        LogProducer::warn("Group", "Group[%d] mcGroupStop called failed when group_status is %s", id_, sm_.getGroupStatusString(status).c_str());
        return GROUP_STATE_EXE_INVALID;
    }

    ErrorCode ret = SUCCESS;
    std::map<int32_t, Axis*>::iterator it;
    for (it = axis_group_.begin(); it != axis_group_.end(); ++it)
    {
        ErrorCode err = it->second->mcStop();
        if (err != SUCCESS)
            ret = err;
    }
    
    if (ret == SUCCESS)
    {
        LogProducer::info("Group", "Group[%d] mcGroupStop success", id_);
    }
    else
    {
        LogProducer::error("Group", "Group[%d] mcGroupStop failed, err = 0x%x", id_, ret);
    }    
    return ret;
}


ErrorCode Group::mcGroupReadError(void)
{
    if (group_error_ == SUCCESS)
        group_error_ = fdb_.getAxisError();
    return group_error_;
}
ErrorCode Group::mcGroupReadStatus(GroupStatus_e &status, bool &in_position)
{
    status = sm_.getGroupStatus();
    in_position = fdb_.isTargetReached();
    return SUCCESS;
}
ErrorCode Group::mcGroupReadConfiguration(int32_t id_in_group, axis_space::Axis &axis_ref)
{
    std::map<int32_t, Axis*>::iterator it = axis_group_.find(id_in_group);
    if (it != axis_group_.end())
    {
        memcpy(&axis_ref, it->second, sizeof(Axis));
        LogProducer::info("Group", "Group[%d] mcGroupReadConfiguration success, id_in_group is %d", id_, id_in_group);
        return SUCCESS;
    }
    LogProducer::error("Group", "Group[%d] mcGroupReadConfiguration failed, id_in_group(%d) is not existed", id_, id_in_group);
    return GROUP_READ_CONFIG_FAILED;
}


void Group::processStateMachine()
{  
    sm_.processStatemachine();
}

void Group::processFdbPdo()
{
    fdb_.processAxesFdb();
}

int32_t Group::getID()
{
    return id_;
}

void Group::setError(ErrorCode err)
{
    sm_.setError();
    group_error_ = err;
    ErrorQueue::instance().push(group_error_);
}

void Group::clearError(void)
{
    sm_.clearError();
    group_error_ = SUCCESS;
}

