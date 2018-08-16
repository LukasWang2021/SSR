#include "motion_control.h"
#include <motion_control_arm_group.h>

using namespace fst_base;
using namespace fst_mc;
using namespace fst_hal;
using namespace fst_ctrl;

MotionControl::MotionControl():
    log_ptr_(NULL), 
    param_ptr_(NULL),
    device_manager_ptr_(NULL), 
    axis_group_manager_ptr_(NULL), 
    coordinate_manager_ptr_(NULL), 
    tool_manager_ptr_(NULL),
    error_monitor_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    assert(log_ptr_ != NULL);
    param_ptr_ = new MotionControlParam();
    assert(param_ptr_ != NULL);
    group_ptr_ = new ArmGroup(log_ptr_);
    assert(group_ptr_ != NULL);
    //group_ptr_ = new ScalaGroup(log_ptr_);

}

MotionControl::~MotionControl()
{
    if (log_ptr_ != NULL)   {delete log_ptr_; log_ptr_ = NULL;};
    if (param_ptr_ != NULL)   {delete param_ptr_; param_ptr_ = NULL;};
    if (group_ptr_ != NULL)   {delete group_ptr_; group_ptr_ = NULL;};
}

ErrorCode MotionControl::init(fst_hal::DeviceManager* device_manager_ptr, AxisGroupManager* axis_group_manager_ptr,
                        fst_ctrl::CoordinateManager* coordinate_manager_ptr, fst_ctrl::ToolManager* tool_manager_ptr,
                        fst_base::ErrorMonitor* error_monitor_ptr)
{
    device_manager_ptr_ = device_manager_ptr;
    axis_group_manager_ptr_ = axis_group_manager_ptr;
    coordinate_manager_ptr_ = coordinate_manager_ptr;
    tool_manager_ptr_ = tool_manager_ptr;
    error_monitor_ptr_ = error_monitor_ptr;
    ErrorCode  err = group_ptr_->initGroup(error_monitor_ptr);    
    
    if (err == SUCCESS)
    {
        FST_INFO("Initialize motion group success.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to init motion group");
        return err;
    }
}


ErrorCode MotionControl::setManualMode(ManualMode mode)
{
    return group_ptr_->setManualMode(mode);
}

ErrorCode MotionControl::setManualFrame(ManualFrame frame)
{
    return group_ptr_->setManualFrame(frame);
}

ErrorCode MotionControl::manualMove(const ManualDirection *direction)
{
    return group_ptr_->manualMove(direction);
}

ErrorCode MotionControl::manualMove(const Joint &joint)
{
    return group_ptr_->manualMove(joint);
}

ErrorCode MotionControl::manualStop(void)
{
    return group_ptr_->manualStop();
}

ErrorCode MotionControl::stopGroup(void)
{
    return group_ptr_->stopGroup();
}

ErrorCode MotionControl::resetGroup(void)
{
    return group_ptr_->resetGroup();
}

GroupState MotionControl::getGroupState(void)
{
    return group_ptr_->getGroupState();
}

ServoState MotionControl::getServoState(void)
{
    return group_ptr_->getServoState();
}

Joint MotionControl::getServoJoint(void)
{
    return group_ptr_->getLatestJoint();
}

void MotionControl::getServoJoint(Joint &joint)
{
    group_ptr_->getLatestJoint(joint);
}

size_t MotionControl::getFIFOLength(void)
{
    return group_ptr_->getFIFOLength();
}

void MotionControl::rtTask(void)
{
    ErrorCode err = group_ptr_->realtimeTask();

    if (err == SUCCESS)
    {
        FST_INFO("RT task quit with SUCCESS");
    }
    else
    {
        FST_ERROR("RT task quit with error = 0x%llx", err);
    }
}

