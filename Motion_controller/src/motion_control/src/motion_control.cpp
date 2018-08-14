#include "motion_control.h"
#include <motion_control_arm_group.h>

#define FST_LOG(fmt, ...)       log_ptr_->log(fmt, ##__VA_ARGS__)
#define FST_INFO(fmt, ...)      log_ptr_->info(fmt, ##__VA_ARGS__)
#define FST_WARN(fmt, ...)      log_ptr_->warn(fmt, ##__VA_ARGS__)
#define FST_ERROR(fmt, ...)     log_ptr_->error(fmt, ##__VA_ARGS__)

using namespace fst_mc;
using namespace fst_hal;
using namespace fst_ctrl;

MotionControl::MotionControl(DeviceManager* device_manager_ptr, AxisGroupManager* axis_group_manager_ptr,
                                CoordinateManager* coordinate_manager_ptr, ToolManager* tool_manager_ptr):
    device_manager_ptr_(device_manager_ptr), axis_group_manager_ptr_(axis_group_manager_ptr), 
    coordinate_manager_ptr_(coordinate_manager_ptr), tool_manager_ptr_(tool_manager_ptr),
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    assert(log_ptr_ != NULL);
    param_ptr_ = new MotionControlParam();
    assert(param_ptr_ != NULL);
    group_ptr_ = new ArmGroup(log_ptr_);
    assert(group_ptr_ != NULL);
    //group_ptr_ = new ScalaGroup(log_ptr_);
    if (group_ptr_->initGroup() != SUCCESS)
    {
        FST_ERROR("Fail to inti group");
    }
}
                    
MotionControl::~MotionControl()
{}

MotionControl::MotionControl():
    device_manager_ptr_(NULL), axis_group_manager_ptr_(NULL), 
    coordinate_manager_ptr_(NULL), tool_manager_ptr_(NULL)
{}


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

ErrorCode MotionControl::sendPoint(void)
{
    return group_ptr_->sendPoint();
}

