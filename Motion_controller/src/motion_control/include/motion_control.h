#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H


#include "common_log.h"
#include "motion_control_param.h"
#include "device_manager.h"
#include "axis_group_manager.h"
#include "coordinate_manager.h"
#include "tool_manager.h"
#include "motion_control_arm_group.h"


namespace fst_mc
{
class MotionControl
{
public:
    MotionControl(fst_hal::DeviceManager* device_manager_ptr, AxisGroupManager* axis_group_manager_ptr,
                        fst_ctrl::CoordinateManager* coordinate_manager_ptr, fst_ctrl::ToolManager* tool_manager_ptr);
    ~MotionControl();

    // API for teaching
    ErrorCode setManualMode(ManualMode mode);
    ErrorCode setManualFrame(ManualFrame frame);
    ErrorCode manualMove(const ManualDirection *direction);
    ErrorCode manualMove(const Joint &joint);
    ErrorCode manualStop(void);

    // API for auto run
    // ...

    // API for Axis Group Enable/Disable/Halt/Stop/Reset
    ErrorCode stopGroup(void);
    ErrorCode resetGroup(void);
    ErrorCode sendPoint(void);


    // parameter access
    // ...
    
private:
    MotionControlParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_hal::DeviceManager* device_manager_ptr_;
    AxisGroupManager* axis_group_manager_ptr_;
    fst_ctrl::CoordinateManager* coordinate_manager_ptr_;
    fst_ctrl::ToolManager* tool_manager_ptr_;
    BaseGroup *group_ptr_;

    MotionControl();
};



}

#endif

