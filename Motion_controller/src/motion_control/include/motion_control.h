#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H


#include "common_log.h"
#include "motion_control_param.h"
#include "device_manager.h"
#include "axis_group_manager.h"
#include "coordinate_manager.h"
#include "tool_manager.h"
#include "motion_control_arm_group.h"
#include "error_monitor.h"


namespace fst_mc
{
class MotionControl
{
public:
    MotionControl();
    ~MotionControl();

    ErrorCode init(fst_hal::DeviceManager* device_manager_ptr, AxisGroupManager* axis_group_manager_ptr,
                    fst_ctrl::CoordinateManager* coordinate_manager_ptr, fst_ctrl::ToolManager* tool_manager_ptr, 
                    fst_base::ErrorMonitor *error_monitor_ptr);

    // API for teaching
    ErrorCode setManualFrame(ManualFrame frame);
    ErrorCode doStepManualMove(const GroupDirection &direction);
    ErrorCode doContinuousManualMove(const GroupDirection &direction);
    ErrorCode doGotoPointManualMove(const Joint &joint);
    ErrorCode doGotoPointManualMove(const PoseEuler &pose);
    ErrorCode manualStop(void);

    // API for auto run
    // ...

    // API for Axis Group Enable/Disable/Halt/Stop/Reset
    ErrorCode stopGroup(void);
    ErrorCode resetGroup(void);

    // more API
    GroupState getGroupState(void);
    ServoState getServoState(void);
    Joint   getServoJoint(void);
    void    getServoJoint(Joint &joint);
    size_t  getFIFOLength(void);

    void rtTask(void);


    // parameter access
    // ...
    
private:
    MotionControlParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_hal::DeviceManager* device_manager_ptr_;
    AxisGroupManager* axis_group_manager_ptr_;
    fst_ctrl::CoordinateManager* coordinate_manager_ptr_;
    fst_ctrl::ToolManager* tool_manager_ptr_;
    fst_base::ErrorMonitor* error_monitor_ptr_;
    BaseGroup *group_ptr_;

};



}

#endif

