#include "motion_control.h"

using namespace fst_mc;
using namespace fst_hal;
using namespace fst_ctrl;

MotionControl::MotionControl(DeviceManager* device_manager_ptr, AxisGroupManager* axis_group_manager_ptr,
                                CoordinateManager* coordinate_manager_ptr, ToolManager* tool_manager_ptr):
    device_manager_ptr_(device_manager_ptr), axis_group_manager_ptr_(axis_group_manager_ptr), 
    coordinate_manager_ptr_(coordinate_manager_ptr), tool_manager_ptr_(tool_manager_ptr)
{

}
                    
MotionControl::~MotionControl()
{

}

MotionControl::MotionControl():
    device_manager_ptr_(NULL), axis_group_manager_ptr_(NULL), 
    coordinate_manager_ptr_(NULL), tool_manager_ptr_(NULL)
{

}

