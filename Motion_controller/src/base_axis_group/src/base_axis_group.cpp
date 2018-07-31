#include "base_axis_group.h"
#include <iostream>


using namespace fst_mc;
using namespace fst_hal;

BaseAxisGroup::BaseAxisGroup():
    is_valid_(false)
{

}

BaseAxisGroup::~BaseAxisGroup()
{

}

bool BaseAxisGroup::addAxisToGroup(int device_index, BaseDevice* device_ptr)
{
    AxisInfo axis_info;
    axis_info.device_index = device_index;
    axis_info.device_ptr = device_ptr;
    axis_list_.push_back(axis_info);
    return true;
}

bool BaseAxisGroup::removeAxisFromGroup(int device_index)
{
    return true;
}

bool BaseAxisGroup::unGroupAllAxes()
{
    return true;
}

bool BaseAxisGroup::bindModelToGroup(AxisGroupModel& model)
{
    return true;
}

bool BaseAxisGroup::unbindModelToGroup()
{
    return true;
}

bool BaseAxisGroup::checkValid()
{
    return false;
}

bool BaseAxisGroup::isValid()
{
    return is_valid_;
}

BaseDevice* BaseAxisGroup::getAxisByPos(int pos)
{
    return NULL;
}

AxisGroupModel* BaseAxisGroup::getModel()
{
    return NULL;
}


