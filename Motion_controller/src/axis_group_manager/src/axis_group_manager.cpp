#include "axis_group_manager.h"
#include <iostream>


using namespace fst_mc;


AxisGroupManager::AxisGroupManager():
    param_ptr_(NULL),
    log_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new AxisGroupManagerParam();
}

AxisGroupManager::~AxisGroupManager()
{

}

bool AxisGroupManager::init()
{
    return true;
}

bool AxisGroupManager::addAxisGroup(int axis_group_index, AxisGroupConfig device_config)
{
    return true;
}

bool AxisGroupManager::removeAxisGroup(int axis_group_index)
{
    return true;
}

BaseAxisGroup* AxisGroupManager::getAxisGroupByIndex(int axis_group_index)
{
    return NULL;
}


