#include "axis_group_manager.h"
#include <iostream>


using namespace fst_mc;


AxisGroupManager::AxisGroupManager()
{

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


