/*************************************************************************
	> File Name: motion_control_core_interface.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 15时35分37秒
 ************************************************************************/

#include <iostream>
#include <motion_control_core_interface.h>

using namespace std;
using namespace basic_alg;
using namespace axis_space;
using namespace group_space;

namespace group_space
{


BareCoreInterface::BareCoreInterface(void)
{
    joint_num_ = 0;
    point_cache_.is_empty = true;
}

BareCoreInterface::~BareCoreInterface(void)
{}

bool BareCoreInterface::initInterface(uint32_t joint_num, std::map<int32_t, axis_space::Axis*>* axis_group_ptr, GroupSm* sm_ptr,
    servo_comm_space::ServoCpuCommBase* cpu_comm_ptr)
{
    joint_num_ = joint_num;
    axis_group_ptr_ = axis_group_ptr;
    sm_ptr_ = sm_ptr;
    cpu_comm_ptr_ = cpu_comm_ptr;
    if (joint_num_ != axis_group_ptr->size())
        return false;
    
    return true;
}

bool BareCoreInterface::isPointCacheEmpty(void)
{
    return point_cache_.is_empty;
}

bool BareCoreInterface::clearPointCache(void)
{
    point_cache_.is_empty = true;
    return true;
}

bool BareCoreInterface::fillPointCache(TrajectoryPoint *points, size_t length, PointProperty property)
{
    return true;
    // if (point_cache_.is_empty && length > 0 && length <= JC_POINT_NUM)
    // {
    //     for (size_t i = 0; i < length; i++)
    //     {
    //         memcpy(point_cache_.cache.points[i].angle, &points[i].state.angle, JOINT_NUM * sizeof(double));
    //         memcpy(point_cache_.cache.points[i].omega, &points[i].state.omega, JOINT_NUM * sizeof(double));
    //         memcpy(point_cache_.cache.points[i].alpha, &points[i].state.alpha, JOINT_NUM * sizeof(double));
    //         memcpy(point_cache_.cache.points[i].inertia, &points[i].state.torque, JOINT_NUM * sizeof(double));
    //         point_cache_.cache.points[i].point_position = points[i].level;
    //     }

    //     point_cache_.cache.total_points = length;
    //     point_cache_.is_empty = false;
    //     point_cache_.property = property;
    //     return true;
    // }
    // else
    // {
    //     return false;
    // }
}

bool BareCoreInterface::sendPoint(void)
{
    return true;
    // if (!point_cache_.is_empty)
    // {
    //     if (core_interface_.sendBareCore(point_cache_.cache, point_cache_.property) == SUCCESS)
    //     {
    //         point_cache_.is_empty = true;
    //         return true;
    //     }
    //     else
    //     {
    //         return false;
    //     }
    // }
    // else
    // {
    //     return false;
    // }

    //point_cache_.is_empty = true;
    //return true;
}

bool BareCoreInterface::getLatestJoint(Joint &joint, uint32_t (&encoder_state)[NUM_OF_JOINT], ServoState &state)
{
    std::map<int32_t, Axis*>::iterator it;
    uint32_t i = 0;
    for (it = axis_group_ptr_->begin(); it != axis_group_ptr_->end(); ++it, ++i)
    {
        it->second->mcReadActualPosition(joint[i]);
        encoder_state[i] = 0;
    }
    state = (ServoState)sm_ptr_->getGroupStatus();

    return true;
}


bool BareCoreInterface::setConfigData(int id, const vector<double> &data)
{
    return true;
}

bool BareCoreInterface::setConfigData(int id, const vector<int> &data)
{
    return true;
}

bool BareCoreInterface::getConfigData(int id, vector<double> &data)
{
    return true;
}

bool BareCoreInterface::getEncoder(vector<int> &data)
{
    return  true;
}

bool BareCoreInterface::getEncoderError(vector<int> &data)
{
    return true;
}

bool BareCoreInterface::resetEncoderError(size_t index)
{
    return true;
}

bool BareCoreInterface::resetEncoderError(void)
{
    return true;
}

bool BareCoreInterface::getControlPosition(double *data, size_t size)
{
    return true;
}



}

