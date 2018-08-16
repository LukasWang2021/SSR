/*************************************************************************
	> File Name: motion_control_core_interface.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 15时35分37秒
 ************************************************************************/

#include <iostream>
#include <motion_control_core_interface.h>

namespace fst_mc
{

const static size_t MAX_ATTEMPTS = 100;

BareCoreInterface::BareCoreInterface(void)
{
    point_cache_.is_empty = true;
}

BareCoreInterface::~BareCoreInterface(void)
{}

bool BareCoreInterface::initInterface(void)
{
    if (core_interface_.init() != SUCCESS)
    {
        return false;
    }

    if (command_interface_.createChannel(COMM_REQ, COMM_IPC, "JTAC") != SUCCESS)
    {
        return false;
    }

    return true;
}

bool BareCoreInterface::isPointCacheEmpty(void)
{
    return point_cache_.is_empty;
}

bool BareCoreInterface::fillPointCache(TrajectoryPoint *points, size_t length, PointProperty property)
{
    if (point_cache_.is_empty && length > 0 && length <= JC_POINT_NUM)
    {
        for (size_t i = 0; i < length; i++)
        {
            memcpy(point_cache_.cache.points[i].positions, &points[i].angle, JOINT_NUM * sizeof(double));
            memcpy(point_cache_.cache.points[i].omega, &points[i].omega, JOINT_NUM * sizeof(double));
            memcpy(point_cache_.cache.points[i].inertia, &points[i].inertia, JOINT_NUM * sizeof(double));
            point_cache_.cache.points[i].point_position = points[i].level;
        }

        point_cache_.cache.total_points = length;
        point_cache_.is_empty = false;
        point_cache_.property = property;
        return true;
    }
    else
    {
        return false;
    }
}

bool BareCoreInterface::sendPoint(void)
{
    if (!point_cache_.is_empty)
    {
        if (core_interface_.sendBareCore(point_cache_.cache, point_cache_.property) == SUCCESS)
        {
            point_cache_.is_empty = true;
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

bool BareCoreInterface::getLatestJoint(Joint &joint, ServoState &state)
{
    FeedbackJointState fbjs;

    for (size_t cnt = 0; cnt < MAX_ATTEMPTS; cnt++)
    {
        if (core_interface_.recvBareCore(fbjs) == SUCCESS)
        {
            state = ServoState(fbjs.state);
            memcpy(&joint, fbjs.position, JOINT_NUM * sizeof(double));
            return true;
        }
        else
        {
            usleep(1000);
            continue;
        }
    }

    return false;
}

bool BareCoreInterface::resetBareCore(void)
{
    int cmd = 0;
    ServiceRequest req;

    req.req_id = JTAC_CMD_SID;
    memcpy(req.req_buff, &cmd, sizeof(cmd));

    for (size_t cnt = 0; cnt < MAX_ATTEMPTS; cnt++)
    {
        if (command_interface_.send(&req, sizeof(ServiceRequest), COMM_DONTWAIT) != SUCCESS)
        {
            usleep(1000);
            continue;
        }
        else
        {
            return true;
        }
    }

    return false;
}

bool BareCoreInterface::stopBareCore(void)
{
    int cmd = 1;
    ServiceRequest req;

    req.req_id = JTAC_CMD_SID;
    memcpy(req.req_buff, &cmd, sizeof(cmd));

    for (size_t cnt = 0; cnt < MAX_ATTEMPTS; cnt++)
    {
        if (command_interface_.send(&req, sizeof(ServiceRequest), COMM_DONTWAIT) != SUCCESS)
        {
            usleep(1000);
            continue;
        }
        else
        {
            return true;
        }
    }

    return false;
}

}

