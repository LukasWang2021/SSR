/*************************************************************************
	> File Name: motion_control_core_interface.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 15时35分37秒
 ************************************************************************/

#include <iostream>
#include <motion_control_core_interface.h>

using namespace std;
using namespace fst_core_interface;
using namespace fst_comm_interface;

namespace fst_mc
{

const static size_t MAX_ATTEMPTS = 100;
const static unsigned char WRITE_BY_ID  = 0x2D;
const static unsigned char GET_ENCODER  = 0x70;
const static unsigned char READ_BY_ADDR     = 0x14;
const static unsigned char READ_BY_ID       = 0x1D;
const static unsigned char WRITE_BY_ADDR    = 0x24;


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

    if (jtac_param_interface_.createChannel(COMM_REQ, COMM_IPC, "servo_param") != SUCCESS)
    {
        return false;
    }

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
    if (point_cache_.is_empty && length > 0 && length <= JC_POINT_NUM)
    {
        for (size_t i = 0; i < length; i++)
        {
            memcpy(point_cache_.cache.points[i].positions, &points[i].angle, JOINT_NUM * sizeof(double));
            memcpy(point_cache_.cache.points[i].omega, &points[i].omega, JOINT_NUM * sizeof(double));
            memcpy(point_cache_.cache.points[i].inertia, &points[i].ma_cv_g, JOINT_NUM * sizeof(double));
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
    ServiceRequest req;
    req.req_id = JTAC_CMD_SID;
    int cmd = 0;
    memcpy(req.req_buff, &cmd, sizeof(cmd));
    return sendRequest(command_interface_, req);
}

bool BareCoreInterface::stopBareCore(void)
{
    ServiceRequest req;
    req.req_id = JTAC_CMD_SID;
    int cmd = 1;
    memcpy(req.req_buff, &cmd, sizeof(cmd));
    return sendRequest(command_interface_, req);
}

bool BareCoreInterface::setConfigData(int id, const vector<double> &data)
{
    ServiceRequest req;
    req.req_id = WRITE_BY_ID;
    int len = data.size();
    memcpy(&req.req_buff[0], (char*)&id, sizeof(id));
    memcpy(&req.req_buff[4], (char*)&len, sizeof(len));
    memcpy(&req.req_buff[8], (char*)&data[0], len * sizeof(double));
    return sendRequest(jtac_param_interface_ , req);
}

bool BareCoreInterface::getConfigData(int id, vector<double> &data)
{
    ServiceRequest req;
    req.req_id = READ_BY_ID;
    int len = data.size();
    memcpy(&req.req_buff[0], (char*)&id, sizeof(id));
    memcpy(&req.req_buff[4], (char*)&len, sizeof(len));

    if (sendRequest(jtac_param_interface_, req))
    {
        ServiceResponse res;

        if (recvResponse(jtac_param_interface_, res))
        {
            if (*((int*)(&res.res_buff[0])) == id && *((int*)(&res.res_buff[4])) == len)
            {
                memcpy(&data[0], &res.res_buff[8], len * sizeof(double));
                return true;
            }
        }
    }
    return false;
}

bool BareCoreInterface::getEncoder(vector<int> &data)
{
    ServiceRequest req;
    req.req_id = GET_ENCODER;
    int len = data.size();
    memcpy(&req.req_buff[0], (char*)&len, sizeof(len));

    if (sendRequest(jtac_param_interface_, req))
    {
        ServiceResponse res;

        if (recvResponse(jtac_param_interface_, res))
        {
            if (*((int*)(&res.res_buff[0])) == len)
            {
                memcpy(&data[0], &res.res_buff[4], len * sizeof(int));
                return true;
            }
        }
    }

    return false;
}

bool BareCoreInterface::sendRequest(CommInterface &comm, const ServiceRequest &req)
{
    for (size_t cnt = 0; cnt < MAX_ATTEMPTS; cnt++)
    {
        if (comm.send(&req, sizeof(ServiceRequest), COMM_DONTWAIT) != SUCCESS)
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

bool BareCoreInterface::recvResponse(CommInterface &comm, ServiceResponse &res)
{
    for (size_t cnt = 0; cnt < MAX_ATTEMPTS; cnt++)
    {
        if (comm.recv(&res, sizeof(ServiceResponse), COMM_DONTWAIT) != SUCCESS)
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

