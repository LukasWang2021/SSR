/*************************************************************************
	> File Name: motion_control_core_interface.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 15时35分37秒
 ************************************************************************/

#include <iostream>
#include <motion_control_core_interface.h>

using namespace std;
//TODOusing namespace fst_core_interface;
//TODOusing namespace fst_comm_interface;
using namespace basic_alg;

namespace fst_mc
{

const static size_t MAX_ATTEMPTS = 100;


BareCoreInterface::BareCoreInterface(void)
{
    joint_num_ = 0;
    point_cache_.is_empty = true;
}

BareCoreInterface::~BareCoreInterface(void)
{}

bool BareCoreInterface::initInterface(uint32_t joint_num)
{
    // if (core_interface_.init() != SUCCESS)
    // {
    //     return false;
    // }

    // if (command_interface_.createChannel(COMM_REQ, COMM_IPC, "JTAC") != SUCCESS)
    // {
    //     return false;
    // }

    //if (jtac_param_interface_.createChannel(COMM_REQ, COMM_IPC, "servo_param") != SUCCESS)
    //{
    //    return false;
    //}

    joint_num_ = joint_num;
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
    return true;
    // FeedbackJointState fbjs;

    // for (uint32_t cnt = 0; cnt < MAX_ATTEMPTS; cnt++)
    // {
    //     if (core_interface_.recvBareCore(fbjs) == SUCCESS)
    //     {
    //         state = ServoState(fbjs.state);

    //         for (uint32_t j = 0; j < joint_num_; j++)
    //         {
    //             joint[j] = fbjs.position[j];
    //             encoder_state[j] = fbjs.encoder_state[j];
    //         }
            
    //         return true;
    //     }
    //     else
    //     {
    //         usleep(1000);
    //         continue;
    //     }
    // }

    // for (uint32_t j = 0; j < joint_num_; j++)
    // {
    //     joint[j] = 0;
    //     encoder_state[j] = INVALID;
    // }

    // state = SERVO_INIT;
    // return false;
}

bool BareCoreInterface::resetBareCore(void)
{
    return true;
    // ServiceRequest req;
    // req.req_id = JTAC_CMD_SID;
    // int cmd = 0;
    // memcpy(req.req_buff, &cmd, sizeof(cmd));
    // return sendRequest(command_interface_, req);
}

bool BareCoreInterface::stopBareCore(void)
{
    return true;
    // ServiceRequest req;
    // req.req_id = JTAC_CMD_SID;
    // int cmd = 1;
    // memcpy(req.req_buff, &cmd, sizeof(cmd));
    // return sendRequest(command_interface_, req);
}

bool BareCoreInterface::setConfigData(int id, const vector<double> &data)
{
    return true;
    // ServiceRequest req;
    // req.req_id = WRTIE_DATA_BY_ID;
    // int len = data.size();
    // memcpy(&req.req_buff[0], (char*)&id, sizeof(id));
    // memcpy(&req.req_buff[4], (char*)&len, sizeof(len));
    // memcpy(&req.req_buff[8], (char*)&data[0], len * sizeof(double));
    // return sendRequest(command_interface_ , req);
}

bool BareCoreInterface::setConfigData(int id, const vector<int> &data)
{
    return true;
    // ServiceRequest req;
    // req.req_id = WRTIE_DATA_BY_ID;
    // int len = data.size();
    // memcpy(&req.req_buff[0], (char*)&id, sizeof(id));
    // memcpy(&req.req_buff[4], (char*)&len, sizeof(len));
    // memcpy(&req.req_buff[8], (char*)&data[0], len * sizeof(int));
    // return sendRequest(command_interface_ , req);
}

bool BareCoreInterface::getConfigData(int id, vector<double> &data)
{
    return true;
    // ServiceRequest req;
    // req.req_id = READ_DATA_BY_ID;
    // int len = data.size();
    // memcpy(&req.req_buff[0], (char*)&id, sizeof(id));
    // memcpy(&req.req_buff[4], (char*)&len, sizeof(len));

    // //if (sendRequest(jtac_param_interface_, req))
    // if (sendRequest(command_interface_, req))
    // {
    //     ServiceResponse res;

    //     //if (recvResponse(jtac_param_interface_, res))
    //     if (recvResponse(command_interface_, res))
    //     {
    //         if (*((int*)(&res.res_buff[0])) == id && *((int*)(&res.res_buff[4])) == len)
    //         {
    //             memcpy(&data[0], &res.res_buff[8], len * sizeof(double));
    //             return true;
    //         }
    //     }
    // }
    // return false;
}

bool BareCoreInterface::getEncoder(vector<int> &data)
{
    return  true;
    // ServiceRequest req;
    // req.req_id = GET_ENCODER_SID;
    // data.resize(joint_num_);
    // int len = joint_num_;
    // memcpy(&req.req_buff[0], (void*)&len, sizeof(len));

    // //if (sendRequest(jtac_param_interface_, req))
    // if (sendRequest(command_interface_, req))
    // {
    //     ServiceResponse res;

    //     //if (recvResponse(jtac_param_interface_, res))
    //     if (recvResponse(command_interface_, res))
    //     {
    //         if (*((int*)(&res.res_buff[0])) == len)
    //         {
    //             memcpy(&data[0], &res.res_buff[4], len * sizeof(int));
    //             return true;
    //         }
    //     }
    // }

    // return false;
}

bool BareCoreInterface::getEncoderError(vector<int> &data)
{
    return true;
    // ServiceRequest req;
    // req.req_id = GET_ENCODER_ERR_SID;
    // data.resize(joint_num_);
    // int len = joint_num_;
    // memcpy(&req.req_buff[0], (void*)&len, sizeof(len));

    // //if (sendRequest(jtac_param_interface_, req))
    // if (sendRequest(command_interface_, req))
    // {
    //     ServiceResponse res;

    //     //if (recvResponse(jtac_param_interface_, res))
    //     if (recvResponse(command_interface_, res))
    //     {
    //         if (*((int*)(&res.res_buff[0])) == len)
    //         {
    //             memcpy(&data[0], &res.res_buff[4], len * sizeof(int));
    //             return true;
    //         }
    //     }
    // }

    // return false;
}

bool BareCoreInterface::resetEncoderError(size_t index)
{
    return true;
    // ServiceRequest req;
    // size_t length = 1;
    // req.req_id = RESET_ENCODER_ERR_SID;
    // memcpy(req.req_buff, &length, sizeof(size_t));
    // memcpy(&req.req_buff[4], &index, sizeof(size_t));

    // //if (sendRequest(jtac_param_interface_, req))
    // if (sendRequest(command_interface_, req))
    // {
    //     ServiceResponse res;

    //     //if (recvResponse(jtac_param_interface_, res))
    //     if (recvResponse(command_interface_, res))
    //     {
    //         return true;
    //     }
    // }

    // return false;
}

bool BareCoreInterface::resetEncoderError(void)
{
    return true;
    // ServiceRequest req;
    // size_t length = 0;

    // req.req_id = RESET_ENCODER_ERR_SID;
    // memcpy(req.req_buff, &length, sizeof(size_t));

    // //if (sendRequest(jtac_param_interface_, req))
    // if (sendRequest(command_interface_, req))
    // {
    //     ServiceResponse res;

    //     //if (recvResponse(jtac_param_interface_, res))
    //     if (recvResponse(command_interface_, res))
    //     {
    //         return true;
    //     }
    // }

    // return false;
}

bool BareCoreInterface::getControlPosition(double *data, size_t size)
{
    return true;
    // if (size > NUM_OF_JOINT) return false;

    // int len = size;
    // ServiceRequest req;
    // req.req_id = GET_CONTROL_POS_SID;
    // memcpy(&req.req_buff[0], (char*)&len, sizeof(len));

    // //if (sendRequest(jtac_param_interface_, req))
    // if (sendRequest(command_interface_, req))
    // {
    //     ServiceResponse res;

    //     //if (recvResponse(jtac_param_interface_, res))
    //     if (recvResponse(command_interface_, res))
    //     {
    //         if (*((int*)(&res.res_buff[0])) == len)
    //         {
    //             memcpy(data, &res.res_buff[4], len * sizeof(double));
    //             return true;
    //         }
    //     }
    // }

    // return false;
}

bool BareCoreInterface::setOmegaFilter(uint32_t filter_half_length, double *weights, size_t weights_size)
{
    return true;
    // uint32_t len = filter_half_length;
    // ServiceRequest req;
    // req.req_id = SET_OMEGA_FILTER_SID;
    // memcpy(&req.req_buff[0], (char*)&len, sizeof(len));
    // memcpy(&req.req_buff[8], weights, weights_size * sizeof(double));

    // //if (sendRequest(jtac_param_interface_, req))
    // if (sendRequest(command_interface_, req))
    // {
    //     ServiceResponse res;

    //     //if (recvResponse(jtac_param_interface_, res))
    //     if (recvResponse(command_interface_, res))
    //     {
    //         if (*((uint32_t*)(&res.res_buff[0])) == len)
    //         {
    //             return true;
    //         }
    //     }
    // }

    // return false;
}

bool BareCoreInterface::readVersion(char *buffer, size_t size)
{
    return true;
    // ServiceRequest req;
    // req.req_id = READ_VERSION_SID;
    // if (sendRequest(command_interface_, req))
    // {
    //     ServiceResponse res;

    //     if (recvResponse(command_interface_, res))
    //     {
    //         size_t len = strlen(res.res_buff);
    //         len = len < size ? len : size - 1;
    //         memcpy(buffer, res.res_buff, len);
    //         buffer[len] = '\0';
    //         return true;
    //     }
    // }

    // return false;
}

bool BareCoreInterface::downloadServoParam(uint32_t addr, const char *data, uint32_t length)
{
    return true;
    // ServiceRequest req;
    // req.req_id = WRITE_SERVO_DATA_BY_ADDR;
    
    // memcpy(&req.req_buff[0], (void*)&addr, sizeof(uint32_t));
    // //std::cout << *((uint32_t*)&req.req_buff[0]) << std::endl;
    // memcpy(&req.req_buff[4], (void*)&length, sizeof(uint32_t));
    // //std::cout<<*((unsigned int *)&req.req_buff[4])<<std::endl;
    // memcpy(&req.req_buff[8], (void*)data, length);
    // //std::cout<<*((unsigned short *)&req.req_buff[8])<<std::endl;

    // if (sendRequest(command_interface_, req))
    // {
    //     ServiceResponse res;

    //     if (recvResponse(command_interface_, res))
    //     {
    //         if (*((uint32_t*)(&res.res_buff[0])) == addr && *((uint32_t*)(&res.res_buff[4])) == length)
    //         {
    //             return true;
    //         }
    //     }
    // }

    // return false;
}

// bool BareCoreInterface::sendRequest(CommInterface &comm, const ServiceRequest &req)
// {
    // for (size_t cnt = 0; cnt < MAX_ATTEMPTS; cnt++)
    // {
    //     if (comm.send(&req, sizeof(ServiceRequest), COMM_DONTWAIT) != SUCCESS)
    //     {
    //         usleep(1000);
    //         continue;
    //     }
    //     else
    //     {
    //         return true;
    //     }
    // }

    // return false;
// }

// bool BareCoreInterface::recvResponse(CommInterface &comm, ServiceResponse &res)
// {
    // for (size_t cnt = 0; cnt < MAX_ATTEMPTS; cnt++)
    // {
    //     if (comm.recv(&res, sizeof(ServiceResponse), COMM_DONTWAIT) != SUCCESS)
    //     {
    //         usleep(1000);
    //         continue;
    //     }
    //     else
    //     {
    //         return true;
    //     }
    // }

    // return false;
// }


}

