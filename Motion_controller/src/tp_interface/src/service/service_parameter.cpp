/**
 * @file service_parameter.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-12-27
 */

#include "service_parameter.h"


static const unsigned char READ_BY_ADDR     = 0x14;
static const unsigned char READ_BY_ID       = 0x1D;
static const unsigned char WRITE_BY_ADDR    = 0x24;
static const unsigned char WRITE_BY_ID      = 0x2D;

ServiceParam* ServiceParam::instance()
{
    static ServiceParam serv_param;

    return &serv_param;
}

bool ServiceParam::setConfigData(int id, const vector<double> &data)
{
    ServiceRequest req;
    req.req_id = WRITE_BY_ID;
    int len = data.size();
    memcpy(&req.req_buff[0], (char*)&id, sizeof(id));
    memcpy(&req.req_buff[4], (char*)&len, sizeof(len));
    memcpy(&req.req_buff[8], (char*)&data[0], len * sizeof(double));
    return sendRequest(&req);
}

bool ServiceParam::getConfigData(int id, vector<double> &data)
{
    ServiceRequest req;
    req.req_id = READ_BY_ID;
    int len = data.size();
    memcpy(&req.req_buff[0], (char*)&id, sizeof(id));
    memcpy(&req.req_buff[4], (char*)&len, sizeof(len));
   // printf("service_id:%x, sub id:%x, len:%d\n", req.req_id, id, len);
    if (sendCountOut(&req))
    {
        ServiceResponse resp;
        recvTimeout(&resp);
        if (*((int*)(&resp.res_buff[0])) == id 
        && *((int*)(&resp.res_buff[4])) == len)
        {
            for (int i = 0; i < len; ++i) 
            {
                data[i] = *((double*)(&resp.res_buff[8 + i * sizeof(double)]));
            }
            return true;
        }

    }
    return false;
}
