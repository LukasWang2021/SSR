/**
 * @file service_jtac.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-09-26
 */

#include "service_jtac.h"

ServiceJTAC* ServiceJTAC::instance()
{
    static ServiceJTAC serv_m;
    return &serv_m;
}

bool ServiceJTAC::resetBareMetal(void)
{
    ServiceRequest req = {JTAC_CMD_SID, ""};
    int reset = 0;
    memcpy(&req.req_buff[0], &reset, sizeof(reset));
    return sendRequest(&req);
}

bool ServiceJTAC::stopBareMetal(void)
{
    ServiceRequest req = {JTAC_CMD_SID, ""};
    int stop = 1;
    memcpy(&req.req_buff[0], &stop, sizeof(stop));
    return sendRequest(&req);
}



