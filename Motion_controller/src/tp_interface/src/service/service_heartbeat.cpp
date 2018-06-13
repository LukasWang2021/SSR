/**
 * @file service_heartbeat.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-09-26
 */


#include "service_heartbeat.h"

ServiceHeartbeat* ServiceHeartbeat::instance()
{
    static ServiceHeartbeat serv_hb;
    return &serv_hb;
}

void ServiceHeartbeat::sendRequest()
{
    ServiceRequest req = {MONITOR_HEARTBEAT_SID, ""};

    ServiceResponse resp;
    if (sendCountOut(&req) && recvTimeout(&resp))     
    {
        int err_size = *(int*)resp.res_buff;
        if (err_size > 0)
        {
        	printf("recvTimeout and err_size = %llx.\n", err_size);
            U64 *err_list = (U64*)&resp.res_buff[8];
            for (int i = 0; i < err_size; i++)
            {
                rcs::Error::instance()->add(err_list[i]);
            }
        }// if (err_size > 0)
    }// if (sendCountOut(req) && recvTimeout(resp))            
}

