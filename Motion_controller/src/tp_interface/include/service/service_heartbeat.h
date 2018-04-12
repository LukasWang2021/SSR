/**
 * @file service_heartbeat.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-09-26
 */
#ifndef SERVICE_HEARTBEAT_H_
#define SERVICE_HEARTBEAT_H_

#include "service_base.h"

class ServiceHeartbeat:public ServiceBase
{
  public:
    ServiceHeartbeat():ServiceBase("heartbeat")
    {
    
    }
    ~ServiceHeartbeat()
    {
    
    }

    static ServiceHeartbeat* instance();
    void sendRequest();
};

#endif

