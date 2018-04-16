/**
 * @file service_jtac.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-09-26
 */

#ifndef SERVICE_JTAC_H_
#define SERVICE_JTAC_H_

#include "service_base.h"

class ServiceJTAC:public ServiceBase
{
  public:
    ServiceJTAC():ServiceBase("JTAC")
    {
    
    }
    ~ServiceJTAC()
    {
    
    }

    static ServiceJTAC* instance();

    bool resetBareMetal(void);

    bool stopBareMetal(void);
};

#endif

