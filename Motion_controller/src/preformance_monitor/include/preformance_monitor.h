#ifndef PREFORMANCE_MONITOR_H
#define PREFORMANCE_MONITOR_H


#include <stdio.h>
#include <time.h>

namespace fst_base
{
class PreformanceMonitor
{
public:
    PreformanceMonitor();
    ~PreformanceMonitor();
#if 0
    void init();
    void setEnable(bool enabled);
    void registerMonitorObject();
    
    void startClock();
    void stopClock();
#endif
private:
    //clock_t 
};

}

#endif

