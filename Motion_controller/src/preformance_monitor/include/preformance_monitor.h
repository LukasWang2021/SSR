#ifndef PREFORMANCE_MONITOR_H
#define PREFORMANCE_MONITOR_H


#include <stdio.h>
#include <time.h>
#include <string>
#include <map>
#include "monitor_timer.h"

namespace fst_base
{
class PreformanceMonitor
{
public:
    PreformanceMonitor();
    ~PreformanceMonitor();

    /*
    average_window_size = 0 imply disabling the computation of average time consumption
    ignore_window_size !=0 imply ignoring the first-N sampling of average time consuption
    rw_delay_window_size imply the delay of realtime show
    */
    bool addTimer(int id, std::string name, int rw_delay_window_size, 
                    int ignore_window_size, int real_time_window_size, int total_window_size);
    void startTimer(int id);
    void stopTimer(int id);

    void printRealTimeStatistic(int interval);
    void saveTotalStatisticToFile();

private:
    std::map<int, MonitorTimer*> timer_map_;
    int interval_count_;
};

}

#endif

