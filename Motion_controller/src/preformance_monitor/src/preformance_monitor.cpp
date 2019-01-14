#include "preformance_monitor.h"
#include <iostream>


using namespace fst_base;
using namespace std;

PreformanceMonitor::PreformanceMonitor():
    interval_count_(0)
{

}

PreformanceMonitor::~PreformanceMonitor()
{
    std::map<int, MonitorTimer*>::iterator it;
    for(it = timer_map_.begin(); it != timer_map_.end(); ++it)
    {
        if(it->second != NULL)
        {
            delete it->second;
        }
    }
    timer_map_.clear();
}

bool PreformanceMonitor::addTimer(int id, std::string name, int rw_delay_window_size, 
                                    int ignore_window_size, int real_time_window_size, int total_window_size)
{
    MonitorTimer* timer_ptr = new MonitorTimer(name, rw_delay_window_size, ignore_window_size, real_time_window_size, total_window_size);
    if(timer_ptr == NULL)
    {
        return false;
    }
    timer_map_[id] = timer_ptr;
    return true;
}
                                    
void PreformanceMonitor::startTimer(int id)
{
    timer_map_[id]->startTimer();
}

void PreformanceMonitor::stopTimer(int id)
{
    timer_map_[id]->stopTimer();
}

void PreformanceMonitor::printRealTimeStatistic(int interval)
{
    ++interval_count_;
    if(interval_count_ > interval)
    {
        interval_count_ = 0;
        std::cout<<"\033[2J";
        std::cout<<std::dec<<"CLOCKS_PER_SEC = "<<CLOCKS_PER_SEC<<std::endl;
        std::map<int, MonitorTimer*>::iterator it;
        for(it = timer_map_.begin(); it != timer_map_.end(); ++it)
        {
            it->second->printRealTimeStatistic();
        }
    }
}

void PreformanceMonitor::saveTotalStatisticToFile()
{

}



