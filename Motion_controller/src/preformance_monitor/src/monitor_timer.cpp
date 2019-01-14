#include "monitor_timer.h"
#include <cstring>
#include <climits>
#include <iostream>

using namespace fst_base;
using namespace std;

MonitorTimer::MonitorTimer(std::string name, int rw_delay_window_size, int ignore_window_size, 
                                int real_time_window_size, int total_window_size):
    name_(name),
    rw_delay_window_size_(rw_delay_window_size),
    ignore_window_size_(ignore_window_size),
    real_time_window_size_(real_time_window_size),
    total_window_size_(total_window_size),
    ignored_count(0),
    real_time_consumption_sum_(0),
    real_time_interval_sum_(0),
    real_time_count_(0),
    last_start_time_(0)
{
    memset(&real_time_statistic_, 0, sizeof(PreformanceStatistic));
    real_time_statistic_.consumption_min = LONG_MAX;
    real_time_statistic_.interval_min = LONG_MAX;
    memset(&total_statistic_, 0, sizeof(PreformanceStatistic));
    total_statistic_.consumption_min = LONG_MAX;
    total_statistic_.interval_min = LONG_MAX;
}
    
MonitorTimer::~MonitorTimer()
{
    sample_list_.clear();
}

void MonitorTimer::startTimer()
{
    sample_.start_time = clock();
}

void MonitorTimer::stopTimer()
{
    sample_.stop_time = clock();
    
    sample_list_.push_back(sample_);
    if(list_size_ >= total_window_size_)
    {
        sample_list_.pop_front();
    }
    else
    {
        ++list_size_;
    }

    long current_consumption = sample_.stop_time - sample_.start_time;
    long current_interval = sample_.start_time - last_start_time_;
    last_start_time_ = sample_.start_time;

    // ignore
    if(ignored_count < ignore_window_size_)
    {
        ++ignored_count;
        return;
    }

    if(current_consumption > real_time_statistic_.consumption_max)
    {
        real_time_statistic_.consumption_max = current_consumption;
    }
    else if(current_consumption < real_time_statistic_.consumption_min)
    {
        real_time_statistic_.consumption_min = current_consumption;
    }
    else{}
    if(current_interval > real_time_statistic_.interval_max)
    {
        real_time_statistic_.interval_max = current_interval;
    }
    else if(current_interval < real_time_statistic_.interval_min)
    {
        real_time_statistic_.interval_min = current_interval;
    }
    else{}

    real_time_consumption_sum_ += current_consumption;
    if(real_time_count_ < real_time_window_size_)
    {
        ++real_time_count_;
        real_time_statistic_.consumption_average = real_time_consumption_sum_/real_time_count_;
    }
    else
    {
        real_time_consumption_sum_ -= real_time_statistic_.consumption_average;
        real_time_statistic_.consumption_average = real_time_consumption_sum_/real_time_window_size_;
    }

    real_time_interval_sum_ += current_interval;
    if(real_time_count_ < real_time_window_size_)
    {
        ++real_time_count_;
        real_time_statistic_.interval_average = real_time_interval_sum_/real_time_count_;
    }
    else
    {
        real_time_interval_sum_ -= real_time_statistic_.interval_average;
        real_time_statistic_.interval_average = real_time_interval_sum_/real_time_window_size_;
    }    
}

void MonitorTimer::popHeadSample()
{

}

void MonitorTimer::printRealTimeStatistic()
{
    std::cout<<std::dec<<"Monitor: "<<name_<<std::endl;
    std::cout<<"   consumption_max = "<<real_time_statistic_.consumption_max<<std::endl;
    std::cout<<"   consumption_min = "<<real_time_statistic_.consumption_min<<std::endl;
    std::cout<<"   consumption_avg = "<<real_time_statistic_.consumption_average<<std::endl;
    std::cout<<"   interval_max    = "<<real_time_statistic_.interval_max<<std::endl;
    std::cout<<"   interval_min    = "<<real_time_statistic_.interval_min<<std::endl;
    std::cout<<"   interval_avg    = "<<real_time_statistic_.interval_average<<std::endl;
}

PreformanceStatistic MonitorTimer::getTotalStatistic()
{
    return total_statistic_;
}

MonitorTimer::MonitorTimer():
    name_("dummy"),
    rw_delay_window_size_(0),
    ignore_window_size_(0),
    real_time_window_size_(0),
    total_window_size_(0),
    ignored_count(0),
    real_time_consumption_sum_(0),
    real_time_interval_sum_(0),
    real_time_count_(0),
    last_start_time_(0)
{

}

