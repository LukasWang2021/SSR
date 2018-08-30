#include <stdio.h>
#include <time.h>
#include <string>
#include <list>

namespace fst_base
{
typedef struct
{
    long consumption_max;
    long consumption_min;
    long consumption_average;
    long interval_max;
    long interval_min;
    long interval_average;
}PreformanceStatistic;

class MonitorTimer
{
public:
    MonitorTimer(std::string name, int rw_delay_window_size, int ignore_window_size, int real_time_window_size, int total_window_size);
    ~MonitorTimer();

    void startTimer();
    void stopTimer();    
    void popHeadSample();
    void printRealTimeStatistic();
    PreformanceStatistic getTotalStatistic();
private:
    // param
    std::string name_;
    int rw_delay_window_size_;
    int ignore_window_size_;
    int real_time_window_size_;
    int total_window_size_;

    // variable
    typedef struct
    {
        clock_t start_time;
        clock_t stop_time;
    }PreformanceSample;
    std::list<PreformanceSample> sample_list_;
    int list_size_; // list.size is O(n) complex, so replace it with list_size_;
    PreformanceSample sample_;

    PreformanceStatistic real_time_statistic_;
    PreformanceStatistic total_statistic_;

    // runtime variable
    int ignored_count;
    long long real_time_consumption_sum_;
    long long real_time_interval_sum_;
    int real_time_count_;
    clock_t last_start_time_;

    MonitorTimer();
    
};

}

