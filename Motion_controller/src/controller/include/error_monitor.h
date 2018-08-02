#ifndef ERROR_MONITOR_H_ 
#define ERROR_MONITOR_H_

#include "lockfree_queue.h"
#include <string>


namespace fst_ctrl
{
class ErrorMonitor
{
  public:
	ErrorMonitor();
	~ErrorMonitor();
    static ErrorMonitor* instance();
    void clear();
    bool add(unsigned long long code);
    std::string getErrorBytes();
    bool updated();
    bool isInitError();
    int getWarningLevel();
  private:
    enum {MAX_ERRORS = 10,};
  
    std::atomic_bool updated_flag_;
    std::atomic_bool init_err_flag_;
    std::atomic_int warning_level_; //warning level 
    std::atomic_int err_cnt_;   //number of error codes
    std::atomic_ullong pre_code_;  //previous error code
    LFQueue<unsigned long long> err_queue_; //queue to store error codes
    LFQueue<unsigned long long> init_err_qu_; //store init error codes
};

}

#endif

