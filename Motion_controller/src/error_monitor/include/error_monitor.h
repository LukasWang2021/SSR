#ifndef ERROR_MONITOR_H_ 
#define ERROR_MONITOR_H_

#include "lockfree_queue.h"
#include <string>


namespace fst_base
{
class ErrorMonitor
{
  public:
	ErrorMonitor();
	~ErrorMonitor();
    static ErrorMonitor* instance();
    void clear();
    bool add(unsigned long long code);
    bool pop(unsigned long long& error_code);
    int getWarningLevel();
  private:
    enum {MAX_ERRORS = 10,};
  
    std::atomic_int warning_level_; //warning level 
    std::atomic_int err_cnt_;   //number of error codes
    std::atomic_ullong pre_code_;  //previous error code
    LFQueue<unsigned long long> err_queue_; //queue to store error codes
};

}

#endif

