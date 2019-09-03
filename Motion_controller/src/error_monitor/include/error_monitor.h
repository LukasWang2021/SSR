#ifndef ERROR_MONITOR_H_ 
#define ERROR_MONITOR_H_

#include "lockfree_queue.h"
#include <string>
#include <list>


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
    std::list<uint64_t> getErrorList(void);
    bool isCore0Error(unsigned long long code);
    int getErrorLevel(unsigned long long code);

  private:
    enum {MAX_ERRORS = 10,};
    void addErrorList(unsigned long long error_code);

    std::atomic_int warning_level_; //warning level 
    std::atomic_int err_cnt_;   //number of error codes
    std::atomic_ullong pre_code_;  //previous error code
    LFQueue<unsigned long long> err_queue_; //queue to store error codes
    std::list<uint64_t> err_list_; // list to store the pop error codes
};

}

#endif

