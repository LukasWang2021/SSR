#ifndef ANYBUS_MANAGER_H
#define ANYBUS_MANAGER_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdlib.h>
#include <map>

#include "thread_help.h"
#include "common_log.h"

#include "abcc.h"
#include "abcc_versions.h"
#include "abcc_obj_cfg.h"
#include "abp_etn.h"
#include "etn_obj.h"
#include "safe_obj.h"
#include "abcc_sw_port.h"

#include "appl_abcc_handler.h"
#include "anybus_manager_param.h"

using namespace std;

namespace fst_anybus
{
class AnybusManager
{
public:
    AnybusManager();
    ~AnybusManager();

    unsigned long long int init();
    unsigned long long int initAnybus();
    
    unsigned long long int openServer();
    void closeServer();

    void* anybusManagerThreadFunc();
    bool isRunning();

private:
    fst_anybus::AnybusManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_base::ThreadHelp thread_ptr_;
    APPL_AbccHandlerStatusType abcc_handler_status_;
    bool is_running_;

    int operation_mode_;
    int cycle_time_;
    int abcc_page_size_;
    unsigned int abcc_page_addr_;
    int appl_page_size_;
    unsigned int appl_page_addr_;
    bool safety_enable_;
};

}

#endif

void* anybusManagerRoutineThreadFunc(void* arg);
