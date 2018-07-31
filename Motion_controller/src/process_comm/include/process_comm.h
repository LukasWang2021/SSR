#ifndef PROCESS_COMM_H
#define PROCESS_COMM_H


#include "process_comm_param.h"
#include "common_log.h"

namespace fst_base
{
class ProcessComm
{
public:
    ProcessComm();
    ~ProcessComm();

private:
    ProcessCommParam param_;
    fst_log::Logger log_;
 
};

}

#endif


