#include "process_comm.h"


using namespace fst_base;


ProcessComm::ProcessComm():
    log_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
}

ProcessComm::~ProcessComm()
{

}


