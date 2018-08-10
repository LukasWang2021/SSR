#include "process_comm.h"
#include <iostream>

using namespace fst_base;
using namespace std;

ProcessComm* ProcessComm::instance_ = NULL;

ProcessComm::ProcessComm():
    log_ptr_(NULL), param_ptr_(NULL), controller_server_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ProcessCommParam(); 
    FST_LOG_INIT("ProcessComm");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load process comm component parameters");
    } 
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);      

    controller_server_ptr_ = new ControllerServer(log_ptr_, param_ptr_);
    if(controller_server_ptr_ == NULL)
    {
        FST_ERROR("Failed to new controller server");
    }

    controller_client_ptr_ = new ControllerClient(log_ptr_, param_ptr_);
    if(controller_client_ptr_ == NULL)
    {
        FST_ERROR("Failed to new controller client");
    }

    interpreter_server_ptr_ = new InterpreterServer(log_ptr_, param_ptr_);
    if(interpreter_server_ptr_ == NULL)
    {
        FST_ERROR("Failed to new interpreter server");
    }
    
    interpreter_client_ptr_ = new InterpreterClient(log_ptr_, param_ptr_);
    if(interpreter_client_ptr_ == NULL)
    {
        FST_ERROR("Failed to new interpreter client");
    }    
}

ProcessComm::~ProcessComm()
{

}

ProcessComm* ProcessComm::getInstance()
{
    if(instance_ == NULL)
    {
        instance_ = new ProcessComm();
    }
    return instance_;
}

ControllerServer* ProcessComm::getControllerServerPtr()
{
    return controller_server_ptr_;
}

ControllerClient* ProcessComm::getControllerClientPtr()
{
    return controller_client_ptr_;
}

InterpreterServer* ProcessComm::getInterpreterServerPtr()
{
    return interpreter_server_ptr_;
}

InterpreterClient* ProcessComm::getInterpreterClientPtr()
{
    return interpreter_client_ptr_;
}


