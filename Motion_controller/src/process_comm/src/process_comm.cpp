#include "process_comm.h"
#include <iostream>
#include "error_code.h"


using namespace fst_base;
using namespace std;

ProcessComm* ProcessComm::instance_ = NULL;
ErrorCode ProcessComm::init_error_code_ = SUCCESS;

ProcessComm::ProcessComm():
    log_ptr_(NULL), 
    param_ptr_(NULL), 
    controller_server_ptr_(NULL),
    controller_client_ptr_(NULL),
    interpreter_server_ptr_(NULL),
    interpreter_client_ptr_(NULL),
    heartbeat_client_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ProcessCommParam(); 
    FST_LOG_INIT("ProcessComm");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load process comm component parameters");
        init_error_code_ = PROCESS_COMM_LOAD_PARAM_FAILED;
        return;
    } 
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);      

    controller_server_ptr_ = new ControllerServer(log_ptr_, param_ptr_);
    if(controller_server_ptr_ == NULL)
    {
        FST_ERROR("Failed to new controller server");
        init_error_code_ = PROCESS_COMM_INIT_OBJECT_FAILED;
        return;
    }

    controller_client_ptr_ = new ControllerClient(log_ptr_, param_ptr_);
    if(controller_client_ptr_ == NULL)
    {
        FST_ERROR("Failed to new controller client");
        init_error_code_ = PROCESS_COMM_INIT_OBJECT_FAILED;
        return;
    }

    interpreter_server_ptr_ = new InterpreterServer(log_ptr_, param_ptr_);
    if(interpreter_server_ptr_ == NULL)
    {
        FST_ERROR("Failed to new interpreter server");
        init_error_code_ = PROCESS_COMM_INIT_OBJECT_FAILED;
        return;
    }
    
    interpreter_client_ptr_ = new InterpreterClient(log_ptr_, param_ptr_);
    if(interpreter_client_ptr_ == NULL)
    {
        FST_ERROR("Failed to new interpreter client");
        init_error_code_ = PROCESS_COMM_INIT_OBJECT_FAILED;
        return;
    }

    heartbeat_client_ptr_ = new HeartbeatClient(log_ptr_, param_ptr_);
    if(heartbeat_client_ptr_ == NULL)
    {
        FST_ERROR("Failed to new heartbeat client");
        init_error_code_ = PROCESS_COMM_INIT_OBJECT_FAILED;
        return;
    }  
}

ProcessComm::~ProcessComm()
{
    if(log_ptr_ != NULL)
    {
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if(param_ptr_ != NULL)
    {
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

ProcessComm* ProcessComm::getInstance()
{
    if(instance_ == NULL)
    {
        instance_ = new ProcessComm();
    }
    return instance_;
}

ErrorCode ProcessComm::getInitErrorCode()
{
    return init_error_code_;
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

HeartbeatClient* ProcessComm::getHeartbeatClientPtr()
{
    return heartbeat_client_ptr_;
}

