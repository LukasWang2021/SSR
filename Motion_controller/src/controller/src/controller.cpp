#include "controller.h"
#include "error_monitor.h"
#include "base_datatype.h"
#include <unistd.h>
#include <iostream>


using namespace fst_ctrl;
using namespace fst_base;
using namespace fst_comm;
using namespace std;

Controller* Controller::instance_ = NULL;

Controller::Controller():
    is_exit_(false),
    log_ptr_(NULL),
    param_ptr_(NULL),
    process_comm_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ControllerParam();
    FST_LOG_INIT("Controller");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}


Controller::~Controller()
{

}

Controller* Controller::getInstance()
{
    if(instance_ == NULL)
    {
        instance_ = new Controller();
    }
    return instance_;
}

bool Controller::init()
{
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load controller component parameters");
        return false;
    } 
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);   
    
    virtual_core1_.init(log_ptr_);
    state_machine_.init(log_ptr_, param_ptr_, &motion_control_, &virtual_core1_);

    if(!device_manager_.init())
    {
        return false;
    }
    
    if(!tool_manager_.init())
    {
        return false;
    }

    if(!coordinate_manager_.init())
    {
        return false;
    }

    if(!reg_manager_.init())
    {
        return false;
    }

    if(motion_control_.init(&device_manager_, NULL, &coordinate_manager_, &tool_manager_, ErrorMonitor::instance()) != 0)
    {
        return false;
    }

    process_comm_ptr_ = ProcessComm::getInstance();
    ipc_.init(log_ptr_, param_ptr_, process_comm_ptr_->getControllerServerPtr(), &reg_manager_);
    if(!process_comm_ptr_->getControllerServerPtr()->init()
        || !process_comm_ptr_->getControllerServerPtr()->open()
        || !process_comm_ptr_->getControllerClientPtr()->init()
        || !process_comm_ptr_->getHeartbeatClientPtr()->init())
    {
        return false;
    }

    rpc_.init(log_ptr_, param_ptr_, &virtual_core1_, &tp_comm_, &state_machine_, 
        &tool_manager_, &coordinate_manager_, &reg_manager_, &device_manager_, &motion_control_,
        process_comm_ptr_->getControllerClientPtr());

    if(!heartbeat_thread_.run(&heartbeatThreadFunc, this, 50))
    {
        return false;
    }

    if(!routine_thread_.run(&controllerRoutineThreadFunc, this, 50))
    {
        return false;
    }

        
    if(!tp_comm_.init()
        || !tp_comm_.open())
    {
        return false;
    }

    return true;    
}

bool Controller::isExit()
{
    return is_exit_;
}

void Controller::runRoutineThreadFunc()
{
    state_machine_.processStateMachine();
    rpc_.processRpc();
    ipc_.processIpc();
}

void Controller::runHeartbeatThreadFunc()
{
    process_comm_ptr_->getHeartbeatClientPtr()->sendHeartbeat();
}

void controllerRoutineThreadFunc(void* arg)
{
    std::cout<<"---controllerRoutineThreadFunc running"<<std::endl;
    Controller* controller_ptr = static_cast<Controller*>(arg);
    while(!controller_ptr->isExit())
    {
        controller_ptr->runRoutineThreadFunc();
    }
}

void heartbeatThreadFunc(void* arg)
{
    std::cout<<"---heartbeatThreadFunc running"<<std::endl;
    Controller* controller_ptr = static_cast<Controller*>(arg);
    while(!controller_ptr->isExit())
    {
        controller_ptr->runHeartbeatThreadFunc();
    }
}

