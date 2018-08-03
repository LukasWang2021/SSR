#include "controller.h"
#include "error_monitor.h"
#include <unistd.h>
#include <iostream>


using namespace fst_ctrl;
using namespace fst_base;
using namespace std;

Controller* Controller::instance_ = NULL;

Controller::Controller():
    is_exit_(false),
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ControllerParam();
    FST_LOG_INIT("controller");
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
    state_machine_.init(log_ptr_, param_ptr_, &virtual_core1_);
    rpc_.init(log_ptr_, param_ptr_, &virtual_core1_, &tp_comm_, &state_machine_);
    
    if(!routine_thread_.run(&controllerRoutineThreadFunc, this, 50))
    {
        return false;
    }

    if(!tp_comm_.init())
    {
        return false;
    }
    
    if(!tp_comm_.open())
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


