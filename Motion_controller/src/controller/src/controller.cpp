#include "controller.h"
#include "error_monitor.h"
#include "base_datatype.h"
#include <unistd.h>
#include <iostream>
#include "serverAlarmApi.h"
#include <sstream>

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
    routine_thread_.join();
    heartbeat_thread_.join();

    ServerAlarmApi::GetInstance()->pyDecref();    

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

Controller* Controller::getInstance()
{
    if(instance_ == NULL)
    {
        instance_ = new Controller();
    }
    return instance_;
}

ErrorCode Controller::init()
{
    recordLog("Controller initialization start");
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load controller component parameters");
        recordLog(CONTROLLER_LOAD_PARAM_FAILED, "Failed to load controller component parameters");
        return CONTROLLER_LOAD_PARAM_FAILED;
    } 
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);   
    
    //preformance_monitor_.addTimer(1, "routine thread",  0,  10, 100, 1000);
    //preformance_monitor_.addTimer(2, "heartbeat thread",  0,  20, 200, 2000);

    ServerAlarmApi::GetInstance()->setEnable(param_ptr_->enable_log_service_);
    ServerAlarmApi::GetInstance()->sendOneAlarm(CONTROLLER_LOG, std::string("Controller start init..."));
    
    virtual_core1_.init(log_ptr_, param_ptr_);
    state_machine_.init(log_ptr_, param_ptr_, &motion_control_, &virtual_core1_);

    ErrorCode error_code;
    error_code = device_manager_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = tool_manager_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = coordinate_manager_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = reg_manager_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    process_comm_ptr_ = ProcessComm::getInstance();
    error_code = ProcessComm::getInitErrorCode();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = process_comm_ptr_->getControllerServerPtr()->init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = process_comm_ptr_->getControllerClientPtr()->init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = process_comm_ptr_->getHeartbeatClientPtr()->init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = process_comm_ptr_->getControllerServerPtr()->open();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    ipc_.init(log_ptr_, param_ptr_, process_comm_ptr_->getControllerServerPtr(), &reg_manager_);
    rpc_.init(log_ptr_, param_ptr_, &publish_, &virtual_core1_, &tp_comm_, &state_machine_, 
        &tool_manager_, &coordinate_manager_, &reg_manager_, &device_manager_, &motion_control_,
        process_comm_ptr_->getControllerClientPtr());
    publish_.init(log_ptr_, param_ptr_, &virtual_core1_, &tp_comm_, &state_machine_, &motion_control_, &reg_manager_);

    if(!heartbeat_thread_.run(&heartbeatThreadFunc, this, param_ptr_->heartbeat_thread_priority_))
    {
        recordLog(CONTROLLER_CREATE_ROUTINE_THREAD_FAILED, "Controller failed to create routine thread");
        return CONTROLLER_CREATE_ROUTINE_THREAD_FAILED;
    }

    if(!routine_thread_.run(&controllerRoutineThreadFunc, this, param_ptr_->routine_thread_priority_))
    {
        recordLog(CONTROLLER_CREATE_HEARTBEAT_THREAD_FAILED, "Controller failed to create heartbeat thread");
        return CONTROLLER_CREATE_HEARTBEAT_THREAD_FAILED;
    }

    error_code = motion_control_.init(&device_manager_, NULL, &coordinate_manager_, &tool_manager_, ErrorMonitor::instance());
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }
    //FIXME: remove it later
    motion_control_.maskOffsetLostError();

    error_code = tp_comm_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = tp_comm_.open();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }
       
    recordLog("Controller initialization success");
    return SUCCESS;    
}

bool Controller::isExit()
{
    return is_exit_;
}

void Controller::setExit()
{
    is_exit_ = true;
}

void Controller::runRoutineThreadFunc()
{
    //preformance_monitor_.startTimer(1);
    state_machine_.processStateMachine();
    rpc_.processRpc();
    ipc_.processIpc();
    publish_.processPublish();    
    //preformance_monitor_.stopTimer(1);
    //preformance_monitor_.printRealTimeStatistic(10);
    usleep(param_ptr_->routine_cycle_time_);
}

void Controller::runHeartbeatThreadFunc()
{
    usleep(param_ptr_->heartbeat_cycle_time_);
    //preformance_monitor_.startTimer(2);
    process_comm_ptr_->getHeartbeatClientPtr()->sendHeartbeat();
    //preformance_monitor_.stopTimer(2);
}

void Controller::recordLog(std::string log_str)
{
    ServerAlarmApi::GetInstance()->sendOneAlarm(CONTROLLER_LOG, log_str);
}

void Controller::recordLog(ErrorCode error_code, std::string log_str)
{
    ServerAlarmApi::GetInstance()->sendOneAlarm(error_code, log_str);
}

void Controller::recordLog(ErrorCode major_error_code, ErrorCode minor_error_code, std::string log_str)
{
    std::stringstream ss;
    ss << log_str;
    ss << std::hex << minor_error_code;
    std::string str;
    ss >> str;
    ServerAlarmApi::GetInstance()->sendOneAlarm(major_error_code, str);
}

void controllerRoutineThreadFunc(void* arg)
{
    std::cout<<"controller routine thread running"<<std::endl;
    Controller* controller_ptr = static_cast<Controller*>(arg);
    while(!controller_ptr->isExit())
    {
        controller_ptr->runRoutineThreadFunc();
    }
    std::cout<<"controller routine thread exit"<<std::endl;
}

void heartbeatThreadFunc(void* arg)
{
    std::cout<<"heartbeat thread running"<<std::endl;
    Controller* controller_ptr = static_cast<Controller*>(arg);
    while(!controller_ptr->isExit())
    {
        controller_ptr->runHeartbeatThreadFunc();
    }
    std::cout<<"heartbeat thread exit"<<std::endl;
}

