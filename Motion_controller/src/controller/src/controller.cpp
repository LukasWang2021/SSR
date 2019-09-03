#include "controller.h"
#include "error_monitor.h"
#include <unistd.h>
#include <iostream>
#include "serverAlarmApi.h"
#include "fst_safety_device.h"
#include <sstream>

using namespace fst_ctrl;
using namespace fst_base;
using namespace fst_comm;
using namespace fst_hal;
using namespace std;
using namespace fst_mc;

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
    /*
    ErrorCode error_code = motion_control_.saveJoint();
    if(error_code == SUCCESS)
    {
        recordLog(CONTROLLER_LOG, "save joint success");
    }
    else
    {
        recordLog(error_code, "save joint failed");
    }
    */

    routine_thread_.join();
    heartbeat_thread_.join();

    if(process_comm_ptr_ != NULL)
    {
        delete process_comm_ptr_;
        process_comm_ptr_ = NULL;
    }
    
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
    FST_INFO("Controller::init()");
    if(!param_ptr_->loadParam())
    {
        recordLog(CONTROLLER_LOAD_PARAM_FAILED, "Failed to load controller component parameters");
        return CONTROLLER_LOAD_PARAM_FAILED;
    } 
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);   
    
    //preformance_monitor_.addTimer(1, "routine thread",  0,  10, 100, 1000);
    //preformance_monitor_.addTimer(2, "heartbeat thread",  0,  20, 200, 2000);

    ServerAlarmApi::GetInstance()->setEnable(param_ptr_->enable_log_service_);
    recordLog("Controller initialization start");
    
    virtual_core1_.init(log_ptr_, param_ptr_);

    ErrorCode error_code;
    error_code = device_manager_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller device manager initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = tool_manager_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller tool manager initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = coordinate_manager_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller coordinate manager initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = reg_manager_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller reg manager initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    process_comm_ptr_ = ProcessComm::getInstance();
    error_code = ProcessComm::getInitErrorCode();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller process comm initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = process_comm_ptr_->getControllerServerPtr()->init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller process comm server initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = process_comm_ptr_->getControllerClientPtr()->init(process_comm_ptr_->getControllerServerPtr());
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller process comm client(controller) initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = process_comm_ptr_->getHeartbeatClientPtr()->init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller process comm heartbeat client initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = process_comm_ptr_->getControllerServerPtr()->open();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller process comm server(controller) initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = io_manager_.init(&device_manager_);
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller IO mananger initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = io_mapping_.init(&io_manager_);
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller IO mapping initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    //use macro to launch program
    program_launching_.init(&io_mapping_, process_comm_ptr_->getControllerClientPtr());

    error_code = system_manager_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller system manager initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    FST_INFO("init param_manager_");
    error_code = param_manager_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller param manager initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    FST_INFO("init state_machine_");
    state_machine_.init(log_ptr_, param_ptr_, &motion_control_, &virtual_core1_, 
                        process_comm_ptr_->getControllerClientPtr(), &device_manager_, &io_mapping_, &program_launching_);
    ipc_.init(log_ptr_, param_ptr_, process_comm_ptr_->getControllerServerPtr(), 
                process_comm_ptr_->getControllerClientPtr(), &reg_manager_, &state_machine_, &device_manager_, 
                &io_mapping_, &motion_control_);
    rpc_.init(log_ptr_, param_ptr_, &publish_, &virtual_core1_, &tp_comm_, &state_machine_, 
        &tool_manager_, &coordinate_manager_, &reg_manager_, &device_manager_, &motion_control_,
        process_comm_ptr_->getControllerClientPtr(), &io_mapping_, &io_manager_,&program_launching_, &file_manager_,
        &system_manager_, &param_manager_);
    publish_.init(log_ptr_, param_ptr_, &virtual_core1_, &tp_comm_, &state_machine_, &motion_control_, &reg_manager_,
                    process_comm_ptr_->getControllerClientPtr(), &io_mapping_, &device_manager_, &io_manager_);

    FST_INFO("init motion_control_");
    error_code = motion_control_.init(&device_manager_, NULL, &coordinate_manager_, &tool_manager_, ErrorMonitor::instance());
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller motion control initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    // wait state mathine of motion control start work 
    usleep(100 * 1000);
    FST_INFO("init check offset");
    error_code = state_machine_.checkOffsetState();
    if(error_code != SUCCESS)
    {
        FST_ERROR("controller init check offset failed");
    }  

    if(!heartbeat_thread_.run(&heartbeatThreadFunc, this, param_ptr_->heartbeat_thread_priority_))
    {
        recordLog(CONTROLLER_CREATE_ROUTINE_THREAD_FAILED, "Controller heartbeat thread failed to create routine thread");
        return CONTROLLER_CREATE_ROUTINE_THREAD_FAILED;
    }

    if(!routine_thread_.run(&controllerRoutineThreadFunc, this, param_ptr_->routine_thread_priority_))
    {
        recordLog(CONTROLLER_CREATE_HEARTBEAT_THREAD_FAILED, "Controller rountine thread failed to create heartbeat thread");
        return CONTROLLER_CREATE_HEARTBEAT_THREAD_FAILED;
    }

    FST_INFO("init tp_comm_");
    error_code = tp_comm_.init();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller TP comm initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    error_code = tp_comm_.open();
    if(error_code != SUCCESS)
    {
        recordLog(CONTROLLER_INIT_OBJECT_FAILED, error_code, "Controller TP comm open initialization failed");
        return CONTROLLER_INIT_OBJECT_FAILED;
    }

    state_machine_.setState(true);
    isOkLed();
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
    std::stringstream stream;
    stream<<"Log_Code: 0x"<<std::hex<<CONTROLLER_LOG<<" : "<<log_str;
    FST_INFO(stream.str().c_str());

    ServerAlarmApi::GetInstance()->sendOneAlarm(CONTROLLER_LOG, log_str);
}

void Controller::recordLog(ErrorCode error_code, std::string log_str)
{
    std::stringstream stream;
    stream<<"Log_Code: 0x"<<std::hex<<error_code<<" : "<<log_str;
    FST_ERROR(stream.str().c_str());
    state_machine_.setSafetyStop(error_code);

    ServerAlarmApi::GetInstance()->sendOneAlarm(error_code, log_str);
}

void Controller::recordLog(ErrorCode major_error_code, ErrorCode minor_error_code, std::string log_str)
{
    std::stringstream ss;
    ss << log_str <<": 0x";
    ss << std::hex << minor_error_code;
    std::string str = ss.str();

    ss<<"Log_Code: 0x"<<std::hex<<major_error_code<<" : "<<str;
    FST_ERROR(ss.str().c_str());
    state_machine_.setSafetyStop(major_error_code);

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

void Controller::isOkLed()
{
	// sent a message outside to hint the controller ok
	int fd_ok_led;
	fd_ok_led = open("/dev/mem", O_RDWR);
	if (fd_ok_led == -1)
		printf("The _ok_led-message cann't be sent. fd = %d\n", fd_ok_led);
	enum msg_ok_led {
		OK_LED_BASE = 0xff300000,
		OK_LED_OFFSET = 0x0020,
		OK_LED_LEN = 0x1000,
	};
	void *ptr_ok_led;
	ptr_ok_led = mmap(NULL, OK_LED_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd_ok_led, OK_LED_BASE);
	if (ptr_ok_led == MAP_FAILED)
	{
		printf("The ok_led-message cann't be sent. mmap = %d\n", (void *)ptr_ok_led);
	}
	else
	{
		uint32_t *p_ok_led;
		p_ok_led = (uint32_t *)((uint8_t*)ptr_ok_led + 0x0020);
		*p_ok_led |= (1 << 2);
	}
}