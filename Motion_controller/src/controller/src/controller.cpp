#include "controller.h"
#include "system/core_comm_system.h"
#include "common/core_comm_servo_datatype.h"
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace user_space;
using namespace core_comm_space;
using namespace servo_comm_space;
using namespace log_space;
using namespace base_space;
using namespace system_model_space;
using namespace hal_space;

Controller* Controller::instance_ = NULL;
uint32_t* g_isr_ptr_ = NULL;

Controller::Controller():
    is_exit_(false),
    servo_1001_ptr_(NULL),
    cpu_comm_ptr_(NULL),
    io_digital_dev_ptr_(NULL)
{    
    for(size_t i = 0; i < AXIS_NUM; ++i)
    {
        servo_comm_ptr_[i] = NULL;
        axis_ptr_[i] = NULL;
    }
    for (size_t i = 0; i < GROUP_NUM; ++i)
    {
        group_ptr_[i] = NULL;
    }

    config_ptr_ = new ControllerConfig();
}


Controller::~Controller()
{
    rt_thread_.join();
    routine_thread_.join();

    for (size_t i = 0; i < GROUP_NUM; ++i)
    {
        if (group_ptr_[i] != NULL)
        {
            delete group_ptr_[i];
            group_ptr_[i] = NULL;
        }
    }

    for(size_t i = 0; i < AXIS_NUM; ++i)
    {
        if (axis_ptr_[i] != NULL)
        {
            size_t dot_pos = axes_config_[i].actuator.servo.find_first_of('.');
            if(dot_pos != std::string::npos)
            {
                std::string name_str = axes_config_[i].actuator.servo.substr(0, dot_pos);
                if(name_str.compare("servo_1000") == 0)
                {
                    axis_space::Axis1000* axis_ptr = static_cast<axis_space::Axis1000*>(axis_ptr_[i]);
                    delete axis_ptr;
                    axis_ptr_[i] = NULL;
                }
                else if (name_str.compare("servo_1001") == 0)
                {
                    axis_space::Axis1001* axis_ptr = static_cast<axis_space::Axis1001*>(axis_ptr_[i]);
                    delete axis_ptr;
                    axis_ptr_[i] = NULL;
                }
            }
        }
    }

    if (servo_1001_ptr_ != NULL)
    {
        delete servo_1001_ptr_;
        servo_1001_ptr_ = NULL;
    }

    if(config_ptr_ != NULL)
    {
        delete config_ptr_;
        config_ptr_ = NULL;
    }

    if(io_digital_dev_ptr_ != NULL)
    {
        delete io_digital_dev_ptr_;
        io_digital_dev_ptr_ = NULL;
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
    uint32_t isr = 0;
    log_manager_.init("controller", &isr);
    ErrorCode error_code = bootUp();
    if(SUCCESS != error_code)
        return error_code;

    /********************init self components***********************************/
    io_digital_dev_ptr_ = new hal_space::Io1000();
    io_digital_dev_ptr_->init(config_ptr_->dio_exist_);
    dev_ptr_list.push_back(io_digital_dev_ptr_);
    
    //axis init
    for (size_t i = 0; i < axes_config_.size(); ++i)
    {
        axis_model_ptr_[i] = model_manager_.getAxisModel(axes_config_[i].axis_id);
        
        size_t dot_pos = axes_config_[i].actuator.servo.find_first_of('.');
        if(dot_pos != std::string::npos)
        {
            std::string name_str = axes_config_[i].actuator.servo.substr(0, dot_pos);
            if(name_str.compare("servo_1000") == 0)
            {
                LogProducer::info("main", "Controller new axis[%d]: pmsm", axes_config_[i].axis_id);
                axis_ptr_[i] = new axis_space::Axis1000(axes_config_[i].axis_id);
            }
            else if (name_str.compare("servo_1001") == 0)
            {
                LogProducer::info("main", "Controller new axis[%d]: stepper", axes_config_[i].axis_id);
                axis_ptr_[i] = new axis_space::Axis1001(axes_config_[i].axis_id);
            }
            else
            {
                LogProducer::error("main", "Controller new axis[%d] failed", axes_config_[i].axis_id);
                return CONTROLLER_INIT_FAILED;
            }
        }
        //download servo parameters
        if (!downloadServoParams(i))
            return CONTROLLER_INIT_FAILED;
        
        cpu_comm_ptr_ = servo_1001_ptr_->getCpuCommPtr();
        assert(cpu_comm_ptr_ != NULL);
        bool ret = axis_ptr_[i]->init(cpu_comm_ptr_, servo_comm_ptr_[i], axis_model_ptr_[i], &(axes_config_[i]));
        if (!ret)
        {
            LogProducer::error("main", "Controller axis[%d] initialization failed", axes_config_[i].axis_id);
            return ret;
        }
    }

    //group init
    for (size_t i = 0; i < group_config_.size(); ++i)
    {
        group_model_ptr_[i] = model_manager_.getGroupModel(group_config_[i].group_id);
        group_ptr_[i] = new group_space::MotionControl(group_config_[i].group_id);
        bool ret = group_ptr_[i]->init(cpu_comm_ptr_, group_model_ptr_[i], &(group_config_[i]));
        if (!ret)
        {
            LogProducer::error("main", "Controller group[%d] initialization failed", group_config_[i].group_id);
            return ret;
        }
    }
    if(SUCCESS != tool_manager_.init())
    {
        LogProducer::error("main", "Controller tool manager initialization failed");
        return error_code;
    }
    if(SUCCESS != coordinate_manager_.init())
    {
        LogProducer::error("main", "Controller coord manager initialization failed");
        return error_code;
    }
    //add axis to group according config.xml
    for (unsigned int i = 0; i < group_config_.size(); ++i)
    {
        for (unsigned int j = 0; j < group_config_[i].axis_id.size(); ++j)
        {
            for (unsigned int k = 0; k < axes_config_.size(); ++k)
            {
                if(group_config_[i].axis_id[j] == axis_ptr_[k]->getID())
                {
                    error_code = group_ptr_[i]->mcAddAxisToGroup(j, *axis_ptr_[k]);
                    if (error_code != SUCCESS)
                    {
                        LogProducer::error("main", "Controller group[%d] add axis[%d] failed", group_config_[i].group_id, group_config_[i].axis_id[j]);
                        return error_code;
                    }
                }
            }
        }
        error_code = group_ptr_[i]->initApplication(&coordinate_manager_, &tool_manager_);
        if (SUCCESS != error_code)
        {
            LogProducer::error("main", "Controller group[%d] application initialization failed", group_config_[i].group_id);
            return error_code;
        }
    }
   
    //setting ISR as soon
    int32_t size = 0;
    g_isr_ptr_ = (uint32_t*)axis_ptr_[0]->rtmReadAxisFdbPdoPtr(&size);

	error_code = tp_comm_.init();
    if(error_code != SUCCESS)
    {
        LogProducer::error("main", "Controller TP comm initialization failed");
        return error_code;
    }

	publish_.init(&tp_comm_, cpu_comm_ptr_, axis_ptr_, io_digital_dev_ptr_);
	rpc_.init(&tp_comm_, &publish_, cpu_comm_ptr_, servo_comm_ptr_, axis_ptr_, axis_model_ptr_, group_ptr_, &file_manager_, io_digital_dev_ptr_);

    if(!routine_thread_.run(&controllerRoutineThreadFunc, this, config_ptr_->routine_thread_priority_))
    {
        return CONTROLLER_CREATE_ROUTINE_THREAD_FAILED;
    } 

    if(!rt_thread_.run(&controllerRealTimeThreadFunc, this, config_ptr_->realtime_thread_priority_))
    {
        return CONTROLLER_CREATE_RT_THREAD_FAILED;
    }    

    LogProducer::warn("main", "Controller init success");
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
    usleep(config_ptr_->routine_cycle_time_);
	publish_.processPublish();
    uploadErrorCode();
}

void Controller::runRtThreadFunc()
{
    usleep(config_ptr_->realtime_cycle_time_);
 
    axis_ptr_[13]->processFdbPdoCurrent(&fdb_current_time_stamp_);
    axis_ptr_[0]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[1]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[2]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[3]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[4]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[5]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[6]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[7]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[8]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[9]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[10]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[11]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[12]->processFdbPdoSync(fdb_current_time_stamp_);

    for (size_t i = 0; i < AXIS_NUM; ++i)
    {
        axis_ptr_[i]->processStateMachine();
    }

    for (size_t i = 0; i < GROUP_NUM; ++i)
    {
        group_ptr_[i]->processFdbPdo();
        group_ptr_[i]->processStateMachine();
    }
    rpc_.processRpc();
}


ErrorCode Controller::bootUp(void)
{
    //load self parameters
    if(!config_ptr_->load()){
        LogProducer::error("main", "Failed to load controller config files");
		return CONTROLLER_INIT_FAILED;
    }
    LogProducer::setLoggingLevel((MessageLevel)config_ptr_->log_level_);

    //loading configuration by model_manager
    bool ret = model_manager_.init();
    if(!ret)
    {
        LogProducer::error("main", "Controller model manager initialization failed");
        return CONTROLLER_INIT_FAILED;
    }
    if(!model_manager_.load())
    {
        LogProducer::error("main", "Controller model manager load failed");
        return CONTROLLER_INIT_FAILED;
    }
    AxesConfig* axes_config_ptr = model_manager_.getAxesConfig();
    if (!axes_config_ptr->load())
    {
        LogProducer::error("main", "Controller axes config load failed");
        return CONTROLLER_INIT_FAILED;
    }
    axes_config_ = axes_config_ptr->getRef();

    GroupsConfig* group_config_ptr = model_manager_.getGroupsConfig();
    if (!group_config_ptr->load())
    {
        LogProducer::error("main", "Controller group config load failed");
        return CONTROLLER_INIT_FAILED;
    }
    group_config_ = group_config_ptr->getRef();

    //boot up
    ErrorCode error_code;
    error_code = core_comm_system_.initAsMaster();
    if(error_code != SUCCESS)
    {
        return error_code;
    }

    error_code = core_comm_system_.bootAsMaster();
    if(error_code != SUCCESS)
    {
        return error_code;
    }   

    while(true)
    {
        if(core_comm_system_.isAllSlaveBooted())
        {
            break;
        }
        LogProducer::info("main", "wait all slaves booted");
        sleep(1);
    }

    // get configuration
    size_t from_block_number, to_block_number;
    CommBlockData_t* from_block_ptr = core_comm_system_.getFromCommBlockDataPtrList(from_block_number);
    CommBlockData_t* to_block_ptr = core_comm_system_.getToCommBlockDataPtrList(to_block_number);

    // init servo1001
    servo_1001_ptr_ = new servo_comm_space::Servo1001(1, 2);
    assert(servo_1001_ptr_ != NULL);
    
    if(servo_1001_ptr_->initServoCpuComm(from_block_ptr, from_block_number, to_block_ptr, to_block_number))
    {
        LogProducer::info("main", "servo_1001_ptr_ initServoCpuComm success");
    }
    else
    {
        LogProducer::error("main", "servo_1001_ptr_ initServoCpuComm failed");
        return CONTROLLER_INIT_FAILED;
    }
    
    // init non pdo communication channel
    if(servo_1001_ptr_->prepareInit2PreOp(from_block_ptr, from_block_number, to_block_ptr, to_block_number))
    {
        LogProducer::info("main", "servo_1001_ptr_ prepareInit2PreOp success");
    }
    else
    {
        LogProducer::error("main", "servo_1001_ptr_ prepareInit2PreOp failed");
        return CONTROLLER_INIT_FAILED;
    }

    // wait all servo trans to preop
    while(!servo_1001_ptr_->isAllServosInExpectedCommState(CORE_COMM_STATE_PREOP))
    {
        sleep(1);
    }

    // get servo_comm pointers
    for(size_t i = 0; i < axes_config_.size(); ++i)
    {
        servo_comm_ptr_[i] = servo_1001_ptr_->getServoCommPtr(axes_config_[i].servo_id);
        assert(servo_comm_ptr_[i] != NULL);
    }

    // set all opeartion_mode for all servos
    for(size_t i = 0; i < axes_config_.size(); ++i)
    {
        if(servo_comm_ptr_[i]->doServoCmdWriteParameter(SERVO_PARAM_OP_MODE, (int32_t)SERVO_OP_MODE_PROFILE_POSITION_MODE) != SUCCESS)
        {
            LogProducer::error("main", "servo_comm_ptr[%d] write op_mode failed", i);
            return CONTROLLER_INIT_FAILED;
        }
    }

    // connect fdb pdo channel to self
    if(servo_1001_ptr_->preparePreOp2SafeOp(to_block_ptr, to_block_number))
    {
        LogProducer::info("main", "servo_1001_ptr_ preparePreOp2SafeOp success");
    }
    else
    {
        LogProducer::error("main", "servo_1001_ptr_ preparePreOp2SafeOp failed");
        return CONTROLLER_INIT_FAILED;
    }

    // transfer all servos to SAFEOP
    if(!servo_1001_ptr_->doServoCmdTransCommState(CORE_COMM_STATE_SAFEOP))
    {
        LogProducer::error("main", "servo_1001_ptr trans state to safeop failed");
        return CONTROLLER_INIT_FAILED;
    }
    
    // wait all servo trans to safeop
    while(!servo_1001_ptr_->isAllServosInExpectedCommState(CORE_COMM_STATE_SAFEOP))
    {
        sleep(1);
    }

    // transfer all servos to OP
    if(!servo_1001_ptr_->doServoCmdTransCommState(CORE_COMM_STATE_OP))
    {
        LogProducer::error("main", "servo_1001_ptr trans state to op failed");
        return CONTROLLER_INIT_FAILED;
    }
    
    // wait all servo trans to op
    while(!servo_1001_ptr_->isAllServosInExpectedCommState(CORE_COMM_STATE_OP))
    {
        sleep(1);
    }
    return SUCCESS;
}

void Controller::uploadErrorCode(void)
{
    TpEventMsg event;
    event.type = 0;
    event.to_id = 0;
    while (ErrorQueue::instance().pop(event.event_data))
    {
        tp_comm_.sendEvent(event);
        LogProducer::error("Upload", "Error: 0x%llx", event.event_data);
    }
}

void Controller::processDevice(void)
{
    ErrorCode ret = 0;
    for(vector<BaseDevice*>::iterator iter = dev_ptr_list.begin(); iter != dev_ptr_list.end(); iter++)
    {
        ret = (*iter)->updateStatus();
        if (ret != SUCCESS)
        {
            ErrorQueue::instance().push(ret);
        }
    }
}

bool Controller::downloadServoParams(int32_t axis_id)
{
    BufferAppData2001_t params;
    memset(&params, 0, sizeof(BufferAppData2001_t));
    int32_t value = 0;
    for(size_t i = 0; i < SERVO_PARAM_BUFFER_SIZE; ++i)
    {
        if (axis_model_ptr_[axis_id]->actuator.servo_ptr->get(i, &value))
        {
            params.param[i] = value;
        }
        else
        {
            LogProducer::error("main", "Controller axis[%d] get servo parameter[%d] failed", axis_id, i);
            return false;
        }
    }

    int32_t* sync_ack_ptr = NULL;
    if (servo_comm_ptr_[axis_id]->downloadServoParameters(&params))
    {
        servo_comm_ptr_[axis_id]->triggerServoCmdDownloadParameters(&sync_ack_ptr);
        int32_t count = 0;
        while(!servo_comm_ptr_[axis_id]->isServoAsyncServiceFinish(sync_ack_ptr) && (count <= 100))
        {
            usleep(10000);
            count++;
        }
        if (count > 100)
        {
            LogProducer::error("main", "Controller axis[%d] download servo parameter timeout", axis_id);
            return false;
        }
    }
    else
    {
        LogProducer::error("main", "Controller axis[%d] download servo parameter failed", axis_id);
        return false;
    }
    LogProducer::info("main", "Controller axis[%d] download servo parameter success", axis_id);
    return true;
}

void* controllerRoutineThreadFunc(void* arg)
{
    Controller* controller_ptr = static_cast<Controller*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("controller_routine", g_isr_ptr_);
    while(!controller_ptr->isExit())
    {
        controller_ptr->runRoutineThreadFunc();
    }
    std::cout<<"controller_rountine exit"<<std::endl;
    return NULL;
}


void* controllerRealTimeThreadFunc(void* arg)
{
    Controller* controller_ptr = static_cast<Controller*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("controller_RT", g_isr_ptr_);
    while(!controller_ptr->isExit())
    {
        controller_ptr->runRtThreadFunc();
    }
    std::cout<<"controller_RT exit"<<std::endl;
    return NULL;
}

