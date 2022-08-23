#include "controller.h"
#include "system/core_comm_system.h"
#include "common/core_comm_servo_datatype.h"
#include <sys/syscall.h>
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
using namespace group_space;

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
    planner_thread_.join();
    priority_thread_.join();
    routine_thread_.join();

    for(size_t i = 0; i < GROUP_NUM; ++i)
    {
        if(group_ptr_[i] != NULL)
        {
            delete group_ptr_[i];
            group_ptr_[i] = NULL;
        }
    }

    for(size_t i = 0; i < AXIS_NUM; ++i)
    {
        if(axis_ptr_[i] != NULL)
        {
            axis_space::Axis1000* axis_ptr = static_cast<axis_space::Axis1000*>(axis_ptr_[i]);
            delete axis_ptr;
            axis_ptr_[i] = NULL;
        }
    }

    if(servo_1001_ptr_ != NULL)
    {
        delete servo_1001_ptr_;
        servo_1001_ptr_ = NULL;
    }

    if(config_ptr_ != NULL)
    {
        delete config_ptr_;
        config_ptr_ = NULL;
    }

    if (io_digital_dev_ptr_ != NULL)
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

    io_safety_dev_ptr_ = new hal_space::IoSafety();
    if(io_safety_dev_ptr_->init(config_ptr_->safety_exist_) == false)
    {
        LogProducer::error("main", "Controller safety io initialization failed");
        return CONTROLLER_INIT_FAILED;
    }
    dev_ptr_list.push_back(io_safety_dev_ptr_);

    //axis init
    for (size_t i = 0; i < axes_config_.size(); ++i)
    {
        axis_model_ptr_[i] = model_manager_.getAxisModel(axes_config_[i].axis_id);
        axis_ptr_[i] = new axis_space::Axis1000(axes_config_[i].axis_id);

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


    force_model_ptr_ = model_manager_.getForceModel(forces_config_[0].force_id);
    //download force control parameters
    if (!downloadForceControlParams())
        return CONTROLLER_INIT_FAILED;


    error_code = tool_manager_.init();
    if(SUCCESS != error_code)
    {
        LogProducer::error("main", "Controller tool manager initialization failed");
        return error_code;
    }
    error_code = coordinate_manager_.init();
    if(SUCCESS != error_code)
    {
        LogProducer::error("main", "Controller coord manager initialization failed");
        return error_code;
    }
    error_code = reg_manager_.init();
    if(SUCCESS != error_code)
    {
        LogProducer::error("main", "Controller reg manager initialization failed");
        return error_code;
    }
    error_code = fio_device_.init(1);
    if(SUCCESS != error_code)
    {
        LogProducer::error("main", "Controller fio_device initialization failed");
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

    //force sensor
	force_sensor_.init(group_ptr_, cpu_comm_ptr_, &force_model_ptr_);

	publish_.init(&tp_comm_, cpu_comm_ptr_, axis_ptr_, group_ptr_, io_digital_dev_ptr_, io_safety_dev_ptr_, &force_sensor_);
    rpc_.init(&tp_comm_, &publish_, cpu_comm_ptr_, servo_comm_ptr_, axis_ptr_, axis_model_ptr_, group_ptr_, &file_manager_, 
    io_digital_dev_ptr_, &tool_manager_, &coordinate_manager_, &reg_manager_, &fio_device_, force_model_ptr_);
	
	if(!InterpCtrl::instance().setApi(group_ptr_,io_digital_dev_ptr_) ||
       !InterpCtrl::instance().init() || 
       !InterpCtrl::instance().regSyncCallback(std::bind(&MotionControl::nextMovePermitted, group_ptr_[0])))
    {
        LogProducer::error("main", "Controller interpreter initialization failed");
        return CONTROLLER_INIT_FAILED;
    }
    else if(!InterpCtrl::instance().run())
    {
        LogProducer::error("main", "Controller interpreter start failed");
        return CONTROLLER_INIT_FAILED;
    }
    if(!routine_thread_.run(&controllerRoutineThreadFunc, this, config_ptr_->routine_thread_priority_))
    {
        return CONTROLLER_CREATE_ROUTINE_THREAD_FAILED;
    } 
    if(!planner_thread_.run(&controllerPlannerThreadFunc, this, config_ptr_->planner_thread_priority_))
    {
        return CONTROLLER_CREATE_ROUTINE_THREAD_FAILED;
    } 
    if(!priority_thread_.run(&controllerPriorityThreadFunc, this, config_ptr_->priority_thread_priority_))
    {
        return CONTROLLER_CREATE_ROUTINE_THREAD_FAILED;
    } 
    if(!rt_thread_.run(&controllerRealTimeThreadFunc, this, config_ptr_->realtime_thread_priority_))
    {
        return CONTROLLER_CREATE_RT_THREAD_FAILED;
    }    
    if(!rpc_thread_.run(&controllerRpcThreadFunc, this, config_ptr_->rpc_thread_priority_))
    {
        return CONTROLLER_CREATE_RPC_THREAD_FAILED;
    }
    if(!online_traj_thread_.run(&controllerOnlineTrajThreadFunc, this, config_ptr_->online_traj_thread_priority_))
    {
        return CONTROLLER_CREATE_ONLIE_THREAD_FAILED;
    }
    sleep(1);
    if(group_ptr_[0]->checkZeroOffset())
    {
        return CONTROLLER_INIT_FAILED;
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

    axis_ptr_[9]->processFdbPdoCurrent(&fdb_current_time_stamp_);
    axis_ptr_[0]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[1]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[2]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[3]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[4]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[5]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[6]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[7]->processFdbPdoSync(fdb_current_time_stamp_);
    axis_ptr_[8]->processFdbPdoSync(fdb_current_time_stamp_);

    for (size_t i = 0; i < AXIS_NUM; ++i)
    {
        axis_ptr_[i]->processStateMachine();
    }

    for (size_t i = 0; i < GROUP_NUM; ++i)
    {
        group_ptr_[i]->processFdbPdo();
        group_ptr_[i]->processStateMachine();
    }
    // rpc_.processRpc();
	publish_.processPublish();
    uploadErrorCode();
    group_ptr_[0]->ringCommonTask();
    fio_device_.FioHeartBeatLoopQuery();
}

void Controller::runRpcThreadFunc()
{
    usleep(config_ptr_->rpc_cycle_time_);
    rpc_.processRpc();
}

void Controller::runPlannerThreadFunc()
{
    usleep(config_ptr_->planner_cycle_time_);
    group_ptr_[0]->ringPlannerTask();
}

void Controller::runOnlineTrajThreadFunc()
{
    usleep(config_ptr_->online_traj_cycle_time_);
    group_ptr_[0]->ringOnlineTrajTask();
}

void Controller::runPriorityThreadFunc()
{
    usleep(config_ptr_->priority_cycle_time_);
    group_ptr_[0]->ringPriorityTask();
}

void Controller::runRtThreadFunc()
{
    usleep(config_ptr_->realtime_cycle_time_);
    group_ptr_[0]->ringRealTimeTask();
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

    ForcesConfig* forces_config_ptr = model_manager_.getForcesConfig();
    if (!forces_config_ptr->load())
    {
        LogProducer::error("main", "Controller forces config load failed");
        return CONTROLLER_INIT_FAILED;
    }
    forces_config_ = forces_config_ptr->getRef();

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
        if(servo_comm_ptr_[i]->doServoCmdWriteParameter(SERVO_PARAM_OP_MODE, (int32_t)SERVO_OP_MODE_INTERPOLATED_POSITION_MODE) != SUCCESS)
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

    // connect command pdo channel to self
    if(servo_1001_ptr_->prepareSafeOp2Op(from_block_ptr, from_block_number))
    {
        LogProducer::info("main", "servo_1001_ptr_ prepareSafeOp2Op success");
    }
    else
    {
        LogProducer::error("main", "servo_1001_ptr_ prepareSafeOp2Op failed");
        return CONTROLLER_INIT_FAILED;
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
        DisableControllerByErrorCode(event.event_data);
    }
}

void Controller::DisableControllerByErrorCode(ErrorCode err)
{
    for (size_t i = 0; i < GROUP_NUM; ++i)
    {
        GroupStatus_e status = GROUP_STATUS_UNKNOWN;
        bool in_pos = false;
        group_ptr_[i]->mcGroupReadStatus(status, in_pos);
        if (status != GROUP_STATUS_ERROR_STOP && status != GROUP_STATUS_DISABLED)
        {
            group_ptr_[i]->mcGroupDisable();
            InterpCtrl::instance().abort();
            group_ptr_[i]->stopGroup();
            group_ptr_[i]->clearGroup();
            group_ptr_[i]->clearTeachGroup();
        }
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

bool Controller::downloadForceControlParams()
{
    CommRegForceControlParam_t params;
    memset(&params, 0, sizeof(CommRegForceControlParam_t));
    int32_t value = 0;
    for(size_t i = 0; i < COMM_REG2_PARAMETER_NUMBER; ++i)
    {
        if (force_model_ptr_->force_param_ptr->get(i, &value))
        {
            params.parameter[i] = value;
        }
        else
        {
            LogProducer::error("main", "Controller force_control get parameter[%d] failed", i);
            return false;
        }
    }

    if (!cpu_comm_ptr_->setForceControlParameters(&params))
    {
        LogProducer::error("main", "Controller force_control download parameter failed");
        return false;
    }
    LogProducer::info("main", "Controller force_control download parameter success");
    
    return true;
}

void* controllerRoutineThreadFunc(void* arg)
{
    Controller* controller_ptr = static_cast<Controller*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("controller_routine", g_isr_ptr_);
    LogProducer::warn("main","controller_routine TID is %ld", syscall(SYS_gettid));
    while(!controller_ptr->isExit())
    {
        controller_ptr->runRoutineThreadFunc();
    }
    std::cout<<"controller_rountine exit"<<std::endl;
    return NULL;
}

void* controllerRpcThreadFunc(void* arg)
{
    Controller* controller_ptr = static_cast<Controller*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("controller_rpc", g_isr_ptr_);
    LogProducer::warn("main","controller_rpc TID is %ld", syscall(SYS_gettid));
    while(!controller_ptr->isExit())
    {
        controller_ptr->runRpcThreadFunc();
    }
    std::cout<<"controller_rpc exit"<<std::endl;
    return NULL;
}

void* controllerPlannerThreadFunc(void* arg)
{
    Controller* controller_ptr = static_cast<Controller*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("controller_planner", g_isr_ptr_);
    LogProducer::warn("main","controller_planner TID is %ld", syscall(SYS_gettid));
    while(!controller_ptr->isExit())
    {
        controller_ptr->runPlannerThreadFunc();
    }
    std::cout<<"controller_planner exit"<<std::endl;
    return NULL;
}

void* controllerOnlineTrajThreadFunc(void* arg)
{
    Controller* controller_ptr = static_cast<Controller*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("controller_online_traj", g_isr_ptr_);
    LogProducer::warn("main","controller_onlie_traj TID is %ld", syscall(SYS_gettid));
    while(!controller_ptr->isExit())
    {
        controller_ptr->runOnlineTrajThreadFunc();
    }
    std::cout<<"controller_online_traj exit"<<std::endl;
    return NULL;
}

void* controllerPriorityThreadFunc(void* arg)
{
    Controller* controller_ptr = static_cast<Controller*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("controller_priority", g_isr_ptr_);
    LogProducer::warn("main","controller_priority TID is %ld", syscall(SYS_gettid));
    while(!controller_ptr->isExit())
    {
        controller_ptr->runPriorityThreadFunc();
    }
    std::cout<<"controller_priority exit"<<std::endl;
    return NULL;
}

void stack_prefault(void) {
    unsigned char dummy[1024 * 1024];//The maximum stack size which is guaranteed safe to access without faulting
    memset(dummy, 0, 1024 * 1024);
}

void* controllerRealTimeThreadFunc(void* arg)
{
    Controller* controller_ptr = static_cast<Controller*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("controller_RT", g_isr_ptr_);
    LogProducer::warn("main","controller_RT TID is %ld", syscall(SYS_gettid));
    
    if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) 
    {
        LogProducer::error("main","controller_RT mlockall failed");
    }
    //Pre-fault our stack
    stack_prefault();

    while(!controller_ptr->isExit())
    {
        controller_ptr->runRtThreadFunc();
    }
    std::cout<<"controller_RT exit"<<std::endl;
    return NULL;
}


