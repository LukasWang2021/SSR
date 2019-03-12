#include "controller_rpc.h"

using namespace fst_ctrl;
using namespace fst_log;
using namespace fst_base;
using namespace fst_hal;
using namespace fst_comm;
using namespace fst_mc;


ControllerRpc::ControllerRpc():
    log_ptr_(NULL),
    param_ptr_(NULL),
    virtual_core1_ptr_(NULL),
    tp_comm_ptr_(NULL),
    state_machine_ptr_(NULL),
    tool_manager_ptr_(NULL),
    coordinate_manager_ptr_(NULL),
    reg_manager_ptr_(NULL),
    device_manager_ptr_(NULL),
    motion_control_ptr_(NULL),
    controller_client_ptr_(NULL),
    io_mapping_ptr_(NULL),
    program_launching_(NULL),
    modbus_manager_ptr_(NULL),
    file_manager_ptr_(NULL),
    system_manager_ptr_(NULL),
    param_manager_ptr_(NULL)
{

}

ControllerRpc::~ControllerRpc()
{

}

void ControllerRpc::init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, ControllerPublish* publish_ptr, VirtualCore1* virtual_core1_ptr, 
                    fst_comm::TpComm* tp_comm_ptr, ControllerSm* state_machine_ptr, ToolManager* tool_manager_ptr, 
                    CoordinateManager* coordinate_manager_ptr, RegManager* reg_manager_ptr, fst_hal::DeviceManager* device_manager_ptr, 
                    fst_mc::MotionControl* motion_control_ptr, fst_base::ControllerClient* controller_client_ptr,
                    IoMapping* io_mapping_ptr,fst_hal::IoManager* io_manager_ptr, ProgramLaunching* program_launching, 
                    fst_base::FileManager* file_manager, fst_ctrl::SystemManager* system_manager, fst_mc::ParamManager* param_manager)
{
    log_ptr_ = log_ptr;
    param_ptr_ = param_ptr;
    publish_ptr_ = publish_ptr;
    virtual_core1_ptr_ = virtual_core1_ptr;
    tp_comm_ptr_ = tp_comm_ptr;
    state_machine_ptr_ = state_machine_ptr;
    tool_manager_ptr_ = tool_manager_ptr;
    coordinate_manager_ptr_ = coordinate_manager_ptr;
    reg_manager_ptr_ = reg_manager_ptr;
    device_manager_ptr_ = device_manager_ptr;
    motion_control_ptr_ = motion_control_ptr;
    controller_client_ptr_ = controller_client_ptr;
    io_mapping_ptr_ = io_mapping_ptr;
    io_manager_ptr_ = io_manager_ptr;//for info list
    program_launching_ = program_launching;
    file_manager_ptr_ = file_manager;
    system_manager_ptr_ = system_manager;
    param_manager_ptr_ = param_manager;

    // get the modbus_manager_ptr from device_manager.
    std::vector<fst_hal::DeviceInfo> device_list = device_manager_ptr_->getDeviceList();
    for(unsigned int i = 0; i < device_list.size(); ++i)
    {
        if (device_list[i].type == DEVICE_TYPE_MODBUS)
        {
            BaseDevice* device_ptr = device_manager_ptr_->getDevicePtrByDeviceIndex(device_list[i].index);
            if(device_ptr == NULL || modbus_manager_ptr_ != NULL) break;
            modbus_manager_ptr_ = static_cast<ModbusManager*>(device_ptr);
        } 
        else if(device_list[i].type == DEVICE_TYPE_FST_SAFETY)// get the safety device ptr.
        {
            BaseDevice* device_ptr = device_manager_ptr_->getDevicePtrByDeviceIndex(device_list[i].index);
            if(device_ptr == NULL || safety_device_ptr_ != NULL) break;
            safety_device_ptr_ = static_cast<FstSafetyDevice*>(device_ptr);
        }
    }

    device_version_.init(log_ptr_, motion_control_ptr_, io_manager_ptr_, safety_device_ptr_);

    if (modbus_manager_ptr_ != NULL)
    {
        ErrorCode error_code = modbus_manager_ptr_->initDevices();
        if (error_code != SUCCESS)
            ErrorMonitor::instance()->add(error_code);
    }

    initRpcTable();
    initRpcQuickSearchTable();
}

void ControllerRpc::processRpc()
{
    HandleRpcFuncPtr func_ptr;
    TpRequestResponse task;
    if(tp_comm_ptr_->popTaskFromRequestList(&task))
    {
        if(tp_comm_ptr_->getResponseSucceed(task.response_data_ptr) == 0)
        {
            func_ptr = getRpcHandlerByHash(task.hash);
            if(func_ptr != NULL)
            {
                (this->*func_ptr)(task.request_data_ptr, task.response_data_ptr);
            }
        }
        tp_comm_ptr_->pushTaskToResponseList(task);
    }
}

void ControllerRpc::initRpcQuickSearchTable()
{
    unsigned int remainder;
    for(unsigned int i = 0; i < rpc_table_.size(); ++i)
    {
        remainder = rpc_table_[i].hash % QUICK_SEARCH_TABLE_SIZE;
        rpc_quick_search_table_[remainder].push_back(rpc_table_[i]);
    }
}

ControllerRpc::HandleRpcFuncPtr ControllerRpc::getRpcHandlerByHash(unsigned int hash)
{
    unsigned int remainder = hash % QUICK_SEARCH_TABLE_SIZE;
    for(unsigned int i = 0; i < rpc_quick_search_table_[remainder].size(); ++i)
    {
        if(rpc_quick_search_table_[remainder][i].hash == hash)
        {
            return rpc_quick_search_table_[remainder][i].rpc_func_ptr;
        }
    }
    return NULL;
}

void ControllerRpc::recordLog(ErrorCode log_code, ErrorCode error_code, std::string rpc_path)
{
    std::string log_str("run ");
    log_str += rpc_path;
    if(error_code == SUCCESS)
    {
        log_str += " success";
        ServerAlarmApi::GetInstance()->sendOneAlarm(log_code, log_str);
    }
    else
    {
        log_str += " failed";
        ServerAlarmApi::GetInstance()->sendOneAlarm(error_code, log_str);
    }    
}

