#include "controller_ipc.h"

using namespace fst_ctrl;
using namespace fst_log;
using namespace fst_base;

ControllerIpc::ControllerIpc():
    log_ptr_(NULL),
    param_ptr_(NULL),
    controller_server_ptr_(NULL),
    controller_client_ptr_(NULL),
    reg_manager_ptr_(NULL),
    state_machine_ptr_(NULL),
    device_manager_ptr_(NULL),
    modbus_manager_ptr_(NULL),
    io_mapping_ptr_(NULL)
{

}

ControllerIpc::~ControllerIpc()
{

}

void ControllerIpc::init(fst_log::Logger* log_ptr, ControllerParam* param_ptr,
                            ControllerServer* controller_server_ptr, ControllerClient* controller_client_ptr,
                            RegManager* reg_manager_ptr, ControllerSm* state_machine_ptr, fst_hal::DeviceManager* device_manager_ptr,
                            IoMapping* io_mapping_ptr)
{
    log_ptr_ = log_ptr;
    param_ptr_ = param_ptr;
    controller_server_ptr_ = controller_server_ptr;
    controller_client_ptr_ = controller_client_ptr;
    reg_manager_ptr_ = reg_manager_ptr;
    state_machine_ptr_ = state_machine_ptr;
    device_manager_ptr_ = device_manager_ptr;
    io_mapping_ptr_ = io_mapping_ptr;

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
    }

    initIpcTable();
}

void ControllerIpc::processIpc()
{
    HandleIpcFuncPtr func_ptr;
    std::vector<ProcessCommRequestResponse>::iterator it;
    std::vector<ProcessCommRequestResponse> request_list = controller_server_ptr_->popTaskFromRequestList();
    for(it = request_list.begin(); it != request_list.end(); ++it)
    {
        func_ptr = ipc_table_[it->cmd_id].ipc_func_ptr;
        if(func_ptr != NULL)
        {
            (this->*func_ptr)(it->request_data_ptr, it->response_data_ptr);
        }
        controller_server_ptr_->pushTaskToResponseList(*it);
    }

    controller_client_ptr_->handleSubscribe();
    controller_client_ptr_->handleEvent();
}

