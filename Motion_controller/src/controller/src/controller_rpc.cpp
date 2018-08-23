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
    controller_client_ptr_(NULL)
    
{

}

ControllerRpc::~ControllerRpc()
{

}

void ControllerRpc::init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, VirtualCore1* virtual_core1_ptr, TpComm* tp_comm_ptr,
                    ControllerSm* state_machine_ptr, ToolManager* tool_manager_ptr, CoordinateManager* coordinate_manager_ptr,
                    RegManager* reg_manager_ptr, DeviceManager* device_manager_ptr, fst_mc::MotionControl* motion_control_ptr,
                    ControllerClient* controller_client_ptr)
{
    log_ptr_ = log_ptr;
    param_ptr_ = param_ptr;
    virtual_core1_ptr_ = virtual_core1_ptr;
    tp_comm_ptr_ = tp_comm_ptr;
    state_machine_ptr_ = state_machine_ptr;
    tool_manager_ptr_ = tool_manager_ptr;
    coordinate_manager_ptr_ = coordinate_manager_ptr;
    reg_manager_ptr_ = reg_manager_ptr;
    device_manager_ptr_ = device_manager_ptr;
    motion_control_ptr_ = motion_control_ptr;
    controller_client_ptr_ = controller_client_ptr;
    initRpcTable();
    initRpcQuickSearchTable();
    publish_.init(log_ptr, param_ptr, virtual_core1_ptr, tp_comm_ptr, state_machine_ptr, motion_control_ptr);
}

void ControllerRpc::processRpc()
{
    HandleRpcFuncPtr func_ptr;
    std::vector<TpRequestResponse>::iterator it;
    std::vector<TpRequestResponse> request_list = tp_comm_ptr_->popTaskFromRequestList();
    for(it = request_list.begin(); it != request_list.end(); ++it)
    {
        if(tp_comm_ptr_->getResponseSucceed(it->response_data_ptr) == 0)
        {
            func_ptr = getRpcHandlerByHash(it->hash);
            if(func_ptr != NULL)
            {
                (this->*func_ptr)(it->request_data_ptr, it->response_data_ptr);
            }
        }
        tp_comm_ptr_->pushTaskToResponseList(*it);
    }
    publish_.updatePublish();
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


