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
    state_machine_ptr_(NULL)
{

}

ControllerIpc::~ControllerIpc()
{

}

void ControllerIpc::init(fst_log::Logger* log_ptr, ControllerParam* param_ptr,
                            ControllerServer* controller_server_ptr, ControllerClient* controller_client_ptr,
                            RegManager* reg_manager_ptr, ControllerSm* state_machine_ptr)
{
    log_ptr_ = log_ptr;
    param_ptr_ = param_ptr;
    controller_server_ptr_ = controller_server_ptr;
    controller_client_ptr_ = controller_client_ptr;
    reg_manager_ptr_ = reg_manager_ptr;
    state_machine_ptr_ = state_machine_ptr;
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

