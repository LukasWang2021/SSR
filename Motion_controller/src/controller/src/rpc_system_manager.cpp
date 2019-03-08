#include "controller_rpc.h"

using namespace fst_ctrl;

//"/rpc/system_manager/moveInstall"	
void ControllerRpc::handleRpc0x00015D3C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    system_manager_ptr_->controllerBackup();
    rs_data_ptr->data.data = SUCCESS;

    recordLog(SYSTEM_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/system_manager/moveInstall"));
}
//"/rpc/system_manager/moveFinish"	
void ControllerRpc::handleRpc0x00016008(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    rs_data_ptr->data.data = system_manager_ptr_->getRunning();//0:running finished, 1:is running
    rs_data_ptr->error_code.data = SUCCESS;

    recordLog(SYSTEM_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/system_manager/moveFinish"));

}
//"/rpc/system_manager/restoreInstall"	
void ControllerRpc::handleRpc0x0000DCBC(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    system_manager_ptr_->controllerRestore();
    rs_data_ptr->data.data = SUCCESS;

    recordLog(SYSTEM_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/system_manager/restoreInstall"));
}
//"/rpc/system_manager/restoreFininsh"	
void ControllerRpc::handleRpc0x00011CE8(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    rs_data_ptr->data.data = system_manager_ptr_->getRunning();//0:false, running finished, 1:true,is running
    rs_data_ptr->error_code.data = SUCCESS;

    recordLog(SYSTEM_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/system_manager/restoreFininsh"));

}

