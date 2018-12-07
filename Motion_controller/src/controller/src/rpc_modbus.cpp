#include "controller_rpc.h"

using namespace fst_ctrl;

//"/rpc/modbus/newServer",
void ControllerRpc::handleRpc0x0000D1B2(void* request_data_ptr, void* response_data_ptr)
{

}

//"/rpc/modbus/deleteServer",
void ControllerRpc::handleRpc0x00006C22(void* request_data_ptr, void* response_data_ptr)
{

}

//"/rpc/modbus/saveServerConfig",
void ControllerRpc::handleRpc0x000050E7(void* request_data_ptr, void* response_data_ptr)
{

}
//"/rpc/modbus/getServerConfig",
void ControllerRpc::handleRpc0x00016947(void* request_data_ptr, void* response_data_ptr)
{

}

//"/rpc/modbus/newClient",
void ControllerRpc::handleRpc0x00009F84(void* request_data_ptr, void* response_data_ptr)
{

}

//"/rpc/modbus/deleteClient",
void ControllerRpc::handleRpc0x00014CF4(void* request_data_ptr, void* response_data_ptr)
{

}

//"/rpc/modbus/saveClientConfig",
void ControllerRpc::handleRpc0x00002B57(void* request_data_ptr, void* response_data_ptr)
{
    
}
//"/rpc/modbus/getClientConfig",
void ControllerRpc::handleRpc0x0000FC17(void* request_data_ptr, void* response_data_ptr)
{

}

//"/rpc/modbus/setConnectionStatus",
void ControllerRpc::handleRpc0x00010C63(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Bool* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Bool*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    //rs_data_ptr->data.data = modbus_manager_ptr_->getConnection();

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getConnectionStatus"));
}

//"/rpc/modbus/getConnectionStatus"
void ControllerRpc::handleRpc0x0000E973(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Bool* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Bool*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    //rs_data_ptr->data.data = modbus_manager_ptr_->getConnection();

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getConnectionStatus"));
}