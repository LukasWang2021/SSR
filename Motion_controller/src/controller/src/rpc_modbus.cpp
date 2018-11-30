#include "controller_rpc.h"

using namespace fst_ctrl;

//"/rpc/modbus/createServer", 
void ControllerRpc::handleRpc0x00017982(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusTcpServer* rq_data_ptr = static_cast<RequestMessageType_ModbusTcpServer*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    struct timeval time_val;
    time_val.tv_sec = rq_data_ptr->data.response_timeout.sec;
    time_val.tv_usec = rq_data_ptr->data.response_timeout.usec;

    rs_data_ptr->data.data = modbus_manager_ptr_->setResponseTimeout(time_val);

    if (rs_data_ptr->data.data != SUCCESS)
    {
        recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/createServer"));
        return;
    }

    rs_data_ptr->data.data = modbus_manager_ptr_->openModbus(fst_hal::SERVER);
    recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/createServer"));
}

//"/rpc/modbus/deleteServer", 
void ControllerRpc::handleRpc0x00006C22(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    modbus_manager_ptr_->closeModbus();
    rs_data_ptr->data.data = SUCCESS;

    recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/createServer"));
}
//"/rpc/modbus/createClient",
void ControllerRpc::handleRpc0x00015F94(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusTcpClient* rq_data_ptr = static_cast<RequestMessageType_ModbusTcpClient*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    ClientInfo client_info;
    client_info.port = rq_data_ptr->data.port;
    client_info.ip = rq_data_ptr->data.ip;
    client_info.response_timeout.tv_sec = rq_data_ptr->data.response_timeout.sec;
    client_info.response_timeout.tv_usec = rq_data_ptr->data.response_timeout.usec;
    client_info.bytes_timeout.tv_sec = rq_data_ptr->data.bytes_timeout.sec;
    client_info.bytes_timeout.tv_usec = rq_data_ptr->data.bytes_timeout.usec;

    rs_data_ptr->data.data = modbus_manager_ptr_->setClientInfo(client_info);
    if (rs_data_ptr->data.data != SUCCESS)
    {
        recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/createServer"));
        return;
    }

    rs_data_ptr->data.data = modbus_manager_ptr_->openModbus(fst_hal::CLIENT);

    recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/createServer"));
}

//"/rpc/modbus/deleteClient", 
void ControllerRpc::handleRpc0x00014CF4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    modbus_manager_ptr_->closeModbus();
    rs_data_ptr->data.data = SUCCESS;

    recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/createServer"));
}

