#include "controller_rpc.h"

using namespace fst_ctrl;

// "/rpc/program_launching/setMethod"
void ControllerRpc::handleRpc0x00011544(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = SUCCESS;//should be function.

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->data.data, std::string("/rpc/program_launching/setMethod"));
}

// "/rpc/program_launching/getMethod"
void ControllerRpc::handleRpc0x00010944(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code.data = SUCCESS;//should be function.

    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
    else
    {
        memset(&rs_data_ptr->data, 0, sizeof(MessageType_Int32));
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->error_code.data, std::string("/rpc/program_launching/getMethod"));
}

// "/rpc/program_launching/syncFileMacroConfig"
void ControllerRpc::handleRpc0x00016B27(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = SUCCESS;//should be function.

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(IO_MAPPING_LOG, rs_data_ptr->data.data, std::string("/rpc/program_launching/syncFileMacroConfig"));
}
