#include "controller_rpc.h"

using namespace fst_ctrl;

// "/rpc/io_mapping/getDIByBit"
void ControllerRpc::handleRpc0x000050B4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = rq_data_ptr->data.data % 2;
    recordLog(IO_MAPPING_LOG, rs_data_ptr->error_code.data, std::string("/rpc/io_mapping/getDIByBit"));
}

// "/rpc/io_mapping/setDIByBit"
void ControllerRpc::handleRpc0x00011754(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
    recordLog(IO_MAPPING_LOG, rs_data_ptr->data.data, std::string("/rpc/io_mapping/setDIByBit"));
}

// "/rpc/io_mapping/getDOByBit"
void ControllerRpc::handleRpc0x00013074(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = rq_data_ptr->data.data % 2;
    recordLog(IO_MAPPING_LOG, rs_data_ptr->error_code.data, std::string("/rpc/io_mapping/getDOByBit"));
}

// "/rpc/io_mapping/setDOByBit"
void ControllerRpc::handleRpc0x00007074(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
    recordLog(IO_MAPPING_LOG, rs_data_ptr->data.data, std::string("/rpc/io_mapping/setDOByBit"));
}

