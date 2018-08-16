#include "controller_rpc.h"

using namespace fst_ctrl;

// "/rpc/io_mapping/getDIByBit"
void ControllerRpc::handleRpc0x000050B4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool_Int32* rs_data_ptr = static_cast<ResponseMessageType_Bool_Int32*>(response_data_ptr);

    rs_data_ptr->success.data = true;
    rs_data_ptr->data.data = rq_data_ptr->data.data % 2;
}

// "/rpc/io_mapping/setDIByBit"
void ControllerRpc::handleRpc0x00011754(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = true;
}

// "/rpc/io_mapping/getDOByBit"
void ControllerRpc::handleRpc0x00013074(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool_Int32* rs_data_ptr = static_cast<ResponseMessageType_Bool_Int32*>(response_data_ptr);

    rs_data_ptr->success.data = true;
    rs_data_ptr->data.data = rq_data_ptr->data.data % 2;
}

// "/rpc/io_mapping/setDOByBit"
void ControllerRpc::handleRpc0x00007074(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = true;
}

