#include "controller_rpc.h"

using namespace fst_ctrl;

// "/rpc/interpreter/start"
void ControllerRpc::handleRpc0x00006154(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = controller_client_ptr_->start(std::string(rq_data_ptr->data.data));    
}

// "/rpc/interpreter/debug"
void ControllerRpc::handleRpc0x000102D7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = controller_client_ptr_->debug(std::string(rq_data_ptr->data.data)); 
}

// "/rpc/interpreter/forward"
void ControllerRpc::handleRpc0x0000D974(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);
    rs_data_ptr->data.data = controller_client_ptr_->forward(); 
}

// "/rpc/interpreter/backward"
void ControllerRpc::handleRpc0x00008E74(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);
    rs_data_ptr->data.data = controller_client_ptr_->backward(); 
}

// "/rpc/interpreter/jump"
void ControllerRpc::handleRpc0x00015930(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = controller_client_ptr_->jump(rq_data_ptr->data.data); 
}

// "/rpc/interpreter/pause"
void ControllerRpc::handleRpc0x0000BA55(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);
    rs_data_ptr->data.data = controller_client_ptr_->pause(); 
}

// "/rpc/interpreter/resume"
void ControllerRpc::handleRpc0x0000CF55(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);
    rs_data_ptr->data.data = controller_client_ptr_->resume(); 
}

// "/rpc/interpreter/abort"
void ControllerRpc::handleRpc0x000086F4(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);
    rs_data_ptr->data.data = controller_client_ptr_->abort(); 
}

