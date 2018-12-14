#include "controller_rpc.h"

using namespace fst_ctrl;

//"/rpc/file_manager/readFile"
void ControllerRpc::handleRpc0x0000A545(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64_Bytes* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Bytes*>(response_data_ptr);

    long length = 0;
    uint8_t* ptr = NULL;
    rs_data_ptr->error_code.data = file_manager_ptr_->readFileStream(ptr, length, rq_data_ptr->data.data);

    memcpy(&rs_data_ptr->data.data, ptr, length);

    FST_INFO("rpc-readFile: %s, ret = 0x%llx\n", &rq_data_ptr->data.data, rs_data_ptr->error_code.data);
}

//"/rpc/file_manager/writeFile"
void ControllerRpc::handleRpc0x00010D95(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String_Bytes* rq_data_ptr = static_cast<RequestMessageType_String_Bytes*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    //long length = sizeof(rq_data_ptr->data2.data);
    //uint8_t* ptr = new uint8_t[length];
    //memcpy(ptr, &rq_data_ptr->data2.data, length);
    //delete ptr;

    long length = sizeof(rq_data_ptr->data2.data); // data2 is the byte stream to be written
    rs_data_ptr->data.data = file_manager_ptr_->writeFileStream(&rq_data_ptr->data2.data, length, rq_data_ptr->data1.data);//data1 is file path.
    
    FST_INFO("rpc-writeFile: %s, ret = 0x%llx\n", &rq_data_ptr->data1.data, rs_data_ptr->data.data);
}

