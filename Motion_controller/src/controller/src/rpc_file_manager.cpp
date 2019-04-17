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

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        memcpy(&rs_data_ptr->data.data.bytes, ptr, length);
        rs_data_ptr->data.data.size = length;
        //printf("rs_data, size=%d, bytes = %s\n",rs_data_ptr->data.data.size, rs_data_ptr->data.data.bytes);
    }
    else
    {
        rs_data_ptr->data.data.size = 0;
    }
    
    //if (rs_data_ptr->error_code.data != SUCCESS)
        FST_INFO("rpc-readFile: %s, ret = 0x%llx\n", &rq_data_ptr->data.data, rs_data_ptr->error_code.data);
}

//"/rpc/file_manager/writeFile"
void ControllerRpc::handleRpc0x00010D95(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String_Bytes* rq_data_ptr = static_cast<RequestMessageType_String_Bytes*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    long length = rq_data_ptr->data2.data.size;// data2 is the byte stream to be written
    if (length <= 65535)
    {
        rs_data_ptr->data.data = file_manager_ptr_->writeFileStream(&rq_data_ptr->data2.data.bytes, length, rq_data_ptr->data1.data);//data1 is file path.
    }
    else
    {
        rs_data_ptr->data.data = FILE_MANAGER_WRITE_FILE_FAILED;
    }
   
    //if (rs_data_ptr->data.data != SUCCESS)
        FST_INFO("rpc-writeFile: %s, ret = 0x%llx\n", &rq_data_ptr->data1.data, rs_data_ptr->data.data);
}

