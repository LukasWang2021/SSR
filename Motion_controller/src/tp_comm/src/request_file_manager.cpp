#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;


// "/rpc/file_manager/readFile"
void TpComm::handleRequest0x0000A545(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_String* request_data_ptr = new RequestMessageType_String;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "/rpc/file_manager/readFile: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Bytes* response_data_ptr = new ResponseMessageType_Uint64_Bytes;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "/rpc/file_manager/readFile: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000A545, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_String_fields, -1);
}

//"/rpc/file_manager/writeFile"
void TpComm::handleRequest0x00010D95(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_String_Bytes* request_data_ptr = new RequestMessageType_String_Bytes;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "/rpc/file_manager/writeFile: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "/rpc/file_manager/writeFile: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00010D95, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_String_Bytes_fields, -1);
}
