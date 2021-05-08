#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;


//"/rpc/publish/addTopic",
void TpComm::handleRequest0x000050E3(int recv_bytes)
{   
    // create object for request and response package
    RequestMessageType_Topic* request_data_ptr = new RequestMessageType_Topic;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "/rpc/publish/addTopic: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "/rpc/publish/addTopic: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000050E3, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Topic_fields, -1);
}

//"/rpc/publish/deleteTopic"
void TpComm::handleRequest0x00004403(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Uint32* request_data_ptr = new RequestMessageType_Uint32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "/rpc/publish/deleteTopic: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "/rpc/publish/deleteTopic: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00004403, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Uint32_fields, -1);
}


