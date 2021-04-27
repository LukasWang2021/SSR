#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;

//"/rpc/controller/getVersion"
void TpComm::handleRequest0x000093EE(int recv_bytes)
{
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64_Uint32List* response_data_ptr = new ResponseMessageType_Uint64_Uint32List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000093EE, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/controller/setSystemTime"
void TpComm::handleRequest0x000167C5(int recv_bytes)
{
    RequestMessageType_Uint64* request_data_ptr = new RequestMessageType_Uint64;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000167C5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Uint64_fields, -1);
}

//"/rpc/controller/getSystemTime"
void TpComm::handleRequest0x000003F5(int recv_bytes)
{
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64_Uint64* response_data_ptr = new ResponseMessageType_Uint64_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000003F5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/controller/setWorkMode"
void TpComm::handleRequest0x00006825(int recv_bytes)
{
    RequestMessageType_Uint32* request_data_ptr = new RequestMessageType_Uint32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00006825, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Uint32_fields, -1);
}

//"/rpc/controller/getWorkMode"
void TpComm::handleRequest0x00003325(int recv_bytes)
{
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64_Uint32* response_data_ptr = new ResponseMessageType_Uint64_Uint32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00003325, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}



