#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;

/********rpc/interpreter/start, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x00006154(int recv_bytes)
{
    RequestMessageType_String* request_data_ptr = new RequestMessageType_String;
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
	
    handleRequestPackage(0x00006154, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_String_fields, -1);
}

/********rpc/interpreter/pause, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x0000BA55(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
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
	
    handleRequestPackage(0x0000BA55, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/interpreter/resume, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x0000CF55(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
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
	
    handleRequestPackage(0x0000CF55, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/interpreter/abort, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x000086F4(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
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
	
    handleRequestPackage(0x000086F4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/interpreter/forward, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x0000D974(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
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
	
    handleRequestPackage(0x0000D974, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/interpreter/backward, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x00008E74(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
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
	
    handleRequestPackage(0x00008E74, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/interpreter/jump, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x00015930(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
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
	
    handleRequestPackage(0x00015930, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}
