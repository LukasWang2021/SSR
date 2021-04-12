#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;

/********rpc/io/readDI, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x000185A9(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64_Int32* response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000185A9, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/io/readDO, RequestMessageType_Int32**********/
void TpComm::handleRequest0x000185AF(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64_Int32* response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000185AF, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}


/********rpc/io/writeDO, RequestMessageType_Int32List(count=2)**********/	
void TpComm::handleRequest0x00000C1F(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
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
	
    handleRequestPackage(0x00000C1F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/io/readAI, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x00018679(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64_Int32* response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00018679, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/io/readAO, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x0001867F(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64_Int32* response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0001867F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/io/writeAO, RequestMessageType_Int32List(count=2)**********/	
void TpComm::handleRequest0x00000C4F(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
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
	
    handleRequestPackage(0x00000C4F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

