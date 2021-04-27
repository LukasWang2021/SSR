#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;


/********rpc/group/mcGroupReset, RequestMessageType_Int32**********/    
void TpComm::handleRequest0x00016FF4(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/group/mcGroupReset: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/group/mcGroupReset: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00016FF4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/group/mcGroupEnable, RequestMessageType_Int32**********/   
void TpComm::handleRequest0x00003615(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/group/mcGroupEnable: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/group/mcGroupEnable: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00003615, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/group/mcGroupDisable, RequestMessageType_Int32**********/  
void TpComm::handleRequest0x0000D185(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/group/mcGroupDisable: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/group/mcGroupDisable: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000D185, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}


/********rpc/group/mcGroupReadError, RequestMessageType_Int32**********/   
void TpComm::handleRequest0x00004BE2(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/group/mcGroupReadError: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Uint64* response_data_ptr = new ResponseMessageType_Uint64_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/group/mcGroupReadError: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00004BE2, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/group/mcGroupReadStatus, RequestMessageType_Int32**********/   
void TpComm::handleRequest0x00002A83(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/group/mcGroupReadStatus: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_GroupStatus_Bool* response_data_ptr = new ResponseMessageType_Uint64_GroupStatus_Bool;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/group/mcGroupReadStatus: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00002A83, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}


