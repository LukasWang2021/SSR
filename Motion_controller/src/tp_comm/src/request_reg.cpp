#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;


/********rpc/reg_manager/pr/addReg, RequestMessageType_PrRegData**********/	
void TpComm::handleRequest0x000154E7(int recv_bytes)
{
    RequestMessageType_PrRegData* request_data_ptr = new RequestMessageType_PrRegData;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000154E7, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_PrRegData_fields, -1);
}
/********rpc/reg_manager/pr/deleteReg, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x00001097(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00001097, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}
/********rpc/reg_manager/pr/updateReg, RequestMessageType_PrRegData**********/	
void TpComm::handleRequest0x00009EF7(int recv_bytes)
{
    RequestMessageType_PrRegData* request_data_ptr = new RequestMessageType_PrRegData;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00009EF7, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_PrRegData_fields, -1);
}
/********rpc/reg_manager/pr/getReg, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x00017207(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64_PrRegData* response_data_ptr = new ResponseMessageType_Uint64_PrRegData;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00017207, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}
/********rpc/reg_manager/pr/moveReg, RequestMessageType_Int32List(count = 2) **********/	
void TpComm::handleRequest0x0000D7C7(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000D7C7, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}
/********rpc/reg_manager/pr/getChangedList, RequestMessageType_Int32List(count = 2) **********/	
void TpComm::handleRequest0x0000B454(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64_BaseRegSummaryList* response_data_ptr = new ResponseMessageType_Uint64_BaseRegSummaryList;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000B454, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}
/********rpc/reg_manager/pr/getValidList, RequestMessageType_Int32List(count = 2) **********/	
void TpComm::handleRequest0x00009354(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data\n");
        return;
    }
    ResponseMessageType_Uint64_BaseRegSummaryList* response_data_ptr = new ResponseMessageType_Uint64_BaseRegSummaryList;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data\n");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00009354, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

    

