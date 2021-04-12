#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;


/********rpc/servo_sampling/setSamplingConfiguration, RequestMessageType_Int32_Uint32List(count=2)**********/   
void TpComm::handleRequest0x0000845E(int recv_bytes)
{
    RequestMessageType_Int32_Uint32List* request_data_ptr = new RequestMessageType_Int32_Uint32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/setSamplingConfiguration: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/setSamplingConfiguration: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000845E, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_Uint32List_fields, -1);
}

/********rpc/servo_sampling/getSamplingConfiguration, RequestMessageType_Int32**********/   
void TpComm::handleRequest0x000106EE(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/getSamplingConfiguration: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Uint32List* response_data_ptr = new ResponseMessageType_Uint64_Uint32List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/getSamplingConfiguration: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000106EE, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/servo_sampling/activateSamplingConfiguration, RequestMessageType_Int32**********/ 
void TpComm::handleRequest0x0000CDDE(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/activateSamplingConfiguration: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/activateSamplingConfiguration: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000CDDE, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/servo_sampling/setSamplingSync, RequestMessageType_Int32_Uint32**********/ 
void TpComm::handleRequest0x00003743(int recv_bytes)
{
    RequestMessageType_Int32_Uint32* request_data_ptr = new RequestMessageType_Int32_Uint32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/setSamplingSync: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/setSamplingSync: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00003743, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_Uint32_fields, -1);
}

/********rpc/servo_sampling/getSamplingSync, RequestMessageType_Int32**********/   
void TpComm::handleRequest0x00006343(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/getSamplingSync: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Uint32* response_data_ptr = new ResponseMessageType_Uint64_Uint32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/getSamplingSync: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00006343, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/servo_sampling/setSamplingChannel, RequestMessageType_Int32List(count=4)**********/    
void TpComm::handleRequest0x0000BACC(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/setSamplingChannel: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/setSamplingChannel: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000BACC, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo_sampling/getSamplingChannel, RequestMessageType_Int32**********/ 
void TpComm::handleRequest0x0000556C(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/getSamplingChannel: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Uint32List* response_data_ptr = new ResponseMessageType_Uint64_Uint32List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/getSamplingChannel: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000556C, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}
/********rpc/servo_sampling/saveSamplingBufferData, RequestMessageType_Int32_String**********/  
void TpComm::handleRequest0x00004E41(int recv_bytes)
{
    RequestMessageType_Int32_String* request_data_ptr = new RequestMessageType_Int32_String;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/saveSamplingBufferData: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("rpc", "rpc/servo_sampling/saveSamplingBufferData: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    handleRequestPackage(0x00004E41, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_String_fields, -1);
}
    


