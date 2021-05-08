#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;


/********rpc/axis/mcPower, RequestMessageType_Int32_Bool**********/ 
void TpComm::handleRequest0x000053E2(int recv_bytes)
{
    RequestMessageType_Int32_Bool* request_data_ptr = new RequestMessageType_Int32_Bool;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000053E2, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_Bool_fields, -1);
}

/********rpc/axis/mcReset, RequestMessageType_Int32**********/  
void TpComm::handleRequest0x000180C4(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000180C4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/mcStop, RequestMessageType_Int32**********/   
void TpComm::handleRequest0x00002820(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00002820, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/mcHalt, RequestMessageType_Int32**********/   
void TpComm::handleRequest0x00004BB4(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00004BB4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/mcSetPosition, RequestMessageType_Int32_Double**********/ 
void TpComm::handleRequest0x0001798E(int recv_bytes)
{
    RequestMessageType_Int32_Double* request_data_ptr = new RequestMessageType_Int32_Double;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0001798E, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_Double_fields, -1);
}

/********rpc/axis/mcReadParameter, RequestMessageType_Int32List(count=2)**********/ 
void TpComm::handleRequest0x00016BF2(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32* response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00016BF2, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/axis/mcWriteParameter, RequestMessageType_Int32List(count=3)**********/    
void TpComm::handleRequest0x00005732(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00005732, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/axis/mcMoveAbsolute, RequestMessageType_Int32_DoubleList(count=5)**********/   
void TpComm::handleRequest0x000051F5(int recv_bytes)
{
    RequestMessageType_Int32_DoubleList* request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000051F5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_DoubleList_fields, -1);
}

/********rpc/axis/mcMoveVelocity, RequestMessageType_Int32List_DoubleList(count=2,count=4)**********/   
void TpComm::handleRequest0x00016CF9(int recv_bytes)
{
    RequestMessageType_Int32List_DoubleList* request_data_ptr = new RequestMessageType_Int32List_DoubleList;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00016CF9, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_DoubleList_fields, -1);
}

/********rpc/axis/mcReadActualPosition, RequestMessageType_Int32**********/ 
void TpComm::handleRequest0x000012BE(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double* response_data_ptr = new ResponseMessageType_Uint64_Double;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000012BE, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/mcReadActualVelocity, RequestMessageType_Int32**********/ 
void TpComm::handleRequest0x00002EA9(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double* response_data_ptr = new ResponseMessageType_Uint64_Double;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00002EA9, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/mcReadActualTorque, RequestMessageType_Int32**********/   
void TpComm::handleRequest0x00014265(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Double* response_data_ptr = new ResponseMessageType_Uint64_Double;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00014265, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/mcReadAxisInfo, RequestMessageType_Int32**********/   
void TpComm::handleRequest0x0000314F(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_AxisInfo* response_data_ptr = new ResponseMessageType_Uint64_AxisInfo;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000314F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/mcReadStatus, RequestMessageType_Int32**********/ 
void TpComm::handleRequest0x00003E53(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_AxisStatus* response_data_ptr = new ResponseMessageType_Uint64_AxisStatus;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00003E53, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/mcReadAxisError, RequestMessageType_Int32**********/  
void TpComm::handleRequest0x000063C2(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Uint64* response_data_ptr = new ResponseMessageType_Uint64_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000063C2, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/mcReadAxisErrorHistory, RequestMessageType_Int32**********/ 
void TpComm::handleRequest0x00018469(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Uint64List* response_data_ptr = new ResponseMessageType_Uint64_Uint64List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00018469, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/mcMoveRelative, RequestMessageType_Int32_DoubleList(count=5)**********/	
void TpComm::handleRequest0x0000CC85(int recv_bytes)
{
    RequestMessageType_Int32_DoubleList* request_data_ptr = new RequestMessageType_Int32_DoubleList;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000CC85, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_DoubleList_fields, -1);
}

/********rpc/axis/mcHome, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x000059B5(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000059B5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/rtmAbortHoming, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x0000E4B7(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000E4B7, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/rtmReadAxisFdbPdoPtr, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x0000A632(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32List* response_data_ptr = new ResponseMessageType_Uint64_Int32List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000A632, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/axis/rtmResetEncoder, RequestMessageType_Int32**********/
void TpComm::handleRequest0x00000BA2(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00000BA2, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}
