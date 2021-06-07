#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;


/********rpc/servo1001/servo/shutDown, RequestMessageType_Int32List(count=2)**********/ 
void TpComm::handleRequest0x0000863E(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/shutDown: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/shutDown: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000863E, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/switchOn, RequestMessageType_Int32List(count=2)**********/ 
void TpComm::handleRequest0x0000E5CE(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/switchOn: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/switchOn: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000E5CE, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/disableVoltage, RequestMessageType_Int32List(count=2)**********/  
void TpComm::handleRequest0x00004755(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/disableVoltage: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/disableVoltage: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00004755, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/enableOperation, RequestMessageType_Int32List(count=2)**********/ 
void TpComm::handleRequest0x0000313E(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/enableOperation: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/enableOperation: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000313E, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/switchOnAndEnableOperation, RequestMessageType_Int32List(count=2)**********/  
void TpComm::handleRequest0x000177CE(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/switchOnAndEnableOperation: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/switchOnAndEnableOperation: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000177CE, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/disableOperation, RequestMessageType_Int32List(count=2)**********/ 
void TpComm::handleRequest0x000026AE(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/disableOperation: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/disableOperation: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000026AE, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/quickStop, RequestMessageType_Int32List(count=2)**********/   
void TpComm::handleRequest0x00000580(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/quickStop: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/quickStop: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00000580, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/resetFault, RequestMessageType_Int32List(count=2)**********/  
void TpComm::handleRequest0x00010584(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/resetFault: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/resetFault: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00010584, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/transCommState, RequestMessageType_Int32List_CoreCommState(count=2)**********/ 
void TpComm::handleRequest0x000153C5(int recv_bytes)
{
    RequestMessageType_Int32List_CoreCommState* request_data_ptr = new RequestMessageType_Int32List_CoreCommState;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/transCommState: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/transCommState: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000153C5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_CoreCommState_fields, -1);
}

/********rpc/servo1001/servo/readParameter, RequestMessageType_Int32List(count=3)**********/    
void TpComm::handleRequest0x00006892(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/readParameter: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32* response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/readParameter: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00006892, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/writeParameter, RequestMessageType_Int32List(count=4)**********/  
void TpComm::handleRequest0x00007C32(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/writeParameter: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/writeParameter: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00007C32, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/moveVelocity, RequestMessageType_Int32List(count=7)**********/ 
void TpComm::handleRequest0x000164D9(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/moveVelocity: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/moveVelocity: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000164D9, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/moveAbsolute, RequestMessageType_Int32List(count=7)**********/ 
void TpComm::handleRequest0x00004DD5(int recv_bytes)
{
    RequestMessageType_Int32List_Int64* request_data_ptr = new RequestMessageType_Int32List_Int64;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/moveAbsolute: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/moveAbsolute: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00004DD5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_Int64_fields, -1);
}

/********rpc/servo1001/servo/triggerUploadParameters, RequestMessageType_Int32List(count=2)**********/ 
void TpComm::handleRequest0x000020B3(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/triggerUploadParameter: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/triggerUploadParameter: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000020B3, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/uploadParameters, RequestMessageType_Int32List(count=2)**********/ 
void TpComm::handleRequest0x0000E003(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/uploadParameters: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32List* response_data_ptr = new ResponseMessageType_Uint64_Int32List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/uploadParameters: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000E003, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/triggerDownloadParameters, RequestMessageType_Int32List(count=2)**********/  
void TpComm::handleRequest0x00011C53(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/triggerDownloadParameters: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/triggerDownloadParameters: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00011C53, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/downloadParameters, RequestMessageType_Int32List_Int32List(count=2,count=512)**********/  
void TpComm::handleRequest0x00017063(int recv_bytes)
{
    RequestMessageType_Int32List_Int32List* request_data_ptr = new RequestMessageType_Int32List_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/downloadParameters: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/downloadParameters: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00017063, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_Int32List_fields, -1);
}

/********rpc/servo1001/servo/isAsyncServiceFinish, RequestMessageType_Int32List(count=2)**********/ 
void TpComm::handleRequest0x000043B8(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/isAsyncServiceFinish: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Bool* response_data_ptr = new ResponseMessageType_Uint64_Bool;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/isAsyncServiceFinish: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000043B8, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/getCommState, RequestMessageType_Int32List(count=2)**********/ 
void TpComm::handleRequest0x0000F485(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/getCommState: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_CoreCommState* response_data_ptr = new ResponseMessageType_Uint64_CoreCommState;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/getCommState: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000F485, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/getServoState, RequestMessageType_Int32List(count=2)**********/ 
void TpComm::handleRequest0x000032F5(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/getServoState: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32* response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/getServoState: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000032F5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);

}  
/********rpc/servo1001/servo/moveRelative, RequestMessageType_Int32_DoubleList(count=5)**********/	
void TpComm::handleRequest0x000172C5(int recv_bytes)
{
    RequestMessageType_Int32List_Int64* request_data_ptr = new RequestMessageType_Int32List_Int64;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/moveRelative: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/moveRelative: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000172C5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_Int64_fields, -1);
}

/********rpc/servo1001/servo/resetEncoder, RequestMessageType_Int32List(count=2)**********/	
void TpComm::handleRequest0x0000EFE2(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/resetEncoder: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/resetEncoder: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000EFE2, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}


/********rpc/servo1001/servo/goHome, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x00013BB5(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/goHome: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/goHome: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00013BB5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/abortHoming, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x00015AB7(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/abortHoming: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/abortHoming: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00015AB7, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/servo/getServoDefinedInfo, RequestMessageType_Int32_Int32List(count=9)**********/	
void TpComm::handleRequest0x0000C87F(int recv_bytes)
{
    RequestMessageType_Int32_Int32List* request_data_ptr = new RequestMessageType_Int32_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/getServoDefinedInfo: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32List* response_data_ptr = new ResponseMessageType_Uint64_Int32List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/aborgetServoDefinedInfotHoming: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000C87F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_Int32List_fields, -1);
}

/********rpc/servo1001/cpu/getVersion, RequestMessageType_Int32**********/  
void TpComm::handleRequest0x0001192E(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getVersion: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Uint32List* response_data_ptr = new ResponseMessageType_Uint64_Uint32List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getVersion: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0001192E, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/servo1001/cpu/setCtrlPdoSync, RequestMessageType_Int32List_Uint32**********/   
void TpComm::handleRequest0x00005123(int recv_bytes)
{
    RequestMessageType_Int32List_Uint32* request_data_ptr = new RequestMessageType_Int32List_Uint32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/setCtrlPdoSync: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/setCtrlPdoSync: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00005123, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_Uint32_fields, -1);
}

/********rpc/servo1001/cpu/getCtrlPdoSync, RequestMessageType_Int32List**********/  
void TpComm::handleRequest0x00005463(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getCtrlPdoSync: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32_Uint32* response_data_ptr = new ResponseMessageType_Uint64_Int32_Uint32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getCtrlPdoSync: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00005463, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/cpu/setSamplingSync, RequestMessageType_Int32_Uint32**********/ 
void TpComm::handleRequest0x00004023(int recv_bytes)
{
    RequestMessageType_Int32_Uint32* request_data_ptr = new RequestMessageType_Int32_Uint32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/setSamplingSync: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/setSamplingSync: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00004023, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_Uint32_fields, -1);
}

/********rpc/servo1001/cpu/getSamplingSync, RequestMessageType_Int32**********/ 
void TpComm::handleRequest0x00006C23(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getSamplingSync: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Uint32* response_data_ptr = new ResponseMessageType_Uint64_Uint32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getSamplingSync: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00006C23, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/servo1001/cpu/setSamplingInterval, RequestMessageType_Int32_Uint32**********/ 
void TpComm::handleRequest0x00003EEC(int recv_bytes)
{
    RequestMessageType_Int32_Uint32* request_data_ptr = new RequestMessageType_Int32_Uint32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/setSamplingInterval: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/setSamplingInterval: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00003EEC, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_Uint32_fields, -1);
}

/********rpc/servo1001/cpu/getSamplingInterval, RequestMessageType_Int32**********/ 
void TpComm::handleRequest0x00001C2C(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getSamplingInterval: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Uint32* response_data_ptr = new ResponseMessageType_Uint64_Uint32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getSamplingInterval: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00001C2C, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/servo1001/cpu/setSamplingMaxTimes, RequestMessageType_Int32_Uint32**********/  
void TpComm::handleRequest0x000110A3(int recv_bytes)
{
    RequestMessageType_Int32_Uint32* request_data_ptr = new RequestMessageType_Int32_Uint32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/setSamplingMaxTimes: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/setSamplingMaxTimes: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x000110A3, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_Uint32_fields, -1);
}

/********rpc/servo1001/cpu/getSamplingMaxTimes, RequestMessageType_Int32**********/ 
void TpComm::handleRequest0x00013363(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getSamplingMaxTimes: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Uint32* response_data_ptr = new ResponseMessageType_Uint64_Uint32;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getSamplingMaxTimes: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00013363, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/servo1001/cpu/setSamplingChannel, RequestMessageType_Int32_Uint32List(count=3)**********/  
void TpComm::handleRequest0x00008E5C(int recv_bytes)
{
    RequestMessageType_Int32_Uint32List* request_data_ptr = new RequestMessageType_Int32_Uint32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/setSamplingChannel: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/setSamplingChannel: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00008E5C, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_Uint32List_fields, -1);
}

/********rpc/servo1001/cpu/getSamplingChannel, RequestMessageType_Int32**********/ 
void TpComm::handleRequest0x0000FD9C(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getSamplingChannel: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Uint32List* response_data_ptr = new ResponseMessageType_Uint64_Uint32List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getSamplingChannel: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000FD9C, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/servo1001/cpu/activateSamplingConfiguration, RequestMessageType_Int32**********/
void TpComm::handleRequest0x0000939E(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/activateSamplingConfiguration: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/activateSamplingConfiguration: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000939E, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}


/********rpc/servo1001/cpu/saveSamplingBufferData, RequestMessageType_Int32_String**********/  
void TpComm::handleRequest0x00015621(int recv_bytes)
{
    RequestMessageType_Int32_String* request_data_ptr = new RequestMessageType_Int32_String;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/saveSamplingBufferData: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/saveSamplingBufferData: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x00015621, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_String_fields, -1);
}

/********rpc/servo1001/servo/getServoCommInfo, RequestMessageType_Int32List(count=2)**********/
void TpComm::handleRequest0x0000BF1F(int recv_bytes)
{
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/getServoCommInfo: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32List* response_data_ptr = new ResponseMessageType_Uint64_Int32List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/servo/getServoCommInfo: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000BF1F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/servo1001/cpu/getServoCpuCommInfo, RequestMessageType_Int32**********/ 
void TpComm::handleRequest0x0000FE5F(int recv_bytes)
{
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getServoCpuCommInfo: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32List* response_data_ptr = new ResponseMessageType_Uint64_Int32List;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/servo1001/cpu/getServoCpuCommInfo: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
	
    handleRequestPackage(0x0000FE5F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

