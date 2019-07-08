#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>
#include "error_monitor.h"
#include "error_code.h"
#include "tp_comm.h"
#include "protoc.h"

using namespace fst_base;
using namespace fst_comm;

//"/rpc/motion_control/axis_group/getPayload"
void TpComm::handleRequest0x000180B4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32* response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000180B4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}


//"/rpc/motion_control/axis_group/setPayload"
void TpComm::handleRequest0x00014094(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00014094, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/deletePayload"
void TpComm::handleRequest0x00014F84(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00014F84, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}
//"/rpc/motion_control/axis_group/updatePayload"
void TpComm::handleRequest0x00017074(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_PayloadInfo* request_data_ptr = new RequestMessageType_PayloadInfo;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00017074, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_PayloadInfo_fields, -1);
}

//"/rpc/motion_control/axis_group/movePayload"
void TpComm::handleRequest0x00006CE4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00006CE4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

//"/rpc/motion_control/axis_group/getPayloadInfoById"
void TpComm::handleRequest0x00010C34(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_PayloadInfo* response_data_ptr = new ResponseMessageType_Uint64_PayloadInfo;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00010C34, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/motion_control/axis_group/getAllValidPayloadSummaryInfo"
void TpComm::handleRequest0x00010C8F(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_PayloadSummaryList* response_data_ptr = new ResponseMessageType_Uint64_PayloadSummaryList;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00010C8F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/motion_control/axis_group/addPayload"
void TpComm::handleRequest0x000178A4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_PayloadInfo *request_data_ptr = new RequestMessageType_PayloadInfo;

    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000178A4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_PayloadInfo_fields, -1);
}