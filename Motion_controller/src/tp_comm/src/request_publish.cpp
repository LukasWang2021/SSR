#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "error_monitor.h"
#include "error_code.h"
#include "tp_comm.h"

using namespace fst_base;
using namespace fst_comm;

//"/rpc/publish/addTopic",
void TpComm::handleRequest0x000050E3(int recv_bytes)
{   
    // create object for request and response package
    RequestMessageType_Topic* request_data_ptr = new RequestMessageType_Topic;
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
        recordLog(TP_COMM_LOG, TP_COMM_MEMORY_OPERATION_FAILED, "/rpc/publish/deleteTopic");
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        recordLog(TP_COMM_LOG, TP_COMM_MEMORY_OPERATION_FAILED, "/rpc/publish/deleteTopic");
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00004403, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Uint32_fields, -1);
}

//"/rpc/publish/addRegTopic"
void TpComm::handleRequest0x000163A3(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Topic* request_data_ptr = new RequestMessageType_Topic;
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
    
    handleRequestPackage(0x000163A3, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Topic_fields, -1);
}

//"/rpc/publish/addIoTopic"
void TpComm::handleRequest0x000058F3(int recv_bytes)
{  
    // create object for request and response package
    RequestMessageType_Topic* request_data_ptr = new RequestMessageType_Topic;
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

    handleRequestPackage(0x000058F3, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Topic_fields, -1);
}

// delete reg topic
void TpComm::handleRequest0x00010353(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Uint32* request_data_ptr = new RequestMessageType_Uint32;
    if(request_data_ptr == NULL)
    {
        recordLog(TP_COMM_LOG, TP_COMM_MEMORY_OPERATION_FAILED, "/rpc/publish/deleteRegTopic");
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        recordLog(TP_COMM_LOG, TP_COMM_MEMORY_OPERATION_FAILED, "/rpc/publish/deleteRegTopic");
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00010353, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Uint32_fields, -1);
}

// delete io topic 
void TpComm::handleRequest0x0000DD03(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Uint32* request_data_ptr = new RequestMessageType_Uint32;
    if(request_data_ptr == NULL)
    {
        recordLog(TP_COMM_LOG, TP_COMM_MEMORY_OPERATION_FAILED, "/rpc/publish/deleteIoTopic");
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        recordLog(TP_COMM_LOG, TP_COMM_MEMORY_OPERATION_FAILED, "/rpc/publish/deleteIoTopic");
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000DD03, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Uint32_fields, -1);
}

