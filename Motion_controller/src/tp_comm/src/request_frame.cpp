#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>
#include "common_error_code.h"
#include "tp_comm.h"

using namespace base_space;
using namespace user_space;
using namespace log_space;

/********rpc/tool_manager/addTool, RequestMessageType_ToolInfo**********/
void TpComm::handleRequest0x0000A22C(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_ToolInfo* request_data_ptr = new RequestMessageType_ToolInfo;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/addTool: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/addTool: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000A22C, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_ToolInfo_fields, -1);
}

/********rpc/tool_manager/deleteTool, RequestMessageType_Int32**********/
void TpComm::handleRequest0x00010E4C(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/deleteTool: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/deleteTool: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00010E4C, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/tool_manager/updateTool, RequestMessageType_ToolInfo**********/
void TpComm::handleRequest0x0000C78C(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_ToolInfo* request_data_ptr = new RequestMessageType_ToolInfo;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/updateTool: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/updateTool: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000C78C, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_ToolInfo_fields, -1);
}

/********rpc/tool_manager/moveTool, RequestMessageType_Int32List(count = 2) **********/	
void TpComm::handleRequest0x000085FC(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/moveTool: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/moveTool: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000085FC, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/tool_manager/getToolInfoById, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x00009E34(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/getToolInfoById: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ToolInfo* response_data_ptr = new ResponseMessageType_Uint64_ToolInfo;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/getToolInfoById: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00009E34, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/tool_manager/getAllValidToolSummaryInfo, RequestMessageType_Void**********/
void TpComm::handleRequest0x0001104F(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/getAllValidToolSummaryInfo: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ToolSummaryList* response_data_ptr = new ResponseMessageType_Uint64_ToolSummaryList;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/tool_manager/getAllValidToolSummaryInfo: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0001104F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}


/********rpc/coordinate_manager/addUserCoord, RequestMessageType_UserCoordInfo**********/
void TpComm::handleRequest0x00016764(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_UserCoordInfo* request_data_ptr = new RequestMessageType_UserCoordInfo;
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

    handleRequestPackage(0x00016764, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_UserCoordInfo_fields, -1);
}

/********rpc/coordinate_manager/deleteUserCoord, RequestMessageType_Int32**********/
void TpComm::handleRequest0x0000BAF4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/coordinate_manager/deleteUserCoord: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/coordinate_manager/deleteUserCoord: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000BAF4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/coordinate_manager/updateUserCoord, RequestMessageType_UserCoordInfo**********/
void TpComm::handleRequest0x0000EC14(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_UserCoordInfo* request_data_ptr = new RequestMessageType_UserCoordInfo;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/coordinate_manager/updateUserCoord: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/coordinate_manager/updateUserCoord: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000EC14, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_UserCoordInfo_fields, -1);
}

/********rpc/coordinate_manager/moveUserCoord, RequestMessageType_Int32List(count = 2) **********/
void TpComm::handleRequest0x0000E104(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/coordinate_manager/moveUserCoord: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/coordinate_manager/moveUserCoord: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000E104, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

/********rpc/coordinate_manager/getUserCoordInfoById, RequestMessageType_Int32**********/	
void TpComm::handleRequest0x00004324(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/coordinate_manager/getUserCoordInfoById: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_UserCoordInfo* response_data_ptr = new ResponseMessageType_Uint64_UserCoordInfo;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/coordinate_manager/getUserCoordInfoById: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00004324, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

/********rpc/coordinate_manager/getAllValidUserCoordSummaryInfo, RequestMessageType_Void**********/	
void TpComm::handleRequest0x0001838F(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/coordinate_manager/getAllValidUserCoordSummaryInfo: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_UserCoordSummaryList* response_data_ptr = new ResponseMessageType_Uint64_UserCoordSummaryList;
    if(response_data_ptr == NULL)
    {
        ErrorQueue::instance().push(TP_COMM_MEMORY_OPERATION_FAILED);
        LogProducer::error("comm", "rpc/coordinate_manager/getAllValidUserCoordSummaryInfo: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0001838F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}
