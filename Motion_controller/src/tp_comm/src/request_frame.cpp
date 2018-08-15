#include "tp_comm.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

using namespace fst_comm;

// add tool_frame
void TpComm::handleRequest0x0000A22C(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_ToolInfo* request_data_ptr = new RequestMessageType_ToolInfo;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000A22C, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_ToolInfo_fields, -1);
}

// delete tool frame
void TpComm::handleRequest0x00010E4C(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00010E4C, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

// update tool frame
void TpComm::handleRequest0x0000C78C(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_ToolInfo* request_data_ptr = new RequestMessageType_ToolInfo;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000C78C, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_ToolInfo_fields, -1);
}

void TpComm::handleRequest0x000085FC(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000085FC, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

// get tool info by id
void TpComm::handleRequest0x00009E34(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool_ToolInfo* response_data_ptr = new ResponseMessageType_Bool_ToolInfo;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00009E34, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

// getAllValidToolSummaryInfo
void TpComm::handleRequest0x0001104F(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_ToolSummaryList* response_data_ptr = new ResponseMessageType_ToolSummaryList;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0001104F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

// add user frame 
void TpComm::handleRequest0x00016764(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_UserInfo* request_data_ptr = new RequestMessageType_UserInfo;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00016764, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_UserInfo_fields, -1);
}

// delete frame
void TpComm::handleRequest0x0000BAF4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000BAF4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

// update user coodinate
void TpComm::handleRequest0x0000EC14(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_UserInfo* request_data_ptr = new RequestMessageType_UserInfo;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000EC14, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_UserInfo_fields, -1);
}

// moveUserCoord
void TpComm::handleRequest0x0000E104(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32List* request_data_ptr = new RequestMessageType_Int32List;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool* response_data_ptr = new ResponseMessageType_Bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000E104, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32List_fields, -1);
}

// getUserCoordInfoById
void TpComm::handleRequest0x00004324(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Bool_UserInfo* response_data_ptr = new ResponseMessageType_Bool_UserInfo;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00004324, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

// getAllValidUserCoordSummaryInfo
void TpComm::handleRequest0x0001838F(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_UserSummaryList* response_data_ptr = new ResponseMessageType_UserSummaryList;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0001838F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}
