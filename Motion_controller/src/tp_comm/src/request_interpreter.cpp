#include "tp_comm.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

using namespace fst_comm;

// "/rpc/interpreter/start",
void TpComm::handleRequest0x00006154(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_String* request_data_ptr = new RequestMessageType_String;
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
    
    handleRequestPackage(0x00006154, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_String_fields, -1);
}

// "/rpc/interpreter/debug"
void TpComm::handleRequest0x000102D7(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_String* request_data_ptr = new RequestMessageType_String;
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
    
    handleRequestPackage(0x000102D7, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_String_fields, -1);
}

// "/rpc/interpreter/forward"
void TpComm::handleRequest0x0000D974(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
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
    
    handleRequestPackage(0x0000D974, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/interpreter/backward"
void TpComm::handleRequest0x00008E74(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
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
    
    handleRequestPackage(0x00008E74, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

// "/rpc/interpreter/jump"
void TpComm::handleRequest0x00015930(int recv_bytes)
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
    
    handleRequestPackage(0x00015930, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/interpreter/pause"
void TpComm::handleRequest0x0000BA55(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
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
    
    handleRequestPackage(0x0000BA55, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

// "/rpc/interpreter/resume"
void TpComm::handleRequest0x0000CF55(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
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
    
    handleRequestPackage(0x0000CF55, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

// "/rpc/interpreter/abort"
void TpComm::handleRequest0x000086F4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
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
    
    handleRequestPackage(0x000086F4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}
