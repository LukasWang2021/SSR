#include "tp_comm.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

using namespace fst_comm;



//"/rpc/publish/addTopic",
void TpComm::handleRequest0x000050E3(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Topic* request_data_ptr = new RequestMessageType_Topic;
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
    
    handleRequestPackage(0x000050E3, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Topic_fields, -1);
}

//"/rpc/publish/deleteTopic"
void TpComm::handleRequest0x00004403(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_UnsignedInt32* request_data_ptr = new RequestMessageType_UnsignedInt32;
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
    
    handleRequestPackage(0x00004403, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_UnsignedInt32_fields, -1);
}

//"/rpc/publish/addRegTopic"
void TpComm::handleRequest0x000163A3(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Topic* request_data_ptr = new RequestMessageType_Topic;
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
    
    handleRequestPackage(0x000058F3, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Topic_fields, -1);
}

