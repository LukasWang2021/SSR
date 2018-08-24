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

    if(!decodeRequestPackage(RequestMessageType_Uint32_fields, (void*)request_data_ptr, recv_bytes))
    {
        ErrorMonitor::instance()->add(TP_COMM_DECODE_FAILED);
        FST_ERROR("handleRequestPackage:  decode data failed");
        return ;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(0x00004403);

    if(checkAuthority(request_data_ptr->property.authority, controller_authority))
    {
        if (isTopicExisted(request_data_ptr->data.data))
        {
            initResponsePackage(request_data_ptr, response_data_ptr, -1);
            response_data_ptr->data.data = 0;

            eraseTaskFromPublishList(request_data_ptr->data.data);
        }
        else
        {
            ErrorMonitor::instance()->add(TP_COMM_DELETE_TOPIC_FAILED);
            FST_ERROR("Topic dose not exist");

            initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
            response_data_ptr->data.data = TP_COMM_DELETE_TOPIC_FAILED;
        }
    }
    else
    {
        ErrorMonitor::instance()->add(TP_COMM_AUTHORITY_CHECK_FAILED);
        FST_ERROR("Operation is not authorized");

        initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
    }

    TpRequestResponse package;
    package.hash = 0x00004403;
    package.request_data_ptr = request_data_ptr;
    package.response_data_ptr = response_data_ptr;

    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
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

    if(!decodeRequestPackage(RequestMessageType_Uint32_fields, (void*)request_data_ptr, recv_bytes))
    {
        ErrorMonitor::instance()->add(TP_COMM_DECODE_FAILED);
        FST_ERROR("handleRequestPackage:  decode data failed");
        return ;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(0x00010353);

    if(checkAuthority(request_data_ptr->property.authority, controller_authority))
    {
        if (isRegTopicExisted(request_data_ptr->data.data))
        {
            initResponsePackage(request_data_ptr, response_data_ptr, -1);
            response_data_ptr->data.data = 0;

            eraseTaskFromRegPublishList(request_data_ptr->data.data);
        }
        else
        {
            ErrorMonitor::instance()->add(TP_COMM_DELETE_TOPIC_FAILED);
            FST_ERROR("Topic dose not exist");

            initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
            response_data_ptr->data.data = TP_COMM_DELETE_TOPIC_FAILED;
        }
    }
    else
    {
        ErrorMonitor::instance()->add(TP_COMM_AUTHORITY_CHECK_FAILED);
        FST_ERROR("Operation is not authorized");

        initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
    }

    TpRequestResponse package;
    package.hash = 0x00010353;
    package.request_data_ptr = request_data_ptr;
    package.response_data_ptr = response_data_ptr;

    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
}

// delete io topic 
void TpComm::handleRequest0x0000DD03(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Uint32* request_data_ptr = new RequestMessageType_Uint32;
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

    if(!decodeRequestPackage(RequestMessageType_Uint32_fields, (void*)request_data_ptr, recv_bytes))
    {
        ErrorMonitor::instance()->add(TP_COMM_DECODE_FAILED);
        FST_ERROR("handleRequestPackage:  decode data failed");
        return ;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(0x0000DD03);

    if(checkAuthority(request_data_ptr->property.authority, controller_authority))
    {
        if (isIoTopicExisted(request_data_ptr->data.data))
        {
            initResponsePackage(request_data_ptr, response_data_ptr, -1);
            response_data_ptr->data.data = 0;

            eraseTaskFromIoPublishList(request_data_ptr->data.data);
        }
        else
        {
            ErrorMonitor::instance()->add(TP_COMM_DELETE_TOPIC_FAILED);
            FST_ERROR("Topic dose not exist");

            initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
            response_data_ptr->data.data = TP_COMM_DELETE_TOPIC_FAILED;
        }
    }
    else
    {
        ErrorMonitor::instance()->add(TP_COMM_AUTHORITY_CHECK_FAILED);
        FST_ERROR("Operation is not authorized");

        initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
    }

    TpRequestResponse package;
    package.hash = 0x0000DD03;
    package.request_data_ptr = request_data_ptr;
    package.response_data_ptr = response_data_ptr;

    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
}

