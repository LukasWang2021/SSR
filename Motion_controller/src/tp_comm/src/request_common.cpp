#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>
#include "error_monitor.h"
#include "error_code.h"
#include "tp_comm.h"

using namespace fst_base;
using namespace fst_comm;
using namespace std;

// get rpc table
void TpComm::handleRequest0x00004FA5(int recv_bytes)
{
   // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;

    if(request_data_ptr == NULL)
    {
        recordLog(TP_COMM_LOG, TP_COMM_MEMORY_OPERATION_FAILED, "/rpc/tp_comm/getRpcTable");
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }

    ResponseMessageType_Uint64_RpcTable* response_data_ptr = new ResponseMessageType_Uint64_RpcTable;
    if(response_data_ptr == NULL)
    {
        recordLog(TP_COMM_LOG, TP_COMM_MEMORY_OPERATION_FAILED, "/rpc/tp_comm/getRpcTable");
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    if(!decodeRequestPackage(RequestMessageType_Void_fields, (void*)request_data_ptr, recv_bytes))
    {
        recordLog(TP_COMM_LOG, TP_COMM_DECODE_FAILED, "/rpc/tp_comm/getRpcTable");
        FST_ERROR("Decode data failed");
        return ;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(0x00004FA5);
    if(checkAuthority(request_data_ptr->property.authority, controller_authority))
    {
        initResponsePackage(request_data_ptr, response_data_ptr, -1);    
        response_data_ptr->error_code.data = SUCCESS;
        recordLog(TP_COMM_LOG, SUCCESS, "/rpc/tp_comm/getRpcTable");

        int element_index = 0;
        for (std::vector<RpcService>::iterator iter = rpc_table_.begin(); iter != rpc_table_.end(); iter++)
        {
            Comm_Authority rpc_element_authority = getRpcTableElementAuthorityByHash(iter->hash);

            if (checkAuthority(request_data_ptr->property.authority, rpc_element_authority))
            {
                iter->path.copy(response_data_ptr->data.element[element_index].path, iter->path.length(), 0);
                response_data_ptr->data.element[element_index].path[iter->path.length()] = '\0';

                iter->request_type.copy(response_data_ptr->data.element[element_index].request_message_type,
                    iter->request_type.length(), 0);
                response_data_ptr->data.element[element_index].request_message_type[iter->request_type.length()] = '\0';

                iter->response_type.copy(response_data_ptr->data.element[element_index].response_message_type,
                    iter->response_type.length(), 0);
                response_data_ptr->data.element[element_index].response_message_type[iter->response_type.length()] = '\0';

                response_data_ptr->data.element[element_index].hash = iter->hash;

                element_index++;
            }
        }
        response_data_ptr->data.element_count = element_index;
    }
    else
    {
        recordLog(TP_COMM_LOG, TP_COMM_AUTHORITY_CHECK_FAILED, "/rpc/tp_comm/getRpcTable");
        initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
    }

    TpRequestResponse package;
    package.hash = 0x00004FA5;
    package.request_data_ptr = request_data_ptr;
    package.response_data_ptr = response_data_ptr; 

    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
}

// get publish table
void TpComm::handleRequest0x000147A5(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;

    if(request_data_ptr == NULL)
    {
        recordLog(TP_COMM_LOG, TP_COMM_MEMORY_OPERATION_FAILED, "/rpc/tp_comm/getPublishTable");
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }

    ResponseMessageType_Uint64_PublishTable* response_data_ptr = new ResponseMessageType_Uint64_PublishTable;
    if(response_data_ptr == NULL)
    {
        recordLog(TP_COMM_LOG, TP_COMM_MEMORY_OPERATION_FAILED, "/rpc/tp_comm/getPublishTable");
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    if(!decodeRequestPackage(RequestMessageType_Void_fields, (void*)request_data_ptr, recv_bytes))
    {
        recordLog(TP_COMM_LOG, TP_COMM_DECODE_FAILED, "/rpc/tp_comm/getPublishTable");
        FST_ERROR("Decode data failed");
        return ;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(0x000147A5);
    if(checkAuthority(request_data_ptr->property.authority, controller_authority))
    {
        initResponsePackage(request_data_ptr, response_data_ptr, -1);
        response_data_ptr->error_code.data = SUCCESS;
        recordLog(TP_COMM_LOG, SUCCESS, "/rpc/tp_comm/getPublishTable");

        int element_index = 0;
        for (std::vector<PublishService>::iterator iter = publish_element_table_.begin(); iter != publish_element_table_.end(); iter++)
        {
            Comm_Authority publish_element_authority = getPublishElementAuthorityByHash(iter->hash);

            if (checkAuthority(request_data_ptr->property.authority, publish_element_authority))
            {
                iter->element_path.copy(response_data_ptr->data.element[element_index].path, iter->element_path.length(), 0);
                response_data_ptr->data.element[element_index].path[iter->element_path.length()] = '\0';
                iter->element_type.copy(response_data_ptr->data.element[element_index].message_type, iter->element_type.length(), 0);
                response_data_ptr->data.element[element_index].message_type[iter->element_type.length()] = '\0';
                response_data_ptr->data.element[element_index].hash = iter->hash;
                element_index++;
            }
        }
        response_data_ptr->data.element_count = element_index;
    }
    else
    {
        recordLog(TP_COMM_LOG, TP_COMM_AUTHORITY_CHECK_FAILED, "/rpc/tp_comm/getPublishTable");
        initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
    }

    TpRequestResponse package;
    package.hash = 0x000147A5;
    package.request_data_ptr = request_data_ptr;
    package.response_data_ptr = response_data_ptr;

    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
}

// nonexistent rpc
void TpComm::handleRequestNonexistentHash(int hash, int recv_bytes)
{
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;

    if(request_data_ptr == NULL)
    {
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }

    ResponseMessageType_Void* response_data_ptr = new ResponseMessageType_Void;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    response_data_ptr->header.time_stamp = 0;
    response_data_ptr->header.package_left = -1;
    response_data_ptr->header.error_code = TP_COMM_INVALID_REQUEST;

    TpRequestResponse package;
    package.hash = hash;
    package.request_data_ptr = request_data_ptr;
    package.response_data_ptr = response_data_ptr;

    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
}