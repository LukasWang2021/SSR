#include "tp_comm.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

using namespace fst_comm;

using namespace std;


// get rpc table
void TpComm::handleRequest0x00004FA5(int recv_bytes)
{
   // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;

    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }

    ResponseMessageType_RpcTable* response_data_ptr = new ResponseMessageType_RpcTable;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    if(!decodeRequestPackage(RequestMessageType_Void_fields, (void*)request_data_ptr, recv_bytes))
    {
        FST_ERROR("handleRequestPackage: decode data failed");
        return ;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(0x00004FA5);
    if(!checkAuthority(request_data_ptr->property.authority, controller_authority))
    {
        FST_ERROR("handleRequestPackage: operation is not authorized");
        initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
        return;
    }

    initResponsePackage(request_data_ptr, response_data_ptr, -1);

    response_data_ptr->header.time_stamp = 11;
    response_data_ptr->header.package_left = 12;
    response_data_ptr->header.error_code = 0;
    response_data_ptr->property.authority = Comm_Authority_TP;

    std::vector<RpcService>::iterator iter;
    int element_index = 0;

    for (iter = rpc_table_.begin(); iter != rpc_table_.end(); iter++)
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

    TpRequestResponse package;
    package.hash = 0x00004FA5;
    package.request_data_ptr = request_data_ptr;
    package.response_data_ptr = response_data_ptr; 

    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
}

// get publish tables
void TpComm::handleRequest0x000147A5(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;

    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }

    ResponseMessageType_PublishTable* response_data_ptr = new ResponseMessageType_PublishTable;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    if(!decodeRequestPackage(RequestMessageType_Void_fields, (void*)request_data_ptr, recv_bytes))
    {
        FST_ERROR("handleRequestPackage: decode data failed");
        return ;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(0x000147A5);
    if(!checkAuthority(request_data_ptr->property.authority, controller_authority))
    {
        FST_ERROR("handleRequestPackage: operation is not authorized");
        initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
    }
    else
    {
        initResponsePackage(request_data_ptr, response_data_ptr, -1);
    }

    TpRequestResponse package;
    package.hash = 0x000147A5;
    package.request_data_ptr = request_data_ptr;

    response_data_ptr->header.time_stamp = 11;
    response_data_ptr->header.package_left = 12;
    response_data_ptr->header.error_code = 0;
    response_data_ptr->property.authority = Comm_Authority_TP;

    std::vector<PublishService>::iterator iter;
    int element_index = 0;

    for (iter = publish_element_table_.begin(); iter != publish_element_table_.end(); iter++)
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

    package.response_data_ptr = response_data_ptr; 

    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
}

