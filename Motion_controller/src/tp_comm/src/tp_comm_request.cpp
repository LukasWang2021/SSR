#include "tp_comm.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>
#include "common/common.h"


using namespace std;

// "/tp_comm/test_request"
void TpComm::handleRequest0xcf0be243(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Int32* response_data_ptr = new ResponseMessageType_Int32;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0xcf0be243, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

void TpComm::handleRequest0x0000F933(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Int32* response_data_ptr = new ResponseMessageType_Int32;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000F933, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

void TpComm::handleRequest0x0000E9D3(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Int32* response_data_ptr = new ResponseMessageType_Int32;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000E9D3, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}
void TpComm::handleRequest0x00000AB3(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Int32* response_data_ptr = new ResponseMessageType_Int32;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00000AB3, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

void TpComm::handleRequest0x00010945(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Int32* response_data_ptr = new ResponseMessageType_Int32;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00010945, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

void TpComm::handleRequest0x000067A4(int recv_bytes)
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

    handleRequestPackage(0x000067A4, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_Int32_fields, -1);
}

void TpComm::handleRequest0x00010685(int recv_bytes)
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
    
    handleRequestPackage(0x00010685, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_Int32_fields, -1);
}

void TpComm::handleRequest0x00010AB3(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_String* response_data_ptr = new ResponseMessageType_String;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00010AB3, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_Void_fields, -1);
}

void TpComm::handleRequest0x00017C25(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_UnsignedInt64* response_data_ptr = new ResponseMessageType_UnsignedInt64;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00017C25, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_Void_fields, -1);
}

void TpComm::handleRequest0x000093EE(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Int32List* response_data_ptr = new ResponseMessageType_Int32List;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000093EE, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_Void_fields, -1);
}

void TpComm::handleRequest0x0000FC15(int recv_bytes)
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
        FST_ERROR("handleRequestPackage: /tp_comm/test_request: decode data failed");
        return ;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(0x0000FC15);
    if(!checkAuthority(request_data_ptr->property.authority, controller_authority))
    {
        FST_ERROR("handleRequestPackage: /tp_comm/test_request: operation is not authorized");
        initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
    }
    else
    {
        initResponsePackage(request_data_ptr, response_data_ptr, -1);
    }

    TpRequestResponse package;
    package.hash = 0x0000FC15;
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

    FST_INFO("Here : response_list_mutex_ push over");
}

void TpComm::handleRequest0x00013EA3(int recv_bytes)
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

//    handleRequestPackage(0x00013EA3, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
//        RequestMessageType_Topic_fields, -1);

    if(!decodeRequestPackage(RequestMessageType_Topic_fields, (void*)request_data_ptr, recv_bytes))
    {
        FST_ERROR("handleRequestPackage: ...: decode data failed");
        return ;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(0x00013EA3);
    if(!checkAuthority(request_data_ptr->property.authority, controller_authority))
    {
        FST_ERROR("handleRequestPackage: ...: operation is not authorized");
        initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
        return;
    }

    for (int i = 0; i != request_data_ptr->data.element_hash_list_count; ++i)
    {
        Comm_Authority publish_element_authority = 
            getPublishElementAuthorityByHash(request_data_ptr->data.element_hash_list[i]);

        if (!checkAuthority(request_data_ptr->property.authority, publish_element_authority))
        {
            FST_ERROR("handleRequestPackage: ...: publish element is not authorized");
            initCommFailedResponsePackage(request_data_ptr, response_data_ptr);

            TpRequestResponse package;
            package.hash = 0x00013EA3;
            package.request_data_ptr = request_data_ptr;
            package.response_data_ptr = response_data_ptr; 
        
            response_list_mutex_.lock();
            response_list_.push_back(package);
            response_list_mutex_.unlock();
            return;
        }
    }

    initResponsePackage(request_data_ptr, response_data_ptr, -1);

    TpPublish task;
    task.hash = request_data_ptr->data.topic_hash;
    task.interval_min = request_data_ptr->data.time_min;
    task.interval_max = request_data_ptr->data.time_max;
    task.is_element_changed = true;
    task.last_publish_time.tv_sec = 0;
    task.last_publish_time.tv_usec = 0;
    task.package.element_count = request_data_ptr->data.element_hash_list_count;
    task.package.time_stamp = 100;
    task.package.element[0].hash = request_data_ptr->data.element_hash_list[0];
    task.package.element[1].hash =request_data_ptr->data.element_hash_list[1];
    task.package.element[2].hash =request_data_ptr->data.element_hash_list[2];
    addTpPublishElement(task, task.package.element[0].hash, (void*)&task.interval_max);
    addTpPublishElement(task, task.package.element[1].hash, (void*)&task.interval_max);
    addTpPublishElement(task, task.package.element[0].hash, (void*)&task.interval_max);
    pushTaskToPublishList(task);

    response_data_ptr->header.time_stamp = 11;
    response_data_ptr->header.package_left = 12;
    response_data_ptr->header.error_code = 0;
    response_data_ptr->property.authority = Comm_Authority_TP;
    response_data_ptr->data.data = true;

    TpRequestResponse package;
    package.hash = 0x00013EA3;
    package.request_data_ptr = request_data_ptr;
    package.response_data_ptr = response_data_ptr; 

    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
}

void TpComm::handleRequest0x00009BC5(int recv_bytes)
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
        FST_ERROR("handleRequestPackage: /tp_comm/test_request: decode data failed");
        return ;
    }

    Comm_Authority controller_authority = getRpcTableElementAuthorityByHash(0x00009BC5);
    if(!checkAuthority(request_data_ptr->property.authority, controller_authority))
    {
        FST_ERROR("handleRequestPackage: /tp_comm/test_request: operation is not authorized");
        initCommFailedResponsePackage(request_data_ptr, response_data_ptr);
        return;
    }

    initResponsePackage(request_data_ptr, response_data_ptr, -1);

    TpRequestResponse package;
    package.hash = 0x00009BC5;
    package.request_data_ptr = request_data_ptr;

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
    package.response_data_ptr = response_data_ptr; 

    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();

    FST_INFO("Here : response_list_mutex_ push over");
}

//"/rpc/register/addR"
void TpComm::handleRequest0x00007CF2(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_RegisterR* request_data_ptr = new RequestMessageType_RegisterR;
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
    
    handleRequestPackage(0x00007CF2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_RegisterR_fields, -1);
}

// "/rpc/register/updateR"
void TpComm::handleRequest0x000031B2(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_RegisterR* request_data_ptr = new RequestMessageType_RegisterR;
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
    
    handleRequestPackage(0x000031B2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_RegisterR_fields, -1);
}

// "/rpc/register/deleteR"
void TpComm::handleRequest0x00013062(int recv_bytes)
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
    
    handleRequestPackage(0x00013062, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}

// "/rpc/register/getR"
void TpComm::handleRequest0x000116F2(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_UnsignedInt32* request_data_ptr = new RequestMessageType_UnsignedInt32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_RegisterR* response_data_ptr = new ResponseMessageType_RegisterR;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000116F2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}

// "/rpc/register/setActivateR"
void TpComm::handleRequest0x000098C2(int recv_bytes)
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
    
    handleRequestPackage(0x000098C2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}

// "/rpc/register/getActivateR"
void TpComm::handleRequest0x00005602(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_UnsignedInt32* response_data_ptr = new ResponseMessageType_UnsignedInt32;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000098C2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_Void_fields, -1);
}

// add pr
void TpComm::handleRequest0x0000F862(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_RegisterPR* request_data_ptr = new RequestMessageType_RegisterPR;
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
    
    handleRequestPackage(0x0000F862, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_RegisterPR_fields, -1);
}

// "/rpc/register/updatePR"
void TpComm::handleRequest0x0000C032(int recv_bytes)
{
   // create object for request and response package
    RequestMessageType_RegisterPR* request_data_ptr = new RequestMessageType_RegisterPR;
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
    
    handleRequestPackage(0x0000C032, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_RegisterPR_fields, -1);
}

// delete pr
void TpComm::handleRequest0x00005392(int recv_bytes)
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
    
    handleRequestPackage(0x00005392, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}

// get pr
void TpComm::handleRequest0x000170A2(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_UnsignedInt32* request_data_ptr = new RequestMessageType_UnsignedInt32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_RegisterPR* response_data_ptr = new ResponseMessageType_RegisterPR;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000170A2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}

// set activate pr
void TpComm::handleRequest0x000116B2(int recv_bytes)
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
    
    handleRequestPackage(0x000116B2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}
// get activate pr
void TpComm::handleRequest0x0000F402(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_UnsignedInt32* response_data_ptr = new ResponseMessageType_UnsignedInt32;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000F402, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_Void_fields, -1);
}

// "/rpc/register/addMR"
void TpComm::handleRequest0x0000F892(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_RegisterMR* request_data_ptr = new RequestMessageType_RegisterMR;
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
    
    handleRequestPackage(0x0000F892, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_RegisterMR_fields, -1);
}

// update mr
void TpComm::handleRequest0x0000C1C2(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_RegisterMR* request_data_ptr = new RequestMessageType_RegisterMR;
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
    
    handleRequestPackage(0x0000C1C2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_RegisterMR_fields, -1);
}

// delete mr
void TpComm::handleRequest0x000053E2(int recv_bytes)
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
    
    handleRequestPackage(0x000053E2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}

// get mr
void TpComm::handleRequest0x000170D2(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_UnsignedInt32* request_data_ptr = new RequestMessageType_UnsignedInt32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_RegisterMR* response_data_ptr = new ResponseMessageType_RegisterMR;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000170D2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}

// set activate mr
void TpComm::handleRequest0x00011642(int recv_bytes)
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
    
    handleRequestPackage(0x00011642, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}

// get activate mr
void TpComm::handleRequest0x0000F3B2(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_UnsignedInt32* response_data_ptr = new ResponseMessageType_UnsignedInt32;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000F3B2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_Void_fields, -1);
}

// add sr
void TpComm::handleRequest0x0000F932(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_RegisterSR* request_data_ptr = new RequestMessageType_RegisterSR;
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
    
    handleRequestPackage(0x0000F932, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_RegisterSR_fields, -1);
}

// update sr
void TpComm::handleRequest0x0000BF62(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_RegisterSR* request_data_ptr = new RequestMessageType_RegisterSR;
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
    
    handleRequestPackage(0x0000BF62, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_RegisterSR_fields, -1);
}

// delete sr
void TpComm::handleRequest0x00005342(int recv_bytes)
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
    
    handleRequestPackage(0x00005342, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}

// get sr
void TpComm::handleRequest0x00017172(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_UnsignedInt32* request_data_ptr = new RequestMessageType_UnsignedInt32;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_RegisterSR* response_data_ptr = new ResponseMessageType_RegisterSR;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00017172, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}

// set activate sr
void TpComm::handleRequest0x000115E2(int recv_bytes)
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
    
    handleRequestPackage(0x000115E2, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_UnsignedInt32_fields, -1);
}

// get activate sr
void TpComm::handleRequest0x0000F352(int recv_bytes)
{
        // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_UnsignedInt32* response_data_ptr = new ResponseMessageType_UnsignedInt32;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000F352, (void*)request_data_ptr, (void*)response_data_ptr, recv_bytes, 
        RequestMessageType_Void_fields, -1);
}
