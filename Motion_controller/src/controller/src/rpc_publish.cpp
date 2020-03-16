#include "controller_rpc.h"
#include "error_code.h"

using namespace fst_ctrl;
using namespace fst_comm;

// "/rpc/controller/addTopic"
void ControllerRpc::handleRpc0x000050E3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Topic* rq_data_ptr = static_cast<RequestMessageType_Topic*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(tp_comm_ptr_->isTopicExisted(rq_data_ptr->data.topic_hash))
    {
        rs_data_ptr->data.data = CONTROLLER_PUBLISH_EXIST;
        //recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/addTopic"));
        return;        
    }

    uint32_t element_count = 0;
    void* data_ptr;    
    TpPublish task = tp_comm_ptr_->generateTpPublishTask(rq_data_ptr->data.topic_hash, rq_data_ptr->data.time_min, rq_data_ptr->data.time_max);
    for(unsigned int i = 0; i < rq_data_ptr->data.element_hash_list_count; ++i)
    {
        ControllerPublish::HandlePublishFuncPtr func_ptr = publish_ptr_->getPublishHandlerByHash(rq_data_ptr->data.element_hash_list[i]);
        if(func_ptr != NULL)
        {
            data_ptr = (publish_ptr_->*func_ptr)();
            if(data_ptr != NULL)
            {                
                ControllerPublish::HandleUpdateFuncPtr update_func_ptr = publish_ptr_->getUpdateHandlerByHash(rq_data_ptr->data.element_hash_list[i]);
                if(update_func_ptr != NULL)
                {
                    publish_ptr_->addTaskToUpdateList(update_func_ptr);
                }
                tp_comm_ptr_->addTpPublishElement(task, rq_data_ptr->data.element_hash_list[i], data_ptr);
                ++element_count;
            }
        }
    }  

    if(element_count == rq_data_ptr->data.element_hash_list_count)
    {
        tp_comm_ptr_->pushTaskToPublishList(task);
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = CONTROLLER_PUBLISH_NONE;
    }
    recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/addTopic"));
}

// "/rpc/publish/addRegTopic"
void ControllerRpc::handleRpc0x000163A3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Topic* rq_data_ptr = static_cast<RequestMessageType_Topic*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    // check if topic have been exist
    if(tp_comm_ptr_->isRegTopicExisted(rq_data_ptr->data.topic_hash))
    {
        rs_data_ptr->data.data = CONTROLLER_PUBLISH_EXIST;
        //recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/addRegTopic"));
        return;        
    }
    RegType reg_type;
    int reg_index;
    // check if all requested regs were valid
    for(unsigned int i = 0; i < rq_data_ptr->data.element_hash_list_count; ++i)
    {
        reg_type = (RegType)(rq_data_ptr->data.element_hash_list[i] >> 16);
        reg_index = (rq_data_ptr->data.element_hash_list[i] & 0x0000FFFF);
        if(!reg_manager_ptr_->isRegValid(reg_type, reg_index))
        {
            rs_data_ptr->data.data = CONTROLLER_PUBLISH_NONE;
            recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/addRegTopic"));
            return; 
        }
    }

    uint32_t element_count = 0;
    void* data_ptr;
    TpPublish task = tp_comm_ptr_->generateTpPublishTask(rq_data_ptr->data.topic_hash, rq_data_ptr->data.time_min, rq_data_ptr->data.time_max);
    for(unsigned int i = 0; i < rq_data_ptr->data.element_hash_list_count; ++i)
    {
        reg_type = (RegType)(rq_data_ptr->data.element_hash_list[i] >> 16);
        reg_index = (rq_data_ptr->data.element_hash_list[i] & 0x0000FFFF);
        data_ptr = publish_ptr_->addTaskToRegUpdateList(reg_type, reg_index);
        if(data_ptr != NULL)
        {            
            tp_comm_ptr_->addTpPublishElement(task, rq_data_ptr->data.element_hash_list[i], data_ptr);
            ++element_count;
        }        
    }
 
    if(element_count == rq_data_ptr->data.element_hash_list_count)
    {
        tp_comm_ptr_->pushTaskToRegPublishList(task);
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = CONTROLLER_PUBLISH_NONE;
    }
    recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/addRegTopic"));
}

// "/rpc/publish/addIoTopic"
void ControllerRpc::handleRpc0x000058F3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Topic* rq_data_ptr = static_cast<RequestMessageType_Topic*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(tp_comm_ptr_->isIoTopicExisted(rq_data_ptr->data.topic_hash))
    {
        rs_data_ptr->data.data = CONTROLLER_PUBLISH_EXIST;
        //recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/addIoTopic"));
        return;        
    }

    uint32_t element_count = 0;
    void* data_ptr;
    uint32_t port_type;
    uint32_t port_offset;
    TpPublish task = tp_comm_ptr_->generateTpPublishTask(rq_data_ptr->data.topic_hash, rq_data_ptr->data.time_min, rq_data_ptr->data.time_max);
    for(unsigned int i = 0; i < rq_data_ptr->data.element_hash_list_count; ++i)
    {
        port_type = (rq_data_ptr->data.element_hash_list[i] >> 16) & 0x0000FFFF;
        port_offset = (rq_data_ptr->data.element_hash_list[i] & 0x0000FFFF);

        data_ptr = publish_ptr_->addTaskToIoUpdateList(port_type, port_offset);
        if (data_ptr != NULL)
        {
            tp_comm_ptr_->addTpPublishElement(task, rq_data_ptr->data.element_hash_list[i], data_ptr);
            ++element_count;
        }
    }
    
    if(element_count == rq_data_ptr->data.element_hash_list_count)
    {
        tp_comm_ptr_->pushTaskToIoPublishList(task);
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = CONTROLLER_PUBLISH_NONE;
    }
    recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/addIoTopic"));
}

// "/rpc/publish/deleteTopic"
void ControllerRpc::handleRpc0x00004403(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Uint32* rq_data_ptr = static_cast<RequestMessageType_Uint32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    std::vector<TpPublishElement> publish_element_list = tp_comm_ptr_->eraseTaskFromPublishList(rq_data_ptr->data.data);
    publish_ptr_->deleteTaskFromUpdateList(publish_element_list);
    rs_data_ptr->data.data = SUCCESS;
    recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/deleteTopic"));
}

// "/rpc/publish/deleteRegTopic"
void ControllerRpc::handleRpc0x00010353(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Uint32* rq_data_ptr = static_cast<RequestMessageType_Uint32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    std::vector<TpPublishElement> publish_element_list = tp_comm_ptr_->eraseTaskFromRegPublishList(rq_data_ptr->data.data);
    publish_ptr_->deleteTaskFromRegUpdateList(publish_element_list);
    rs_data_ptr->data.data = SUCCESS;
    recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/deleteRegTopic"));
}

// "/rpc/publish/deleteIoTopic"
void ControllerRpc::handleRpc0x0000DD03(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Uint32* rq_data_ptr = static_cast<RequestMessageType_Uint32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    std::vector<TpPublishElement> publish_element_list = tp_comm_ptr_->eraseTaskFromIoPublishList(rq_data_ptr->data.data);
    publish_ptr_->deleteTaskFromIoUpdateList(publish_element_list);
    rs_data_ptr->data.data = SUCCESS;
    recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/deleteIoTopic"));
}


