#include "controller_rpc.h"
#include "common_error_code.h"

using namespace user_space;
using namespace log_space;

// "/rpc/controller/addTopic"
void ControllerRpc::handleRpc0x000050E3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Topic* rq_data_ptr = static_cast<RequestMessageType_Topic*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(tp_comm_ptr_->isTopicExisted(rq_data_ptr->data.topic_hash))
    {
        rs_data_ptr->data.data = 0;
        LogProducer::warn("rpc", "/rpc/controller/addTopic is exist. Error = 0x%llx", rs_data_ptr->data.data);
        return;        
    }

    unsigned int element_count = 0;
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
    LogProducer::warn("rpc", "element_count=%d, rq_data_ptr->data.element_hash_list_count=%d", element_count,rq_data_ptr->data.element_hash_list_count);
    if(element_count == rq_data_ptr->data.element_hash_list_count)
    {
        tp_comm_ptr_->pushTaskToPublishList(task);
        rs_data_ptr->data.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/controller/addTopic success");
    }
    else
    {
        rs_data_ptr->data.data = CONTROLLER_PUBLISH_NONE;
        LogProducer::error("rpc", "/rpc/controller/addTopic failed. Error = 0x%llx", rs_data_ptr->data.data);
    }
}


// "/rpc/publish/deleteTopic"
void ControllerRpc::handleRpc0x00004403(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Uint32* rq_data_ptr = static_cast<RequestMessageType_Uint32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    std::vector<TpPublishElement> publish_element_list = tp_comm_ptr_->eraseTaskFromPublishList(rq_data_ptr->data.data);
    publish_ptr_->deleteTaskFromUpdateList(publish_element_list);
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/controller/deleteTopic success");
}


