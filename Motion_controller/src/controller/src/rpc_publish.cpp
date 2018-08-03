#include "controller_rpc.h"

using namespace fst_ctrl;


// "/rpc/controller/addTopic"
void ControllerRpc::handleRpc0x00000773(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Topic* rq_data_ptr = static_cast<RequestMessageType_Topic*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);
    int element_count = 0;
    void* data_ptr;
    TpPublish task = tp_comm_ptr_->generateTpPublishTask(rq_data_ptr->data.topic_hash, rq_data_ptr->data.time_min, rq_data_ptr->data.time_max);
    for(unsigned int i = 0; i < rq_data_ptr->data.element_hash_list_count; ++i)
    {
        ControllerPublish::HandlePublishFuncPtr func_ptr = publish_.getPublishHandlerByHash(rq_data_ptr->data.element_hash_list[i]);
        if(func_ptr != NULL)
        {
            data_ptr = (publish_.*func_ptr)();
            if(data_ptr != NULL)
            {
                tp_comm_ptr_->addTpPublishElement(task, rq_data_ptr->data.element_hash_list[i], data_ptr);
                ++element_count;
            }
        }
    }
    if(element_count > 0)
    {
        tp_comm_ptr_->pushTaskToPublishList(task);
        rs_data_ptr->data.data = true;
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

