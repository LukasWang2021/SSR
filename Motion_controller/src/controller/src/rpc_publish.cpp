#include "controller_rpc.h"
#include "error_code.h"

using namespace fst_ctrl;
using namespace fst_comm;

// "/rpc/controller/addTopic"
void ControllerRpc::handleRpc0x000050E3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Topic* rq_data_ptr = static_cast<RequestMessageType_Topic*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    int element_count = 0;
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
        rs_data_ptr->data.data = CONTROLLER_PUBLISH_FAILED;
    }
    recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/addTopic"));
}

// "/rpc/publish/addRegTopic"
void ControllerRpc::handleRpc0x000163A3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Topic* rq_data_ptr = static_cast<RequestMessageType_Topic*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int element_count = 0;
    void* data_ptr;
    RegType reg_type;
    int reg_index;
    TpPublish task = tp_comm_ptr_->generateTpPublishTask(rq_data_ptr->data.topic_hash, rq_data_ptr->data.time_min, rq_data_ptr->data.time_max);
    for(unsigned int i = 0; i < rq_data_ptr->data.element_hash_list_count; ++i)
    {
        reg_type = (RegType)(rq_data_ptr->data.element_hash_list[i] >> 16);
        reg_index = (rq_data_ptr->data.element_hash_list[i] & 0x0000FFFF);
        switch(reg_type)
        {
            case REG_TYPE_PR:   data_ptr = reg_manager_ptr_->getPrRegValueById(reg_index); break;
            case REG_TYPE_HR:   data_ptr = reg_manager_ptr_->getHrRegValueById(reg_index); break;
            case REG_TYPE_SR:   data_ptr = reg_manager_ptr_->getSrRegValueById(reg_index); break;
            case REG_TYPE_MR:   data_ptr = reg_manager_ptr_->getMrRegValueById(reg_index); break;
            case REG_TYPE_R:    data_ptr = reg_manager_ptr_->getRRegValueById(reg_index); break;
            default:    data_ptr = NULL;
        }
        if(data_ptr != NULL)
        {
            tp_comm_ptr_->addTpPublishElement(task, rq_data_ptr->data.element_hash_list[i], data_ptr);
            ++element_count;
        }
    }
 
    if(element_count == rq_data_ptr->data.element_hash_list_count)
    {
        tp_comm_ptr_->pushTaskToPublishList(task);
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = CONTROLLER_PUBLISH_FAILED;
    }
    recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/addRegTopic"));
}

// "/rpc/publish/addIoTopic"
void ControllerRpc::handleRpc0x000058F3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Topic* rq_data_ptr = static_cast<RequestMessageType_Topic*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int element_count = 0;
    void* data_ptr;
    int device_index;
    int io_type;
    int io_offset;
    TpPublish task = tp_comm_ptr_->generateTpPublishTask(rq_data_ptr->data.topic_hash, rq_data_ptr->data.time_min, rq_data_ptr->data.time_max);
    for(unsigned int i = 0; i < rq_data_ptr->data.element_hash_list_count; ++i)
    {
        device_index = rq_data_ptr->data.element_hash_list[i] >> 24;
        io_type = (rq_data_ptr->data.element_hash_list[i] >> 16) & 0x000000FF;
        io_offset = (rq_data_ptr->data.element_hash_list[i] & 0x0000FFFF);

        // do io publish mapping
    }
    
    if(element_count == rq_data_ptr->data.element_hash_list_count)
    {
        tp_comm_ptr_->pushTaskToPublishList(task);
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = CONTROLLER_PUBLISH_FAILED;
    }
    recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/addIoTopic"));
}


