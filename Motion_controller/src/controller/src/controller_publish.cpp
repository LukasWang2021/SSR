#include "controller_publish.h"

using namespace user_space;


ControllerPublish::ControllerPublish():
    tp_comm_ptr_(NULL),
    cpu_comm_ptr_(NULL)
{
    for(size_t i = 0; i < AXIS_NUM; ++i)
    {
        axis_ptr_[i] = NULL;
    }
}

ControllerPublish::~ControllerPublish()
{

}

void ControllerPublish::init(user_space::TpComm* tp_comm_ptr, servo_comm_space::ServoCpuCommBase* cpu_comm_ptr, 
        axis_space::Axis* axis_ptr[AXIS_NUM], group_space::MotionControl* group_ptr[GROUP_NUM], 
        hal_space::Io1000* io_dev_ptr, hal_space::IoSafety* safety_ptr)
{
    tp_comm_ptr_ = tp_comm_ptr; 
    cpu_comm_ptr_ = cpu_comm_ptr;
    for(size_t i = 0; i < AXIS_NUM; ++i)
    {
        axis_ptr_[i] = axis_ptr[i];
    }
    for(size_t i = 0; i < GROUP_NUM; ++i)
    {
        group_ptr_[i] = group_ptr[i];
    }
    io_dev_ptr_ = io_dev_ptr;
    safety_ptr_ = safety_ptr;

    initPublishTable();
    initPublishQuickSearchTable();
}

ControllerPublish::HandlePublishFuncPtr ControllerPublish::getPublishHandlerByHash(unsigned int hash)
{
    unsigned int remainder = hash % QUICK_SEARCH_TABLE_SIZE;
    for(unsigned int i = 0; i < publish_quick_search_table_[remainder].size(); ++i)
    {
        if(publish_quick_search_table_[remainder][i].hash == hash)
        {
            return publish_quick_search_table_[remainder][i].publish_func_ptr;
        }
    }
    return NULL;
}

ControllerPublish::HandleUpdateFuncPtr ControllerPublish::getUpdateHandlerByHash(unsigned int hash)
{
    unsigned int remainder = hash % QUICK_SEARCH_TABLE_SIZE;
    for(unsigned int i = 0; i < publish_quick_search_table_[remainder].size(); ++i)
    {
        if(publish_quick_search_table_[remainder][i].hash == hash)
        {
            return publish_quick_search_table_[remainder][i].update_func_ptr;
        }
    }
    return NULL;
}

void ControllerPublish::initPublishQuickSearchTable()
{
    unsigned int remainder;
    for(unsigned int i = 0; i < publish_table_.size(); ++i)
    {
        remainder = publish_table_[i].hash % QUICK_SEARCH_TABLE_SIZE;
        publish_quick_search_table_[remainder].push_back(publish_table_[i]);
    }
}

void ControllerPublish::addTaskToUpdateList(HandleUpdateFuncPtr func_ptr)
{
    std::list<PublishUpdate>::iterator it;
    for(it = update_list_.begin(); it != update_list_.end(); ++it)
    {
        if(it->func_ptr == func_ptr)
        {
            ++it->ref_count;
            return;
        }
    }
    PublishUpdate publish_update;
    publish_update.func_ptr = func_ptr;
    publish_update.ref_count = 1;
    update_list_.push_back(publish_update);
}

void ControllerPublish::deleteTaskFromUpdateList(std::vector<user_space::TpPublishElement>& publish_element_list)
{
    HandleUpdateFuncPtr func_ptr;
    std::vector<TpPublishElement>::iterator it;
    for(it = publish_element_list.begin(); it != publish_element_list.end(); ++it)
    {
        func_ptr = getUpdateHandlerByHash(it->hash);
        unrefUpdateListElement(func_ptr);
    }
    cleanUpdateList();    
}

void ControllerPublish::unrefUpdateListElement(HandleUpdateFuncPtr func_ptr)
{
    std::list<PublishUpdate>::iterator it;
    for(it = update_list_.begin(); it != update_list_.end(); ++it)
    {
        if(it->func_ptr == func_ptr)
        {
            --it->ref_count;
            return;
        }
    }
}

void ControllerPublish::cleanUpdateList()
{
    std::list<PublishUpdate>::iterator it;
    for(it = update_list_.begin(); it != update_list_.end(); ++it)
    {
        if(it->ref_count <= 0)
        {
            it = update_list_.erase(it);
            --it;
        }
    }
}


void ControllerPublish::processPublish()
{
    HandleUpdateFuncPtr update_func_ptr;
    std::list<PublishUpdate>::iterator it;
    tp_comm_ptr_->lockPublishMutex(); 
    for(it = update_list_.begin(); it != update_list_.end(); ++it)
    {
        update_func_ptr = it->func_ptr;
        if(update_func_ptr != NULL)
        {
            (this->*update_func_ptr)();
        }
    }
    tp_comm_ptr_->unlockPublishMutex(); 

}



