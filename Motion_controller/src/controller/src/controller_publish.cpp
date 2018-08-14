#include "controller_publish.h"

using namespace fst_ctrl;
using namespace fst_log;
using namespace fst_base;
using namespace fst_comm;

ControllerPublish::ControllerPublish():
    log_ptr_(NULL),
    param_ptr_(NULL),
    virtual_core1_ptr_(NULL),
    tp_comm_ptr_(NULL),
    state_machine_ptr_(NULL)
{

}

ControllerPublish::~ControllerPublish()
{

}

void ControllerPublish::init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, VirtualCore1* virtual_core1_ptr, TpComm* tp_comm_ptr,
                    ControllerSm* state_machine_ptr)
{
    log_ptr_ = log_ptr;
    param_ptr_ = param_ptr;
    virtual_core1_ptr_ = virtual_core1_ptr;
    tp_comm_ptr_ = tp_comm_ptr;
    state_machine_ptr_ = state_machine_ptr;

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

void ControllerPublish::initPublishQuickSearchTable()
{
    unsigned int remainder;
    for(unsigned int i = 0; i < publish_table_.size(); ++i)
    {
        remainder = publish_table_[i].hash % QUICK_SEARCH_TABLE_SIZE;
        publish_quick_search_table_[remainder].push_back(publish_table_[i]);
    }
}




