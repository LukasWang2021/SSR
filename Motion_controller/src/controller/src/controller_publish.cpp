#include "controller_publish.h"

using namespace fst_ctrl;
using namespace fst_log;
using namespace fst_base;
using namespace fst_comm;
using namespace fst_mc;

ControllerPublish::ControllerPublish():
    log_ptr_(NULL),
    param_ptr_(NULL),
    virtual_core1_ptr_(NULL),
    tp_comm_ptr_(NULL),
    state_machine_ptr_(NULL),
    reg_manager_ptr_(NULL),
    controller_client_ptr_(NULL)
{

}

ControllerPublish::~ControllerPublish()
{

}

void ControllerPublish::init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, VirtualCore1* virtual_core1_ptr, TpComm* tp_comm_ptr,
                    ControllerSm* state_machine_ptr, MotionControl* motion_control_ptr, RegManager* reg_manager_ptr,
                    ControllerClient* controller_client_ptr,
                    fst_hal::DeviceManager* device_manager_ptr, fst_ctrl::IoMapping* io_mapping_ptr)//feng add device_manager_ptr and mapping
{
    log_ptr_ = log_ptr;
    param_ptr_ = param_ptr;
    virtual_core1_ptr_ = virtual_core1_ptr;
    tp_comm_ptr_ = tp_comm_ptr;
    state_machine_ptr_ = state_machine_ptr;
    motion_control_ptr_ = motion_control_ptr;
    reg_manager_ptr_ = reg_manager_ptr;
    controller_client_ptr_ = controller_client_ptr;
    device_manager_ptr_ = device_manager_ptr;//feng add for rpc-addIoTopic
    io_mapping_ptr_ = io_mapping_ptr; //feng add for mapping.

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

void* ControllerPublish::addTaskToRegUpdateList(RegType reg_type, int reg_index)
{
    std::list<RegPublishUpdate>::iterator it;
    for(it = reg_update_list_.begin(); it != reg_update_list_.end(); ++it)
    {
        if(it->reg_type == reg_type
            && it->reg_index == reg_index)
        {
            goto PTR_RETURN;
        }
    }

    if(reg_update_list_.size() >= param_ptr_->max_reg_publish_number_)
    {
        return NULL;
    }

    RegPublishUpdate reg_publish_update;
    reg_publish_update.reg_type = reg_type;
    reg_publish_update.reg_index = reg_index;
    reg_publish_update.ref_count = 0;
    reg_update_list_.push_back(reg_publish_update);
    it = reg_update_list_.end();
    --it;
PTR_RETURN:
    ++it->ref_count;
    switch(reg_type)
    {
        case REG_TYPE_PR: return (void*)&it->pr_value;
        case REG_TYPE_HR: return (void*)&it->hr_value;
        case REG_TYPE_SR: return (void*)&it->sr_value;
        case REG_TYPE_MR: return (void*)&it->mr_value;
        case REG_TYPE_R: return (void*)&it->r_value;
        default: return NULL;
    }
}

//feng add
void* ControllerPublish::addTaskToIoUpdateList(int device_index, int port_type, int port_offset)//feng add for io publish
{
    std::vector<fst_hal::DeviceInfo> device_list = device_manager_ptr_->getDeviceList();
    int address = -1;

    std::list<IoPublishUpdate>::iterator it;
    for(it = io_update_list_.begin(); it != io_update_list_.end(); ++it)
    {
        if(it->device_index == device_index && it->port_type == port_type && it->port_offset == port_offset)
        {
            goto PTR_RETURN;
        }
    }

    if(io_update_list_.size() >= param_ptr_->max_io_publish_number_)
    {
        return NULL;
    }

    //find address according device_index.
    for (unsigned int i = 0; i < device_list.size(); ++i)
    {
        if (device_list[i].index == device_index)
        {
            address = device_list[i].address;
            break;
        }
    }

    IoPublishUpdate io_publish_update;
    io_publish_update.device_index = device_index;
    io_publish_update.address = address;
    io_publish_update.port_type = port_type;
    io_publish_update.port_offset = port_offset;
    io_publish_update.is_valid = true;
    io_publish_update.ref_count = 0;
    io_update_list_.push_back(io_publish_update);
    it = io_update_list_.end();
    --it;
PTR_RETURN:
    ++it->ref_count;
    if(it->is_valid == true)
    {
        return (void*)&it->value;
    }
    return NULL;


/*
    fst_hal::PhysicsID id;
    id.info.port = port_offset;
    id.info.port_type = port_type;
    id.info.address = address;
    id.info.dev_type = fst_hal::DEVICE_TYPE_FST_IO;

    uint8_t value = 0;
    ErrorCode ret = SUCCESS;
    switch(port_type)
    {
        case fst_hal::IO_TYPE_DI: ret = io_mapping_ptr_->getDIByBit(id.number, value);break;
        case fst_hal::IO_TYPE_DO: ret = io_mapping_ptr_->getDOByBit(id.number, value);break;
        case fst_hal::IO_TYPE_RI: ret = io_mapping_ptr_->getRIByBit(id.number, value);break;
        case fst_hal::IO_TYPE_RO: ret = io_mapping_ptr_->getROByBit(id.number, value);break;
        default: return NULL;
    }

    MessageType_Uint32 v = static_cast<MessageType_Uint32>(value);
    return (void*)&v;
    */
}



void ControllerPublish::deleteTaskFromUpdateList(std::vector<fst_comm::TpPublishElement>& publish_element_list)
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

void ControllerPublish::deleteTaskFromRegUpdateList(std::vector<fst_comm::TpPublishElement>& publish_element_list)
{
    RegType reg_type;
    int reg_index;
    std::vector<TpPublishElement>::iterator it;
    for(it = publish_element_list.begin(); it != publish_element_list.end(); ++it)
    {
        reg_type = (RegType)(it->hash >> 16);
        reg_index = (it->hash & 0x0000FFFF);
        unrefRegUpdateListElement(reg_type, reg_index);
    }
    cleanRegUpdateList();
}

void ControllerPublish::unrefRegUpdateListElement(RegType reg_type, int reg_index)
{
    std::list<RegPublishUpdate>::iterator it;
    for(it = reg_update_list_.begin(); it != reg_update_list_.end(); ++it)
    {
        if(it->reg_type == reg_type
            && it->reg_index == reg_index)
        {
            --it->ref_count;
            return;
        }
    }
}

void ControllerPublish::cleanRegUpdateList()
{
    std::list<RegPublishUpdate>::iterator it;
    for(it = reg_update_list_.begin(); it != reg_update_list_.end(); ++it)
    {
        if(it->ref_count <= 0)
        {
            it = reg_update_list_.erase(it);
            --it;
        }
    }
}


//feng add
void ControllerPublish::deleteTaskFromIoUpdateList(std::vector<fst_comm::TpPublishElement>& publish_element_list)
{
    int device_index;
    int port_type;
    int port_offset;
    std::vector<TpPublishElement>::iterator it;
    for(it = publish_element_list.begin(); it != publish_element_list.end(); ++it)
    {
        device_index = (it->hash >> 24);
        port_type = ((it->hash >> 16) & 0x000000FF);
        port_offset = (it->hash & 0x0000FFFF);
        unrefIoUpdateListElement(device_index, port_type, port_offset);
    }
    cleanIoUpdateList();
}
//feng add
void ControllerPublish::unrefIoUpdateListElement(int device_index, int port_type, int port_offset)
{
    std::list<IoPublishUpdate>::iterator it;
    for(it = io_update_list_.begin(); it != io_update_list_.end(); ++it)
    {
        if(it->device_index == device_index && it->port_type == port_type && it->port_offset == port_offset)
        {
            --it->ref_count;
            return;
        }
    }
}
//feng add
void ControllerPublish::cleanIoUpdateList()
{
    std::list<IoPublishUpdate>::iterator it;
    for(it = io_update_list_.begin(); it != io_update_list_.end(); ++it)
    {
        if(it->ref_count <= 0)
        {
            it = io_update_list_.erase(it);
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

    tp_comm_ptr_->lockRegPublishMutex();
    updateReg();
    tp_comm_ptr_->unlockRegPublishMutex();

    //tp_comm_ptr_->lockIoPublishMutex();
    updateIo();
    //tp_comm_ptr_->unlockIoPublishMutex();

}



