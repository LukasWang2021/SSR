#include "system/core_comm_system.h"
#include "common/core_comm_config_base.h"
#include "common/boardcast_base.h"
#include "common/cpu_ack_base.h"
#include <iostream>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>

using namespace std;
using namespace core_comm_space;

CoreCommSystem::CoreCommSystem():
    boardcast_ptr_(NULL),
    config_ptr_(NULL),
    comm_ptr_(NULL),
    from_block_ptr_(NULL),
    from_block_number_(0),
    to_block_ptr_(NULL),
    to_block_number_(0)
{
    
}

CoreCommSystem::~CoreCommSystem()
{
    boardcast_ptr_ = NULL;
    config_ptr_ = NULL;
    comm_ptr_ = NULL;
    closeDevice(device_);
    if(from_block_ptr_ != NULL)
    {
        from_block_number_ = 0;
        free(from_block_ptr_);
        from_block_ptr_ = NULL;
    }
    if(to_block_ptr_ != NULL)
    {
        to_block_number_ = 0;
        free(to_block_ptr_);
        to_block_ptr_ = NULL;
    }
}

ErrorCode CoreCommSystem::initAsMaster()
{
    std::string error_msg;
    if(!param_.loadParam())
    {
        return CORE_COMM_LOAD_PARAM_FAILED;
    }
    
    if(!config_.parseConfigXml(param_.config_file_, error_msg))
    {
        std::cout<<error_msg<<std::endl;
        return CORE_COMM_LOAD_CORE_COMM_CONFIG_FAILED;
    }
    else
    {
        ;//config_.printCoreCommConfig();
    }
    
    if(!openDevice(param_.device_path_, CORE_COMM_BASE_ADDRESS, CORE_COMM_TOTAL_BYTE_SIZE, device_))
    {
        return CORE_COMM_OPEN_INIT_DEVICE_FAILED;
    }  

    boardcast_ptr_ = device_.device_ptr;
    config_ptr_ = device_.device_ptr + CORE_COMM_BOARDCAST_BYTE_SIZE;
    comm_ptr_ = device_.device_ptr + (config_.config_data_.base_address - CORE_COMM_BASE_ADDRESS);

    // init memory ptr
    initCoreCommConfigMemeoryPtr(&config_.config_data_, comm_ptr_);
    
    // clear master boardcast and slave ack event area    
    BoardcastCommData_t boardcast_data;
    boardcast_data.data = 0;
    setBoardcast(boardcast_ptr_, &boardcast_data);

    size_t number;
    CpuAckBlockData_t* cpu_ack_block_data_ptr = getCpuAckBlockDataByTo(&config_.config_data_, param_.cpu_id_, &number);
    CpuAckCommData_t cpu_ack_data;
    cpu_ack_data.data = 0;    
    for(size_t i = 0; i < number; ++i)
    {
        setCpuAck(&cpu_ack_block_data_ptr[i], &cpu_ack_data);
    }
    if(cpu_ack_block_data_ptr != NULL)
    {
        free(cpu_ack_block_data_ptr);
    }
    
    return SUCCESS;
}

ErrorCode CoreCommSystem::bootAsMaster()
{
    assert(boardcast_ptr_ != NULL);
    assert(config_ptr_ != NULL);
    assert(comm_ptr_ != NULL);
    assert(from_block_ptr_ == NULL);
    assert(to_block_ptr_ == NULL);
    
    // init local channel
    initLocalChannel(&config_.config_data_, param_.cpu_id_, 
                     &from_block_ptr_, &from_block_number_,
                     &to_block_ptr_, &to_block_number_);
    
    // boardcast configuration to all slaves
    memcpy(config_ptr_, &config_.config_data_, sizeof(CoreCommConfig_t));
    BoardcastCommData_t boardcast_data;
    boardcast_data.data = 1;    
    setBoardcast(boardcast_ptr_, &boardcast_data);        
    return SUCCESS;
}

bool CoreCommSystem::isAllSlaveBooted()
{
    bool ret = true;
    size_t number;
    CpuAckBlockData_t* cpu_ack_block_data_ptr = getCpuAckBlockDataByTo(&config_.config_data_, param_.cpu_id_, &number);
    CpuAckCommData_t cpu_ack_data;
    for(size_t i = 0; i < number; ++i)
    {
        if(!getCpuAck(&cpu_ack_block_data_ptr[i], &cpu_ack_data)
            || cpu_ack_data.data != 1)
        {
            ret = false;
            break;
        }
    }
    
    if(cpu_ack_block_data_ptr != NULL)
    {
        free(cpu_ack_block_data_ptr);
    }
    return ret;
}

ErrorCode CoreCommSystem::initAsSlave()
{
    if(!param_.loadParamSlave())
    {
        return CORE_COMM_LOAD_PARAM_FAILED;
    }

    if(!openDevice(param_.device_path_, CORE_COMM_BASE_ADDRESS, CORE_COMM_TOTAL_BYTE_SIZE, device_))
    {
        return CORE_COMM_OPEN_INIT_DEVICE_FAILED;
    }

    boardcast_ptr_ = device_.device_ptr;
    config_ptr_ = device_.device_ptr + CORE_COMM_BOARDCAST_BYTE_SIZE;
    comm_ptr_ = NULL;

    return SUCCESS;
}

bool CoreCommSystem::isMasterBooted()
{
    BoardcastCommData_t boardcast_data;
    if(!getBoardcast(boardcast_ptr_, &boardcast_data)
        || boardcast_data.data != 1)
    {
        return false;
    }
    else
    {
        return true;
    }
}

ErrorCode CoreCommSystem::bootAsSlave()
{
    assert(boardcast_ptr_ != NULL);
    assert(config_ptr_ != NULL);
    assert(comm_ptr_ == NULL);
    assert(from_block_ptr_ == NULL);
    assert(to_block_ptr_ == NULL);

    memcpy(&config_.config_data_, config_ptr_, sizeof(CoreCommConfig_t));    
    //config_.printCoreCommConfig();
    comm_ptr_ = device_.device_ptr + (config_.config_data_.base_address - CORE_COMM_BASE_ADDRESS);



    // init memory ptr
    initCoreCommConfigMemeoryPtr(&config_.config_data_, comm_ptr_);

    // init local channel
    initLocalChannel(&config_.config_data_, param_.cpu_id_, 
                     &from_block_ptr_, &from_block_number_,
                     &to_block_ptr_, &to_block_number_);

    // dealing with CoreCommConfig_t
    size_t number;
    CpuAckBlockData_t* cpu_ack_block_data_ptr = getCpuAckBlockDataByFrom(&config_.config_data_, param_.cpu_id_, &number); 
    if(number != 1)
    {
        if(cpu_ack_block_data_ptr != NULL)
        {
            free(cpu_ack_block_data_ptr);
        }
        return CORE_COMM_SLAVE_EVENT_CONFIG_INVALID;
    }
    CpuAckCommData_t cpu_ack_data;
    cpu_ack_data.data = 1;
    setCpuAck(&cpu_ack_block_data_ptr[0], &cpu_ack_data);

    if(cpu_ack_block_data_ptr != NULL)
    {
        free(cpu_ack_block_data_ptr);
    }
    return SUCCESS;
}

CommBlockData_t* CoreCommSystem::getFromCommBlockDataPtrList(size_t& from_block_number)
{
    from_block_number = from_block_number_;
    return from_block_ptr_;
}

CommBlockData_t* CoreCommSystem::getToCommBlockDataPtrList(size_t& to_block_number)
{
    to_block_number = to_block_number_;
    return to_block_ptr_;
}

bool CoreCommSystem::openDevice(std::string device_path, uint32_t base_address, size_t byte_size, Device_t& device)
{
    printf("CoreCommSystem::openDevice:%s, addr=0x%x, len=0x%x\n",device_path.c_str(),base_address,byte_size);
    int device_fd = open(device_path.c_str(), O_RDWR|O_SYNC);
    if(device_fd < 0) return false;
    device.device_ptr = (char*)mmap(NULL, 0x10000000, PROT_READ|PROT_WRITE, MAP_SHARED, device_fd, 0x30000000);
    if (device.device_ptr == MAP_FAILED) 
    {
        close(device_fd);
        device.device_ptr = NULL;
        device.base_address = 0;
        device.byte_size = 0;        
        return false;
    }
    else
    {
        close(device_fd);
        device.base_address = base_address;
        device.byte_size = byte_size;
        return true;
    }
}

void CoreCommSystem::closeDevice(Device_t& device)
{
    if(device.device_ptr != NULL)
    {
        munmap(device.device_ptr, device.byte_size);
        device.device_ptr = NULL;
        device.base_address = 0;
        device.byte_size = 0;
    }
}


