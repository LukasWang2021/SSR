#include "system/servo_cpu_comm_base.h"
#include <assert.h>
#include <unistd.h>

using namespace servo_comm_space;


ServoCpuCommBase::ServoCpuCommBase(int32_t controller_id, int32_t servo_id)
{
    comm_ptr_ = createServoCpuCommByController(controller_id, servo_id);
}

ServoCpuCommBase::~ServoCpuCommBase()
{
    freeServoCpuComm(comm_ptr_);
}

bool ServoCpuCommBase::init(CommBlockData_t* from_block_ptr, size_t from_block_number, CommBlockData_t* to_block_ptr, size_t to_block_number)
{
    return initServoCpuCommByController(comm_ptr_, from_block_ptr, from_block_number, to_block_ptr, to_block_number); 
}

uint32_t ServoCpuCommBase::getMajorVersion()
{
    return getServoCpuCommMajorVersion(comm_ptr_);
}

uint32_t ServoCpuCommBase::getMinorVersion()
{
    return getServoCpuCommMinorVersion(comm_ptr_);
}

void ServoCpuCommBase::setCtrlPdoSync(uint32_t index, uint32_t ctrl_pdo_sync)
{
    setServoCpuCommCtrlPdoSync(comm_ptr_, index, ctrl_pdo_sync);    
}

uint32_t ServoCpuCommBase::getCtrlPdoSync(uint32_t index)
{
    return getServoCpuCommCtrlPdoSync(comm_ptr_, index);
}

void ServoCpuCommBase::setSamplingSync(uint32_t sampling_sync)
{
    setServoCpuCommSamplingSync(comm_ptr_, sampling_sync);
}

uint32_t ServoCpuCommBase::getSamplingSync()
{
    return getServoCpuCommSamplingSync(comm_ptr_);
}

void ServoCpuCommBase::enableSamplingCfg(uint32_t pulse_width)
{
    setServoCpuCommSamplingCfg(comm_ptr_, 1);
    usleep(pulse_width);
    setServoCpuCommSamplingCfg(comm_ptr_, 0);
}

void ServoCpuCommBase::setSamplingInterval(uint32_t sampling_interval)
{
    setServoCpuCommSamplingInterval(comm_ptr_, sampling_interval);
}

uint32_t ServoCpuCommBase::getSamplingInterval()
{
    return getServoCpuCommSamplingInterval(comm_ptr_);
}

void ServoCpuCommBase::setSamplingMaxTimes(uint32_t sampling_max_times)
{
    setServoCpuCommSamplingMaxTimes(comm_ptr_, sampling_max_times);
}

uint32_t ServoCpuCommBase::getSamplingMaxTimes()
{
    return getServoCpuCommSamplingMaxTimes(comm_ptr_);
}

void ServoCpuCommBase::setSamplingChannel(uint32_t channel_index, uint32_t channel_value)
{
    setServoCpuCommSamplingChannel(comm_ptr_, channel_index, channel_value);
}

std::vector<uint32_t> ServoCpuCommBase::getSamplingChannel()
{
    std::vector<uint32_t> vec;
    uint32_t value = 0;
    for (size_t i = 0; i < COMM_REG1_SAMPLING_CHANNEL_NUMBER; ++i)
    {
        value = getServoCpuCommSamplingChannel(comm_ptr_, i);
        vec.push_back(value);
    }
    return vec;
}

void ServoCpuCommBase::getSamplingBufferData(uint8_t* data_ptr, int32_t* data_byte_size_ptr)
{
    getServoCpuCommSamplingBuffer(comm_ptr_, data_ptr, data_byte_size_ptr);    
}

void ServoCpuCommBase::getServoCpuCommInfo(ServoCpuCommInfo_t* info)
{
    assert(comm_ptr_ != NULL);
    if (comm_ptr_->comm_reg_ptr == NULL)
    {
        info->comm_reg_id = -1;
    }else
    {
        info->comm_reg_id = comm_ptr_->comm_reg_ptr->application_id;
    }
    
    if (comm_ptr_->sampling_buffer_ptr == NULL)
    {
        info->sampling_buffer_id = -1;
    }else
    {
        info->sampling_buffer_id = comm_ptr_->sampling_buffer_ptr->application_id;
    }
}

void ServoCpuCommBase::setServoControlMode(ServoControlMode control_mode)
{
    setServoCpuCommControlMode(comm_ptr_, control_mode);
}

uint32_t ServoCpuCommBase::getServoControlMode()
{
    return getServoCpuCommControlMode(comm_ptr_);
}

bool ServoCpuCommBase::setForceControlParameters(const CommRegForceControlParam_t* data_ptr)
{
    if(!setServoCpuCommForceControlParameters(comm_ptr_, (uint8_t*)data_ptr, sizeof(CommRegForceControlParam_t)))
        return false;

    uint32_t flag_value = 1;
    if(!setServoCpuCommForceControlUpdateFlag(comm_ptr_, flag_value)) 
        return false;

    return true;
}

bool ServoCpuCommBase::getForceControlParameters(CommRegForceControlParam_t* data_ptr)
{
    uint32_t data_size = 0;
    if(!getServoCpuCommForceControlParameters(comm_ptr_, (uint8_t*)data_ptr, &data_size))
        return false;
        
    if(data_size != sizeof(CommRegForceControlParam_t))
        return false;

    return true;
}


bool ServoCpuCommBase::setTorqueSensorSync(const uint32_t* data_ptr)
{
    uint32_t flag_value = 1;
    if(!setServoCpuCommTorqueSensorUpdateFlag(comm_ptr_, flag_value)) 
        return false;
    return true;
}
bool ServoCpuCommBase::getTorqueSensorSync(uint32_t * data_ptr)
{
    if(!getServoCpuCommTorqueSensorUpdateFlag(comm_ptr_, data_ptr)) 
        return false;
    return true;
}
bool ServoCpuCommBase::getTorqueSensorData(CommRegTorqueData_t * data_ptr)
{
    uint32_t data_size = 0;
    if(!getServoCpuCommTorqueSensorData(comm_ptr_, (uint8_t*)data_ptr, &data_size))
        return false;
    if(data_size != sizeof(CommRegTorqueData_t))
        return false;
    return true;
}
ServoCpuCommBase::ServoCpuCommBase():
    comm_ptr_(NULL)
{

}
    
