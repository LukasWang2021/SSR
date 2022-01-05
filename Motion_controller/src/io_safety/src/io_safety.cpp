#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <string.h> 
#include <climits>
#include <map>
#include "io_safety.h"

using namespace log_space;
using namespace hal_space;
using namespace std;

IoSafety::IoSafety(void):
    BaseDevice(hal_space::DEVICE_TYPE_SAFETY),
    is_real_(true),
    pre_err_io_(SUCCESS)
{
}

IoSafety::~IoSafety(void)
{

}

bool IoSafety::init(bool is_real)
{
    device_ = openDevice(IO_SAFETY_DEVICE_PATH, IO_SAFETY_BASE_ADDRESS, IO_SAFETY_TOTAL_BYTE_SIZE);

    if(device_ == NULL)
        return false;

    return true;
}

ErrorCode IoSafety::readStatusBit(uint32_t offset, uint8_t &value)
{
    if(offset < 0 || offset > IO_SAFETY_MAX_SIZE)
        return IO_SAFETY_INVALID_OFFSET;
    
    updateStatus();
    value = (uint8_t)((state_.all >> offset) & 0x1);

    return SUCCESS;
}

ErrorCode IoSafety::writeStatusBit(uint32_t offset, uint8_t value)
{
    return SUCCESS;
}

ErrorCode IoSafety::readStatusAll(uint32_t &value_lower, uint32_t &value_upper)
{
    updateStatus();
    value_lower = (uint32_t)state_.all;
    value_upper = (uint32_t)(state_.all >> 32);
    return SUCCESS;
}

ErrorCode IoSafety::updateStatus(void)
{
    state_.all = *(uint64_t *)(device_->device_ptr);
    return SUCCESS;
}


