#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <string.h> 
#include <climits>
#include <map>
#include "io_1000.h"

using namespace log_space;
using namespace hal_space;
using namespace std;
uint8_t DI_bits[255]={0};//xzc--20210928
uint8_t DO_bits[255]={0};

Io1000::Io1000(void):
    BaseDevice(hal_space::DEVICE_TYPE_DIO),
    is_real_(false),
    pre_err_io_(SUCCESS)
{
}

Io1000::~Io1000(void)
{

}

bool Io1000::init(bool is_real)
{

    return true;
}

ErrorCode Io1000::writeDoBit(uint32_t offset, uint8_t value)
{ 
    DO_bits[offset] = value;
    //printf("### writeDoBit[%d]=%d,DO_bits[%d]=%d ",offset,value, offset, DO_bits[offset]); 
    return SUCCESS;
}
ErrorCode Io1000::readDoBit(uint32_t offset, uint8_t &value)
{
    value = DO_bits[offset];
    //printf("### readDoBit[%d]=%d ",offset,value);
    return SUCCESS;
}
ErrorCode Io1000::readDiBit(uint32_t offset, uint8_t &value)
{
    value = DI_bits[offset];
    //printf("### readDiBit[%d]=%d ",offset,value);
    return SUCCESS;
}



ErrorCode Io1000::readDiAll(uint32_t &value_lower, uint32_t &value_upper)
{

    return SUCCESS;
}

ErrorCode Io1000::readDoAll(uint32_t &value_lower, uint32_t &value_upper)
{

    return SUCCESS;
}

ErrorCode Io1000::readStepperDiBit(uint32_t offset, uint8_t &value)
{
    value = 0;
    return SUCCESS;
}

ErrorCode Io1000::updateStatus(void)
{

    return SUCCESS;
}

bool Io1000::openDevice(std::string device_path, uint32_t base_address, size_t byte_size, Device_t& device)
{
    int device_fd = open(device_path.c_str(), O_RDWR);
    device.device_ptr = (char *)mmap(NULL, byte_size, PROT_READ|PROT_WRITE, MAP_SHARED, device_fd, base_address);
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

void Io1000::closeDevice(Device_t& device)
{
    if(device.device_ptr != NULL)
    {
        munmap(device.device_ptr, device.byte_size);
        device.device_ptr = NULL;
        device.base_address = 0;
        device.byte_size = 0;
    }
}


