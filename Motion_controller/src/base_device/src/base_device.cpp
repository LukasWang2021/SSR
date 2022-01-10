#include "base_device.h"
#include "stdio.h"
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h> 
#include <climits>
#include <map>

using namespace hal_space;

BaseDevice::BaseDevice(DeviceType type)
{
    type_ = type;
}

BaseDevice::~BaseDevice()
{}

DeviceType BaseDevice::getDeviceType()
{
    return type_;
}

BaseDevice::BaseDevice():
    type_(DEVICE_TYPE_INVALID)
{

}

Device_t* BaseDevice::openDevice(std::string device_path, uint32_t base_address, size_t byte_size)
{
    Device_t *device = new Device_t;

    int device_fd = open(device_path.c_str(), O_RDWR);
    
    device->device_ptr = (char *)mmap(NULL, byte_size, PROT_READ|PROT_WRITE, MAP_SHARED, device_fd, base_address);
    if (device->device_ptr == MAP_FAILED) 
    {
        close(device_fd);
        device->device_ptr = NULL;
        device->base_address = 0;
        device->byte_size = 0;        
        return NULL;
    }
    else
    {
        close(device_fd);
        device->base_address = base_address;
        device->byte_size = byte_size;
        return device;
    }
}


void BaseDevice::closeDevice(Device_t* device)
{
    if(device->device_ptr != NULL)
    {
        munmap(device->device_ptr, device->byte_size);
        device->device_ptr = NULL;
        device->base_address = 0;
        device->byte_size = 0;
    }
}


