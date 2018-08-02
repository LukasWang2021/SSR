#include "device_manager.h"
#include "fst_io_device.h"
#include "fst_safety_device.h"
#include "fst_axis_device.h"
#include "virtual_axis_device.h"


using namespace fst_hal;


DeviceManager::DeviceManager():
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new DeviceManagerParam();
}

DeviceManager::~DeviceManager()
{

}


