#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H


#include "base_device.h"
#include "device_manager_param.h"
#include "common_log.h"
#include "xml_help.h"

namespace fst_hal
{
class DeviceManager
{
public:
    DeviceManager();
    ~DeviceManager();

private:
    DeviceManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_base::XmlHelp xml_help_;
};

}

#endif


