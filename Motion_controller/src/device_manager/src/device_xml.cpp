#include "device_xml.h"

using namespace fst_hal;

DeviceXml::DeviceXml(fst_log::Logger* log_ptr, DeviceManagerParam* param_ptr):
    log_ptr_(log_ptr), param_ptr_(param_ptr)
{

}

DeviceXml::~DeviceXml()
{

}

bool DeviceXml::loadDeviceConfig()
{
    return false;
}

bool DeviceXml::saveDeviceConfig()
{
    return false;
}

DeviceXml::DeviceXml():
    log_ptr_(NULL), param_ptr_(NULL)
{

}

