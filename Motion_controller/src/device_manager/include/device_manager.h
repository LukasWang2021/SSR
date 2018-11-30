#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H


#include "device_manager_param.h"
#include "common_log.h"
#include "device_xml.h"
#include "base_device.h"
#include <map>
#include <vector>
#include "base_datatype.h"

namespace fst_hal
{
typedef struct
{
    int index;
    int address;
    DeviceType type;
    bool is_valid;
}DeviceInfo;

class DeviceManager
{
public:
    DeviceManager();
    ~DeviceManager();

    ErrorCode init();
    ErrorCode setModbusDetail(BaseDevice* device_ptr, FstFstModbusConfigDetail detail);

    BaseDevice* getDevicePtrByDeviceIndex(int device_index);
    std::vector<DeviceInfo> getDeviceList();

private:
    DeviceManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    DeviceXml* device_xml_ptr_;
    std::map<int, BaseDevice*> device_map_;

    ErrorCode addDevice(int device_index, BaseDevice* device_ptr);
};

}

#endif


