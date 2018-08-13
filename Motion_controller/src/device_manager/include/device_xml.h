#ifndef DEVICE_XML_H
#define DEVICE_XML_H


#include "common_log.h"
#include "device_manager_param.h"
#include "base_device.h"
#include <string>
#include <vector>


namespace fst_hal
{
typedef struct
{
    std::string motor;
    std::string servo;
    std::string internal_encoder;
    std::string external_encoder;
    std::string chain;
}FstAxisConfigDetail;

typedef struct
{
    int dummy;
}FstIoConfigDetail;

typedef struct
{
    int dummy;
}FstSafetyConfigDetail;

typedef struct
{
    int dummy;
}FstFstAnybusConfigDetail;

typedef struct
{
    int dummy;
}NormalConfigDetail;

typedef struct
{
    int dummy;
}VirtualAxisConfigDetail;

typedef struct
{
    int dummy;
}VirtualIoConfigDetail;

typedef struct
{
    int dummy;
}VirtualSafetyConfigDetail;

typedef struct
{
    int device_index;
    int address;
    DeviceType device_type;
    /*union
    {
        FstAxisConfigDetail fst_axis;
        FstIoConfigDetail fst_io;
        FstSafetyConfigDetail fst_safety;
        FstFstAnybusConfigDetail any_bus;
        NormalConfigDetail normal;
        VirtualAxisConfigDetail virtual_axis;
        VirtualIoConfigDetail virtual_io;
        VirtualSafetyConfigDetail virtual_safety;
    }DeviceConfigDetail;*/    
}DeviceConfig;


class DeviceXml
{
public:
    DeviceXml(fst_log::Logger* log_ptr, DeviceManagerParam* param_ptr);
    ~DeviceXml();

    bool loadDeviceConfig();
    bool saveDeviceConfig();

    std::vector<DeviceConfig> device_config_;
private:
    fst_log::Logger* log_ptr_;
    DeviceManagerParam* param_ptr_;
    
    
    DeviceXml();

};

}


#endif

