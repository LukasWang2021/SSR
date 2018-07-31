#include "fst_safety_device_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_hal;

FstSafetyDeviceParam::FstSafetyDeviceParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(3),  // default is Error Level
    cycle_time_(0),
    is_virtual_(false)
{
    file_path_ += "fst_safety_device.yaml";
}

FstSafetyDeviceParam::~FstSafetyDeviceParam()
{

}

bool FstSafetyDeviceParam::loadParam()
{    
    return true;
}

bool FstSafetyDeviceParam::saveParam()
{
    return true;
}

