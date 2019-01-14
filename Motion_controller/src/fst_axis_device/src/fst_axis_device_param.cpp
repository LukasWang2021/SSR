#include "fst_axis_device_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_hal;

FstAxisDeviceParam::FstAxisDeviceParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(3),  // default is Error Level
    cycle_time_(0)
{
    file_path_ += "fst_axis_device.yaml";
}

FstAxisDeviceParam::~FstAxisDeviceParam()
{

}

bool FstAxisDeviceParam::loadParam()
{    
    return true;
}

bool FstAxisDeviceParam::saveParam()
{
    return true;
}

