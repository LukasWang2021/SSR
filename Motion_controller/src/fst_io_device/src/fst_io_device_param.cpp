#include "fst_io_device_param.h"
#include "common_file_path.h"
#include <string>

using namespace fst_hal;

FstIoDeviceParam::FstIoDeviceParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(3),  // default is Error Level
    cycle_time_(0),
    input_byte_size_(0),
    output_byte_size_(0),
    is_virtual_(false)
{
    file_path_ += "fst_io_device.yaml";
}

FstIoDeviceParam::~FstIoDeviceParam()
{

}

bool FstIoDeviceParam::loadParam()
{    
    return true;
}

bool FstIoDeviceParam::saveParam()
{
    return true;
}

