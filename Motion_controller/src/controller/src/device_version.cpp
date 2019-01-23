#include "device_version.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>

using namespace fst_ctrl;

#define SERVO_PARAM_FILE_DIR "/root/install/share/configuration/machine/"

DeviceVersion::DeviceVersion():
    log_ptr_(NULL),
    file_path_(SERVO_PARAM_FILE_DIR),
    servo_param_version_(0),
    servo_param_sum_(0),
    io_manager_ptr_(NULL),
    safety_device_ptr_(NULL)   
{
    file_path_ += "servo_param.yaml";
}

DeviceVersion::~DeviceVersion()
{

}

void DeviceVersion::init(fst_log::Logger* log_ptr, fst_hal::IoManager* io_manager_ptr, fst_hal::FstSafetyDevice* safety_device_ptr)
{
    log_ptr_ = log_ptr;
    io_manager_ptr_ = io_manager_ptr;
    safety_device_ptr_ = safety_device_ptr;

    std::vector<int> servo_stored_param;
    if (yaml_help_.loadParamFile(file_path_.c_str()))
    {
        if(yaml_help_.getParam("servo.stored_param", servo_stored_param))
        {
            servo_param_version_ = servo_stored_param[SERVO_PARAM_VER_ADDR];
            FST_INFO("servo_param_vesrion: 0x%llx", servo_param_version_);
        }
        else
        {
            servo_param_version_ = 0;
            FST_INFO("servo_param_vesrion: Failed to load param");
        }
    }
    else
    {
        servo_param_version_ = 0;
        FST_INFO("servo_param_vesrion: Failed to load file");
    }
    printf("--servo_param_vesrion: 0x%lx\n", servo_param_version_);

    //only for test
    std::string servo_param_name;
    std::string servo_param_version;
    getServoParamVersion(&servo_param_name, &servo_param_version);
    printf("-----name:%s, vesrion: %s\n",servo_param_name.c_str(), servo_param_version.c_str());
}

ErrorCode DeviceVersion::getServoParamVersion(std::string *name, std::string *version)
{
    *name = "Servo_Param";

    char servo_param_version[32] = "";
    sprintf(servo_param_version, "%lx", servo_param_version_);
    *version = servo_param_version;

    return SUCCESS;
}