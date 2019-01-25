#ifndef DEVICE_VERSION_H
#define DEVICE_VERSION_H


#include "parameter_manager/parameter_manager_param_group.h"
#include "motion_control.h"
#include "io_manager.h"
#include "fst_safety_device.h"

namespace fst_ctrl
{
class DeviceVersion
{
public:
    DeviceVersion();
    ~DeviceVersion();

    void init(fst_log::Logger* log_ptr, fst_mc::MotionControl* motion_control_ptr,
        fst_hal::IoManager* io_manager_ptr, fst_hal::FstSafetyDevice* safety_device_ptr);
    
    std::map<std::string, std::string> getDeviceVersionList(void);

    //servo control bin
    void getServoControlVersion(std::string &name, std::string &version);

    //hand off version
    void getHandOffVersion(std::string &name, std::string &version);

    //rbf version
    void getRbfVersion(std::string &name, std::string &version);

    //servo param version
    void getServoParamVersion(std::string &name, std::string &version);

    //safety board software version
    void getSafetyBoardVersion(std::string &name, std::string &version);

    //io board software version
    std::map<std::string, std::string> getIoBoardVersion(void);
    

private:
    fst_parameter::ParamGroup yaml_help_;
    fst_log::Logger* log_ptr_;
    std::string file_path_;
    
    fst_mc::MotionControl* motion_control_ptr_;
    fst_hal::FstSafetyDevice* safety_device_ptr_;
    fst_hal::IoManager* io_manager_ptr_;

    int *hand_off_ptr_;                            //for version
    int *rbf_ptr_;                                 //for version
    static const int HAND_OFF_ADDR   = 0x38110000; //for version
    static const int RBF_ADDR        = 0xC0010000; //for version
    static const int HAND_OFF_LENGTH = 0x04;       //for version
    static const int RBF_LENGTH      = 0x04;       //for version

};

}


#endif

