#include "device_version.h"
#include "controller_version.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>
#include <sys/mman.h>

using namespace fst_ctrl;

#define SERVO_PARAM_FILE_DIR "/root/install/share/configuration/machine/"

DeviceVersion::DeviceVersion():
    log_ptr_(NULL),
    file_path_(SERVO_PARAM_FILE_DIR),
    io_manager_ptr_(NULL),
    safety_device_ptr_(NULL),
    hand_off_ptr_(NULL),
    rbf_ptr_(NULL)   
{
    file_path_ += "servo_param.yaml";
}

DeviceVersion::~DeviceVersion()
{
    if(hand_off_ptr_ != NULL && hand_off_ptr_ != -1)
    {
        munmap(hand_off_ptr_, HAND_OFF_LENGTH);
    }
    if(rbf_ptr_ != NULL && rbf_ptr_ != -1)
    {
        munmap(rbf_ptr_, RBF_LENGTH);
    }

}

void DeviceVersion::init(fst_log::Logger* log_ptr, fst_mc::MotionControl* motion_control_ptr,
    fst_hal::IoManager* io_manager_ptr, fst_hal::FstSafetyDevice* safety_device_ptr)
{
    log_ptr_ = log_ptr;
    motion_control_ptr_ = motion_control_ptr;
    io_manager_ptr_ = io_manager_ptr;
    safety_device_ptr_ = safety_device_ptr;

    // for hand_off version address
    int hand_off_fd = open("/dev/mem", O_RDWR);
    if (hand_off_fd == -1)
    {
        FST_WARN("Failed to open /dev/mem for hand_off_version\n");
    }
    else
    {
        hand_off_ptr_ = (int *) mmap(NULL, HAND_OFF_LENGTH, PROT_READ|PROT_WRITE, MAP_SHARED, hand_off_fd, HAND_OFF_ADDR);
        close(hand_off_fd);
        printf("handoff ptr=%p\n", hand_off_ptr_);
    }

    // for rbf version address
    int rbf_fd = open("/dev/mem", O_RDWR);
    if (rbf_fd == -1)
    {
        FST_WARN("Failed to open /dev/mem for rbf_version\n");
    }
    else
    {
        rbf_ptr_ = (int *) mmap(NULL, RBF_LENGTH, PROT_READ|PROT_WRITE, MAP_SHARED, rbf_fd, RBF_ADDR);
        close(rbf_fd);
        printf("rbf ptr=%p\n", rbf_ptr_);
    }
}

std::map<std::string, std::string> DeviceVersion::getDeviceVersionList(void)
{
    std::map<std::string, std::string> version_map;
    //std::pair<map<std::string, std::string>::iterator, bool> insert_pair;//insert success or failed
    std::string name, version;

    // get servo control software version
    getServoControlVersion(name, version);
    version_map.insert(std::pair<std::string, std::string>(name, version)); 

    // get hand off software version
    name.clear();version.clear();
    getHandOffVersion(name, version);
    version_map.insert(std::pair<std::string, std::string>(name, version)); 

    // get rbf version
    name.clear();version.clear();
    getRbfVersion(name, version);
    version_map.insert(std::pair<std::string, std::string>(name, version)); 

    // get servo parameter version
    name.clear();version.clear();
    getServoParamVersion(name, version);
    version_map.insert(std::pair<std::string, std::string>(name, version)); 

    // get safety board software version
    name.clear();version.clear();
    getSafetyBoardVersion(name, version);
    version_map.insert(std::pair<std::string, std::string>(name, version));

    // get io boards software version
    std::map<std::string, std::string> io_version_map = getIoBoardVersion();
    for (std::map<std::string, std::string>::iterator iter = io_version_map.begin();iter != io_version_map.end(); ++iter)
    {
        version_map.insert(std::pair<std::string, std::string>(iter->first, iter->second));
    }

    return version_map;
}


void DeviceVersion::getServoControlVersion(std::string &name, std::string &version)
{
    name = "Servo_Control";
    motion_control_ptr_->getServoVersion(version);
}

//hand off version
void DeviceVersion::getHandOffVersion(std::string &name, std::string &version)
{
    name = "Hand_Off";
    if (hand_off_ptr_ == NULL || hand_off_ptr_ == -1)
    {
        version = "NONE";
        return;
    }
    sprintf(version.c_str(), "%lx", *hand_off_ptr_);
}

//rbf version
void DeviceVersion::getRbfVersion(std::string &name, std::string &version)
{
    name = "RBF";
    if (rbf_ptr_ == NULL || rbf_ptr_ == -1)
    {
        version = "NONE";
        return;
    }
    sprintf(version.c_str(), "%lx", *rbf_ptr_);
}

void DeviceVersion::getServoParamVersion(std::string &name, std::string &version)
{
    name = "Servo_Param";

    if (yaml_help_.loadParamFile(file_path_.c_str()))
    {
        if(!yaml_help_.getParam("servo.version", version))
        {
            FST_INFO("servo_param_vesrion: Failed to load param");
        }
    }
    else
    {
        FST_INFO("servo_param_vesrion: Failed to load file");
    }

}

void DeviceVersion::getSafetyBoardVersion(std::string &name, std::string &version)
{
    name = "Safety_Board";

    // version convert from int to string
    int local_version = 0;
    safety_device_ptr_->getSafetyBoardVersion(local_version);
    sprintf(version.c_str(), "%lx", local_version);
}

std::map<std::string, std::string> DeviceVersion::getIoBoardVersion(void)
{
    std::string board_name = "IO_Board_";
    std::string name, version;
    std::map<std::string, std::string> io_version_map;

    //version convert from int to string
    std::map<int, int> io_version_list;
    io_version_list = io_manager_ptr_->getIoBoardVersion();
    for (std::map<int, int>::iterator iter = io_version_list.begin();iter != io_version_list.end(); ++iter)
    {
        name = board_name + to_string(iter->first);
        sprintf(version.c_str(), "%lx", iter->second);
        io_version_map.insert(pair<std::string, std::string>(name, version));
    }
    return io_version_map;
}

