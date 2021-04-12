#include "device_version.h"
#include "controller_version.h"
#include <string>
#include <sys/mman.h>

using namespace user_space;

DeviceVersion::DeviceVersion() 
{
    
}

DeviceVersion::~DeviceVersion()
{

}

void DeviceVersion::init(void)
{

}

//controller version
int DeviceVersion::getControllerMajorVersion(void)
{
    return controller_VERSION_MAJOR;
}

int DeviceVersion::getControllerMinorVersion(void)
{
    return controller_VERSION_MINOR;
}

std::string DeviceVersion::getControllerVersion(void)
{
    char version_all[64] = "";

#ifndef controller_MAIN_VERSION
    sprintf(version_all, "%d.%d.%d.%s.%s", controller_VERSION_MAJOR,
            controller_VERSION_MINOR, controller_VERSION_PATCH,
            controller_BUILD_DATE,controller_VERSION_COMMIT);

#else 
    sprintf(version_all, "%d.%d", controller_VERSION_MAJOR,controller_VERSION_MINOR);

#endif

    std::string controller_version = version_all;
    printf("controller_version:%s\n", controller_version.c_str());
    return controller_version;
}


std::map<std::string, std::string> DeviceVersion::getDeviceVersionList(void)
{
    std::map<std::string, std::string> version_map;
    std::string name, version;


    return version_map;
}




