#ifndef DEVICE_VERSION_H
#define DEVICE_VERSION_H

/**
 * @file device_version.h
 * @brief The file is the header file of class "DeviceVersion".
 * @author Feng.Wu
 */

#include <map>

/**
 * @brief user_space includes the user level implementation.
 */
namespace user_space
{

/**
 * @brief DeviceVersion gets the version number of the controller.
 * @details 
 */
class DeviceVersion
{
public:
    /**
     * @brief Constructor of the class.
     */
    DeviceVersion();
    /**
     * @brief Destructor of the class. 
     */  
    ~DeviceVersion();

    /**
     * @brief Initialization.
     * @details
     * @return void.
     */
    void init(void);

    /**
     * @brief Get the major version of the controller.
     * @details
     * @return Major Version
     */
    int getControllerMajorVersion(void);

    /**
     * @brief Get the minor version of the controller.
     * @details
     * @return Minor Version
     */
    int getControllerMinorVersion(void);

    /**
     * @brief Get the string of the controller version.
     * @details
     * @return The string of the full version
     */
    std::string getControllerVersion(void);

    /**
     * @brief Get the version strings of all the devices.
     * @details
     * @return A list of the device name and version string.
     */
    std::map<std::string, std::string> getDeviceVersionList(void);
    

private:

};

}


#endif

