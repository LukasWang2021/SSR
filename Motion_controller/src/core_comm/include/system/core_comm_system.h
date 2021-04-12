#ifndef CORE_COMM_SYSTEM_H
#define CORE_COMM_SYSTEM_H

/**
 * @file core_comm_system.h
 * @brief The file includes the class for establishing the inter core communication.
 * @author zhengyu.shen
 */

#include "common_error_code.h"
#include "system/core_comm_config.h"
#include "system/core_comm_param.h"
#include "common/core_comm_datatype.h"
#include <sys/mman.h>

/**
 * @brief core_comm_space includes all inter core communication related implementation.
 */
namespace core_comm_space
{
/**
 * @brief Defines the share memory for inter core communication.
 */
typedef struct
{
    char* device_ptr;       /**< File descriptor of share memory device.*/
    off_t base_address;     /**< Base Address of the share memory.*/
    size_t byte_size;       /**< Byte size of the share memory.*/
}Device_t;

/**
 * @brief CoreCommSystem is the object to handle the establishment of the inter core communication.
 */
class CoreCommSystem
{
public:
    /**
     * @brief Constructor of the class.
     */     
    CoreCommSystem();
    /**
     * @brief Destructor of the class.
     */     
    ~CoreCommSystem();
    /**
     * @brief Initialize the local data for establishing the inter core communication.
     * @details The Caller must be the master of the inter core communication.
     * @retval CORE_COMM_LOAD_PARAM_FAILED Failed to load the configuration file of the module.
     * @retval CORE_COMM_LOAD_CORE_COMM_CONFIG_FAILED Failed to load the configuration file of the inter core communication.
     * @retval CORE_COMM_OPEN_INIT_DEVICE_FAILED Failed to open the share memory device.
     * @retval SUCCESS Operation is successful.
     */
    ErrorCode initAsMaster();
    /**
     * @brief Try to establish the inter core communication as master.
     * @details The Caller must be the master of the inter core communication.
     * @retval SUCCESS Operation is successful.
     */    
    ErrorCode bootAsMaster();
    /**
     * @brief Check if all expected slaves finish their booting phase.
     * @details The Caller must be the master of the inter core communication.\n
     *          It should be called after bootAsMaster().\n
     * @retval true All slaves are ok to communicate.
     * @retval false At least one slave is not ready to communicate.
     */        
    bool isAllSlaveBooted();
    /**
     * @brief Initialize the local data for establishing the inter core communication.
     * @details The Caller must be some slave of the inter core communication.\n
     * @retval CORE_COMM_LOAD_PARAM_FAILED Failed to load the configuration file of the module.
     * @retval CORE_COMM_OPEN_INIT_DEVICE_FAILED Failed to open the share memory device.
     * @retval SUCCESS Operation is successful.
     */    
    ErrorCode initAsSlave();
    /**
     * @brief Check if the master has made all configuration data ready for slaves to visit.
     * @details The Caller must be the some slave of the inter core communication.\n
     * @retval true Configuration data is ready to visit.
     * @retval false Configuration data is not ready to visit.
     */      
    bool isMasterBooted();
    /**
     * @brief Try to establish the inter core communication as slave.
     * @details The Caller must be some slave of the inter core communication.\n
     *          It should be called after isMasterBooted() returns true.\n
     * @retval CORE_COMM_SLAVE_EVENT_CONFIG_INVALID The configuration of the slave's cpu ack channel is wrong.
     * @retval SUCCESS Operation is successful.
     */      
    ErrorCode bootAsSlave();
    /**
     * @brief Get the list of all communication channels that sponsor from the local cpu id.
     * @param [out] from_block_number The number of communication channels.
     * @return Pointer of the communication channel list.
     */
    CommBlockData_t* getFromCommBlockDataPtrList(size_t& from_block_number);
    /**
     * @brief Get the list of all communication channels that end to the local cpu id.
     * @param [out] to_block_number The number of communication channels.
     * @return Pointer of the communication channel list.
     */    
    CommBlockData_t* getToCommBlockDataPtrList(size_t& to_block_number);
    
private:
    CoreCommParam param_;   /**< The object to handle the configuration file of the module.*/
    CoreCommConfig config_; /**< The object to handle the configuration file of the inter core communication.*/

    Device_t device_;       /**< Stores the information of the share memory.*/
    char* boardcast_ptr_;   /**< Point to the boardcasting area in the share memory.*/
    char* config_ptr_;      /**< Point to the area used to store the configuration of inter core communication in the share memory.*/
    char* comm_ptr_;        /**< Point to the area used to communication in the share memory.*/

    CommBlockData_t* from_block_ptr_;   /**< The list of all communication channels that sponsor from the local cpu id.*/
    size_t from_block_number_;          /**< The number of communication channels that sponsor from the local cpu id.*/
    CommBlockData_t* to_block_ptr_;     /**< The list of all communication channels that end to the local cpu id.*/
    size_t to_block_number_;            /**< The number of communication channels that end to the local cpu id.*/

    bool openDevice(std::string device_path, uint32_t base_address, size_t byte_size, Device_t& device);
    void closeDevice(Device_t& device);
};

}


#endif

