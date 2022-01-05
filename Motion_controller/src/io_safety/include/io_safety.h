#ifndef IO_SAFETY_H
#define IO_SAFETY_H

/**
 * @file io_safety.h
 * @brief The file is the header file of class "IoSafety".
 * @author Feng.Wu
 */

#include <mutex>
#include "common_datatype.h"
#include "common_error_code.h"
#include "log_manager_producer.h"
#include "base_device.h"
#include "io_safety_datatype.h"


/**
 * @brief io_space includes all io related definitions and implementation.
 */
namespace hal_space {
/**
 * @brief IoSafety is a device to update the IO values.
 * @details 
 */
class IoSafety: public BaseDevice{
  public:
    /**
     * @brief Constructor of the class. 
     */    
    IoSafety(void);
    /**
     * @brief Destructor of the class. 
     */    
    ~IoSafety(void);

    /**
     * @brief Initialization.
     * @details Open io device.\n
     * @retval true success.
     * @retval false Failed to initialize.
     */
    virtual bool init(bool is_real);

    /**
     * @brief Read safety io by bit.
     * @details
     * @param [in] offset The offset from the first bit.Between [0, 63]
     * @param [out] value High:1, Low:0.
     * @return  ErrorCode
     */
    ErrorCode readStatusBit(uint32_t offset, uint8_t &value);

    /**
     * @brief Write safety io bi bit.
     * @details
     * @param [in] offset The offset from the first bit.Between [0, 63]
     * @param [in] value High:1, Low:0.
     * @return  ErrorCode
     */
    ErrorCode writeStatusBit(uint32_t offset, uint8_t value);

    /**
     * @brief Read 64-bits safety io feedback.
     * @details
     * @param [out] value_lower The lower value
     * @param [out] value_upper the upper value.
     * @return  ErrorCode
     */
    ErrorCode readStatusAll(uint32_t &value_lower, uint32_t &value_upper);
    
    /**
     * @brief Check if the status is ok or not.
     * @details
     * @retval  ErrorCode
     * @retval SUCCESS Normal status.
     * @retval IO_DEV_DISABLE The device is offline.
     * @retval IO_DATA_VIRIFY_FAILED Invalid data is received.
     */
    virtual ErrorCode updateStatus(void);

  private:    	
    Device_t *device_;         /**< Stores the information of the share memory.*/
    IoSafetyState_u state_;
    bool is_real_;            /**< True indicates operating on the real device while false means no checking device.*/
    ErrorCode pre_err_io_;
};
}
#endif
