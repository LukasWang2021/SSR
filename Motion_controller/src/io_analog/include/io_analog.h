#ifndef IO_ANALOG_H
#define IO_ANALOG_H

/**
 * @file io_analog.h
 * @brief The file is the header file of class "IoAnalog".
 * @author Feng.Wu
 */
#include "common_datatype.h"
#include "common_error_code.h"
#include "log_manager_producer.h"
#include "base_device.h"
#include "io_analog_datatype.h"


/**
 * @brief io_space includes all io related definitions and implementation.
 */
namespace hal_space {

/**
 * @brief Defines the share memory for io communication.
 */

/**
 * @brief Io1000 is a device to update the IO values.
 * @details 
 */
class IoAnalog: public BaseDevice {
  public:
    /**
     * @brief Constructor of the class. 
     */    
    IoAnalog(void);
    /**
     * @brief Destructor of the class. 
     */    
    ~IoAnalog(void);

    /**
     * @brief Initialization.
     * @details Open io device.\n
     * @retval true success.
     * @retval false Failed to initialize.
     */
    virtual bool init(bool is_real);

    /**
     * @brief write 32-bits AO.
     * @details 
     * @param [in] board_id The reference of the board.
     * @param [in] offset The offset from the first bit.
     * @param [in] value Ananlog value
     * @return  ErrorCode
     */
    ErrorCode writeAO(uint32_t board_id, uint32_t offset, int16_t value);

    /**
     * @brief Read 32-bits AI.
     * @details 
     * @param [in] board_id The reference of the board.
     * @param [in] offset The offset from the first bit.Between [0, 0]
     * @param [out] value Ananlog value
     * @return  ErrorCode
     */
    ErrorCode readAIO(uint32_t board_id, uint32_t offset, int16_t &value);

    /**
     * @brief Check if the status is ok or not.
     * @details
     * @retval  ErrorCode
     * @retval SUCCESS Normal status.
     * @retval IO_DEV_DISABLE The device is offline.
     * @retval IO_DATA_VIRIFY_FAILED Invalid data is received.
     */
    virtual ErrorCode updateStatus(void);

        /**
     * @brief Reset to clear error.
     * @details
     */
    void resetError(void);
    

  private:    	
    Device_t device_;                /**< Stores the information of the share memory.*/
    IOAnalogReg_t* reg_ptr_;         /**< The pointer to write the register of IO*/
    IOAnalogMapReadReg_t* read_ptr_; /**< The pointer to read the register of IO*/
    bool is_real_;                   /**< True indicates operating on the real device while false means no checking device.*/
    ErrorCode pre_err_;
    
    bool openDevice(std::string device_path, uint32_t base_address, size_t byte_size, Device_t& device);
    void closeDevice(Device_t& device);
    
};
}
#endif
