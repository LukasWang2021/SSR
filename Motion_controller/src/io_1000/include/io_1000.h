#ifndef IO_1000_H
#define IO_1000_H

/**
 * @file io_1000.h
 * @brief The file is the header file of class "Io1000".
 * @author Feng.Wu
 */

#include <mutex>
#include "common_datatype.h"
#include "common_error_code.h"
#include "log_manager_producer.h"
#include "base_device.h"
#include "io_1000_datatype.h"


/**
 * @brief io_space includes all io related definitions and implementation.
 */
namespace hal_space {

/**
 * @brief Defines the share memory for io communication.
 */

typedef struct
{
    uint32_t id;
    uint32_t do_read;
    uint32_t do_set;
    uint32_t do_reset;
    uint32_t do2_read;
    uint32_t do2_set;
    uint32_t do2_reset;
    uint32_t reseved[3];
    uint32_t di_read;
    uint32_t di2_read;
}IOReg_t;

/**
 * @brief Io1000 is a device to update the IO values.
 * @details 
 */
class Io1000: public BaseDevice{
  public:
    /**
     * @brief Constructor of the class. 
     */    
    Io1000(void);
    /**
     * @brief Destructor of the class. 
     */    
    ~Io1000(void);

    /**
     * @brief Initialization.
     * @details Open io device.\n
     * @retval true success.
     * @retval false Failed to initialize.
     */
    virtual bool init(bool is_real);

    /**
     * @brief Write down a DO bit.
     * @details 
     * @param [in] offset The offset from the first bit.Between [0, 63]
     * @param [in] value High:1, Low:0.
     * @return  ErrorCode
     */
    ErrorCode writeDoBit(uint32_t offset, uint8_t value);
    /**
     * @brief Read a DO bit.
     * @details 
     * @param [in] offset The offset from the first bit.Between [0, 63]
     * @param [out] value High:1, Low:0.
     * @return  ErrorCode
     */
    ErrorCode readDiBit(uint32_t offset, uint8_t &value);
    /**
     * @brief Read a DO bit.
     * @details 
     * @param [in] offset The offset from the first bit.Between [0, 63]
     * @param [out] value High:1, Low:0.
     * @return  ErrorCode
     */
    ErrorCode readDoBit(uint32_t offset, uint8_t &value);

    /**
     * @brief Read 64-bits DI.
     * @details 
     * @param [out] value_lower The lower value
     * @param [out] value_upper the upper value.
     * @return  ErrorCode
     */
    ErrorCode readDiAll(uint32_t &value_lower, uint32_t &value_upper);
    /**
     * @brief Read 64-bits DO.
     * @details
     * @param [out] value_lower The lower value
     * @param [out] value_upper the upper value.
     * @return  ErrorCode
     */
    ErrorCode readDoAll(uint32_t &value_lower, uint32_t &value_upper);

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
    IOReg_t* io_ptr_;         /**< The pointer to the register of IO*/
    uint32_t* io_status_ptr_;
    bool is_real_;            /**< True indicates operating on the real device while false means no checking device.*/
    ErrorCode pre_err_io_;

    ErrorCode readStepperDiBit(uint32_t offset, uint8_t &value);
};
}
#endif
