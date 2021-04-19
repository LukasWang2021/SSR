#ifndef IO_1000_DATATYPE_H
#define IO_1000_DATATYPE_H

/**
 * @file io_1000_datatype.h
 * @brief The file includes the definitions of data structures relates to Io1000.
 * @author Feng.Wu
 */

#define IO_BASE_ADDRESS         0x400D0000  /**< The base address of the share memory for io communication.*/
#define IO_TOTAL_BYTE_SIZE      0x1000      /**< The total byte size for io communication.*/
#define IO_DI_MAX_SIZE          63          /**< The maximum number of DI.*/
#define IO_DO_MAX_SIZE          40          /**< The maximum number of DO.*/
#define IO_STATUS_OFFSET        0x70

typedef struct
{
	uint32_t comm_err:1;                 /**< Enable flag, 0 is enable; 1 is disable.*/
    uint32_t rsvd:31;                    /**< Reserved.*/ 
}Io1000State_b;


typedef union
{
	uint32_t all;                        /**< Operated attribute by word.*/
	Io1000State_b bit;                   /**< Operated attribute by bit.*/
}Io1000State_u;

#endif
