#ifndef IO_SAFETY_DATATYPE_H
#define IO_SAFETY_DATATYPE_H

/**
 * @file io_safety_datatype.h
 * @brief The file includes the definitions of data structures relates to IoSafety.
 * @author wuym
 */

#define IO_SAFETY_BASE_ADDRESS         0xA00C0000  /**< The base address of the share memory for io communication.*/
#define IO_SAFETY_TOTAL_BYTE_SIZE      0x8         /**< The total byte size for io communication.*/
#define IO_SAFETY_MAX_SIZE             63          /**< The maximum number of DI.*/
#define IO_SAFETY_DEVICE_PATH          "/dev/mem"
#define IO_SAFETY_STATUS_OFFSET        0x28

typedef struct
{
    uint32_t break1 : 1;
    uint32_t break2 : 1;
    uint32_t break3 : 1;
    uint32_t relay1 : 1;
    uint32_t relay2 : 1;
    uint32_t soft_reset : 1;
    uint32_t cabinet_reset : 1;
    uint32_t user_reset : 1;

    uint32_t mcu1_crc_err : 1;
    uint32_t mcu1_rec_timeout : 1;
    uint32_t mcu1_err_stop : 1;
    uint32_t mcu1_deadman_stop : 1;
    uint32_t mcu1_ext_stop : 1;
    uint32_t mcu1_tp_stop : 1;
    uint32_t mcu1_cabinet_stop : 1;
    uint32_t reserved1 : 1;

    uint32_t mcu2_crc_err : 1;
    uint32_t mcu2_rec_timeout : 1;
    uint32_t mcu2_err_stop : 1;
    uint32_t mcu2_deadman_stop : 1;
    uint32_t mcu2_ext_stop : 1;
    uint32_t mcu2_tp_stop : 1;
    uint32_t mcu2_cabinet_stop : 1;
    uint32_t reserved2 : 1;

    uint32_t mcu3_main_crc_err : 1;
    uint32_t mcu3_main_timeout : 1;
    uint32_t mcu3_mcu1_crc_err : 1;
    uint32_t mcu3_mcu1_timeout : 1;
    uint32_t mcu3_mcu2_crc_err : 1;
    uint32_t mcu3_mcu2_timeout : 1;
    uint32_t mcu3_relay_open_err : 1;
    uint32_t mcu3_relay_close_err : 1;

    uint32_t mcu3_open_break_relay_err : 1;
    uint32_t mcu3_close_break_relay_err : 1;
    uint32_t core0_err_stop : 1;
    uint32_t core1_err_stop : 1;
    uint32_t reserved3 : 4;

    uint32_t soft_reset_fdb : 1;
    uint32_t break_enable : 1;
    uint32_t busbar_enable : 1;
    uint32_t core0_err_stop_fdb : 1;
    uint32_t core1_err_stop_fdb : 1;
    uint32_t state_machine : 3;

    uint32_t reserved4 : 8;
    uint32_t reserved5 : 8;
}IoSafetyState_b;

typedef union
{
	uint64_t all;                          /**< Operated attribute by double word.*/
	IoSafetyState_b bit;                   /**< Operated attribute by bit.*/
}IoSafetyState_u;

#endif
