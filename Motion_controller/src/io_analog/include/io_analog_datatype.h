#ifndef IO_ANALOG_DATATYPE_H
#define IO_ANALOG_DATATYPE_H

/**
 * @file io_analog_datatype.h
 * @brief The file includes the definitions of data structures relates to IoAnalog.
 * @author Feng.Wu
 */

#define IO_AIO_BASE_ADDRESS      0x400E0000//0x400E0000|0x3D100000  /**< The base address of the share memory for io communication.*/
#define IO_AIO_TOTAL_BYTE_SIZE   0x1000      /**< The total byte size for io communication.*/
#define IO_AIO_MAX_SIZE          14          /**< The maximum number of AIO.*/
#define IO_AIO_REG_OFFSET        0x0
#define IO_AIO_MAP_READ_OFFSET   0x100

typedef struct
{
    uint32_t connection:1;
    uint32_t lost_connection:1;
    uint32_t line_error:1;
    uint32_t crc_error:1;
    uint32_t bad_frame_error:1;
    uint32_t reserved1:27;
}IOAnalogStateReg_t;

typedef struct
{
    uint32_t aio_val:16;
    uint32_t addr:8;
    uint32_t board_id:4;
    uint32_t reserved:2;
    uint32_t is_write:1;
    uint32_t op_start:1;
}IOAnalogCmdReg_t;

// typedef struct
// {
//     int32_t reserved1:16;
//     int32_t map_period:8;
//     int32_t board_id:4;
//     int32_t reserved2:3;
//     int32_t map_en:1;
// }IOAnalogRmapCtrlReg_t;

typedef struct
{
    uint32_t id;
    IOAnalogStateReg_t state;
    uint32_t ir_mask;
	IOAnalogCmdReg_t cmd;
	uint32_t r_map_ctrl;
}IOAnalogReg_t;

typedef struct
{
    uint16_t aio[IO_AIO_MAX_SIZE];
}IOAnalogMapReadReg_t;


#endif
