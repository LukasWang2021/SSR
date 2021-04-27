#ifndef COMMON_DATA_TYPE_H
#define COMMON_DATA_TYPE_H

/**
 * @file common_datatype.h
 * @brief The file includes data type definitions used in the whole system.
 * @author zhengyu.shen
 */

#include <stdint.h>
/**
 * @def AXIS_NUM 
 * The number of axes in the system.
 */
#define AXIS_NUM 14
/**
 * @def GROUP_NUM 
 * The number of groups in the system.
 */
#define GROUP_NUM 1

/**
 * @brief The attribute of a parameter defined by bit.
 */
typedef struct
{
    uint32_t read:1;    /**< Read flag, 0 is not readable; 1 is readable.*/
    uint32_t write:1;   /**< Write flag, 0 is not writeable; 1 is writeable.*/
    uint32_t rsvd:30;   /**< Reserved.*/ 
}ParamAttr_b;

/**
 * @brief The attribute of a parameter defined by union.
 */
typedef union
{
    int32_t all;        /**< Operated attribute by word.*/
    ParamAttr_b bit;    /**< Operated attribute by bit.*/
}ParamAttr_u;

/**
 * @brief The validity of a parameter defined by bit.
 */
typedef struct
{
    uint32_t active_level:2;    /**< Define the way to make a parameter take effect, 0 is NULL; 1 is power-on again; 2 is reset; 3 is immediate.*/ 
    uint32_t rsvd:30;           /**< Reserved.*/ 
}ParamValidity_b;

/**
 * @brief The validity of a parameter defined by union.
 */
typedef union
{
    int32_t all;            /**< Operated validity by word.*/
    ParamValidity_b bit;    /**< Operated validity by bit.*/
}ParamValidity_u;

/**
 * @brief The struct to define a parameter.
 * @attention If upper_limit_value and lower_limit_value are all zero, it means no limitation exists.
 */
typedef struct
{
    int32_t operation_value;    /**< The operation value of the parameter.*/
    int32_t default_value;      /**< The default value of the parameter.*/
    int32_t upper_limit_value;  /**< The upper limit value of the parameter.*/
    int32_t lower_limit_value;  /**< The lower limit value of the parameter.*/
    ParamAttr_u attr;           /**< The attribute of the parameter.*/
    ParamValidity_u validity;   /**< The validity of the parameter.*/
    char unit[16];              /**< The unit of the parameter.*/
}ParamDetail_t;

typedef struct
{
    char* device_ptr;       /**< File descriptor of share memory device.*/
    int32_t base_address;     /**< Base Address of the share memory.*/
    uint32_t byte_size;       /**< Byte size of the share memory.*/
}Device_t;

typedef enum
{
    AXIS_PP_X = 0,       /**<Rotation of PP Arm.*/
    AXIS_BIN_T = 1,      /**<Rotation of the wafer table.*/
    AXIS_WAFER_X = 2,    /**<X-axis of the wafer table.*/
    AXIS_WAFER_Y = 3,    /**<Y-axis of the wafer table.*/
    AXIS_BIN_X = 4,      /**<X-axis of the bin table.*/
    AXIS_BIN_Y = 5,      /**<Y-axis of the bin table.*/    
    AXIS_LOADER_X = 6,   /**<X-axis of the loader.*/
    AXIS_PP_Z1 = 7,      /**<the wafer-side cap*/
    AXIS_PP_Z2 = 8,      /**<the bin-side cap*/
    AXIS_NEEDLE = 9,     /**<the needle*/
    AXIS_PP_Y = 10,      /**<Y-axis of the wafer island*/
    AXIS_LOADER_Z1 = 11, /**<Left clamp of the loader*/ 
    AXIS_LOADER_Z2 = 12, /**<Right clamp of the loader*/
    AXIS_LOADER_Y = 13,  /**<Y-axis of the loader*/
}AxisNo_e;

#endif

