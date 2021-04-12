#ifndef BOARDCAST_DATATYPE_H
#define BOARDCAST_DATATYPE_H

/**
 * @file boardcast_datatype.h
 * @brief The file includes the definition of the data type for boardcast channel.
 * @author zhengyu.shen
 */

#include <stdint.h>

/**
 * @brief Defines the data flowing on boardcast channel.
 */
typedef struct
{
    uint8_t data;   /**< The value keeps 0 in the initialization phase and set to 1 after the master is ready to wait slaves acknowledgements.*/
}BoardcastCommData_t;


#endif
