#ifndef INTERPRETER_DEVICE_H
#define INTERPRETER_DEVICE_H

#include "base_device.h"
#include "common_datatype.h"


bool InterpDevice_Init(hal_space::BaseDevice *dev);//  hal_space::Io1000* io_digital_dev_ptr_;

#ifdef __cplusplus
extern "C" {
#endif

ErrorCode InterpDevice_GetDIBit(uint32_t offset, uint8_t &value);
ErrorCode InterpDevice_GetDOBit(uint32_t offset, uint8_t &value);
ErrorCode InterpDevice_SetDOBit(uint32_t offset, uint8_t value);
ErrorCode InterpDevice_GetForceValue(uint32_t id, double value[6]);

#ifdef __cplusplus
}
#endif

#endif

