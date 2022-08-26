#ifndef INTERPRETER_DEVICE_H
#define INTERPRETER_DEVICE_H

#include "common_datatype.h"
#include "base_device.h"
#include "force_sensor.h"

bool InterpDevice_Init(std::vector<hal_space::BaseDevice *> io_ptr, sensors_space::ForceSensor *force_sn_ptr);

#ifdef __cplusplus
extern "C" {
#endif

ErrorCode InterpDevice_GetDIBit(uint32_t offset, uint8_t &value);
ErrorCode InterpDevice_GetDOBit(uint32_t offset, uint8_t &value);
ErrorCode InterpDevice_SetDOBit(uint32_t offset, uint8_t value);
ErrorCode InterpDevice_GetForceRawValue(uint32_t id, double value[6]);
ErrorCode InterpDevice_GetForceCalibValue(uint32_t id, double value[6]);
ErrorCode InterpDevice_FioControl(uint32_t cmd_type, uint32_t cmd_value, uint32_t *cmd_result);

#ifdef __cplusplus
}
#endif

#endif

