#ifndef INTERPRETER_SYSMODEL_H
#define INTERPRETER_SYSMODEL_H

#include "common_datatype.h"
#include "base_device.h"
#include "system_model_manager.h"

bool InterpSysModel_Init(system_model_space::SystemModelManager *model_manager_ptr);

#ifdef __cplusplus
extern "C" {
#endif

ErrorCode InterpSysModel_SetParam(uint32_t param_part, uint32_t param_index, int32_t param_value);
ErrorCode InterpSysModel_GetParam(uint32_t param_part, uint32_t param_index, int32_t *param_value);
ErrorCode InterpSysModel_SaveParam(uint32_t param_part);

#ifdef __cplusplus
}
#endif

#endif

