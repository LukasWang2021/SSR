#include "interpreter_sysmodel.h"
#include "log_manager_producer.h"

using namespace log_space;
using namespace system_model_space;


static SystemModelManager *sys_model_ptr = NULL;

bool InterpSysModel_Init(SystemModelManager *model_manager_ptr)
{
    if (model_manager_ptr == NULL) return false;
    sys_model_ptr = model_manager_ptr;
    return true;
}

// TODO parameter part seprate
ErrorCode InterpSysModel_SetParam(uint32_t param_part, uint32_t param_index, int32_t param_value)
{
    ForceModel_t* force_model_ptr_ = sys_model_ptr->getForceModel(0);
    force_model_ptr_->force_param_ptr->set(param_index, param_value);
    return SUCCESS;
}

ErrorCode InterpSysModel_GetParam(uint32_t param_part, uint32_t param_index, int32_t *param_value)
{
    ForceModel_t* force_model_ptr_ = sys_model_ptr->getForceModel(0);
    force_model_ptr_->force_param_ptr->get(param_index, param_value);
    return SUCCESS;
}

ErrorCode InterpSysModel_SaveParam(uint32_t param_part)
{
    ForceModel_t* force_model_ptr_ = sys_model_ptr->getForceModel(0);
    force_model_ptr_->force_param_ptr->save();
    return SUCCESS;
}
