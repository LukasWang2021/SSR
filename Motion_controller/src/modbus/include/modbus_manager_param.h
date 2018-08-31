#ifndef MODBUS_MANAGER_PARAM
#define MODBUS_MANAGER_PARAM

#include "parameter_manager/parameter_manager_param_group.h"
#include <string>
using namespace std;

namespace fst_modbus
{
class ModbusManagerParam
{
public:
    ModbusManagerParam();
    ~ModbusManagerParam(){}

    bool loadParam();
    bool saveParam();
    
    // param to load & save
    int log_level_;

private:
};
}

#endif


