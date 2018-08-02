#ifndef COORDINATE_MANAGER_H
#define COORDINATE_MANAGER_H


#include "coordinate_manager_param.h"
#include "common_log.h"

namespace fst_ctrl
{
class CoordinateManager
{
public:
    CoordinateManager();
    ~CoordinateManager();

private:
    CoordinateManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
 
};

}

#endif


