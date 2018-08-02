#ifndef TOOL_MANAGER_H
#define TOOL_MANAGER_H


#include "tool_manager_param.h"
#include "common_log.h"

namespace fst_ctrl
{
class ToolManager
{
public:
    ToolManager();
    ~ToolManager();

private:
    ToolManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
 
};

}

#endif


