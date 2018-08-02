#include "tool_manager.h"


using namespace fst_ctrl;


ToolManager::ToolManager():
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ToolManagerParam();
}

ToolManager::~ToolManager()
{

}


