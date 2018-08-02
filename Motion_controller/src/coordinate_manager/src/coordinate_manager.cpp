#include "coordinate_manager.h"


using namespace fst_ctrl;


CoordinateManager::CoordinateManager():
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new CoordinateManagerParam();
}

CoordinateManager::~CoordinateManager()
{

}


