#include "reg_manager.h"


using namespace fst_ctrl;


RegManager::RegManager():
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new RegManagerParam();
}

RegManager::~RegManager()
{

}


