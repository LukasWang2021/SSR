#include "io_mapping.h"


using namespace fst_ctrl;


IoMapping::IoMapping():
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new IoMappingParam();
}

IoMapping::~IoMapping()
{

}


