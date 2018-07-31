#ifndef REG_MANAGER_H
#define REG_MANAGER_H


#include "reg_manager_param.h"
#include "common_log.h"

namespace fst_ctrl
{
class RegManager
{
public:
    RegManager();
    ~RegManager();

private:
    RegManagerParam param_;
    fst_log::Logger log_;
 
};

}

#endif


