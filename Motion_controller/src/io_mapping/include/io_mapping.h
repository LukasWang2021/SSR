#ifndef IO_MAPPING_H
#define IO_MAPPING_H


#include "io_mapping_param.h"
#include "common_log.h"

namespace fst_ctrl
{
class IoMapping
{
public:
    IoMapping();
    ~IoMapping();

private:
    IoMappingParam param_;
    fst_log::Logger log_;
 
};

}

#endif


