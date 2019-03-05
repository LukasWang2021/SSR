#ifndef PARAM_MANAGER_H
#define PARAM_MANAGER_H

#include "parameter_manager/parameter_manager_param_group.h"
#include <stdint.h>
#include <vector>
#include "error_code.h"

namespace fst_mc
{
typedef enum
{
    PARAM_GROUP_USER = 0,
    PARAM_GROUP_MANU,
}ParamGroup_e;

typedef enum
{
    PARAM_INFO_INT = 0,
    PARAM_INFO_DOUBLE,   
    PARAM_INFO_BOOL,
}ParamInfoType_e;

typedef struct
{
    char name[256];
    ParamInfoType_e type;
    uint8_t data[256];    
}ParamInfo_t;

class ParamManager
{
public:
    ParamManager();
    ~ParamManager();

    ErrorCode init();

    std::vector<ParamInfo_t> getParamInfoList(ParamGroup_e param_group);
    ErrorCode setParamInfo(ParamGroup_e param_group, ParamInfo_t& param_info);
    void printParamInfoList(ParamGroup_e param_group);
    
private:
    fst_parameter::ParamGroup yaml_help_;
    std::vector<ParamInfo_t> user_param_list_;
    std::vector<ParamInfo_t> manu_param_list_;
    std::vector<ParamInfo_t> dummy_list_;
};

}




#endif

