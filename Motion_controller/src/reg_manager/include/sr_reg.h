#ifndef SR_REG_H
#define SR_REG_H

#include "base_reg.h"
#include "parameter_manager/parameter_manager_param_group.h"
#include "reg_manager_param.h"
#include "common_log.h"

namespace fst_ctrl
{
typedef struct
{
    int id;
    std::string name;
    std::string comment;
    std::string value;
}SrRegData;

typedef struct
{
    int id;
    char value[256];
}SrRegDataIpc;

class SrReg:public BaseReg
{
public:
    SrReg(RegManagerParam* param_ptr);
    virtual ~SrReg();

    virtual ErrorCode init();
    virtual ErrorCode addReg(void* data_ptr);
    virtual ErrorCode deleteReg(int id);
    virtual ErrorCode getReg(int id, void* data_ptr);
    virtual ErrorCode updateReg(void* data_ptr);
    virtual ErrorCode moveReg(int expect_id, int original_id);
    void* getRegValueById(int id);
    bool updateRegValue(SrRegDataIpc* data_ptr);
    bool getRegValue(int id, SrRegDataIpc* data_ptr);
    
private:
    RegManagerParam* param_ptr_;
    std::string file_path_;
    fst_parameter::ParamGroup yaml_help_;
    std::vector<std::string> data_list_;
    fst_log::Logger* log_ptr_;

    SrReg();
    bool createYaml();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data, const std::string& data);
    std::string getRegPath(int reg_id);
};

}


#endif

