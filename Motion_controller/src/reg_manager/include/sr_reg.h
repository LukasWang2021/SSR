#ifndef SR_REG_H
#define SR_REG_H

#include "base_reg.h"
#include "parameter_manager/parameter_manager_param_group.h"
#include "reg_manager_param.h"

namespace fst_ctrl
{
typedef struct
{
    int id;
    std::string name;
    std::string comment;
    std::string value;
}SrRegData;

class SrReg:public BaseReg
{
public:
    SrReg(RegManagerParam* param_ptr);
    ~SrReg();

    virtual bool init();
    virtual bool addReg(void* data_ptr);
    virtual bool deleteReg(int id);
    virtual bool getReg(int id, void* data_ptr);
    virtual bool updateReg(void* data_ptr);
    virtual bool moveReg(int expect_id, int original_id);
    void* getRegValueById(int id);
    
private:
    RegManagerParam* param_ptr_;
    std::string file_path_;
    fst_parameter::ParamGroup yaml_help_;
    std::vector<std::string> data_list_;

    SrReg();
    bool createYaml();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data, const std::string& data);
    std::string getRegPath(int reg_id);
};

}


#endif

