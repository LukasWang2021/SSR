#ifndef MR_REG_H
#define MR_REG_H

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
    int value;
}MrRegData;

class MrReg:public BaseReg
{
public:
    MrReg(RegManagerParam* param_ptr);
    ~MrReg();

    virtual bool init();
    virtual bool addReg(void* data_ptr);
    virtual bool deleteReg(int id);
    virtual bool getReg(int id, void* data_ptr);
    virtual bool setReg(void* data_ptr);
    virtual bool moveReg(int expect_id, int original_id);
    void* getRegValueById(int id);
    
private:
    RegManagerParam* param_ptr_;
    std::string file_path_;
    fst_parameter::ParamGroup yaml_help_;
    std::vector<int> data_list_;

    MrReg();
    bool createYaml();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data, const int& data);
    std::string getRegPath(int reg_id);
};

}


#endif

