#ifndef HR_REG_H
#define HR_REG_H

#include "base_reg.h"
#include "parameter_manager/parameter_manager_param_group.h"
#include "reg_manager_param.h"

namespace fst_ctrl
{
typedef struct
{
    double joint_pos[9];        // support up to 9 axes per control group
    double diff_pos[9];
    int group_id;
}HrValue;

typedef struct
{
    int id;
    std::string name;
    std::string comment;
    HrValue value;
}HrRegData;

typedef struct
{
    int id;
    double joint_pos[9];
}HrRegDataIpc;

class HrReg:public BaseReg
{
public:
    HrReg(RegManagerParam* param_ptr);
    ~HrReg();

    virtual bool init();
    virtual bool addReg(void* data_ptr);
    virtual bool deleteReg(int id);
    virtual bool getReg(int id, void* data_ptr);
    virtual bool updateReg(void* data_ptr);
    virtual bool moveReg(int expect_id, int original_id);
    void* getRegValueById(int id);
    bool updateRegJointPos(HrRegDataIpc* data_ptr);
    bool getRegJointPos(int id, HrRegDataIpc* data_ptr);
    
private:
    RegManagerParam* param_ptr_;
    std::string file_path_;
    fst_parameter::ParamGroup yaml_help_;
    std::vector<HrValue> data_list_;

    HrReg();
    bool createYaml();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data, const HrValue& data);
    std::string getRegPath(int reg_id);
};

}


#endif

