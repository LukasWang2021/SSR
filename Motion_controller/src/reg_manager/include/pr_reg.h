#ifndef PR_REG_H
#define PR_REG_H

#include "base_reg.h"
#include "parameter_manager/parameter_manager_param_group.h"


namespace fst_ctrl
{

#define MAX_PR_REG_POS_VALUE 99999999.999

typedef struct
{
    double joint_pos[9];        // support up to 9 axes per control group
    int group_id;
}PrValue;

typedef struct
{
    int id;
    std::string name;
    std::string comment;
    PrValue value;
}PrRegData;

class PrReg:public BaseReg
{
public:
    PrReg(int size, std::string file_path);
    ~PrReg();

    virtual bool init();
    virtual bool addReg(void* data_ptr);
    virtual bool deleteReg(int id);
    virtual bool getReg(int id, void* data_ptr);
    virtual bool setReg(void* data_ptr);
    
private:
    std::string file_path_;
    fst_parameter::ParamGroup yaml_help_;
    std::vector<PrValue> data_list_;

    PrReg();
    bool createYaml();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data, const PrValue& data);
    std::string getRegPath(int reg_id);
};

}


#endif

