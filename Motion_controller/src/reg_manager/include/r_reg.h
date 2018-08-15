#ifndef R_REG_H
#define R_REG_H

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
    double value;
}RRegData;

typedef struct
{
    int id;
    char name[32];
    char comment[256];
    double value;
}RRegDataIpc;

class RReg:public BaseReg
{
public:
    RReg(RegManagerParam* param_ptr);
    ~RReg();

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
    std::vector<double> data_list_;

    RReg();
    bool createYaml();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data, const double& data);
    std::string getRegPath(int reg_id);
};

}


#endif

