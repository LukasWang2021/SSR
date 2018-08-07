#ifndef R_REG_H
#define R_REG_H

#include "base_reg.h"
#include "parameter_manager/parameter_manager_param_group.h"


namespace fst_ctrl
{

#define MAX_R_REG_VALUE 99999999.999 


typedef struct
{
    int id;
    std::string name;
    std::string comment;
    double value;
}RRegData;

class RReg:public BaseReg
{
public:
    RReg(int size, std::string file_path);
    ~RReg();

    virtual bool init();
    virtual bool addReg(void* data_ptr);
    virtual bool deleteReg(int id);
    virtual bool getReg(int id, void* data_ptr);
    virtual bool setReg(void* data_ptr);
    
private:
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

