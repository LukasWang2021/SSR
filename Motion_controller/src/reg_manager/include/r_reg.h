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
	double value;
	char cs[2];
}NVRamRRegData;

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
    double value;
}RRegDataIpc;

class RReg:public BaseReg
{
public:
    RReg(RegManagerParam* param_ptr);
    ~RReg();

    virtual ErrorCode init();
    virtual ErrorCode addReg(void* data_ptr);
    virtual ErrorCode deleteReg(int id);
    virtual ErrorCode getReg(int id, void* data_ptr);
    virtual ErrorCode updateReg(void* data_ptr);
    virtual ErrorCode moveReg(int expect_id, int original_id);
    void* getRegValueById(int id);
    bool updateRegValue(RRegDataIpc* data_ptr);
    bool getRegValue(int id, RRegDataIpc* data_ptr);
    
private:
    RegManagerParam* param_ptr_;
    std::string file_path_;
    fst_parameter::ParamGroup yaml_help_;
    std::vector<double> data_list_;

	Nvram nvram_obj_ ;
    int use_nvram_;

    RReg();
    bool createYaml();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data, const double& data);
    std::string getRegPath(int reg_id);
};

}


#endif

