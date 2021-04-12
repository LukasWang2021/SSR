#ifndef HR_REG_H
#define HR_REG_H

#include "base_reg.h"
#include "reg_manager_param.h"
#include "nvram_handler.h"
#include "yaml_help.h"
#include "log_manager_producer.h"

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
    HrReg(RegManagerParam* param_ptr, rtm_nvram::NvramHandler* nvram_ptr);
    virtual ~HrReg();

    virtual ErrorCode init();
    virtual ErrorCode addReg(void* data_ptr);
    virtual ErrorCode deleteReg(int id);
    virtual ErrorCode getReg(int id, void* data_ptr);
    virtual ErrorCode updateReg(void* data_ptr);
    virtual ErrorCode moveReg(int expect_id, int original_id);
    bool getRegValueById(int id, HrValue& hr_value);
    bool updateRegJointPos(HrRegDataIpc* data_ptr);
    bool getRegJointPos(int id, HrRegDataIpc* data_ptr);
    
private:
    RegManagerParam* param_ptr_;
    rtm_nvram::NvramHandler* nvram_ptr_;
    std::string file_path_;
    std::string file_path_modified_;
    base_space::YamlHelp yaml_help_;

    HrReg();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data);
    std::string getRegPath(int reg_id);
};

}


#endif

