#ifndef PR_REG_H
#define PR_REG_H

#include "base_reg.h"
#include "reg_manager_param.h"
#include "nvram_handler.h"
#include "yaml_help.h"
#include "log_manager_producer.h"

namespace fst_ctrl
{
typedef enum
{
    PR_REG_POS_TYPE_JOINT = 0,
    PR_REG_POS_TYPE_CARTESIAN = 1,
}PrRegPosType;

typedef struct
{
    double pos[9];        // support up to 9 axes per control group
    uint8_t pos_type;
    int8_t posture[4];
    int8_t turn[9];
    int32_t group_id;
}PrValue;

typedef struct
{
    int id;
    std::string name;
    std::string comment;
    PrValue value;
}PrRegData;

typedef struct
{
    int id;
    PrValue value;
}PrRegDataIpc;

class PrReg:public BaseReg
{
public:
    PrReg(RegManagerParam* param_ptr, rtm_nvram::NvramHandler* nvram_ptr);
    virtual ~PrReg();

    virtual ErrorCode init();
    virtual ErrorCode addReg(void* data_ptr);
    virtual ErrorCode deleteReg(int id);
    virtual ErrorCode getReg(int id, void* data_ptr);
    virtual ErrorCode updateReg(void* data_ptr);
    virtual ErrorCode moveReg(int expect_id, int original_id);
    bool getRegValueById(int id, PrValue& pr_value);
    bool updateRegPos(PrRegDataIpc* data_ptr);
    bool getRegPos(int id, PrRegDataIpc* data_ptr);
    
private:
    RegManagerParam* param_ptr_;
    rtm_nvram::NvramHandler* nvram_ptr_;
    std::string file_path_;
    std::string file_path_modified_;
    base_space::YamlHelp yaml_help_;

    PrReg();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data);
    std::string getRegPath(int reg_id);
};

}


#endif

