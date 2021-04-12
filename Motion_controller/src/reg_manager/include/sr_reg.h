#ifndef SR_REG_H
#define SR_REG_H

#include "base_reg.h"
#include "reg_manager_param.h"
#include "nvram_handler.h"
#include "yaml_help.h"
#include "log_manager_producer.h"

#define STRING_REG_MAX_LENGTH 256

namespace fst_ctrl
{
typedef struct
{
	char value[STRING_REG_MAX_LENGTH];
}SrValue;

typedef struct
{
    int id;
    std::string name;
    std::string comment;
    SrValue value;
}SrRegData;

typedef struct
{
    int id;
    SrValue value;
}SrRegDataIpc;

class SrReg:public BaseReg
{
public:
    SrReg(RegManagerParam* param_ptr, rtm_nvram::NvramHandler* nvram_ptr);
    virtual ~SrReg();

    virtual ErrorCode init();
    virtual ErrorCode addReg(void* data_ptr);
    virtual ErrorCode deleteReg(int id);
    virtual ErrorCode getReg(int id, void* data_ptr);
    virtual ErrorCode updateReg(void* data_ptr);
    virtual ErrorCode moveReg(int expect_id, int original_id);
    bool getRegValueById(int id, SrValue& sr_value);
    bool updateRegValue(SrRegDataIpc* data_ptr);
    bool getRegValue(int id, SrRegDataIpc* data_ptr);
    
private:
    RegManagerParam* param_ptr_;
    rtm_nvram::NvramHandler* nvram_ptr_;
    std::string file_path_;
    std::string file_path_modified_;
    base_space::YamlHelp yaml_help_;

    SrReg();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data);
    std::string getRegPath(int reg_id);
};

}


#endif

