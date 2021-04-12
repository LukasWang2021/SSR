#ifndef MR_REG_H
#define MR_REG_H

#include "base_reg.h"
#include "reg_manager_param.h"
#include "nvram_handler.h"
#include "yaml_help.h"
#include "log_manager_producer.h"

namespace fst_ctrl
{
typedef struct
{
	int value;
}MrValue;

typedef struct
{
    int id;
    std::string name;
    std::string comment;
    MrValue value;
}MrRegData;

typedef struct
{
    int id;
    MrValue value;
}MrRegDataIpc;

class MrReg:public BaseReg
{
public:
    MrReg(RegManagerParam* param_ptr, rtm_nvram::NvramHandler* nvram_ptr);
    virtual ~MrReg();

    virtual ErrorCode init();
    virtual ErrorCode addReg(void* data_ptr);
    virtual ErrorCode deleteReg(int id);
    virtual ErrorCode getReg(int id, void* data_ptr);
    virtual ErrorCode updateReg(void* data_ptr);
    virtual ErrorCode moveReg(int expect_id, int original_id);
    bool getRegValueById(int id, MrValue& mr_value);
    bool updateRegValue(MrRegDataIpc* data_ptr);
    bool getRegValue(int id, MrRegDataIpc* data_ptr);
    
private:
    RegManagerParam* param_ptr_;
    rtm_nvram::NvramHandler* nvram_ptr_;
    std::string file_path_;
    std::string file_path_modified_;
    base_space::YamlHelp yaml_help_;

    MrReg();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data);
    std::string getRegPath(int reg_id);
};

}


#endif

