#ifndef R_REG_H
#define R_REG_H

#include "base_reg.h"
#include "reg_manager_param.h"
#include "nvram_handler.h"
#include "yaml_help.h"
#include "log_manager_producer.h"

namespace fst_ctrl
{
typedef struct
{
	double value;
}RValue;

typedef struct
{
    int id;
    std::string name;
    std::string comment;
    RValue value;
}RRegData;

typedef struct
{
    int id;
    RValue value;
}RRegDataIpc;

class RReg:public BaseReg
{
public:
    RReg(RegManagerParam* param_ptr, rtm_nvram::NvramHandler* nvram_ptr);
    virtual ~RReg();

    virtual ErrorCode init();
    virtual ErrorCode addReg(void* data_ptr);
    virtual ErrorCode deleteReg(int id);
    virtual ErrorCode getReg(int id, void* data_ptr);
    virtual ErrorCode updateReg(void* data_ptr);
    virtual ErrorCode moveReg(int expect_id, int original_id);
    bool getRegValueById(int id, RValue& r_value);
    bool updateRegValue(RRegDataIpc* data_ptr);
    bool getRegValue(int id, RRegDataIpc* data_ptr);
    
private:
    RegManagerParam* param_ptr_;
    rtm_nvram::NvramHandler* nvram_ptr_;
    std::string file_path_;
    std::string file_path_modified_;
    base_space::YamlHelp yaml_help_;

    RReg();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data);
    std::string getRegPath(int reg_id);
};

}


#endif

