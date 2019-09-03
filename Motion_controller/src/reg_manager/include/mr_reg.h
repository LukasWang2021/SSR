#ifndef MR_REG_H
#define MR_REG_H

#include "base_reg.h"
#include "parameter_manager/parameter_manager_param_group.h"
#include "reg_manager_param.h"

namespace fst_ctrl
{
typedef struct
{
	int id;
	int value;
	char cs[2];
}NVRamMrRegData;

typedef struct
{
    int id;
    std::string name;
    std::string comment;
    int value;
}MrRegData;

typedef struct
{
    int id;
    int value;
}MrRegDataIpc;

class MrReg:public BaseReg
{
public:
    MrReg(RegManagerParam* param_ptr);
    ~MrReg();

    virtual ErrorCode init();
    virtual ErrorCode addReg(void* data_ptr);
    virtual ErrorCode deleteReg(int id);
    virtual ErrorCode getReg(int id, void* data_ptr);
    virtual ErrorCode updateReg(void* data_ptr);
    virtual ErrorCode moveReg(int expect_id, int original_id);
    void* getRegValueById(int id);
    bool updateRegValue(MrRegDataIpc* data_ptr);
    bool getRegValue(int id, MrRegDataIpc* data_ptr);
    
private:
    RegManagerParam* param_ptr_;
    std::string file_path_;
    fst_parameter::ParamGroup yaml_help_;
    std::vector<int> data_list_;

	Nvram nvram_obj_ ;
    int use_nvram_;

    MrReg();
    bool createYaml();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data, const int& data);
    std::string getRegPath(int reg_id);
};

}


#endif

