#ifndef PR_REG_H
#define PR_REG_H

#include "base_reg.h"
#include "parameter_manager/parameter_manager_param_group.h"
#include "reg_manager_param.h"
#include "common_log.h"

namespace fst_ctrl
{
typedef enum
{
    PR_REG_POS_TYPE_JOINT = 0,
    PR_REG_POS_TYPE_CARTESIAN = 1,
}PrRegPosType;

typedef struct
{
    int pos_type;
    double pos[9];        // support up to 9 axes per control group
    int turn[9];
    int posture[4];
    int group_id;
}PrValue;

typedef struct
{
	int id;
	PrValue value;
	char cs[2];
}NVRamPrRegData;

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
    PrReg(RegManagerParam* param_ptr);
    virtual ~PrReg();

    virtual ErrorCode init();
    virtual ErrorCode addReg(void* data_ptr);
    virtual ErrorCode deleteReg(int id);
    virtual ErrorCode getReg(int id, void* data_ptr);
    virtual ErrorCode updateReg(void* data_ptr);
    virtual ErrorCode moveReg(int expect_id, int original_id);
    void* getRegValueById(int id);
    bool updateRegPos(PrRegDataIpc* data_ptr);
    bool getRegPos(int id, PrRegDataIpc* data_ptr);
    
private:
    RegManagerParam* param_ptr_;
    std::string file_path_;
    fst_parameter::ParamGroup yaml_help_;
    std::vector<PrValue> data_list_;
    fst_log::Logger* log_ptr_;

//	Nvram nvram_obj_ ;
//    int use_nvram_;

    PrReg();
    bool createYaml();
    bool readAllRegDataFromYaml();
    bool writeRegDataToYaml(const BaseRegData& base_data, const PrValue& data);
    std::string getRegPath(int reg_id);
};

}


#endif

