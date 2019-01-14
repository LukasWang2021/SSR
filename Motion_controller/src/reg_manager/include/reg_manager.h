#ifndef REG_MANAGER_H
#define REG_MANAGER_H


#include "reg_manager_param.h"
#include "base_datatype.h"
#include "common_log.h"
#include "base_reg.h"
#include "pr_reg.h"
#include "hr_reg.h"
#include "mr_reg.h"
#include "sr_reg.h"
#include "r_reg.h"
#include <string>

namespace fst_ctrl
{
class RegManager
{
public:
    RegManager();
    ~RegManager();

    ErrorCode init();

    bool isRegValid(RegType reg_type, int reg_index);

    ErrorCode addPrReg(PrRegData* data_ptr);
    ErrorCode deletePrReg(int id);
    ErrorCode getPrReg(int id, PrRegData* data_ptr);
    ErrorCode updatePrReg(PrRegData* data_ptr);
    ErrorCode movePrReg(int expect_id, int original_id);
    std::vector<BaseRegSummary> getPrRegChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getPrRegValidList(int start_id, int size);

    ErrorCode addHrReg(HrRegData* data_ptr);
    ErrorCode deleteHrReg(int id);
    ErrorCode getHrReg(int id, HrRegData* data_ptr);
    ErrorCode updateHrReg(HrRegData* data_ptr);
    ErrorCode moveHrReg(int expect_id, int original_id);
    std::vector<BaseRegSummary> getHrRegChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getHrRegValidList(int start_id, int size);   

    ErrorCode addMrReg(MrRegData* data_ptr);
    ErrorCode deleteMrReg(int id);
    ErrorCode getMrReg(int id, MrRegData* data_ptr);
    ErrorCode updateMrReg(MrRegData* data_ptr);
    ErrorCode moveMrReg(int expect_id, int original_id);
    std::vector<BaseRegSummary> getMrRegChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getMrRegValidList(int start_id, int size);   

    ErrorCode addSrReg(SrRegData* data_ptr);
    ErrorCode deleteSrReg(int id);
    ErrorCode getSrReg(int id, SrRegData* data_ptr);
    ErrorCode updateSrReg(SrRegData* data_ptr);
    ErrorCode moveSrReg(int expect_id, int original_id);
    std::vector<BaseRegSummary> getSrRegChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getSrRegValidList(int start_id, int size);   

    ErrorCode addRReg(RRegData* data_ptr);
    ErrorCode deleteRReg(int id);
    ErrorCode getRReg(int id, RRegData* data_ptr);
    ErrorCode updateRReg(RRegData* data_ptr);
    ErrorCode moveRReg(int expect_id, int original_id);
    std::vector<BaseRegSummary> getRRegChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getRRegValidList(int start_id, int size);    

    // rpc call
    void* getPrRegValueById(int id);
    void* getHrRegValueById(int id);
    void* getMrRegValueById(int id);
    void* getSrRegValueById(int id);
    void* getRRegValueById(int id);

    // ipc call
    bool updatePrRegPos(PrRegDataIpc* data_ptr);
    bool updateHrRegJointPos(HrRegDataIpc* data_ptr);
    bool updateMrRegValue(MrRegDataIpc* data_ptr);
    bool updateSrRegValue(SrRegDataIpc* data_ptr);
    bool updateRRegValue(RRegDataIpc* data_ptr);
    bool getPrRegPos(int id, PrRegDataIpc* data_ptr);
    bool getHrRegJointPos(int id, HrRegDataIpc* data_ptr);
    bool getMrRegValue(int id, MrRegDataIpc* data_ptr);
    bool getSrRegValue(int id, SrRegDataIpc* data_ptr);
    bool getRRegValue(int id, RRegDataIpc* data_ptr);
    
private:
    RegManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    BaseReg* reg_ptr_[REG_TYPE_MAX];
};

}

#endif


