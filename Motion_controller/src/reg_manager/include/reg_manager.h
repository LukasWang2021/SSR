#ifndef REG_MANAGER_H
#define REG_MANAGER_H


#include "reg_manager_param.h"
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

    bool init();

    bool addPrReg(PrRegData* data_ptr);
    bool deletePrReg(int id);
    bool getPrReg(int id, PrRegData* data_ptr);
    bool updatePrReg(PrRegData* data_ptr);
    bool movePrReg(int expect_id, int original_id);
    std::vector<BaseRegSummary> getPrRegChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getPrRegValidList(int start_id, int size);

    bool addHrReg(HrRegData* data_ptr);
    bool deleteHrReg(int id);
    bool getHrReg(int id, HrRegData* data_ptr);
    bool updateHrReg(HrRegData* data_ptr);
    bool moveHrReg(int expect_id, int original_id);
    std::vector<BaseRegSummary> getHrRegChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getHrRegValidList(int start_id, int size);   

    bool addMrReg(MrRegData* data_ptr);
    bool deleteMrReg(int id);
    bool getMrReg(int id, MrRegData* data_ptr);
    bool updateMrReg(MrRegData* data_ptr);
    bool moveMrReg(int expect_id, int original_id);
    std::vector<BaseRegSummary> getMrRegChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getMrRegValidList(int start_id, int size);   

    bool addSrReg(SrRegData* data_ptr);
    bool deleteSrReg(int id);
    bool getSrReg(int id, SrRegData* data_ptr);
    bool updateSrReg(SrRegData* data_ptr);
    bool moveSrReg(int expect_id, int original_id);
    std::vector<BaseRegSummary> getSrRegChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getSrRegValidList(int start_id, int size);   

    bool addRReg(RRegData* data_ptr);
    bool deleteRReg(int id);
    bool getRReg(int id, RRegData* data_ptr);
    bool updateRReg(RRegData* data_ptr);
    bool moveRReg(int expect_id, int original_id);
    std::vector<BaseRegSummary> getRRegChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getRRegValidList(int start_id, int size);    

    void* getPrRegValueById(int id);
    void* getHrRegValueById(int id);
    void* getMrRegValueById(int id);
    void* getSrRegValueById(int id);
    void* getRRegValueById(int id);
    
private:
    RegManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    BaseReg* reg_ptr_[REG_TYPE_MAX];
};

}

#endif


