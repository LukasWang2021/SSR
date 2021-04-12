#ifndef BASE_REG_H
#define BASE_REG_H

#include <memory.h>
#include <vector>
#include <string>
#include "basic_alg_datatype.h"
#include "common_error_code.h"


namespace fst_ctrl
{
typedef enum
{
    REG_TYPE_PR = 0,
    REG_TYPE_HR = 1,
    REG_TYPE_SR = 2,
    REG_TYPE_MR = 3,
    REG_TYPE_R = 4,
    REG_TYPE_MAX = 5,
    REG_TYPE_INVALID = 6,
}RegType;

typedef enum
{
    PR_DATA_SIZE = 0x5000,
    HR_DATA_SIZE = 0x1000,
    SR_DATA_SIZE = 0x13000,
    MR_DATA_SIZE = 0x1000,
    R_DATA_SIZE = 0x3000,
}RegNvramDataSize;

typedef enum
{
    PR_START_ADDR = 0x0,
    HR_START_ADDR = PR_START_ADDR + PR_DATA_SIZE,
    SR_START_ADDR = HR_START_ADDR + HR_DATA_SIZE,
    MR_START_ADDR = SR_START_ADDR + SR_DATA_SIZE,
    R_START_ADDR = MR_START_ADDR + MR_DATA_SIZE,
}RegNvramAddress;


typedef struct
{
    int id;
    std::string name;
    std::string comment;
}BaseRegSummary;

typedef struct
{
    int id;
    bool is_valid;
    bool is_changed;
    std::string name;
    std::string comment;
}BaseRegData;

class BaseReg
{
public:
    BaseReg(RegType type, size_t size);
    virtual ~BaseReg();

    virtual ErrorCode init() = 0;
    virtual ErrorCode addReg(void* data_ptr) = 0;
    virtual ErrorCode deleteReg(int id) = 0;
    virtual ErrorCode getReg(int id, void* data_ptr) = 0;
    virtual ErrorCode updateReg(void* data_ptr) = 0;
    virtual ErrorCode moveReg(int expect_id, int original_id) = 0;
    std::vector<BaseRegSummary> getChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getValidList(int start_id, int size);
    bool isRegValid(int id);

    BaseRegData* getBaseRegDataById(int id);
    RegType getRegType();
    size_t getListSize();
    bool isValid(int id);
    bool setValid(int id, bool is_valid);
    bool setName(int id, std::string name);
    std::string getName(int id);
    bool setComment(int id, std::string comment);
    std::string getComment(int id);
    bool setRegList(BaseRegData& data);
    bool getRegList(int id, BaseRegData& data);
    bool isAddInputValid(int id);
    bool isDeleteInputValid(int id);
    bool isUpdateInputValid(int id);
    bool isGetInputValid(int id);
    bool isMoveInputValid(int expect_id, int original_id);
    void packAddRegData(BaseRegData& data, int id, std::string name, std::string comment);
    void packDeleteRegData(BaseRegData& data, int id);
    void packSetRegData(BaseRegData& data, int id, std::string name, std::string comment);
    
protected:
    BaseReg();
    RegType type_;
    std::vector<BaseRegData> reg_list_;
};

}


#endif

