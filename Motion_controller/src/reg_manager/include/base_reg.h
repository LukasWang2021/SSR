#ifndef BASE_REG_H
#define BASE_REG_H

#include <vector>
#include <string>

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
    BaseReg(RegType type, int size);
    ~BaseReg();

    virtual bool init() = 0;
    virtual bool addReg(void* data_ptr) = 0;
    virtual bool deleteReg(int id) = 0;
    virtual bool getReg(int id, void* data_ptr) = 0;
    virtual bool updateReg(void* data_ptr) = 0;
    virtual bool moveReg(int expect_id, int original_id) = 0;
    virtual void* getRegValueById(int id) = 0;
    std::vector<BaseRegSummary> getChangedList(int start_id, int size);
    std::vector<BaseRegSummary> getValidList(int start_id, int size);

    RegType getRegType();
    int getListSize();
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
    bool isSetInputValid(int id);
    bool isGetInputValid(int id);
    bool isMoveInputValid(int expect_id, int original_id);
    void packAddRegData(BaseRegData& data, int id, std::string name, std::string comment);
    void packDeleteRegData(BaseRegData& data, int id);
    void packSetRegData(BaseRegData& data, int id, std::string name, std::string comment);
    
private:
    BaseReg();
    RegType type_;
    std::vector<BaseRegData> reg_list_;
};

}


#endif

