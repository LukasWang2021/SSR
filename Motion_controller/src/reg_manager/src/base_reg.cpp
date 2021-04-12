#include "base_reg.h"
#include <cstring>

using namespace std;
using namespace fst_ctrl;


BaseReg::BaseReg(RegType type, size_t size):
    type_(type)
{
    reg_list_.resize(size + 1); // id=0 is not used, id start from 1
}

BaseReg::~BaseReg()
{

}

vector<BaseRegSummary> BaseReg::getChangedList(int start_id, int size)
{
    BaseRegSummary summary;
    vector<BaseRegSummary> list;
    uint32_t end_id = static_cast<uint32_t>(start_id + size);
    start_id = (start_id <= 0 ? 1 : start_id);
    end_id = (end_id < reg_list_.size() ? end_id : reg_list_.size());
    for(uint32_t i = static_cast<uint32_t>(start_id); i < end_id; ++i)
    {
        if(reg_list_[i].is_changed)
        {
            summary.id = reg_list_[i].id;
            summary.name = reg_list_[i].name;
            summary.comment = reg_list_[i].comment;
            list.push_back(summary);
            reg_list_[i].is_changed = false;
        }
    }
    return list;
}

vector<BaseRegSummary> BaseReg::getValidList(int start_id, int size)
{
    BaseRegSummary summary;
    vector<BaseRegSummary> list;
    uint32_t end_id = static_cast<uint32_t>(start_id + size);
    start_id = (start_id <= 0 ? 1 : start_id);
    end_id = (end_id < reg_list_.size() ? end_id : reg_list_.size());
    for(uint32_t i = static_cast<uint32_t>(start_id); i < end_id; ++i)
    {
        if(reg_list_[i].is_valid)
        {
            summary.id = reg_list_[i].id;
            summary.name = reg_list_[i].name;
            summary.comment = reg_list_[i].comment;
            list.push_back(summary);
        }
    }
    return list;
}

bool BaseReg::isRegValid(int id)
{
    if (id <= 0) return false;
    uint32_t uid = static_cast<uint32_t>(id);
    if (uid >= reg_list_.size() || !reg_list_[uid].is_valid)
    {
        return false;
    }
    else
    {
        return true;
    }
}

BaseRegData* BaseReg::getBaseRegDataById(int id)
{
    if (id <= 0) return NULL;
    uint32_t uid = static_cast<uint32_t>(id);

    if(uid >= reg_list_.size())
    {
        return NULL;
    }
    else
    {
        return &reg_list_[uid];
    }
}

RegType BaseReg::getRegType()
{
    return type_;
}

bool BaseReg::isValid(int id)
{
    if (id <= 0) return false;
    uint32_t uid = static_cast<uint32_t>(id);

    if(uid >= reg_list_.size())
    {
        return false;
    }
    else
    {
        return reg_list_[uid].is_valid;
    }
}

size_t BaseReg::getListSize()
{
    return reg_list_.size();
}

bool BaseReg::setValid(int id, bool is_valid)
{
    if (id <= 0) return false;
    uint32_t uid = static_cast<uint32_t>(id);

    if(uid >= reg_list_.size())
    {
        return false;
    }
    else
    {
        reg_list_[uid].is_valid = is_valid;
        return true;
    }
}

bool BaseReg::setName(int id, string name)
{
    if (id <= 0) return false;
    uint32_t uid = static_cast<uint32_t>(id);

    if(uid >= reg_list_.size())
    {
        return false;
    }
    else
    {
        reg_list_[uid].name = name;
        return true;
    }
}

string BaseReg::getName(int id)
{
    if (id <= 0) return NULL;
    uint32_t uid = static_cast<uint32_t>(id);

    if(uid >= reg_list_.size())
    {
        return NULL;
    }
    else
    {
        return reg_list_[uid].name;
    }
}

bool BaseReg::setComment(int id, string comment)
{
    if (id <= 0) return false;
    uint32_t uid = static_cast<uint32_t>(id);

    if(uid >= reg_list_.size())
    {
        return false;
    }
    else
    {
        reg_list_[uid].comment = comment;
        return true;
    }
}

string BaseReg::getComment(int id)
{
    if (id <= 0) return string("");
    uint32_t uid = static_cast<uint32_t>(id);

    if(uid >= reg_list_.size())
    {
        return string("");
    }
    else
    {
        return reg_list_[uid].comment;
    }
}

bool BaseReg::setRegList(BaseRegData& data)
{
    if(data.id <= 0 || static_cast<uint32_t>(data.id) >= reg_list_.size())
    {
        return false;
    }
    else
    {
        reg_list_[data.id] = data;
        return true;
    }
}

bool BaseReg::getRegList(int id, BaseRegData& data)
{
    if(id <= 0 || static_cast<uint32_t>(id) >= reg_list_.size())
    {
        return false;
    }
    else
    {
        data = reg_list_[id];
        return true;
    }
}

bool BaseReg::isAddInputValid(int id)
{
    if(id <= 0
        || static_cast<uint32_t>(id) >= reg_list_.size()
        || reg_list_[id].is_valid)   // can't add some exist reg
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool BaseReg::isDeleteInputValid(int id)
{
    if(id <= 0
        || static_cast<uint32_t>(id) >= reg_list_.size())
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool BaseReg::isUpdateInputValid(int id)
{
    if(id <= 0
        || static_cast<uint32_t>(id) >= reg_list_.size()
        || !reg_list_[id].is_valid)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool BaseReg::isGetInputValid(int id)
{
    return isUpdateInputValid(id);
}

bool BaseReg::isMoveInputValid(int expect_id, int original_id)
{
    if(reg_list_[expect_id].is_valid
        || !reg_list_[original_id].is_valid)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void BaseReg::packAddRegData(BaseRegData& data, int id, string name, string comment)
{
    data.id = id;
    data.is_valid = true;
    data.is_changed = true;
    data.name = name;
    data.comment = comment;
}

void BaseReg::packDeleteRegData(BaseRegData& data, int id)
{
    data.id = id;
    data.is_valid = false;
    data.is_changed = true;
    data.name = "";
    data.comment = "";
}

void BaseReg::packSetRegData(BaseRegData& data, int id, string name, string comment)
{
    packAddRegData(data, id, name, comment);
}

BaseReg::BaseReg()
{

}


