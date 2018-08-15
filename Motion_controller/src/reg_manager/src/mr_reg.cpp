#include "mr_reg.h"
#include <cstddef>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <sstream>


using namespace std;
using namespace fst_ctrl;
using namespace fst_parameter;


MrReg::MrReg(RegManagerParam* param_ptr):
    BaseReg(REG_TYPE_MR, param_ptr->mr_reg_number_), 
        param_ptr_(param_ptr), file_path_(param_ptr->reg_info_dir_)
{
    file_path_ += param_ptr_->mr_reg_file_name_;
}

MrReg::~MrReg()
{

}

bool MrReg::init()
{
    data_list_.resize(getListSize());    // id=0 is not used, id start from 1

    /*if(access(file_path_.c_str(), 0) != 0)
    {
        if(!createYaml())
        {
            return false;
        }
    }*/

    if(!readAllRegDataFromYaml())
    {
        return false;
    }
    
    return true;
}

bool MrReg::addReg(void* data_ptr)
{
    if(data_ptr == NULL)
    {
        return false;
    }

    MrRegData* reg_ptr = reinterpret_cast<MrRegData*>(data_ptr);
    if(!isAddInputValid(reg_ptr->id)
        || reg_ptr->value > param_ptr_->mr_value_limit_
        || reg_ptr->value < -param_ptr_->mr_value_limit_)
    {
        return false;
    }
    BaseRegData reg_data;
    packAddRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);
    if(!setRegList(reg_data))
    {
        return false;
    }
    data_list_[reg_data.id] = reg_ptr->value;
    return writeRegDataToYaml(reg_data, data_list_[reg_data.id]);
}

bool MrReg::deleteReg(int id)
{
    if(!isDeleteInputValid(id))
    {
        return false;
    }

    BaseRegData reg_data;
    packDeleteRegData(reg_data, id);
    if(!setRegList(reg_data))
    {
        return false;
    }
    data_list_[id] = 0;
    return writeRegDataToYaml(reg_data, data_list_[id]);
}

bool MrReg::getReg(int id, void* data_ptr)
{
    if(!isGetInputValid(id))
    {
        return false;
    }

    MrRegData* reg_ptr = reinterpret_cast<MrRegData*>(data_ptr);
    BaseRegData reg_data;
    if(!getRegList(id, reg_data))
    {
        return false;
    }
    reg_ptr->id = reg_data.id;
    reg_ptr->name = reg_data.name;
    reg_ptr->comment = reg_data.comment;
    reg_ptr->value = data_list_[id];
    return true;
}

bool MrReg::updateReg(void* data_ptr)
{
    if(data_ptr == NULL)
    {
        return false;
    }

    MrRegData* reg_ptr = reinterpret_cast<MrRegData*>(data_ptr);
    if(!isSetInputValid(reg_ptr->id)
        || reg_ptr->value > param_ptr_->mr_value_limit_
        || reg_ptr->value < -param_ptr_->mr_value_limit_)
    {
        return false;
    }
        
    BaseRegData reg_data;
    packSetRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);
    if(!setRegList(reg_data))
    {
        return false;
    }
    data_list_[reg_data.id] = reg_ptr->value;
    return writeRegDataToYaml(reg_data, data_list_[reg_data.id]);
}

bool MrReg::moveReg(int expect_id, int original_id)
{
    if(!isMoveInputValid(expect_id, original_id))
    {
        return false;
    }

    MrRegData data;
    if(!getReg(original_id, (void*)&data)
        || !deleteReg(original_id))
    {
        return false;
    }
    data.id = expect_id;
    return addReg((void*)&data);
}

void* MrReg::getRegValueById(int id)
{
    if(id <= 0
        || id >= data_list_.size())
    {
        return NULL;
    }
        
    return (void*)&data_list_[id];
}

MrReg::MrReg():
    BaseReg(REG_TYPE_INVALID, 0)
{

}

bool MrReg::createYaml()
{
    std::ofstream fd;
    fd.open(file_path_.c_str(), std::ios::out);
    if(!fd.is_open())
    {
        return false;
    }
    fd.close();
    
    yaml_help_.loadParamFile(file_path_.c_str());
    for(int i = 1; i < data_list_.size(); ++i)
    {
        std::string reg_path = getRegPath(i);
        yaml_help_.setParam(reg_path + "/id", i);
        yaml_help_.setParam(reg_path + "/is_valid", false);
        yaml_help_.setParam(reg_path + "/name", std::string("default"));
        yaml_help_.setParam(reg_path + "/comment", std::string("default"));
        yaml_help_.setParam(reg_path + "/value", (double)0);
    }
    return yaml_help_.dumpParamFile(file_path_.c_str());
}

bool MrReg::readAllRegDataFromYaml()
{
    yaml_help_.loadParamFile(file_path_.c_str());
    for(int i = 1; i < data_list_.size(); ++i)
    {
        std::string reg_path = getRegPath(i);
        BaseRegData base_data;
        std::string name, comment;
        yaml_help_.getParam(reg_path + "/id", base_data.id);
        yaml_help_.getParam(reg_path + "/is_valid", base_data.is_valid);
        yaml_help_.getParam(reg_path + "/name", name);
        yaml_help_.getParam(reg_path + "/comment", comment);
        base_data.is_changed = true;
        if(!setRegList(base_data))
        {
            return false;
        }
        yaml_help_.getParam(reg_path + "/value", data_list_[i]);        
    }
    return true;
}

bool MrReg::writeRegDataToYaml(const BaseRegData& base_data, const int& data)
{
    std::string reg_path = getRegPath(base_data.id);
    yaml_help_.setParam(reg_path + "/id", base_data.id);
    yaml_help_.setParam(reg_path + "/is_valid", base_data.is_valid);
    yaml_help_.setParam(reg_path + "/name", base_data.name);
    yaml_help_.setParam(reg_path + "/comment", base_data.comment);
    yaml_help_.setParam(reg_path + "/value", data);
    return yaml_help_.dumpParamFile(file_path_.c_str());
}

std::string MrReg::getRegPath(int reg_id)
{
    std::string id_str;
    std::stringstream stream;
    stream << reg_id;
    stream >> id_str;
    return (std::string("MR") + id_str);
}

