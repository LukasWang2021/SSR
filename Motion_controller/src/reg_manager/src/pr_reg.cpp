#include "pr_reg.h"
#include <cstddef>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include "error_code.h"
#include <iostream>


using namespace std;
using namespace fst_ctrl;
using namespace fst_parameter;


PrReg::PrReg(RegManagerParam* param_ptr):
    BaseReg(REG_TYPE_PR, param_ptr->pr_reg_number_), 
        param_ptr_(param_ptr), file_path_(param_ptr->reg_info_dir_)
{
    file_path_ += param_ptr_->pr_reg_file_name_;
}

PrReg::~PrReg()
{

}

ErrorCode PrReg::init()
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
        return REG_MANAGER_LOAD_PR_FAILED;
    }
    
    return SUCCESS;
}

ErrorCode PrReg::addReg(void* data_ptr)
{
    if(data_ptr == NULL)
    {
        return REG_MANAGER_INVALID_ARG;
    }

    PrRegData* reg_ptr = reinterpret_cast<PrRegData*>(data_ptr);
    if(!isAddInputValid(reg_ptr->id))
    {
        return REG_MANAGER_INVALID_ARG;
    }
    if(reg_ptr->value.pos_type != PR_REG_POS_TYPE_JOINT
        && reg_ptr->value.pos_type != PR_REG_POS_TYPE_CARTESIAN)
    {
        return REG_MANAGER_INVALID_ARG;
    }
    if(reg_ptr->value.pos[0] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[0] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[1] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[1] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[2] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[2] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[3] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[3] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[4] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[4] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[5] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[5] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[6] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[6] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[7] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[7] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[8] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[8] < -param_ptr_->pr_value_limit_)
    {
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packAddRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);
    if(!setRegList(reg_data))
    {
        return REG_MANAGER_INVALID_ARG;
    }
    data_list_[reg_data.id].pos_type = reg_ptr->value.pos_type;
    data_list_[reg_data.id].group_id = reg_ptr->value.group_id;
    data_list_[reg_data.id].pos[0] = reg_ptr->value.pos[0];
    data_list_[reg_data.id].pos[1] = reg_ptr->value.pos[1];
    data_list_[reg_data.id].pos[2] = reg_ptr->value.pos[2];
    data_list_[reg_data.id].pos[3] = reg_ptr->value.pos[3];
    data_list_[reg_data.id].pos[4] = reg_ptr->value.pos[4];
    data_list_[reg_data.id].pos[5] = reg_ptr->value.pos[5];
    data_list_[reg_data.id].pos[6] = reg_ptr->value.pos[6];
    data_list_[reg_data.id].pos[7] = reg_ptr->value.pos[7];
    data_list_[reg_data.id].pos[8] = reg_ptr->value.pos[8];
    data_list_[reg_data.id].posture[0] = reg_ptr->value.posture[0];
    data_list_[reg_data.id].posture[1] = reg_ptr->value.posture[1];
    data_list_[reg_data.id].posture[2] = reg_ptr->value.posture[2];
    data_list_[reg_data.id].posture[3] = reg_ptr->value.posture[3];
    if(!writeRegDataToYaml(reg_data, data_list_[reg_data.id]))
    {
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode PrReg::deleteReg(int id)
{
    if(!isDeleteInputValid(id))
    {
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packDeleteRegData(reg_data, id);
    if(!setRegList(reg_data))
    {
        return REG_MANAGER_INVALID_ARG;
    }
    data_list_[id].pos_type = PR_REG_POS_TYPE_JOINT;
    data_list_[id].group_id = -1;
    data_list_[id].pos[0] = 0;
    data_list_[id].pos[1] = 0;
    data_list_[id].pos[2] = 0;
    data_list_[id].pos[3] = 0;
    data_list_[id].pos[4] = 0;
    data_list_[id].pos[5] = 0;
    data_list_[id].pos[6] = 0;
    data_list_[id].pos[7] = 0;
    data_list_[id].pos[8] = 0;
    data_list_[id].posture[0] = false;
    data_list_[id].posture[1] = false;
    data_list_[id].posture[2] = false;
    data_list_[id].posture[3] = false;
    if(!writeRegDataToYaml(reg_data, data_list_[id]))
    {
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode PrReg::getReg(int id, void* data_ptr)
{
    if(!isGetInputValid(id))
    {
        return REG_MANAGER_INVALID_ARG;
    }

    PrRegData* reg_ptr = reinterpret_cast<PrRegData*>(data_ptr);
    BaseRegData reg_data;
    if(!getRegList(id, reg_data))
    {
        return REG_MANAGER_INVALID_ARG;
    }
    reg_ptr->id = reg_data.id;
    reg_ptr->name = reg_data.name;
    reg_ptr->comment = reg_data.comment;
    reg_ptr->value.pos_type = data_list_[id].pos_type;
    reg_ptr->value.group_id = data_list_[id].group_id;
    reg_ptr->value.pos[0] = data_list_[id].pos[0];
    reg_ptr->value.pos[1] = data_list_[id].pos[1];
    reg_ptr->value.pos[2] = data_list_[id].pos[2];
    reg_ptr->value.pos[3] = data_list_[id].pos[3];
    reg_ptr->value.pos[4] = data_list_[id].pos[4];
    reg_ptr->value.pos[5] = data_list_[id].pos[5];
    reg_ptr->value.pos[6] = data_list_[id].pos[6];
    reg_ptr->value.pos[7] = data_list_[id].pos[7];
    reg_ptr->value.pos[8] = data_list_[id].pos[8];
    reg_ptr->value.posture[0] = data_list_[id].posture[0];
    reg_ptr->value.posture[1] = data_list_[id].posture[1];
    reg_ptr->value.posture[2] = data_list_[id].posture[2];
    reg_ptr->value.posture[3] = data_list_[id].posture[3];
    return SUCCESS;
}

ErrorCode PrReg::updateReg(void* data_ptr)
{
    if(data_ptr == NULL)
    {
        return REG_MANAGER_INVALID_ARG;
    }

    PrRegData* reg_ptr = reinterpret_cast<PrRegData*>(data_ptr);
    if(!isUpdateInputValid(reg_ptr->id))
    {
        return REG_MANAGER_INVALID_ARG;
    }
    
    if(reg_ptr->value.pos_type != PR_REG_POS_TYPE_JOINT
        && reg_ptr->value.pos_type != PR_REG_POS_TYPE_CARTESIAN)
    {
        return REG_MANAGER_INVALID_ARG;
    }
        
    if(reg_ptr->value.pos[0] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[0] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[1] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[1] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[2] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[2] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[3] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[3] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[4] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[4] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[5] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[5] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[6] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[6] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[7] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[7] < -param_ptr_->pr_value_limit_
        || reg_ptr->value.pos[8] > param_ptr_->pr_value_limit_ || reg_ptr->value.pos[8] < -param_ptr_->pr_value_limit_)
    {
        return REG_MANAGER_INVALID_ARG;
    }
        
    BaseRegData reg_data;
    packSetRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);
    if(!setRegList(reg_data))
    {
        return REG_MANAGER_INVALID_ARG;
    }
    data_list_[reg_data.id].pos_type = reg_ptr->value.pos_type;
    data_list_[reg_data.id].group_id = reg_ptr->value.group_id;
    data_list_[reg_data.id].pos[0] = reg_ptr->value.pos[0];
    data_list_[reg_data.id].pos[1] = reg_ptr->value.pos[1];
    data_list_[reg_data.id].pos[2] = reg_ptr->value.pos[2];
    data_list_[reg_data.id].pos[3] = reg_ptr->value.pos[3];
    data_list_[reg_data.id].pos[4] = reg_ptr->value.pos[4];
    data_list_[reg_data.id].pos[5] = reg_ptr->value.pos[5];
    data_list_[reg_data.id].pos[6] = reg_ptr->value.pos[6];
    data_list_[reg_data.id].pos[7] = reg_ptr->value.pos[7];
    data_list_[reg_data.id].pos[8] = reg_ptr->value.pos[8];    
    data_list_[reg_data.id].posture[0] = reg_ptr->value.posture[0];
    data_list_[reg_data.id].posture[1] = reg_ptr->value.posture[1];
    data_list_[reg_data.id].posture[2] = reg_ptr->value.posture[2];
    data_list_[reg_data.id].posture[3] = reg_ptr->value.posture[3];
    if(!writeRegDataToYaml(reg_data, data_list_[reg_data.id]))
    {
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode PrReg::moveReg(int expect_id, int original_id)
{
    if(!isMoveInputValid(expect_id, original_id))
    {
        return REG_MANAGER_INVALID_ARG;
    }

    PrRegData data;
    ErrorCode error_code;
    error_code = getReg(original_id, (void*)&data);
    if(error_code != SUCCESS)
    {
        return error_code;
    }
    error_code = deleteReg(original_id);
    if(error_code != SUCCESS)
    {
        return error_code;
    }
    data.id = expect_id;
    return addReg((void*)&data);
}

void* PrReg::getRegValueById(int id)
{
    if(id <= 0
        || id >= data_list_.size())
    {
        return NULL;
    }
        
    return (void*)&data_list_[id];
}

bool PrReg::updateRegPos(PrRegDataIpc* data_ptr)
{
    if(data_ptr == NULL)
    {
        return false;
    }

    if(!isUpdateInputValid(data_ptr->id)
        || data_ptr->pos[0] > param_ptr_->pr_value_limit_ || data_ptr->pos[0] < -param_ptr_->pr_value_limit_
        || data_ptr->pos[1] > param_ptr_->pr_value_limit_ || data_ptr->pos[1] < -param_ptr_->pr_value_limit_
        || data_ptr->pos[2] > param_ptr_->pr_value_limit_ || data_ptr->pos[2] < -param_ptr_->pr_value_limit_
        || data_ptr->pos[3] > param_ptr_->pr_value_limit_ || data_ptr->pos[3] < -param_ptr_->pr_value_limit_
        || data_ptr->pos[4] > param_ptr_->pr_value_limit_ || data_ptr->pos[4] < -param_ptr_->pr_value_limit_
        || data_ptr->pos[5] > param_ptr_->pr_value_limit_ || data_ptr->pos[5] < -param_ptr_->pr_value_limit_
        || data_ptr->pos[6] > param_ptr_->pr_value_limit_ || data_ptr->pos[6] < -param_ptr_->pr_value_limit_
        || data_ptr->pos[7] > param_ptr_->pr_value_limit_ || data_ptr->pos[7] < -param_ptr_->pr_value_limit_
        || data_ptr->pos[8] > param_ptr_->pr_value_limit_ || data_ptr->pos[8] < -param_ptr_->pr_value_limit_)
    {
        return false;
    }
        
    BaseRegData* reg_data_ptr = getBaseRegDataById(data_ptr->id);
    if(reg_data_ptr == NULL)
    {
        return false;
    }
    memcpy(&data_list_[data_ptr->id].pos[0], &data_ptr->pos[0], 9*sizeof(double));
    return writeRegDataToYaml(*reg_data_ptr, data_list_[data_ptr->id]);
}

bool PrReg::getRegPos(int id, PrRegDataIpc* data_ptr)
{
    if(!isGetInputValid(id))
    {
        return false;
    }

    data_ptr->id = id;
    memcpy(&data_ptr->pos[0], &data_list_[data_ptr->id].pos[0], 9*sizeof(double));
    return true;
}


PrReg::PrReg():
    BaseReg(REG_TYPE_INVALID, 0)
{

}

bool PrReg::createYaml()
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
        yaml_help_.setParam(reg_path + "/pos_type", PR_REG_POS_TYPE_JOINT);
        yaml_help_.setParam(reg_path + "/group_id", -1);
        yaml_help_.setParam(reg_path + "/pos1", 0);
        yaml_help_.setParam(reg_path + "/pos2", 0);
        yaml_help_.setParam(reg_path + "/pos3", 0);
        yaml_help_.setParam(reg_path + "/pos4", 0);
        yaml_help_.setParam(reg_path + "/pos5", 0);
        yaml_help_.setParam(reg_path + "/pos6", 0);
        yaml_help_.setParam(reg_path + "/pos7", 0);
        yaml_help_.setParam(reg_path + "/pos8", 0);
        yaml_help_.setParam(reg_path + "/pos9", 0);
        yaml_help_.setParam(reg_path + "/posture1", false);
        yaml_help_.setParam(reg_path + "/posture2", false);
        yaml_help_.setParam(reg_path + "/posture3", false);
        yaml_help_.setParam(reg_path + "/posture4", false);
    }
    return yaml_help_.dumpParamFile(file_path_.c_str());
}

bool PrReg::readAllRegDataFromYaml()
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
        yaml_help_.getParam(reg_path + "/pos_type", data_list_[i].pos_type);
        yaml_help_.getParam(reg_path + "/group_id", data_list_[i].group_id);
        yaml_help_.getParam(reg_path + "/pos1", data_list_[i].pos[0]);
        yaml_help_.getParam(reg_path + "/pos2", data_list_[i].pos[1]);
        yaml_help_.getParam(reg_path + "/pos3", data_list_[i].pos[2]);
        yaml_help_.getParam(reg_path + "/pos4", data_list_[i].pos[3]);
        yaml_help_.getParam(reg_path + "/pos5", data_list_[i].pos[4]);
        yaml_help_.getParam(reg_path + "/pos6", data_list_[i].pos[5]);
        yaml_help_.getParam(reg_path + "/pos7", data_list_[i].pos[6]);
        yaml_help_.getParam(reg_path + "/pos8", data_list_[i].pos[7]);
        yaml_help_.getParam(reg_path + "/pos9", data_list_[i].pos[8]);
        yaml_help_.getParam(reg_path + "/posture1", data_list_[i].posture[0]);
        yaml_help_.getParam(reg_path + "/posture2", data_list_[i].posture[1]);
        yaml_help_.getParam(reg_path + "/posture3", data_list_[i].posture[2]);
        yaml_help_.getParam(reg_path + "/posture4", data_list_[i].posture[3]);
    }
    return true;
}

bool PrReg::writeRegDataToYaml(const BaseRegData& base_data, const PrValue& data)
{
    std::string reg_path = getRegPath(base_data.id);
    yaml_help_.setParam(reg_path + "/id", base_data.id);
    yaml_help_.setParam(reg_path + "/is_valid", base_data.is_valid);
    yaml_help_.setParam(reg_path + "/name", base_data.name);
    yaml_help_.setParam(reg_path + "/comment", base_data.comment);
    yaml_help_.setParam(reg_path + "/pos_type", data.pos_type);
    yaml_help_.setParam(reg_path + "/group_id", data.group_id);
    yaml_help_.setParam(reg_path + "/pos1", data.pos[0]);
    yaml_help_.setParam(reg_path + "/pos2", data.pos[1]);
    yaml_help_.setParam(reg_path + "/pos3", data.pos[2]);
    yaml_help_.setParam(reg_path + "/pos4", data.pos[3]);
    yaml_help_.setParam(reg_path + "/pos5", data.pos[4]);
    yaml_help_.setParam(reg_path + "/pos6", data.pos[5]);
    yaml_help_.setParam(reg_path + "/pos7", data.pos[6]);
    yaml_help_.setParam(reg_path + "/pos8", data.pos[7]);
    yaml_help_.setParam(reg_path + "/pos9", data.pos[8]);
    yaml_help_.setParam(reg_path + "/posture1", data.posture[0]);
    yaml_help_.setParam(reg_path + "/posture2", data.posture[1]);
    yaml_help_.setParam(reg_path + "/posture3", data.posture[2]);
    yaml_help_.setParam(reg_path + "/posture4", data.posture[3]);
    return yaml_help_.dumpParamFile(file_path_.c_str());
}

std::string PrReg::getRegPath(int reg_id)
{
    std::string id_str;
    std::stringstream stream;
    stream << reg_id;
    stream >> id_str;
    return (std::string("PR") + id_str);
}

