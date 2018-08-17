#include "hr_reg.h"
#include <cstddef>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <sstream>


using namespace std;
using namespace fst_ctrl;
using namespace fst_parameter;


HrReg::HrReg(RegManagerParam* param_ptr):
    BaseReg(REG_TYPE_HR, param_ptr->hr_reg_number_), 
        param_ptr_(param_ptr), file_path_(param_ptr->reg_info_dir_)
{
    file_path_ += param_ptr_->hr_reg_file_name_;
}

HrReg::~HrReg()
{

}

bool HrReg::init()
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

bool HrReg::addReg(void* data_ptr)
{
    if(data_ptr == NULL)
    {
        return false;
    }

    HrRegData* reg_ptr = reinterpret_cast<HrRegData*>(data_ptr);
    if(!isAddInputValid(reg_ptr->id)
        || reg_ptr->value.joint_pos[0] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[0] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[1] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[1] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[2] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[2] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[3] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[3] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[4] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[4] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[5] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[5] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[6] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[6] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[7] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[7] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[8] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[8] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[0] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[0] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[1] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[1] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[2] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[2] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[3] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[3] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[4] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[4] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[5] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[5] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[6] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[6] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[7] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[7] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[8] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[8] < -param_ptr_->hr_value_limit_)
    {
        return false;
    }
    BaseRegData reg_data;
    packAddRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);
    if(!setRegList(reg_data))
    {
        return false;
    }
    data_list_[reg_data.id].group_id = reg_ptr->value.group_id;
    data_list_[reg_data.id].joint_pos[0] = reg_ptr->value.joint_pos[0];
    data_list_[reg_data.id].joint_pos[1] = reg_ptr->value.joint_pos[1];
    data_list_[reg_data.id].joint_pos[2] = reg_ptr->value.joint_pos[2];
    data_list_[reg_data.id].joint_pos[3] = reg_ptr->value.joint_pos[3];
    data_list_[reg_data.id].joint_pos[4] = reg_ptr->value.joint_pos[4];
    data_list_[reg_data.id].joint_pos[5] = reg_ptr->value.joint_pos[5];
    data_list_[reg_data.id].joint_pos[6] = reg_ptr->value.joint_pos[6];
    data_list_[reg_data.id].joint_pos[7] = reg_ptr->value.joint_pos[7];
    data_list_[reg_data.id].joint_pos[8] = reg_ptr->value.joint_pos[8];
    data_list_[reg_data.id].diff_pos[0] = reg_ptr->value.diff_pos[0];
    data_list_[reg_data.id].diff_pos[1] = reg_ptr->value.diff_pos[1];
    data_list_[reg_data.id].diff_pos[2] = reg_ptr->value.diff_pos[2];
    data_list_[reg_data.id].diff_pos[3] = reg_ptr->value.diff_pos[3];
    data_list_[reg_data.id].diff_pos[4] = reg_ptr->value.diff_pos[4];
    data_list_[reg_data.id].diff_pos[5] = reg_ptr->value.diff_pos[5];
    data_list_[reg_data.id].diff_pos[6] = reg_ptr->value.diff_pos[6];
    data_list_[reg_data.id].diff_pos[7] = reg_ptr->value.diff_pos[7];
    data_list_[reg_data.id].diff_pos[8] = reg_ptr->value.diff_pos[8];
    return writeRegDataToYaml(reg_data, data_list_[reg_data.id]);
}

bool HrReg::deleteReg(int id)
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
    data_list_[id].group_id = -1;
    data_list_[id].joint_pos[0] = 0;
    data_list_[id].joint_pos[1] = 0;
    data_list_[id].joint_pos[2] = 0;
    data_list_[id].joint_pos[3] = 0;
    data_list_[id].joint_pos[4] = 0;
    data_list_[id].joint_pos[5] = 0;
    data_list_[id].joint_pos[6] = 0;
    data_list_[id].joint_pos[7] = 0;
    data_list_[id].joint_pos[8] = 0;
    data_list_[id].diff_pos[0] = 0;
    data_list_[id].diff_pos[1] = 0;
    data_list_[id].diff_pos[2] = 0;
    data_list_[id].diff_pos[3] = 0;
    data_list_[id].diff_pos[4] = 0;
    data_list_[id].diff_pos[5] = 0;
    data_list_[id].diff_pos[6] = 0;
    data_list_[id].diff_pos[7] = 0;
    data_list_[id].diff_pos[8] = 0;
    return writeRegDataToYaml(reg_data, data_list_[id]);
}

bool HrReg::getReg(int id, void* data_ptr)
{
    if(!isGetInputValid(id))
    {
        return false;
    }

    HrRegData* reg_ptr = reinterpret_cast<HrRegData*>(data_ptr);
    BaseRegData reg_data;
    if(!getRegList(id, reg_data))
    {
        return false;
    }
    reg_ptr->id = reg_data.id;
    reg_ptr->name = reg_data.name;
    reg_ptr->comment = reg_data.comment;
    reg_ptr->value.group_id = data_list_[id].group_id;
    reg_ptr->value.joint_pos[0] = data_list_[id].joint_pos[0];
    reg_ptr->value.joint_pos[1] = data_list_[id].joint_pos[1];
    reg_ptr->value.joint_pos[2] = data_list_[id].joint_pos[2];
    reg_ptr->value.joint_pos[3] = data_list_[id].joint_pos[3];
    reg_ptr->value.joint_pos[4] = data_list_[id].joint_pos[4];
    reg_ptr->value.joint_pos[5] = data_list_[id].joint_pos[5];
    reg_ptr->value.joint_pos[6] = data_list_[id].joint_pos[6];
    reg_ptr->value.joint_pos[7] = data_list_[id].joint_pos[7];
    reg_ptr->value.joint_pos[8] = data_list_[id].joint_pos[8];
    reg_ptr->value.diff_pos[0] = data_list_[id].diff_pos[0];
    reg_ptr->value.diff_pos[1] = data_list_[id].diff_pos[1];
    reg_ptr->value.diff_pos[2] = data_list_[id].diff_pos[2];
    reg_ptr->value.diff_pos[3] = data_list_[id].diff_pos[3];
    reg_ptr->value.diff_pos[4] = data_list_[id].diff_pos[4];
    reg_ptr->value.diff_pos[5] = data_list_[id].diff_pos[5];
    reg_ptr->value.diff_pos[6] = data_list_[id].diff_pos[6];
    reg_ptr->value.diff_pos[7] = data_list_[id].diff_pos[7];
    reg_ptr->value.diff_pos[8] = data_list_[id].diff_pos[8];
    return true;
}

bool HrReg::updateReg(void* data_ptr)
{
    if(data_ptr == NULL)
    {
        return false;
    }

    HrRegData* reg_ptr = reinterpret_cast<HrRegData*>(data_ptr);
    if(!isUpdateInputValid(reg_ptr->id)
        || reg_ptr->value.joint_pos[0] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[0] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[1] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[1] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[2] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[2] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[3] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[3] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[4] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[4] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[5] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[5] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[6] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[6] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[7] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[7] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.joint_pos[8] > param_ptr_->hr_value_limit_ || reg_ptr->value.joint_pos[8] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[0] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[0] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[1] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[1] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[2] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[2] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[3] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[3] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[4] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[4] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[5] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[5] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[6] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[6] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[7] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[7] < -param_ptr_->hr_value_limit_
        || reg_ptr->value.diff_pos[8] > param_ptr_->hr_value_limit_ || reg_ptr->value.diff_pos[8] < -param_ptr_->hr_value_limit_)
    {
        return false;
    }
        
    BaseRegData reg_data;
    packSetRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);
    if(!setRegList(reg_data))
    {
        return false;
    }
    data_list_[reg_data.id].group_id = reg_ptr->value.group_id;
    data_list_[reg_data.id].joint_pos[0] = reg_ptr->value.joint_pos[0];
    data_list_[reg_data.id].joint_pos[1] = reg_ptr->value.joint_pos[1];
    data_list_[reg_data.id].joint_pos[2] = reg_ptr->value.joint_pos[2];
    data_list_[reg_data.id].joint_pos[3] = reg_ptr->value.joint_pos[3];
    data_list_[reg_data.id].joint_pos[4] = reg_ptr->value.joint_pos[4];
    data_list_[reg_data.id].joint_pos[5] = reg_ptr->value.joint_pos[5];
    data_list_[reg_data.id].joint_pos[6] = reg_ptr->value.joint_pos[6];
    data_list_[reg_data.id].joint_pos[7] = reg_ptr->value.joint_pos[7];
    data_list_[reg_data.id].joint_pos[8] = reg_ptr->value.joint_pos[8];    
    data_list_[reg_data.id].diff_pos[0] = reg_ptr->value.diff_pos[0];
    data_list_[reg_data.id].diff_pos[1] = reg_ptr->value.diff_pos[1];
    data_list_[reg_data.id].diff_pos[2] = reg_ptr->value.diff_pos[2];
    data_list_[reg_data.id].diff_pos[3] = reg_ptr->value.diff_pos[3];
    data_list_[reg_data.id].diff_pos[4] = reg_ptr->value.diff_pos[4];
    data_list_[reg_data.id].diff_pos[5] = reg_ptr->value.diff_pos[5];
    data_list_[reg_data.id].diff_pos[6] = reg_ptr->value.diff_pos[6];
    data_list_[reg_data.id].diff_pos[7] = reg_ptr->value.diff_pos[7];
    data_list_[reg_data.id].diff_pos[8] = reg_ptr->value.diff_pos[8]; 
    return writeRegDataToYaml(reg_data, data_list_[reg_data.id]);
}

bool HrReg::moveReg(int expect_id, int original_id)
{
    if(!isMoveInputValid(expect_id, original_id))
    {
        return false;
    }

    HrRegData data;
    if(!getReg(original_id, (void*)&data)
        || !deleteReg(original_id))
    {
        return false;
    }
    data.id = expect_id;
    return addReg((void*)&data);
}

void* HrReg::getRegValueById(int id)
{
    if(id <= 0
        || id >= data_list_.size())
    {
        return NULL;
    }
        
    return (void*)&data_list_[id];
}

bool HrReg::updateRegJointPos(HrRegDataIpc* data_ptr)
{
    if(data_ptr == NULL)
    {
        return false;
    }

    if(!isUpdateInputValid(data_ptr->id)
        || data_ptr->joint_pos[0] > param_ptr_->hr_value_limit_ || data_ptr->joint_pos[0] < -param_ptr_->hr_value_limit_
        || data_ptr->joint_pos[1] > param_ptr_->hr_value_limit_ || data_ptr->joint_pos[1] < -param_ptr_->hr_value_limit_
        || data_ptr->joint_pos[2] > param_ptr_->hr_value_limit_ || data_ptr->joint_pos[2] < -param_ptr_->hr_value_limit_
        || data_ptr->joint_pos[3] > param_ptr_->hr_value_limit_ || data_ptr->joint_pos[3] < -param_ptr_->hr_value_limit_
        || data_ptr->joint_pos[4] > param_ptr_->hr_value_limit_ || data_ptr->joint_pos[4] < -param_ptr_->hr_value_limit_
        || data_ptr->joint_pos[5] > param_ptr_->hr_value_limit_ || data_ptr->joint_pos[5] < -param_ptr_->hr_value_limit_
        || data_ptr->joint_pos[6] > param_ptr_->hr_value_limit_ || data_ptr->joint_pos[6] < -param_ptr_->hr_value_limit_
        || data_ptr->joint_pos[7] > param_ptr_->hr_value_limit_ || data_ptr->joint_pos[7] < -param_ptr_->hr_value_limit_
        || data_ptr->joint_pos[8] > param_ptr_->hr_value_limit_ || data_ptr->joint_pos[8] < -param_ptr_->hr_value_limit_)
    {
        return false;
    }
        
    BaseRegData* reg_data_ptr = getBaseRegDataById(data_ptr->id);
    if(reg_data_ptr == NULL)
    {
        return false;
    }
    memcpy(&data_list_[data_ptr->id].joint_pos[0], &data_ptr->joint_pos[0], 9*sizeof(double));
    return writeRegDataToYaml(*reg_data_ptr, data_list_[data_ptr->id]);
}

bool HrReg::getRegJointPos(int id, HrRegDataIpc* data_ptr)
{
    if(!isGetInputValid(id))
    {
        return false;
    }
    
    data_ptr->id = id;
    memcpy(&data_ptr->joint_pos[0], &data_list_[data_ptr->id].joint_pos[0], 9*sizeof(double));
    return true;
}

HrReg::HrReg():
    BaseReg(REG_TYPE_INVALID, 0)
{

}

bool HrReg::createYaml()
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
        yaml_help_.setParam(reg_path + "/group_id", -1);
        yaml_help_.setParam(reg_path + "/joint_pos1", 0);
        yaml_help_.setParam(reg_path + "/joint_pos2", 0);
        yaml_help_.setParam(reg_path + "/joint_pos3", 0);
        yaml_help_.setParam(reg_path + "/joint_pos4", 0);
        yaml_help_.setParam(reg_path + "/joint_pos5", 0);
        yaml_help_.setParam(reg_path + "/joint_pos6", 0);
        yaml_help_.setParam(reg_path + "/joint_pos7", 0);
        yaml_help_.setParam(reg_path + "/joint_pos8", 0);
        yaml_help_.setParam(reg_path + "/joint_pos9", 0);
        yaml_help_.setParam(reg_path + "/diff_pos1", 0);
        yaml_help_.setParam(reg_path + "/diff_pos2", 0);
        yaml_help_.setParam(reg_path + "/diff_pos3", 0);
        yaml_help_.setParam(reg_path + "/diff_pos4", 0);
        yaml_help_.setParam(reg_path + "/diff_pos5", 0);
        yaml_help_.setParam(reg_path + "/diff_pos6", 0);
        yaml_help_.setParam(reg_path + "/diff_pos7", 0);
        yaml_help_.setParam(reg_path + "/diff_pos8", 0);
        yaml_help_.setParam(reg_path + "/diff_pos9", 0);
    }
    return yaml_help_.dumpParamFile(file_path_.c_str());
}

bool HrReg::readAllRegDataFromYaml()
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
        yaml_help_.getParam(reg_path + "/group_id", data_list_[i].group_id);
        yaml_help_.getParam(reg_path + "/joint_pos1", data_list_[i].joint_pos[0]);
        yaml_help_.getParam(reg_path + "/joint_pos2", data_list_[i].joint_pos[1]);
        yaml_help_.getParam(reg_path + "/joint_pos3", data_list_[i].joint_pos[2]);
        yaml_help_.getParam(reg_path + "/joint_pos4", data_list_[i].joint_pos[3]);
        yaml_help_.getParam(reg_path + "/joint_pos5", data_list_[i].joint_pos[4]);
        yaml_help_.getParam(reg_path + "/joint_pos6", data_list_[i].joint_pos[5]);
        yaml_help_.getParam(reg_path + "/joint_pos7", data_list_[i].joint_pos[6]);
        yaml_help_.getParam(reg_path + "/joint_pos8", data_list_[i].joint_pos[7]);
        yaml_help_.getParam(reg_path + "/joint_pos9", data_list_[i].joint_pos[8]);
        yaml_help_.getParam(reg_path + "/diff_pos1", data_list_[i].joint_pos[0]);
        yaml_help_.getParam(reg_path + "/diff_pos2", data_list_[i].joint_pos[1]);
        yaml_help_.getParam(reg_path + "/diff_pos3", data_list_[i].joint_pos[2]);
        yaml_help_.getParam(reg_path + "/diff_pos4", data_list_[i].joint_pos[3]);
        yaml_help_.getParam(reg_path + "/diff_pos5", data_list_[i].joint_pos[4]);
        yaml_help_.getParam(reg_path + "/diff_pos6", data_list_[i].joint_pos[5]);
        yaml_help_.getParam(reg_path + "/diff_pos7", data_list_[i].joint_pos[6]);
        yaml_help_.getParam(reg_path + "/diff_pos8", data_list_[i].joint_pos[7]);
        yaml_help_.getParam(reg_path + "/diff_pos9", data_list_[i].joint_pos[8]);
    }
    return true;
}

bool HrReg::writeRegDataToYaml(const BaseRegData& base_data, const HrValue& data)
{
    std::string reg_path = getRegPath(base_data.id);
    yaml_help_.setParam(reg_path + "/id", base_data.id);
    yaml_help_.setParam(reg_path + "/is_valid", base_data.is_valid);
    yaml_help_.setParam(reg_path + "/name", base_data.name);
    yaml_help_.setParam(reg_path + "/comment", base_data.comment);
    yaml_help_.setParam(reg_path + "/group_id", data.group_id);
    yaml_help_.setParam(reg_path + "/joint_pos1", data.joint_pos[0]);
    yaml_help_.setParam(reg_path + "/joint_pos2", data.joint_pos[1]);
    yaml_help_.setParam(reg_path + "/joint_pos3", data.joint_pos[2]);
    yaml_help_.setParam(reg_path + "/joint_pos4", data.joint_pos[3]);
    yaml_help_.setParam(reg_path + "/joint_pos5", data.joint_pos[4]);
    yaml_help_.setParam(reg_path + "/joint_pos6", data.joint_pos[5]);
    yaml_help_.setParam(reg_path + "/joint_pos7", data.joint_pos[6]);
    yaml_help_.setParam(reg_path + "/joint_pos8", data.joint_pos[7]);
    yaml_help_.setParam(reg_path + "/joint_pos9", data.joint_pos[8]);
    yaml_help_.setParam(reg_path + "/diff_pos1", data.joint_pos[0]);
    yaml_help_.setParam(reg_path + "/diff_pos2", data.joint_pos[1]);
    yaml_help_.setParam(reg_path + "/diff_pos3", data.joint_pos[2]);
    yaml_help_.setParam(reg_path + "/diff_pos4", data.joint_pos[3]);
    yaml_help_.setParam(reg_path + "/diff_pos5", data.joint_pos[4]);
    yaml_help_.setParam(reg_path + "/diff_pos6", data.joint_pos[5]);
    yaml_help_.setParam(reg_path + "/diff_pos7", data.joint_pos[6]);
    yaml_help_.setParam(reg_path + "/diff_pos8", data.joint_pos[7]);
    yaml_help_.setParam(reg_path + "/diff_pos9", data.joint_pos[8]);    
    return yaml_help_.dumpParamFile(file_path_.c_str());
}

std::string HrReg::getRegPath(int reg_id)
{
    std::string id_str;
    std::stringstream stream;
    stream << reg_id;
    stream >> id_str;
    return (std::string("HR") + id_str);
}

