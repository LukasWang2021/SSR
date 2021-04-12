#include "hr_reg.h"
#include <cstddef>
#include <cstring>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <sstream>
#include "common_file_path.h"
#include "copy_file.h"


using namespace std;
using namespace fst_ctrl;
using namespace rtm_nvram;
using namespace log_space;


HrReg::HrReg(RegManagerParam* param_ptr, NvramHandler* nvram_ptr):
    BaseReg(REG_TYPE_HR, param_ptr->hr_reg_number_), param_ptr_(param_ptr),
    nvram_ptr_(nvram_ptr),
    file_path_(REG_DIR), file_path_modified_(REG_DIR_MODIFIED)
{
    file_path_ += param_ptr_->hr_reg_file_name_;
    file_path_modified_ += param_ptr_->hr_reg_file_name_; 
}

HrReg::~HrReg()
{
}

ErrorCode HrReg::init()
{
    if (!copyFile(file_path_.c_str(), file_path_modified_.c_str()))
    {
        LogProducer::error("reg_manager", "Fail to copy %s to %s", file_path_.c_str(), file_path_modified_.c_str());
        return REG_MANAGER_LOAD_HR_FAILED;
    }

    if (!readAllRegDataFromYaml())
    {
        LogProducer::error("reg_manager", "Fail to load hr from yaml");
        return REG_MANAGER_LOAD_HR_FAILED;
    }
    
    return SUCCESS;
}

ErrorCode HrReg::addReg(void* data_ptr)
{
    if (data_ptr == NULL)
    {
        LogProducer::error("reg_manager", "addReg: input hr ptr = NULL");
        return REG_MANAGER_INVALID_ARG;
    }

    HrRegData* reg_ptr = reinterpret_cast<HrRegData*>(data_ptr);
    if (!isAddInputValid(reg_ptr->id)
        || fabs(reg_ptr->value.joint_pos[0]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[1]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[2]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[3]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[4]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[5]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[6]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[7]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[8]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[0]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[1]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[2]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[3]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[4]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[5]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[6]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[7]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[8]) > param_ptr_->hr_value_limit_)
    {
        LogProducer::error("reg_manager", "addReg: input hr id = %d, data invalid", reg_ptr->id);
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packAddRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);

    if (!setRegList(reg_data))
    {
        LogProducer::error("reg_manager", "addReg: fail to set hr list");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
        LogProducer::error("reg_manager", "addReg: fail to dump hr to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    return nvram_ptr_->writeNvram(HR_START_ADDR + sizeof(HrValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(HrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode HrReg::deleteReg(int id)
{
    if (!isDeleteInputValid(id))
    {
        LogProducer::error("reg_manager", "deleteReg: input hr id = %d invalid", id);
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packDeleteRegData(reg_data, id);

    if (!setRegList(reg_data))
    {
        LogProducer::error("reg_manager", "deleteReg: fail to set hr list");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
        LogProducer::error("reg_manager", "deleteReg: fail to dump hr to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    HrValue hr_value;
    memset(&hr_value, 0, sizeof(hr_value));
    hr_value.group_id = -1;
    return nvram_ptr_->writeNvram(HR_START_ADDR + sizeof(HrValue) * id, (uint8_t*)&hr_value, sizeof(HrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode HrReg::getReg(int id, void* data_ptr)
{
    if (!isGetInputValid(id))
    {
        LogProducer::error("reg_manager", "getReg: input hr id = %d invalid", id);
        return REG_MANAGER_INVALID_ARG;
    }

    HrRegData* reg_ptr = reinterpret_cast<HrRegData*>(data_ptr);
    BaseRegData reg_data;

    if (!getRegList(id, reg_data))
    {
        LogProducer::error("reg_manager", "getReg: fail to get hr list");
        return REG_MANAGER_INVALID_ARG;
    }

    reg_ptr->id = reg_data.id;
    reg_ptr->name = reg_data.name;
    reg_ptr->comment = reg_data.comment;
    return nvram_ptr_->readNvram(HR_START_ADDR + sizeof(HrValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(HrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode HrReg::updateReg(void* data_ptr)
{
    if (data_ptr == NULL)
    {
        LogProducer::error("reg_manager", "updateReg: input hr ptr = NULL");
        return REG_MANAGER_INVALID_ARG;
    }

    HrRegData* reg_ptr = reinterpret_cast<HrRegData*>(data_ptr);
    if (!isUpdateInputValid(reg_ptr->id)
        || fabs(reg_ptr->value.joint_pos[0]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[1]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[2]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[3]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[4]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[5]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[6]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[7]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.joint_pos[8]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[0]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[1]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[2]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[3]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[4]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[5]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[6]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[7]) > param_ptr_->hr_value_limit_
        || fabs(reg_ptr->value.diff_pos[8]) > param_ptr_->hr_value_limit_)
    {
        LogProducer::error("reg_manager", "updateReg: input hr id = %d, data invalid", reg_ptr->id);
        return REG_MANAGER_INVALID_ARG;
    }
        
    BaseRegData reg_data;
    packSetRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);

    if (!setRegList(reg_data))
    {
        LogProducer::error("reg_manager", "updateReg: fail to set hr list");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
        LogProducer::error("reg_manager", "updateReg: fail to dump hr to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    return nvram_ptr_->writeNvram(HR_START_ADDR + sizeof(HrValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(HrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode HrReg::moveReg(int expect_id, int original_id)
{
    if (!isMoveInputValid(expect_id, original_id))
    {
        LogProducer::error("reg_manager", "moveReg: input hr expect_id = %d original_id = %d invalid", expect_id, original_id);
        return REG_MANAGER_INVALID_ARG;
    }

    HrRegData data;
    ErrorCode error_code;
    error_code = getReg(original_id, (void*)&data);

    if (error_code != SUCCESS)
    {
        return error_code;
    }

    error_code = deleteReg(original_id);

    if (error_code != SUCCESS)
    {
        return error_code;
    }

    data.id = expect_id;
    return addReg((void*)&data);
}

bool HrReg::getRegValueById(int id, HrValue& hr_value)
{
    if (!isRegValid(id))
    {
        LogProducer::error("reg_manager", "getRegValueById: input hr id = %d invalid", id);
        return false;
    }

    return nvram_ptr_->readNvram(HR_START_ADDR + sizeof(HrValue) * id, (uint8_t*)&hr_value, sizeof(HrValue));
}

bool HrReg::updateRegJointPos(HrRegDataIpc* data_ptr)
{
    if(data_ptr == NULL)
    {
        LogProducer::error("reg_manager", "updateRegJointPos: input hr ptr = NULL");
        return false;
    }

    if(!isUpdateInputValid(data_ptr->id)
        || fabs(data_ptr->joint_pos[0] > param_ptr_->hr_value_limit_)
        || fabs(data_ptr->joint_pos[1] > param_ptr_->hr_value_limit_)
        || fabs(data_ptr->joint_pos[2] > param_ptr_->hr_value_limit_)
        || fabs(data_ptr->joint_pos[3] > param_ptr_->hr_value_limit_)
        || fabs(data_ptr->joint_pos[4] > param_ptr_->hr_value_limit_)
        || fabs(data_ptr->joint_pos[5] > param_ptr_->hr_value_limit_)
        || fabs(data_ptr->joint_pos[6] > param_ptr_->hr_value_limit_)
        || fabs(data_ptr->joint_pos[7] > param_ptr_->hr_value_limit_)
        || fabs(data_ptr->joint_pos[8] > param_ptr_->hr_value_limit_))
    {
        LogProducer::error("reg_manager", "updateRegJointPos: input hr id = %d, data invalid", data_ptr->id);
        return false;
    }
        
    HrValue hr_value; 
    nvram_ptr_->readNvram(HR_START_ADDR + sizeof(HrValue) * data_ptr->id, (uint8_t*)&hr_value, sizeof(HrValue));
    memcpy(hr_value.joint_pos, data_ptr->joint_pos, sizeof(hr_value.joint_pos));
    return nvram_ptr_->writeNvram(HR_START_ADDR + sizeof(HrValue) * data_ptr->id, (uint8_t*)&hr_value, sizeof(HrValue));
}

bool HrReg::getRegJointPos(int id, HrRegDataIpc* data_ptr)
{
    if(!isGetInputValid(id))
    {
        LogProducer::error("reg_manager", "getRegJointPos: input hr id = %d invalid", id);
        return false;
    }
    
    HrValue hr_value; 
    nvram_ptr_->readNvram(HR_START_ADDR + sizeof(HrValue) * id, (uint8_t*)&hr_value, sizeof(HrValue));
    data_ptr->id = id;
    memcpy(data_ptr->joint_pos, hr_value.joint_pos, sizeof(data_ptr->joint_pos));
    return true;
}

HrReg::HrReg(): BaseReg(REG_TYPE_INVALID, 0)
{
}

bool HrReg::readAllRegDataFromYaml()
{
    yaml_help_.loadParamFile(file_path_modified_.c_str());
    uint32_t length = reg_list_.size();

    for (uint32_t i = 1; i < length; ++i)
    {
        string reg_path = getRegPath(i);
        BaseRegData base_data;
        yaml_help_.getParam(reg_path + "/id", base_data.id);
        yaml_help_.getParam(reg_path + "/is_valid", base_data.is_valid);
        yaml_help_.getParam(reg_path + "/name", base_data.name);
        yaml_help_.getParam(reg_path + "/comment", base_data.comment);
        base_data.is_changed = true;

        if (!setRegList(base_data))
        {
            LogProducer::error("reg_manager", "readAllRegDataFromYaml: fail to set hr list %d", i);
            return false;
        }
    }

    return true;
}

bool HrReg::writeRegDataToYaml(const BaseRegData& base_data)
{
    string reg_path = getRegPath(base_data.id);
    yaml_help_.setParam(reg_path + "/id", base_data.id);
    yaml_help_.setParam(reg_path + "/is_valid", base_data.is_valid);
    yaml_help_.setParam(reg_path + "/name", base_data.name);
    yaml_help_.setParam(reg_path + "/comment", base_data.comment);  
    return yaml_help_.dumpParamFile(file_path_modified_.c_str());
}

string HrReg::getRegPath(int reg_id)
{
    string id_str;
    stringstream stream;
    stream << reg_id;
    stream >> id_str;
    return (string("HR") + id_str);
}

