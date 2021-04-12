#include "pr_reg.h"
#include <cstddef>
#include <cstring>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include "common_file_path.h"
#include "copy_file.h"


using namespace std;
using namespace fst_ctrl;
using namespace rtm_nvram;
using namespace log_space;


PrReg::PrReg(RegManagerParam* param_ptr, NvramHandler* nvram_ptr):
    BaseReg(REG_TYPE_PR, param_ptr->pr_reg_number_),
    param_ptr_(param_ptr), 
    nvram_ptr_(nvram_ptr),
    file_path_(REG_DIR),
    file_path_modified_(REG_DIR_MODIFIED)
{
    file_path_ += param_ptr_->pr_reg_file_name_;
    file_path_modified_ += param_ptr_->pr_reg_file_name_;
}

PrReg::~PrReg()
{}

ErrorCode PrReg::init()
{
    if (!copyFile(file_path_.c_str(), file_path_modified_.c_str()))
    {
        LogProducer::error("reg_manager", "Fail to copy %s to %s", file_path_.c_str(), file_path_modified_.c_str());
        return REG_MANAGER_LOAD_PR_FAILED;
    }

    if (!readAllRegDataFromYaml())
    {
		LogProducer::error("reg_manager", "Fail to load pr from yaml");
        return REG_MANAGER_LOAD_PR_FAILED;
    }

    return SUCCESS;
}

ErrorCode PrReg::addReg(void* data_ptr)
{
    if (data_ptr == NULL)
    {
		LogProducer::error("reg_manager", "addReg: input pr ptr = NULL");
        return REG_MANAGER_INVALID_ARG;
    }

    PrRegData* reg_ptr = reinterpret_cast<PrRegData*>(data_ptr);
    if (!isAddInputValid(reg_ptr->id))
    {
		LogProducer::error("reg_manager", "addReg: input pr id = %d invalid)", reg_ptr->id);
        return REG_MANAGER_INVALID_ARG;
    }

    if (reg_ptr->value.pos_type != PR_REG_POS_TYPE_JOINT && reg_ptr->value.pos_type != PR_REG_POS_TYPE_CARTESIAN)
    {
		LogProducer::error("reg_manager", "addReg: pr type = %d invalid", reg_ptr->value.pos_type);
        return REG_MANAGER_INVALID_ARG;
    }

    if (fabs(reg_ptr->value.pos[0]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[1]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[2]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[3]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[4]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[5]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[6]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[7]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[8]) > param_ptr_->pr_value_limit_)
    {
		LogProducer::error("reg_manager", "addReg: pr value out of limit");
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packAddRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);

    if (!setRegList(reg_data))
    {
		LogProducer::error("reg_manager", "addReg: fail to set pr list");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
		LogProducer::error("reg_manager", "addReg: fail to dump pr to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    return nvram_ptr_->writeNvram(PR_START_ADDR + sizeof(PrValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(PrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode PrReg::deleteReg(int id)
{
    if (!isDeleteInputValid(id))
    {
		LogProducer::error("reg_manager", "deleteReg: input pr id = %d invalid", id);
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packDeleteRegData(reg_data, id);

    if (!setRegList(reg_data))
    {
		LogProducer::error("reg_manager", "deleteReg: fail to dump pr to yaml");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
		LogProducer::error("reg_manager", "deleteReg: fail to dump pr to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    PrValue pr_value;
    memset(&pr_value, 0, sizeof(pr_value));
    pr_value.group_id = -1;
    return nvram_ptr_->writeNvram(PR_START_ADDR + sizeof(PrValue) * id, (uint8_t*)&pr_value, sizeof(PrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode PrReg::getReg(int id, void* data_ptr)
{
    if (!isGetInputValid(id))
    {
		LogProducer::error("reg_manager", "getReg: input pr id = %d invalid", id);
        return REG_MANAGER_INVALID_ARG;
    }

    PrRegData* reg_ptr = reinterpret_cast<PrRegData*>(data_ptr);
    BaseRegData reg_data;

    if (!getRegList(id, reg_data))
    {
		LogProducer::error("reg_manager", "getReg: fail to get pr list");
        return REG_MANAGER_INVALID_ARG;
    }

    reg_ptr->id = reg_data.id;
    reg_ptr->name = reg_data.name;
    reg_ptr->comment = reg_data.comment;
    return nvram_ptr_->readNvram(PR_START_ADDR + sizeof(PrValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(PrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode PrReg::updateReg(void* data_ptr)
{
    if (data_ptr == NULL)
    {
		LogProducer::error("reg_manager", "updateReg: input pr ptr = NULL");
        return REG_MANAGER_INVALID_ARG;
    }

    PrRegData* reg_ptr = reinterpret_cast<PrRegData*>(data_ptr);
    if (!isUpdateInputValid(reg_ptr->id))
    {
        LogProducer::error("reg_manager", "updateReg: input pr id = %d invalid", reg_ptr->id);
        return REG_MANAGER_INVALID_ARG;
    }
    
    if (reg_ptr->value.pos_type != PR_REG_POS_TYPE_JOINT && reg_ptr->value.pos_type != PR_REG_POS_TYPE_CARTESIAN)
    {
		LogProducer::error("reg_manager", "updateReg: pr type = %d invalid", reg_ptr->value.pos_type);
        return REG_MANAGER_INVALID_ARG;
    }
        
    if (fabs(reg_ptr->value.pos[0]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[1]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[2]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[3]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[4]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[5]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[6]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[7]) > param_ptr_->pr_value_limit_
        || fabs(reg_ptr->value.pos[8]) > param_ptr_->pr_value_limit_)
    {
        LogProducer::error("reg_manager", "updateReg: pr value out of limit");
        return REG_MANAGER_INVALID_ARG;
    }
        
    BaseRegData reg_data;
    packSetRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);

    if (!setRegList(reg_data))
    {
		LogProducer::error("reg_manager", "updateReg: fail to set pr list");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
		LogProducer::error("reg_manager", "updateReg: fail to dump pr to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    return nvram_ptr_->writeNvram(PR_START_ADDR + sizeof(PrValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(PrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode PrReg::moveReg(int expect_id, int original_id)
{
    if (!isMoveInputValid(expect_id, original_id))
    {
		LogProducer::error("reg_manager", "moveReg: input pr expect_id = %d original_id = %d invalid", expect_id, original_id);
        return REG_MANAGER_INVALID_ARG;
    }

    PrRegData data;
    ErrorCode error_code = getReg(original_id, (void*)&data);

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

bool PrReg::getRegValueById(int id, PrValue& pr_value)
{
    if (!isRegValid(id))
    {
        LogProducer::error("reg_manager", "getRegValueById: input pr id = %d invalid", id);
        return false;
    }

    return nvram_ptr_->readNvram(PR_START_ADDR + sizeof(PrValue) * id, (uint8_t*)&pr_value, sizeof(PrValue));
}

bool PrReg::updateRegPos(PrRegDataIpc* data_ptr)
{
    if (data_ptr == NULL)
    {
		LogProducer::error("reg_manager", "updateRegPos: input pr ptr = NULL");
        return false;
    }

    if (!isUpdateInputValid(data_ptr->id)
        || fabs(data_ptr->value.pos[0]) > param_ptr_->pr_value_limit_
        || fabs(data_ptr->value.pos[1]) > param_ptr_->pr_value_limit_
        || fabs(data_ptr->value.pos[2]) > param_ptr_->pr_value_limit_
        || fabs(data_ptr->value.pos[3]) > param_ptr_->pr_value_limit_
        || fabs(data_ptr->value.pos[4]) > param_ptr_->pr_value_limit_
        || fabs(data_ptr->value.pos[5]) > param_ptr_->pr_value_limit_
        || fabs(data_ptr->value.pos[6]) > param_ptr_->pr_value_limit_
        || fabs(data_ptr->value.pos[7]) > param_ptr_->pr_value_limit_
        || fabs(data_ptr->value.pos[8]) > param_ptr_->pr_value_limit_)
    {
        LogProducer::error("reg_manager", "updateRegPos: input pr id = %d invalid", data_ptr->id);
        return false;
    }

    return nvram_ptr_->writeNvram(PR_START_ADDR + sizeof(PrValue) * data_ptr->id, (uint8_t*)&data_ptr->value, sizeof(PrValue));
}

bool PrReg::getRegPos(int id, PrRegDataIpc* data_ptr)
{
    if (!isGetInputValid(id))
    {
		LogProducer::error("reg_manager", "getRegPos: input pr id = %d invalid", id);
        return false;
    }

    data_ptr->id = id;
    nvram_ptr_->readNvram(PR_START_ADDR + sizeof(PrValue) * id, (uint8_t*)&data_ptr->value, sizeof(PrValue));
    return true;
}

PrReg::PrReg(): BaseReg(REG_TYPE_INVALID, 0)
{
}

bool PrReg::readAllRegDataFromYaml()
{
    yaml_help_.loadParamFile(file_path_modified_.c_str());
    uint32_t length = reg_list_.size();

    for (uint32_t i = 1; i < length; ++i)
    {
        string reg_path = getRegPath(static_cast<int>(i));
        BaseRegData base_data;
        yaml_help_.getParam(reg_path + "/id", base_data.id);
        yaml_help_.getParam(reg_path + "/is_valid", base_data.is_valid);
        yaml_help_.getParam(reg_path + "/name", base_data.name);
        yaml_help_.getParam(reg_path + "/comment", base_data.comment);
        base_data.is_changed = true;

        if (!setRegList(base_data))
        {
            LogProducer::error("reg_manager", "readAllRegDataFromYaml: fail to set pr list %d", i);
            return false;
        }
    }

    return true;
}

bool PrReg::writeRegDataToYaml(const BaseRegData& base_data)
{
    string reg_path = getRegPath(base_data.id);
    yaml_help_.setParam(reg_path + "/id", base_data.id);
    yaml_help_.setParam(reg_path + "/is_valid", base_data.is_valid);
    yaml_help_.setParam(reg_path + "/name", base_data.name);
    yaml_help_.setParam(reg_path + "/comment", base_data.comment);
    return yaml_help_.dumpParamFile(file_path_modified_.c_str());
}

std::string PrReg::getRegPath(int reg_id)
{
    string id_str;
    stringstream stream;
    stream << reg_id;
    stream >> id_str;
    return (string("PR") + id_str);
}

