#include "mr_reg.h"
#include <cstddef>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include "common_file_path.h"
#include "copy_file.h"

using namespace std;
using namespace fst_ctrl;
using namespace rtm_nvram;
using namespace log_space;


MrReg::MrReg(RegManagerParam* param_ptr, NvramHandler* nvram_ptr):
    BaseReg(REG_TYPE_MR, param_ptr->mr_reg_number_), param_ptr_(param_ptr), 
    nvram_ptr_(nvram_ptr),
    file_path_(REG_DIR), file_path_modified_(REG_DIR_MODIFIED)
{
    file_path_ += param_ptr_->mr_reg_file_name_;
    file_path_modified_ += param_ptr_->mr_reg_file_name_;
}

MrReg::~MrReg()
{
}

ErrorCode MrReg::init()
{
    if (!copyFile(file_path_.c_str(), file_path_modified_.c_str()))
    {
        LogProducer::error("reg_manager", "Fail to copy %s to %s", file_path_.c_str(), file_path_modified_.c_str());
        return REG_MANAGER_LOAD_MR_FAILED;
    }
    
    if (!readAllRegDataFromYaml())
    {
        LogProducer::error("reg_manager", "Fail to load mr from yaml");
        return REG_MANAGER_LOAD_MR_FAILED;
    }
	
    return SUCCESS;
}

ErrorCode MrReg::addReg(void* data_ptr)
{
    if (data_ptr == NULL)
    {
		LogProducer::error("reg_manager", "addReg: input mr ptr = NULL");
        return REG_MANAGER_INVALID_ARG;
    }

    MrRegData* reg_ptr = reinterpret_cast<MrRegData*>(data_ptr);
    if (!isAddInputValid(reg_ptr->id) || abs(reg_ptr->value.value) > param_ptr_->mr_value_limit_)
    {
		LogProducer::error("reg_manager", "addReg: input mr id = %d, data = %d invalid)", reg_ptr->id, reg_ptr->value.value);
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packAddRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);

    if (!setRegList(reg_data))
    {
		LogProducer::error("reg_manager", "addReg: fail to set mr list");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
        LogProducer::error("reg_manager", "addReg: fail to dump mr to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    return nvram_ptr_->writeNvram(MR_START_ADDR + sizeof(MrValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(MrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode MrReg::deleteReg(int id)
{
    if (!isDeleteInputValid(id))
    {
		LogProducer::error("reg_manager", "deleteReg: input mr id = %d invalid", id);
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packDeleteRegData(reg_data, id);

    if (!setRegList(reg_data))
    {
		LogProducer::error("reg_manager", "deleteReg: fail to dump mr to yaml");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
        LogProducer::error("reg_manager", "deleteReg: fail to dump mr to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    MrValue mr_value;
    mr_value.value = 0;
    return nvram_ptr_->writeNvram(MR_START_ADDR + sizeof(MrValue) * id, (uint8_t*)&mr_value, sizeof(MrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode MrReg::getReg(int id, void* data_ptr)
{
    if (!isGetInputValid(id))
    {
        LogProducer::error("reg_manager", "getReg: input mr id = %d invalid", id);
        return REG_MANAGER_INVALID_ARG;
    }

    MrRegData* reg_ptr = reinterpret_cast<MrRegData*>(data_ptr);
    BaseRegData reg_data;

    if (!getRegList(id, reg_data))
    {
		LogProducer::error("reg_manager", "getReg: fail to get mr list");
        return REG_MANAGER_INVALID_ARG;
    }
	
    reg_ptr->id = reg_data.id;
    reg_ptr->name = reg_data.name;
    reg_ptr->comment = reg_data.comment;
    return nvram_ptr_->readNvram(MR_START_ADDR + sizeof(MrValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(MrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode MrReg::updateReg(void* data_ptr)
{
    if (data_ptr == NULL)
    {
		LogProducer::error("reg_manager", "updateReg: input mr ptr = NULL");
        return REG_MANAGER_INVALID_ARG;
    }

	MrRegData* reg_ptr = reinterpret_cast<MrRegData*>(data_ptr);
	if (!isUpdateInputValid(reg_ptr->id) || abs(reg_ptr->value.value) > param_ptr_->mr_value_limit_)
	{
        LogProducer::error("reg_manager", "updateReg: input mr id = %d, data = %d invalid", reg_ptr->id, reg_ptr->value.value);
		return REG_MANAGER_INVALID_ARG;
	}

	// Save to Yaml		
	BaseRegData reg_data;
	packSetRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);

	if (!setRegList(reg_data))
	{
		LogProducer::error("reg_manager", "updateReg: fail to set mr list");
		return REG_MANAGER_INVALID_ARG;
	}

    if (!writeRegDataToYaml(reg_data))
	{
		LogProducer::error("reg_manager", "updateReg: fail to dump mr to yaml");
		return REG_MANAGER_REG_FILE_WRITE_FAILED;
	}

	return nvram_ptr_->writeNvram(MR_START_ADDR + sizeof(MrValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(MrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode MrReg::moveReg(int expect_id, int original_id)
{
    if (!isMoveInputValid(expect_id, original_id))
    {
		LogProducer::error("reg_manager", "moveReg: input mr expect_id = %d original_id = %d invalid", expect_id, original_id);
        return REG_MANAGER_INVALID_ARG;
    }

    MrRegData data;
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

bool MrReg::getRegValueById(int id, MrValue& mr_value)
{
    if (!isRegValid(id))
    {
        LogProducer::error("reg_manager", "getRegValueById: input mr id = %d invalid", id);
        return false;
    }

    return nvram_ptr_->readNvram(MR_START_ADDR + sizeof(MrValue) * id, (uint8_t*)&mr_value, sizeof(MrValue));
}

bool MrReg::updateRegValue(MrRegDataIpc* data_ptr)
{
    if (data_ptr == NULL)
    {
		LogProducer::error("reg_manager", "updateRegValue: input mr ptr = NULL");
        return false;
    }

    if (!isUpdateInputValid(data_ptr->id) || abs(data_ptr->value.value) > param_ptr_->mr_value_limit_)
    {
        LogProducer::error("reg_manager", "updateRegValue: input mr id = %d, data = %d invalid", data_ptr->id, data_ptr->value.value);
        return false;
    }

    return nvram_ptr_->writeNvram(MR_START_ADDR + sizeof(MrValue) * data_ptr->id, (uint8_t*)&data_ptr->value, sizeof(MrValue));
}

bool MrReg::getRegValue(int id, MrRegDataIpc* data_ptr)
{
    if (!isGetInputValid(id))
    {
        LogProducer::error("reg_manager", "getRegValue: input mr id = %d invalid", id);
        return false;
    }

    MrValue mr_value; 
    nvram_ptr_->readNvram(MR_START_ADDR + sizeof(MrValue) * id, (uint8_t*)&mr_value, sizeof(MrValue));
    data_ptr->id = id;
    data_ptr->value = mr_value;
    return true;
}

MrReg::MrReg(): BaseReg(REG_TYPE_INVALID, 0)
{
}

bool MrReg::readAllRegDataFromYaml()
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
            LogProducer::error("reg_manager", "readAllRegDataFromYaml: fail to set mr list %d", i);
            return false;
        }
    }

    return true;
}

bool MrReg::writeRegDataToYaml(const BaseRegData& base_data)
{
    string reg_path = getRegPath(base_data.id);
    yaml_help_.setParam(reg_path + "/id", base_data.id);
    yaml_help_.setParam(reg_path + "/is_valid", base_data.is_valid);
    yaml_help_.setParam(reg_path + "/name", base_data.name);
    yaml_help_.setParam(reg_path + "/comment", base_data.comment);
    return yaml_help_.dumpParamFile(file_path_modified_.c_str());
}

string MrReg::getRegPath(int reg_id)
{
    string id_str;
    stringstream stream;
    stream << reg_id;
    stream >> id_str;
    return (string("MR") + id_str);
}

