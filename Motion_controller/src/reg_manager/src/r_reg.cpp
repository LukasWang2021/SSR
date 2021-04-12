#include "r_reg.h"
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


RReg::RReg(RegManagerParam* param_ptr, NvramHandler* nvram_ptr):
    BaseReg(REG_TYPE_R, param_ptr->r_reg_number_),
    param_ptr_(param_ptr), 
    nvram_ptr_(nvram_ptr),
    file_path_(REG_DIR),
    file_path_modified_(REG_DIR_MODIFIED)
{
    file_path_ += param_ptr_->r_reg_file_name_;
    file_path_modified_ += param_ptr_->r_reg_file_name_;
}

RReg::~RReg()
{}

ErrorCode RReg::init()
{
    if (!copyFile(file_path_.c_str(), file_path_modified_.c_str()))
    {
        LogProducer::error("reg_manager", "Fail to copy %s to %s", file_path_.c_str(), file_path_modified_.c_str());
        return REG_MANAGER_LOAD_R_FAILED;
    }

    if (!readAllRegDataFromYaml())
    {
		LogProducer::error("reg_manager", "Fail to load r from yaml");
        return REG_MANAGER_LOAD_R_FAILED;
    }
    
    return SUCCESS;
}

ErrorCode RReg::addReg(void* data_ptr)
{
    if (data_ptr == NULL)
    {
		LogProducer::error("reg_manager", "addReg: input r ptr = NULL");
        return REG_MANAGER_INVALID_ARG;
    }

    RRegData* reg_ptr = reinterpret_cast<RRegData*>(data_ptr);
    if (!isAddInputValid(reg_ptr->id) || fabs(reg_ptr->value.value) > param_ptr_->r_value_limit_)
    {
		LogProducer::error("reg_manager", "addReg: input r id = %d, data = %.6f invalid)", reg_ptr->id, reg_ptr->value.value);
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packAddRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);

    if (!setRegList(reg_data))
    {
		LogProducer::error("reg_manager", "addReg: fail to set r list");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
        LogProducer::error("reg_manager", "addReg: fail to dump r to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    return nvram_ptr_->writeNvram(R_START_ADDR + sizeof(RValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(RValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode RReg::deleteReg(int id)
{
    if (!isDeleteInputValid(id))
    {
		LogProducer::error("reg_manager", "deleteReg: input r id = %d invalid", id);
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packDeleteRegData(reg_data, id);

    if (!setRegList(reg_data))
    {
		LogProducer::error("reg_manager", "deleteReg: fail to dump r to yaml");
        return REG_MANAGER_INVALID_ARG;
    }
	
	if (!writeRegDataToYaml(reg_data))
    {
        LogProducer::error("reg_manager", "deleteReg: fail to dump r to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }
	
    RValue r_value;
    r_value.value = 0;
    return nvram_ptr_->writeNvram(R_START_ADDR + sizeof(RValue) * id, (uint8_t*)&r_value, sizeof(RValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode RReg::getReg(int id, void* data_ptr)
{
    if (!isGetInputValid(id))
    {
		LogProducer::error("reg_manager", "getReg: input r id = %d invalid", id);
        return REG_MANAGER_INVALID_ARG;
    }

    RRegData* reg_ptr = reinterpret_cast<RRegData*>(data_ptr);
    BaseRegData reg_data;

    if (!getRegList(id, reg_data))
    {
		LogProducer::error("reg_manager", "getReg: fail to get r list");
        return REG_MANAGER_INVALID_ARG;
    }

    reg_ptr->id = reg_data.id;
    reg_ptr->name = reg_data.name;
    reg_ptr->comment = reg_data.comment;
    return nvram_ptr_->readNvram(R_START_ADDR + sizeof(RValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(RValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode RReg::updateReg(void* data_ptr)
{
    if (data_ptr == NULL)
    {
		LogProducer::error("reg_manager", "updateReg: input r ptr = NULL");
        return REG_MANAGER_INVALID_ARG;
    }

	RRegData* reg_ptr = reinterpret_cast<RRegData*>(data_ptr);
    if (!isUpdateInputValid(reg_ptr->id) || fabs(reg_ptr->value.value) > param_ptr_->r_value_limit_)
    {
        LogProducer::error("reg_manager", "updateReg: input r id = %d, data = %d invalid", reg_ptr->id, reg_ptr->value.value);
        return REG_MANAGER_INVALID_ARG;
    }
        
	// Save to Yaml
    BaseRegData reg_data;
    packSetRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);

    if (!setRegList(reg_data))
    {
		LogProducer::error("reg_manager", "updateReg: fail to set r list");
        return REG_MANAGER_INVALID_ARG;
    }
	
    if (!writeRegDataToYaml(reg_data))
    {
		LogProducer::error("reg_manager", "updateReg: fail to dump r to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    return nvram_ptr_->writeNvram(R_START_ADDR + sizeof(RValue) * reg_ptr->id, (uint8_t*)&reg_ptr->value, sizeof(RValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode RReg::moveReg(int expect_id, int original_id)
{
    if (!isMoveInputValid(expect_id, original_id))
    {
		LogProducer::error("reg_manager", "moveReg: input r expect_id = %d original_id = %d invalid", expect_id, original_id);
        return REG_MANAGER_INVALID_ARG;
    }

    RRegData data;
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

bool RReg::getRegValueById(int id, RValue& r_value)
{
    if (!isRegValid(id))
    {
        LogProducer::error("reg_manager", "getRegValueById: input r id = %d invalid", id);
        return false;
    }

    return nvram_ptr_->readNvram(R_START_ADDR + sizeof(RValue) * id, (uint8_t*)&r_value, sizeof(RValue));
}

bool RReg::updateRegValue(RRegDataIpc* data_ptr)
{
    if (data_ptr == NULL)
    {
		LogProducer::error("reg_manager", "updateRegValue: input r ptr = NULL");
        return false;
    }

    if (!isUpdateInputValid(data_ptr->id) || fabs(data_ptr->value.value) > param_ptr_->r_value_limit_)
    {
        LogProducer::error("reg_manager", "updateRegValue: input r id = %d, data = %.6f invalid", data_ptr->id, data_ptr->value.value);
        return false;
    }

	return nvram_ptr_->writeNvram(R_START_ADDR + sizeof(RValue) * data_ptr->id, (uint8_t*)&data_ptr->value, sizeof(RValue));
}

bool RReg::getRegValue(int id, RRegDataIpc* data_ptr)
{
    if (!isGetInputValid(id))
    {
        LogProducer::error("reg_manager", "getRegValue: input r id = %d invalid", id);
        return false;
    }
    
    RValue r_value;
    nvram_ptr_->readNvram(R_START_ADDR + sizeof(RValue) * id, (uint8_t*)&r_value, sizeof(RValue));
    data_ptr->id = id;
    data_ptr->value = r_value;
    return true;
}

RReg::RReg(): BaseReg(REG_TYPE_INVALID, 0)
{
}

bool RReg::readAllRegDataFromYaml()
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
            LogProducer::error("reg_manager", "readAllRegDataFromYaml: fail to set r list %d", i);
            return false;
        }     
    }

    return true;
}

bool RReg::writeRegDataToYaml(const BaseRegData& base_data)
{
    string reg_path = getRegPath(base_data.id);
    yaml_help_.setParam(reg_path + "/id", base_data.id);
    yaml_help_.setParam(reg_path + "/is_valid", base_data.is_valid);
    yaml_help_.setParam(reg_path + "/name", base_data.name);
    yaml_help_.setParam(reg_path + "/comment", base_data.comment);
    return yaml_help_.dumpParamFile(file_path_modified_.c_str());
}

string RReg::getRegPath(int reg_id)
{
    string id_str;
    stringstream stream;
    stream << reg_id;
    stream >> id_str;
    return (string("R") + id_str);
}

