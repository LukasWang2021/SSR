#include "sr_reg.h"
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


SrReg::SrReg(RegManagerParam* param_ptr, NvramHandler* nvram_ptr):
    BaseReg(REG_TYPE_SR, param_ptr->sr_reg_number_), 
    param_ptr_(param_ptr), 
    nvram_ptr_(nvram_ptr),
    file_path_(REG_DIR),
    file_path_modified_(REG_DIR_MODIFIED)
{
    file_path_ += param_ptr_->sr_reg_file_name_;
    file_path_modified_ += param_ptr_->sr_reg_file_name_;
}

SrReg::~SrReg()
{
}

ErrorCode SrReg::init()
{
    if (!copyFile(file_path_.c_str(), file_path_modified_.c_str()))
    {
        LogProducer::error("reg_manager", "Fail to copy %s to %s", file_path_.c_str(), file_path_modified_.c_str());
        return REG_MANAGER_LOAD_SR_FAILED;
    }

    if (!readAllRegDataFromYaml())
    {
		LogProducer::error("reg_manager", "Fail to load sr from yaml");
        return REG_MANAGER_LOAD_SR_FAILED;
    }
    
    return SUCCESS;
}

ErrorCode SrReg::addReg(void* data_ptr)
{
    if (data_ptr == NULL)
    {
        LogProducer::error("reg_manager", "addReg: input sr ptr = NULL");
        return REG_MANAGER_INVALID_ARG;
    }

    SrRegData* reg_ptr = reinterpret_cast<SrRegData*>(data_ptr);
    if (!isAddInputValid(reg_ptr->id) || static_cast<int>(strlen(reg_ptr->value.value)) > param_ptr_->sr_value_limit_)
    {
		LogProducer::error("reg_manager", "addReg: input sr id = %d or string invalid)", reg_ptr->id);
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packAddRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);

    if (!setRegList(reg_data))
    {
		LogProducer::error("reg_manager", "addReg: fail to set sr list");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
		LogProducer::error("reg_manager", "addReg: fail to dump sr to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    reg_ptr->value.value[STRING_REG_MAX_LENGTH - 1] = '\0';
    return nvram_ptr_->writeNvram(SR_START_ADDR + sizeof(SrValue) * reg_ptr->id, (uint8_t*)reg_ptr->value.value, sizeof(SrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode SrReg::deleteReg(int id)
{
    if (!isDeleteInputValid(id))
    {
		LogProducer::error("reg_manager", "deleteReg: input sr id = %d invalid", id);
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packDeleteRegData(reg_data, id);

    if (!setRegList(reg_data))
    {
        LogProducer::error("reg_manager", "deleteReg: fail to dump sr to yaml");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
		LogProducer::error("reg_manager", "deleteReg: fail to dump sr to yaml");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }

    SrValue sr_value;
    memset(&sr_value, 0, sizeof(sr_value));
    return nvram_ptr_->writeNvram(SR_START_ADDR + sizeof(SrValue) * id, (uint8_t*)sr_value.value, sizeof(SrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode SrReg::getReg(int id, void* data_ptr)
{
    if (!isGetInputValid(id))
    {
		LogProducer::error("reg_manager", "getReg: input sr id = %d invalid", id);
        return REG_MANAGER_INVALID_ARG;
    }

    SrRegData* reg_ptr = reinterpret_cast<SrRegData*>(data_ptr);
    BaseRegData reg_data;

    if (!getRegList(id, reg_data))
    {
		LogProducer::error("reg_manager", "getReg: fail to get sr list");
        return REG_MANAGER_INVALID_ARG;
    }

    SrValue sr_value;
    memset(&sr_value, 0, sizeof(sr_value));
    nvram_ptr_->readNvram(SR_START_ADDR + sizeof(SrValue) * id, (uint8_t*)sr_value.value, sizeof(SrValue));
    reg_ptr->id = reg_data.id;
    reg_ptr->name = reg_data.name;
    reg_ptr->comment = reg_data.comment;
    reg_ptr->value = sr_value;
    return SUCCESS;
}

ErrorCode SrReg::updateReg(void* data_ptr)
{
    if (data_ptr == NULL)
    {
		LogProducer::error("reg_manager", "updateReg: input sr ptr = NULL");
        return REG_MANAGER_INVALID_ARG;
    }

    SrRegData* reg_ptr = reinterpret_cast<SrRegData*>(data_ptr);
    if (!isUpdateInputValid(reg_ptr->id) || static_cast<int>(strlen(reg_ptr->value.value)) > param_ptr_->sr_value_limit_)
    {
        LogProducer::error("reg_manager", "updateReg: input sr id = %d or data invalid", reg_ptr->id);
        return REG_MANAGER_INVALID_ARG;
    }
        
    BaseRegData reg_data;
    packSetRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);

    if (!setRegList(reg_data))
    {
		LogProducer::error("reg_manager", "updateReg: fail to set sr list");
        return REG_MANAGER_INVALID_ARG;
    }

    if (!writeRegDataToYaml(reg_data))
    {
		LogProducer::error("reg_manager", "updateReg: fail to dump sr to yaml");
        return REG_MANAGER_INVALID_ARG;
    }
    
    reg_ptr->value.value[STRING_REG_MAX_LENGTH - 1] = '\0';
    return nvram_ptr_->writeNvram(SR_START_ADDR + sizeof(SrValue) * reg_ptr->id, (uint8_t*)reg_ptr->value.value, sizeof(SrValue)) ? SUCCESS : REG_MANAGER_OPERATE_NVRAM_FAILED;
}

ErrorCode SrReg::moveReg(int expect_id, int original_id)
{
    if (!isMoveInputValid(expect_id, original_id))
    {
		LogProducer::error("reg_manager", "moveReg: input sr expect_id = %d original_id = %d invalid", expect_id, original_id);
        return REG_MANAGER_INVALID_ARG;
    }

    SrRegData data;
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

bool SrReg::getRegValueById(int id, SrValue& sr_value)
{
    //printf("=====>3 enter SrReg::getRegValueById(int id, SrValue& sr_value),  id=%d\n",id);
    if (!isRegValid(id))
    {
        LogProducer::error("reg_manager", "getRegValueById: input sr id = %d invalid", id);
        //printf("=====>3_1 enter SrReg::getRegValueById(int id, SrValue& sr_value),  id=%d is invalid\n",id);
        return false;
    }
    return nvram_ptr_->readNvram(SR_START_ADDR + sizeof(SrValue) * id, (uint8_t*)sr_value.value, sizeof(SrValue));
}

bool SrReg::updateRegValue(SrRegDataIpc* data_ptr)
{
    //printf("=====>3 enter SrReg::updateRegValue(SrRegDataIpc* data_ptr)  data_ptr->id=%d, data_ptr->value.value=%s\n", data_ptr->id, data_ptr->value.value);
    if (data_ptr == NULL)
    {
		LogProducer::error("reg_manager", "updateRegValue: input sr ptr = NULL");
        //printf("=====>3_1 enter SrReg::updateRegValue(SrRegDataIpc* data_ptr)  ERROR:data_ptr==null\n");
        return false;
    }
    if (!isUpdateInputValid(data_ptr->id) || static_cast<int>(strlen(data_ptr->value.value)) > param_ptr_->sr_value_limit_)
    {
        LogProducer::error("reg_manager", "updateRegValue: input sr id = %d or invalid", data_ptr->id);
        /*
        if(!isUpdateInputValid(data_ptr->id))
        {
            printf("=====>3_2 data_ptr->id invalid\n");
        }
        if(static_cast<int>(strlen(data_ptr->value.value)) > param_ptr_->sr_value_limit_)
        {
            printf("=====>3_3 strlen(data_ptr->value.value) > param_ptr_->sr_value_limit_\n");
        }*/
        return false;
    }
    data_ptr->value.value[STRING_REG_MAX_LENGTH - 1] = '\0';
    return nvram_ptr_->writeNvram(SR_START_ADDR + sizeof(SrValue) * data_ptr->id, (uint8_t*)data_ptr->value.value, sizeof(SrValue));
}

bool SrReg::getRegValue(int id, SrRegDataIpc* data_ptr)
{
    if (!isGetInputValid(id))
    {
        LogProducer::error("reg_manager", "getRegValue: input sr id = %d invalid", id);
        return false;
    }
    
    SrValue sr_value;
    nvram_ptr_->readNvram(SR_START_ADDR + sizeof(SrValue) * id, (uint8_t*)sr_value.value, sizeof(SrValue));

    if (sr_value.value[STRING_REG_MAX_LENGTH - 1] != '\0') sr_value.value[STRING_REG_MAX_LENGTH - 1] = '\0';
    data_ptr->id = id;
    data_ptr->value = sr_value;
    return true;
}

SrReg::SrReg(): BaseReg(REG_TYPE_INVALID, 0)
{
}

bool SrReg::readAllRegDataFromYaml()
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
            LogProducer::error("reg_manager", "readAllRegDataFromYaml: fail to set sr list %d", i);
            return false;
        }
    }

    return true;
}

bool SrReg::writeRegDataToYaml(const BaseRegData& base_data)
{
    string reg_path = getRegPath(base_data.id);
    yaml_help_.setParam(reg_path + "/id", base_data.id);
    yaml_help_.setParam(reg_path + "/is_valid", base_data.is_valid);
    yaml_help_.setParam(reg_path + "/name", base_data.name);
    yaml_help_.setParam(reg_path + "/comment", base_data.comment);
    return yaml_help_.dumpParamFile(file_path_modified_.c_str());
}

string SrReg::getRegPath(int reg_id)
{
    string id_str;
    stringstream stream;
    stream << reg_id;
    stream >> id_str;
    return (string("SR") + id_str);
}

