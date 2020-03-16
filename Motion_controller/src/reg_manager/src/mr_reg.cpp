#include "mr_reg.h"
#include <cstddef>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include "error_code.h"

using namespace std;
using namespace fst_ctrl;
using namespace fst_parameter;


MrReg::MrReg(RegManagerParam* param_ptr):
    BaseReg(REG_TYPE_MR, param_ptr->mr_reg_number_), 
    param_ptr_(param_ptr), 
    file_path_(param_ptr->reg_info_dir_),
    log_ptr_(NULL), 
    nvram_obj_(NVRAM_AREA_LENGTH),
    use_nvram_(0)
{
    log_ptr_ = new fst_log::Logger();
    FST_LOG_INIT("MrRegister");
    file_path_ += param_ptr_->mr_reg_file_name_;
    use_nvram_ += param_ptr_->use_nvram_;
}

MrReg::~MrReg()
{
    if(log_ptr_ != NULL)
    {
        delete log_ptr_;
        log_ptr_ = NULL;
    }

}

ErrorCode MrReg::init()
{
	NVRamMrRegData objNVRamMrRegData ;
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
		FST_ERROR("MrReg::init readAllRegDataFromYaml failed");
        return REG_MANAGER_LOAD_MR_FAILED;
    }
	
	if(use_nvram_ == REG_USE_NVRAM)
	{
		ErrCode error = nvram_obj_.openNvram();
		if(error == FST_NVRAM_OK)
		{
			error = nvram_obj_.isNvramReady();
			if(error == FST_NVRAM_OK)
			{
				for(unsigned int i=0; i < data_list_.size(); i++)
				{
					memset(&objNVRamMrRegData, 0x00, sizeof(NVRamMrRegData));
					nvram_obj_.read((uint8_t*)&objNVRamMrRegData, 
						NVRAM_MR_AREA + i * sizeof(NVRamMrRegData), sizeof(NVRamMrRegData));
								
					// printf("\n MrReg::init: %d .\n", objNVRamMrRegData.value);	
					if(objNVRamMrRegData.id > 0)
					{
                        // comment in 20200106 to fix name and comment lost after reboot
						FST_INFO("MRReg::init: %d:%d", objNVRamMrRegData.id, i);
					    // BaseRegData reg_data;
	    				// packAddRegData(reg_data, objNVRamMrRegData.id, "", "");
					    // if(!setRegList(reg_data))
					    // {
						// 	printf("\n setRegList: %d .\n", objNVRamMrRegData.id);	
					    // //  return REG_MANAGER_INVALID_ARG;
					    // }
						// else 
						// {
							data_list_[i] = objNVRamMrRegData.value ;
						// }
				    }
				}
			}
		}
	}
    return SUCCESS;
}

ErrorCode MrReg::addReg(void* data_ptr)
{
	NVRamMrRegData objNVRamMrRegData ;
    if(data_ptr == NULL)
    {
		FST_ERROR("MrReg::addReg data_ptr == NULL");
        return REG_MANAGER_INVALID_ARG;
    }

    MrRegData* reg_ptr = reinterpret_cast<MrRegData*>(data_ptr);
    if(!isAddInputValid(reg_ptr->id)
        || reg_ptr->value > param_ptr_->mr_value_limit_
        || reg_ptr->value < -param_ptr_->mr_value_limit_)
    {
		FST_ERROR("MrReg::addReg isAddInputValid(reg_ptr->id = %d)", reg_ptr->id);
        return REG_MANAGER_INVALID_ARG;
    }
    BaseRegData reg_data;
    packAddRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);
    if(!setRegList(reg_data))
    {
		FST_ERROR("MrReg::addReg setRegList failed");
        return REG_MANAGER_INVALID_ARG;
    }
    data_list_[reg_data.id] = reg_ptr->value;
	
	if(use_nvram_ == REG_USE_NVRAM)
	{
		memset(&objNVRamMrRegData, 0x00, sizeof(NVRamMrRegData));
		objNVRamMrRegData.id    = reg_data.id;
		objNVRamMrRegData.value = data_list_[reg_data.id];
		nvram_obj_.write((uint8_t*)&objNVRamMrRegData, 
			NVRAM_MR_AREA + reg_data.id * sizeof(NVRamMrRegData), sizeof(NVRamMrRegData));
		usleep(30000);
	}
	// else 
	// {
    if(!writeRegDataToYaml(reg_data, data_list_[reg_data.id]))
    {
        FST_ERROR("MrReg::addReg writeRegDataToYaml failed");
        return REG_MANAGER_REG_FILE_WRITE_FAILED;
    }
	// }
    return SUCCESS;
}

ErrorCode MrReg::deleteReg(int id)
{
	NVRamMrRegData objNVRamMrRegData ;
    if(!isDeleteInputValid(id))
    {
		FST_ERROR("MrReg::deleteReg isDeleteInputValid failed");
        return REG_MANAGER_INVALID_ARG;
    }

    BaseRegData reg_data;
    packDeleteRegData(reg_data, id);
    if(!setRegList(reg_data))
    {
		FST_ERROR("MrReg::deleteReg setRegList failed");
        return REG_MANAGER_INVALID_ARG;
    }
    data_list_[id] = 0;
	
	if(use_nvram_ == REG_USE_NVRAM)
	{
		memset(&objNVRamMrRegData, 0x00, sizeof(NVRamMrRegData));
		objNVRamMrRegData.id    = reg_data.id;
		objNVRamMrRegData.value = 0;
		nvram_obj_.write((uint8_t*)&objNVRamMrRegData, 
			NVRAM_MR_AREA + reg_data.id * sizeof(NVRamMrRegData), sizeof(NVRamMrRegData));
		usleep(30000);
    }
	else
	{
	    if(!writeRegDataToYaml(reg_data, data_list_[reg_data.id]))
	    {
		    FST_ERROR("MrReg::deleteReg writeRegDataToYaml failed");
	        return REG_MANAGER_REG_FILE_WRITE_FAILED;
	    }
	}
    return SUCCESS;
}

ErrorCode MrReg::getReg(int id, void* data_ptr)
{
    if(!isGetInputValid(id))
    {
		FST_ERROR("MrReg::getReg isGetInputValid failed");
        return REG_MANAGER_INVALID_ARG;
    }

    MrRegData* reg_ptr = reinterpret_cast<MrRegData*>(data_ptr);
    BaseRegData reg_data;
    if(!getRegList(id, reg_data))
    {
		FST_ERROR("MrReg::getReg getRegList failed");
        return REG_MANAGER_INVALID_ARG;
    }
	
    reg_ptr->id = reg_data.id;
    reg_ptr->name = reg_data.name;
    reg_ptr->comment = reg_data.comment;
    reg_ptr->value = data_list_[id];
	// printf("\nMrReg::getReg: %d .\n", data_list_[id]);
    return SUCCESS;
}

ErrorCode MrReg::updateReg(void* data_ptr)
{
	NVRamMrRegData objNVRamMrRegData ;
    if(data_ptr == NULL)
    {
		FST_ERROR("MrReg::updateReg data_ptr == NULL");
        return REG_MANAGER_INVALID_ARG;
    }
	MrRegData* reg_ptr = reinterpret_cast<MrRegData*>(data_ptr);

	if(use_nvram_ == REG_USE_NVRAM)
	{
		data_list_[reg_ptr->id] = reg_ptr->value;
		memset(&objNVRamMrRegData, 0x00, sizeof(NVRamMrRegData));
		objNVRamMrRegData.id    = reg_ptr->id;
		objNVRamMrRegData.value = data_list_[reg_ptr->id];
		nvram_obj_.write((uint8_t*)&objNVRamMrRegData, 
			NVRAM_MR_AREA + reg_ptr->id * sizeof(NVRamMrRegData), sizeof(NVRamMrRegData));
		usleep(30000);
		// printf("\nMrReg::updateReg: %d .\n", objNVRamMrRegData.value);
    }
	// Save to Yaml
	if(!isUpdateInputValid(reg_ptr->id)
		|| reg_ptr->value > param_ptr_->mr_value_limit_
		|| reg_ptr->value < -param_ptr_->mr_value_limit_)
	{
		FST_ERROR("MrReg::updateReg isUpdateInputValid failed ");
		return REG_MANAGER_INVALID_ARG;
	}
		
	BaseRegData reg_data;
	packSetRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);
	if(!setRegList(reg_data))
	{
		FST_ERROR("MrReg::updateReg setRegList failed");
		return REG_MANAGER_INVALID_ARG;
	}	 
	data_list_[reg_data.id] = reg_ptr->value;
	if(!writeRegDataToYaml(reg_data, data_list_[reg_data.id]))
	{
		FST_ERROR("MrReg::updateReg writeRegDataToYaml failed");
		return REG_MANAGER_REG_FILE_WRITE_FAILED;
	}
    return SUCCESS;
}

ErrorCode MrReg::moveReg(int expect_id, int original_id)
{
    if(!isMoveInputValid(expect_id, original_id))
    {
		FST_ERROR("MrReg::moveReg isMoveInputValid failed");
        return REG_MANAGER_INVALID_ARG;
    }

    MrRegData data;
    ErrorCode error_code;
    error_code = getReg(original_id, (void*)&data);
    if(error_code != SUCCESS)
    {
		FST_ERROR("MrReg::moveReg getReg failed");
        return error_code;
    }
    error_code = deleteReg(original_id);
    if(error_code != SUCCESS)
    {
		FST_ERROR("MrReg::moveReg deleteReg failed");
        return error_code;
    }
    data.id = expect_id;
    return addReg((void*)&data);
}

void* MrReg::getRegValueById(int id)
{
    if(!isRegValid(id))
    {
        return NULL;
    }
        
    return (void*)&data_list_[id];
}

bool MrReg::updateRegValue(MrRegDataIpc* data_ptr)
{
	NVRamMrRegData objNVRamMrRegData ;
    if(data_ptr == NULL)
    {
		FST_ERROR("MrReg::updateRegValue data_ptr == NULL");
        return false;
    }

	
	if(use_nvram_ == REG_USE_NVRAM)
	{
	    data_list_[data_ptr->id] = data_ptr->value;
		memset(&objNVRamMrRegData, 0x00, sizeof(objNVRamMrRegData));
		objNVRamMrRegData.id    = data_ptr->id;
		objNVRamMrRegData.value = data_list_[data_ptr->id];
		nvram_obj_.write((uint8_t*)&objNVRamMrRegData, 
			NVRAM_MR_AREA + data_ptr->id * sizeof(objNVRamMrRegData), sizeof(objNVRamMrRegData));
		usleep(30000);
		// printf("\n MrReg::updateRegValue: %d .\n", objNVRamMrRegData.value);
	    return true ;
    }
	else
	{
	    if(!isUpdateInputValid(data_ptr->id)
	        || data_ptr->value > param_ptr_->mr_value_limit_
	        || data_ptr->value < -param_ptr_->mr_value_limit_)
	    {
	        return false;
	    }
	        
	    BaseRegData* reg_data_ptr = getBaseRegDataById(data_ptr->id);
	    if(reg_data_ptr == NULL)
	    {
	        return false;
	    }
	    data_list_[data_ptr->id] = data_ptr->value;
    	return writeRegDataToYaml(*reg_data_ptr, data_list_[data_ptr->id]);
    }
}

bool MrReg::getRegValue(int id, MrRegDataIpc* data_ptr)
{
    if(!isGetInputValid(id))
    {
        return false;
    }
    
    data_ptr->id = id;
    data_ptr->value = data_list_[id];
    return true;
}

MrReg::MrReg():
    BaseReg(REG_TYPE_INVALID, 0),
        nvram_obj_(NVRAM_AREA_LENGTH)
		
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
    for(int i = 1; i < static_cast<int>(data_list_.size()); ++i)
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
    for(int i = 1; i < static_cast<int>(data_list_.size()); ++i)
    {
        std::string reg_path = getRegPath(i);
        BaseRegData base_data;
        // std::string name, comment;
        yaml_help_.getParam(reg_path + "/id", base_data.id);
        yaml_help_.getParam(reg_path + "/is_valid", base_data.is_valid);
        yaml_help_.getParam(reg_path + "/name", base_data.name);
        yaml_help_.getParam(reg_path + "/comment", base_data.comment);
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

