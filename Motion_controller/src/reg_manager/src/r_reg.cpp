#include "r_reg.h"
#include <cstddef>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include "error_code.h"


using namespace std;
using namespace fst_ctrl;
using namespace fst_parameter;


RReg::RReg(RegManagerParam* param_ptr):
    BaseReg(REG_TYPE_R, param_ptr->r_reg_number_), 
        param_ptr_(param_ptr), file_path_(param_ptr->reg_info_dir_),
        nvram_obj_(NVRAM_AREA_LENGTH)
{
    file_path_ += param_ptr_->r_reg_file_name_;
    use_nvram_ = param_ptr_->use_nvram_;
}

RReg::~RReg()
{

}

ErrorCode RReg::init()
{
	NVRamRRegData objNVRamRRegData ;
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
        return REG_MANAGER_LOAD_R_FAILED;
    }
	
	if(use_nvram_ == REG_USE_NVRAM)
	{
		ErrCode error = nvram_obj_.openNvram();
		if(error == FST_NVRAM_OK)
		{
			for(unsigned int i=0; i < data_list_.size(); i++)
			{
				memset(&objNVRamRRegData, 0x00, sizeof(NVRamRRegData));
				nvram_obj_.read((uint8_t*)&objNVRamRRegData, 
					NVRAM_R_AREA + i * sizeof(NVRamRRegData), sizeof(NVRamRRegData));
				
			//	printf("\nRReg::init: %f .\n", objNVRamRRegData.value);
				data_list_[i] = objNVRamRRegData.value ;
			}
		}
	}
    
    return SUCCESS;
}

ErrorCode RReg::addReg(void* data_ptr)
{
	NVRamRRegData objNVRamRRegData ;
    if(data_ptr == NULL)
    {
        return REG_MANAGER_INVALID_ARG;
    }

    RRegData* reg_ptr = reinterpret_cast<RRegData*>(data_ptr);
    if(!isAddInputValid(reg_ptr->id)
        || reg_ptr->value > param_ptr_->r_value_limit_
        || reg_ptr->value < -param_ptr_->r_value_limit_)
    {
        return REG_MANAGER_INVALID_ARG;
    }
    BaseRegData reg_data;
    packAddRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);
    if(!setRegList(reg_data))
    {
        return REG_MANAGER_INVALID_ARG;
    }
    data_list_[reg_data.id] = reg_ptr->value;
	if(use_nvram_ == REG_USE_NVRAM)
	{
		memset(&objNVRamRRegData, 0x00, sizeof(NVRamRRegData));
		objNVRamRRegData.id    = reg_data.id;
		objNVRamRRegData.value = data_list_[reg_data.id];
		nvram_obj_.write((uint8_t*)&objNVRamRRegData, 
			NVRAM_R_AREA + reg_data.id * sizeof(NVRamRRegData), sizeof(NVRamRRegData));
		usleep(30000);
	}
	else
	{
	    if(!writeRegDataToYaml(reg_data, data_list_[reg_data.id]))
	    {
	        return REG_MANAGER_REG_FILE_WRITE_FAILED;
	    }
	}
    return SUCCESS;
}

ErrorCode RReg::deleteReg(int id)
{
	NVRamRRegData objNVRamRRegData ;
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
    data_list_[id] = 0;
	
	if(use_nvram_ == REG_USE_NVRAM)
	{
		memset(&objNVRamRRegData, 0x00, sizeof(NVRamRRegData));
		objNVRamRRegData.id    = reg_data.id;
		objNVRamRRegData.value = 0;
		nvram_obj_.write((uint8_t*)&objNVRamRRegData, 
			NVRAM_R_AREA + reg_data.id * sizeof(NVRamRRegData), sizeof(NVRamRRegData));
		usleep(30000);
	}
	else
	{
	    if(!writeRegDataToYaml(reg_data, data_list_[reg_data.id]))
	    {
	        return REG_MANAGER_REG_FILE_WRITE_FAILED;
	    }
	}
    return SUCCESS;
}

ErrorCode RReg::getReg(int id, void* data_ptr)
{
    if(!isGetInputValid(id))
    {
        return REG_MANAGER_INVALID_ARG;
    }

    RRegData* reg_ptr = reinterpret_cast<RRegData*>(data_ptr);
    BaseRegData reg_data;
    if(!getRegList(id, reg_data))
    {
        return REG_MANAGER_INVALID_ARG;
    }
    reg_ptr->id = reg_data.id;
    reg_ptr->name = reg_data.name;
    reg_ptr->comment = reg_data.comment;
    reg_ptr->value = data_list_[id];
	// printf("\nRReg::getReg: %d .\n", data_list_[id]);
    return SUCCESS;
}

ErrorCode RReg::updateReg(void* data_ptr)
{
	NVRamRRegData objNVRamRRegData ;
    if(data_ptr == NULL)
    {
        return REG_MANAGER_INVALID_ARG;
    }
	RRegData* reg_ptr = reinterpret_cast<RRegData*>(data_ptr);
	
	if(use_nvram_ == REG_USE_NVRAM)
	{
	    data_list_[reg_ptr->id] = reg_ptr->value;
		
		memset(&objNVRamRRegData, 0x00, sizeof(NVRamRRegData));
		objNVRamRRegData.id    = reg_ptr->id;
		objNVRamRRegData.value = data_list_[reg_ptr->id];
		nvram_obj_.write((uint8_t*)&objNVRamRRegData, 
			NVRAM_R_AREA + reg_ptr->id * sizeof(NVRamRRegData), sizeof(NVRamRRegData));
		usleep(30000);
		printf("\nRReg::updateReg: %f .\n", objNVRamRRegData.value);
	}
	else
	{
	    if(!isUpdateInputValid(reg_ptr->id)
	        || reg_ptr->value > param_ptr_->r_value_limit_
	        || reg_ptr->value < -param_ptr_->r_value_limit_)
	    {
	        return REG_MANAGER_INVALID_ARG;
	    }
	        
	    BaseRegData reg_data;
	    packSetRegData(reg_data, reg_ptr->id, reg_ptr->name, reg_ptr->comment);
	    if(!setRegList(reg_data))
	    {
	        return REG_MANAGER_INVALID_ARG;
	    }
	    data_list_[reg_data.id] = reg_ptr->value;
		
	    if(!writeRegDataToYaml(reg_data, data_list_[reg_data.id]))
	    {
	        return REG_MANAGER_REG_FILE_WRITE_FAILED;
	    }
	}
    return SUCCESS;
}

ErrorCode RReg::moveReg(int expect_id, int original_id)
{
    if(!isMoveInputValid(expect_id, original_id))
    {
        return REG_MANAGER_INVALID_ARG;
    }

    RRegData data;
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

void* RReg::getRegValueById(int id)
{
    if(!isRegValid(id))
    {
        return NULL;
    }
        
    return (void*)&data_list_[id];
}

bool RReg::updateRegValue(RRegDataIpc* data_ptr)
{
	NVRamRRegData objNVRamRRegData ;
    if(data_ptr == NULL)
    {
        return false;
    }
	
	if(use_nvram_ == REG_USE_NVRAM)
	{
		printf("use_nvram_ data_list_[%d] = %f\n", data_ptr->id, data_ptr->value);
	    data_list_[data_ptr->id] = data_ptr->value;
		
		memset(&objNVRamRRegData, 0x00, sizeof(NVRamRRegData));
		objNVRamRRegData.id    = data_ptr->id;
		objNVRamRRegData.value = data_list_[data_ptr->id];
		nvram_obj_.write((uint8_t*)&objNVRamRRegData, 
			NVRAM_R_AREA + data_ptr->id * sizeof(NVRamRRegData), sizeof(NVRamRRegData));
		usleep(30000);
	//	printf("\nRReg::updateRegValue: %f .\n", objNVRamRRegData.value);
	}
	else
	{
	    printf("use_nvram_ = %d and getListSize() = %d \n", use_nvram_, getListSize());
	    if(!isUpdateInputValid(data_ptr->id)
	        || data_ptr->value > param_ptr_->r_value_limit_
	        || data_ptr->value < -param_ptr_->r_value_limit_)
	    {
			printf("return false :: isUpdateInputValid() = %d\n", data_ptr->id);
	        return false;
	    }
	    
		printf("getListSize() = %d\n", getListSize());
	    BaseRegData* reg_data_ptr = getBaseRegDataById(data_ptr->id);
	    if(reg_data_ptr == NULL)
	    {
			printf("return false :: getListSize() = %d\n", getListSize());
	        return false;
	    }
	    data_list_[data_ptr->id] = data_ptr->value;
		
    	return writeRegDataToYaml(*reg_data_ptr, data_list_[data_ptr->id]);
	}
}

bool RReg::getRegValue(int id, RRegDataIpc* data_ptr)
{
    if(!isGetInputValid(id))
    {
        return false;
    }
    
    data_ptr->id = id;
    data_ptr->value = data_list_[id];
	// printf("\nRReg::getRegValue: %f .\n", data_list_[id]);
    return true;
}

RReg::RReg():
    BaseReg(REG_TYPE_INVALID, 0),
    nvram_obj_(NVRAM_AREA_LENGTH)
{

}

bool RReg::createYaml()
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

bool RReg::readAllRegDataFromYaml()
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

bool RReg::writeRegDataToYaml(const BaseRegData& base_data, const double& data)
{
    std::string reg_path = getRegPath(base_data.id);
    yaml_help_.setParam(reg_path + "/id", base_data.id);
    yaml_help_.setParam(reg_path + "/is_valid", base_data.is_valid);
    yaml_help_.setParam(reg_path + "/name", base_data.name);
    yaml_help_.setParam(reg_path + "/comment", base_data.comment);
    yaml_help_.setParam(reg_path + "/value", data);
    return yaml_help_.dumpParamFile(file_path_.c_str());
}

std::string RReg::getRegPath(int reg_id)
{
    std::string id_str;
    std::stringstream stream;
    stream << reg_id;
    stream >> id_str;
    return (std::string("R") + id_str);
}

