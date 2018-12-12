/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_mapping.cpp
Author:     Feng.Wu Jiaming.Lu 
Create:     15-Obt-2018
Modify:     15-Obt-2018
Summary:    implement io mapping
**********************************************/
#include "io_mapping.h"
#include "error_monitor.h"

using namespace fst_ctrl;
using namespace fst_hal;
using namespace fst_base;

IoMapping::IoMapping():
    log_ptr_(NULL),
	param_ptr_(NULL),
	sim_ptr_(NULL),
	io_dev_ptr_(NULL),
	modbus_manager_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new IoMappingParam();
    sim_ptr_ = new IoSimulation(log_ptr_, param_ptr_);
    FST_LOG_INIT("io_mapping");
}

IoMapping::~IoMapping()
{
    if(log_ptr_ != NULL){
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if(param_ptr_ != NULL){
        delete param_ptr_;
        param_ptr_ = NULL;
    }
	if(sim_ptr_ != NULL){
		delete sim_ptr_;
		sim_ptr_ = NULL;
	}
    if(io_dev_ptr_ != NULL){
        delete io_dev_ptr_;
        io_dev_ptr_ = NULL;
    }
}

ErrorCode IoMapping::init(fst_hal::FstIoDevice* io_device_ptr,
	fst_hal::ModbusManager* modbus_manager)
{
	io_dev_ptr_ = io_device_ptr;

	if(!param_ptr_->loadParam()){
		FST_INFO("Failed to load io_mapping component parameters");
		ErrorMonitor::instance()->add(IO_MAPPING_LOAD_PARAM_FAILED);
	} else {
		FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
	}

	ErrorCode ret = sim_ptr_->init();
	if (ret != SUCCESS){
		ErrorMonitor::instance()->add(ret);
	}

	loadProgramsPath();

	ret = updateMappingFile();
	if (ret != SUCCESS){
		ErrorMonitor::instance()->add(ret);
	}

	modbus_manager_ = modbus_manager;
	return SUCCESS;
}

ErrorCode IoMapping::updateMappingFile()
{
	io_mapper_.clear();

	/*  to do..., check file faulty first, pass if no files. */

	char io_map_file_name[128] = {};
	sprintf(io_map_file_name, "%s/di_mapping.json", getProgramsPath());
	if (!appendSingleIOMapper(io_map_file_name, "DI"))
	{
		FST_ERROR("Failed to read mapping parameters:%s", io_map_file_name);
		return IO_MAPPING_LOAD_MAP_FILE_FAILED;
	}

	sprintf(io_map_file_name, "%s/do_mapping.json", getProgramsPath());
	if (!appendSingleIOMapper(io_map_file_name, "DO"))
	{
		FST_ERROR("Failed to read mapping parameters", io_map_file_name);
		return IO_MAPPING_LOAD_MAP_FILE_FAILED;
	}

	sprintf(io_map_file_name, "%s/ri_mapping.json", getProgramsPath());
	if (!appendSingleIOMapper(io_map_file_name, "RI"))
	{
		FST_ERROR("Failed to read mapping parameters", io_map_file_name);
		return IO_MAPPING_LOAD_MAP_FILE_FAILED;
	}

	sprintf(io_map_file_name, "%s/ro_mapping.json", getProgramsPath());
	if (!appendSingleIOMapper(io_map_file_name, "RO"))
	{
		FST_ERROR("Failed to read mapping parameters", io_map_file_name);
		return IO_MAPPING_LOAD_MAP_FILE_FAILED;
	}

	FST_INFO("Success update io_mapping files");
	return SUCCESS;
}

ErrorCode IoMapping::updateSimFile()
{
	return sim_ptr_->updateSimFile();
}


ErrorCode IoMapping::getDIByBit(uint32_t user_port, uint8_t &value)
{
	/* make a string "DI[user_port]" */
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "DI", user_port);

	/* check if the port is simUsed */
	bool data = false;
	if (sim_ptr_->isSimUsed(cTemp, data))
	{
		value = data;
		return SUCCESS;
	}

	/* get physics ID by a map. */
	string strKey;
	strKey.assign(cTemp);

	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end())
	{
		PhysicsID physics_id;
		physics_id.number = iter->second;

		if (physics_id.info.dev_type == DEVICE_TYPE_MODBUS)
		{
			if (modbus_manager_ == NULL || !modbus_manager_->isValid())
			{
				return MODBUS_INVALID;
			}

			int server_id = 0;
			return modbus_manager_->readDiscreteInputs(server_id, physics_id.info.port, 1, &value);
		}

		return io_dev_ptr_->getDIOByBit(iter->second, value);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::setDIByBit(uint32_t user_port, uint8_t value)
{
	// make a string "DO[user_port]"
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "DI", user_port);

	// set value if this port is in simulation status
	if (sim_ptr_->setSimValue(cTemp, value)){
		return SUCCESS;
	}

	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::getDOByBit(uint32_t user_port, uint8_t &value)
{
	/* make a string "DO[user_port]" */
    char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "DO", user_port);

	/* get physics ID by a map. */
	string strKey;
	strKey.assign(cTemp);

	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end())
	{
		PhysicsID physics_id;
		physics_id.number = iter->second;
		
		if (physics_id.info.dev_type == DEVICE_TYPE_MODBUS)
		{
			if (modbus_manager_ == NULL || !modbus_manager_->isValid())
			{
				return MODBUS_INVALID;
			}

			int server_id = 0;
			return modbus_manager_->readCoils(server_id, physics_id.info.port, 1, &value);
		}

		return io_dev_ptr_->getDIOByBit(iter->second, value);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::setDOByBit(uint32_t user_port, uint8_t value)
{
	/* make a string "DO[user_port]" */
    char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "DO", user_port);

	/* get physics ID by a map. */
	string strKey;
	strKey.assign(cTemp);

	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end())
	{
		PhysicsID physics_id;
		physics_id.number = iter->second;
		
		if (physics_id.info.dev_type == DEVICE_TYPE_MODBUS)
		{
			if (modbus_manager_ == NULL || !modbus_manager_->isValid())
			{
				return MODBUS_INVALID;
			}

			int server_id = 0;
			return modbus_manager_->writeCoils(server_id, physics_id.info.port, 1, &value);
		}

		return io_dev_ptr_->setDIOByBit(iter->second, value);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());

	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::getRIByBit(uint32_t user_port, uint8_t &value)
{
	/* make a string "RI[user_port]" */
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "RI", user_port);

	/* check if the port is simUsed */
	bool data = false;
	if (sim_ptr_->isSimUsed(cTemp, data))
	{
		value = data;
		return SUCCESS;
	}

	// get physics ID by a map.
	string  strKey;
	strKey.assign(cTemp);

	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end())
		return io_dev_ptr_->getDIOByBit(iter->second, value);

	FST_WARN("invalid strKey=%s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}


ErrorCode IoMapping::setRIByBit(uint32_t user_port, uint8_t value)
{
	/* make a string "RO[user_port]" */
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "RI", user_port);

	/* set value if this port is in simulation status */
	if (sim_ptr_->setSimValue(cTemp, value)) return SUCCESS;

	return IO_INVALID_PARAM_ID;
}


ErrorCode IoMapping::getROByBit(uint32_t user_port, uint8_t &value)
{
	/* make a string "RO[user_port]" */
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "RO", user_port);

	/* get physics ID by a map. */
	string strKey;
	strKey.assign(cTemp);

	ErrorCode ret = SUCCESS;
	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);

	if (iter != io_mapper_.end())
		return io_dev_ptr_->getDIOByBit(iter->second, value);

	FST_WARN("invalid strKey = %s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::setROByBit(uint32_t user_port, uint8_t value)
{
	// make a string "RO[user_port]"
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "RO", user_port);

	// get physics ID by a map.
	string strKey;
	strKey.assign(cTemp);

	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);

	if (iter != io_mapper_.end())
		return io_dev_ptr_->setDIOByBit(iter->second, value);

	FST_WARN("invalid strKey = %s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

void IoMapping::loadProgramsPath()
{
	files_manager_data_path_ = "";

	if(getenv("ROBOT_DATA_PREFIX") != NULL)
		files_manager_data_path_ = string(getenv("ROBOT_DATA_PREFIX")); //ROBOT_DATA_PREFIX=/root
	else
		files_manager_data_path_ = "/home/fst/fortest"; // for self test only.

	files_manager_data_path_ += "/robot_data/io/io_mapping";
	FST_INFO("io_mapping_load_programs_path: %s", files_manager_data_path_.c_str());
}

char * IoMapping::getProgramsPath()
{
	return (char *)files_manager_data_path_.c_str();
}

bool IoMapping::generateIOInfo(IOMapJsonInfo &objInfo, const char * strIOType)
{
    /* //use "/" to split */
    vector<string> result = split(objInfo.module, "/");

    /* '3' in "RS485/DIGITAL_IN_OUT_DEVICE/3" or 
    "FR-P8A/RS485/1(address)/IN/25(port_offset)" */
    PhysicsID id;
    id.info.address = atoi(result[2].c_str());

    char cUpperType[8] = {};
    strcpy(cUpperType, strIOType);

    if (result[0].compare("ModbusServer") == 0)
    {
        id.info.dev_type = fst_hal::DEVICE_TYPE_MODBUS;

        if (strcasecmp(cUpperType, "DI") == 0) id.info.port_type = IO_TYPE_DI;
        else if (strcasecmp(cUpperType, "DO") == 0) id.info.port_type = IO_TYPE_DO;
        else;
    }
    else
    {
        id.info.dev_type = fst_hal::DEVICE_TYPE_FST_IO;//=2

        if (strcasecmp(cUpperType, "DI") == 0)
            id.info.port_type = IO_TYPE_DI;
        else if (strcasecmp(cUpperType, "DO") == 0)
            id.info.port_type = IO_TYPE_DO;
        else if (strcasecmp(cUpperType, "RI") == 0)
            id.info.port_type = IO_TYPE_RI;
        else if (strcasecmp(cUpperType, "RO") == 0)
            id.info.port_type = IO_TYPE_RO;
        else;
    }

    id.info.port = objInfo.index;

    for (int i = objInfo.from; i <= objInfo.to; i++)
    {
        char cTemp[16] = {};
        sprintf(cTemp, "%s[%d]", strIOType, i);

        string strKey;
        strKey.assign(cTemp);
        FST_INFO("iomapping:strkey = %s  , physics id=%x", strKey.c_str(), id.number);

        io_mapper_.insert(map<string, uint32_t>::value_type(strKey, id.number));
        id.info.port++;
    }

    return true;
}

bool IoMapping::parseIOObject(cJSON *jsonIObject, const char * strIOType)
{
	cJSON *child = jsonIObject->child;
	IOMapJsonInfo objInfo;

	while (child)
	{
		if(strcmp(child->string, "from") == 0)
		{
			objInfo.from = child->valueint;
		}
		else if(strcmp(child->string, "index") == 0)
		{
			objInfo.index = child->valueint;
		}
		else if(strcmp(child->string, "module") == 0)
		{
			strcpy(objInfo.module, child->valuestring);
		}
		else if(strcmp(child->string, "to") == 0)
		{
			objInfo.to = child->valueint;
		}
		else{}

		child=child->next;
	}
	
	generateIOInfo(objInfo, strIOType);
	return true;
}

bool IoMapping::parseIO(cJSON *jsonDI, const char * strIOType)
{
	cJSON *child = jsonDI->child;

	while (child) 
	{
		switch ((child->type) & 0xff)
		{
			case cJSON_Number:	
				break;
			case cJSON_String:	
				break;
			case cJSON_Object:
        	    /* the second catelog, such as [{},{}]. */
				parseIOObject(child, strIOType);
				break;
			default:;
		}
		child=child->next;
	}
	return true;
}

bool IoMapping::parseIOMap(char* data, const char* strIOType)
{
    cJSON* json = cJSON_Parse(data);
    if (json == NULL) return false;

    cJSON *child = json->child;
    while(child)
    {
        switch ((child->type) & 0xff)
        {
            case cJSON_True:
                break;
            case cJSON_Number:
                break;
            case cJSON_String:
                break;
            case cJSON_Array:
                parseIO(child, strIOType);
                break;
            case cJSON_Object:
                break;
            default:;
        }
        child = child->next;
    }

    cJSON_Delete(json);
    return true;
}

bool IoMapping::printIOMapper()
{
	printf("\t\tobjThreadCntrolBlock->io_mapper_ has %d elements \n", 
		io_mapper_.size());

	map<string, uint32_t>::iterator it;
	for (it = io_mapper_.begin(); it != io_mapper_.end(); ++it)
		printf("\t\t%s :: 0x%x \n", it->first.c_str(), it->second);

	return true;
}

bool IoMapping::appendSingleIOMapper(char *filename, const char * strIOType)
{
	FILE *f = fopen(filename,"rb");

	if(f != NULL)
	{
        // read file to data, then parse data.
	    fseek(f, 0, SEEK_END); 
		long len = ftell(f);
		fseek(f,0,SEEK_SET);

	    char* data = (char*)malloc(len + 1);
		fread(data, 1, len, f);
		fclose(f);

		parseIOMap(data, strIOType);
		free(data);
		return true;
	} 

	return false;
}

vector<string> IoMapping::split(string str,string pattern)
{
	str += pattern;

	vector<string> result;
	for(int i = 0; i < str.size(); i++)
	{
		string::size_type pos = str.find(pattern, i);

		if((int)pos < str.size())
		{
			string s = str.substr(i, pos-i);
			result.push_back(s);
			i = pos + pattern.size() - 1;
		}
	}

	return result;
}

