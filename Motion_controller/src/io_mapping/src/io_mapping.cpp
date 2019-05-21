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
	bypass_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new IoMappingParam();
    sim_ptr_ = new IoSimulation(log_ptr_, param_ptr_);
	bypass_ptr_ = new IoBypass(log_ptr_, param_ptr_);
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
	if(bypass_ptr_ != NULL){
		delete bypass_ptr_;
		bypass_ptr_ = NULL;
	}
}

ErrorCode IoMapping::init(fst_hal::IoManager* io_manager_ptr)
{
	io_manager_ptr_ = io_manager_ptr;

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
	
	ret = bypass_ptr_->init();
	if (ret != SUCCESS){
		ErrorMonitor::instance()->add(ret);
	}

	loadProgramsPath();

	ret = updateMappingFile();
	if (ret != SUCCESS){
		ErrorMonitor::instance()->add(ret);
	}

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

	sprintf(io_map_file_name, "%s/ui_mapping.json", getProgramsPath());
	if (!appendSingleIOMapper(io_map_file_name, "UI"))
	{
		FST_ERROR("Failed to read mapping parameters:%s", io_map_file_name);
		return IO_MAPPING_LOAD_MAP_FILE_FAILED;
	}

	sprintf(io_map_file_name, "%s/uo_mapping.json", getProgramsPath());
	if (!appendSingleIOMapper(io_map_file_name, "UO"))
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
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
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
		return io_manager_ptr_->getBitValue(physics_id, value);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::setDIByBit(uint32_t user_port, uint8_t value)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
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
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
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

		return io_manager_ptr_->getBitValue(physics_id, value);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::setDOByBit(uint32_t user_port, uint8_t value)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
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

		return io_manager_ptr_->setBitValue(physics_id, value);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());

	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::getRIByBit(uint32_t user_port, uint8_t &value)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
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
	{
		PhysicsID physics_id;
		physics_id.number = iter->second;

		return io_manager_ptr_->getBitValue(physics_id, value);
	}

	FST_WARN("invalid strKey=%s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}


ErrorCode IoMapping::setRIByBit(uint32_t user_port, uint8_t value)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
	/* make a string "RO[user_port]" */
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "RI", user_port);

	/* set value if this port is in simulation status */
	if (sim_ptr_->setSimValue(cTemp, value)) return SUCCESS;

	return IO_INVALID_PARAM_ID;
}


ErrorCode IoMapping::getROByBit(uint32_t user_port, uint8_t &value)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
	/* make a string "RO[user_port]" */
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "RO", user_port);

	/* get physics ID by a map. */
	string strKey;
	strKey.assign(cTemp);

	ErrorCode ret = SUCCESS;
	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);

	if (iter != io_mapper_.end())
	{
		PhysicsID physics_id;
		physics_id.number = iter->second;

		return io_manager_ptr_->getBitValue(physics_id, value);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::setROByBit(uint32_t user_port, uint8_t value)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
	// make a string "RO[user_port]"
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "RO", user_port);

	// get physics ID by a map.
	string strKey;
	strKey.assign(cTemp);

	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);

	if (iter != io_mapper_.end())
	{
		PhysicsID physics_id;
		physics_id.number = iter->second;

		return io_manager_ptr_->setBitValue(physics_id, value);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}


ErrorCode IoMapping::getUIByBit(uint32_t user_port, uint8_t &value)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
	/* make a string "UI[user_port]" */
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "UI", user_port);

	/* check if the port is bypass */
	bool data = false;
	if (bypass_ptr_->isBypassUsed(cTemp, data))
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
		return io_manager_ptr_->getBitValue(physics_id, value);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::setUIByBit(uint32_t user_port, uint8_t yes_or_no)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
	// make a string "UI[user_port]"
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "UI", user_port);

	// set the port to be bypass or not.
	if (bypass_ptr_->setBypass(cTemp, yes_or_no)){
		return SUCCESS;
	}

	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::getUOByBit(uint32_t user_port, uint8_t &value)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
	/* make a string "UO[user_port]" */
    char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "UO", user_port);

	/* get physics ID by a map. */
	string strKey;
	strKey.assign(cTemp);

	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end())
	{
		PhysicsID physics_id;
		physics_id.number = iter->second;

		return io_manager_ptr_->getBitValue(physics_id, value);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::setUOByBit(uint32_t user_port, uint8_t value)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
	/* make a string "UO[user_port]" */
    char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "UO", user_port);

	/* get physics ID by a map. */
	string strKey;
	strKey.assign(cTemp);

	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end())
	{
		PhysicsID physics_id;
		physics_id.number = iter->second;

		return io_manager_ptr_->setBitValue(physics_id, value);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());

	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::setDOPulse(uint32_t user_port, double time)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
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

		return io_manager_ptr_->setBitPulse(physics_id, time);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());

	return IO_INVALID_PARAM_ID;
}

ErrorCode IoMapping::setROPulse(uint32_t user_port, double time)
{
	if (user_port > param_ptr_->max_mapping_number_)
	{
		return IO_INVALID_PARAM_ID;
	}
	// make a string "RO[user_port]"
	char cTemp[16] = {};
	sprintf(cTemp, "%s[%d]", "RO", user_port);

	// get physics ID by a map.
	string strKey;
	strKey.assign(cTemp);

	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);

	if (iter != io_mapper_.end())
	{
		PhysicsID physics_id;
		physics_id.number = iter->second;

		return io_manager_ptr_->setBitPulse(physics_id, time);
	}

	FST_WARN("invalid strKey = %s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}


bool IoMapping::isEnableInAutoMode(void)
{
	return param_ptr_->enable_set_io_in_auto_;
}



void IoMapping::loadProgramsPath()
{
	files_manager_data_path_ = "";

	if(getenv("ROBOT_DATA_PREFIX") != NULL)
		files_manager_data_path_ = string(getenv("ROBOT_DATA_PREFIX")); //ROBOT_DATA_PREFIX=/root
	else
		files_manager_data_path_ = "/root"; // for self test only.

	files_manager_data_path_ += "/robot_data/io/io_mapping";
	FST_INFO("io_mapping_load_programs_path: %s", files_manager_data_path_.c_str());
}

char * IoMapping::getProgramsPath()
{
	return (char *)files_manager_data_path_.c_str();
}

bool IoMapping::generateIOInfo(IOMapJsonInfo &objInfo, const char * strIOType)
{
    vector<string> string_array = split(objInfo.module, "/"); //use "/" to split 
    // '3' in "RS485/DIGITAL_IN_OUT_DEVICE/3" or "IMB00401/RS485/1(address)/IN/25(port_offset)" 
    PhysicsID id;

	//add address value
    id.info.address = atoi(string_array[2].c_str());

    //add dev_type value
	if (string_array[0].compare("ModbusServer") == 0)
        id.info.dev_type = fst_hal::DEVICE_TYPE_MODBUS;//=9
	else if (string_array[0].compare(0,3, "IMB") == 0)
	    id.info.dev_type = fst_hal::DEVICE_TYPE_FST_IO;//=2
	else if (string_array[0].compare("VirtualIoBoard") == 0)
	    id.info.dev_type = fst_hal::DEVICE_TYPE_VIRTUAL_IO;//=6
	else 
	    id.info.dev_type = fst_hal::DEVICE_TYPE_INVALID;//=0

    //add port_type value
    char port_type_str[8] = {};
    strcpy(port_type_str, strIOType);
	if (strcasecmp(port_type_str, "DI") == 0)
        id.info.port_type = MessageType_IoType_DI;
    else if (strcasecmp(port_type_str, "DO") == 0)
        id.info.port_type = MessageType_IoType_DO;
    else if (strcasecmp(port_type_str, "RI") == 0)
        id.info.port_type = MessageType_IoType_RI;
    else if (strcasecmp(port_type_str, "RO") == 0)
        id.info.port_type = MessageType_IoType_RO;
	else if (strcasecmp(port_type_str, "UI") == 0)
        id.info.port_type = MessageType_IoType_UI;
    else if (strcasecmp(port_type_str, "UO") == 0)
        id.info.port_type = MessageType_IoType_UO;
	else 
	    id.info.port_type = 0xFF;// need to add invalid value

    //add index value
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

