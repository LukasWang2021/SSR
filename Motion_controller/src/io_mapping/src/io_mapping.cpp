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
	io_dev_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new IoMappingParam();
    sim_ptr_ = new IoSimulation(log_ptr_, param_ptr_);
    FST_LOG_INIT("io_mapping");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
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

//------------------------------------------------------------
// Function:    init
// Summary:
// In:      None.
// Out:     None.
// Return:  1  -> success.
//          -  -> failed.
//------------------------------------------------------------
ErrorCode IoMapping::init(fst_hal::FstIoDevice* io_device_ptr)
{
	io_dev_ptr_ = io_device_ptr;

	if(!param_ptr_->loadParam()){
		FST_WARN("Failed to load io_mapping component parameters");
		ErrorMonitor::instance()->add(IO_MAPPING_LOAD_PARAM_FAILED);
	} else {
		FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
	}

	ErrorCode ret = sim_ptr_->init();
	if (ret != SUCCESS){
		ErrorMonitor::instance()->add(ret);
		return ret;
	}

	io_mapper_.clear();
	loadProgramsPath();

	ret = updateMappingFile();
	if (ret != SUCCESS){
		ErrorMonitor::instance()->add(ret);
		return ret;
	}

	io_dev_ptr_->init();
	return SUCCESS;
}

//------------------------------------------------------------
// Function:    updateMappingFile
// Summary: read again the io mapping json files.
// In:      None.
// Out:     None.
// Return:  int   -> 1 is success, -1 is failed.
//------------------------------------------------------------
ErrorCode IoMapping::updateMappingFile()
{
	char io_map_file_name[128];
	int ret = 1;

	// DI
	sprintf(io_map_file_name, "%s/di_mapping.json", getProgramsPath());
	//printf(" the io_mapping file is %s\n",io_map_file_name);
	ret = appendSingleIOMapper(io_map_file_name, "DI");
	if (ret == -1){
		FST_WARN("Failed to read mapping parameters:%s", io_map_file_name);
		return IO_MAPPING_LOAD_MAP_FILE_FAILED;
	}
	// DO
	sprintf(io_map_file_name, "%s/do_mapping.json", getProgramsPath());
	ret = appendSingleIOMapper(io_map_file_name, "DO");
	if (ret == -1){
		FST_WARN("Failed to read mapping parameters", io_map_file_name);
		return IO_MAPPING_LOAD_MAP_FILE_FAILED;
	}
	// RI
	sprintf(io_map_file_name, "%s/ri_mapping.json", getProgramsPath());
	ret = appendSingleIOMapper(io_map_file_name, "RI");
	if (ret == -1){
		FST_WARN("Failed to read mapping parameters", io_map_file_name);
		return IO_MAPPING_LOAD_MAP_FILE_FAILED;
	}
	// RO
	sprintf(io_map_file_name, "%s/ro_mapping.json", getProgramsPath());
	ret = appendSingleIOMapper(io_map_file_name, "RO");
	if (ret == -1){
		FST_WARN("Failed to read mapping parameters", io_map_file_name);
		return IO_MAPPING_LOAD_MAP_FILE_FAILED;
	}

	FST_INFO("Update io_mapping files");
	return SUCCESS;
}

ErrorCode IoMapping::updateSimFile()
{
	FST_INFO("Update io_status files");
	return sim_ptr_->updateSimFile();
}

//------------------------------------------------------------
// Function:    getDIByBit
// Summary: get the value to the user port.
// In:      user_port  -> the port defined by the user.
// Out      value      -> 1 = ON, 0 = OFF.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IoMapping::getDIByBit(uint32_t user_port, uint8_t &value)
{
	// make a string "DI[user_port]"
	char cTemp[16];
    memset(cTemp, 0x00, sizeof(cTemp));
	sprintf(cTemp, "%s[%d]", "DI", user_port);

	// check if the port is simUsed
	bool data = false;
	if (sim_ptr_->isSimUsed(cTemp, data)){
		value = data;
		return SUCCESS;
	}

	// get physics ID by a map.
	string strKey;
	strKey.assign(cTemp);
	uint32_t map_id = 0;
	ErrorCode ret = SUCCESS;
	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end()){
		map_id = iter->second;
		ret = io_dev_ptr_->getDIOByBit(map_id, value);
		return ret;
	}
	FST_WARN("invalid strKey=%s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

//------------------------------------------------------------
// Function:    setDIByBit
// Summary: Set the value to the the port.
// In:      user_port  -> the port defined by the user.
//          value      -> 1 = ON, 0 = OFF.
// Out:     None.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IoMapping::setDIByBit(uint32_t user_port, uint8_t value)
{
	// make a string "DO[user_port]"
	char cTemp[16];
    memset(cTemp, 0x00, sizeof(cTemp));
	sprintf(cTemp, "%s[%d]", "DI", user_port);

	// set value if this port is in simulation status
	if (sim_ptr_->setSimValue(cTemp, value)){
		return SUCCESS;
	}

	fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
	return IO_INVALID_PARAM_ID;
}

//------------------------------------------------------------
// Function:    getDOByBit
// Summary: get the value to the user port.
// In:      user_port  -> the port defined by the user.
// Out:     value      -> 1 = ON, 0 = OFF.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IoMapping::getDOByBit(uint32_t user_port, uint8_t &value)
{
	// make a string "DO[user_port]"
    char cTemp[16];
    memset(cTemp, 0x00, sizeof(cTemp));
	sprintf(cTemp, "%s[%d]", "DO", user_port);

	// get physics ID by a map.
	string strKey;
	strKey.assign(cTemp);
	uint32_t map_id = 0;
	ErrorCode ret = SUCCESS;
	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end()){
		map_id = iter->second;
		ret = io_dev_ptr_->getDIOByBit(map_id, value);
		return ret;
	}
	FST_WARN("invalid strKey=%s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

//------------------------------------------------------------
// Function:    setDOByBit
// Summary: Set the output to the the port.
// In:      user_port  -> the port defined by the user.
//          value      -> 1 = ON, 0 = OFF.
// Out:     None.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IoMapping::setDOByBit(uint32_t user_port, uint8_t value)
{
	// make a string "DO[user_port]"
    char cTemp[16];
    memset(cTemp, 0x00, sizeof(cTemp));
	sprintf(cTemp, "%s[%d]", "DO", user_port);

	// get physics ID by a map.
	string  strKey;
	strKey.assign(cTemp);
	uint32_t map_id = 0;
	ErrorCode ret = SUCCESS;
	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end()){
		map_id = iter->second;
		ret = io_dev_ptr_->setDIOByBit(map_id, value);
		return ret;
	}
	FST_WARN("invalid strKey=%s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}


//------------------------------------------------------------
// Function:    getRIByBit
// Summary: get the value to the user port.
// In:      user_port  -> the port defined by the user.
// Out      value      -> 1 = ON, 0 = OFF.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IoMapping::getRIByBit(uint32_t user_port, uint8_t &value)
{
	// make a string "RI[user_port]"
	char cTemp[16];
	memset(cTemp, 0x00, sizeof(cTemp));
	sprintf(cTemp, "%s[%d]", "RI", user_port);

	// check if the port is simUsed
	bool data = false;
	if (sim_ptr_->isSimUsed(cTemp, data)){
		value = data;
		return SUCCESS;
	}

	// get physics ID by a map.
	string  strKey;
	strKey.assign(cTemp);
	uint32_t map_id = 0;
	ErrorCode ret = SUCCESS;
	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end()){
		map_id = iter->second;
		ret = io_dev_ptr_->getDIOByBit(map_id, value);
		return ret;
	}
	FST_WARN("invalid strKey=%s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

//------------------------------------------------------------
// Function:    setRIByBit
// Summary: Set the value to the the port.
// In:      user_port  -> the port defined by the user.
//          value      -> 1 = ON, 0 = OFF.
// Out:     None.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IoMapping::setRIByBit(uint32_t user_port, uint8_t value)
{
	// make a string "RO[user_port]"
	char cTemp[16];
	memset(cTemp, 0x00, sizeof(cTemp));
	sprintf(cTemp, "%s[%d]", "RI", user_port);

	// set value if this port is in simulation status
	if (sim_ptr_->setSimValue(cTemp, value)){
		return SUCCESS;
	}

	fst_base::ErrorMonitor::instance()->add(IO_INVALID_PARAM_ID);
	return IO_INVALID_PARAM_ID;
}

//------------------------------------------------------------
// Function:    getROByBit
// Summary: get the value to the user port.
// In:      user_port  -> the port defined by the user.
// Out:     value      -> 1 = ON, 0 = OFF.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IoMapping::getROByBit(uint32_t user_port, uint8_t &value)
{
	// make a string "RO[user_port]"
	char cTemp[16];
	memset(cTemp, 0x00, sizeof(cTemp));
	sprintf(cTemp, "%s[%d]", "RO", user_port);

	// get physics ID by a map.
	string strKey;
	strKey.assign(cTemp);
	uint32_t map_id = 0;
	ErrorCode ret = SUCCESS;
	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end()){
		map_id = iter->second;
		ret = io_dev_ptr_->getDIOByBit(map_id, value);
		return ret;
	}
	FST_WARN("invalid strKey=%s\n", strKey.c_str());
	return IO_INVALID_PARAM_ID;
}

//------------------------------------------------------------
// Function:    setROByBit
// Summary: Set the output to the the port.
// In:      user_port  -> the port defined by the user.
//          value      -> 1 = ON, 0 = OFF.
// Out:     None.
// Return:  ErrorCode   -> error codes.
//------------------------------------------------------------
ErrorCode IoMapping::setROByBit(uint32_t user_port, uint8_t value)
{
	// make a string "RO[user_port]"
	char cTemp[16];
	memset(cTemp, 0x00, sizeof(cTemp));
	sprintf(cTemp, "%s[%d]", "RO", user_port);

	// get physics ID by a map.
	string strKey;
	strKey.assign(cTemp);
	uint32_t map_id = 0;
	ErrorCode ret = SUCCESS;
	map<string, uint32_t>::iterator iter = io_mapper_.find(strKey);
	if (iter != io_mapper_.end()){
		map_id = iter->second;
		ret = io_dev_ptr_->setDIOByBit(map_id, value);
		return ret;
	}
	FST_WARN("invalid strKey=%s\n", strKey.c_str());
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
	printf("io_mapping_load_programs_path: %s .\n", files_manager_data_path_.c_str()); //delete soon
	FST_INFO("io_mapping_load_programs_path: %s", files_manager_data_path_.c_str());
}

char * IoMapping::getProgramsPath()
{
	return (char *)files_manager_data_path_.c_str();
}


int IoMapping::generateIOInfo(IOMapJsonInfo &objInfo, const char * strIOType)
{
	char cTemp[16];
	char cUpperType[8];
	string strKey, strValue ;
	
    strcpy(cUpperType, strIOType);
 
    //delete: path to path mapping.
/*	for (int i = objInfo.from ; i <= objInfo.to ; i++)
	{
		memset(cTemp, 0x00, 128);
		sprintf(cTemp, "%s[%d]", strIOType, i);
		strKey.assign(cTemp);
		
        vector<string> result=split(objInfo.module, "/"); //use "/" to split
		sprintf(cTemp, "root/IO/%s/%s/%s/%d", 
			result[0].c_str(), result[2].c_str(), cUpperType, 
			objInfo.index + i - objInfo.from);
		strValue.assign(cTemp);
        //printf("strkey = %s\n",strKey.c_str());
        //printf("strValue = %s\n",strValue.c_str());

		io_mapper.insert(
			map<string, string>::value_type(strKey, strValue));
		// Add revert element
		io_mapper.insert(
			map<string, string>::value_type(strValue, strKey));
	}*/

    // id mapping with union
    PhysicsID id;
    id.info.dev_type = fst_hal::DEVICE_TYPE_FST_IO;//=2
    
    vector<string> result=split(objInfo.module, "/"); //use "/" to split
    id.info.address = atoi(result[2].c_str());  // '3' in "RS485/DIGITAL_IN_OUT_DEVICE/3"

    if (strcasecmp(cUpperType, "DI") == 0)
        id.info.port_type = IO_TYPE_DI;
    else if (strcasecmp(cUpperType, "DO") == 0)
        id.info.port_type = IO_TYPE_DO;
    else if (strcasecmp(cUpperType, "RI") == 0)
    	id.info.port_type = IO_TYPE_RI;
    else if (strcasecmp(cUpperType, "RO") == 0)
    	id.info.port_type = IO_TYPE_RO;

    id.info.port = objInfo.index;
    for (int i = objInfo.from ; i <= objInfo.to ; i++)
	{
		memset(cTemp, 0x00, sizeof(cTemp));
		sprintf(cTemp, "%s[%d]", strIOType, i);
		strKey.assign(cTemp);
        FST_INFO("iomapping:strkey = %s  , physics id=%x", strKey.c_str(), id.number);

		io_mapper_.insert(map<string, uint32_t>::value_type(strKey, id.number));
        id.info.port++;
	}

	return 1;
}

int IoMapping::parseIOObject(cJSON *jsonIObject, const char * strIOType)
{
	IOMapJsonInfo objInfo ;
	cJSON *child=jsonIObject->child;
	
	//printf("parseDIObject:cJSON_Array--- %s\n", jsonIObject->string);
	while (child) 
	{
        // the last catelog, such as from, index... 
		//printf("parseIOObject: cJSON_object %s\n", child->string);
		if(strcmp(child->string, "from") == 0)
		{
			objInfo.from = child->valueint ;
		}
		else if(strcmp(child->string, "index") == 0)
		{
			objInfo.index = child->valueint ;
		}
		else if(strcmp(child->string, "module") == 0)
		{
			strcpy(objInfo.module, child->valuestring) ;
		}
		else if(strcmp(child->string, "to") == 0)
		{
			objInfo.to = child->valueint ;
		}
		child=child->next;
	}
	
	generateIOInfo(objInfo, strIOType);
	return 1;
}

int IoMapping::parseIO(cJSON *jsonDI, const char * strIOType)
{
	cJSON *child=jsonDI->child;

	while (child) // && !fail)
	{
		//printf("parseIO:cJSON_object--- %s\n", child->string);
		switch ((child->type)&255)
		{
		case cJSON_Number:	
			//printf("parseIO:cJSON_Number %d\n", child->valueint); 
			break;
		case cJSON_String:	
			//printf("parseIO:cJSON_String %s\n", child->valuestring); 
			break;
		case cJSON_Object:
            // the second catelog, such as [{},{}].
			//printf("parseIO:cJSON_Object\n"); 
			parseIOObject(child, strIOType);
			break;
		}
		child=child->next;
	}
	return 1;
}

int IoMapping::parseIOMap(char * data, const char * strIOType)
{
	cJSON *json;
	json=cJSON_Parse(data);
	if(json == NULL)
		return -1;
	
	cJSON *child=json->child;
	while(child)
	{
		switch ((child->type)&255)
		{
		case cJSON_True:	
			//printf("parseIOMap:cJSON_True"); 
			break;
		case cJSON_Number:	
			//printf("parseIOMap:cJSON_Number %d\n", child->valueint); 
			break;
		case cJSON_String:	
			//printf("parseIOMap:cJSON_String %s\n", child->valuestring);
			break;
		case cJSON_Array:	
			{
                // First catelog, such as DI, DO, AI, AO.
				//printf("parseIOMap:cJSON_Array %s\n", child->string);
                parseIO(child, strIOType);
				break;
			}
		case cJSON_Object:	
			//printf("cJSON_Object\n"); 
			break;
		}
		child = child->next ;
	}
	cJSON_Delete(json);
	return 1;
}

int IoMapping::printIOMapper()
{
	map<string, uint32_t>::iterator it;

	it = io_mapper_.begin();
	
	printf("\t\tobjThreadCntrolBlock->io_mapper_ has %d elements \n", 
			io_mapper_.size());

	while(it != io_mapper_.end())
	{
		printf("\t\t%s :: 0x%x \n", 
				it->first.c_str(), it->second);
		it++;         
	}
	return 1;
}

int IoMapping::appendSingleIOMapper(char *filename, const char * strIOType)
{
	FILE *f;long len;char *data;

	f=fopen(filename,"rb"); 
	if(f)
	{
        // read file to data, then parse data.
	    fseek(f,0,SEEK_END); len=ftell(f); fseek(f,0,SEEK_SET);
	    data=(char*)malloc(len+1); fread(data,1,len,f); 
		fclose(f);
		parseIOMap(data, strIOType);
		free(data);
	} else {
		return -1;
	}
	return 1;
}

vector<string> IoMapping::split(string str,string pattern)
{
	string::size_type pos;
	vector<string> result;
	str+=pattern;
	int size=str.size();

	for(int i=0; i<size; i++)
	{
		pos=str.find(pattern,i);
		if((int)pos<size)
		{
			string s=str.substr(i,pos-i);
			result.push_back(s);
			i=pos+pattern.size()-1;
		}
	}
	return result;
}

