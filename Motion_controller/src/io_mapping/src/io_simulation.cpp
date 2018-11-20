/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_simulation.cpp
Author:     Feng.Wu
Create:     29-Oct-2018
Modify:     29-Oct-2018
Summary:    to store the simulation values.
**********************************************/
#include "io_simulation.h"
#include "parameter_manager/parameter_manager_param_group.h"

using namespace fst_ctrl;


IoSimulation::IoSimulation(fst_log::Logger *logger, IoMappingParam *param):
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = logger;
    param_ptr_ = param;
}

IoSimulation::~IoSimulation()
{
}

ErrorCode IoSimulation::init()
{
    io_sim_.clear();
    loadProgramsPath();
    ErrorCode ret = updateSimFile();
    if (ret != SUCCESS)
        return ret;

    return SUCCESS;
}

//------------------------------------------------------------
// Function:    updateSimFile
// Summary: read again the io status json files.
// In:      None.
// Out:     None.
// Return:  ErrorCode
//------------------------------------------------------------
ErrorCode IoSimulation::updateSimFile()
{
    char io_status_file_name[128];
    int ret = 1;
    // DI
    sprintf(io_status_file_name, "%s/di_status.json", getProgramsPath());
    //printf(" the io_status file is %s\n",io_status_file_name);
    ret = appendSingleIOMapper(io_status_file_name, "DI");
    if (ret == -1){
        FST_WARN("Failed to read mapping sim-status parameters:%s", io_status_file_name);
        return IO_MAPPING_LOAD_SIM_FILE_FAILED;
    }
    // RI
    sprintf(io_status_file_name, "%s/ri_status.json", getProgramsPath());
    ret = appendSingleIOMapper(io_status_file_name, "RI");
    if (ret == -1){
        FST_WARN("Failed to read mapping sim-status parameters:%s", io_status_file_name);
        return IO_MAPPING_LOAD_SIM_FILE_FAILED;
    }

    return SUCCESS;
}

//------------------------------------------------------------
// Function:    isSimUsed
// Summary: check if the user port is in the sim status.
// In:      userString  ->the user defined IO such as DI[1], DO[2]
// Out:     value       ->high or low.
// Return:  true  -> the port is in sim status.
//          false -> not.
//------------------------------------------------------------
bool IoSimulation::isSimUsed(const char * userString, bool & value)
{
    string key;
    key.assign(userString);
    map<string, bool>::iterator iter = io_sim_.find(key);
    if (iter != io_sim_.end()){
        value = iter->second;
        //printf("key = %s, isSimUsed value = %d\n", key.c_str(), value);
        return true;
    }
    return false;
}

//------------------------------------------------------------
// Function:    setSimValue
// Summary: set the user port value.
// In:      userString  ->the user defined IO such as DI[1], RI[1]
//          value       ->high or low.
// Out:     None.
// Return:  true  -> set success.
//          false -> the port is not in sim status.
//------------------------------------------------------------
bool IoSimulation::setSimValue(const char * userString, bool value)
{
    string key;
    key.assign(userString);
    map<string, bool>::iterator iter = io_sim_.find(key);
    if (iter != io_sim_.end()){
        iter->second = value;
        //printf("key = %s, set SimValue value= %d\n", key.c_str(), value);
        return true;
    }
    return false;
}


void IoSimulation::loadProgramsPath()
{
    files_manager_data_path_ = "";

    if(getenv("ROBOT_DATA_PREFIX") != NULL)
        files_manager_data_path_ = string(getenv("ROBOT_DATA_PREFIX")); //ROBOT_DATA_PREFIX=/root
    else
        files_manager_data_path_ = "/home/fst/fortest"; // for self test only.

    files_manager_data_path_ += "/robot_data/io/io_status";
    printf("io_status_load_programs_path: %s .\n", files_manager_data_path_.c_str()); //delete soon
    FST_INFO("io_status_load_programs_path: %s", files_manager_data_path_.c_str());
}

char * IoSimulation::getProgramsPath()
{
    return (char *)files_manager_data_path_.c_str();
}


int IoSimulation::generateIOInfo(IOStatusJsonSim &objInfo, const char * strIOType)
{
    char cTemp[16];
    bool simValue = false;
    string strKey;

    //map between userPort->simValue if simUsed is "YES"
    if (strcasecmp(objInfo.simUsed, "YES") == 0){
        if (strcasecmp(objInfo.simValue, "ON") == 0)
            simValue = true;

        memset(cTemp, 0x00, sizeof(cTemp));
        sprintf(cTemp, "%s[%d]", strIOType, objInfo.userPort);
        strKey.assign(cTemp);
        FST_INFO("sim: strkey = %s    , simValue = %d",strKey.c_str(), simValue);

        io_sim_.insert(map<string, bool>::value_type(strKey, simValue));
    }
    return 1;
}

int IoSimulation::parseIOObject(cJSON *jsonIObject, const char * strIOType)
{
    IOStatusJsonSim objInfo ;
    cJSON *child=jsonIObject->child;

    //printf("parseDIObject:cJSON_Array--- %s\n", jsonIObject->string);
    while (child)
    {
        // the last catelog, such as from, index...
        //printf("parseIOObject: cJSON_object %s\n", child->string);
        if(strcmp(child->string, "simUsed") == 0)
        {
            strcpy(objInfo.simUsed, child->valuestring);
        }
        else if(strcmp(child->string, "simValue") == 0)
        {
            strcpy(objInfo.simValue, child->valuestring);
        }
        else if(strcmp(child->string, "userPort") == 0)
        {
            objInfo.userPort = child->valueint ;
        }
        child=child->next;
    }

    generateIOInfo(objInfo, strIOType);
    return 1;
}

int IoSimulation::parseIO(cJSON *jsonDI, const char * strIOType)
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

int IoSimulation::parseIOMap(char * data, const char * strIOType)
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

int IoSimulation::printIOMapper()
{
    map<string, bool>::iterator it;

    it = io_sim_.begin();

    printf("\t\tobjThreadCntrolBlock->io_sim_ has %d elements \n",
           io_sim_.size());

    while(it != io_sim_.end())
    {
        printf("\t\t%s :: 0x%x \n",
               it->first.c_str(), it->second);
        it++;
    }
    return 1;
}

int IoSimulation::appendSingleIOMapper(char *filename, const char * strIOType)
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

vector<string> IoSimulation::split(string str,string pattern)
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