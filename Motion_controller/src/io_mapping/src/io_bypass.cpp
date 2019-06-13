/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_bypass.cpp
Author:     Feng.Wu
Create:     30-Jan-2019
Modify:     30-Jan-2019
Summary:    to handle the bypass values.
**********************************************/
#include "io_bypass.h"
#include "parameter_manager/parameter_manager_param_group.h"

using namespace fst_ctrl;


IoBypass::IoBypass(fst_log::Logger *logger, IoMappingParam *param):
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = logger;
    param_ptr_ = param;
}

IoBypass::~IoBypass()
{
}

ErrorCode IoBypass::init()
{
    loadProgramsPath();
    ErrorCode ret = updateBypassFile();
    if (ret != SUCCESS)
        return ret;

    return SUCCESS;
}

//------------------------------------------------------------
// Function:    updateBypassFile
// Summary: read again the io status json files.
// In:      None.
// Out:     None.
// Return:  ErrorCode
//------------------------------------------------------------
ErrorCode IoBypass::updateBypassFile()
{
    char io_status_file_name[128];
    int ret = 1;
    io_bypass_.clear();

    // UI
    sprintf(io_status_file_name, "%s/ui_status.json", getProgramsPath());
    //printf(" the io_status file is %s\n",io_status_file_name);
    ret = appendSingleIOMapper(io_status_file_name, "UI");
    if (ret == -1){
        FST_INFO("Failed to read bypass-status parameters:%s", io_status_file_name);
        //return IO_MAPPING_LOAD_Bypass_FILE_FAILED;
    }
    
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    isBypassUsed
// Summary: check if the user port is in the Bypass status.
// In:      userString  ->the user defined IO such as UI[1]
// Out:     value       ->high or low.
// Return:  true  -> the port is in Bypass status.
//          false -> not.
//------------------------------------------------------------
bool IoBypass::isBypassUsed(const char * userString, bool & value)
{
    string key;
    key.assign(userString);
    map<string, bool>::iterator iter = io_bypass_.find(key);
    if (iter != io_bypass_.end()){
        value = iter->second;
        //printf("key = %s, isBypassUsed value = %d\n", key.c_str(), value);
        return true;
    }
    return false;
}


//------------------------------------------------------------
// Function:    setBypassValue
// Summary: set the user port value.
// In:      userString  ->the user defined IO such as UI[1]
//          value       ->high or low.
// Out:     None.
// Return:  true  -> set success.
//          false -> the port is not in Bypass status.
//------------------------------------------------------------
bool IoBypass::setBypass(const char * userString, bool yes_or_no)
{
    string key;
    key.assign(userString);
    if(yes_or_no == true)
    {
        map<string, bool>::iterator iter = value_map_.find(key);
        if (iter != value_map_.end())
        {
            io_bypass_[key] = iter->second;
            FST_INFO("setBypass = %s, map_size is = %d, vaule= %d", key.c_str(), io_bypass_.size(), io_bypass_[key]);
            return true;
        }
        return false;
    } 
    else if(yes_or_no == false)
    {
        map<string, bool>::iterator iter = io_bypass_.find(key);
        if (iter != io_bypass_.end())
        {
            io_bypass_.erase(key);
            FST_INFO("withDrawBypass = %s, map_size is = %d", key.c_str(), io_bypass_.size());
            return true;
        }
    }

    return true;   
}


void IoBypass::loadProgramsPath()
{
    files_manager_data_path_ = "";

    if(getenv("ROBOT_DATA_PREFIX") != NULL)
        files_manager_data_path_ = string(getenv("ROBOT_DATA_PREFIX")); //ROBOT_DATA_PREFIX=/root
    else
        files_manager_data_path_ = "/root"; // for self test only.

    files_manager_data_path_ += "/robot_data/io/io_status";
    FST_INFO("io_status_load_programs_path: %s", files_manager_data_path_.c_str());
}

char * IoBypass::getProgramsPath()
{
    return (char *)files_manager_data_path_.c_str();
}


int IoBypass::generateIOInfo(IOStatusJsonBypass &objInfo, const char * strIOType)
{
    char cTemp[16];
    bool bypassValue = false;
    string strKey;

    if (strcasecmp(objInfo.bypassValue, "ON") == 0)
            bypassValue = true;

    memset(cTemp, 0x00, sizeof(cTemp));
    sprintf(cTemp, "%s[%d]", strIOType, objInfo.userPort);
    strKey.assign(cTemp); 

    //record all the bypass value.
    value_map_.insert(map<string, bool>::value_type(strKey, bypassValue));

    //get the setting bypass.
    if (strcasecmp(objInfo.useBypass, "YES") == 0 || strcasecmp(objInfo.useBypass, "NO") == 0)
    {      
        io_bypass_.insert(map<string, bool>::value_type(strKey, bypassValue));
        FST_INFO("bypass: strkey = %s    , bypassValue = %d",strKey.c_str(), bypassValue);
    }
    
    return 1;
}

int IoBypass::parseIOObject(cJSON *jsonIObject, const char * strIOType)
{
    IOStatusJsonBypass objInfo ;
    cJSON *child=jsonIObject->child;

    //printf("parseDIObject:cJSON_Array--- %s\n", jsonIObject->string);
    while (child)
    {
        // the last catelog, such as from, index...
        //printf("parseIOObject: cJSON_object %s\n", child->string);
        if(strcmp(child->string, "useBypass") == 0)
        {
            strcpy(objInfo.useBypass, child->valuestring);
        }
        else if(strcmp(child->string, "bypassValue") == 0)
        {
            strcpy(objInfo.bypassValue, child->valuestring);
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

int IoBypass::parseIO(cJSON *jsonDI, const char * strIOType)
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

int IoBypass::parseIOMap(char * data, const char * strIOType)
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

int IoBypass::printIOMapper()
{
    map<string, bool>::iterator it;

    it = io_bypass_.begin();

    printf("ioBypass->io_bypass_ has %d elements \n", io_bypass_.size());

    while(it != io_bypass_.end())
    {
        printf("\t\t%s :: 0x%x \n", it->first.c_str(), it->second);
        it++;
    }
    return 1;
}

int IoBypass::appendSingleIOMapper(char *filename, const char * strIOType)
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

vector<string> IoBypass::split(string str,string pattern)
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