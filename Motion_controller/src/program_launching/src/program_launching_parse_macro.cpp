/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       program_launching_parse_macro.cpp
Author:     Feng.Wu 
Create:     03-Dec-2018
Modify:     03-Dec-2018
Summary:    
**********************************************/
#include "program_launching.h"
#include "error_monitor.h"

using namespace fst_ctrl;
using namespace fst_base;

void ProgramLaunching::loadProgramsPath()
{
	files_manager_data_path_ = "";

	if(getenv("ROBOT_DATA_PREFIX") != NULL)
		files_manager_data_path_ = string(getenv("ROBOT_DATA_PREFIX")); //ROBOT_DATA_PREFIX=/root
	else
		files_manager_data_path_ = "/home/fst/fortest"; // for self test only.

	files_manager_data_path_ += "/robot_data/launch_setting";
}

char * ProgramLaunching::getProgramsPath()
{
	return (char *)files_manager_data_path_.c_str();
}

void ProgramLaunching::generateMacroConfigInfo(MacroConfigInfo &objInfo)
{
	macro_vector_.push_back(objInfo);
}

bool ProgramLaunching::parseMacroConfigObject(cJSON *object)
{
	MacroConfigInfo objInfo ;
	cJSON *child=object->child;
	
	//printf("parseMacroConfigObject:cJSON_Array--- %s\n", jsonIObject->string);
	while (child) 
	{
		// printf("parseMacroConfigObject: cJSON_object %s\n", child->string);
		if(strcmp(child->string, "userDefined") == 0)
		{
			strcpy(objInfo.userDefined, child->valuestring) ;
		}
		else if(strcmp(child->string, "macroProgram") == 0)
		{
			strcpy(objInfo.macroProgram, child->valuestring) ;
		}
		else if(strcmp(child->string, "ioType") == 0)
		{
			strcpy(objInfo.ioType, child->valuestring) ;
		}
		else if(strcmp(child->string, "ioPort") == 0)
		{
			objInfo.ioPort = child->valueint;
		}
		child=child->next;
	}
	
	generateMacroConfigInfo(objInfo);
	return true;
}

bool ProgramLaunching::parseMacroConfig(char * data)
{
	cJSON *json;
	json=cJSON_Parse(data);
	if(json == NULL)
		return false;
	
	cJSON *child=json->child;
	while(child)
	{
		switch ((child->type)&255)
		{
		case cJSON_True:	
			//printf("parseMacroConfig:cJSON_True"); 
			break;
		case cJSON_Number:	
			//printf("parseMacroConfig:cJSON_Number %d\n", child->valueint); 
			break;
		case cJSON_String:	
			//printf("parseMacroConfig:cJSON_String %s\n", child->valuestring);
			break;
		case cJSON_Array:	
			//printf("parseMacroConfig:cJSON_Array %s\n", child->string);                
			break;
		case cJSON_Object:	
			//printf("parseMacroConfig:cJSON_Object\n"); 
			parseMacroConfigObject(child);
			break;
		}
		child = child->next ;
	}
	cJSON_Delete(json);
	return true;
}

bool ProgramLaunching::openFileMacroConfig(char *filename)
{
	FILE *f;
	long len = 0;
	char *data = NULL;

	f=fopen(filename,"rb"); 
	if(f)
	{
        // read file to data, then parse data.
	    fseek(f,0,SEEK_END); len=ftell(f); fseek(f,0,SEEK_SET);
	    data=(char*)malloc(len+1); fread(data,1,len,f); 
		fclose(f);
		parseMacroConfig(data);
		free(data);
	} else 
	{
		return false;
	}
	return true;
}

void ProgramLaunching::printMacroConfigInfo()
{
	int num = macro_vector_.size();
	for (int i = 0; i < num; ++i)
	{
		FST_INFO("macro index = %d, userDefined = %s, macroProgram = %s, ioType = %s, ioPort = %d", i,
		    macro_vector_[i].userDefined, macro_vector_[i].macroProgram, macro_vector_[i].ioType, macro_vector_[i].ioPort);
	}
}


