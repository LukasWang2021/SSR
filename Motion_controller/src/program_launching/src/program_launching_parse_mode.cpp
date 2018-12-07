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


bool ProgramLaunching::parseLaunchMode(char * data, int &mode)
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
		case cJSON_Number:	
			//printf("parseLaunchMode:cJSON_Number %d\n", child->valueint); 
			mode = child->valueint;
			break;
		default:	
			break;
		}
		child = child->next ;
	}
	cJSON_Delete(json);
	return true;
}

bool ProgramLaunching::openFileLaunchMode(char *filename, int &mode)
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
		parseLaunchMode(data, mode);
		free(data);
	} else 
	{
		return false;
	}
	return true;
}


