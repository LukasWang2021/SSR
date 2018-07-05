/**
 * @file reg_interface.cpp
 * @brief 
 * @author Lujiaming
 * @version 1.0.0
 * @date 2018-04-12
 */
	 
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>

#include "launch_code_mgr.h"
#include "common.h"
#include "error_code.h"
#include "error_monitor.h"
#include <boost/algorithm/string.hpp>
#include "forsight_cJSON.h"

LaunchCodeMgr::LaunchCodeMgr()
{
    U64 result = initial();
    if (result != TPI_SUCCESS)
    {
        // rcs::Error::instance()->add(result);
    }
}

LaunchCodeMgr::~LaunchCodeMgr()
{
	launchCodeList.clear();
}


int LaunchCodeMgr::parseLaunchCode(char * data)
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
			printf("cJSON_True"); break;
		case cJSON_Number:		
			{
				if(strcmp(child->string, "launchCode") == 0)
				{
					cJSON_Delete(json);
					return child->valueint ;
				}
			}
			break;
		case cJSON_String:
			break;
		case cJSON_Array:
			break;
		case cJSON_Object:	
			// printf("cJSON_Object\n"); 
			break;
		}
		child = child->next ;
	}
	cJSON_Delete(json);
	return -1;
}


int LaunchCodeMgr::parseProgramPropFile(char *filename, char * progName)
{
	int iCode = 0;
	FILE *f;long len;char *data;

	f=fopen(filename,"rb"); 
	if(f)
	{
	    fseek(f,0,SEEK_END); len=ftell(f); fseek(f,0,SEEK_SET);
	    data=(char*)malloc(len+1); fread(data,1,len,f); 
		fclose(f);
		iCode = parseLaunchCode(data);
		launchCodeList.insert(std::pair<int, std::string>(iCode, progName));
		free(data);
	}
	return 1;
}

int LaunchCodeMgr::readFileList(char *basePath)
{
    DIR *dir;
    struct dirent *ptr;
    char base[1000];
    char filename[256];
	char * strExtPtr ;

    if ((dir=opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
        return;
    }

    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
        {
        	printf("d_name:%s/%s\n",basePath,ptr->d_name);
			strExtPtr = strrchr(ptr->d_name, '.');
			if(strExtPtr)
			{
				if(strcmp(strExtPtr, ".json") == 0)
				{
		            memset(base,'\0',sizeof(base));
		            strcpy(base,basePath);
		            strcat(base,"/");
		            strcat(base,ptr->d_name);
					
		            memset(filename,'\0',sizeof(filename));
					strcpy(filename, ptr->d_name);
					strExtPtr = strrchr(filename, '.');
					strExtPtr[0] = '\0';
					parseProgramPropFile(base, filename);
				}
			}
        }
        else if(ptr->d_type == 10)    ///link file
        {
            printf("d_name:%s/%s\n",basePath,ptr->d_name);
        }
        else if(ptr->d_type == 4)    ///dir
        {
            memset(base,'\0',sizeof(base));
            strcpy(base,basePath);
            strcat(base,"/");
            strcat(base,ptr->d_name);
            readFileList(base);
        }
    }
    closedir(dir);
    return 1;
}

U64 LaunchCodeMgr::initial()
{
	readFileList("\/data\/programs\/");
	// printLaunchCodeList();
	return TPI_SUCCESS;
}
	
U64 LaunchCodeMgr::updateAll()
{	
    return initial();
}

std::string LaunchCodeMgr::getProgramByCode(int iLaunchCode)
{
    return launchCodeList[iLaunchCode];
}

int LaunchCodeMgr::printLaunchCodeList()
{
	std::map<int, std::string>::iterator it;

	it = launchCodeList.begin();

	while(it != launchCodeList.end())
	{
		// it->first;  // it->second;
		printf("\t LaunchCodeMgr: %d :: %s \n", it->first, it->second.c_str());
		it++;         
	}
	return 1;
}


