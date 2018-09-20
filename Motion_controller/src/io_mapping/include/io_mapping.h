#ifndef IO_MAPPING_H
#define IO_MAPPING_H

#include<stdio.h>
#ifdef WIN32
#pragma warning(disable : 4786)
#endif
#include <memory.h>
#include <string.h>
#include<string>
#include<vector>
#include<map>
#include <algorithm>
using namespace std;

#include "io_mapping_cJSON.h"

#include "io_mapping_param.h"
#include "common_log.h"

namespace fst_ctrl
{
	
#ifndef WIN32
#define stricmp     strcasecmp
#endif
	
typedef struct _IOMapJsonInfo
{
	int 		from;
	int 		index;
	char		module[128];
	int 		to;
}IOMapJsonInfo;

typedef struct _IOMapVarInfo
{
	char		in[8];
	char		out[8];
}IOMapVarInfo;

class IoMapping
{
public:
    IoMapping();
    ~IoMapping();
	
	int init();
        // map<string, string> getIOMapper() { return io_mapper; }
        string getIOPathByName(string key) { return io_mapper[key]; }

private:
	int generateIOInfo(IOMapJsonInfo &objInfo, char * strIOType);
	int parseIOObject(cJSON *jsonIObject, char * strIOType);
	int parseIO(cJSON *jsonDI, char * strIOType);
	int parseIOMap(char * data, IOMapVarInfo &varInfo);
	int printIOMapper();
	int appendSingleIOMapper(char *filename, IOMapVarInfo &varInfo);
	
	vector<string> split(string str,string pattern);
	
	void loadProgramsPath();
	char * getProgramsPath();
private:
    IoMappingParam* param_ptr_;
    fst_log::Logger* log_ptr_;
 
	map<string, string> io_mapper;
	std::string files_manager_data_path = "";
};

}

#endif


