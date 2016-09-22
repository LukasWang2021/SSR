/**
 * @file json_parse.h
 * @brief 
 * @author Wang Wei
 * @version 1.0.0
 * @date 2016-08-22
 */
#include <map>
#include <string>
#include <boost/property_tree/ptree.hpp>  
#include <boost/property_tree/json_parser.hpp> 
#include "base_types.pb.h"

using namespace std;

#define FILE_API_PATH	("/opt/controller/config/API.txt")

class Json_Parse
{
public:
	string file_API_path;
	map<string, BaseTypes_ParamInfo> param_info_list;

	Json_Parse(string file_API_path):file_API_path(file_API_path)
	{
	
	}

	bool Parse_API_Json();
};
