/**
 * @file json_parse.cpp
 * @brief 
 * @author Wang Wei
 * @version 1.0.0
 * @date 2016-08-22
 */
#include <string>
#include <iostream>
#include "json_parse.h"

using namespace std;

bool Json_Parse::Parse_API_Json()
{
	int i;
	boost::property_tree::ptree root;  
	BaseTypes_ParamInfo	param_info;

	boost::property_tree::read_json<boost::property_tree::ptree>(file_API_path,root); 
	for(boost::property_tree::ptree::iterator it=root.begin(); it != root.end(); it++)
	{
		//cout<<it->second.get<string>("path")<<endl;
		strcpy((char*)param_info.path, it->second.get<string>("path").c_str());
	    param_info.id = it->second.get<int>("id");
		param_info.overwrite_active = it->second.get<int>("overwrite_active");
		param_info.data_type = it->second.get<int>("data_type");
		param_info.data_size = it->second.get<int>("data_size");
		param_info.number_of_elements = it->second.get<int>("number_of_elements");
	    param_info.param_type = (BaseTypes_ParamType)it->second.get<int>("param_type");
		param_info.permission = (BaseTypes_Permission)it->second.get<int>("permission");
		param_info.user_level = (BaseTypes_UserLevel)it->second.get<int>("user_level");
		param_info.unit = (BaseTypes_Unit)it->second.get<int>("unit");
	
		param_info_list.insert(map<string, BaseTypes_ParamInfo>::value_type(param_info.path, param_info));
	}
}
