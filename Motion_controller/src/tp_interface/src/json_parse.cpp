/**
 * @file json_parse_.cpp
 * @brief 
 * @author Wang Wei
 * @version 1.0.0
 * @date 2016-08-22
 */
#include <string>
#include <iostream>
#include <pb_encode.h>
#include "json_parse.h"
#include "common.h"

 
/**
 * @brief: callback for encode param list msg  
 *
 * @param stream: input
 * @param field: input
 * @param arg: the class of JsonParse
 *
 * @return: true if success 
 */
bool listCallback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    JsonParse *json_paser = (JsonParse*) *arg;
    BaseTypes_ParameterMsg param_msg = {};
    
     map<string, uint32_t>::iterator it = json_paser->path_id_map_.begin();
	for (; it != json_paser->path_id_map_.end(); ++it)    
    {
        param_msg.has_info = true;
        strcpy(param_msg.info.path, it->first.c_str());
        int id = it->second;
        ParamProperty property = json_paser->params_list_map_[id];
		param_msg.info.id = id;
		param_msg.info.overwrite_active = property.overwrite_active;
		param_msg.info.data_type = property.data_type;
		param_msg.info.data_size = property.data_size;
		param_msg.info.number_of_elements = property.number_of_elements;
		param_msg.info.param_type = property.param_type;
		param_msg.info.permission = property.permission;
		param_msg.info.user_level = property.user_level;
		param_msg.info.unit = property.unit;
        
        /* This encodes the header for the field, based on the constant info
         * from pb_field_t. */
        if (!pb_encode_tag_for_field(stream, field))
            return false;
        
        /* This encodes the data for the field, based on our FileInfo structure. */
        if (!pb_encode_submessage(stream, BaseTypes_ParameterMsg_fields, &param_msg))
            return false;
    }
    param_msg.has_info = true;
    param_msg.info.overwrite_active = 0; //not active
    param_msg.info.data_type = 2;  // uint8
    param_msg.info.data_size = 1;  // 1 byte
    param_msg.info.number_of_elements = 1;  
    param_msg.info.param_type = BaseTypes_ParamType_INPUT_SIGNAL;  //input
    param_msg.info.permission = BaseTypes_Permission_permission_undefined;
    param_msg.info.user_level = BaseTypes_UserLevel_user_level_undefined;
    param_msg.info.unit = BaseTypes_Unit_unit_undefined;
    IOInterface *io_interface = json_paser->io_interface_;
    fst_io_manager::IODeviceInfo *info = io_interface->getDevInfoPtr();
    FST_INFO("the io num :%d", io_interface->getIODevNum());
    for (int i = 0; i < io_interface->getIODevNum(); i++, info++)
    {        
        strcpy(param_msg.info.path, info->path.c_str());
        param_msg.info.id = info->id;

        if (!pb_encode_tag_for_field(stream, field))
            return false;        
        if (!pb_encode_submessage(stream, BaseTypes_ParameterMsg_fields, &param_msg))
            return false;
        
        /*for (unsigned int j = 0; j < info->input; j++)*/
        //{
            //int di_id = info->id + j + 1;
            //sprintf(param_msg.info.path, "%s/DI/%d", info->path.c_str(), di_id);
            //param_msg.info.id = di_id;
            //if (!pb_encode_tag_for_field(stream, field))
                //return false;        
            //if (!pb_encode_submessage(stream, BaseTypes_ParameterMsg_fields, &param_msg))
                //return false;
        //}
        //int offset = info->input + 1;
        //param_msg.info.param_type = BaseTypes_ParamType_OUTPUT_SIGNAL;  
        //for (unsigned int j = 0; j < info->output; j++)
        //{
            //int do_id = info->id + j + offset;
            //sprintf(param_msg.info.path, "%s/DO/%d", info->path.c_str(), do_id);
            //param_msg.info.id = do_id;
            //if (!pb_encode_tag_for_field(stream, field))
                //return false;        
            //if (!pb_encode_submessage(stream, BaseTypes_ParameterMsg_fields, &param_msg))
                //return false;
        /*}*/
    }

    return true;
}

JsonParse::JsonParse(string path)
{
	char buf[1024] = {0};
	try
	{
		readlink("/proc/self/exe" , buf , sizeof(buf));
		boost::filesystem::path pa(buf);
		std::string absolute_path(pa.parent_path().parent_path().parent_path().string()+path);
		file_API_path_ = absolute_path;

		parseAPIJson();		
	}
	catch(boost::filesystem::filesystem_error e)
	{
		FST_ERROR("Error parse json construction:%s", e.what());
		exit(0);
	}
    
    
}

JsonParse::~JsonParse()
{

}
/**
 * @brief: parse the file to a map list
 *
 * @return: true if success
 */
void JsonParse::parseAPIJson()
{
	boost::property_tree::ptree root;  
	ParamProperty param_property;

	try
	{
		boost::property_tree::read_json<boost::property_tree::ptree>(file_API_path_,root); 
		for (boost::property_tree::ptree::iterator it=root.begin(); it != root.end(); it++)
		{
			//std::cout<<it->second.get<string>("path")<<std::endl;
		//	strcpy((char*)param_property.path, it->second.get<string>("path").c_str());
			uint32_t id = it->second.get<int>("id");
			//id++; //add self to identify this message parameter
			param_property.overwrite_active = (bool)it->second.get<int>("overwrite_active");
			param_property.data_type = it->second.get<int>("data_type");
			param_property.data_size = it->second.get<int>("data_size");
			param_property.number_of_elements = it->second.get<int>("number_of_elements");
			param_property.param_type = (BaseTypes_ParamType)it->second.get<int>("param_type");
			param_property.permission = (BaseTypes_Permission)it->second.get<int>("permission");
			param_property.user_level = (BaseTypes_UserLevel)it->second.get<int>("user_level");
			param_property.unit = (BaseTypes_Unit)it->second.get<int>("unit");
		
			param_property.update_freq = 0;
			params_list_map_.insert(map<uint32_t, ParamProperty>::value_type(id, param_property));
		//	params_list_.push_back(param_property);
			path_id_map_.insert(map<string, uint32_t>::value_type(it->second.get<string>("path").c_str(), id));
		}//end for (boost::property_tree::ptree::iterator it=root.begin(); it != root.end(); it++)

		param_length_ = path_id_map_.size();
	}//end try
	catch(boost::property_tree::ptree_error & e)
	{
		FST_ERROR("parse json file failed, error:%s", e.what());
		exit(0);
	}
}

/**
 * @brief: get parameter list from the map
 *
 * @return: the struct of BaseTypes_ParameterListMsg 
 */
BaseTypes_ParameterListMsg JsonParse::getParamListMsg()
{
	BaseTypes_ParameterListMsg param_list_msg;
	param_list_msg.has_header = false;
	param_list_msg.params.funcs.encode = &listCallback;
    param_list_msg.params.arg = this;

	return param_list_msg;
}

/**
 * @brief: get the size of params_list_map_ 
 *
 * @return:the size 
 */
int JsonParse::getParamsListSize()
{
	return param_length_;
}

/**
 * @brief:get parameter from an ID 
 *
 * @param id: input==> the param id
 * @param param_property: output==>the param_property
 *
 * @return: true if success 
 */
bool JsonParse::getParamFromID(uint32_t id, ParamProperty &param_property)
{	
	map<uint32_t, ParamProperty>::iterator it = params_list_map_.find(id);
	if (it == params_list_map_.end())
	{
		FST_INFO("getParamFromID:can't find ID:%d",id);
		return false;
	}

	param_property = it->second;		
	return true;
}
/**
 * @brief:get parameter from an path
 *
 * @param id: input==> the param path
 * @param param_property: output==>the param id
 * 
 * @return: true if success 
 */
bool JsonParse::getIDFromPath(const char *path, uint32_t &id)
{
	map<string, uint32_t>::iterator it = path_id_map_.find(path);
	
	if (it == path_id_map_.end())
	{
		FST_INFO("getIDFromPath:%s,can't find ID:%d",path, id);
		return false;
	}

	id = path_id_map_[path];
	return true;
}

ParamProperty* JsonParse::getParamProperty(uint32_t id)
{
    map<uint32_t, ParamProperty>::iterator it = params_list_map_.find(id);

    if (it == params_list_map_.end())
    {
        FST_ERROR("can't find id:%d", id);
        return NULL;
    }

    return &(it->second);
}

/**
 * @brief: getParamInfo from the struct of ParamProperty 
 *
 * @param path: input==>the parameter path
 * @param param_property: input==>the parameter property
 *
 * @return: the pointer of parameter infomation 
 */
BaseTypes_ParamInfo* JsonParse::getParamInfo(const char *path, uint32_t id, ParamProperty* param_property)
{
    static BaseTypes_ParamInfo param_info;

    if (param_property == NULL)
    {
        return NULL;
    }

    strcpy(param_info.path, path);
	param_info.id = id;
	param_info.overwrite_active = param_property->overwrite_active;
	param_info.data_type = param_property->data_type;
	param_info.data_size = param_property->data_size;
	param_info.number_of_elements = param_property->number_of_elements;
	param_info.param_type = param_property->param_type;
	param_info.permission = param_property->permission;
	param_info.user_level = param_property->user_level;
	param_info.unit = param_property->unit;

	return &param_info;
}

void JsonParse::addIODevices()
{
    fst_io_manager::IODeviceInfo *info = io_interface_->getDevInfoPtr();
    for (int i = 0; i < io_interface_->getIODevNum(); i++)
    {
        path_id_map_.insert(map<string, uint32_t>::value_type(info[i].path, info[i].id));
        FST_INFO("path:%s,id:%d, input:%d, output:%d", \
                info[i].path.c_str(), info[i].id, info[i].input, info[i].output);

    }
}


void JsonParse::getDeviceList(motion_spec_DeviceList &dev_list)
{
    int num = io_interface_->getIODevNum();

    boost::mutex::scoped_lock lock(mutex_);

    fst_io_manager::IODeviceInfo* info = io_interface_->getDevInfoPtr();
    dev_list.dev_info_count = num;

    for (int i = 0; i < num; i++)
    {
        //FST_INFO("++input:%d,output:%d", info[i].input, info[i].output);
        strcpy(dev_list.dev_info[i].communication_type, info[i].communication_type.c_str());
        dev_list.dev_info[i].device_number = info[i].device_number;
        dev_list.dev_info[i].device_type = (motion_spec_DeviceType)info[i].device_type;
        dev_list.dev_info[i].input = info[i].input;
        dev_list.dev_info[i].output = info[i].output;
    }
}

