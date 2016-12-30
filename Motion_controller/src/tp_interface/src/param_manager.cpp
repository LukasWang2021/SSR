#include "param_manager.h"
#include <boost/filesystem.hpp>
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
bool callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    ParamManager *param_manager = (ParamManager*) *arg;
    BaseTypes_ParameterMsg param_msg = {};
    
  //   map<uint32_t, ParamPropty>::iterator it = param_manager->params_list_.begin();
	/*for (; it != json_paser->path_id_map_.end(); ++it)    */
    //{
        //param_msg.has_info = true;
        //strcpy(param_msg.info.path, it->first.c_str());
        //int id = it->second;
        //ParamProperty property = json_paser->params_list_map_[id];
		//param_msg.info.id = id;
		//param_msg.info.overwrite_active = property.overwrite_active;
		//param_msg.info.data_type = property.data_type;
		//param_msg.info.data_size = property.data_size;
		//param_msg.info.number_of_elements = property.number_of_elements;
		//param_msg.info.param_type = property.param_type;
		//param_msg.info.permission = property.permission;
		//param_msg.info.user_level = property.user_level;
		//param_msg.info.unit = property.unit;
        
        /* This encodes the header for the field, based on the constant info
         * from pb_field_t. */
        //if (!pb_encode_tag_for_field(stream, field))
            //return false;
        
        //[> This encodes the data for the field, based on our FileInfo structure. <]
        //if (!pb_encode_submessage(stream, BaseTypes_ParameterMsg_fields, &param_msg))
            //return false;
    /*}*/
    
    return true;
}
 
ParamManager::ParamManager() 
{
    map<string, XmlRpc::XmlRpcValue> rpc_value_list;
    param_group_.getParam("/fst/tp_interface", rpc_value_list);

    map<string, XmlRpc::XmlRpcValue>::iterator it = rpc_value_list.begin();
    for(; it != rpc_value_list.end(); ++it)
    {
        boost::filesystem::path path(it->first);
        if(path.filename() == "id")
        {
            int id = it->second;
            FST_INFO("id:%d",id);
        }
    }
}

ParamManager::~ParamManager()
{

}

BaseTypes_ParameterListMsg ParamManager::getParamListMsg()
{
    BaseTypes_ParameterListMsg param_list_msg;
	param_list_msg.has_header = false;
	param_list_msg.params.funcs.encode = &callback;
    param_list_msg.params.arg = this;

	return param_list_msg;
}
bool ParamManager::getParamID(const string &path, int &id)
{
    string key = "/fst/tp_interface" + path + "/id";
    return param_group_.getParam(path, id);
}
bool ParamManager::getParamInfo(const string &path, BaseTypes_ParamInfo &param_info)
{
    
}


