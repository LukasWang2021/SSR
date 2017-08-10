/**
 * @file json_parse_.h
 * @brief 
 * @author Wang Wei
 * @version 1.0.0
 * @date 2016-08-22
 */
#ifndef TP_INTERFACE_JASON_PARSE_H_
#define TP_INTERFACE_JASON_PARSE_H_

#include <map>
#include <string>
#include <boost/property_tree/ptree.hpp>  
#include <boost/property_tree/json_parser.hpp> 
#include "base_types.pb.h"
#include "motionSL.pb.h"
#include <boost/filesystem.hpp>
#include "io_manager/io_manager.h"
#include "common.h"
#include "io_interface.h"


using std::map;
using std::string;
using std::vector;

#define MAX_LIST_NUM	(1024)
#define FILE_API_PATH	("/share/tp_interface/config/API.json")

typedef struct _ParamProperty
{
	//uint32_t id;
    bool overwrite_active;
    uint32_t data_type;
    uint32_t data_size;
    uint32_t number_of_elements;
    BaseTypes_ParamType param_type;
    BaseTypes_Permission permission;
    BaseTypes_UserLevel user_level;
    BaseTypes_Unit unit;
	uint32_t update_freq;
    void (*getparam)();
    void (*setParam)(void* params);
}ParamProperty;

class JsonParse
{
  public:	
    IOInterface *io_interface_;
	map<string, uint32_t> path_id_map_;
	map<uint32_t, ParamProperty> params_list_map_; // store the parameter list

	JsonParse(string path);
	~JsonParse();

	/**
	 * @brief: parse the file to a map list
	 */
	void parseAPIJson();
	/**
	 * @brief: get parameter list from the map
	 *
	 * @return: the struct of BaseTypes_ParameterListMsg 
	 */
	BaseTypes_ParameterListMsg getParamListMsg();
	/**
	 * @brief: get the size of params_list_map_ 
	 *
	 * @return:the size 
	 */
	int getParamsListSize();
	/**
	 * @brief:get parameter from an ID 
	 *
	 * @param id: input==> the param id
	 * @param param_property: output==>the param_property
	 *
	 * @return: true if success 
	 */
	bool getParamFromID(uint32_t id, ParamProperty &param_property);
	/**
	 * @brief:get parameter from an path
	 *
	 * @param id: input==> the param path
	 * @param param_property: output==>the param id
	 * 
	 * @return: true if success 
	 */
	bool getIDFromPath(const char *path, uint32_t &id);
    /**
     * @brief 
     *
     * @param id: input ==> parameter id
     *
     * @return 
     */
    ParamProperty* getParamProperty(uint32_t id);
	/**
     * @brief: getParamInfo from the struct of ParamProperty 
     *
     * @param path: input==>the parameter path
     * @param param_property: input==>the parameter property
     *
     * @return: the pointer of parameter infomation 
     */
	BaseTypes_ParamInfo* getParamInfo(const char *path, uint32_t id, ParamProperty* param_property);

    /**
     * @brief: add io device infomation to dev_info_ 
     *
     * @return: 0 if success 
     */
    void addIODevices();

    /**
     * @brief 
     *
     * @param dev_list: output
     */
    void getDeviceList(motion_spec_DeviceList &dev_list);


  private:
	string file_API_path_; //path of API.txt file
	uint32_t param_length_;

    boost::mutex	mutex_;		//mutex lock
};

#endif
