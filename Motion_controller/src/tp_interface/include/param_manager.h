/**
 * @file param_manager.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-11-21
 */
#ifndef TP_INTERFACE_PARAM_MANAGER_H_
#define TP_INTERFACE_PARAM_MANAGER_H_

#include "parameter_manager/parameter_manager_param_group.h"
#include "base_types.pb.h"
#include <map>

using std::string;
using std::map;
using namespace fst_parameter;

typedef struct _ParamPropty
{
	uint32_t id;
    bool overwrite_active;
    uint32_t data_type;
    uint32_t data_size; 
    uint32_t number_of_elements;
    BaseTypes_ParamType param_type;
    BaseTypes_Permission permission;
    BaseTypes_UserLevel user_level;
    BaseTypes_Unit unit;
	uint32_t update_freq;
}ParamPropty;

class ParamManager
{
  public:
    map<uint32_t, ParamPropty> params_list_;

	ParamManager();
	~ParamManager();

	bool getParamID(const string &path, int &id);
    bool getParamInfo(const string &path, BaseTypes_ParamInfo &param_info);
	BaseTypes_ParameterListMsg getParamListMsg(); 
  private:    
    ParamGroup param_group_;
};

#endif
