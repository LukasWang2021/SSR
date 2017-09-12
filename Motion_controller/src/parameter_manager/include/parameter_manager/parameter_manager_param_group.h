/*************************************************************************
	> File Name: parameter_manager_param_group.h
	> Author: 
	> Mail: 
	> Created Time: 2016年11月07日 星期一 17时15分29秒
 ************************************************************************/

#ifndef _PARAMETER_MANAGER_PARAM_GROUP_H
#define _PARAMETER_MANAGER_PARAM_GROUP_H

#include <parameter_manager/parameter_manager_param_value.h>
#include <string>
#include <vector>


namespace fst_parameter {

typedef unsigned long long int ErrorCode;
enum ScalarType {
    e_type_invalid,
    e_type_bool,
    e_type_int,
    e_type_double,
    e_type_string
};

class ParamGroup {
  public:
    ParamGroup(const std::string &file = "");
    ~ParamGroup();

    bool dumpParamFile(const std::string &file = "");
    bool loadParamFile(const std::string &file);

    bool deleteParam(const std::string &key = "");

    void clearLastError(void);
    const ErrorCode& getLastError(void);

    void getParamList(std::vector<std::string> &list);

    bool getParam(const std::string &key, bool &value);
    bool getParam(const std::string &key, int &value);
    bool getParam(const std::string &key, double &value);
    bool getParam(const std::string &key, std::string &value);
    bool getParam(const std::string &key, std::vector<bool> &value);
    bool getParam(const std::string &key, std::vector<int> &value);
    bool getParam(const std::string &key, std::vector<double> &value);
    bool getParam(const std::string &key, std::vector<std::string> &value);
    bool getParam(const std::string &key, ParamValue &value);

    bool hasParam(const std::string &key);
    
    bool setParam(const std::string &key, bool value);
    bool setParam(const std::string &key, int value);
    bool setParam(const std::string &key, double value);
    bool setParam(const std::string &key, const std::string &value);
    bool setParam(const std::string &key, const char *value);
    bool setParam(const std::string &key, const std::vector<bool> &value);
    bool setParam(const std::string &key, const std::vector<int> &value);
    bool setParam(const std::string &key, const std::vector<double> &value);
    bool setParam(const std::string &key, const std::vector<std::string> &value);
    bool setParam(const std::string &key, const ParamValue &value);

    void reset(void);
    // void test(void);

  private:
    void resolve(std::string &str);
    void split(const std::string &raw, std::vector<std::string> &cooked);

    //bool getNodeFromParamValue(ParamValue &value, YAML::Node &node);
    //bool getParamValueFromNode(YAML::Node &node, ParamValue &value);
    //bool getParamValueFromString(const std::string &scalar, ParamValue &value);
    void getListFromParamValue(ParamValue &value, const std::string &ns, std::vector<std::string> &list);

    void info(const char *format, ...);
    void warn(const char *format, ...);
    void error(const char *format, ...);

    std::string file_name_;
    ParamValue  value_;
    ErrorCode   last_error_;
};

}

#endif
