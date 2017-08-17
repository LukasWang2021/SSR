/*************************************************************************
	> File Name: parameter_manager_param_group.h
	> Author: 
	> Mail: 
	> Created Time: 2016年11月07日 星期一 17时15分29秒
 ************************************************************************/

#ifndef _PARAMETER_MANAGER_PARAM_GROUP_H
#define _PARAMETER_MANAGER_PARAM_GROUP_H

#include <string>
#include <vector>
#include <map>

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <parameter_manager/parameter_manager_param_value.h>

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
    ParamGroup(const std::string &file = "", const std::string &ns = "");
    ~ParamGroup();

    bool dumpParamFile(const std::string &file);
    bool loadParamFile(const std::string &file);

    bool deleteParam(const std::string &key = "");

    void clearLastError(void);
    const ErrorCode& getLastError(void);

    const std::string getNamespace(void);
    void setNamespace(const std::string &ns);
    
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
    /*
    bool getRemoteParam(const std::string &key, XmlRpc::XmlRpcValue &value);
    bool getRemoteParam(const std::string &key, int &value);
    bool getRemoteParam(const std::string &key, double &value);
    bool getRemoteParam(const std::string &key, bool &value);
    bool getRemoteParam(const std::string &key, std::string &value);
    bool getRemoteParam(const std::string &key, std::vector<int> &value);
    bool getRemoteParam(const std::string &key, std::vector<double> &value);
    bool getRemoteParam(const std::string &key, std::map<std::string, XmlRpc::XmlRpcValue> &value);

    static bool getRemoteParamImpl(const std::string &key, XmlRpc::XmlRpcValue &value);
    static bool getRemoteParamImpl(const std::string &key, int &value);
    static bool getRemoteParamImpl(const std::string &key, double &value);
    static bool getRemoteParamImpl(const std::string &key, bool &value);
    static bool getRemoteParamImpl(const std::string &key, std::string &value);
    static bool getRemoteParamImpl(const std::string &key, std::vector<int> &value);
    static bool getRemoteParamImpl(const std::string &key, std::vector<double> &value);
    static bool getRemoteParamImpl(const std::string &key, std::map<std::string, XmlRpc::XmlRpcValue> &value);

    bool uploadParam();
    bool uploadParam(const YAML::Node &node, const std::string &path);
    */
    void test(void);

  private:
    void resolve(std::string &str);
    void split(const std::string &raw, std::vector<std::string> &cooked);

    bool getNodeFromParamValue(ParamValue &value, YAML::Node &node);
    bool getParamValueFromNode(YAML::Node &node, ParamValue &value);
    bool getParamValueFromString(const std::string &scalar, ParamValue &value);

    void info(const char *format, ...);
    void warn(const char *format, ...);
    void error(const char *format, ...);
    
    ParamValue  value_;
    std::string namespace_;
    ErrorCode   last_error_;
};

}

#endif
