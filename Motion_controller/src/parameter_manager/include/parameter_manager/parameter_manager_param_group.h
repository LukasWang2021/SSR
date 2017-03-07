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
    ParamGroup(const std::string &file = "", const std::string &ns = "/fst_param");
    ~ParamGroup();

    bool deleteParam(const std::string &key);
    void deleteParamTree(void);

    const ErrorCode& getLastError(void);
    void clearLastError(void);

    const std::string getNamespace(void);
    // bool setNamespace(const std::string &space);
    
    bool getParam(const std::string &key, int &value);
    bool getParam(const std::string &key, double &value);
    bool getParam(const std::string &key, bool &value);
    bool getParam(const std::string &key, std::string &value);
    bool getParam(const std::string &key, std::vector<int> &value);
    bool getParam(const std::string &key, std::vector<double> &value);
    bool getParam(const std::string &key, std::vector<std::string> &value);
    bool getParam(const std::string &key, std::map<std::string, XmlRpc::XmlRpcValue> &value);

//    bool getParamNames(std::vector<std::string> &keys);
    bool hasParam(const std::string &key);
    
    bool dumpParamFile(const std::string &file);
    bool loadParamFile(const std::string &file);

//    template<typename T>
//    bool setParam(const std::string &key, const T &value);

    bool setParam(const std::string &key, int value);
    bool setParam(const std::string &key, double value);
    bool setParam(const std::string &key, bool value);
    bool setParam(const std::string &key, const std::string &value);
    bool setParam(const std::string &key, const std::vector<int> &value);
    bool setParam(const std::string &key, const std::vector<double> &value);
    bool setParam(const std::string &key, const std::vector<std::string> &value);
    bool setParam(const std::string &key, const std::map<std::string, XmlRpc::XmlRpcValue> &value);
    
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
    
//    void test(void);

  private:
    ScalarType judgeType(const std::string &scalar);
    bool parseScalar(const std::string &scalar, XmlRpc::XmlRpcValue &value);
    void split(const std::string &raw, std::vector<std::string> &cooked);
    std::string resolve(const std::string &str);

    inline void logMessage(const std::string &str);
    inline void printInfo(const char *str);
    template<typename T>
    inline void printInfo(const char *str, T value);
    inline void printError(const char *str);
    inline void printError(const char *str, const ErrorCode &err);

    YAML::Node *root_;
    std::string root_namespace_;
    std::string sub_namespace_;
    bool is_initialized_;
    ErrorCode last_error_;
};

}

#endif
