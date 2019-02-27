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
#include <error_code.h>
#include <parameter_manager/parameter_manager_param_value.h>


namespace fst_parameter
{

class ParamGroup
{
  public:
    ParamGroup(const std::string &file = "");
    ~ParamGroup();

    //------------------------------------------------------------------------------
    // 方法：  dumpParamFile
    // 摘要：  将参数树回写到yaml文件中，执行文件保护策略；
    // 输入：  file - 文件路径以及文件名
    // 输出：  None
    // 返回：  操作成功/失败
    //------------------------------------------------------------------------------
    bool dumpParamFile(const std::string &file = "");

    //------------------------------------------------------------------------------
    // 方法：  loadParamFile
    // 摘要：  读取yaml文件，执行文件保护策略，并将参数文件中的字符串解析到参数树中；
    // 输入：  file - 文件路径以及文件名
    // 输出：  None
    // 返回：  操作成功/失败
    //------------------------------------------------------------------------------
    bool loadParamFile(const std::string &file);

    //------------------------------------------------------------------------------
    // 方法：  deleteParam
    // 摘要：  删除指定参数；
    // 输入：  key - 指定参数路径，key为空时删除整棵参数树
    // 输出：  None
    // 返回：  操作成功/失败
    //------------------------------------------------------------------------------
    bool deleteParam(const std::string &key = "");

    //------------------------------------------------------------------------------
    // 方法：  clearLastError
    // 摘要：  清除记录中的错误码；
    // 输入：  None
    // 输出：  None
    // 返回：  操作成功/失败
    //------------------------------------------------------------------------------
    void clearLastError(void);

    //------------------------------------------------------------------------------
    // 方法：  getLastError
    // 摘要：  获取上一次发生错误时产生的错误码；
    // 输入：  None
    // 输出：  None
    // 返回：  错误码
    //------------------------------------------------------------------------------
    const ErrorCode& getLastError(void);

    //------------------------------------------------------------------------------
    // 方法：  getParamList
    // 摘要：  获取参数树中所有参数的路径列表；
    // 输入：  None
    // 输出：  list - 参数路径列表
    // 返回：  None
    //------------------------------------------------------------------------------
    void getParamList(std::vector<std::string> &list);

    //------------------------------------------------------------------------------
    // 方法：  getParam
    // 摘要：  获取参数树中指定的参数；
    // 输入：  key - 指定参数路径
    //        size - 数组有效长度
    // 输出：  value - 指定参数路径所对应的参数
    // 返回：  操作成功/失败
    //------------------------------------------------------------------------------
    bool getParam(const std::string &key, bool &value);
    bool getParam(const std::string &key, int &value);
    bool getParam(const std::string &key, double &value);
    bool getParam(const std::string &key, std::string &value);
    bool getParam(const std::string &key, std::vector<bool> &value);
    bool getParam(const std::string &key, std::vector<int> &value);
    bool getParam(const std::string &key, std::vector<double> &value);
    bool getParam(const std::string &key, std::vector<std::string> &value);
    bool getParam(const std::string &key, ParamValue &value);
    bool getParam(const std::string &key, int *value, size_t size);
    bool getParam(const std::string &key, double *value, size_t size);

    //------------------------------------------------------------------------------
    // 方法：  hasParam
    // 摘要：  判断参数树中是否存在指定的参数；
    // 输入：  key - 指定参数路径
    // 输出：  None
    // 返回：  参数存在/不存在
    //------------------------------------------------------------------------------
    bool hasParam(const std::string &key);
    
    //------------------------------------------------------------------------------
    // 方法：  setParam
    // 摘要：  设置参数树中指定的参数值；
    // 输入：  key - 指定参数路径
    //        value - 新的参数值
    // 输出：  None
    // 返回：  操作成功/失败
    //------------------------------------------------------------------------------
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

    //------------------------------------------------------------------------------
    // 方法：  reset
    // 摘要：  重置ParamGroup到初始状态；
    // 输入：  None
    // 输出：  None
    // 返回：  None
    //------------------------------------------------------------------------------
    void reset(void);

  private:
    void resolve(std::string &str);
    void split(const std::string &raw, std::vector<std::string> &cooked);

    //bool getNodeFromParamValue(ParamValue &value, YAML::Node &node);
    //bool getParamValueFromNode(YAML::Node &node, ParamValue &value);
    //bool getParamValueFromString(const std::string &scalar, ParamValue &value);
    ParamValue* findParam(const std::string &key);
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
