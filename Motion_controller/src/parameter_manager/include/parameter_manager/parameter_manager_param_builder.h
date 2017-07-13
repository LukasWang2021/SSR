/*************************************************************************
	> File Name: parameter_manager_param_builder.h
	> Author: 
	> Mail: 
	> Created Time: 2017年03月24日 星期五 09时54分44秒
 ************************************************************************/

#ifndef _PARAMETER_MANAGER_PARAM_BUILDER_H
#define _PARAMETER_MANAGER_PARAM_BUILDER_H

#include <parameter_manager/parameter_manager_param_value.h>

namespace fst_parameter {

struct YamlLine {
    std::string  content;
    unsigned int indent;
    std::string  name;
    std::string  value;
};



class ParamBuilder {
  public:
    bool buildParamFromString(const std::string &yaml_str, ParamValue &param_value);

  //private:
    void buildParamFromLines(std::vector<YamlLine>::iterator begin,
                             std::vector<YamlLine>::iterator end,
                             ParamValue &value);
    ParamValue getParamValueFromString(const std::string &str);
    void cleanYamlString(std::string &str);
    void splitYamlString(const std::string &raw, std::vector<YamlLine> &cooked);
    void cleanYamlLines(std::vector<YamlLine> &lines);
    void analysisYamlLines(std::vector<YamlLine> &lines);
    size_t deleteSpaceAround(std::string &str, size_t pos);
    bool isEmptyLine(const YamlLine &line);
};


}

#endif
