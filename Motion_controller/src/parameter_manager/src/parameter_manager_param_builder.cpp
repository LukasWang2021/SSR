/*************************************************************************
	> File Name: parameter_manager_param_builder.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年03月24日 星期五 09时53分36秒
 ************************************************************************/

#include<iostream>
#include<cstring>
#include<vector>
#include<string>
#include<sstream>
#include<unistd.h>
#include<parameter_manager/parameter_manager_param_builder.h>

using std::string;
using std::vector;

namespace fst_parameter {

bool ParamBuilder::buildParamFromString(const string &yaml_str, ParamValue &param_value) {
    vector<YamlLine> lines;

    try {
        string raw = yaml_str;
        cleanYamlString(raw);
        splitYamlString(raw, lines);
        cleanYamlLines(lines);
        analysisYamlLines(lines);
        
        //unsigned int cnt = 0;
        //for (vector<YamlLine>::iterator it = lines.begin(); it != lines.end(); ++it) {
        //    std::cout << "line " << cnt << ": indent=" << it->indent << " content=" << it->content << " name=" << it->name << " value=" << it->value << std::endl;
        //    cnt++;
        //}
        buildParamFromLines(lines.begin(), lines.end(), param_value);
    }
    catch(ParamException e) {
        std::cout << e.getMessage() << std::endl;
        std::cout << "Error while building ParamValue, error code=" << e.getCode() << std::endl;
        return false;
    }

    return true;
}

void ParamBuilder::buildParamFromLines(vector<YamlLine>::iterator begin,
                                       vector<YamlLine>::iterator end,
                                       ParamValue &value) {
    value.clear();

    // empty vector
    if (begin == end) {
        return;
    }
    // vector contains only one YAML line
    else if (begin + 1 == end) {
        if (!begin->name.empty() && !begin->value.empty()) {
            value[begin->name] = getParamValueFromString(begin->value);
        }
        else
        {
            string error_string = "Cannot parse YAML string, name=" + begin->name + " value=" + begin->value;
            throw ParamException(error_string, PARAM_PARSE_ERROR);
        }
    }
    else {
        vector<YamlLine>::iterator it = begin;
        while (it != end) {
            if (!it->name.empty() && !it->value.empty()) {
                // there is a map element
                // std::cout << "single line" << std::endl;
                value[it->name] = getParamValueFromString(it->value);
                ++it;
            }
            else if (!it->name.empty() && it->value.empty() && (it + 1 != end) && (it + 1)->indent > it->indent) {
                // there is a map
                // std::cout << "is a map" << std::endl;
                vector<YamlLine>::iterator temp_it = it + 1;
                while (temp_it->indent > it->indent && temp_it != end)  ++temp_it;
                ParamValue temp_value;
                buildParamFromLines(it + 1, temp_it, temp_value);
                value[it->name] = temp_value;
                it = temp_it;
            }
            else if (it->name.empty() && !it->value.empty()) {
                // there is a array
                // std::cout << "is a array" << std::endl;
                vector<YamlLine>::iterator temp_it = it;
                unsigned int cnt = 0;
                while (temp_it->name.empty() && !temp_it->value.empty() && 
                       temp_it->indent == it->indent) {
                    size_t pos = temp_it->value.find_first_not_of(" ");
                    value[cnt++] = getParamValueFromString(temp_it->value.substr(pos));
                    ++temp_it;
                }

                it = temp_it;
            }
            else {
                string error_string = "Cannot parse YAML string, name=" + it->name + " value=" + it->value;
                throw ParamException(error_string, PARAM_PARSE_ERROR);
            }
        }
    }
}

ParamValue ParamBuilder::getParamValueFromString(const string &str) {
    //std::cout << "raw:" << str << std::endl;
    ParamValue value;
    string copy_str(str);
    cleanYamlString(copy_str);
    //std::cout << "clean:\n" << copy_str << std::endl;
    if (copy_str.empty()) {
        string error_string = "Parse error, try to build a ParamValue from empty string";
        throw ParamException(error_string, PARAM_PARSE_ERROR);
    }

    if (copy_str[0] == '{') {
        // Is a map
        size_t ptr = 0;
        size_t pos = copy_str.find_first_of(':');
        if (pos == string::npos) {
            string error_string = "Parse error, string=" + copy_str;
            throw ParamException(error_string, PARAM_PARSE_ERROR);
        }
        string struct_name  = "";
        string struct_value = "";

        while (pos != string::npos) {
            pos = deleteSpaceAround(copy_str, pos);
            ptr = copy_str.find_last_of(" ,{", pos);
            ptr++;
            struct_name = copy_str.substr(ptr, pos - ptr);
            if (struct_name.empty()) {
                string error_string = "Parse error, string=" + copy_str;
                throw ParamException(error_string, PARAM_PARSE_ERROR);
            }
            if (copy_str[pos + 1] == '{') {
                size_t temp_ptr = pos + 1;
                size_t cnt = 0;
                while (temp_ptr < copy_str.length()) {
                    if      (copy_str[temp_ptr] == '{') cnt++;
                    else if (copy_str[temp_ptr] == '}') cnt--;
                    if (cnt == 0) break;
                    temp_ptr++;
                }
                if (cnt == 0) {
                    struct_value = copy_str.substr(pos + 1, temp_ptr - pos);
                    value[struct_name] = getParamValueFromString(struct_value);
                    pos = copy_str.find_first_of(':', temp_ptr);
                }
                else {
                    string error_string = "Parse error, string=" + copy_str.substr(pos + 1, temp_ptr - pos);
                    throw ParamException(error_string, PARAM_PARSE_ERROR);
                }
            }
            else if (copy_str[pos + 1] == '[') {
                size_t temp_ptr = pos + 1;
                size_t cnt = 0;
                while (temp_ptr < copy_str.length()) {
                    if      (copy_str[temp_ptr] == '[') cnt++;
                    else if (copy_str[temp_ptr] == ']') cnt--;
                    if (cnt == 0) break;
                    temp_ptr++;
                }
                if (cnt == 0) {
                    struct_value = copy_str.substr(pos + 1, temp_ptr - pos);
                    value[struct_name] = getParamValueFromString(struct_value);
                    pos = copy_str.find_first_of(':', temp_ptr);
                }
                else {
                    string error_string = "Parse error, string=" + copy_str.substr(pos + 1, temp_ptr - pos);
                    throw ParamException(error_string, PARAM_PARSE_ERROR);
                }
            }
            else {
                size_t temp_ptr = pos + 1;
                size_t end_ptr  = copy_str.find_first_of(" ,}", temp_ptr);
                value[struct_name] = getParamValueFromString(copy_str.substr(temp_ptr, end_ptr - temp_ptr));
                pos = copy_str.find_first_of(':', end_ptr);
            }
        }
        return value;
    }
    else if (copy_str[0] == '[') {
        // Is an array
        deleteSpaceAround(copy_str, 0);
        string array_value = "";
        size_t pos = 1;
        size_t array_cnt = 0;
        
        while (pos < copy_str.length()) {
            if (copy_str[pos] == '{') {
                size_t temp_ptr = pos;
                size_t temp_cnt = 0;
                while (temp_ptr < copy_str.length()) {
                    if      (copy_str[temp_ptr] == '{') temp_cnt++;
                    else if (copy_str[temp_ptr] == '}') temp_cnt--;
                    temp_ptr++;
                    if (temp_cnt == 0) break;
                }
                if (temp_cnt == 0) {
                    array_value = copy_str.substr(pos, temp_ptr - pos);
                    value[array_cnt++] = getParamValueFromString(array_value);
                    pos = copy_str.find_first_of(',', temp_ptr);
                    if (pos != string::npos)    pos = copy_str.find_first_not_of(' ', pos + 1);
                }
                else {
                    string error_string = "Parse error, string=" + copy_str.substr(pos, temp_ptr - pos);
                    throw ParamException(error_string, PARAM_PARSE_ERROR);
                }
            }
            else if (copy_str[pos] == '[') {
                size_t temp_ptr = pos;
                size_t temp_cnt = 0;
                while (temp_ptr < copy_str.length()) {
                    if      (copy_str[temp_ptr] == '[') temp_cnt++;
                    else if (copy_str[temp_ptr] == ']') temp_cnt--;
                    temp_ptr++;
                    if (temp_cnt == 0) break;
                }
                if (temp_cnt == 0) {
                    array_value = copy_str.substr(pos, temp_ptr - pos);
                    value[array_cnt++] = getParamValueFromString(array_value);
                    pos = copy_str.find_first_of(',', temp_ptr);
                    if (pos != string::npos)    pos = copy_str.find_first_not_of(' ', pos + 1);
                }
                else {
                    string error_string = "Parse error, string=" + copy_str.substr(pos, temp_ptr - pos);
                    throw ParamException(error_string, PARAM_PARSE_ERROR);
                }
            }
            else {
                size_t temp_ptr = pos;
                size_t end_ptr  = copy_str.find_first_of(",]", temp_ptr);
                value[array_cnt++] = getParamValueFromString(copy_str.substr(temp_ptr, end_ptr - temp_ptr));
                pos = copy_str.find_first_of(',', end_ptr);
                if (pos != string::npos)    pos = copy_str.find_first_not_of(' ', pos + 1);
            }
        }
        return value;
    }
    else if (copy_str[0] == '\'' || copy_str[0] == '\"'){
        // Is a string
        size_t begin = copy_str.find_first_not_of("\'\"");
        size_t end   = copy_str.find_last_not_of(" \'\"");
        value = str.substr(begin, end - begin + 1);
        return value;

    }
    else {
        // Is a scalar
        std::stringstream ss(copy_str);
        int int_value;
        ss >> int_value;
        if (ss.eof()) {
            value = int_value;
            return value;
        }
    
        ss.str("");
        ss.clear();
        ss << copy_str;
        double double_value;
        ss >> double_value;
        if (ss.eof()) {
            value = double_value;
            return value;
        }

        if (copy_str == "true" || copy_str == "True" || copy_str == "TRUE") {
            value = true;
            return value;
        }
        else if (copy_str == "false" || copy_str == "False" || copy_str == "FALSE") {
            value = false;
            return value;
        }

        value = str;
        return value;
    }
}

void ParamBuilder::analysisYamlLines(vector<YamlLine> &lines) {
    vector<YamlLine>::iterator it;
    //std::cout << "YAML Lines:" << std::endl;
    //for (it = lines.begin(); it != lines.end(); ++it)
    //   std::cout << it->content << std::endl;
    for (it = lines.begin(); it != lines.end(); ++it) {
        it->indent = it->content.find_first_not_of(' ');
        if (it->indent > 0) it->content.erase(0, it->indent);

        if (it->content[0] == '-') {
 
            it->name.clear();
            deleteSpaceAround(it->content, 0);
            it->value = it->content.substr(1);
        }
        else {
            size_t pos = it->content.find_first_of(':');
            if (pos != string::npos) {
                if (pos != 0) {
                    pos = deleteSpaceAround(it->content, pos);
                    it->name  = it->content.substr(0, pos);
                    it->value = it->content.substr(pos + 1);
                }
                else {
                    string error_string = "Format error, string=" + it->content;
                    throw ParamException(error_string, PARAM_PARSE_ERROR);
                }
            }
            else {
                string error_string = "Format error, string=" + it->content;
                throw ParamException(error_string, PARAM_PARSE_ERROR);
                //it->name.clear();
                //if (pos == string::npos)    it->value = it->content;
                //else                        it->value = it->content.substr(pos + 1);
            }
        }
    }
    for (it = lines.begin(); it != lines.end(); ++it) {
        if (it->content[0] == '-') {
            if (it == lines.begin()) {
                string error_string = "A valid config file should begin as a struct, string=" + it->content;
                throw ParamException(error_string, PARAM_PARSE_ERROR);
            }
            else {
                if ((it - 1)->content[0] == '-' && (it - 1)->indent != it->indent) {
                    string error_string = "Format error:\n" + (it - 1)->content + "\n" + it->content;
                    throw ParamException(error_string, PARAM_PARSE_ERROR);
                }
                if ((it - 1)->content[0] != '-' && (it - 1)->indent >= it->indent) {
                    string error_string = "Format error:\n" + (it - 1)->content + "\n" + it->content;
                    throw ParamException(error_string, PARAM_PARSE_ERROR);
                }
            }
            if ((it + 1) != lines.end()) {
                if ((it + 1)->content[0] == '-' && (it + 1)->indent != it->indent) {
                    string error_string = "Format error:\n" + it->content + "\n" + (it + 1)->content;
                    throw ParamException(error_string, PARAM_PARSE_ERROR);
                }
                if ((it + 1)->content[0] != '-' && (it + 1)->indent >= it->indent) {
                    string error_string = "Format error:\n" + it->content + "\n" + (it + 1)->content;
                    throw ParamException(error_string, PARAM_PARSE_ERROR);
                }
            }
        }
        else {
            if (it->name.empty()) {
                string error_string = "A struct line should begin with a name, string=" + it->content;
                throw ParamException(error_string, PARAM_PARSE_ERROR);
            }
            else {
                if (!it->value.empty()) {
                    if ((it + 1) != lines.end() && (it + 1)->indent > it->indent) {
                        string error_string = "Format error:\n" + it->content + "\n" + (it + 1)->content;
                        throw ParamException(error_string, PARAM_PARSE_ERROR);
                    }
                }
                else {
                    if ((it + 1) == lines.end() || (it + 1)->indent <= it->indent) {
                        string error_string = "Format error:\n" + it->content + "\n" + (it + 1)->content;
                        throw ParamException(error_string, PARAM_PARSE_ERROR);
                    }
                }
            }
        }
    }
}

void ParamBuilder::cleanYamlLines(vector<YamlLine> &lines) {
    vector<YamlLine>::iterator it;
    for (it = lines.begin(); it != lines.end(); ) {
        if (isEmptyLine(*it)) {
            lines.erase(it);
        }
        else {
            // delete spaces at the end of line
            deleteSpaceAround(it->content, it->content.length());
            ++it;
        }
    }
}

// Delete spaces around the character specified by pos in str
// If pos >= the length of str, then delete all the spaces at the end of str
size_t ParamBuilder::deleteSpaceAround(string &str, size_t pos) {
    if (pos >= str.length()) pos = str.length();

    size_t ptr = pos + 1;
    size_t cnt = 0;
    while (ptr < str.length() && str[ptr] == ' ') {
        cnt++;
        ptr++;
    }
    if (cnt > 0) str.erase(pos + 1, cnt);

    if (pos == 0) return pos;
    
    ptr = pos - 1;
    cnt = 0;
    while(ptr < str.length() && str[ptr] == ' ') {
        cnt++;
        ptr--;
    }
    if (cnt > 0) str.erase(pos - cnt, cnt);
    return pos - cnt;
}

bool ParamBuilder::isEmptyLine(const YamlLine &line) {
    size_t pos = line.content.find_first_not_of(" ");
    
    if (pos == string::npos)        return true;
    if (line.content[pos] == '#')   return true;
    return false;
}

void ParamBuilder::splitYamlString(const string &raw, vector<YamlLine> &cooked) {
    cooked.clear();
    if (raw.empty())
        return;
    YamlLine yaml_line;
    string sep = "\n";
    string temp;
    string::size_type begin = raw.find_first_not_of(sep);
    string::size_type position = 0;

    while (begin != string::npos) {
        position = raw.find(sep, begin);
        if (position != string::npos) {
            temp = raw.substr(begin, position - begin);
            begin = position + sep.length();
        }
        else {
            temp = raw.substr(begin);
            begin = position;
        }

        if (!temp.empty()) {
            yaml_line.content = temp;
            cooked.push_back(yaml_line);
            temp.clear();
        }
    }
    return;
}

void ParamBuilder::cleanYamlString(string &str) {
    if (str.empty())
        return;

    //std::cout << "raw_str length=" << str.length() << std::endl;

    /*
    char *raw = new char[str.length() + 1];
    memcpy(raw, str.c_str(), str.length());
    raw[str.length()] = '\0';
    */

    // delete all white spaces in back of the string
    size_t pos = str.find_last_not_of(" \r\n\t\v\f");
    if (pos == string::npos) {
        str.clear();
        return;
    }
    else {
        if (pos < str.length() - 1)     str.erase(pos + 1);
    }
    /*
    char *pchar = raw + (int)str.length() - 1;
    while (pchar >= raw && (*pchar == '\n' || *pchar == '\r' || *pchar == ' ')) {
        *pchar = '\0';
        --pchar;
    }*/

    // delete all white spaces in front of the string
    pos = str.find_first_not_of(" \r\n\t\v\f");
    if (pos == string::npos) {
        str.clear();
        return;
    }
    else {
        str = str.substr(pos);
    }
    /*
    pchar = raw;
    int cnt = 0;
    while (*pchar == '\n' || *pchar == '\r' || *pchar == ' ') {
        cnt++;
        pchar++;
    }
    if (cnt > 0) {
        char *tmp = raw;
        while (*pchar != '\0') {
            *tmp = *pchar;
              tmp++;
            pchar++;
        }
        *tmp = '\0';
    }*/

    // delete all '\r' '\t' '\v' '\f' in the string
    pos = str.find_first_of("\r\t\v\f");
    while(pos != string::npos) {
        str.erase(pos, 1);
        pos = str.find_first_of("\r\t\v\f", pos);
    }
    /*
    for (pchar = raw; *pchar != '\0';) {
        if (*pchar == '\r') {
            for (char *tmp = pchar; *tmp != '\0'; ++tmp)
                *tmp = *(tmp + 1);
        }
        else {
            pchar++;
        }
    }*/

    /*
    // delete all '\n' following '\n'
    for (pchar = raw; *pchar != '\0'; ++pchar) {
        if (*pchar == '\n') {
            int cnt = 0;
            char *tmp = pchar + 1;
            while (*tmp == '\n') {
                cnt++;
                tmp++;
            }
            if (cnt > 0) {
                char *des = pchar + 1;
                while (*tmp != '\0') {
                    *des = *tmp;
                    tmp++;
                    des++;
                }
                *des = '\0';
            }
        }
    }
    str = raw;
    delete[] raw;
    */
    //std::cout << "cooked str length=" << str.length() << ":" << std::endl << str << std::endl;
}

}

