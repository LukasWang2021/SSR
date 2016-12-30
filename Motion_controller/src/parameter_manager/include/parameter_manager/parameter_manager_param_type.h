/*************************************************************************
	> File Name: parameter_manager_type.h
	> Author: 
	> Mail: 
	> Created Time: 2016年11月07日 星期一 15时56分58秒
 ************************************************************************/

#ifndef _PARAMETER_MANAGER_PARAM_TYPE_H
#define _PARAMETER_MANAGER_PARAM_TYPE_H

#include <map>
#include <string>
#include <vector>

namespace fst_parameter {

struct Param;
typedef std::vector<char> BinaryDate;
typedef std::vector<Param> ParamArray;
typedef std::map<std::string, Param> ParamStruct;

enum Type {
    e_type_invalid,
    e_type_bool,
    e_type_int,
    e_type_double,
    e_type_string,
    e_type_array,
    e_type_struct
};

struct Param {
    Type type;
    union {
        bool    as_bool;
        int     as_int;
        double  as_double;
        std::string* as_string;
        ParamArray*  as_array;
        ParamStruct* as_struct;
    } value;
};


}

std::ostream& operator<<(std::ostream& os, fst_parameter::Param& v);

#endif
