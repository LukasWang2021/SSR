#include "model_base.h"
#include <cstring>

using namespace system_model_space;
using namespace base_space;
using namespace std;

ModelBase::ModelBase(ModelType_e type, std::string style_str, std::string file_path,   ParamDetail_t* param_ptr, uint32_t param_number)
{
    type_ = type;
    name_.empty();
    style_ = style_str;
    file_path_ = file_path;
    param_ptr_ = param_ptr;
    param_number_ = param_number;
}

ModelBase::~ModelBase()
{

}

void ModelBase::addParam(const std::string& param_name, const uint32_t param_index)
{
    ModelParamMap_t param;
    param.param_name = param_name;
    param.param_index = param_index;
    param_map_.push_back(param);
}

bool ModelBase::load()
{
    if(!yaml_help_.loadParamFile(file_path_))
    {
        return false;
    }
    if(!yaml_help_.getParam("name", name_))
    {
        return false;   
    }
    if(param_map_.size() != param_number_)
    {
        return false;
    }
    std::string unit;
    for(size_t i = 0; i < param_number_; ++i)
    {
        if(!yaml_help_.getParam(param_map_[i].param_name + "/operation_value", param_ptr_[param_map_[i].param_index].operation_value)
            || !yaml_help_.getParam(param_map_[i].param_name + "/default_value", param_ptr_[param_map_[i].param_index].default_value)
            || !yaml_help_.getParam(param_map_[i].param_name + "/upper_limit_value", param_ptr_[param_map_[i].param_index].upper_limit_value)
            || !yaml_help_.getParam(param_map_[i].param_name + "/lower_limit_value", param_ptr_[param_map_[i].param_index].lower_limit_value)
            || !yaml_help_.getParam(param_map_[i].param_name + "/attr", param_ptr_[param_map_[i].param_index].attr.all)
            || !yaml_help_.getParam(param_map_[i].param_name + "/validity", param_ptr_[param_map_[i].param_index].validity.all)
            || !yaml_help_.getParam(param_map_[i].param_name + "/unit", unit))
        {
            return false;
        }
        //printf("param name=%s, value=%d\n",param_map_[i].param_name.c_str(), param_ptr_[param_map_[i].param_index].operation_value);       
        formatUnitStr2Char(unit, param_ptr_[param_map_[i].param_index].unit);
    }
    return true;
}

bool ModelBase::save()
{
    if(!yaml_help_.setParam("name", name_))
    {
        return false;
    }

    std::string unit;
    for(size_t i = 0; i < param_number_; ++i)
    {
        formatUnitChar2Str(param_ptr_[param_map_[i].param_index].unit, unit);
        if(!yaml_help_.setParam(param_map_[i].param_name + "/operation_value", param_ptr_[param_map_[i].param_index].operation_value)
            || !yaml_help_.setParam(param_map_[i].param_name + "/default_value", param_ptr_[param_map_[i].param_index].default_value)
            || !yaml_help_.setParam(param_map_[i].param_name + "/upper_limit_value", param_ptr_[param_map_[i].param_index].upper_limit_value)
            || !yaml_help_.setParam(param_map_[i].param_name + "/lower_limit_value", param_ptr_[param_map_[i].param_index].lower_limit_value)
            || !yaml_help_.setParam(param_map_[i].param_name + "/attr", param_ptr_[param_map_[i].param_index].attr.all)
            || !yaml_help_.setParam(param_map_[i].param_name + "/validity", param_ptr_[param_map_[i].param_index].validity.all)
            || !yaml_help_.setParam(param_map_[i].param_name + "/unit", unit))
        {
            return false;
        }
    }
    if(!yaml_help_.dumpParamFile(file_path_))
    {
        return false;
    }
    return true;
}

ModelType_e ModelBase::getType()
{
    return type_;
}

std::string ModelBase::getName()
{
    return name_;
}

std::string ModelBase::getStyle()
{
    return style_;
}

void ModelBase::setName(const std::string& name)
{
    name_ = name;
}

uint32_t ModelBase::getParamNumber()
{
    return param_number_;
}

bool ModelBase::set(const uint32_t param_index, const int32_t param_value)
{
    if(param_index < param_number_ 
        && param_ptr_[param_index].attr.bit.write == 1)
    {
        if((param_ptr_[param_index].lower_limit_value == param_ptr_[param_index].upper_limit_value)
            || (param_value >= param_ptr_[param_index].lower_limit_value && param_value <= param_ptr_[param_index].upper_limit_value))
        {
            param_ptr_[param_index].operation_value = param_value;
            return true;
        }
    }
    return false;
}

bool ModelBase::get(const uint32_t param_index, int32_t* param_value_ptr)
{
    if(param_index < param_number_
        && param_ptr_[param_index].attr.bit.read == 1)
    {
        *param_value_ptr = param_ptr_[param_index].operation_value;
        return true;
    }
    else
    {
        return false;
    }
}

bool ModelBase::getDetail(const uint32_t param_index, ParamDetail_t* param_detail_ptr)
{
    if(param_index < param_number_
        && param_ptr_[param_index].attr.bit.read == 1)
    {
        *param_detail_ptr = param_ptr_[param_index];
        return true;
    }
    else
    {
        return false;
    }
}

ParamDetail_t* ModelBase::getAll()
{
    return param_ptr_;
}

void ModelBase::formatUnitStr2Char(const std::string& unit_str, char* unit_char16)
{
    size_t unit_length = unit_str.length();
    if(unit_length >= 16)
    {
        unit_length = 15;
    }
    memcpy(unit_char16, unit_str.c_str(), unit_length);
    unit_char16[unit_length] = 0;
}

void ModelBase::formatUnitChar2Str(const char* unit_char16, std::string& unit_str)
{
    unit_str = unit_char16;
}



