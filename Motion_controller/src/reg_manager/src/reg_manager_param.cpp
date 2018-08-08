#include "reg_manager_param.h"
#include "common_file_path.h"
#include "common_log.h"
#include <string>


using namespace fst_ctrl;

RegManagerParam::RegManagerParam():
    file_path_(COMPONENT_PARAM_FILE_DIR),
    log_level_(fst_log::MSG_LEVEL_ERROR)
{
    file_path_ += "reg_manager.yaml";
}

RegManagerParam::~RegManagerParam()
{

}
std::string reg_info_dir_;
std::string pr_reg_file_name_;
std::string hr_reg_file_name_;
std::string mr_reg_file_name_;
std::string sr_reg_file_name_;
std::string r_reg_file_name_;
int pr_reg_number_;
int hr_reg_number_;
int mr_reg_number_;
int sr_reg_number_;
int r_reg_number_;
double pr_value_limit_;
double hr_value_limit_;
int mr_value_limit_;
int sr_value_limit_;    // size in byte
double r_value_limit_;

bool RegManagerParam::loadParam()
{    
    if (!yaml_help_.loadParamFile(file_path_.c_str())
        || !yaml_help_.getParam("log_level", log_level_)
        || !yaml_help_.getParam("reg_info_dir", reg_info_dir_)
        || !yaml_help_.getParam("pr_reg_file_name", pr_reg_file_name_)
        || !yaml_help_.getParam("hr_reg_file_name", hr_reg_file_name_)
        || !yaml_help_.getParam("mr_reg_file_name", mr_reg_file_name_)
        || !yaml_help_.getParam("sr_reg_file_name", sr_reg_file_name_)
        || !yaml_help_.getParam("r_reg_file_name", r_reg_file_name_)
        || !yaml_help_.getParam("pr_reg_number", pr_reg_number_)
        || !yaml_help_.getParam("hr_reg_number", hr_reg_number_)
        || !yaml_help_.getParam("mr_reg_number", mr_reg_number_)
        || !yaml_help_.getParam("sr_reg_number", sr_reg_number_)
        || !yaml_help_.getParam("r_reg_number", r_reg_number_)
        || !yaml_help_.getParam("pr_value_limit", pr_value_limit_)
        || !yaml_help_.getParam("hr_value_limit", hr_value_limit_)
        || !yaml_help_.getParam("mr_value_limit", mr_value_limit_)
        || !yaml_help_.getParam("sr_value_limit", sr_value_limit_)
        || !yaml_help_.getParam("r_value_limit", r_value_limit_))
    {
        return false;
    }
    else    
    {
        return true;
    }
}

bool RegManagerParam::saveParam()
{
    if(!yaml_help_.setParam("log_level", log_level_)
        || !yaml_help_.setParam("reg_info_dir", reg_info_dir_)
        || !yaml_help_.setParam("pr_reg_file_name", pr_reg_file_name_)
        || !yaml_help_.setParam("hr_reg_file_name", hr_reg_file_name_)
        || !yaml_help_.setParam("mr_reg_file_name", mr_reg_file_name_)
        || !yaml_help_.setParam("sr_reg_file_name", sr_reg_file_name_)
        || !yaml_help_.setParam("r_reg_file_name", r_reg_file_name_)
        || !yaml_help_.setParam("pr_reg_number", pr_reg_number_)
        || !yaml_help_.setParam("hr_reg_number", hr_reg_number_)
        || !yaml_help_.setParam("mr_reg_number", mr_reg_number_)
        || !yaml_help_.setParam("sr_reg_number", sr_reg_number_)
        || !yaml_help_.setParam("r_reg_number", r_reg_number_)
        || !yaml_help_.setParam("pr_value_limit", pr_value_limit_)
        || !yaml_help_.setParam("hr_value_limit", hr_value_limit_)
        || !yaml_help_.setParam("mr_value_limit", mr_value_limit_)
        || !yaml_help_.setParam("sr_value_limit", sr_value_limit_)
        || !yaml_help_.setParam("r_value_limit", r_value_limit_)
        || !yaml_help_.dumpParamFile(file_path_.c_str()))
    {
        return false;
    }
    else
    {
        return true;
    }
}

