#include "tool_manager.h"
#include "common_file_path.h"
#include <cstring>
#include <sstream>
#include "error_code.h"


using namespace fst_ctrl;
using namespace basic_alg;

ToolManager::ToolManager():
    param_ptr_(NULL),
    log_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ToolManagerParam();
    FST_LOG_INIT("ToolManager");
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
}

ToolManager::~ToolManager()
{
    if(log_ptr_ != NULL)
    {
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if(param_ptr_ != NULL)
    {
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

ErrorCode ToolManager::init()
{
    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load ToolManager component parameters");
        return TOOL_MANAGER_LOAD_PARAM_FAILED;
    } 
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);   

    tool_info_file_path_ = std::string(TOOL_DIR);
    tool_info_file_path_ += param_ptr_->tool_info_file_name_;
    if(!readAllToolInfoFromYaml(param_ptr_->max_number_of_tools_))
    {
        FST_ERROR("Failed to load tool info");
        return TOOL_MANAGER_LOAD_TOOLINFO_FAILED;
    }
    return SUCCESS;
}

ErrorCode ToolManager::addTool(ToolInfo& info)
{
    if(info.id >= static_cast<int>(tool_set_.size())
        || info.id == 0
        || tool_set_[info.id].is_valid)
    {
        FST_ERROR("Failed to add tool %d, invalid tool id", info.id);
        return TOOL_MANAGER_INVALID_ARG;
    }
        
    tool_set_[info.id].id = info.id;
    tool_set_[info.id].is_valid = true;
    if(info.name.size() == 0)
    {
        tool_set_[info.id].name = std::string("default");
    }
    else
    {
        tool_set_[info.id].name = info.name;
    }
    if(info.comment.size() == 0)
    {
        tool_set_[info.id].comment = std::string("default");
    }
    else
    {
        tool_set_[info.id].comment = info.comment;
    }
    tool_set_[info.id].group_id = info.group_id;
    tool_set_[info.id].data = info.data;
    if(!writeToolInfoToYaml(tool_set_[info.id]))
    {
        FST_ERROR("Failed to add tool %d, write yaml failed", info.id);
        return TOOL_MANAGER_TOOLINFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode ToolManager::deleteTool(int id)
{
    if(id >= static_cast<int>(tool_set_.size())
        || id == 0)
    {
        FST_ERROR("Failed to delete tool %d, invalid tool id", id);
        return TOOL_MANAGER_INVALID_ARG;
    }
        
    tool_set_[id].is_valid = false;
    tool_set_[id].name = std::string("default");
    tool_set_[id].comment = std::string("default");
    tool_set_[id].group_id = -1;
    memset(&tool_set_[id].data, 0, sizeof(PoseEuler));
    if(!writeToolInfoToYaml(tool_set_[id]))
    {
        FST_ERROR("Failed to delete tool %d, write yaml failed", id);
        return TOOL_MANAGER_TOOLINFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode ToolManager::updateTool(ToolInfo& info)
{
    if(info.id >= static_cast<int>(tool_set_.size())
        || info.id == 0
        || !tool_set_[info.id].is_valid)
    {
        FST_ERROR("Failed to update tool %d, invalid tool id", info.id);
        return TOOL_MANAGER_INVALID_ARG;
    }
        
    tool_set_[info.id].id = info.id;
    tool_set_[info.id].is_valid = true;
    if(info.name.size() == 0)
    {
        tool_set_[info.id].name = std::string("default");
    }
    else
    {
        tool_set_[info.id].name = info.name;
    }
    if(info.comment.size() == 0)
    {
        tool_set_[info.id].comment = std::string("default");
    }
    else
    {
        tool_set_[info.id].comment = info.comment;
    }
    tool_set_[info.id].group_id = info.group_id;
    tool_set_[info.id].data = info.data;

    if(!writeToolInfoToYaml(tool_set_[info.id]))
    {
        FST_ERROR("Failed to update tool %d, write yaml failed", info.id);
        return TOOL_MANAGER_TOOLINFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode ToolManager::moveTool(int expect_id, int original_id)
{
    if(tool_set_[expect_id].is_valid
        || original_id == expect_id
        || !tool_set_[original_id].is_valid)
    {
        FST_ERROR("Failed to move tool %d to %d, invalid tool id", expect_id, original_id);
        return TOOL_MANAGER_INVALID_ARG;
    }

    tool_set_[expect_id].id = expect_id;
    tool_set_[expect_id].is_valid = true;
    tool_set_[expect_id].name = tool_set_[original_id].name;
    tool_set_[expect_id].comment = tool_set_[original_id].comment;
    tool_set_[expect_id].group_id = tool_set_[original_id].group_id;
    tool_set_[expect_id].data = tool_set_[original_id].data;

    tool_set_[original_id].id = original_id;
    tool_set_[original_id].is_valid = false;
    tool_set_[original_id].name = std::string("default");
    tool_set_[original_id].comment = std::string("default");
    tool_set_[original_id].group_id = -1;
    memset(&tool_set_[original_id].data, 0, sizeof(PoseEuler));

    if(!writeToolInfoToYaml(tool_set_[original_id]) 
        || !writeToolInfoToYaml(tool_set_[expect_id]))
    {
        FST_ERROR("Failed to move tool %d to %d, write yaml failed", expect_id, original_id);
        return TOOL_MANAGER_TOOLINFO_FILE_WRITE_FAILED;
    }
    return SUCCESS;
}

ErrorCode ToolManager::getToolInfoById(int id, ToolInfo& info)
{
    if(id >= static_cast<int>(tool_set_.size())
        || id <= 0)
    {
        FST_ERROR("Failed to get tool %d information, invalid tool id", id);
        return TOOL_MANAGER_INVALID_ARG;
    }

    info.id = tool_set_[id].id;
    info.is_valid = tool_set_[id].is_valid;
    info.name = tool_set_[id].name;
    info.comment = tool_set_[id].comment;
    info.group_id = tool_set_[id].group_id;
    info.data = tool_set_[id].data;
    return SUCCESS;
}

std::vector<ToolSummaryInfo> ToolManager::getAllValidToolSummaryInfo()
{
    ToolSummaryInfo summary_info;
    std::vector<ToolSummaryInfo> summary_list;
    for(unsigned int i = 1; i < tool_set_.size(); ++i)
    {
        if(tool_set_[i].is_valid)
        {
            summary_info.id = i;
            summary_info.name = tool_set_[i].name;
            summary_info.comment = tool_set_[i].comment;
            summary_info.group_id = tool_set_[i].group_id;
            summary_list.push_back(summary_info);
        }
    }
    return summary_list;
}

void ToolManager::packDummyToolInfo(ToolInfo& info)
{
    info.id = 0;
    info.is_valid = true;
    info.name = std::string("no tool");
    info.comment = std::string("no tool");
    info.group_id = -1;
    memset(&info.data, 0, sizeof(PoseEuler));
}

std::string ToolManager::getToolInfoPath(int tool_id)
{
    std::string id_str;
    std::stringstream stream;
    stream << tool_id;
    stream >> id_str;
    return (std::string("tool") + id_str);
}

bool ToolManager::readAllToolInfoFromYaml(int number_of_tools)
{
    ToolInfo info;

    tool_set_.clear();
    packDummyToolInfo(info);
    tool_set_.push_back(info);
    
    if (tool_info_yaml_help_.loadParamFile(tool_info_file_path_.c_str()))
    {
        for(int i = 1; i <= number_of_tools; ++i)
        {
            std::string tool_info_path;
            tool_info_path = getToolInfoPath(i);
            tool_info_yaml_help_.getParam(tool_info_path + "/id", info.id);
            if(info.id != i)
            {
                return false;
            }
            tool_info_yaml_help_.getParam(tool_info_path + "/is_valid", info.is_valid);
            tool_info_yaml_help_.getParam(tool_info_path + "/name", info.name);
            tool_info_yaml_help_.getParam(tool_info_path + "/comment", info.comment);
            tool_info_yaml_help_.getParam(tool_info_path + "/group_id", info.group_id);
            std::string point_path = tool_info_path + std::string("/point");
            tool_info_yaml_help_.getParam(point_path + "/x", info.data.point_.x_);
            tool_info_yaml_help_.getParam(point_path + "/y", info.data.point_.y_);
            tool_info_yaml_help_.getParam(point_path + "/z", info.data.point_.z_);
            std::string euler_path = tool_info_path + std::string("/euler");
            tool_info_yaml_help_.getParam(euler_path + "/a", info.data.euler_.a_);
            tool_info_yaml_help_.getParam(euler_path + "/b", info.data.euler_.b_);
            tool_info_yaml_help_.getParam(euler_path + "/c", info.data.euler_.c_);
            tool_set_.push_back(info);
        }
	    return true;
    }
    else
    {
        FST_ERROR("Failed to read tool info from yaml: %s", tool_info_file_path_.c_str());
	    return false;
    }
}

bool ToolManager::writeToolInfoToYaml(ToolInfo& info)
{
    std::string tool_info_path = getToolInfoPath(info.id);
    tool_info_yaml_help_.setParam(tool_info_path + "/id", info.id);
    tool_info_yaml_help_.setParam(tool_info_path + "/is_valid", info.is_valid);
    tool_info_yaml_help_.setParam(tool_info_path + "/name", info.name);
    tool_info_yaml_help_.setParam(tool_info_path + "/comment", info.comment);
    tool_info_yaml_help_.setParam(tool_info_path + "/group_id", info.group_id);
    std::string point_path = tool_info_path + std::string("/point");
    tool_info_yaml_help_.setParam(point_path + "/x", info.data.point_.x_);
    tool_info_yaml_help_.setParam(point_path + "/y", info.data.point_.y_);
    tool_info_yaml_help_.setParam(point_path + "/z", info.data.point_.z_);
    std::string euler_path = tool_info_path + std::string("/euler");
    tool_info_yaml_help_.setParam(euler_path + "/a", info.data.euler_.a_);
    tool_info_yaml_help_.setParam(euler_path + "/b", info.data.euler_.b_);
    tool_info_yaml_help_.setParam(euler_path + "/c", info.data.euler_.c_);
    if(tool_info_yaml_help_.dumpParamFile(tool_info_file_path_.c_str()))
    {
        return true;
    }
    else
    {
        FST_ERROR("Failed to write tool info to yaml: %s", tool_info_file_path_.c_str());
        return false;
    }
}


