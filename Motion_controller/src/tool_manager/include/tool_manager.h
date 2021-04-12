#ifndef TOOL_MANAGER_H
#define TOOL_MANAGER_H


#include "tool_manager_param.h"
#include <string>
#include <vector>
#include "common_error_code.h"
#include "basic_alg_datatype.h"
#include "yaml_help.h"
#include "log_manager_producer.h"


namespace fst_ctrl
{
typedef struct
{
    int id;
    bool is_valid;
    std::string name;
    std::string comment;
    int group_id;
    basic_alg::PoseEuler data;
}ToolInfo;

typedef struct
{
    int id;
    std::string name;
    std::string comment;
    int group_id;
}ToolSummaryInfo;

class ToolManager
{
public:
    ToolManager();
    ~ToolManager();

    ErrorCode init();

    ErrorCode addTool(ToolInfo& info);
    ErrorCode deleteTool(int id);
    ErrorCode updateTool(ToolInfo& info);
    ErrorCode moveTool(int expect_id, int original_id);
    ErrorCode getToolInfoById(int id, ToolInfo& info);
    std::vector<ToolSummaryInfo> getAllValidToolSummaryInfo();   

private:
    ToolManagerParam* param_ptr_;
    std::vector<ToolInfo> tool_set_;
    base_space::YamlHelp tool_info_yaml_help_;
    std::string tool_info_file_path_;
    std::string tool_info_file_path_modified_;

    void packDummyToolInfo(ToolInfo& info);
    std::string getToolInfoPath(int tool_id);
    bool readAllToolInfoFromYaml(int number_of_tools);
    bool writeToolInfoToYaml(ToolInfo& info);
};

}

#endif


