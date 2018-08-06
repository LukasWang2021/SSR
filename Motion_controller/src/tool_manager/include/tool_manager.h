#ifndef TOOL_MANAGER_H
#define TOOL_MANAGER_H


#include "tool_manager_param.h"
#include "common_log.h"
#include <string>
#include <vector>
#include "base_datatype.h"
#include "parameter_manager/parameter_manager_param_group.h"


namespace fst_ctrl
{
typedef struct
{
    int id;
    bool is_valid;
    std::string name;
    std::string comment;
    int group_id;
    fst_base::PoseEuler data;
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

    bool init();

    bool addTool(ToolInfo& info);
    bool deleteTool(int id);
    bool updateTool(ToolInfo& info);
    bool moveTool(int expect_id, int original_id);
    bool getToolInfoById(int id, ToolInfo& info);
    std::vector<ToolSummaryInfo> getAllValidToolSummaryInfo();   

private:
    ToolManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    std::vector<ToolInfo> tool_set_;
    fst_parameter::ParamGroup tool_info_yaml_help_;
    std::string tool_info_file_path_;

    void packDummyToolInfo(ToolInfo& info);
    std::string getToolInfoPath(int tool_id);
    bool readAllToolInfoFromYaml(int number_of_tools);
    bool writeToolInfoToYaml(ToolInfo& info);
};

}

#endif


