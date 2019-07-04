#include "controller_rpc.h"
#include "error_code.h"
#include <unistd.h>
#include <stdlib.h>

using namespace fst_ctrl;


// "/rpc/tool_manager/addTool"
void ControllerRpc::handleRpc0x0000A22C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ToolInfo* rq_data_ptr = static_cast<RequestMessageType_ToolInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/tool_manager/addTool can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
        return;
    }

    if (rq_data_ptr->data.data.data_count == 6)
    {
        ToolInfo info;
        info.id = rq_data_ptr->data.id;
        info.name = rq_data_ptr->data.name;
        info.comment = rq_data_ptr->data.comment;
        info.is_valid = false;  // not used, but initialized
        info.group_id = rq_data_ptr->data.group_id;
        info.data.point_.x_ = rq_data_ptr->data.data.data[0];
        info.data.point_.y_ = rq_data_ptr->data.data.data[1];
        info.data.point_.z_ = rq_data_ptr->data.data.data[2];
        info.data.euler_.a_ = rq_data_ptr->data.data.data[3];
        info.data.euler_.b_ = rq_data_ptr->data.data.data[4];
        info.data.euler_.c_ = rq_data_ptr->data.data.data[5];
        rs_data_ptr->data.data = tool_manager_ptr_->addTool(info);
    }
    else
    {
        rs_data_ptr->data.data = TOOL_MANAGER_INVALID_ARG;
    }

    recordLog(TOOL_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/tool_manager/addTool"));
}

// "/rpc/tool_manager/deleteTool"
void ControllerRpc::handleRpc0x00010E4C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = tool_manager_ptr_->deleteTool(rq_data_ptr->data.data);
    recordLog(TOOL_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/tool_manager/deleteTool"));
}

// "/rpc/tool_manager/updateTool"
void ControllerRpc::handleRpc0x0000C78C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ToolInfo* rq_data_ptr = static_cast<RequestMessageType_ToolInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/tool_manager/updateTool can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
        return;
    }

    if (rq_data_ptr->data.data.data_count == 6)
    {
        ToolInfo info;
        info.id = rq_data_ptr->data.id;
        info.name = rq_data_ptr->data.name;
        info.comment = rq_data_ptr->data.comment;
        info.is_valid = false;  // not used, but initialized
        info.group_id = rq_data_ptr->data.group_id;
        info.data.point_.x_ = rq_data_ptr->data.data.data[0];
        info.data.point_.y_ = rq_data_ptr->data.data.data[1];
        info.data.point_.z_ = rq_data_ptr->data.data.data[2];
        info.data.euler_.a_ = rq_data_ptr->data.data.data[3];
        info.data.euler_.b_ = rq_data_ptr->data.data.data[4];
        info.data.euler_.c_ = rq_data_ptr->data.data.data[5];
        rs_data_ptr->data.data = tool_manager_ptr_->updateTool(info);

        int current_id = 0;
        motion_control_ptr_->getToolFrame(current_id);
        if (current_id == rq_data_ptr->data.id && rs_data_ptr->data.data == SUCCESS)
        {
            rs_data_ptr->data.data = motion_control_ptr_->setToolFrame(current_id);
        }
        
    }
    else
    {
        rs_data_ptr->data.data = TOOL_MANAGER_INVALID_ARG;
    }

    recordLog(TOOL_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/tool_manager/updateTool"));
}

// "/rpc/tool_manager/moveTool"
void ControllerRpc::handleRpc0x000085FC(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = tool_manager_ptr_->moveTool(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = TOOL_MANAGER_INVALID_ARG;
    }
    recordLog(TOOL_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/tool_manager/moveTool"));
}

// "/rpc/tool_manager/getToolInfoById"
void ControllerRpc::handleRpc0x00009E34(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_ToolInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ToolInfo*>(response_data_ptr);

    ToolInfo info;
    rs_data_ptr->error_code.data = tool_manager_ptr_->getToolInfoById(rq_data_ptr->data.data, info);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.id = info.id;
        strncpy(rs_data_ptr->data.name, info.name.c_str(), 31);
        rs_data_ptr->data.name[31] = 0;
        strncpy(rs_data_ptr->data.comment, info.comment.c_str(), 255);
        rs_data_ptr->data.comment[255] = 0;
        rs_data_ptr->data.group_id = info.group_id;
        rs_data_ptr->data.data.data_count = 6;
        rs_data_ptr->data.data.data[0] = info.data.point_.x_;
        rs_data_ptr->data.data.data[1] = info.data.point_.y_;
        rs_data_ptr->data.data.data[2] = info.data.point_.z_;
        rs_data_ptr->data.data.data[3] = info.data.euler_.a_;
        rs_data_ptr->data.data.data[4] = info.data.euler_.b_;
        rs_data_ptr->data.data.data[5] = info.data.euler_.c_;
    }
    recordLog(TOOL_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/tool_manager/getToolInfoById"));
}

// "/rpc/tool_manager/getAllValidToolSummaryInfo"
void ControllerRpc::handleRpc0x0001104F(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_ToolSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ToolSummaryList*>(response_data_ptr);

    std::vector<ToolSummaryInfo> info_list;
    info_list = tool_manager_ptr_->getAllValidToolSummaryInfo();
    for(unsigned int i = 0; i < info_list.size(); ++i)
    {
        rs_data_ptr->data.tool_summary_info[i].id = info_list[i].id;
        strncpy(rs_data_ptr->data.tool_summary_info[i].name, info_list[i].name.c_str(), 31);
        rs_data_ptr->data.tool_summary_info[i].name[31] = 0;
        strncpy(rs_data_ptr->data.tool_summary_info[i].comment, info_list[i].comment.c_str(), 255);
        rs_data_ptr->data.tool_summary_info[i].comment[255] = 0;
        rs_data_ptr->data.tool_summary_info[i].group_id = info_list[i].group_id;
    }
    rs_data_ptr->data.tool_summary_info_count = info_list.size();
    rs_data_ptr->error_code.data = SUCCESS;
    recordLog(TOOL_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/tool_manager/getAllValidToolSummaryInfo"));
}

