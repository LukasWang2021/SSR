#include "controller_rpc.h"
#include "error_code.h"

using namespace fst_ctrl;


// "/rpc/coordinate_manager/addUserCoord"
void ControllerRpc::handleRpc0x00016764(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_UserCoordInfo* rq_data_ptr = static_cast<RequestMessageType_UserCoordInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/coordinate_manager/addUserCoord can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    CoordInfo info;
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
    rs_data_ptr->data.data = coordinate_manager_ptr_->addCoord(info);
    recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/coordinate_manager/addUserCoord"));
}

// "/rpc/coordinate_manager/deleteUserCoord"
void ControllerRpc::handleRpc0x0000BAF4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = coordinate_manager_ptr_->deleteCoord(rq_data_ptr->data.data);
    recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/coordinate_manager/deleteUserCoord"));
}

// "/rpc/coordinate_manager/updateUserCoord"
void ControllerRpc::handleRpc0x0000EC14(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_UserCoordInfo* rq_data_ptr = static_cast<RequestMessageType_UserCoordInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/coordinate_manager/updateUserCoord can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
    }

    CoordInfo info;
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
    rs_data_ptr->data.data = coordinate_manager_ptr_->updateCoord(info);

    int current_id = 0;
    motion_control_ptr_->getUserFrame(current_id);
    if (current_id == rq_data_ptr->data.id && rs_data_ptr->data.data == SUCCESS)
    {
        rs_data_ptr->data.data = motion_control_ptr_->setUserFrame(current_id);
    }

    recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/coordinate_manager/updateUserCoord"));
}

// "/rpc/coordinate_manager/moveUserCoord"
void ControllerRpc::handleRpc0x0000E104(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = coordinate_manager_ptr_->moveCoord(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = COORDINATE_MANAGER_INVALID_ARG;
    }
    recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->data.data, std::string("/rpc/coordinate_manager/moveUserCoord"));
}

// "/rpc/coordinate_manager/getUserCoordInfoById"
void ControllerRpc::handleRpc0x00004324(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_UserCoordInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_UserCoordInfo*>(response_data_ptr);

    CoordInfo info;
    rs_data_ptr->error_code.data = coordinate_manager_ptr_->getCoordInfoById(rq_data_ptr->data.data, info);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.id = info.id;
        strncpy(rs_data_ptr->data.name, info.name.c_str(), 31);
        rs_data_ptr->data.name[31] = 0;
        strncpy(rs_data_ptr->data.comment, info.comment.c_str(), 255);
        rs_data_ptr->data.comment[255] = 0;
        rs_data_ptr->data.group_id = info.group_id;
        rs_data_ptr->data.data.data_cout = 6;
        rs_data_ptr->data.data.data[0] = info.data.point_.x_;
        rs_data_ptr->data.data.data[1] = info.data.point_.y_;
        rs_data_ptr->data.data.data[2] = info.data.point_.z_;
        rs_data_ptr->data.data.data[3] = info.data.euler_.a_;
        rs_data_ptr->data.data.data[4] = info.data.euler_.b_;
        rs_data_ptr->data.data.data[5] = info.data.euler_.c_;
    }
    recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/coordinate_manager/getUserCoordInfoById"));
}

// "/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo"
void ControllerRpc::handleRpc0x0001838F(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_UserCoordSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_UserCoordSummaryList*>(response_data_ptr);

    std::vector<CoordSummaryInfo> info_list;
    info_list = coordinate_manager_ptr_->getAllValidCoordSummaryInfo();
    for(unsigned int i = 0; i < info_list.size(); ++i)
    {
        rs_data_ptr->data.user_coord_summary[i].id = info_list[i].id;
        strncpy(rs_data_ptr->data.user_coord_summary[i].name, info_list[i].name.c_str(), 31);
        rs_data_ptr->data.user_coord_summary[i].name[31] = 0;
        strncpy(rs_data_ptr->data.user_coord_summary[i].comment, info_list[i].comment.c_str(), 255);
        rs_data_ptr->data.user_coord_summary[i].comment[255] = 0;
        rs_data_ptr->data.user_coord_summary[i].group_id = info_list[i].group_id;
    }
    rs_data_ptr->data.user_coord_summary_count = info_list.size();    
    rs_data_ptr->error_code.data = SUCCESS;
    
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(COORDINATE_MANAGER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo"));
}

