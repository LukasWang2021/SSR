#include "controller_rpc.h"

using namespace fst_ctrl;
using namespace user_space;
using namespace log_space;
using namespace group_space;

// "/rpc/coordinate_manager/addUserCoord"
void ControllerRpc::handleRpc0x00016764(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_UserCoordInfo* rq_data_ptr = static_cast<RequestMessageType_UserCoordInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.group_id;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/coordinate_manager/addUserCoord input invalid params group_id = %d", group_id);
        return;
    }

    GroupStatus_e status = GROUP_STATUS_UNKNOWN;
    bool in_position = false;
    group_ptr_[group_id]->mcGroupReadStatus(status, in_position);
    if (status == GROUP_STATUS_ERROR_STOP || status == GROUP_STATUS_DISABLED)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        LogProducer::error("rpc", "/rpc/coordinate_manager/addUserCoord can't run under status(%d), ret = %llx\n", status, rs_data_ptr->data.data);
        return;
    }

    if (rq_data_ptr->data.data.data_count == 6)
    {
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
    }
    else
    {
        rs_data_ptr->data.data = COORDINATE_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/coordinate_manager/addUserCoord for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/coordinate_manager/addUserCoord for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/coordinate_manager/deleteUserCoord"
void ControllerRpc::handleRpc0x0000BAF4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = coordinate_manager_ptr_->deleteCoord(rq_data_ptr->data.data);

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/coordinate_manager/deleteUserCoord success");
    else
        LogProducer::error("rpc", "/rpc/coordinate_manager/deleteUserCoord failed. Error = 0x%llx", rs_data_ptr->data.data);

}

// "/rpc/coordinate_manager/updateUserCoord"
void ControllerRpc::handleRpc0x0000EC14(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_UserCoordInfo* rq_data_ptr = static_cast<RequestMessageType_UserCoordInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.group_id;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/coordinate_manager/addUserCoord input invalid params group_id = %d", group_id);
        return;
    }

    GroupStatus_e status = GROUP_STATUS_UNKNOWN;
    bool in_position = false;
    group_ptr_[group_id]->mcGroupReadStatus(status, in_position);
    if (status == GROUP_STATUS_ERROR_STOP || status == GROUP_STATUS_DISABLED)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        LogProducer::error("rpc", "/rpc/coordinate_manager/addUserCoord can't run under status(%d), ret = %llx\n", status, rs_data_ptr->data.data);
        return;
    }

    if (rq_data_ptr->data.data.data_count == 6)
    {
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
        group_ptr_[group_id]->getUserFrame(current_id);
        if (current_id == rq_data_ptr->data.id && rs_data_ptr->data.data == SUCCESS)
        {
            rs_data_ptr->data.data = group_ptr_[group_id]->setUserFrame(current_id);
        }

    }
    else
    {
        rs_data_ptr->data.data = COORDINATE_MANAGER_INVALID_ARG;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/coordinate_manager/updateUserCoord success");
    else
        LogProducer::error("rpc", "/rpc/coordinate_manager/updateUserCoord failed. Error = 0x%llx", rs_data_ptr->data.data); 

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

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/coordinate_manager/moveUserCoord success");
    else
        LogProducer::error("rpc", "/rpc/coordinate_manager/moveUserCoord failed. Error = 0x%llx", rs_data_ptr->data.data); 

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
        rs_data_ptr->data.data.data_count = 6;
        rs_data_ptr->data.data.data[0] = info.data.point_.x_;
        rs_data_ptr->data.data.data[1] = info.data.point_.y_;
        rs_data_ptr->data.data.data[2] = info.data.point_.z_;
        rs_data_ptr->data.data.data[3] = info.data.euler_.a_;
        rs_data_ptr->data.data.data[4] = info.data.euler_.b_;
        rs_data_ptr->data.data.data[5] = info.data.euler_.c_;
    }

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/coordinate_manager/getUserCoordInfoById id = %d success", info.id);
    else
        LogProducer::error("rpc", "/rpc/coordinate_manager/getUserCoordInfoById id = %d failed. Error = 0x%llx", info.id, rs_data_ptr->error_code.data); 

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
    LogProducer::info("rpc", "/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo success");
}

