#include "controller_rpc.h"

using namespace fst_ctrl;


// "/rpc/coordinate_manager/addUserCoord"
void ControllerRpc::handleRpc0x00016764(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_UserInfo* rq_data_ptr = static_cast<RequestMessageType_UserInfo*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    CoordInfo info;
    info.id = rq_data_ptr->data.id;
    info.name = rq_data_ptr->data.name;
    info.comment = rq_data_ptr->data.comment;
    info.is_valid = false;  // not used, but initialized
    info.group_id = rq_data_ptr->data.group_id;
    info.data.position.x = rq_data_ptr->data.data.x;
    info.data.position.y = rq_data_ptr->data.data.y;
    info.data.position.z = rq_data_ptr->data.data.z;
    info.data.orientation.a = rq_data_ptr->data.data.a;
    info.data.orientation.b = rq_data_ptr->data.data.b;
    info.data.orientation.c = rq_data_ptr->data.data.c;
    rs_data_ptr->data.data = coordinate_manager_ptr_->addCoord(info);
}

// "/rpc/coordinate_manager/deleteUserCoord"
void ControllerRpc::handleRpc0x0000BAF4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = coordinate_manager_ptr_->deleteCoord(rq_data_ptr->data.data);
}

// "/rpc/coordinate_manager/updateUserCoord"
void ControllerRpc::handleRpc0x0000EC14(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_UserInfo* rq_data_ptr = static_cast<RequestMessageType_UserInfo*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    CoordInfo info;
    info.id = rq_data_ptr->data.id;
    info.name = rq_data_ptr->data.name;
    info.comment = rq_data_ptr->data.comment;
    info.is_valid = false;  // not used, but initialized
    info.group_id = rq_data_ptr->data.group_id;
    info.data.position.x = rq_data_ptr->data.data.x;
    info.data.position.y = rq_data_ptr->data.data.y;
    info.data.position.z = rq_data_ptr->data.data.z;
    info.data.orientation.a = rq_data_ptr->data.data.a;
    info.data.orientation.b = rq_data_ptr->data.data.b;
    info.data.orientation.c = rq_data_ptr->data.data.c;
    rs_data_ptr->data.data = coordinate_manager_ptr_->updateCoord(info);
}

// "/rpc/coordinate_manager/moveUserCoord"
void ControllerRpc::handleRpc0x0000E104(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = coordinate_manager_ptr_->moveCoord(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
}

// "/rpc/coordinate_manager/getUserCoordInfoById"
void ControllerRpc::handleRpc0x00004324(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool_UserInfo* rs_data_ptr = static_cast<ResponseMessageType_Bool_UserInfo*>(response_data_ptr);

    CoordInfo info;
    rs_data_ptr->success.data = coordinate_manager_ptr_->getCoordInfoById(rq_data_ptr->data.data, info);
    if(rs_data_ptr->success.data)
    {
        rs_data_ptr->data.id = info.id;
        strncpy(rs_data_ptr->data.name, info.name.c_str(), 31);
        rs_data_ptr->data.name[31] = 0;
        strncpy(rs_data_ptr->data.comment, info.comment.c_str(), 255);
        rs_data_ptr->data.comment[255] = 0;
        rs_data_ptr->data.group_id = info.group_id;
        rs_data_ptr->data.data.x = info.data.position.x;
        rs_data_ptr->data.data.y = info.data.position.y;
        rs_data_ptr->data.data.z = info.data.position.z;
        rs_data_ptr->data.data.a = info.data.orientation.a;
        rs_data_ptr->data.data.b = info.data.orientation.b;
        rs_data_ptr->data.data.c = info.data.orientation.c;
    }
}

// "/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo"
void ControllerRpc::handleRpc0x0001838F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_UserSummaryList* rs_data_ptr = static_cast<ResponseMessageType_UserSummaryList*>(response_data_ptr);

    std::vector<CoordSummaryInfo> info_list;
    info_list = coordinate_manager_ptr_->getAllValidCoordSummaryInfo();
    for(unsigned int i = 0; i < info_list.size(); ++i)
    {
        rs_data_ptr->data.user_summary_info[i].id = info_list[i].id;
        strncpy(rs_data_ptr->data.user_summary_info[i].name, info_list[i].name.c_str(), 31);
        rs_data_ptr->data.user_summary_info[i].name[31] = 0;
        strncpy(rs_data_ptr->data.user_summary_info[i].comment, info_list[i].comment.c_str(), 255);
        rs_data_ptr->data.user_summary_info[i].comment[255] = 0;
        rs_data_ptr->data.user_summary_info[i].group_id = info_list[i].group_id;
    }
    rs_data_ptr->data.user_summary_info_count = info_list.size();
}

