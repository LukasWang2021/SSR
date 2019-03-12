#include "controller_rpc.h"

using namespace fst_ctrl;

//"/rpc/param_manager/getParamInfoList"	
void ControllerRpc::handleRpc0x0000F0B4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_ParamInfoList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ParamInfoList*>(response_data_ptr);
/*
    std::vector list = param_manager_ptr_->getParamInfoList(rq_data_ptr.data.data);
    rs_data_ptr->data.param_info_count = list.size();
    for(int i = 0; rs_data_ptr->data.param_info_count; ++i)
    {
        strcpy(rs_data_ptr->data.param_info[i].name, list.name.c_str());
        rs_data_ptr->data.param_info[i].type = list.type;
        memcpy(&rs_data_ptr->data.param_info[i].bytes, ptr, length);
    }

    rs_data_ptr->error_code.data = SUCCESS;
*/
    FST_INFO("/rpc/motion_control/axis_group/doManualStop runs");
}
//"/rpc/param_manager/setParamInfo"	
void ControllerRpc::handleRpc0x0001393F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_ParamInfo* rq_data_ptr = static_cast<RequestMessageType_Int32_ParamInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    
    FST_INFO("rpc-setParamInfo");
}

