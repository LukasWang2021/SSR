#include "controller_rpc.h"

using namespace fst_ctrl;

//"/rpc/param_manager/getParamInfoList"	
void ControllerRpc::handleRpc0x0000F0B4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_ParamInfoList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ParamInfoList*>(response_data_ptr);

    std::vector<ParamInfo_t> list = param_manager_ptr_->getParamInfoList(rq_data_ptr->data.data);
    rs_data_ptr->data.param_info_count = list.size();
    for(int i = 0; rs_data_ptr->data.param_info_count; ++i)
    {
        strcpy(rs_data_ptr->data.param_info[i].name, list[i].name);
        rs_data_ptr->data.param_info[i].type = list[i].type;
        memcpy(&rs_data_ptr->data.param_info[i].data.bytes, list[i].data, sizeof(list[i].data));
    }
    
    rs_data_ptr->error_code.data = SUCCESS;

    FST_INFO("/rpc/motion_control/axis_group/doManualStop runs");
}
//"/rpc/param_manager/setParamInfo"	
void ControllerRpc::handleRpc0x0001393F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_ParamInfo* rq_data_ptr = static_cast<RequestMessageType_Int32_ParamInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    ParamInfo_t info;
    strcpy(info.name, rq_data_ptr->data2.name);
    info.type = rq_data_ptr->data2.type;
    memcpy(&info.data, rq_data_ptr->data2.data.bytes, sizeof(info.data));

    rs_data_ptr->data.data = param_manager_ptr_->setParamInfo(rq_data_ptr->data1.data, info);
    
    FST_INFO("/rpc/param_manager/setParamInfo runs");
}

