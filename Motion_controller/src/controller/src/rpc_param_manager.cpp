#include "controller_rpc.h"

using namespace fst_ctrl;

//"/rpc/param_manager/getParamInfoList"	
void ControllerRpc::handleRpc0x0000F0B4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_ParamInfoList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ParamInfoList*>(response_data_ptr);

    param_manager_ptr_->printParamInfoList(rq_data_ptr->data.data);

    std::vector<ParamInfo_t> list = param_manager_ptr_->getParamInfoList(rq_data_ptr->data.data);
    rs_data_ptr->data.param_info_count = list.size();
    for(int i = 0; i < rs_data_ptr->data.param_info_count; ++i)
    {
        strcpy(rs_data_ptr->data.param_info[i].name, list[i].name);
        rs_data_ptr->data.param_info[i].type = list[i].type;
        memcpy(&rs_data_ptr->data.param_info[i].data.bytes, list[i].data, sizeof(list[i].data));
        if(list[i].type == PARAM_INFO_INT)
        {
            rs_data_ptr->data.param_info[i].data.size = sizeof(int);
        }
        else if (list[i].type == PARAM_INFO_DOUBLE)
        {
            rs_data_ptr->data.param_info[i].data.size = sizeof(double);
        }
        else if (list[i].type == PARAM_INFO_BOOL)
        {
            rs_data_ptr->data.param_info[i].data.size = sizeof(bool);
        }
        else
        {
            rs_data_ptr->data.param_info[i].data.size = sizeof(int);
        }
        
        /* print for checking
        printf("rpc name = %s, type = %d, ", list[i].name, list[i].type);
        printf("rpc size = %d, data = %02x-%02x-%02x-%02x | %02x-%02x-%02x-%02x\n", rs_data_ptr->data.param_info[i].data.size,
        rs_data_ptr->data.param_info[i].data.bytes[7], rs_data_ptr->data.param_info[i].data.bytes[6],
        rs_data_ptr->data.param_info[i].data.bytes[5],rs_data_ptr->data.param_info[i].data.bytes[4],
        rs_data_ptr->data.param_info[i].data.bytes[3], rs_data_ptr->data.param_info[i].data.bytes[2],
        rs_data_ptr->data.param_info[i].data.bytes[1],rs_data_ptr->data.param_info[i].data.bytes[0]);
        */
    }
    
    rs_data_ptr->error_code.data = SUCCESS;

    FST_INFO("/rpc/param_manager/getParamInfoList runs");
}
//"/rpc/param_manager/setParamInfo"	
void ControllerRpc::handleRpc0x0001393F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_ParamInfo* rq_data_ptr = static_cast<RequestMessageType_Int32_ParamInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    ParamInfo_t info;
    strcpy(info.name, rq_data_ptr->data2.name);
    info.type = rq_data_ptr->data2.type;
    int size = rq_data_ptr->data2.data.size;
    memcpy(&info.data, &rq_data_ptr->data2.data.bytes, size);

    rs_data_ptr->data.data = param_manager_ptr_->setParamInfo(rq_data_ptr->data1.data, info);

    /* print for checking
    printf("rpc set:rq_data=%d name=%s,type=%d, data=%02x-%02x-%02x-%02x |%02x-%02x-%02x-%02x \n", rq_data_ptr->data1.data, info.name, info.type, 
         info.data[7],info.data[6],info.data[5],info.data[4], info.data[3],info.data[2],info.data[1],info.data[0]);
    */
    FST_INFO("/rpc/param_manager/setParamInfo runs");
}

