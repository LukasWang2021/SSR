#include "controller_rpc.h"

using namespace user_space;
using namespace log_space;
using namespace group_space;

//"/rpc/group/mcGroupReset"	
void ControllerRpc::handleRpc0x00016FF4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id < GROUP_NUM && group_id >= 0)
    {
        rs_data_ptr->data.data = group_ptr_[group_id]->mcGroupReset();
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/group/mcGroupReset input invalid params group_id = %d", group_id);
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/group/mcGroupReset for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/group/mcGroupReset for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);
}

//"/rpc/group/mcGroupEnable"	
void ControllerRpc::handleRpc0x00003615(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id < GROUP_NUM && group_id >= 0)
    {
        GroupStatus_e status = GROUP_STATUS_UNKNOWN;
        bool in_pos = false;
        group_ptr_[group_id]->mcGroupReadStatus(status, in_pos);
        //clear trajectory buffer
        if (status == GROUP_STATUS_ERROR_STOP || status == GROUP_STATUS_DISABLED)
        {
            group_ptr_[group_id]->clearGroup();
            group_ptr_[group_id]->clearTeachGroup();
        }
        //check if the zero offset is valid.
        if(group_ptr_[group_id]->getCalibrateState() == MOTION_FORBIDDEN)
        {			
            group_ptr_[group_id]->mcGroupDisable();
            InterpCtrl::instance().abort();
            group_ptr_[group_id]->stopGroup();
            group_ptr_[group_id]->clearGroup();
            group_ptr_[group_id]->clearTeachGroup();
            rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
            LogProducer::error("rpc","/rpc/group/mcGroupEnable check offset failure");
            return;
        }
        group_ptr_[group_id]->resetGroup();
        rs_data_ptr->data.data = group_ptr_[group_id]->mcGroupEnable();
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/group/mcGroupEnable input invalid params group_id = %d", group_id);
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/group/mcGroupEnable for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/group/mcGroupEnable for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);
}

//"/rpc/group/mcGroupDisable"	
void ControllerRpc::handleRpc0x0000D185(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id < GROUP_NUM && group_id >= 0)
    {
        rs_data_ptr->data.data = group_ptr_[group_id]->mcGroupDisable();
        InterpCtrl::instance().abort();
        group_ptr_[group_id]->stopGroup();
        group_ptr_[group_id]->clearGroup();
        group_ptr_[group_id]->clearTeachGroup();
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/group/mcGroupDisable input invalid params group_id = %d", group_id);
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/group/mcGroupDisable for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/group/mcGroupDisable for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);
}

//"/rpc/group/mcGroupReadError"	
void ControllerRpc::handleRpc0x00004BE2(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    rs_data_ptr->error_code.data = SUCCESS;
    if(group_id < GROUP_NUM && group_id >= 0)
    {
        rs_data_ptr->data.data = group_ptr_[group_id]->mcGroupReadError();
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/group/mcGroupReadError input invalid params group_id = %d", group_id);
    }
    
    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/group/mcGroupReadError for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/group/mcGroupReadError for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->error_code.data);
}

//"/rpc/group/mcGroupReadStatus"	
void ControllerRpc::handleRpc0x00002A83(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_GroupStatus_Bool* rs_data_ptr = static_cast<ResponseMessageType_Uint64_GroupStatus_Bool*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    GroupStatus_e status = GROUP_STATUS_UNKNOWN;
    bool in_position = false;
    if(group_id < GROUP_NUM && group_id >= 0)
    {
        rs_data_ptr->error_code.data = group_ptr_[group_id]->mcGroupReadStatus(status, in_position);
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/group/mcGroupReadStatus input invalid params group_id = %d", group_id);
    }

    switch (status)
    {
        case GROUP_STATUS_ERROR_STOP:
            rs_data_ptr->data1 = MessageType_GroupStatus_ERROR_STOP;
            break;
        case GROUP_STATUS_DISABLED:
            rs_data_ptr->data1 = MessageType_GroupStatus_DISABLED;
            break;
        case GROUP_STATUS_STANDBY:
            rs_data_ptr->data1 = MessageType_GroupStatus_STANDBY;
            break;
        case GROUP_STATUS_STOPPING:
            rs_data_ptr->data1 = MessageType_GroupStatus_STOPPING;
            break;
        case GROUP_STATUS_HOMING:
            rs_data_ptr->data1 = MessageType_GroupStatus_HOMING;
            break;
        case GROUP_STATUS_MOVING:
            rs_data_ptr->data1 = MessageType_GroupStatus_MOVING;
            break;
        default:
            rs_data_ptr->data1 = MessageType_GroupStatus_UNKNOWN;
            break;
    }
    rs_data_ptr->data2.data = in_position;
    
    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/group/mcGroupReadStatus for group[%d] success, status = %d, in_position = %d", group_id, status, in_position);
    else
        LogProducer::error("rpc", "/rpc/group/mcGroupReadStatus for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->error_code.data);
}

//"/rpc/group/resetAllEncoder"	
void ControllerRpc::handleRpc0x000019D2(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    int32_t group_id = rq_data_ptr->data.data;
    if(group_id < GROUP_NUM && group_id >= 0)
    {
        rs_data_ptr->data.data = group_ptr_[group_id]->resetEncoderMultiTurnValue();
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/group/resetAllEncoder input invalid params group_id = %d", group_id);
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/group/resetAllEncoder for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/group/resetAllEncoder for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);
}