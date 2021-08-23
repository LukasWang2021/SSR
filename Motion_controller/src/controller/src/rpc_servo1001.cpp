#include "controller_rpc.h"

using namespace user_space;
using namespace log_space;
using namespace servo_comm_space;

//"/rpc/servo1001/servo/shutDown"	
void ControllerRpc::handleRpc0x0000863E(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/shutDown input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/shutDown input invalid params for axis[%d]", axis_id);        
        return;
    }
        
    if(servo_comm_ptr_[axis_id]->emitServoCmdShutDown())
    {
        rs_data_ptr->data.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/servo1001/servo/shutDown for axis(%d) success", axis_id);
    }
    else
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/servo/shutDown for axis(%d) failed", axis_id);
    }
}

//"/rpc/servo1001/servo/switchOn"	
void ControllerRpc::handleRpc0x0000E5CE(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/switchOn input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/switchOn input invalid params for axis[%d]", axis_id);        
        return;
    }
        
    if(servo_comm_ptr_[axis_id]->emitServoCmdSwitchOn())
    {
        rs_data_ptr->data.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/servo1001/servo/switchOn for axis(%d) success", axis_id);
    }
    else
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/servo/switchOn for axis(%d) failed", axis_id);
    }
}

//"/rpc/servo1001/servo/disableVoltage"	
void ControllerRpc::handleRpc0x00004755(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/disableVoltage input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/disableVoltage input invalid params for axis[%d]", axis_id);        
        return;
    }
     
    if(servo_comm_ptr_[axis_id]->emitServoCmdDisableVoltage())
    {
        rs_data_ptr->data.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/servo1001/servo/disableVoltage for axis(%d) success", axis_id);
    }
    else
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/servo/disableVoltage for axis(%d) failed", axis_id);
    }
}
//"/rpc/servo1001/servo/enableOperation"	
void ControllerRpc::handleRpc0x0000313E(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/enableOperation input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];   
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/enableOperation input invalid params for axis[%d]", axis_id);        
        return;
    }
    
    if(servo_comm_ptr_[axis_id]->emitServoCmdEnableOperation())
    {
        rs_data_ptr->data.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/servo1001/servo/enableOperation for axis(%d) success", axis_id);
    }
    else
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/servo/enableOperation for axis(%d) failed", axis_id);
    }
}

//"/rpc/servo1001/servo/switchOnAndEnableOperation"	
void ControllerRpc::handleRpc0x000177CE(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/switchOnAndEnableOperation input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/switchOnAndEnableOperation input invalid params for axis[%d]", axis_id);        
        return;
    }

    if(servo_comm_ptr_[axis_id]->emitServoCmdSwitchOnAndEnableOperation())
    {
        rs_data_ptr->data.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/servo1001/servo/switchOnAndEnableOperation for axis(%d) success", axis_id);
    }
    else
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/servo/switchOnAndEnableOperation for axis(%d) failed", axis_id);
    }
}

//"/rpc/servo1001/servo/disableOperation"	
void ControllerRpc::handleRpc0x000026AE(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/disableOperation input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/disableOperation input invalid params for axis[%d]", axis_id);        
        return;
    }
    
    if(servo_comm_ptr_[axis_id]->emitServoCmdDisableOperation())
    {
        rs_data_ptr->data.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/servo1001/servo/disableOperation for axis(%d) success", axis_id);
    }
    else
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/servo/disableOperation for axis(%d) failed", axis_id);
    }
}

//"/rpc/servo1001/servo/quickStop"	
void ControllerRpc::handleRpc0x00000580(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/quickStop input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/quickStop input invalid params for axis[%d]", axis_id);        
        return;
    }
    
    if(servo_comm_ptr_[axis_id]->emitServoCmdQuickStop())
    {
        rs_data_ptr->data.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/servo1001/servo/quickStop for axis(%d) success", axis_id);
    }
    else
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/servo/quickStop for axis(%d) failed", axis_id);
    }
}

//"/rpc/servo1001/servo/resetFault"	
void ControllerRpc::handleRpc0x00010584(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/resetFault input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/resetFault input invalid params for axis[%d]", axis_id);        
        return;
    }
    
    if(servo_comm_ptr_[axis_id]->emitServoCmdFaultReset())
    {
        usleep(100*1000);//high signal last 100ms.

        if(servo_comm_ptr_[axis_id]->emitServoCmdResetFaultReset())
        {
            rs_data_ptr->data.data = SUCCESS;
            LogProducer::info("rpc", "/rpc/servo1001/servo/resetFault for axis(%d) success", axis_id);
        }
        else
        {
            rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
            LogProducer::error("rpc", "/rpc/servo1001/servo/resetFault reset signal for axis(%d) failed", axis_id);
        }
    }
    else
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/servo/resetFault for axis(%d) failed", axis_id);
    }
}

//"/rpc/servo1001/servo/transCommState"	
void ControllerRpc::handleRpc0x000153C5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_CoreCommState* rq_data_ptr = static_cast<RequestMessageType_Int32List_CoreCommState*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data1.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/transCommState input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data1.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data1.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/transCommState input invalid params for axis[%d]", axis_id);        
        return;
    }
    
    CoreCommState_e state = CORE_COMM_STATE_INIT;
    switch (rq_data_ptr->data2)
    {
        case MessageType_CoreCommState_INIT:
            state = CORE_COMM_STATE_INIT;
            break;
        case MessageType_CoreCommState_PREOP:
            state = CORE_COMM_STATE_PREOP;
            break;
        case MessageType_CoreCommState_SAFEOP:
            state = CORE_COMM_STATE_SAFEOP;
            break;
        case MessageType_CoreCommState_OP:
            state = CORE_COMM_STATE_OP;
            break;
        case MessageType_CoreCommState_UNKNOWN:
            state = CORE_COMM_STATE_UNKNOWN;
            break;
        default:
            state = CORE_COMM_STATE_UNKNOWN;
            break;
    }
    rs_data_ptr->data.data = servo_comm_ptr_[axis_id]->doServoCmdTransCommState(state);
    if (rs_data_ptr->data.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/transCommState for axis(%d) to %d success", axis_id, state);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/transCommState for axis(%d) failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);
    }

    
}

//"/rpc/servo1001/servo/readParameter"	
void ControllerRpc::handleRpc0x00006892(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 3)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/readParameter input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/readParameter input invalid params for axis[%d]", axis_id);        
        return;
    }    
    int32_t param_index = rq_data_ptr->data.data[2];
    rs_data_ptr->error_code.data = servo_comm_ptr_[axis_id]->doServoCmdReadParameter(param_index, &rs_data_ptr->data.data);
    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/readParameter for axis(%d) success, index:%d, value:%d", 
            axis_id, param_index, rs_data_ptr->data.data);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/readParameter for axis(%d) failed. Error = 0x%llx", axis_id, rs_data_ptr->error_code.data);
    }
}

//"/rpc/servo1001/servo/writeParameter"	
void ControllerRpc::handleRpc0x00007C32(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 4)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/writeParameter input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/writeParameter input invalid params for axis[%d]", axis_id);        
        return;
    }    
    int32_t param_index = rq_data_ptr->data.data[2];
    int32_t param_value = rq_data_ptr->data.data[3];
    rs_data_ptr->data.data = servo_comm_ptr_[axis_id]->doServoCmdWriteParameter(param_index, param_value);
    if (rs_data_ptr->data.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/writeParameter for axis(%d) success, index:%d, value:%d", 
            axis_id, param_index, param_value);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/writeParameter for axis(%d) failed. Error = 0x%llx", 
            axis_id, rs_data_ptr->data.data);
    }
}

//"/rpc/servo1001/servo/moveVelocity"	
void ControllerRpc::handleRpc0x000164D9(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 7)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/moveVelocity input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/moveVelocity input invalid params for axis[%d]", axis_id);        
        return;
    }    
    int32_t velocity = rq_data_ptr->data.data[2];
    int32_t acc = rq_data_ptr->data.data[3];
    int32_t dec = rq_data_ptr->data.data[4];
    int32_t jerk = rq_data_ptr->data.data[5];
    int32_t direction = rq_data_ptr->data.data[6];
    rs_data_ptr->data.data = servo_comm_ptr_[axis_id]->doServoCmdMoveVelocity(velocity, acc, dec, jerk, direction);
    if (rs_data_ptr->data.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/moveVelocity for axis(%d) success", axis_id);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/moveVelocity for axis(%d) failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);
    }
}

//"/rpc/servo1001/servo/moveAbsolute"	
void ControllerRpc::handleRpc0x00004DD5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_Int64* rq_data_ptr = static_cast<RequestMessageType_Int32List_Int64*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data1.data_count != 6)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/moveAbsolute input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data1.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data1.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/moveAbsolute input invalid params for axis[%d]", axis_id);        
        return;
    }
    int32_t velocity = rq_data_ptr->data1.data[2];
    int32_t acc = rq_data_ptr->data1.data[3];
    int32_t dec = rq_data_ptr->data1.data[4];
    int32_t jerk = rq_data_ptr->data1.data[5];
    int64_t position = rq_data_ptr->data2.data;
    rs_data_ptr->data.data = servo_comm_ptr_[axis_id]->doServoCmdMoveAbsolute(position, velocity, acc, dec, jerk);
    if (rs_data_ptr->data.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/moveAbsolute for axis(%d) success", axis_id);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/moveAbsolute for axis(%d) failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);
    }
}

//"/rpc/servo1001/servo/triggerUploadParameters"	
void ControllerRpc::handleRpc0x000020B3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/triggerUploadParameters input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/triggerUploadParameters input invalid params for axis[%d]", axis_id);        
        return;
    }
    rs_data_ptr->data.data = servo_comm_ptr_[axis_id]->triggerServoCmdUploadParameters(&sync_ack_ptr_);
    if (rs_data_ptr->data.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/triggerUploadParameters for axis(%d) success", axis_id);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/triggerUploadParameters for axis(%d) failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);
    }
}

//"/rpc/servo1001/servo/uploadParameters"	
void ControllerRpc::handleRpc0x0000E003(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/uploadParameters input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/uploadParameters input invalid params for axis[%d]", axis_id);        
        return;
    }
    BufferAppData2002_t params;
    
    if (servo_comm_ptr_[axis_id]->uploadServoParameters(&params))
    {
        rs_data_ptr->data.data_count = 512;
        for (size_t i = 0; i < rs_data_ptr->data.data_count; ++i)
        {
            rs_data_ptr->data.data[i] = params.param[i];
        }
        rs_data_ptr->error_code.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/servo1001/servo/uploadParameters for axis(%d) success", axis_id);
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/servo/uploadParameters for axis(%d) failed", axis_id);
    }
}

//"/rpc/servo1001/servo/triggerDownloadParameters"	
void ControllerRpc::handleRpc0x00011C53(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/triggerDownloadParameters input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/triggerDownloadParameters input invalid params for axis[%d]", axis_id);        
        return;
    }
    rs_data_ptr->data.data = servo_comm_ptr_[axis_id]->triggerServoCmdDownloadParameters(&sync_ack_ptr_);
    if (rs_data_ptr->data.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/triggerDownloadParameters for axis(%d) success", axis_id);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/triggerDownloadParameters for axis(%d) failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);
    }
}

//"/rpc/servo1001/servo/downloadParameters"	
void ControllerRpc::handleRpc0x00017063(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data1.data_count != 2 || rq_data_ptr->data2.data_count != 512)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/downloadParameters input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data1.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data1.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/downloadParamters input invalid params for axis[%d]", axis_id);        
        return;
    }
    BufferAppData2001_t params;
    for (size_t i = 0; i < rq_data_ptr->data2.data_count; ++i)
    {
        params.param[i] = rq_data_ptr->data2.data[i];
    }
    if (servo_comm_ptr_[axis_id]->downloadServoParameters(&params))
    {
        rs_data_ptr->data.data = SUCCESS;
        //LogProducer::info("rpc", "/rpc/servo1001/servo/downloadParameters for axis(%d) success", axis_id);
    }
    else
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/servo/downloadParameters for axis(%d) failed", axis_id);
        return;
    }
    //save servo parameters to files
    for (size_t i = 0; i < rq_data_ptr->data2.data_count; ++i)
    {
        axis_model_ptr_[axis_id]->actuator.servo_ptr->set(i, params.param[i]);
    }
    LogProducer::info("rpc", "/rpc/servo1001/servo/downloadParameters to save servo parameters for axis(%d) ", axis_id);
    if (!axis_model_ptr_[axis_id]->actuator.servo_ptr->save())
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/downloadParameters to save servo parameters for axis(%d) failed", axis_id);
    }
}

//"/rpc/servo1001/servo/isAsyncServiceFinish"	
void ControllerRpc::handleRpc0x000043B8(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_Bool* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Bool*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/isAsyncServiceFinish input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/isAsyncServiceFinish input invalid params for axis[%d]", axis_id);        
        return;
    }

    if (sync_ack_ptr_ == NULL)
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/servo/isAsyncServiceFinish should be called after trigger_rpc");        
        return;
    }
    
    rs_data_ptr->data.data = servo_comm_ptr_[axis_id]->isServoAsyncServiceFinish(sync_ack_ptr_);
    
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/servo/isAsyncServiceFinish for axis(%d) success, isFinish:%d", 
        axis_id, rs_data_ptr->data.data);
}

//"/rpc/servo1001/servo/getCommState"	
void ControllerRpc::handleRpc0x0000F485(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_CoreCommState* rs_data_ptr = static_cast<ResponseMessageType_Uint64_CoreCommState*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/getCommState input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/getCommState input invalid params for axis[%d]", axis_id);        
        return;
    }
    CoreCommState_e state = servo_comm_ptr_[axis_id]->getCommState();
    switch (state)
    {
        case CORE_COMM_STATE_INIT:
            rs_data_ptr->data = MessageType_CoreCommState_INIT;
            break;
        case CORE_COMM_STATE_PREOP:
            rs_data_ptr->data = MessageType_CoreCommState_PREOP;
            break;
        case CORE_COMM_STATE_SAFEOP:
            rs_data_ptr->data = MessageType_CoreCommState_SAFEOP;
            break;
        case CORE_COMM_STATE_OP:
            rs_data_ptr->data = MessageType_CoreCommState_OP;
            break;
        case CORE_COMM_STATE_UNKNOWN:
            rs_data_ptr->data = MessageType_CoreCommState_UNKNOWN;
            break;
        default:
            rs_data_ptr->data = MessageType_CoreCommState_UNKNOWN;
            break;
    }

    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/servo/getCommState for axis(%d) success, state:%d", 
        axis_id, state);
}

//"/rpc/servo1001/servo/getServoState" 
void ControllerRpc::handleRpc0x000032F5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/getServoState input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/getServoState input invalid params for axis[%d]", axis_id);        
        return;
    }

    CircleBufferAppData3000_t fdb_pdo;
    uint32_t current_time_stamp = 0;
    servo_comm_ptr_[axis_id]->processFdbPdoCurrent((uint8_t*)&fdb_pdo, &current_time_stamp);
	ServoSm_e servo_state = servo_comm_ptr_[axis_id]->getServoState(fdb_pdo.state_word);

    rs_data_ptr->data.data = servo_state;
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/servo/getServoState for axis(%d) success, state:%d", 
        axis_id, servo_state);

}

//"/rpc/servo1001/servo/moveRelative"  
void ControllerRpc::handleRpc0x000172C5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_Int64* rq_data_ptr = static_cast<RequestMessageType_Int32List_Int64*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data1.data_count != 6)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/moveRelative input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data1.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data1.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/moveRelative input invalid params for axis[%d]", axis_id);        
        return;
    }
    int32_t velocity = rq_data_ptr->data1.data[2];
    int32_t acc = rq_data_ptr->data1.data[3];
    int32_t dec = rq_data_ptr->data1.data[4];
    int32_t jerk = rq_data_ptr->data1.data[5];
    int64_t position = rq_data_ptr->data2.data;
    rs_data_ptr->data.data = servo_comm_ptr_[axis_id]->doServoCmdMoveRelative(position, velocity, acc, dec, jerk);
    if (rs_data_ptr->data.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/moveRelative for axis(%d) success", axis_id);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/moveRelative for axis(%d) failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);
    }
}

//"/rpc/servo1001/servo/resetEncoder"	
void ControllerRpc::handleRpc0x0000EFE2(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/resetEncoder input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/resetEncoder input invalid params for axis[%d]", axis_id);        
        return;
    }
        
    rs_data_ptr->data.data = servo_comm_ptr_[axis_id]->doServoCmdResetEncoder();   
    if(rs_data_ptr->data.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/resetEncoder for axis(%d) success", axis_id);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/resetEncoder for axis(%d) failed: 0x%llx", axis_id, rs_data_ptr->data.data);
    }
}

//"/rpc/servo1001/servo/goHome"  
void ControllerRpc::handleRpc0x00013BB5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/goHome input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/goHome input invalid params for axis[%d]", axis_id);        
        return;
    }

    rs_data_ptr->data.data = servo_comm_ptr_[axis_id]->doServoCmdHoming();   
    if(rs_data_ptr->data.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/goHome for axis(%d) success", axis_id);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/goHome for axis(%d) failed: 0x%llx", axis_id, rs_data_ptr->data.data);
    }
}

//"/rpc/servo1001/servo/abortHoming"  
void ControllerRpc::handleRpc0x00015AB7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/abortHoming input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/abortHoming input invalid params for axis[%d]", axis_id);        
        return;
    }

    rs_data_ptr->data.data = servo_comm_ptr_[axis_id]->doServoCmdAbortHoming();
    if(rs_data_ptr->data.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/abortHoming for axis(%d) success", axis_id);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/abortHoming for axis(%d) failed", axis_id);
    }
}

//"/rpc/servo1001/servo/getServoDefinedInfo"	
void ControllerRpc::handleRpc0x0000C87F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    if (rq_data_ptr->data2.data_count != 9)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/getServoDefinedInfo input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data1.data;//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data2.data[0];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/getServoDefinedInfo input invalid params for axis[%d]", axis_id);        
        return;
    }
    
    int32_t req[8] = {0};
    int32_t res[8] = {0};
    for(size_t i = 0; i < 8; ++i)
    {
        req[i] = rq_data_ptr->data2.data[i + 1];
    }
    rs_data_ptr->error_code.data = servo_comm_ptr_[axis_id]->doServoCmdGetServoDefinedInfo(&req[0], &res[0]);
    rs_data_ptr->data.data_count = 8;
    for(size_t i = 0; i < rs_data_ptr->data.data_count; ++i)
    {
        rs_data_ptr->data.data[i] = res[i];
    }
    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/servo1001/servo/getServoDefinedInfo called axis[%d] success", axis_id);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/servo/getServoDefinedInfo called axis[%d] failed", axis_id);
    }
}

//"/rpc/servo1001/cpu/getVersion"	
void ControllerRpc::handleRpc0x0001192E(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint32List*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data.data;//todo with axis_manager
    rs_data_ptr->data.data_count = 2;
    rs_data_ptr->data.data[0] = cpu_comm_ptr_->getMajorVersion();
    rs_data_ptr->data.data[1] = cpu_comm_ptr_->getMinorVersion();
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/getVersion: cpu(%d) version: %d.%d", 
        cpu, rs_data_ptr->data.data[0], rs_data_ptr->data.data[1]);
}

//"/rpc/servo1001/cpu/setCtrlPdoSync"	
void ControllerRpc::handleRpc0x00005123(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_Uint32* rq_data_ptr = static_cast<RequestMessageType_Int32List_Uint32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if (rq_data_ptr->data1.data_count != 2)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/cpu/setCtrlPdoSync input invalid params");
        return;
    }
    
    int32_t cpu = rq_data_ptr->data1.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data1.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/setCtrlPdoSync input invalid params for axis[%d]", axis_id);        
        return;
    }
    uint32_t pdo_sync = rq_data_ptr->data2.data;
    int32_t index = 0; 

    rs_data_ptr->data.data = axis_ptr_[axis_id]->mcReadParamter(SERVO_PARAM_CTRL_PDO_SYNC_INDEX, index);
    if (rs_data_ptr->data.data == SUCCESS)
    {
        cpu_comm_ptr_->setCtrlPdoSync(index, pdo_sync);
        LogProducer::info("rpc", "/rpc/servo1001/cpu/setCtrlPdoSync called cpu(%d) success", cpu);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/cpu/setCtrlPdoSync called cpu(%d) failed", cpu);
    }
}

//"/rpc/servo1001/cpu/getCtrlPdoSync"	
void ControllerRpc::handleRpc0x00005463(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32_Uint32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32_Uint32*>(response_data_ptr);
    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/cpu/getCtrlPdoSync input invalid params");
        return;
    }

    int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/getCtrlPdoSync input invalid params for axis[%d]", axis_id);        
        return;
    }
    int32_t index = 0; 

    rs_data_ptr->error_code.data = axis_ptr_[axis_id]->mcReadParamter(SERVO_PARAM_CTRL_PDO_SYNC_INDEX, index);
    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data1.data = axis_id;
        rs_data_ptr->data2.data = cpu_comm_ptr_->getCtrlPdoSync(index);
        LogProducer::info("rpc", "/rpc/servo1001/cpu/getCtrlPdoSync called cpu(%d) axis(%d) success, sync:%x", 
            cpu, axis_id, rs_data_ptr->data2.data);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/servo1001/cpu/getCtrlPdoSync called cpu(%d) axis(%d) failed", cpu, axis_id);
    }
}

//"/rpc/servo1001/cpu/setSamplingSync"	
void ControllerRpc::handleRpc0x00004023(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Uint32* rq_data_ptr = static_cast<RequestMessageType_Int32_Uint32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data1.data;//todo with axis_manager
    uint32_t sampling_sync = rq_data_ptr->data2.data;
    cpu_comm_ptr_->setSamplingSync(sampling_sync);
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/setSamplingSync called cpu(%d) success, setting sync:%x", 
        cpu, sampling_sync);
}

//"/rpc/servo1001/cpu/getSamplingSync"	
void ControllerRpc::handleRpc0x00006C23(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint32*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data.data;//todo with axis_manager
    rs_data_ptr->data.data = cpu_comm_ptr_->getSamplingSync();
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/getSamplingSync called cpu(%d) success, syc:%x", 
        cpu, rs_data_ptr->data.data);
}

//"/rpc/servo1001/cpu/setSamplingInterval"	
void ControllerRpc::handleRpc0x00003EEC(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Uint32* rq_data_ptr = static_cast<RequestMessageType_Int32_Uint32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data1.data;//todo with axis_manager
    uint32_t sampling_interval = rq_data_ptr->data2.data;
    cpu_comm_ptr_->setSamplingInterval(sampling_interval);
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/setSamplingInterval called cpu(%d) success, setting interval:%u", 
        cpu, sampling_interval);
}

//"/rpc/servo1001/cpu/getSamplingInterval"	
void ControllerRpc::handleRpc0x00001C2C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint32*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data.data;//todo with axis_manager
    rs_data_ptr->data.data = cpu_comm_ptr_->getSamplingInterval();
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/getSamplingInterval called cpu(%d) success, interval:%u", 
        cpu, rs_data_ptr->data.data);
}

//"/rpc/servo1001/cpu/setSamplingMaxTimes"	
void ControllerRpc::handleRpc0x000110A3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Uint32* rq_data_ptr = static_cast<RequestMessageType_Int32_Uint32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data1.data;//todo with axis_manager
    uint32_t sampling_max_times = rq_data_ptr->data2.data;
    cpu_comm_ptr_->setSamplingMaxTimes(sampling_max_times);
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/setSamplingMaxTimes called cpu(%d) success, setting times:%u", 
        cpu, sampling_max_times);
}

//"/rpc/servo1001/cpu/getSamplingMaxTimes"	
void ControllerRpc::handleRpc0x00013363(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint32*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data.data;//todo with axis_manager
    rs_data_ptr->data.data = cpu_comm_ptr_->getSamplingMaxTimes();
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/getSamplingMaxTimes called cpu(%d) success, times:%u",
        cpu, rs_data_ptr->data.data);
}

//"/rpc/servo1001/cpu/setSamplingChannel"	
void ControllerRpc::handleRpc0x00008E5C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Uint32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Uint32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data2.data_count != 3)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/setSamplingChannel input invalid params");
        return;
    }

    int32_t cpu = rq_data_ptr->data1.data;//todo with axis_manager
    uint32_t channel_index = rq_data_ptr->data2.data[0];
    uint32_t servo_index = rq_data_ptr->data2.data[1];
    uint32_t servo_param_index = rq_data_ptr->data2.data[2];
    uint32_t channel_value = servo_param_index + (servo_index << 16);
    cpu_comm_ptr_->setSamplingChannel(channel_index, channel_value);
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/setSamplingChannel called cpu(%d) success, setting channel_index=%u, servo_index:%u, servo_param_index:%u, channel_value:%u",
        cpu, channel_index, servo_index, servo_param_index, channel_value);
}

//"/rpc/servo1001/cpu/getSamplingChannel"	
void ControllerRpc::handleRpc0x0000FD9C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint32List*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data.data;//todo with axis_manager

    std::vector<uint32_t> vec = cpu_comm_ptr_->getSamplingChannel();
    rs_data_ptr->data.data_count = 16;
    for (size_t i = 0; i < vec.size(); ++i)
    {
        rs_data_ptr->data.data[i] = vec[i];
    }
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/getSamplingChannel called cpu(%d) success", cpu);
}

//"/rpc/servo1001/cpu/activateSamplingConfiguration"	
void ControllerRpc::handleRpc0x0000939E(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data.data;//todo with axis_manager
    cpu_comm_ptr_->enableSamplingCfg();
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/activateSamplingConfiguration called cpu(%d) success", cpu);
}

//"/rpc/servo1001/cpu/saveSamplingBufferData"	
extern uint8_t g_sampling_data[];
void ControllerRpc::handleRpc0x00015621(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_String* rq_data_ptr = static_cast<RequestMessageType_Int32_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data1.data;//todo with axis_manager
    sampling_file_path = rq_data_ptr->data2.data;

    // check if destination directory is existed.
    std::string dest = rq_data_ptr->data2.data;
    dest = dest.substr(0, dest.rfind('/') + 1);                   
    if (access(dest.c_str(), 0) == -1)
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/cpu/saveSamplingBufferData path not exist: %s", dest.c_str());
        return;
    }
    
    cpu_comm_ptr_->getSamplingBufferData(&g_sampling_data[0], &sampling_data_byte_size);

    if(!save_file_thread_.run(rpcSaveSamplingDataToFileThreadFunc, this, 20))
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/cpu/saveSamplingBufferData start thread failed");
        return;
    }

    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/saveSamplingBufferData called cpu(%d) success", cpu);
}

//"/rpc/servo1001/servo/getServoCommInfo"	
void ControllerRpc::handleRpc0x0000BF1F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/getServoCommInfo input invalid params");
        return;
    }
    int32_t cpu = rq_data_ptr->data.data[0];//todo with axis_manager
    int32_t axis_id = rq_data_ptr->data.data[1];
    if(axis_id >= AXIS_NUM || axis_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/servo/getServoCommInfo input invalid params for axis[%d]", axis_id);        
        return;
    }

    servo_comm_space::ServoCommInfo_t info;
    servo_comm_ptr_[axis_id]->getServoCommInfo(&info);
    rs_data_ptr->data.data_count = 6;
    rs_data_ptr->data.data[0] = info.comm_reg_id;
    rs_data_ptr->data.data[1] = info.service_id;
    rs_data_ptr->data.data[2] = info.download_param_id;
    rs_data_ptr->data.data[3] = info.upload_param_id;
    rs_data_ptr->data.data[4] = info.ctrl_pdo_id;
    rs_data_ptr->data.data[5] = info.fdb_pdo_id;
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/servo/getServoCommInfo called cpu(%d) success", cpu);
}

//"/rpc/servo1001/cpu/getServoCpuCommInfo"	
void ControllerRpc::handleRpc0x0000FE5F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    int32_t cpu = rq_data_ptr->data.data;//todo with axis_manager

    servo_comm_space::ServoCpuCommInfo_t info;
    cpu_comm_ptr_->getServoCpuCommInfo(&info);
    rs_data_ptr->data.data_count = 3;
    rs_data_ptr->data.data[0] = info.comm_reg_id;
    rs_data_ptr->data.data[1] = info.sampling_buffer_id;
    rs_data_ptr->data.data[2] = info.param_reg_id;
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/servo1001/cpu/getServoCpuCommInfo called cpu(%d) success, reg_id:%d, buffer_id:%d, param_reg_id:%d", 
        cpu, info.comm_reg_id, info.sampling_buffer_id, info.param_reg_id);
}

//"/rpc/servo1001/cpu/setForceControlParameters"	
void ControllerRpc::handleRpc0x00005F53(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data2.data_count != COMM_REG2_PARAMETER_NUMBER)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/servo1001/cpu/setStatorParameters input invalid params");
        return;
    }
    //int32_t cpu = rq_data_ptr->data1.data;

    CommRegForceControlParam_t params;
    for(size_t i = 0; i < rq_data_ptr->data2.data_count; ++i)
    {
        params.parameter[i] = rq_data_ptr->data2.data[i];
    }

    if (cpu_comm_ptr_->setForceControlParameters(&params))
    {
        rs_data_ptr->data.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/servo1001/cpu/setForceControlParameters success");
    }
    else
    {
        rs_data_ptr->data.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/cpu/setForceControlParameters failed");
        return;
    }

    //save force_control parameters to files 
    for (size_t i = 0; i < rq_data_ptr->data2.data_count; ++i)
    {
        force_model_ptr_->force_param_ptr->set(i, params.parameter[i]);
    }
    LogProducer::info("rpc", "/rpc/servo1001/cpu/setForceControlParameters to save parameters");
    if (!force_model_ptr_->force_param_ptr->save())
    {
        LogProducer::error("rpc", "/rpc/servo1001/cpu/setForceControlParameters to save parameters failed");
    }

}
//"/rpc/servo1001/cpu/getForceControlParameters"	
void ControllerRpc::handleRpc0x00008203(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    //int32_t cpu = rq_data_ptr->data.data;
    CommRegForceControlParam_t params;
    memset(&params, 0, sizeof(params));
    if (cpu_comm_ptr_->getForceControlParameters(&params))
    {
        rs_data_ptr->data.data_count = COMM_REG2_PARAMETER_NUMBER;
        for(size_t i = 0; i < rs_data_ptr->data.data_count; ++i)
        {
            rs_data_ptr->data.data[i] = params.parameter[i];
        }
        rs_data_ptr->error_code.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/servo1001/cpu/getForceControlParameters success");
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_EXECUTE_FAILED;
        LogProducer::error("rpc", "/rpc/servo1001/cpu/getForceControlParameters failed");
        return;
    }
}


