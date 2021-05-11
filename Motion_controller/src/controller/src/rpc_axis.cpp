#include "controller_rpc.h"

using namespace user_space;
using namespace log_space;
using namespace axis_space;


//"/rpc/axis/mcPower"	
void ControllerRpc::handleRpc0x000053E2(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Bool* rq_data_ptr = static_cast<RequestMessageType_Int32_Bool*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data1.data;
    bool enable = rq_data_ptr->data2.data;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->mcPower(enable);
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcPower for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/mcPower for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);        
}

//"/rpc/axis/mcReset"	
void ControllerRpc::handleRpc0x000180C4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->mcReset();
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcReset for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/mcReset for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);  
}

//"/rpc/axis/mcStop"	
void ControllerRpc::handleRpc0x00002820(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->mcStop();
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcStop for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/mcStop for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data); 
}

//"/rpc/axis/mcHalt"	
void ControllerRpc::handleRpc0x00004BB4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->mcHalt();
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcHalt for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/mcHalt for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data); 
}

//"/rpc/axis/mcSetPosition"	
void ControllerRpc::handleRpc0x0001798E(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Double* rq_data_ptr = static_cast<RequestMessageType_Int32_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data1.data;
    double position = rq_data_ptr->data2.data;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->mcSetPosition(position);
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }
    
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcSetPosition for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/mcSetPosition for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data); 
}

//"/rpc/axis/mcReadParameter"	
void ControllerRpc::handleRpc0x00016BF2(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/axis/mcReadParameter input invalid params");
        return;
    }

    int32_t axis_id = rq_data_ptr->data.data[0];
    int32_t param_index = rq_data_ptr->data.data[1];
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->error_code.data = axis_ptr_[axis_id]->mcReadParamter(param_index, rs_data_ptr->data.data);
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
    }

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcReadParameter for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/mcReadParameter for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->error_code.data); 
}

//"/rpc/axis/mcWriteParameter"	
void ControllerRpc::handleRpc0x00005732(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data.data_count != 3)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/axis/mcWriteParameter input invalid params");
        return;
    }

    int32_t axis_id = rq_data_ptr->data.data[0];
    int32_t param_index = rq_data_ptr->data.data[1];
    int32_t param_value = rq_data_ptr->data.data[2];
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->mcWriteParamter(param_index, param_value);
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }
    
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcWriteParameter for axis[%d] success, index:%d, value:%d", axis_id, param_index, param_value);
    else
        LogProducer::error("rpc", "/rpc/axis/mcWriteParameter for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);
}

//"/rpc/axis/mcMoveAbsolute"	
void ControllerRpc::handleRpc0x000051F5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data2.data_count != 5)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/axis/mcMoveAbsolute input invalid params");
        return;
    }

    int32_t axis_id = rq_data_ptr->data1.data;
    double position = rq_data_ptr->data2.data[0];
    double velocity = rq_data_ptr->data2.data[1];
    double acc = rq_data_ptr->data2.data[2];
    double dec = rq_data_ptr->data2.data[3];
    double jerk = rq_data_ptr->data2.data[4];
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->mcMoveAbsolute(position, velocity, acc, dec, jerk);
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }
    

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcMoveAbsolute for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/mcMoveAbsolute for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);
}

//"/rpc/axis/mcMoveVelocity"	
void ControllerRpc::handleRpc0x00016CF9(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32List_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data1.data_count != 2 || rq_data_ptr->data2.data_count != 4)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/axis/mcMoveVelocity input invalid params");
        return;
    }

    int32_t axis_id = rq_data_ptr->data1.data[0];
    int32_t direction = rq_data_ptr->data1.data[1];
    double velocity = rq_data_ptr->data2.data[0];
    double acc = rq_data_ptr->data2.data[1];
    double dec = rq_data_ptr->data2.data[2];
    double jerk = rq_data_ptr->data2.data[3];
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->mcMoveVelocity(velocity, acc, dec, jerk, (axis_space::AxisDirection_e)direction);
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }
    
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcMoveVelocity for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/mcMoveVelocity for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);

}

//"/rpc/axis/mcReadActualPosition"	
void ControllerRpc::handleRpc0x000012BE(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->error_code.data = axis_ptr_[axis_id]->mcReadActualPosition(rs_data_ptr->data.data);
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
    }

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcReadActualPosition for axis[%d] success, position:%lf", axis_id, rs_data_ptr->data.data);
    else
        LogProducer::error("rpc", "/rpc/axis/mcReadActualPosition for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->error_code.data); 
}

//"/rpc/axis/mcReadActualVelocity"	
void ControllerRpc::handleRpc0x00002EA9(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->error_code.data = axis_ptr_[axis_id]->mcReadActualVelocity(rs_data_ptr->data.data);
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
    }

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcReadActualVelocity for axis[%d] success, velocity:%lf", axis_id,rs_data_ptr->data.data);
    else
        LogProducer::error("rpc", "/rpc/axis/mcReadActualVelocity for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->error_code.data); 
}

//"/rpc/axis/mcReadActualTorque"	
void ControllerRpc::handleRpc0x00014265(void* request_data_ptr, void* response_data_ptr)
{    
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->error_code.data = axis_ptr_[axis_id]->mcReadActualTorque(rs_data_ptr->data.data);
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
    }    

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcReadActualTorque for axis[%d] success, torque:%lf", axis_id, rs_data_ptr->data.data);
    else
        LogProducer::error("rpc", "/rpc/axis/mcReadActualTorque for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->error_code.data); 
}

//"/rpc/axis/mcReadAxisInfo"	
void ControllerRpc::handleRpc0x0000314F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_AxisInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_AxisInfo*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    axis_space::AxisInfo_b info;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->error_code.data = axis_ptr_[axis_id]->mcReadAxisInfo(info);
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
    }  
    
    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.simulation = info.simulation;
        rs_data_ptr->data.communication_ready = info.communication_ready;
        rs_data_ptr->data.ready_for_power_on = info.ready_power_on;
        rs_data_ptr->data.power_on = info.power_on;
        LogProducer::info("rpc", "/rpc/axis/mcReadAxisInfo for axis[%d] success", axis_id);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/axis/mcReadAxisInfo for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->error_code.data); 
    }
}

//"/rpc/axis/mcReadStatus"	
void ControllerRpc::handleRpc0x00003E53(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_AxisStatus* rs_data_ptr = static_cast<ResponseMessageType_Uint64_AxisStatus*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    axis_space::AxisStatus_e status;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->error_code.data = axis_ptr_[axis_id]->mcReadStatus(status);
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
    } 
    
    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        switch (status)
        {
            case AXIS_STATUS_UNKNOWN:          
                rs_data_ptr->data = MessageType_AxisStatus_UNKNOWN;
                break;
            case AXIS_STATUS_ERRORSTOP:
                rs_data_ptr->data = MessageType_AxisStatus_ERRORSTOP;
                break;
            case AXIS_STATUS_DISABLED:
                rs_data_ptr->data = MessageType_AxisStatus_DISABLED;
                break;
            case AXIS_STATUS_STANDSTILL:
                rs_data_ptr->data = MessageType_AxisStatus_STANDSTILL;
                break;            
            case AXIS_STATUS_STOPPING:
                rs_data_ptr->data = MessageType_AxisStatus_STOPPING;
                break;
            case AXIS_STATUS_HOMING:
                rs_data_ptr->data = MessageType_AxisStatus_HOMING;
                break;
            case AXIS_STATUS_DISCRETE_MOTION:
                rs_data_ptr->data = MessageType_AxisStatus_DISCRETE_MOTION;
                break;
            case AXIS_STATUS_CONTINUOUS_MOTION:
                rs_data_ptr->data = MessageType_AxisStatus_CONTINUOUS_MOTION;
                break;
            case AXIS_STATUS_SYNCHRONIZED_MOTION:
                rs_data_ptr->data = MessageType_AxisStatus_SYNCHRONIZED_MOTION;
                break;
            default:
                rs_data_ptr->data = MessageType_AxisStatus_UNKNOWN;
                break;
        }
        LogProducer::info("rpc", "/rpc/axis/mcReadStatus for axis[%d] success, status:%d", axis_id, status);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/axis/mcReadStatus for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->error_code.data); 
    }
}

//"/rpc/axis/mcReadAxisError"	
void ControllerRpc::handleRpc0x000063C2(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint64*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    int32_t error = 0;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->error_code.data = axis_ptr_[axis_id]->mcReadAxisError(error);
        rs_data_ptr->data.data = error;
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
    } 
    
    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/axis/mcReadAxisError for axis[%d] success, error=%x", axis_id, error);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/axis/mcReadAxisError for axis[%d] failed, Error = 0x%llx", axis_id, rs_data_ptr->error_code.data);
    }
}

//"/rpc/axis/mcReadAxisErrorHistory"	
void ControllerRpc::handleRpc0x00018469(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint64List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint64List*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    int size = 8;
    int32_t error_list[8] = {0};
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->error_code.data = axis_ptr_[axis_id]->rtmReadAxisErrorHistory(error_list, size);
        rs_data_ptr->data.data_count = size;
        for (int i = 0; i < size; ++i)
        {
            rs_data_ptr->data.data[i] = error_list[i];
        }
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
    } 
    
    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/axis/mcReadAxisErrorHistory for axis[%d] success", axis_id);
    }
    else
    {
        LogProducer::error("rpc", "/rpc/axis/mcReadAxisErrorHistory for axis[%d] failed", axis_id);
    }
}

//"/rpc/axis/mcMoveRelative"    
void ControllerRpc::handleRpc0x0000CC85(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data2.data_count != 5)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/axis/mcMoveRelative input invalid params");
        return;
    }

    int32_t axis_id = rq_data_ptr->data1.data;
    double position = rq_data_ptr->data2.data[0];
    double velocity = rq_data_ptr->data2.data[1];
    double acc = rq_data_ptr->data2.data[2];
    double dec = rq_data_ptr->data2.data[3];
    double jerk = rq_data_ptr->data2.data[4];
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->mcMoveRelative(position, velocity, acc, dec, jerk);
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }
    
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcMoveRelative for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/mcMoveRelative for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);
}

//"/rpc/axis/mcHome"    
void ControllerRpc::handleRpc0x000059B5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->mcHome();
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/mcHome for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/mcHome for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);  
}

//"/rpc/axis/rtmAbortHoming"    
void ControllerRpc::handleRpc0x0000E4B7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->rtmAbortHoming();
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/rtmAbortHoming for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/rtmAbortHoming for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);  
}

//"/rpc/axis/rtmReadAxisFdbPdoPtr"    
void ControllerRpc::handleRpc0x0000A632(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    int32_t pdo_data_size = 0;
    uint8_t* pdo_data_ptr = NULL;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        pdo_data_ptr = axis_ptr_[axis_id]->rtmReadAxisFdbPdoPtr(&pdo_data_size);
        memcpy(rs_data_ptr->data.data, pdo_data_ptr, pdo_data_size);
        rs_data_ptr->data.data_count = pdo_data_size / sizeof(int32_t);
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
    }

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/rtmReadAxisFdbPdoPtr for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/rtmReadAxisFdbPdoPtr for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);  
}

//"/rpc/axis/rtmResetEncoder"	
void ControllerRpc::handleRpc0x00000BA2(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t axis_id = rq_data_ptr->data.data;
    if(axis_id < AXIS_NUM && axis_id >= 0)
    {
        rs_data_ptr->data.data = axis_ptr_[axis_id]->rtmResetEncoder();
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/axis/rtmResetEncoder for axis[%d] success", axis_id);
    else
        LogProducer::error("rpc", "/rpc/axis/rtmResetEncoder for axis[%d] failed. Error = 0x%llx", axis_id, rs_data_ptr->data.data);  
}
