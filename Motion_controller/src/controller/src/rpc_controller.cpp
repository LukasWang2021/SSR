#include "controller_rpc.h"
#include <sys/time.h>
#include <unistd.h>
#include <list>

using namespace user_space;
using namespace base_space;
using namespace log_space;
using namespace servo_comm_space;

// "/rpc/controller/getVersion"
void ControllerRpc::handleRpc0x000093EE(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Uint32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint32List*>(response_data_ptr);

    rs_data_ptr->data.data_count = 4;
    rs_data_ptr->data.data[0] = device_version_.getControllerMajorVersion();
    rs_data_ptr->data.data[1] = device_version_.getControllerMinorVersion();
    rs_data_ptr->data.data[2] = cpu_comm_ptr_->getMajorVersion();
    rs_data_ptr->data.data[3] = cpu_comm_ptr_->getMinorVersion();
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/controller/getVersion controller:%d.%d, servo:%d.%d", 
        rs_data_ptr->data.data[0], rs_data_ptr->data.data[1], rs_data_ptr->data.data[2], rs_data_ptr->data.data[3]);
}

// "/rpc/controller/setSystemTime"
void ControllerRpc::handleRpc0x000167C5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Uint64* rq_data_ptr = static_cast<RequestMessageType_Uint64*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    struct timeval time_val;
    time_val.tv_sec = rq_data_ptr->data.data;
    time_val.tv_usec = 0;
    if(settimeofday(&time_val, NULL) == 0)
    {
        rs_data_ptr->data.data = SUCCESS;
        LogProducer::info("rpc", "/rpc/axis/setSystemTime: %llu", rq_data_ptr->data.data);
    }
    else
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        LogProducer::error("rpc", "/rpc/controller/setSystemTime failed");
    }
}

// "/rpc/controller/getSystemTime"
void ControllerRpc::handleRpc0x000003F5(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint64*>(response_data_ptr);

    struct timeval time_val;
    gettimeofday(&time_val, NULL);
    rs_data_ptr->data.data = time_val.tv_sec;
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/controller/getSystemTime: %llu", rs_data_ptr->data.data);
}

//"/rpc/controller/setWorkMode"	
void ControllerRpc::handleRpc0x00006825(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Uint32* rq_data_ptr = static_cast<RequestMessageType_Uint32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    for(size_t i = 0; i < GROUP_NUM; ++i)
    {
        group_ptr_[i]->setWorkMode((group_space::UserOpMode)rq_data_ptr->data.data);
    }
    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/controller/setWorkMode: %u", rq_data_ptr->data.data);
}

//"/rpc/controller/getWorkMode"	
void ControllerRpc::handleRpc0x00003325(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Uint32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint32*>(response_data_ptr);

    for(size_t i = 0; i < GROUP_NUM; ++i)
    {
        rs_data_ptr->data.data = group_ptr_[i]->getWorkMode();
    }
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/controller/getWorkMode: %u", rs_data_ptr->data.data);
}

//"/rpc/controller/setControlMode"	
void ControllerRpc::handleRpc0x0000B555(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Uint32* rq_data_ptr = static_cast<RequestMessageType_Uint32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    uint32_t mode = rq_data_ptr->data.data;
    ServoControlMode control_mode = CONTROL_MODE_POSITION;
    switch (mode)
    {
        case 0:
            control_mode = CONTROL_MODE_POSITION;
            break;
        case 1:
            control_mode = CONTROL_MODE_FORCE;
            break;
        default:
            rs_data_ptr->data.data = RPC_PARAM_INVALID;
            LogProducer::error("rpc", "/rpc/controller/setControlMode to be invalid(%u)", mode);
            return;
    }
    cpu_comm_ptr_->setServoControlMode(control_mode);

    rs_data_ptr->data.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/controller/setControlMode to be %u", rq_data_ptr->data.data);
}

//"/rpc/controller/getControlMode"	
void ControllerRpc::handleRpc0x0000B695(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Uint32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint32*>(response_data_ptr);

    rs_data_ptr->data.data = cpu_comm_ptr_->getServoControlMode();
    rs_data_ptr->error_code.data = SUCCESS;
    LogProducer::info("rpc", "/rpc/controller/getControlMode is %u", rs_data_ptr->data.data);
}


