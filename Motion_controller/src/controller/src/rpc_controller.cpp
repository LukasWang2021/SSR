#include "controller_rpc.h"
#include <sys/time.h>
#include <unistd.h>
#include <list>

using namespace user_space;
using namespace base_space;
using namespace log_space;

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
        LogProducer::error("rpc", "/rpc/axis/setSystemTime failed");
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
    LogProducer::info("rpc", "/rpc/axis/getSystemTime: %llu", rs_data_ptr->data.data);
}

//"/rpc/controller/setWorkMode"	
void ControllerRpc::handleRpc0x00006825(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_WorkMode* rq_data_ptr = static_cast<RequestMessageType_WorkMode*>(request_data_ptr);
    //ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    //todo
}

//"/rpc/controller/getWorkMode"	
void ControllerRpc::handleRpc0x00003325(void* request_data_ptr, void* response_data_ptr)
{
    //ResponseMessageType_Uint64_WorkMode* rs_data_ptr = static_cast<ResponseMessageType_Uint64_WorkMode*>(response_data_ptr);

    //todo
}



