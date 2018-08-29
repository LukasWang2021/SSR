#include "controller_rpc.h"
#include <sys/time.h>
#include <unistd.h>


using namespace fst_ctrl;


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
    }
    else
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
    }
}

// "/rpc/controller/getSystemTime"
void ControllerRpc::handleRpc0x000003F5(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64List* rs_data_ptr = static_cast<ResponseMessageType_Uint64List*>(response_data_ptr);

    struct timeval time_val;
    gettimeofday(&time_val, NULL);
    rs_data_ptr->data.data_count = 2;
    rs_data_ptr->data.data[0] = SUCCESS;
    rs_data_ptr->data.data[1] = time_val.tv_sec;
}

