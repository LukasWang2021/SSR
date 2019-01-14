#include "controller_rpc.h"
#include <sys/time.h>
#include <unistd.h>
#include <list>
#include "version.h"

using namespace fst_ctrl;
using namespace fst_base;

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

// "/rpc/controller/getVersion"
void ControllerRpc::handleRpc0x000093EE(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_String* rs_data_ptr = static_cast<ResponseMessageType_Uint64_String*>(response_data_ptr);
    
    std::string str = get_version();
    int len = strlen(str.c_str());
    memcpy(rs_data_ptr->data.data, str.c_str(), len);
    rs_data_ptr->data.data[len] = '\0';
    rs_data_ptr->error_code.data = SUCCESS;
}

// "/rpc/controller/getErrorCodeList"
void ControllerRpc::handleRpc0x00015F44(void* request_data_ptr, void* response_data_ptr)
{ 
    ResponseMessageType_Uint64_Uint64List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Uint64List*>(response_data_ptr);
    
    int index = 0;
    std::list<uint64_t> list = ErrorMonitor::instance()->getErrorList();
    for (std::list<uint64_t>::iterator iter = list.begin(); iter != list.end(); iter++)
    {
        rs_data_ptr->data.data[index] = *iter;
        index ++;
    }
    rs_data_ptr->data.data_count = index;
    rs_data_ptr->error_code.data = SUCCESS;    
}
    
