#include "controller_ipc.h"
#include "io_interface.h"
#include <unistd.h>
#include <stdlib.h>
#include <cstring>

//using namespace fst_hal;
using namespace fst_ctrl;
using namespace fst_base;

void ControllerIpc::handleIpcCheckIo(void* request_data_ptr, void* response_data_ptr)
{
    char* rq_data_ptr = static_cast<char*>(request_data_ptr);
    ResponseCheckIo* rs_data_ptr = static_cast<ResponseCheckIo*>(response_data_ptr);

    
}

void ControllerIpc::handleIpcSetIo(void* request_data_ptr, void* response_data_ptr)
{
    RequestSetIo* rq_data_ptr = static_cast<RequestSetIo*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);

}

void ControllerIpc::handleIpcGetIo(void* request_data_ptr, void* response_data_ptr)
{
    RequestGetIo* rq_data_ptr = static_cast<RequestGetIo*>(request_data_ptr);
    ResponseGetIo* rs_data_ptr = static_cast<ResponseGetIo*>(response_data_ptr);

}

