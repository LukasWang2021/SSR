#include "controller_ipc.h"
#include "process_comm.h"
#include <unistd.h>
#include <stdlib.h>
#include <cstring>

using namespace fst_ctrl;
using namespace fst_base;

void ControllerIpc::handleIpcSetInterpreterServerStatus(void* request_data_ptr, void* response_data_ptr)
{
    bool* rq_data_ptr = static_cast<bool*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);
    *rs_data_ptr = controller_server_ptr_->setInterpreterServerStatus(*rq_data_ptr);    
}

