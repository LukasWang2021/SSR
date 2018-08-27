#include "controller_ipc.h"
#include "interpreter_common.h"
#include <unistd.h>
#include <stdlib.h>
#include <cstring>

using namespace fst_ctrl;
using namespace std;

void ControllerIpc::handleIpcSetInstruction(void* request_data_ptr, void* response_data_ptr)
{
    Instruction* rq_data_ptr = static_cast<Instruction*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    *rs_data_ptr = true;
}

void ControllerIpc::handleIpcIsNextInstructionNeeded(void* request_data_ptr, void* response_data_ptr)
{
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    *rs_data_ptr = true;
}


