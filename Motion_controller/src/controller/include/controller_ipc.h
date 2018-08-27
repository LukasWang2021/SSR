#ifndef CONTROLLER_IPC_H
#define CONTROLLER_IPC_H

#include "controller_param.h"
#include "common_log.h"
#include "process_comm.h"
#include "reg_manager.h"
#include <vector>

namespace fst_ctrl
{
class ControllerIpc
{
public:
    ControllerIpc();
    ~ControllerIpc();

    void init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, 
                fst_base::ControllerServer* controller_server_ptr, RegManager* reg_manager_ptr);

    void processIpc();
    fst_base::ControllerClient* getControllerClientPtr();

private:
    fst_log::Logger* log_ptr_;
    ControllerParam* param_ptr_;
    fst_base::ControllerServer* controller_server_ptr_;
    RegManager* reg_manager_ptr_;

    enum {CMD_ID_BYTE_SIZE = 4,};

    typedef void (ControllerIpc::*HandleIpcFuncPtr)(void* request_data_ptr, void* response_data_ptr);

    typedef struct
    {
        unsigned int cmd_id;
        HandleIpcFuncPtr ipc_func_ptr;
    }IpcService;
    std::vector<IpcService> ipc_table_;

    void initIpcTable();

    void handleIpcSetPrRegPos(void* request_data_ptr, void* response_data_ptr);
    void handleIpcSetHrRegJointPos(void* request_data_ptr, void* response_data_ptr);
    void handleIpcSetMrRegValue(void* request_data_ptr, void* response_data_ptr);
    void handleIpcSetSrRegValue(void* request_data_ptr, void* response_data_ptr);
    void handleIpcSetRRegValue(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetPrRegPos(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetHrRegJointPos(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetMrRegValue(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetSrRegValue(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetRRegValue(void* request_data_ptr, void* response_data_ptr); 
    void handleIpcSetInstruction(void* request_data_ptr, void* response_data_ptr);
    void handleIpcIsNextInstructionNeeded(void* request_data_ptr, void* response_data_ptr);
};

}


#endif

