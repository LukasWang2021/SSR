#ifndef CONTROLLER_IPC_H
#define CONTROLLER_IPC_H

#include "controller_param.h"
#include "common_log.h"
#include "process_comm.h"
#include "reg_manager.h"
#include "device_manager.h"
#include "io_mapping.h"
#include "io_manager.h"
#include "controller_sm.h"
#include <vector>

namespace fst_ctrl
{
class ControllerIpc
{
public:
    ControllerIpc();
    ~ControllerIpc();

    void init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, 
                fst_base::ControllerServer* controller_server_ptr, fst_base::ControllerClient* controller_client_ptr,
                RegManager* reg_manager_ptr, ControllerSm* state_machine_ptr, fst_hal::DeviceManager* device_manager_ptr,
                IoMapping* io_mapping_ptr, fst_mc::MotionControl* motion_control_ptr);

    void processIpc();
private:
    fst_log::Logger* log_ptr_;
    ControllerParam* param_ptr_;
    fst_base::ControllerServer* controller_server_ptr_;
    fst_base::ControllerClient* controller_client_ptr_;
    RegManager* reg_manager_ptr_;
    ControllerSm* state_machine_ptr_;
    fst_hal::DeviceManager* device_manager_ptr_;
    fst_hal::ModbusManager* modbus_manager_ptr_; 
    IoMapping* io_mapping_ptr_;
    fst_mc::MotionControl* motion_control_ptr_;

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
    void handleIpcSetMiValue(void* request_data_ptr, void* response_data_ptr);
    void handleIpcSetMhValue(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetPrRegPos(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetHrRegJointPos(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetMrRegValue(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetSrRegValue(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetRRegValue(void* request_data_ptr, void* response_data_ptr); 
    void handleIpcGetMiValue(void* request_data_ptr, void* response_data_ptr); 
    void handleIpcGetMhValue(void* request_data_ptr, void* response_data_ptr); 
    void handleIpcSetInstruction(void* request_data_ptr, void* response_data_ptr);
    void handleIpcIsNextInstructionNeeded(void* request_data_ptr, void* response_data_ptr); 
    void handleIpcSetInterpreterServerStatus(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetDi(void* request_data_ptr, void* response_data_ptr);
    void handleIpcSetDi(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetDo(void* request_data_ptr, void* response_data_ptr);
    void handleIpcSetDo(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetRi(void* request_data_ptr, void* response_data_ptr);
    void handleIpcSetRi(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetRo(void* request_data_ptr, void* response_data_ptr);
    void handleIpcSetRo(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetUi(void* request_data_ptr, void* response_data_ptr);
    void handleIpcSetUi(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetUo(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetJoint(void* request_data_ptr, void* response_data_ptr);
    void handleIpcGetCart(void* request_data_ptr, void* response_data_ptr);
    void handleIpcCartToJoint(void* request_data_ptr, void* response_data_ptr);
    void handleIpcJointToCart(void* request_data_ptr, void* response_data_ptr);
    void handleIpcUserOpMode(void* request_data_ptr, void* response_data_ptr);

};

}


#endif

