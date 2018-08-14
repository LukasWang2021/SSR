#ifndef CONTROLLER_RPC_H
#define CONTROLLER_RPC_H

#include "controller_param.h"
#include "common_log.h"
#include "virtual_core1.h"
#include "tp_comm.h"
#include "controller_sm.h"
#include "controller_publish.h"
#include "tool_manager.h"
#include "coordinate_manager.h"
#include "device_manager.h"
#include "reg_manager.h"
#include <vector>

namespace fst_ctrl
{
class ControllerRpc
{
public:
    ControllerRpc();
    ~ControllerRpc();

    void init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, VirtualCore1* virtual_core1_ptr, fst_comm::TpComm* tp_comm_ptr,
                    ControllerSm* state_machine_ptr, ToolManager* tool_manager_ptr, CoordinateManager* coordinate_manager_ptr,
                    RegManager* reg_manager_ptr, fst_hal::DeviceManager* device_manager_ptr);

    void processRpc();

private:
    fst_log::Logger* log_ptr_;
    ControllerParam* param_ptr_;
    VirtualCore1* virtual_core1_ptr_;
    fst_comm::TpComm* tp_comm_ptr_;
    ControllerSm* state_machine_ptr_;
    ToolManager* tool_manager_ptr_;
    CoordinateManager* coordinate_manager_ptr_;
    RegManager* reg_manager_ptr_;
    fst_hal::DeviceManager* device_manager_ptr_;
    ControllerPublish publish_;

    enum {HASH_BYTE_SIZE = 4,};
    enum {QUICK_SEARCH_TABLE_SIZE = 128,};

    typedef void (ControllerRpc::*HandleRpcFuncPtr)(void* request_data_ptr, void* response_data_ptr);

    typedef struct
    {
        std::string path;
        unsigned int hash;
        HandleRpcFuncPtr rpc_func_ptr;
    }RpcService;
    std::vector<RpcService> rpc_table_;
    std::vector<RpcService> rpc_quick_search_table_[QUICK_SEARCH_TABLE_SIZE]; 

    void initRpcTable();
    void initRpcQuickSearchTable();
    HandleRpcFuncPtr getRpcHandlerByHash(unsigned int hash);

    /* publish rpc */
    // "/rpc/publish/addRegTopic"
    void handleRpc0x00000001(void* request_data_ptr, void* response_data_ptr);

    /* controller rpc */
    // "/rpc/controller/addTopic"
    void handleRpc0x00000773(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getUserOpMode"
    void handleRpc0x00000C05(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getRunningStatus"
    void handleRpc0x00000AB3(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getInterpreterStatus"
    void handleRpc0x00016483(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getRobotStatus"
    void handleRpc0x00006F83(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getCtrlStatus"
    void handleRpc0x0000E9D3(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getServoStatus"
    void handleRpc0x0000D113(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/getSafetyAlarm"
    void handleRpc0x0000C00D(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/callEstop"
    void handleRpc0x00013940(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/callReset"
    void handleRpc0x000161E4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/controller/setUserOpMode"
    void handleRpc0x00002ED5(void* request_data_ptr, void* response_data_ptr);

    /* tool manager rpc */
    // "/rpc/tool_manager/addTool"
    void handleRpc0x0000A22C(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/tool_manager/deleteTool"
    void handleRpc0x00010E4C(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/tool_manager/updateTool"
    void handleRpc0x0000C78C(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/tool_manager/moveTool"
    void handleRpc0x000085FC(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/tool_manager/getToolInfoById"
    void handleRpc0x00009E34(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/tool_manager/getAllValidToolSummaryInfo"
    void handleRpc0x0001104F(void* request_data_ptr, void* response_data_ptr);

    /* coordinate manager rpc */
    // "/rpc/coordinate_manager/addUserCoord"
    void handleRpc0x00016764(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/coordinate_manager/deleteUserCoord"
    void handleRpc0x0000BAF4(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/coordinate_manager/updateUserCoord"
    void handleRpc0x0000EC14(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/coordinate_manager/moveUserCoord"
    void handleRpc0x0000E104(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/coordinate_manager/getUserCoordInfoById"
    void handleRpc0x00004324(void* request_data_ptr, void* response_data_ptr);
    // "/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo"
    void handleRpc0x0001838F(void* request_data_ptr, void* response_data_ptr);       
};

}


#endif

