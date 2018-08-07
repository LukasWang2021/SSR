#include "controller_rpc.h"


using namespace fst_ctrl;

void ControllerRpc::initRpcTable()
{
    RpcService rpc_service;
    
    rpc_service = {"/rpc/controller/addTopic", 0x00000773, &ControllerRpc::handleRpc0x00000773}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getUserOpMode", 0x00000C05, &ControllerRpc::handleRpc0x00000C05}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getRunningStatus", 0x00000AB3, &ControllerRpc::handleRpc0x00000AB3}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getInterpreterStatus", 0x00016483, &ControllerRpc::handleRpc0x00016483}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getRobotStatus", 0x00006F83, &ControllerRpc::handleRpc0x00006F83}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getCtrlStatus", 0x0000E9D3, &ControllerRpc::handleRpc0x0000E9D3}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getServoStatus", 0x0000D113, &ControllerRpc::handleRpc0x0000D113}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/getSafetyAlarm", 0x0000C00D, &ControllerRpc::handleRpc0x0000C00D}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/callEstop", 0x00013940, &ControllerRpc::handleRpc0x00013940}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/callReset", 0x000161E4, &ControllerRpc::handleRpc0x000161E4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/controller/setUserOpMode", 0x00002ED5, &ControllerRpc::handleRpc0x00002ED5}; rpc_table_.push_back(rpc_service);

    rpc_service = {"/rpc/tool_manager/addTool", 0x0000A22C, &ControllerRpc::handleRpc0x0000A22C}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/tool_manager/deleteTool", 0x00010E4C, &ControllerRpc::handleRpc0x00010E4C}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/tool_manager/updateTool", 0x0000C78C, &ControllerRpc::handleRpc0x0000C78C}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/tool_manager/moveTool", 0x000085FC, &ControllerRpc::handleRpc0x000085FC}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/tool_manager/getToolInfoById", 0x00009E34, &ControllerRpc::handleRpc0x00009E34}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/tool_manager/getAllValidToolSummaryInfo", 0x0001104F, &ControllerRpc::handleRpc0x0001104F}; rpc_table_.push_back(rpc_service);

    rpc_service = {"/rpc/coordinate_manager/addUserCoord", 0x00016764, &ControllerRpc::handleRpc0x00016764}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/coordinate_manager/deleteUserCoord", 0x0000BAF4, &ControllerRpc::handleRpc0x0000BAF4}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/coordinate_manager/updateUserCoord", 0x0000EC14, &ControllerRpc::handleRpc0x0000EC14}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/coordinate_manager/moveUserCoord", 0x0000E104, &ControllerRpc::handleRpc0x0000E104}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/coordinate_manager/getUserCoordInfoById", 0x00004324, &ControllerRpc::handleRpc0x00004324}; rpc_table_.push_back(rpc_service);
    rpc_service = {"/rpc/coordinate_manager/getAllValidUserCoordSummaryInfo", 0x0001838F, &ControllerRpc::handleRpc0x0001838F}; rpc_table_.push_back(rpc_service);

    
}

