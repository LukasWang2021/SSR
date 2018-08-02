#include "controller_rpc.h"


using namespace fst_ctrl;

void ControllerRpc::initRpcTable()
{
    RpcService rpc_service;
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
    
}

