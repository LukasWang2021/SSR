#include "controller_ipc.h"


using namespace fst_ctrl;
using namespace fst_base;

void ControllerIpc::initIpcTable()
{
    IpcService ipc_service;
    
    ipc_service = {CONTROLLER_SERVER_CMD_SET_PR_REG, &ControllerIpc::handleIpcSetPrReg}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_SET_HR_REG, &ControllerIpc::handleIpcSetHrReg}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_SET_MR_REG, &ControllerIpc::handleIpcSetMrReg}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_SET_SR_REG, &ControllerIpc::handleIpcSetSrReg}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_SET_R_REG, &ControllerIpc::handleIpcSetRReg}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_GET_PR_REG, &ControllerIpc::handleIpcGetPrReg}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_GET_HR_REG, &ControllerIpc::handleIpcGetHrReg}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_GET_MR_REG, &ControllerIpc::handleIpcGetMrReg}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_GET_SR_REG, &ControllerIpc::handleIpcGetSrReg}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_GET_R_REG, &ControllerIpc::handleIpcGetRReg}; ipc_table_.push_back(ipc_service);
}

