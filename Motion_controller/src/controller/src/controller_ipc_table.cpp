#include "controller_ipc.h"


using namespace fst_ctrl;
using namespace fst_base;

void ControllerIpc::initIpcTable()
{
    IpcService ipc_service;
    
    ipc_service = {CONTROLLER_SERVER_CMD_SET_PR_REG, &ControllerIpc::handleIpcSetPrRegPos}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_SET_HR_REG, &ControllerIpc::handleIpcSetHrRegJointPos}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_SET_MR_REG, &ControllerIpc::handleIpcSetMrRegValue}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_SET_SR_REG, &ControllerIpc::handleIpcSetSrRegValue}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_SET_R_REG, &ControllerIpc::handleIpcSetRRegValue}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_GET_PR_REG, &ControllerIpc::handleIpcGetPrRegPos}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_GET_HR_REG, &ControllerIpc::handleIpcGetHrRegJointPos}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_GET_MR_REG, &ControllerIpc::handleIpcGetMrRegValue}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_GET_SR_REG, &ControllerIpc::handleIpcGetSrRegValue}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_GET_R_REG, &ControllerIpc::handleIpcGetRRegValue}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_SET_INSTRUCTION, &ControllerIpc::handleIpcSetInstruction}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_IS_NEXT_INSTRUCTION_NEEDED, &ControllerIpc::handleIpcIsNextInstructionNeeded}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_CHECK_IO, &ControllerIpc::handleIpcCheckIo}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_SET_IO, &ControllerIpc::handleIpcSetIo}; ipc_table_.push_back(ipc_service);
    ipc_service = {CONTROLLER_SERVER_CMD_GET_IO, &ControllerIpc::handleIpcGetIo}; ipc_table_.push_back(ipc_service);
}

