#include "controller_server.h"
#include "interpreter_server.h"


using namespace fst_base;

void ControllerServer::initRpcTable()
{
    ControllerRpcService rpc_service;
    rpc_service = {CONTROLLER_SERVER_CMD_SET_PR_REG, &ControllerServer::handleRequestSetPrReg, &ControllerServer::handleResponseSetPrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_HR_REG, &ControllerServer::handleRequestSetHrReg, &ControllerServer::handleResponseSetHrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_MR_REG, &ControllerServer::handleRequestSetMrReg, &ControllerServer::handleResponseSetMrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_SR_REG, &ControllerServer::handleRequestSetSrReg, &ControllerServer::handleResponseSetSrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_R_REG, &ControllerServer::handleRequestSetRReg, &ControllerServer::handleResponseSetRReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_MI, &ControllerServer::handleRequestSetMi, &ControllerServer::handleResponseSetMi}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_MH, &ControllerServer::handleRequestSetMh, &ControllerServer::handleResponseSetMh}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_PR_REG, &ControllerServer::handleRequestGetPrReg, &ControllerServer::handleResponseGetPrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_HR_REG, &ControllerServer::handleRequestGetHrReg, &ControllerServer::handleResponseGetHrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_MR_REG, &ControllerServer::handleRequestGetMrReg, &ControllerServer::handleResponseGetMrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_SR_REG, &ControllerServer::handleRequestGetSrReg, &ControllerServer::handleResponseGetSrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_R_REG, &ControllerServer::handleRequestGetRReg, &ControllerServer::handleResponseGetRReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_MI, &ControllerServer::handleRequestGetMi, &ControllerServer::handleResponseGetMi}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_MH, &ControllerServer::handleRequestGetMh, &ControllerServer::handleResponseGetMh}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_INSTRUCTION, &ControllerServer::handleRequestSetInstruction, &ControllerServer::handleResponseSetInstruction}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_IS_NEXT_INSTRUCTION_NEEDED, &ControllerServer::handleRequestIsNextInstructionNeeded, &ControllerServer::handleResponseIsNextInstructionNeeded}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_INTERPRETER_SERVER_STATUS, &ControllerServer::handleRequestSetInterpreterServerStatus, &ControllerServer::handleResponseSetInterpreterServerStatus}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_DI, &ControllerServer::handleRequestGetDi, &ControllerServer::handleResponseGetDi}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_DI, &ControllerServer::handleRequestSetDi, &ControllerServer::handleResponseSetDi}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_DO, &ControllerServer::handleRequestGetDo, &ControllerServer::handleResponseGetDo}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_DO, &ControllerServer::handleRequestSetDo, &ControllerServer::handleResponseSetDo}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_RI, &ControllerServer::handleRequestGetRi, &ControllerServer::handleResponseGetRi}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_RI, &ControllerServer::handleRequestSetRi, &ControllerServer::handleResponseSetRi}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_RO, &ControllerServer::handleRequestGetRo, &ControllerServer::handleResponseGetRo}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_RO, &ControllerServer::handleRequestSetRo, &ControllerServer::handleResponseSetRo}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_UI, &ControllerServer::handleRequestGetUi, &ControllerServer::handleResponseGetUi}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_UI, &ControllerServer::handleRequestSetUi, &ControllerServer::handleResponseSetUi}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_UO, &ControllerServer::handleRequestGetUo, &ControllerServer::handleResponseGetUo}; rpc_table_.push_back(rpc_service);
    
}

void InterpreterServer::initRpcTable()
{
    InterpreterRpcService rpc_service;
    rpc_service = {INTERPRETER_SERVER_CMD_START, &InterpreterServer::handleRequestStart, &InterpreterServer::handleResponseStart}; rpc_table_.push_back(rpc_service);
    rpc_service = {INTERPRETER_SERVER_CMD_DEBUG, &InterpreterServer::handleRequestDebug, &InterpreterServer::handleResponseDebug}; rpc_table_.push_back(rpc_service);
    rpc_service = {INTERPRETER_SERVER_CMD_FORWARD, &InterpreterServer::handleRequestForward, &InterpreterServer::handleResponseForward}; rpc_table_.push_back(rpc_service);
    rpc_service = {INTERPRETER_SERVER_CMD_BACKWARD, &InterpreterServer::handleRequestBackward, &InterpreterServer::handleResponseBackward}; rpc_table_.push_back(rpc_service);
    rpc_service = {INTERPRETER_SERVER_CMD_JUMP, &InterpreterServer::handleRequestJump, &InterpreterServer::handleResponseJump}; rpc_table_.push_back(rpc_service);
    rpc_service = {INTERPRETER_SERVER_CMD_PAUSE, &InterpreterServer::handleRequestPause, &InterpreterServer::handleResponsePause}; rpc_table_.push_back(rpc_service);
    rpc_service = {INTERPRETER_SERVER_CMD_RESUME, &InterpreterServer::handleRequestResume, &InterpreterServer::handleResponseResume}; rpc_table_.push_back(rpc_service);
    rpc_service = {INTERPRETER_SERVER_CMD_ABORT, &InterpreterServer::handleRequestAbort, &InterpreterServer::handleResponseAbort}; rpc_table_.push_back(rpc_service);
    rpc_service = {INTERPRETER_SERVER_CMD_GET_NEXT_INSTRUCTION, &InterpreterServer::handleRequestGetNextInstruction, &InterpreterServer::handleResponseGetNextInstruction}; rpc_table_.push_back(rpc_service);
    rpc_service = {INTERPRETER_SERVER_CMD_SET_AUTO_START_MODE, &InterpreterServer::handleRequestSetAutoStartMode, &InterpreterServer::handleResponseSetAutoStartMode}; rpc_table_.push_back(rpc_service);
    rpc_service = {INTERPRETER_SERVER_CMD_SWITCH_STEP, &InterpreterServer::handleRequestSwitchStep, &InterpreterServer::handleResponseSwitchStep}; rpc_table_.push_back(rpc_service);
    rpc_service = {INTERPRETER_SERVER_CMD_CODE_START, &InterpreterServer::handleRequestCodeStart, &InterpreterServer::handleResponseCodeStart}; rpc_table_.push_back(rpc_service);
}

