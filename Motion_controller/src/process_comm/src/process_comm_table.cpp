#include "controller_server.h"

using namespace fst_base;

void ControllerServer::initIpcTable()
{
    ControllerRpcService rpc_service;
    rpc_service = {CONTROLLER_SERVER_CMD_SET_PR_REG, &ControllerServer::handleRequestSetPrReg, &ControllerServer::handleResponseSetPrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_HR_REG, &ControllerServer::handleRequestSetHrReg, &ControllerServer::handleResponseSetHrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_MR_REG, &ControllerServer::handleRequestSetMrReg, &ControllerServer::handleResponseSetMrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_SR_REG, &ControllerServer::handleRequestSetSrReg, &ControllerServer::handleResponseSetSrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_SET_R_REG, &ControllerServer::handleRequestSetRReg, &ControllerServer::handleResponseSetRReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_PR_REG, &ControllerServer::handleRequestGetPrReg, &ControllerServer::handleResponseGetPrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_HR_REG, &ControllerServer::handleRequestGetHrReg, &ControllerServer::handleResponseGetHrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_MR_REG, &ControllerServer::handleRequestGetMrReg, &ControllerServer::handleResponseGetMrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_SR_REG, &ControllerServer::handleRequestGetSrReg, &ControllerServer::handleResponseGetSrReg}; rpc_table_.push_back(rpc_service);
    rpc_service = {CONTROLLER_SERVER_CMD_GET_R_REG, &ControllerServer::handleRequestGetRReg, &ControllerServer::handleResponseGetRReg}; rpc_table_.push_back(rpc_service);
}

