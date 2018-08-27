#include "controller_server.h"
#include "reg_manager.h"
#include "interpreter_common.h"


using namespace fst_base;
using namespace fst_ctrl;


// SetPrReg
void ControllerServer::handleRequestSetPrReg()
{
    fst_ctrl::PrRegDataIpc* request_data_ptr = new fst_ctrl::PrRegDataIpc;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(fst_ctrl::PrRegDataIpc));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_PR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// SetHrReg
void ControllerServer::handleRequestSetHrReg()
{
    fst_ctrl::HrRegDataIpc* request_data_ptr = new fst_ctrl::HrRegDataIpc;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(fst_ctrl::HrRegDataIpc));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_HR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// SetMrReg
void ControllerServer::handleRequestSetMrReg()
{
    fst_ctrl::MrRegDataIpc* request_data_ptr = new fst_ctrl::MrRegDataIpc;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(fst_ctrl::MrRegDataIpc));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_MR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// SetSrReg
void ControllerServer::handleRequestSetSrReg()
{
    fst_ctrl::SrRegDataIpc* request_data_ptr = new fst_ctrl::SrRegDataIpc;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(fst_ctrl::SrRegDataIpc));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_SR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// SetRReg
void ControllerServer::handleRequestSetRReg()
{
    fst_ctrl::RRegDataIpc* request_data_ptr = new fst_ctrl::RRegDataIpc;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(fst_ctrl::RRegDataIpc));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_R_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// GetPrReg
void ControllerServer::handleRequestGetPrReg()
{
    int* request_data_ptr = new int;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    fst_ctrl::PrRegDataIpc* response_data_ptr = new fst_ctrl::PrRegDataIpc;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(int));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_PR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// GetHrReg
void ControllerServer::handleRequestGetHrReg()
{
    int* request_data_ptr = new int;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    fst_ctrl::HrRegDataIpc* response_data_ptr = new fst_ctrl::HrRegDataIpc;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(int));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_HR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// GetMrReg
void ControllerServer::handleRequestGetMrReg()
{
    int* request_data_ptr = new int;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    fst_ctrl::MrRegDataIpc* response_data_ptr = new fst_ctrl::MrRegDataIpc;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(int));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_MR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// GetSrReg
void ControllerServer::handleRequestGetSrReg()
{
    int* request_data_ptr = new int;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    fst_ctrl::SrRegDataIpc* response_data_ptr = new fst_ctrl::SrRegDataIpc;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(int));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_SR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// GetRReg
void ControllerServer::handleRequestGetRReg()
{
    int* request_data_ptr = new int;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    fst_ctrl::RRegDataIpc* response_data_ptr = new fst_ctrl::RRegDataIpc;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(int));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_R_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// SetInstruction
void ControllerServer::handleRequestSetInstruction()
{
    Instruction* request_data_ptr = new Instruction;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(Instruction));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_INSTRUCTION, (void*)request_data_ptr, (void*)response_data_ptr);
}

// IsNextInstructionNeeded
void ControllerServer::handleRequestIsNextInstructionNeeded()
{
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        return;
    }
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_IS_NEXT_INSTRUCTION_NEEDED, NULL, (void*)response_data_ptr);
}


