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


// SetInterpreterServerStatus
void ControllerServer::handleRequestSetInterpreterServerStatus()
{
    bool* request_data_ptr = new bool;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    bool* response_data_ptr = new bool;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete[] request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(bool));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_INTERPRETER_SERVER_STATUS, (void*)request_data_ptr, (void*)response_data_ptr);  
}

//GetDi
void ControllerServer::handleRequestGetDi()
{
    RequestGetDi* request_data_ptr = new RequestGetDi;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseGetDi* response_data_ptr = new ResponseGetDi;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestGetDi));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_DI, (void*)request_data_ptr, (void*)response_data_ptr); 
}

//SetDi
void ControllerServer::handleRequestSetDi()
{
    RequestSetDi* request_data_ptr = new RequestSetDi;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    unsigned long long* response_data_ptr = new unsigned long long;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestSetDi));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_DI, (void*)request_data_ptr, (void*)response_data_ptr); 
}

//GetDo
void ControllerServer::handleRequestGetDo()
{
    RequestGetDo* request_data_ptr = new RequestGetDo;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseGetDo* response_data_ptr = new ResponseGetDo;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestGetDo));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_DO, (void*)request_data_ptr, (void*)response_data_ptr); 
}

//SetDo
void ControllerServer::handleRequestSetDo()
{
    RequestSetDo* request_data_ptr = new RequestSetDo;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    unsigned long long* response_data_ptr = new unsigned long long;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestSetDo));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_DO, (void*)request_data_ptr, (void*)response_data_ptr); 
}

//GetRi
void ControllerServer::handleRequestGetRi()
{
    RequestGetRi* request_data_ptr = new RequestGetRi;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseGetRi* response_data_ptr = new ResponseGetRi;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestGetRi));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_RI, (void*)request_data_ptr, (void*)response_data_ptr); 

}

//SetRi
void ControllerServer::handleRequestSetRi()
{
    RequestSetRi* request_data_ptr = new RequestSetRi;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    unsigned long long* response_data_ptr = new unsigned long long;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestSetRi));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_RI, (void*)request_data_ptr, (void*)response_data_ptr);
}

//GetRo
void ControllerServer::handleRequestGetRo()
{
    RequestGetRo* request_data_ptr = new RequestGetRo;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseGetRo* response_data_ptr = new ResponseGetRo;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestGetRo));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_RO, (void*)request_data_ptr, (void*)response_data_ptr); 
}

//SetRo
void ControllerServer::handleRequestSetRo()
{
    RequestSetRo* request_data_ptr = new RequestSetRo;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    unsigned long long* response_data_ptr = new unsigned long long;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestSetRo));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_RO, (void*)request_data_ptr, (void*)response_data_ptr);
}

//GetUi
void ControllerServer::handleRequestGetUi()
{
    RequestGetUi* request_data_ptr = new RequestGetUi;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseGetUi* response_data_ptr = new ResponseGetUi;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestGetUi));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_UI, (void*)request_data_ptr, (void*)response_data_ptr); 
}

//SetUi
void ControllerServer::handleRequestSetUi()
{
    RequestSetUi* request_data_ptr = new RequestSetUi;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    unsigned long long* response_data_ptr = new unsigned long long;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestSetUi));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_UI, (void*)request_data_ptr, (void*)response_data_ptr); 
}

//GetUo
void ControllerServer::handleRequestGetUo()
{
    RequestGetUo* request_data_ptr = new RequestGetUo;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseGetUo* response_data_ptr = new ResponseGetUo;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestGetUo));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_UO, (void*)request_data_ptr, (void*)response_data_ptr); 
}

//SetUo
void ControllerServer::handleRequestSetUo()
{
    RequestSetUo* request_data_ptr = new RequestSetUo;
    if(request_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    unsigned long long* response_data_ptr = new unsigned long long;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(RequestSetUo));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_UO, (void*)request_data_ptr, (void*)response_data_ptr); 
}