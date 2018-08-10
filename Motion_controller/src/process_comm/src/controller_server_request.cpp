#include "controller_server.h"
#include "reg_manager.h"


using namespace fst_base;
using namespace fst_ctrl;


// SetPrReg
void ControllerServer::handleRequestSetPrReg()
{
    fst_ctrl::PrRegData* request_data_ptr = new fst_ctrl::PrRegData;
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
    copyRecvBufferToRequestData(request_data_ptr, sizeof(fst_ctrl::PrRegData));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_PR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// SetHrReg
void ControllerServer::handleRequestSetHrReg()
{
    fst_ctrl::HrRegData* request_data_ptr = new fst_ctrl::HrRegData;
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
    copyRecvBufferToRequestData(request_data_ptr, sizeof(fst_ctrl::HrRegData));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_HR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// SetMrReg
void ControllerServer::handleRequestSetMrReg()
{
    fst_ctrl::MrRegData* request_data_ptr = new fst_ctrl::MrRegData;
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
    copyRecvBufferToRequestData(request_data_ptr, sizeof(fst_ctrl::MrRegData));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_MR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// SetSrReg
void ControllerServer::handleRequestSetSrReg()
{
    fst_ctrl::SrRegData* request_data_ptr = new fst_ctrl::SrRegData;
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
    copyRecvBufferToRequestData(request_data_ptr, sizeof(fst_ctrl::SrRegData));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_SET_SR_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}

// SetRReg
void ControllerServer::handleRequestSetRReg()
{
    fst_ctrl::RRegData* request_data_ptr = new fst_ctrl::RRegData;
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
    copyRecvBufferToRequestData(request_data_ptr, sizeof(fst_ctrl::RRegData));
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
    fst_ctrl::PrRegData* response_data_ptr = new fst_ctrl::PrRegData;
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
    fst_ctrl::PrRegData* response_data_ptr = new fst_ctrl::PrRegData;
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
    fst_ctrl::MrRegData* response_data_ptr = new fst_ctrl::MrRegData;
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
    fst_ctrl::SrRegData* response_data_ptr = new fst_ctrl::SrRegData;
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
    fst_ctrl::RRegData* response_data_ptr = new fst_ctrl::RRegData;
    if(response_data_ptr == NULL)
    {
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    copyRecvBufferToRequestData(request_data_ptr, sizeof(int));
    pushTaskToRequestList(CONTROLLER_SERVER_CMD_GET_R_REG, (void*)request_data_ptr, (void*)response_data_ptr);
}


