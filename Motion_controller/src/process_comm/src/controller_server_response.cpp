#include "controller_server.h"
#include "reg_manager.h"
#include <unistd.h>

using namespace fst_base;
using namespace fst_ctrl;

// SetPrReg
void ControllerServer::handleResponseSetPrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_PR_REG, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (PrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// SetHrReg
void ControllerServer::handleResponseSetHrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_HR_REG, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (HrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// SetMrReg
void ControllerServer::handleResponseSetMrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_MR_REG, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (MrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// SetSrReg
void ControllerServer::handleResponseSetSrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_SR_REG, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (SrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// SetRReg
void ControllerServer::handleResponseSetRReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_R_REG, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// GetPrReg
void ControllerServer::handleResponseGetPrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_PR_REG, task->response_data_ptr, sizeof(PrRegData), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (PrRegData*)task->response_data_ptr;
    }
}

// GetHrReg
void ControllerServer::handleResponseGetHrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_HR_REG, task->response_data_ptr, sizeof(HrRegData), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (HrRegData*)task->response_data_ptr;
    }
}

// GetMrReg
void ControllerServer::handleResponseGetMrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_MR_REG, task->response_data_ptr, sizeof(MrRegData), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (MrRegData*)task->response_data_ptr;
    }
}

// GetSrReg
void ControllerServer::handleResponseGetSrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_SR_REG, task->response_data_ptr, sizeof(SrRegData), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (SrRegData*)task->response_data_ptr;
    }
}

// GetRReg
void ControllerServer::handleResponseGetRReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_R_REG, task->response_data_ptr, sizeof(RRegData), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (RRegData*)task->response_data_ptr;
    }
}


