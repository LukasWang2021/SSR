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
        delete (PrRegDataIpc*)task->request_data_ptr;
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
        delete (HrRegDataIpc*)task->request_data_ptr;
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
        delete (MrRegDataIpc*)task->request_data_ptr;
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
        delete (SrRegDataIpc*)task->request_data_ptr;
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
        delete (fst_ctrl::RRegDataIpc*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// GetPrReg
void ControllerServer::handleResponseGetPrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_PR_REG, task->response_data_ptr, sizeof(PrRegDataIpc), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (PrRegDataIpc*)task->response_data_ptr;
    }
}

// GetHrReg
void ControllerServer::handleResponseGetHrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_HR_REG, task->response_data_ptr, sizeof(HrRegDataIpc), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (HrRegDataIpc*)task->response_data_ptr;
    }
}

// GetMrReg
void ControllerServer::handleResponseGetMrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_MR_REG, task->response_data_ptr, sizeof(MrRegDataIpc), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (MrRegDataIpc*)task->response_data_ptr;
    }
}

// GetSrReg
void ControllerServer::handleResponseGetSrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_SR_REG, task->response_data_ptr, sizeof(SrRegDataIpc), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (SrRegDataIpc*)task->response_data_ptr;
    }
}

// GetRReg
void ControllerServer::handleResponseGetRReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_R_REG, task->response_data_ptr, sizeof(RRegDataIpc), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (RRegDataIpc*)task->response_data_ptr;
    }
}

// SetInstruction
void ControllerServer::handleResponseSetInstruction(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_INSTRUCTION, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (Instruction*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// IsNextInstructionNeeded
void ControllerServer::handleResponseIsNextInstructionNeeded(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_IS_NEXT_INSTRUCTION_NEEDED, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

/*todo delete
// CheckIo
void ControllerServer::handleResponseCheckIo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_CHECK_IO, task->response_data_ptr, sizeof(ResponseCheckIo), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete[] (char*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseCheckIo*)task->response_data_ptr;
    }
}

// SetIo
void ControllerServer::handleResponseSetIo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_IO, task->response_data_ptr, sizeof(unsigned long long), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestSetIo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (unsigned long long*)task->response_data_ptr;
    }
}

// GetIo
void ControllerServer::handleResponseGetIo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_IO, task->response_data_ptr, sizeof(ResponseGetIo), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestGetIo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseGetIo*)task->response_data_ptr;
    }
}
*/

// SetInterpreterServerStatus
void ControllerServer::handleResponseSetInterpreterServerStatus(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_INTERPRETER_SERVER_STATUS, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (bool*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

//GetDi
void ControllerServer::handleResponseGetDi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_DI, task->response_data_ptr, sizeof(ResponseGetDi), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestGetDi*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseGetDi*)task->response_data_ptr;
    }
}

//SetDi
void ControllerServer::handleResponseSetDi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_DI, task->response_data_ptr, sizeof(unsigned long long), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestSetDi*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (unsigned long long*)task->response_data_ptr;
    }
}

//GetDo
void ControllerServer::handleResponseGetDo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_DO, task->response_data_ptr, sizeof(ResponseGetDo), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestGetDo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseGetDo*)task->response_data_ptr;
    }
}

//SetDo
void ControllerServer::handleResponseSetDo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_DO, task->response_data_ptr, sizeof(unsigned long long), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestSetDo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (unsigned long long*)task->response_data_ptr;
    }

}

//GetRi
void ControllerServer::handleResponseGetRi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_RI, task->response_data_ptr, sizeof(ResponseGetRi), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestGetRi*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseGetRi*)task->response_data_ptr;
    }
}

//SetRi
void ControllerServer::handleResponseSetRi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_RI, task->response_data_ptr, sizeof(unsigned long long), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestSetRi*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (unsigned long long*)task->response_data_ptr;
    }
}

//GetRo
void ControllerServer::handleResponseGetRo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_RO, task->response_data_ptr, sizeof(ResponseGetRo), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestGetRo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseGetRo*)task->response_data_ptr;
    }
}

//SetRo
void ControllerServer::handleResponseSetRo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_RO, task->response_data_ptr, sizeof(unsigned long long), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestSetRo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (unsigned long long*)task->response_data_ptr;
    }

}
