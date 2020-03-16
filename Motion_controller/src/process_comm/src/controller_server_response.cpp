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

// SetMi
void ControllerServer::handleResponseSetMi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_MI, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (MiDataIpc*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (bool*)task->response_data_ptr;
    }
}

// SetMh
void ControllerServer::handleResponseSetMh(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_MH, task->response_data_ptr, sizeof(bool), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (MhDataIpc*)task->request_data_ptr;
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

// GetMi
void ControllerServer::handleResponseGetMi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_MI, task->response_data_ptr, sizeof(MiDataIpc), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (MiDataIpc*)task->response_data_ptr;
    }
}

// GetMh
void ControllerServer::handleResponseGetMh(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)        
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_MH, task->response_data_ptr, sizeof(MhDataIpc), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (MhDataIpc*)task->response_data_ptr;
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


//GetUi
void ControllerServer::handleResponseGetUi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_UI, task->response_data_ptr, sizeof(ResponseGetUi), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestGetUi*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseGetUi*)task->response_data_ptr;
    }
}

//SetUi
void ControllerServer::handleResponseSetUi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_UI, task->response_data_ptr, sizeof(unsigned long long), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestSetUi*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (unsigned long long*)task->response_data_ptr;
    }
}

//GetUo
void ControllerServer::handleResponseGetUo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_UO, task->response_data_ptr, sizeof(ResponseGetUo), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestGetUo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseGetUo*)task->response_data_ptr;
    }
}

//GetJoint
void ControllerServer::handleResponseGetJoint(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_JOINT, task->response_data_ptr, sizeof(Joint), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (Joint*)task->response_data_ptr;
    }
}

//GetCart
void ControllerServer::handleResponseGetCart(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_CART, task->response_data_ptr, sizeof(PoseEuler), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (int*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (PoseEuler*)task->response_data_ptr;
    }
}

//CartToJoint
void ControllerServer::handleResponseCartToJoint(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_CART_TO_JOINT, task->response_data_ptr, sizeof(Joint), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (PoseEuler*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (Joint*)task->response_data_ptr;
    }
}

//JointToCart
void ControllerServer::handleResponseJointToCart(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_JOINT_TO_CART, task->response_data_ptr, sizeof(PoseEuler), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (Joint*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (PoseEuler*)task->response_data_ptr;
    }
}

//UserOpMode
void ControllerServer::handleResponseUserOpMode(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_OP_MODE, task->response_data_ptr, sizeof(int), send_buffer_size);
    if(task->response_data_ptr != NULL)
    {
        delete (int*)task->response_data_ptr;
    }
}

//SetDoPulse
void ControllerServer::handleResponseSetDoPulse(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_DO_PULSE, task->response_data_ptr, sizeof(unsigned long long), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestSetPulse*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (unsigned long long*)task->response_data_ptr;
    }
}

//SetRoPulse
void ControllerServer::handleResponseSetRoPulse(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_SET_RO_PULSE, task->response_data_ptr, sizeof(unsigned long long), send_buffer_size);
    if(task->request_data_ptr != NULL)
    {
        delete (RequestSetPulse*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (unsigned long long*)task->response_data_ptr;
    }

}

//getPosture
void ControllerServer::handleResponseGetPosture(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_POSTURE, task->response_data_ptr, sizeof(Posture), send_buffer_size);
    if(task->response_data_ptr != NULL)
    {
        delete (Posture*)task->response_data_ptr;
    }
}

//getTurn
void ControllerServer::handleResponseGetTurn(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size)
{
    copyResponseDataToSendBuffer(CONTROLLER_SERVER_CMD_GET_TURN, task->response_data_ptr, sizeof(Turn), send_buffer_size);
    if(task->response_data_ptr != NULL)
    {
        delete (Turn*)task->response_data_ptr;
    }
}