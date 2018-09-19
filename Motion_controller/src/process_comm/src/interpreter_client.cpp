#include "interpreter_client.h"
#include <nanomsg/ipc.h>
#include <nanomsg/reqrep.h>
#include <cstring>
#include <iostream>
#include "process_comm_datatype.h"
#include "error_code.h"


using namespace fst_base;
using namespace fst_ctrl;

InterpreterClient::InterpreterClient(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr):
    log_ptr_(log_ptr), param_ptr_(param_ptr),
    recv_buffer_ptr_(NULL), send_buffer_ptr_(NULL)
{

}

InterpreterClient::~InterpreterClient()
{
    if(recv_buffer_ptr_ != NULL)
    {
        delete[] recv_buffer_ptr_;
        recv_buffer_ptr_ = NULL;
    }
    
    if(send_buffer_ptr_ != NULL)
    {
        delete[] send_buffer_ptr_;
        send_buffer_ptr_ = NULL;
    }
}

ErrorCode InterpreterClient::init()
{
    req_resp_socket_ = nn_socket(AF_SP, NN_REQ);
    if(req_resp_socket_ == -1) return PROCESS_COMM_INTERPRETER_CLIENT_INIT_FAILED;

    req_resp_endpoint_id_ = nn_connect(req_resp_socket_, param_ptr_->i2c_req_res_ip_.c_str());
    if(req_resp_endpoint_id_ == -1) return PROCESS_COMM_INTERPRETER_CLIENT_INIT_FAILED;

    recv_buffer_ptr_ = new uint8_t[param_ptr_->recv_buffer_size_]();
    send_buffer_ptr_ = new uint8_t[param_ptr_->send_buffer_size_]();

    return SUCCESS;
}

bool InterpreterClient::setPrReg(PrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_PR_REG, data, sizeof(PrRegDataIpc))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_PR_REG)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::setHrReg(HrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_HR_REG, data, sizeof(HrRegDataIpc))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_HR_REG)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::setMrReg(MrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_MR_REG, data, sizeof(MrRegDataIpc))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_MR_REG)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::setSrReg(SrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_SR_REG, data, sizeof(SrRegDataIpc))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_SR_REG)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::setRReg(RRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_R_REG, data, sizeof(RRegDataIpc))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_R_REG)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::getPrReg(int id, PrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_PR_REG, &id, sizeof(int))
        || !recvResponse(sizeof(PrRegDataIpc))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_PR_REG)
    {
        return false;
    }
    
    memcpy(data, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(PrRegDataIpc));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getHrReg(int id, HrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_HR_REG, &id, sizeof(int))
        || !recvResponse(sizeof(HrRegDataIpc))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_HR_REG)
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(HrRegDataIpc));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getMrReg(int id, MrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_MR_REG, &id, sizeof(int))
        || !recvResponse(sizeof(MrRegDataIpc))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_MR_REG)
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(MrRegDataIpc));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getSrReg(int id, SrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_SR_REG, &id, sizeof(int))
        || !recvResponse(sizeof(SrRegDataIpc))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_SR_REG)
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(SrRegDataIpc));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getRReg(int id, RRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_R_REG, &id, sizeof(int))
        || !recvResponse(sizeof(RRegDataIpc))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_R_REG)
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(RRegDataIpc));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::setInstruction(Instruction* data)
{   
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_INSTRUCTION, data, sizeof(Instruction))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_INSTRUCTION)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::isNextInstructionNeeded()
{
    if(!sendRequest(CONTROLLER_SERVER_CMD_IS_NEXT_INSTRUCTION_NEEDED, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_IS_NEXT_INSTRUCTION_NEEDED)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

ErrorCode InterpreterClient::checkIo(char path[256], IOPortInfo* port_info_ptr)
{
    if(strlen(path) >= 256
        || port_info_ptr == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_CHECK_IO, (void*)path, 256)
        || !recvResponse(sizeof(ResponseCheckIo))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_CHECK_IO)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }

    ResponseCheckIo response_check_io;
    memcpy(&response_check_io, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(ResponseCheckIo));
    memcpy(port_info_ptr, &response_check_io.port_info, sizeof(IOPortInfo));
    return response_check_io.error_code;
}

ErrorCode InterpreterClient::setIo(IOPortInfo* port_info_ptr, char value)
{
    if(port_info_ptr == NULL)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    RequestSetIo request_set_io;
    memcpy(&request_set_io.port_info, port_info_ptr, sizeof(IOPortInfo));
    request_set_io.value = value;
    if(!sendRequest(CONTROLLER_SERVER_CMD_SET_IO, (void*)&request_set_io, sizeof(RequestSetIo))
        || !recvResponse(sizeof(unsigned long long))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_IO)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    return *((unsigned long long*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

ErrorCode InterpreterClient::getIo(IOPortInfo* port_info_ptr, int buffer_length, char* value_ptr)
{
    if(port_info_ptr == NULL)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    RequestGetIo request_get_io;
    memcpy(&request_get_io.port_info, port_info_ptr, sizeof(IOPortInfo));
    request_get_io.buffer_length = buffer_length;
    if(!sendRequest(CONTROLLER_SERVER_CMD_GET_IO, (void*)&request_get_io, sizeof(RequestGetIo))
        || !recvResponse(sizeof(ResponseGetIo))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_IO)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    ResponseGetIo response_get_io;
    memcpy(&response_get_io, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(ResponseGetIo));
    *value_ptr = response_get_io.value;
    return response_get_io.error_code;
}

bool InterpreterClient::setInterpreterServerStatus(bool status)
{
    if(!sendRequest(CONTROLLER_SERVER_CMD_SET_INTERPRETER_SERVER_STATUS, &status, sizeof(bool))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_INTERPRETER_SERVER_STATUS)
    {
        return false;
    }

    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::sendRequest(unsigned int cmd_id, void* data_ptr, int send_size)
{
    *((unsigned int*)send_buffer_ptr_) = cmd_id;
    if(data_ptr != NULL && send_size != 0)
    {
        memcpy(send_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, data_ptr, send_size);
    }
    int send_bytes = nn_send(req_resp_socket_, send_buffer_ptr_, send_size + PROCESS_COMM_CMD_ID_SIZE, 0); // block send
    if(send_bytes == -1 || send_bytes != (send_size + PROCESS_COMM_CMD_ID_SIZE))
    {
        FST_ERROR("handleResponseList: send response failed, nn_error = %d", nn_errno());
        return false;
    }
    return true;
}

bool InterpreterClient::recvResponse(int expect_recv_size)
{
    int recv_size = nn_recv(req_resp_socket_, recv_buffer_ptr_, param_ptr_->recv_buffer_size_, 0);
    if(recv_size == -1 
        || recv_size != (expect_recv_size + PROCESS_COMM_CMD_ID_SIZE))
    {
        return false;
    }
    else
    {
        return true;
    }
}

