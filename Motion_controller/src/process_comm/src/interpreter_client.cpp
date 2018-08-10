#include "interpreter_client.h"
#include <nanomsg/ipc.h>
#include <nanomsg/reqrep.h>
#include <cstring>
#include <iostream>
#include "process_comm_datatype.h"

using namespace fst_base;
using namespace fst_ctrl;

InterpreterClient::InterpreterClient(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr):
    log_ptr_(log_ptr), param_ptr_(param_ptr)
{

}

InterpreterClient::~InterpreterClient()
{

}

bool InterpreterClient::init()
{
    req_resp_socket_ = nn_socket(AF_SP, NN_REQ);
    if(req_resp_socket_ == -1) return false;

    req_resp_endpoint_id_ = nn_connect(req_resp_socket_, param_ptr_->i2c_req_res_ip_.c_str());
    if(req_resp_endpoint_id_ == -1) return false;

    recv_buffer_ptr_ = new uint8_t[param_ptr_->recv_buffer_size_]();
    send_buffer_ptr_ = new uint8_t[param_ptr_->send_buffer_size_]();

    return true;
}

bool InterpreterClient::setPrReg(PrRegData* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_PR_REG, data, sizeof(PrRegData))
        || !recvResponse(sizeof(bool)))
    {
        return false;
    }
    
    return *((bool*)recv_buffer_ptr_);
}

bool InterpreterClient::setHrReg(HrRegData* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_HR_REG, data, sizeof(HrRegData))
        || !recvResponse(sizeof(bool)))
    {
        return false;
    }
    
    return *((bool*)recv_buffer_ptr_);
}

bool InterpreterClient::setMrReg(MrRegData* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_MR_REG, data, sizeof(MrRegData))
        || !recvResponse(sizeof(bool)))
    {
        return false;
    }
    
    return *((bool*)recv_buffer_ptr_);
}

bool InterpreterClient::setSrReg(SrRegData* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_SR_REG, data, sizeof(SrRegData))
        || !recvResponse(sizeof(bool)))
    {
        return false;
    }
    
    return *((bool*)recv_buffer_ptr_);
}

bool InterpreterClient::setRReg(RRegData* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_R_REG, data, sizeof(RRegData))
        || !recvResponse(sizeof(bool)))
    {
        return false;
    }
    
    return *((bool*)recv_buffer_ptr_);
}

bool InterpreterClient::getPrReg(int id, PrRegData* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_PR_REG, id, sizeof(int))
        || !recvResponse(sizeof(PrRegData)))
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_, sizeof(PrRegData));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getHrReg(int id, HrRegData* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_HR_REG, id, sizeof(int))
        || !recvResponse(sizeof(HrRegData)))
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_, sizeof(HrRegData));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getMrReg(int id, MrRegData* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_MR_REG, id, sizeof(int))
        || !recvResponse(sizeof(MrRegData)))
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_, sizeof(MrRegData));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getSrReg(int id, SrRegData* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_SR_REG, id, sizeof(int))
        || !recvResponse(sizeof(SrRegData)))
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_, sizeof(SrRegData));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getRReg(int id, RRegData* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_R_REG, id, sizeof(int))
        || !recvResponse(sizeof(RRegData)))
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_, sizeof(RRegData));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::sendRequest(unsigned int cmd_id, void* data_ptr, int send_size)
{
    *((unsigned int*)send_buffer_ptr_) = cmd_id;
    if(data_ptr != NULL && send_size != 0)
    {
        memcpy(send_buffer_ptr_ + 4, data_ptr, send_size);
    }
    int send_bytes = nn_send(req_resp_socket_, send_buffer_ptr_, send_size + 4, 0); // block send
    if(send_bytes == -1 || send_bytes != (send_size + 4))
    {
        FST_ERROR("handleResponseList: send response failed, nn_error = %d", nn_errno());
    }
}

bool InterpreterClient::recvResponse(int expect_recv_size)
{
    int recv_size = nn_recv(req_resp_socket_, recv_buffer_ptr_, param_ptr_->recv_buffer_size_, 0);
    if(recv_size == -1 
        || recv_size != expect_recv_size)
    {
        return false;
    }
    else
    {
        return true;
    }
}

