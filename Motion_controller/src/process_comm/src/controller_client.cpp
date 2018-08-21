#include "controller_client.h"
#include <nanomsg/ipc.h>
#include <nanomsg/reqrep.h>
#include <cstring>
#include <iostream>
#include "process_comm_datatype.h"


using namespace fst_base;

ControllerClient::ControllerClient(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr):
    log_ptr_(log_ptr), param_ptr_(param_ptr)
{

}

ControllerClient::~ControllerClient()
{

}

bool ControllerClient::init()
{
    req_resp_socket_ = nn_socket(AF_SP, NN_REQ);
    if(req_resp_socket_ == -1) return false;

    req_resp_endpoint_id_ = nn_connect(req_resp_socket_, param_ptr_->c2i_req_res_ip_.c_str());
    if(req_resp_endpoint_id_ == -1) return false;

    recv_buffer_ptr_ = new uint8_t[param_ptr_->recv_buffer_size_]();
    send_buffer_ptr_ = new uint8_t[param_ptr_->send_buffer_size_]();

    return true;
}

bool ControllerClient::start(std::string data)
{
    if(data.size() == 0
        || !sendRequest(INTERPRETER_SERVER_CMD_START, data.c_str(), data.size() + 1)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_START)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::debug(std::string data)
{
    if(data.size() == 0
        || !sendRequest(INTERPRETER_SERVER_CMD_DEBUG, data.c_str(), data.size() + 1)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_DEBUG)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::forward()
{
    if(!sendRequest(INTERPRETER_SERVER_CMD_FORWARD, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_FORWARD)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::backward()
{
    if(!sendRequest(INTERPRETER_SERVER_CMD_BACKWARD, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_BACKWARD)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::jump(int data)
{
    if(!sendRequest(INTERPRETER_SERVER_CMD_JUMP, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_JUMP)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::pause()
{
    if(!sendRequest(INTERPRETER_SERVER_CMD_PAUSE, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_PAUSE)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::resume()
{
    if(!sendRequest(INTERPRETER_SERVER_CMD_RESUME, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_RESUME)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::abort()
{
    if(!sendRequest(INTERPRETER_SERVER_CMD_ABORT, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_ABORT)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::getNextInstruction(Instruction* instruction_ptr)
{
    if(instruction_ptr == NULL
        || !sendRequest(INTERPRETER_SERVER_CMD_GET_NEXT_INSTRUCTION, instruction_ptr, sizeof(Instruction))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_GET_NEXT_INSTRUCTION)
    {
        return false;
    }

        
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::sendRequest(unsigned int cmd_id, void* data_ptr, int send_size)
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

bool ControllerClient::recvResponse(int expect_recv_size)
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

