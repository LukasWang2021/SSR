#include "controller_client.h"
#include <nanomsg/ipc.h>
#include <nanomsg/reqrep.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/pipeline.h>
#include <cstring>
#include <iostream>
#include "process_comm_datatype.h"
#include "controller_server.h"
#include "error_code.h"


using namespace fst_base;

ControllerClient::ControllerClient(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr):
    log_ptr_(log_ptr), param_ptr_(param_ptr),
    recv_buffer_ptr_(NULL), send_buffer_ptr_(NULL)
{
    memset(&interpreter_publish_data_, 0, sizeof(InterpreterPublish));
}

ControllerClient::~ControllerClient()
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

ErrorCode ControllerClient::init()
{
    req_resp_socket_ = nn_socket(AF_SP, NN_REQ);
    if(req_resp_socket_ == -1) return PROCESS_COMM_CONTROLLER_CLIENT_INIT_FAILED;

    req_resp_endpoint_id_ = nn_connect(req_resp_socket_, param_ptr_->c2i_req_res_ip_.c_str());
    if(req_resp_endpoint_id_ == -1) return PROCESS_COMM_CONTROLLER_CLIENT_INIT_FAILED;

    sub_socket_ = nn_socket(AF_SP, NN_SUB);
    if(sub_socket_ == -1) return PROCESS_COMM_CONTROLLER_CLIENT_INIT_FAILED;

    sub_endpoint_id_ = nn_connect(sub_socket_, param_ptr_->c2i_pub_ip_.c_str());
    if(sub_endpoint_id_ == -1) return PROCESS_COMM_CONTROLLER_CLIENT_INIT_FAILED;

    event_socket_ = nn_socket(AF_SP, NN_PULL);
    if(event_socket_ == -1) return PROCESS_COMM_CONTROLLER_CLIENT_INIT_FAILED;

    event_endpoint_id_ = nn_connect(event_socket_, param_ptr_->c2i_event_ip_.c_str());
    if(event_endpoint_id_ == -1) return PROCESS_COMM_CONTROLLER_CLIENT_INIT_FAILED;

    poll_sub_fd_.fd = sub_socket_;
    poll_sub_fd_.events = NN_POLLIN;
    poll_event_fd_.fd = event_socket_;
    poll_event_fd_.events = NN_POLLIN;

    recv_buffer_ptr_ = new uint8_t[param_ptr_->recv_buffer_size_]();
    send_buffer_ptr_ = new uint8_t[param_ptr_->send_buffer_size_]();

    return SUCCESS;
}

bool ControllerClient::start(std::string data)
{
    if(!ControllerServer::isInterpreterServerReady()
        || data.size() == 0
        || data.size() >= 256
        || !sendRequest(INTERPRETER_SERVER_CMD_START, data.c_str(), 256)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_START)
    {
        return false;
    }

    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::debug(std::string data)
{
    if(!ControllerServer::isInterpreterServerReady()
        || data.size() == 0
        || data.size() >= 256
        || !sendRequest(INTERPRETER_SERVER_CMD_DEBUG, data.c_str(), 256)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_DEBUG)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::forward()
{
    if(!ControllerServer::isInterpreterServerReady()
        || !sendRequest(INTERPRETER_SERVER_CMD_FORWARD, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_FORWARD)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::backward()
{
    if(!ControllerServer::isInterpreterServerReady()
        || !sendRequest(INTERPRETER_SERVER_CMD_BACKWARD, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_BACKWARD)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::jump(int data)
{
    if(!ControllerServer::isInterpreterServerReady()
        || !sendRequest(INTERPRETER_SERVER_CMD_JUMP, &data, sizeof(int))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_JUMP)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::pause()
{
    if(!ControllerServer::isInterpreterServerReady()
        || !sendRequest(INTERPRETER_SERVER_CMD_PAUSE, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_PAUSE)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::resume()
{
    if(!ControllerServer::isInterpreterServerReady()
        || !sendRequest(INTERPRETER_SERVER_CMD_RESUME, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_RESUME)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::abort()
{
    if(!ControllerServer::isInterpreterServerReady()
        || !sendRequest(INTERPRETER_SERVER_CMD_ABORT, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_ABORT)
    {
        return false;
    }
    
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::getNextInstruction(Instruction* instruction_ptr)
{
    if(!ControllerServer::isInterpreterServerReady()
        || instruction_ptr == NULL
        || !sendRequest(INTERPRETER_SERVER_CMD_GET_NEXT_INSTRUCTION, instruction_ptr, sizeof(Instruction))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_GET_NEXT_INSTRUCTION)
    {
        return false;
    }

        
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool ControllerClient::setAutoStartMode(int start_mode)
{
    if(!ControllerServer::isInterpreterServerReady()
        || !sendRequest(INTERPRETER_SERVER_CMD_SET_AUTO_START_MODE, &start_mode, sizeof(int))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != INTERPRETER_SERVER_CMD_SET_AUTO_START_MODE)
    {
        return false;
    }

        
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

void ControllerClient::handleSubscribe()
{
    if(!ControllerServer::isInterpreterServerReady())
    {
        return;
    }
    
    if(nn_poll(&poll_sub_fd_, 1, 0) == -1)
    {
        return;
    }
    
    int recv_bytes = nn_recv(req_resp_socket_, recv_buffer_ptr_, param_ptr_->recv_buffer_size_, 0);
    if(recv_bytes == -1
        || recv_bytes != sizeof(InterpreterPublish))
    {
        return;
    }

    memcpy(&interpreter_publish_data_, recv_buffer_ptr_, sizeof(InterpreterPublish));
}

void ControllerClient::handleEvent()
{
    if(!ControllerServer::isInterpreterServerReady())
    {
        return;
    }

    if(nn_poll(&poll_event_fd_, 1, 0) == -1)
    {
		FST_ERROR("nn_poll failed");
        return;
    }
    int recv_bytes = nn_recv(event_socket_, recv_buffer_ptr_, param_ptr_->recv_buffer_size_, 0);
    if(recv_bytes == -1
        || recv_bytes != sizeof(ProcessCommEvent))
    {
		FST_ERROR("nn_recv failed = %d", recv_bytes);
        return;
    }    

    FST_ERROR("recv event from interpreter: error_code = %llx", ((ProcessCommEvent*)recv_buffer_ptr_)->data);
}

InterpreterPublish* ControllerClient::getInterpreterPublishPtr()
{
    return &interpreter_publish_data_;
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

