#include "interpreter_server.h"
#include <nanomsg/ipc.h>
#include <nanomsg/reqrep.h>
#include <cstring>
#include <iostream>
#include <unistd.h>

using namespace std;
using namespace fst_base;

InterpreterServer::InterpreterServer(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr):
    log_ptr_(log_ptr), param_ptr_(param_ptr), is_exit_(false)
{

}

InterpreterServer::~InterpreterServer()
{
    this->close();
}

bool InterpreterServer::init()
{
    req_resp_socket_ = nn_socket(AF_SP, NN_REP);
    if(req_resp_socket_ == -1) return false;

    req_resp_endpoint_id_ = nn_bind(req_resp_socket_, param_ptr_->c2i_req_res_ip_.c_str());
    if(req_resp_endpoint_id_ == -1) return false;

    // it is critical to set poll fd here to make the model work correctly
	poll_fd_.fd = req_resp_socket_;
	poll_fd_.events = NN_POLLIN | NN_POLLOUT;

    recv_buffer_ptr_ = new uint8_t[param_ptr_->recv_buffer_size_]();
    send_buffer_ptr_ = new uint8_t[param_ptr_->send_buffer_size_]();

    initRpcTable();
    return true;
}

bool InterpreterServer::isExit()
{
    return is_exit_;
}

bool InterpreterServer::open()
{
    is_exit_ = false;
    if(!thread_.run(&interpreterServerThreadFunc, this, param_ptr_->interpreter_server_thread_priority_))
    {
        return false;
    }
    else
    {
        return true;
    }
}

void InterpreterServer::close()
{
    is_exit_ = true;
    thread_.join();

    if(req_resp_socket_ != -1 && req_resp_endpoint_id_ != -1)
    {
        nn_shutdown(req_resp_socket_, req_resp_endpoint_id_);
    }
}

std::vector<ProcessCommRequestResponse> InterpreterServer::popTaskFromRequestList()
{
    std::vector<ProcessCommRequestResponse> request_list;
    std::vector<ProcessCommRequestResponse>::iterator it;
    request_list_mutex_.lock();
    for(it = request_list_.begin(); it != request_list_.end(); ++it)
    {
        request_list.push_back(*it);
    }
    request_list_.clear();
    request_list_mutex_.unlock();
    return request_list;
}

void InterpreterServer::pushTaskToResponseList(ProcessCommRequestResponse& package)
{
    response_list_mutex_.lock();
    response_list_.push_back(package);
    response_list_mutex_.unlock();
}

void InterpreterServer::runThreadFunc()
{
    handleRequestList();
    handleResponseList();
    usleep(param_ptr_->interpreter_server_cycle_time_);
}

InterpreterServer::InterpreterServer():
    log_ptr_(NULL), param_ptr_(NULL), is_exit_(false)
{

}

void InterpreterServer::handleRequestList()
{
    if(nn_poll (&poll_fd_, 1, 0) == -1)
    {
        return;
    }
    
    int recv_bytes;
    if(poll_fd_.revents & NN_POLLIN)
    {
        recv_bytes = nn_recv(req_resp_socket_, recv_buffer_ptr_, param_ptr_->recv_buffer_size_, 0);
        if(recv_bytes == -1)
        {
            return;
        }
    }
    else
    {
        return;
    }

    unsigned int cmd_id = *((unsigned int*)recv_buffer_ptr_);
    HandleRequestFuncPtr func_ptr = rpc_table_[cmd_id].request_func_ptr;
    if(func_ptr != NULL)
    {
        (this->*func_ptr)();
    }
}

void InterpreterServer::handleResponseList()
{
    int send_buffer_size;
    std::vector<ProcessCommRequestResponse>::iterator it;
    response_list_mutex_.lock();
    for(it = response_list_.begin(); it != response_list_.end(); ++it)
    {
        if(it->cmd_id < rpc_table_.size())
        {
            HandleResponseFuncPtr func_ptr = rpc_table_[it->cmd_id].response_func_ptr;
            if(func_ptr != NULL)
            {
                (this->*func_ptr)(it, send_buffer_size);
            }
            else
            {
                return;
            }
        }
        else
        {
            return;
        }

        int send_bytes = nn_send(req_resp_socket_, send_buffer_ptr_, send_buffer_size, 0); // block send
        if(send_bytes == -1)
        {
            FST_ERROR("handleResponseList: send response failed, nn_error = %d", nn_errno());
        }
    }
    response_list_.clear();
    response_list_mutex_.unlock();
}

void InterpreterServer::pushTaskToRequestList(unsigned int cmd_id, void* request_data_ptr, void* response_data_ptr)
{
    ProcessCommRequestResponse package;
    package.cmd_id = cmd_id;
    package.request_data_ptr = request_data_ptr;
    package.response_data_ptr = response_data_ptr;
    request_list_mutex_.lock();
    request_list_.push_back(package);
    request_list_mutex_.unlock();
}

void InterpreterServer::copyRecvBufferToRequestData(void* request_data_ptr, int size)
{
    memcpy(request_data_ptr, recv_buffer_ptr_ + 4, size);
}

void InterpreterServer::copyResponseDataToSendBuffer(InterpreterServerCmd cmd_id, void* response_data_ptr, 
                                                            int response_data_size, int& send_buffer_size)
{
    *((unsigned int*)send_buffer_ptr_) = cmd_id;
    memcpy(send_buffer_ptr_ + 4, response_data_ptr, response_data_size);
    send_buffer_size = sizeof(bool) + 4;
}

void interpreterServerThreadFunc(void* arg)
{
    std::cout<<"---interpreterServerThreadFunc running"<<std::endl;
    InterpreterServer* interpreter_server_ptr = static_cast<InterpreterServer*>(arg);
    while(!interpreter_server_ptr->isExit())
    {
        interpreter_server_ptr->runThreadFunc();
    }
}




