#include "interpreter_server.h"
#include <nanomsg/ipc.h>
#include <nanomsg/reqrep.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/pipeline.h>
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

    publish_socket_ = nn_socket(AF_SP, NN_PUB);
    if(publish_socket_ == -1) return false;

    publish_endpoint_id_ = nn_bind(publish_socket_, param_ptr_->c2i_pub_ip_.c_str());
    if(publish_endpoint_id_ == -1) return false;

    event_socket_ = nn_socket(AF_SP, NN_PUSH);
    if(publish_socket_ == -1) return false;

    event_endpoint_id_ = nn_bind(event_socket_, param_ptr_->c2i_event_ip_.c_str());
    if(event_endpoint_id_ == -1) return false;


    // it is critical to set poll fd here to make the model work correctly
	poll_req_res_fd_.fd = req_resp_socket_;
	poll_req_res_fd_.events = NN_POLLIN | NN_POLLOUT;
    poll_event_fd_.fd = event_socket_;
    poll_event_fd_.events = NN_POLLOUT;

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

    if(publish_socket_ != -1 && publish_endpoint_id_ != -1)
    {
        nn_shutdown(publish_socket_, publish_endpoint_id_);
    }    
}

void InterpreterServer::runThreadFunc()
{
    handleRequestList();
    handleResponseList();
    handlePublishList();
    handleEventList();
    usleep(param_ptr_->interpreter_server_cycle_time_);
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

bool InterpreterServer::addPublishTask(int interval, InterpreterPublish* data_ptr)
{
    if(interval <= 0
        || data_ptr == NULL)
    {
        return false;
    }

    ProcessCommPublish task;
    task.interval = interval;
    task.data_ptr = data_ptr;
    task.last_publish_time.tv_sec = 0;
    task.last_publish_time.tv_usec = 0;
    publish_list_mutex_.lock();
    publish_list_.push_back(task);
    publish_list_mutex_.unlock();
}

void InterpreterServer::removePublishTask()
{
    publish_list_mutex_.lock();
    publish_list_.clear();
    publish_list_mutex_.unlock();
}

void InterpreterServer::sendEvent(int event_type, void* data_ptr)
{
    ProcessCommEvent event;
    event.event_type = event_type;
    event.data = *((unsigned long long int*)data_ptr);
    event_list_mutex_.lock();
    if(event_list_.size() <  param_ptr_->interpreter_server_event_buffer_size_)
    {
        event_list_.push_back(event);
    }
    event_list_mutex_.unlock();
}

InterpreterServer::InterpreterServer():
    log_ptr_(NULL), param_ptr_(NULL), is_exit_(false)
{

}

void InterpreterServer::handleRequestList()
{
    if(nn_poll(&poll_req_res_fd_, 1, 0) == -1)
    {
        return;
    }
    
    int recv_bytes;
    if(poll_req_res_fd_.revents & NN_POLLIN)
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

void InterpreterServer::handlePublishList()
{
    std::vector<ProcessCommPublish>::iterator it;
    struct timeval time_val;
    long time_elapsed; 
    publish_list_mutex_.lock();
    gettimeofday(&time_val, NULL);
    for(it = publish_list_.begin(); it != publish_list_.end(); ++it)
    {
        time_elapsed = computeTimeElapsed(time_val, it->last_publish_time);
        if(checkPublishCondition(time_elapsed, it->interval))
        {
            int send_bytes = nn_send(publish_socket_, it->data_ptr, sizeof(InterpreterPublish), 0); // block send
            if(send_bytes == -1)
            {
                FST_ERROR("handlePublishList: send publish failed, error = %d", nn_errno());
                break;
            }
            it->last_publish_time = time_val;
        }
    }
    publish_list_mutex_.unlock();
}

void InterpreterServer::handleEventList()
{
    if(nn_poll(&poll_event_fd_, 1, 0) == -1)
    {
        return;
    }

    std::vector<ProcessCommEvent>::iterator it;
    event_list_mutex_.lock();
    for(it = event_list_.begin(); it != event_list_.end(); ++it)
    {
        int send_bytes = nn_send(event_socket_, &it->data, sizeof(unsigned long long int), 0);
        if(send_bytes == -1)
        {
            FST_ERROR("handleEventList: send publish failed, error = %d", nn_errno());
            break;
        }
    }
    event_list_.clear();
    event_list_mutex_.unlock();
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
    memcpy(request_data_ptr, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, size);
}

void InterpreterServer::copyResponseDataToSendBuffer(InterpreterServerCmd cmd_id, void* response_data_ptr, 
                                                            int response_data_size, int& send_buffer_size)
{
    *((unsigned int*)send_buffer_ptr_) = cmd_id;
    memcpy(send_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, response_data_ptr, response_data_size);
    send_buffer_size = response_data_size + PROCESS_COMM_CMD_ID_SIZE;
}

long InterpreterServer::computeTimeElapsed(struct timeval& current_time_val, struct timeval& last_time_val)
{
    long delta_tv_sec = current_time_val.tv_sec - last_time_val.tv_sec;
    long delta_tv_usec = current_time_val.tv_usec - last_time_val.tv_usec;
    return delta_tv_sec * 1000 + delta_tv_usec / 1000;
}

bool InterpreterServer::checkPublishCondition(long time_elapsed, int interval)
{
    return (time_elapsed >= interval ? true : false);
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




