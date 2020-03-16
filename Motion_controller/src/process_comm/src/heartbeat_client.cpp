#include "heartbeat_client.h"
#include <nanomsg/ipc.h>
#include <nanomsg/reqrep.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include "error_monitor.h"
#include "error_code.h"


using namespace fst_base;

HeartbeatClient::HeartbeatClient(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr):
    log_ptr_(log_ptr), param_ptr_(param_ptr),
    recv_buffer_ptr_(NULL), send_buffer_ptr_(NULL)
{

}

HeartbeatClient::~HeartbeatClient()
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

ErrorCode HeartbeatClient::init()
{
    request_data_.cmd = CMD_HEARTBEAT;
    memset(&request_data_.data[0], 0, 1024);
    memset(&response_data_, 0, sizeof(HeartbeatClientRequestResponseData));
    memset(&current_time_, 0, sizeof(struct timeval));
    memset(&last_send_time_, 0, sizeof(struct timeval));

    req_resp_socket_ = nn_socket(AF_SP, NN_REQ);
    if(req_resp_socket_ == -1) return PROCESS_COMM_HEARTBEAT_CLIENT_INIT_FAILED;

    req_resp_endpoint_id_ = nn_connect(req_resp_socket_, param_ptr_->heartbeat_ip_.c_str());
    if(req_resp_endpoint_id_ == -1) return PROCESS_COMM_HEARTBEAT_CLIENT_INIT_FAILED;

	poll_fd_.fd = req_resp_socket_;
	poll_fd_.events = NN_POLLIN | NN_POLLOUT;

    recv_buffer_ptr_ = new uint8_t[param_ptr_->recv_buffer_size_]();
    send_buffer_ptr_ = new uint8_t[param_ptr_->send_buffer_size_]();

    return SUCCESS;
}

void HeartbeatClient::sendHeartbeat()
{
    if(nn_poll (&poll_fd_, 1, 0) <= 0)
    {
        return;
    }
    static bool send_req = false;
    int recv_bytes;
    if(poll_fd_.revents & NN_POLLIN)
    {
        recv_bytes = nn_recv(req_resp_socket_, recv_buffer_ptr_, param_ptr_->recv_buffer_size_, 0);
        if(recv_bytes == -1)
        {
            return;
        }
        send_req = false;
        memcpy(&response_data_, recv_buffer_ptr_, sizeof(HeartbeatClientRequestResponseData));

        if(response_data_.cmd != CMD_HEARTBEAT)
        {
            FST_ERROR("sendHeartbeat: unexpected heartbeat response");
            return false;
        }
        int error_num = *((int*)response_data_.data);
        unsigned long long int* error_code = (unsigned long long int*)&response_data_.data[8];
        for(int i=0; i<error_num; ++i)
        {
            FST_ERROR("sendHeartbeat: core1 report error_code: 0x%llx", error_code[i]);
            ErrorMonitor::instance()->add(error_code[i]);
        }
    }
    else    // socket can send something
    {
        gettimeofday(&current_time_, NULL);
        long long time_elapsed = computeTimeElapsed();
        if(time_elapsed >= (long long)param_ptr_->heartbeat_cycle_time_ && (send_req == false))
        {
            int send_bytes = nn_send(req_resp_socket_, &request_data_, sizeof(HeartbeatClientRequestResponseData), 0);
            if(send_bytes == -1 || send_bytes != sizeof(HeartbeatClientRequestResponseData))
            {
                FST_ERROR("sendHeartbeat: send heartbeat request failed, %s", nn_strerror(errno));
                return;
            }
            else
            {
                last_send_time_ = current_time_;
            }
            send_req = true;
        }
    }
}

long long HeartbeatClient::computeTimeElapsed()
{
    long long delta_tv_sec = current_time_.tv_sec - last_send_time_.tv_sec;
    long long delta_tv_usec = current_time_.tv_usec - last_send_time_.tv_usec;
    return delta_tv_sec * 1000000 + delta_tv_usec;
}

