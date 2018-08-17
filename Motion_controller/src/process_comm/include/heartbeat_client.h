#ifndef HEARTBEAT_CLIENT_H
#define HEARTBEAT_CLIENT_H

#include "process_comm_param.h"
#include "common_log.h"
#include <nanomsg/nn.h>
#include <vector>
#include <string>
#include <sys/time.h>


namespace fst_base
{
typedef struct 
{
    int cmd;
    uint8_t data[1024];
}HeartbeatClientRequestResponseData;

class HeartbeatClient
{
public:
    HeartbeatClient(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr);
    ~HeartbeatClient();

    bool init();
    void sendHeartbeat();

private:
    fst_log::Logger* log_ptr_;
    ProcessCommParam* param_ptr_;  
    int req_resp_socket_;
    int req_resp_endpoint_id_;
    uint8_t* recv_buffer_ptr_;
    uint8_t* send_buffer_ptr_;
    struct nn_pollfd poll_fd_;
    HeartbeatClientRequestResponseData request_data_;
    HeartbeatClientRequestResponseData response_data_;
    struct timeval current_time_;
    struct timeval last_send_time_;

    enum {CMD_HEARTBEAT = 0xA1,};

    long computeTimeElapsed();
};

}

#endif

