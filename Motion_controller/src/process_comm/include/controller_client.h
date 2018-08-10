#ifndef CONTROLLER_CLIENT_H
#define CONTROLLER_CLIENT_H

#include "process_comm_param.h"
#include "common_log.h"
#include <nanomsg/nn.h>
#include <vector>
#include <string>
#include "interpreter_common.h"


namespace fst_base
{
class ControllerClient
{
public:
    ControllerClient(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr);
    ~ControllerClient();

    bool init();

    bool start(std::string file_name);
    bool debug(std::string file_name);
    bool forward();
    bool backward();
    bool jump(int data);
    bool pause();
    bool resume();
    bool abort();
    bool getNextInstruction(Instruction* instruction_ptr); 

private:
    fst_log::Logger* log_ptr_;
    ProcessCommParam* param_ptr_;  
    int req_resp_socket_;
    int req_resp_endpoint_id_;
    uint8_t* recv_buffer_ptr_;
    uint8_t* send_buffer_ptr_;

    bool sendRequest(unsigned int cmd_id, void* data_ptr, int send_size);
    bool recvResponse(int expect_recv_size);
};

}

#endif

