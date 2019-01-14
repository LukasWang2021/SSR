#ifndef CONTROLLER_CLIENT_H
#define CONTROLLER_CLIENT_H

#include "process_comm_param.h"
#include "common_log.h"
#include <nanomsg/nn.h>
#include <vector>
#include <string>
#include "interpreter_common.h"
#include "controller_server.h"
#include "base_datatype.h"

namespace fst_base
{
class ControllerClient
{
public:
    ControllerClient(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr);
    ~ControllerClient();

    ErrorCode init(ControllerServer* controller_server_ptr);

    bool start(std::string file_name);
    bool debug(std::string file_name);
    bool forward();
    bool backward();
    bool jump(std::string xml_path);
    bool pause();
    bool resume();
    bool abort();
    bool getNextInstruction(Instruction* instruction_ptr); 
    bool setAutoStartMode(int start_mode);
    bool switchStep(int data);

    void handleSubscribe();
    void handleEvent();
    InterpreterPublish* getInterpreterPublishPtr();
    

private:
    fst_log::Logger* log_ptr_;
    ProcessCommParam* param_ptr_;
    ControllerServer* controller_server_ptr_;
    int req_resp_socket_;
    int req_resp_endpoint_id_;
    int sub_socket_;
    int sub_endpoint_id_;
    int event_socket_;
    int event_endpoint_id_;
    struct nn_pollfd poll_sub_fd_;
    struct nn_pollfd poll_event_fd_;
    uint8_t* recv_buffer_ptr_;
    uint8_t* send_buffer_ptr_;

    InterpreterPublish interpreter_publish_data_;
        
    bool sendRequest(unsigned int cmd_id, void* data_ptr, int send_size);
    bool recvResponse(int expect_recv_size);
};

}

#endif

