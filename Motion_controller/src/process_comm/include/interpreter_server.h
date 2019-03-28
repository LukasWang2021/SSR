#ifndef INTERPRETER_SERVER_H
#define INTERPRETER_SERVER_H

#include "process_comm_param.h"
#include "process_comm_datatype.h"
#include "common_log.h"
#include "thread_help.h"
#include <nanomsg/nn.h>
#include <mutex>
#include <vector>
#include "interpreter_common.h"
#include "basic_alg_datatype.h"

namespace fst_base
{
class InterpreterServer
{
public:
    InterpreterServer(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr);
    ~InterpreterServer();

    ErrorCode init();
    bool isExit();
    ErrorCode open();
    void close();
    void runThreadFunc();
    
    std::vector<ProcessCommRequestResponse> popTaskFromRequestList();
    void pushTaskToResponseList(ProcessCommRequestResponse& package);
    bool addPublishTask(int interval, InterpreterPublish* data_ptr); // interval unit is ms
    void removePublishTask();
    void sendEvent(int event_type, void* data_ptr);
    
private:
    fst_log::Logger* log_ptr_;
    ProcessCommParam* param_ptr_;    
    static bool is_server_ready_;

    typedef void (InterpreterServer::*HandleRequestFuncPtr)();
    typedef void (InterpreterServer::*HandleResponseFuncPtr)(std::vector<ProcessCommRequestResponse>::iterator&, int&);

    // runtime variables
    bool is_exit_;
    ThreadHelp thread_;
    int req_resp_socket_;
    int publish_socket_;
    int event_socket_;
    int req_resp_endpoint_id_;
    int publish_endpoint_id_;
    int event_endpoint_id_;
    struct nn_pollfd poll_req_res_fd_;
    struct nn_pollfd poll_pub_fd_;
    struct nn_pollfd poll_event_fd_;
    uint8_t* recv_buffer_ptr_;
    uint8_t* send_buffer_ptr_;

    // runtime list & list mutex
    std::mutex request_list_mutex_;
    std::mutex response_list_mutex_;
    std::mutex publish_list_mutex_;
    std::mutex event_list_mutex_;
    std::vector<ProcessCommRequestResponse>  request_list_;
    std::vector<ProcessCommRequestResponse>  response_list_;
    std::vector<ProcessCommPublish> publish_list_;
    std::vector<ProcessCommEvent> event_list_;

    // runtime table
    typedef struct
    {
        unsigned int cmd_id;
        HandleRequestFuncPtr request_func_ptr;
        HandleResponseFuncPtr response_func_ptr;
    }InterpreterRpcService;
    std::vector<InterpreterRpcService> rpc_table_;

    InterpreterServer();
    void initRpcTable();
    void handleRequestList();
    void handleResponseList();
    void handlePublishList();
    void handleEventList();
    void pushTaskToRequestList(unsigned int cmd_id, void* request_data_ptr, void* response_data_ptr);
    void copyRecvBufferToRequestData(void* request_data_ptr, int size);
    void copyResponseDataToSendBuffer(InterpreterServerCmd cmd_id, void* response_data_ptr, 
                                                int response_data_size, int& send_buffer_size);
    long long computeTimeElapsed(struct timeval& current_time_val, struct timeval& last_time_val);
    bool checkPublishCondition(long long time_elapsed, int interval);

    // rpc request handler
    void handleRequestStart();
    void handleRequestLaunch();
    void handleRequestForward();
    void handleRequestBackward();
    void handleRequestJump();
    void handleRequestPause();
    void handleRequestResume();
    void handleRequestAbort();
    void handleRequestGetNextInstruction();
    void handleRequestCodeStart();    

    // rpc response handler
    void handleResponseStart(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
   void handleResponseLaunch(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseForward(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseBackward(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseJump(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponsePause(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseResume(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseAbort(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetNextInstruction(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseCodeStart(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size); 
};

}

void interpreterServerThreadFunc(void* arg);

#endif

