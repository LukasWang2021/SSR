#ifndef CONTROLLER_SERVER_H
#define CONTROLLER_SERVER_H

#include "process_comm_param.h"
#include "process_comm_datatype.h"
#include "common_log.h"
#include "thread_help.h"
#include <nanomsg/nn.h>
#include <mutex>
#include <vector>
#include "base_datatype.h"


namespace fst_base
{
class ControllerServer
{
public:
    ControllerServer(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr);
    ~ControllerServer();

    ErrorCode init();
    bool isExit();
    ErrorCode open();
    void close();
    std::vector<ProcessCommRequestResponse> popTaskFromRequestList();
    void pushTaskToResponseList(ProcessCommRequestResponse& package);
    void runThreadFunc();

    bool setInterpreterServerStatus(bool is_ready);
    bool isInterpreterServerReady();
    
private:
    fst_log::Logger* log_ptr_;
    ProcessCommParam* param_ptr_;
    bool is_interpreter_server_ready_;
    typedef void (ControllerServer::*HandleRequestFuncPtr)();
    typedef void (ControllerServer::*HandleResponseFuncPtr)(std::vector<ProcessCommRequestResponse>::iterator&, int&);

    // runtime variables
    bool is_exit_;
    ThreadHelp thread_;
    int req_resp_socket_;
    int req_resp_endpoint_id_;
    struct nn_pollfd poll_fd_;
    uint8_t* recv_buffer_ptr_;
    uint8_t* send_buffer_ptr_;

    // runtime list & list mutex
    std::mutex request_list_mutex_;
    std::mutex response_list_mutex_;
    std::mutex event_list_mutex_;
    std::vector<ProcessCommRequestResponse>  request_list_;
    std::vector<ProcessCommRequestResponse>  response_list_;

    // runtime table    
    typedef struct
    {
        unsigned int cmd_id;
        HandleRequestFuncPtr request_func_ptr;
        HandleResponseFuncPtr response_func_ptr;
    }ControllerRpcService;
    std::vector<ControllerRpcService> rpc_table_;
    
    ControllerServer();
    void initRpcTable();
    void handleRequestList();
    void handleResponseList();
    void pushTaskToRequestList(unsigned int cmd_id, void* request_data_ptr, void* response_data_ptr);
    void copyRecvBufferToRequestData(void* request_data_ptr, int size);
    void copyResponseDataToSendBuffer(ControllerServerCmd cmd_id, void* response_data_ptr, 
                                                int response_data_size, int& send_buffer_size);

    // rpc request handler
    void handleRequestSetPrReg();
    void handleRequestSetHrReg();
    void handleRequestSetMrReg();
    void handleRequestSetSrReg();
    void handleRequestSetRReg();
    void handleRequestGetPrReg();
    void handleRequestGetHrReg();
    void handleRequestGetMrReg();
    void handleRequestGetSrReg();
    void handleRequestGetRReg();
    void handleRequestGetMi();
    void handleRequestSetInstruction();
    void handleRequestIsNextInstructionNeeded();
    void handleRequestSetInterpreterServerStatus();
    void handleRequestGetDi();
    void handleRequestSetDi();
    void handleRequestGetDo();
    void handleRequestSetDo();
    void handleRequestGetRi();
    void handleRequestSetRi();
    void handleRequestGetRo();
    void handleRequestSetRo();
    void handleRequestGetUi();
    void handleRequestSetUi();
    void handleRequestGetUo();

    // rpc response handler
    void handleResponseSetPrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseSetHrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseSetMrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseSetSrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseSetRReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetPrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetHrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetMrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetSrReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetRReg(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetMi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseSetInstruction(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseIsNextInstructionNeeded(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size); 
    void handleResponseSetInterpreterServerStatus(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetDi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseSetDi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetDo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseSetDo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetRi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseSetRi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetRo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseSetRo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetUi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseSetUi(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    void handleResponseGetUo(std::vector<ProcessCommRequestResponse>::iterator& task, int& send_buffer_size);
    
};

}

void controllerServerThreadFunc(void* arg);

#endif

