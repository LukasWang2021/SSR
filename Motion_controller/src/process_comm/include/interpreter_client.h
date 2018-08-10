#ifndef INTERPRETER_CLIENT_H
#define INTERPRETER_CLIENT_H

#include "process_comm_param.h"
#include "common_log.h"
#include <nanomsg/nn.h>
#include <vector>
#include "reg_manager.h"

namespace fst_base
{
class InterpreterClient
{
public:
    InterpreterClient(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr);
    ~InterpreterClient();

    bool init();

    bool setPrReg(fst_ctrl::PrRegData* data);
    bool setHrReg(fst_ctrl::HrRegData* data);
    bool setMrReg(fst_ctrl::MrRegData* data);
    bool setSrReg(fst_ctrl::SrRegData* data);
    bool setRReg(fst_ctrl::RRegData* data);
    bool getPrReg(int id, fst_ctrl::PrRegData* data);
    bool getHrReg(int id, fst_ctrl::HrRegData* data);
    bool getMrReg(int id, fst_ctrl::MrRegData* data);
    bool getSrReg(int id, fst_ctrl::SrRegData* data);
    bool getRReg(int id, fst_ctrl::RRegData* data); 
private:
    fst_log::Logger* log_ptr_;
    ProcessCommParam* param_ptr_;  
    int req_resp_socket_;
    int req_resp_endpoint_id_;
    uint8_t* recv_buffer_ptr_;
    uint8_t* send_buffer_ptr_;

    bool sendRequest(void* data_ptr, int send_size);
    bool recvResponse(int expect_recv_size);
};

}

#endif
