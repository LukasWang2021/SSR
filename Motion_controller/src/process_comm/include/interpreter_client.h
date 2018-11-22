#ifndef INTERPRETER_CLIENT_H
#define INTERPRETER_CLIENT_H

#include "process_comm_param.h"
#include "common_log.h"
#include <nanomsg/nn.h>
#include <vector>
#include "reg_manager.h"
#include "base_datatype.h"
#include "interpreter_common.h"

#include "fst_io_device.h"

namespace fst_base
{
class InterpreterClient
{
public:
    InterpreterClient(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr);
    ~InterpreterClient();

    ErrorCode init();

    bool setPrReg(fst_ctrl::PrRegDataIpc* data);
    bool setHrReg(fst_ctrl::HrRegDataIpc* data);
    bool setMrReg(fst_ctrl::MrRegDataIpc* data);
    bool setSrReg(fst_ctrl::SrRegDataIpc* data);
    bool setRReg(fst_ctrl::RRegDataIpc* data);
    bool getPrReg(int id, fst_ctrl::PrRegDataIpc* data);
    bool getHrReg(int id, fst_ctrl::HrRegDataIpc* data);
    bool getMrReg(int id, fst_ctrl::MrRegDataIpc* data);
    bool getSrReg(int id, fst_ctrl::SrRegDataIpc* data);
    bool getRReg(int id, fst_ctrl::RRegDataIpc* data); 
    bool setInstruction(Instruction* data);
    bool isNextInstructionNeeded();
    ErrorCode checkIo(char path[256], IOPortInfo* port_info_ptr);//todo
    ErrorCode setIo(IOPortInfo* port_info_ptr, char value);//todo
    ErrorCode getIo(IOPortInfo* port_info_ptr, int buffer_length, char* value_ptr);//todo
    bool setInterpreterServerStatus(bool status);
    ErrorCode getDi(uint32_t &value);
    ErrorCode setDi(void);
    ErrorCode getDo(void);
    ErrorCode setDo(void);
    ErrorCode getRi(void);
    ErrorCode setRi(void);
    ErrorCode getRo(void);
    ErrorCode setRo(void);
    
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
