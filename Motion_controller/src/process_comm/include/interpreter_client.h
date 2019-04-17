#ifndef INTERPRETER_CLIENT_H
#define INTERPRETER_CLIENT_H

#include "process_comm_param.h"
#include "common_log.h"
#include <nanomsg/nn.h>
#include <vector>
#include "reg_manager.h"
#include "process_comm_datatype.h"
#include "interpreter_common.h"
#include "fst_io_device.h"
#include "basic_alg_datatype.h" 

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
    bool setMi(MiDataIpc* data);
    bool setMh(MhDataIpc* data);
    bool getPrReg(int id, fst_ctrl::PrRegDataIpc* data);
    bool getHrReg(int id, fst_ctrl::HrRegDataIpc* data);
    bool getMrReg(int id, fst_ctrl::MrRegDataIpc* data);
    bool getSrReg(int id, fst_ctrl::SrRegDataIpc* data);
    bool getRReg(int id, fst_ctrl::RRegDataIpc* data); 
    bool getMi(int id, MiDataIpc* data); 
    bool getMh(int id, MhDataIpc* data); 
    bool setInstruction(Instruction* data);
    bool isNextInstructionNeeded();
    
    bool setInterpreterServerStatus(bool status);
    ErrorCode getDi(uint32_t port_offset, uint32_t &value);
    ErrorCode setDi(uint32_t port_offset, uint32_t value);
    ErrorCode getDo(uint32_t port_offset, uint32_t &value);
    ErrorCode setDo(uint32_t port_offset, uint32_t value);
    ErrorCode getRi(uint32_t port_offset, uint32_t &value);
    ErrorCode setRi(uint32_t port_offset, uint32_t value);
    ErrorCode getRo(uint32_t port_offset, uint32_t &value);
    ErrorCode setRo(uint32_t port_offset, uint32_t value);
    ErrorCode getUi(uint32_t port_offset, uint32_t &value);
    ErrorCode setUi(uint32_t port_offset, uint32_t value);//not use
    ErrorCode getUo(uint32_t port_offset, uint32_t &value);
    ErrorCode getJoint(int id, basic_alg::Joint &joint);
    ErrorCode getCart(int id, basic_alg::PoseEuler &pos);
    ErrorCode cartToJoint(basic_alg::PoseEuler pos, basic_alg::Joint &joint);
    ErrorCode jointToCart(basic_alg::Joint joint, basic_alg::PoseEuler &pos);
    ErrorCode getUserOpMode(int &mode);
    
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
