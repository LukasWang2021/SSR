#include "interpreter_client.h"
#include <nanomsg/ipc.h>
#include <nanomsg/reqrep.h>
#include <cstring>
#include <iostream>
#include "process_comm_datatype.h"
#include "error_code.h"


using namespace fst_base;
using namespace fst_ctrl;

InterpreterClient::InterpreterClient(fst_log::Logger* log_ptr, ProcessCommParam* param_ptr):
    log_ptr_(log_ptr), param_ptr_(param_ptr),
    recv_buffer_ptr_(NULL), send_buffer_ptr_(NULL)
{

}

InterpreterClient::~InterpreterClient()
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

ErrorCode InterpreterClient::init()
{
    req_resp_socket_ = nn_socket(AF_SP, NN_REQ);
    if(req_resp_socket_ == -1) return PROCESS_COMM_INTERPRETER_CLIENT_INIT_FAILED;

    req_resp_endpoint_id_ = nn_connect(req_resp_socket_, param_ptr_->i2c_req_res_ip_.c_str());
    if(req_resp_endpoint_id_ == -1) return PROCESS_COMM_INTERPRETER_CLIENT_INIT_FAILED;

    recv_buffer_ptr_ = new uint8_t[param_ptr_->recv_buffer_size_]();
    send_buffer_ptr_ = new uint8_t[param_ptr_->send_buffer_size_]();

    return SUCCESS;
}

bool InterpreterClient::setPrReg(PrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_PR_REG, data, sizeof(PrRegDataIpc))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_PR_REG)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::setHrReg(HrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_HR_REG, data, sizeof(HrRegDataIpc))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_HR_REG)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::setMrReg(MrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_MR_REG, data, sizeof(MrRegDataIpc))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_MR_REG)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::setSrReg(SrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_SR_REG, data, sizeof(SrRegDataIpc))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_SR_REG)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::setRReg(RRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_R_REG, data, sizeof(RRegDataIpc))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_R_REG)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::getPrReg(int id, PrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_PR_REG, &id, sizeof(int))
        || !recvResponse(sizeof(PrRegDataIpc))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_PR_REG)
    {
        return false;
    }
    
    memcpy(data, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(PrRegDataIpc));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getHrReg(int id, HrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_HR_REG, &id, sizeof(int))
        || !recvResponse(sizeof(HrRegDataIpc))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_HR_REG)
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(HrRegDataIpc));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getMrReg(int id, MrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_MR_REG, &id, sizeof(int))
        || !recvResponse(sizeof(MrRegDataIpc))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_MR_REG)
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(MrRegDataIpc));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getSrReg(int id, SrRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_SR_REG, &id, sizeof(int))
        || !recvResponse(sizeof(SrRegDataIpc))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_SR_REG)
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(SrRegDataIpc));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::getRReg(int id, RRegDataIpc* data)
{
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_GET_R_REG, &id, sizeof(int))
        || !recvResponse(sizeof(RRegDataIpc))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_R_REG)
    {
        return false;
    }

    memcpy(data, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(RRegDataIpc));
    if(data->id == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool InterpreterClient::setInstruction(Instruction* data)
{   
    if(data == NULL
        || !sendRequest(CONTROLLER_SERVER_CMD_SET_INSTRUCTION, data, sizeof(Instruction))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_INSTRUCTION)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

bool InterpreterClient::isNextInstructionNeeded()
{
    if(!sendRequest(CONTROLLER_SERVER_CMD_IS_NEXT_INSTRUCTION_NEEDED, NULL, 0)
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_IS_NEXT_INSTRUCTION_NEEDED)
    {
        return false;
    }
    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}


bool InterpreterClient::setInterpreterServerStatus(bool status)
{
    if(!sendRequest(CONTROLLER_SERVER_CMD_SET_INTERPRETER_SERVER_STATUS, &status, sizeof(bool))
        || !recvResponse(sizeof(bool))
        || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_INTERPRETER_SERVER_STATUS)
    {
        return false;
    }

    return *((bool*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

//getDi
ErrorCode InterpreterClient::getDi(uint32_t port_offset, uint32_t &value)
{
    RequestGetDi request_get_di;
    request_get_di.port_offset = port_offset;
    if(!sendRequest(CONTROLLER_SERVER_CMD_GET_DI, (void*)&request_get_di, sizeof(RequestGetDi))
       || !recvResponse(sizeof(ResponseGetDi))
       || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_DI)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    ResponseGetDi response_get_di;
    memcpy(&response_get_di, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(ResponseGetDi));
    value = response_get_di.value;
    return response_get_di.error_code;
}

//setDi
ErrorCode InterpreterClient::setDi(uint32_t port_offset, uint32_t value)
{
    RequestSetDi request_set_di;
    request_set_di.port_offset = port_offset;
    request_set_di.value = value;
    if(!sendRequest(CONTROLLER_SERVER_CMD_SET_DI, (void*)&request_set_di, sizeof(RequestSetDi))
       || !recvResponse(sizeof(unsigned long long))
       || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_DI)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    return *((unsigned long long*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

//getDo
ErrorCode InterpreterClient::getDo(uint32_t port_offset, uint32_t &value)
{
    RequestGetDo request_get_do;
    request_get_do.port_offset = port_offset;
    if(!sendRequest(CONTROLLER_SERVER_CMD_GET_DO, (void*)&request_get_do, sizeof(RequestGetDo))
       || !recvResponse(sizeof(ResponseGetDo))
       || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_DO)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    ResponseGetDo response_get_do;
    memcpy(&response_get_do, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(ResponseGetDo));
    value = response_get_do.value;
    return response_get_do.error_code;
}

//setDo
ErrorCode InterpreterClient::setDo(uint32_t port_offset, uint32_t value)
{
    RequestSetDo request_set_do;
    request_set_do.port_offset = port_offset;
    request_set_do.value = value;
    if(!sendRequest(CONTROLLER_SERVER_CMD_SET_DO, (void*)&request_set_do, sizeof(RequestSetDo))
       || !recvResponse(sizeof(unsigned long long))
       || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_DO)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    return *((unsigned long long*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

//getRi
ErrorCode InterpreterClient::getRi(uint32_t port_offset, uint32_t &value)
{
    RequestGetRi request_get_ri;
    request_get_ri.port_offset = port_offset;
    if(!sendRequest(CONTROLLER_SERVER_CMD_GET_RI, (void*)&request_get_ri, sizeof(RequestGetRi))
       || !recvResponse(sizeof(ResponseGetRi))
       || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_RI)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    ResponseGetRi response_get_ri;
    memcpy(&response_get_ri, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(ResponseGetRi));
    value = response_get_ri.value;
    return response_get_ri.error_code;
}

//setRi
ErrorCode InterpreterClient::setRi(uint32_t port_offset, uint32_t value)
{
    RequestSetRi request_set_ri;
    request_set_ri.port_offset = port_offset;
    request_set_ri.value = value;
    if(!sendRequest(CONTROLLER_SERVER_CMD_SET_RI, (void*)&request_set_ri, sizeof(RequestSetRi))
       || !recvResponse(sizeof(unsigned long long))
       || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_RI)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    return *((unsigned long long*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

//getRo
ErrorCode InterpreterClient::getRo(uint32_t port_offset, uint32_t &value)
{
    RequestGetRo request_get_ro;
    request_get_ro.port_offset = port_offset;
    if(!sendRequest(CONTROLLER_SERVER_CMD_GET_RO, (void*)&request_get_ro, sizeof(RequestGetRo))
       || !recvResponse(sizeof(ResponseGetRo))
       || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_RO)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    ResponseGetDo response_get_ro;
    memcpy(&response_get_ro, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(ResponseGetRo));
    value = response_get_ro.value;
    return response_get_ro.error_code;
}

//setRo
ErrorCode InterpreterClient::setRo(uint32_t port_offset, uint32_t value)
{
    RequestSetRo request_set_ro;
    request_set_ro.port_offset = port_offset;
    request_set_ro.value = value;
    if(!sendRequest(CONTROLLER_SERVER_CMD_SET_RO, (void*)&request_set_ro, sizeof(RequestSetRo))
       || !recvResponse(sizeof(unsigned long long))
       || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_RO)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    return *((unsigned long long*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}


//getUi
ErrorCode InterpreterClient::getUi(uint32_t port_offset, uint32_t &value)
{
    RequestGetUi request_get_ui;
    request_get_ui.port_offset = port_offset;
    if(!sendRequest(CONTROLLER_SERVER_CMD_GET_UI, (void*)&request_get_ui, sizeof(RequestGetUi))
       || !recvResponse(sizeof(ResponseGetUi))
       || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_UI)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    ResponseGetUi response_get_ui;
    memcpy(&response_get_ui, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(ResponseGetUi));
    value = response_get_ui.value;
    return response_get_ui.error_code;
}

//setUi
ErrorCode InterpreterClient::setUi(uint32_t port_offset, uint32_t value)
{
    RequestSetUi request_set_ui;
    request_set_ui.port_offset = port_offset;
    request_set_ui.value = value;
    if(!sendRequest(CONTROLLER_SERVER_CMD_SET_UI, (void*)&request_set_ui, sizeof(RequestSetUi))
       || !recvResponse(sizeof(unsigned long long))
       || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_SET_UI)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    return *((unsigned long long*)(recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE));
}

//getUo
ErrorCode InterpreterClient::getUo(uint32_t port_offset, uint32_t &value)
{
    RequestGetUo request_get_uo;
    request_get_uo.port_offset = port_offset;
    if(!sendRequest(CONTROLLER_SERVER_CMD_GET_UO, (void*)&request_get_uo, sizeof(RequestGetUo))
       || !recvResponse(sizeof(ResponseGetUo))
       || *((unsigned int*)recv_buffer_ptr_) != CONTROLLER_SERVER_CMD_GET_UO)
    {
        return PROCESS_COMM_OPERATION_FAILED;
    }
    ResponseGetUo response_get_uo;
    memcpy(&response_get_uo, recv_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, sizeof(ResponseGetUo));
    value = response_get_uo.value;
    return response_get_uo.error_code;
}




bool InterpreterClient::sendRequest(unsigned int cmd_id, void* data_ptr, int send_size)
{
    *((unsigned int*)send_buffer_ptr_) = cmd_id;
    if(data_ptr != NULL && send_size != 0)
    {
        memcpy(send_buffer_ptr_ + PROCESS_COMM_CMD_ID_SIZE, data_ptr, send_size);
    }
    int send_bytes = nn_send(req_resp_socket_, send_buffer_ptr_, send_size + PROCESS_COMM_CMD_ID_SIZE, 0); // block send
    if(send_bytes == -1 || send_bytes != (send_size + PROCESS_COMM_CMD_ID_SIZE))
    {
        FST_ERROR("handleResponseList: send response failed, nn_error = %d", nn_errno());
        return false;
    }
    return true;
}

bool InterpreterClient::recvResponse(int expect_recv_size)
{
    int recv_size = nn_recv(req_resp_socket_, recv_buffer_ptr_, param_ptr_->recv_buffer_size_, 0);
    if(recv_size == -1 
        || recv_size != (expect_recv_size + PROCESS_COMM_CMD_ID_SIZE))
    {
        return false;
    }
    else
    {
        return true;
    }
}

