#include "controller_rpc.h"

using namespace fst_ctrl;


// "/rpc/controller/getUserOpMode"
void ControllerRpc::handleRpc0x00000C05(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getUserOpMode();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getUserOpMode"));
}

// "/rpc/controller/getRunningStatus"
void ControllerRpc::handleRpc0x00000AB3(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getRunningState();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getRunningStatus"));
}

// "/rpc/controller/getInterpreterStatus"
void ControllerRpc::handleRpc0x00016483(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getInterpreterState();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getInterpreterStatus"));
}

// "/rpc/controller/getRobotStatus"
void ControllerRpc::handleRpc0x00006F83(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getRobotState();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getRobotStatus"));
}

// "/rpc/controller/getCtrlStatus"
void ControllerRpc::handleRpc0x0000E9D3(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getCtrlState();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getCtrlStatus"));
}

// "/rpc/controller/getServoStatus"
void ControllerRpc::handleRpc0x0000D113(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getServoState();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getServoStatus"));
}

// "/rpc/controller/getSafetyAlarm"
void ControllerRpc::handleRpc0x0000C00D(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getSafetyAlarm();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getSafetyAlarm"));
}

// "/rpc/controller/callEstop"
void ControllerRpc::handleRpc0x00013940(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    data_ptr->data.data = state_machine_ptr_->callEstop();
    recordLog(CONTROLLER_LOG, data_ptr->data.data, std::string("/rpc/controller/callEstop"));
}

// "/rpc/controller/callReset"
void ControllerRpc::handleRpc0x000161E4(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    data_ptr->data.data = state_machine_ptr_->callReset();
    recordLog(CONTROLLER_LOG, data_ptr->data.data, std::string("/rpc/controller/callReset"));
}

// "/rpc/controller/setUserOpMode"
void ControllerRpc::handleRpc0x00002ED5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if(param_ptr_->is_simmulation_)
    {
        rs_data_ptr->data.data = state_machine_ptr_->setUserOpMode((UserOpMode)rq_data_ptr->data.data);
    }
    else
    {
        FST_ERROR("Not simmulation");
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
    }
    recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/setUserOpMode"));
}

// "/rpc/controller/shutdown"
void ControllerRpc::handleRpc0x0000899E(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    rs_data_ptr->data.data = state_machine_ptr_->callShutdown();
    recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/shutdown"));
}
