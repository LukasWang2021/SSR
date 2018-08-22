#include "controller_rpc.h"

using namespace fst_ctrl;


// "/rpc/controller/getUserOpMode"
void ControllerRpc::handleRpc0x00000C05(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Int32* data_ptr = static_cast<ResponseMessageType_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getUserOpMode();
}

// "/rpc/controller/getRunningStatus"
void ControllerRpc::handleRpc0x00000AB3(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Int32* data_ptr = static_cast<ResponseMessageType_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getRunningState();
}

// "/rpc/controller/getInterpreterStatus"
void ControllerRpc::handleRpc0x00016483(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Int32* data_ptr = static_cast<ResponseMessageType_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getInterpreterState();
}

// "/rpc/controller/getRobotStatus"
void ControllerRpc::handleRpc0x00006F83(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Int32* data_ptr = static_cast<ResponseMessageType_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getRobotState();
}

// "/rpc/controller/getCtrlStatus"
void ControllerRpc::handleRpc0x0000E9D3(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Int32* data_ptr = static_cast<ResponseMessageType_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getCtrlState();
}

// "/rpc/controller/getServoStatus"
void ControllerRpc::handleRpc0x0000D113(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Int32* data_ptr = static_cast<ResponseMessageType_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getServoState();
}

// "/rpc/controller/getSafetyAlarm"
void ControllerRpc::handleRpc0x0000C00D(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Int32* data_ptr = static_cast<ResponseMessageType_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getSafetyAlarm();
}

// "/rpc/controller/callEstop"
void ControllerRpc::handleRpc0x00013940(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Bool* data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);
    data_ptr->data.data = state_machine_ptr_->callEstop();
}

// "/rpc/controller/callReset"
void ControllerRpc::handleRpc0x000161E4(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Bool* data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);
    data_ptr->data.data = state_machine_ptr_->callReset();
}

// "/rpc/controller/setUserOpMode"
void ControllerRpc::handleRpc0x00002ED5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);
    rs_data_ptr->data.data = state_machine_ptr_->setUserOpMode((UserOpMode)rq_data_ptr->data.data);
}

// "/rpc/controller/shutdown"
void ControllerRpc::handleRpc0x0000899E(void* request_data_ptr, void* response_data_ptr)
{

}

