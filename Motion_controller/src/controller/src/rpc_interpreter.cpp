#include "controller_rpc.h"

using namespace fst_ctrl;

// "/rpc/interpreter/start"
void ControllerRpc::handleRpc0x00006154(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getInterpreterState() != INTERPRETER_IDLE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_START;
        return;
    }

    controller_client_ptr_->start(std::string(rq_data_ptr->data.data));
    rs_data_ptr->data.data = SUCCESS;

    state_machine_ptr_->transferRobotStateToRunning();
    
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/start"));
}

// "/rpc/interpreter/launch"
void ControllerRpc::handleRpc0x000072D8(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getInterpreterState() != INTERPRETER_IDLE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_LAUNCH;
        return;
    }

    controller_client_ptr_->launch(std::string(rq_data_ptr->data.data)); 
    rs_data_ptr->data.data = SUCCESS;

    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/launch"));
}

// "/rpc/interpreter/forward"
void ControllerRpc::handleRpc0x0000D974(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_FORWARD;
        return;
    }

    controller_client_ptr_->forward(); 
    rs_data_ptr->data.data = SUCCESS;
    state_machine_ptr_->transferRobotStateToRunning();
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/forward"));
}

// "/rpc/interpreter/backward"
void ControllerRpc::handleRpc0x00008E74(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_BACKWARD;
        return;
    }

    controller_client_ptr_->backward(); 
    rs_data_ptr->data.data = SUCCESS;
    state_machine_ptr_->transferRobotStateToRunning();
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/backward"));
}

// "/rpc/interpreter/jump"
void ControllerRpc::handleRpc0x00015930(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_JUMP;
        return;
    }

    controller_client_ptr_->jump(std::string(rq_data_ptr->data.data)); 
    rs_data_ptr->data.data = SUCCESS;
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/jump"));
}

// "/rpc/interpreter/pause"
void ControllerRpc::handleRpc0x0000BA55(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getInterpreterState() != INTERPRETER_EXECUTE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_PAUSE;
        return;
    }
    
    controller_client_ptr_->pause(); 
    motion_control_ptr_->pauseMove();
    rs_data_ptr->data.data = SUCCESS;
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/pause"));
}

// "/rpc/interpreter/resume"
void ControllerRpc::handleRpc0x0000CF55(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getInterpreterState() != INTERPRETER_PAUSED
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED
        || state_machine_ptr_->getRobotState() != ROBOT_IDLE)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_RESUME;
        return;
    }
    
    motion_control_ptr_->restartMove();
    controller_client_ptr_->resume(); 
    rs_data_ptr->data.data = SUCCESS;
    state_machine_ptr_->transferRobotStateToRunning();
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/resume"));
}

// "/rpc/interpreter/abort"
void ControllerRpc::handleRpc0x000086F4(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    controller_client_ptr_->abort(); 
    motion_control_ptr_->abortMove();
    rs_data_ptr->data.data = SUCCESS;
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/abort"));
}

