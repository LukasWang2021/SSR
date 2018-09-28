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
        FST_ERROR("Failed to load handleRpc0x00006154");
        FST_ERROR("Interpreter state is %d, expect for 0", state_machine_ptr_->getInterpreterState());
        FST_ERROR("Controller state is %d, expect for 1", state_machine_ptr_->getCtrlState());
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }

    controller_client_ptr_->start(std::string(rq_data_ptr->data.data));
    rs_data_ptr->data.data = SUCCESS;

    state_machine_ptr_->transferRobotStateToRunning();
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/start"));
}

// "/rpc/interpreter/debug"
void ControllerRpc::handleRpc0x000102D7(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getInterpreterState() != INTERPRETER_IDLE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        FST_ERROR("Failed to load handleRpc0x000102D7");
        FST_ERROR("User operation mode is %d, expect not 0 and 1", state_machine_ptr_->getUserOpMode());
        FST_ERROR("Controller state is %d, expect for 1", state_machine_ptr_->getCtrlState());
        FST_ERROR("Interpreter state is %d, expect for 0", state_machine_ptr_->getInterpreterState());
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }

    controller_client_ptr_->debug(std::string(rq_data_ptr->data.data)); 
    rs_data_ptr->data.data = SUCCESS;
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/debug"));
}

// "/rpc/interpreter/forward"
void ControllerRpc::handleRpc0x0000D974(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        FST_ERROR("Failed to load handleRpc0x0000D974");
        FST_ERROR("User operation mode is %d, expect not 0 and 1", state_machine_ptr_->getUserOpMode());
        FST_ERROR("Controller state is %d, expect for 1", state_machine_ptr_->getCtrlState());
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
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
        FST_ERROR("Failed to load handleRpc0x00008E74");
        FST_ERROR("User operation mode is %d, expect not 0 and 1", state_machine_ptr_->getUserOpMode());
        FST_ERROR("Controller state is %d, expect for 1", state_machine_ptr_->getCtrlState());

        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
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
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        FST_ERROR("Failed to load handleRpc0x00015930");
        FST_ERROR("User operation mode is %d, expect not 0 and 1", state_machine_ptr_->getUserOpMode());
        FST_ERROR("Controller state is %d, expect for 1", state_machine_ptr_->getCtrlState());

        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }

    controller_client_ptr_->jump(rq_data_ptr->data.data); 
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
        FST_ERROR("Failed to load handleRpc0x0000BA55");
        FST_ERROR("Interpreter state is %d, expect for 1", state_machine_ptr_->getInterpreterState());
        FST_ERROR("Controller state is %d, expect for 1", state_machine_ptr_->getCtrlState());

        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }
        FST_INFO("Load /rpc/interpreter/pause");
    
    controller_client_ptr_->pause(); 
    rs_data_ptr->data.data = SUCCESS;
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/pause"));
}

// "/rpc/interpreter/resume"
void ControllerRpc::handleRpc0x0000CF55(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getInterpreterState() != INTERPRETER_PAUSED
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        FST_ERROR("Failed to load handleRpc0x0000CF55");
        FST_ERROR("Interpreter state is %d, expect for 2", state_machine_ptr_->getInterpreterState());
        FST_ERROR("Controller state is %d, expect for 1", state_machine_ptr_->getCtrlState());
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }
        FST_INFO("Load /rpc/interpreter/resume");
    
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
        FST_ERROR("Load /rpc/interpreter/abort");
    rs_data_ptr->data.data = SUCCESS;
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/abort"));
}

// "/rpc/interpreter/jump"
void ControllerRpc::handleRpc0x000140F0(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    // to do...
    if(// state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO || 
       // state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE || 
       state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        FST_ERROR("Failed to load handleRpc0x000140F0 with %d and %d ",
			(int)state_machine_ptr_->getUserOpMode(), 
			(int)state_machine_ptr_->getCtrlState());
        FST_ERROR("Controller state is %d, expect for 1", state_machine_ptr_->getCtrlState());

        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }

    // to detect...
    controller_client_ptr_->switchStep(rq_data_ptr->data.data); 
    rs_data_ptr->data.data = SUCCESS;
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/switchStep"));
}
