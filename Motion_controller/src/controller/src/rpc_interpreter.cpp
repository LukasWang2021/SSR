#include "controller_rpc.h"
#include "forsight_inter_control.h"

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
        recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/start"));
        return;
    }

//    controller_client_ptr_->start(std::string(rq_data_ptr->data.data));
	InterpreterControl intprt_ctrl ;
	intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_START ;
	parseCtrlComand(intprt_ctrl, rq_data_ptr->data.data);
	
    rs_data_ptr->data.data = SUCCESS;

    state_machine_ptr_->transferRobotStateToRunning();
    state_machine_ptr_->setUoProgramRunOn();//UO[4]=on
    
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
        recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/launch"));
        return;
    }

    // controller_client_ptr_->launch(std::string(rq_data_ptr->data.data)); 
	InterpreterControl intprt_ctrl ;
	intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_LAUNCH ;
	parseCtrlComand(intprt_ctrl, rq_data_ptr->data.data);
	
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
        recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/forward"));
        return;
    }

    // controller_client_ptr_->forward(); 
	InterpreterControl intprt_ctrl ;
	intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_FORWARD ;
	parseCtrlComand(intprt_ctrl, "");
	
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
        recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/backward"));
        return;
    }

    rs_data_ptr->data.data = motion_control_ptr_->abortMove();
    if (rs_data_ptr->data.data == SUCCESS)
    {
    //    controller_client_ptr_->backward(); 
		InterpreterControl intprt_ctrl ;
		intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_BACKWARD ;
		parseCtrlComand(intprt_ctrl, "");
	
        state_machine_ptr_->transferRobotStateToRunning();
    }

    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/backward"));
}

// "/rpc/interpreter/jump"
void ControllerRpc::handleRpc0x00015930(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getRobotState() != ROBOT_IDLE
        || state_machine_ptr_->getInterpreterState() == INTERPRETER_EXECUTE
        || state_machine_ptr_->getInterpreterState() == INTERPRETER_IDLE_TO_EXECUTE
        || state_machine_ptr_->getInterpreterState() == INTERPRETER_PAUSE_TO_EXECUTE)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_JUMP;
        recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/jump"));
        return;
    }

    rs_data_ptr->data.data = motion_control_ptr_->abortMove();
    if (rs_data_ptr->data.data == SUCCESS)
    {
    //    controller_client_ptr_->jump(std::string(rq_data_ptr->data.data));
		InterpreterControl intprt_ctrl ;
		intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_JUMP ;
		parseCtrlComand(intprt_ctrl, rq_data_ptr->data.data);
    }

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
        recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/pause"));
        return;
    }

    rs_data_ptr->data.data = motion_control_ptr_->pauseMove();
    if (rs_data_ptr->data.data == SUCCESS)
    {
        // controller_client_ptr_->pause(); 
		InterpreterControl intprt_ctrl ;
		intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_PAUSE ;
		parseCtrlComand(intprt_ctrl, "");
		
        state_machine_ptr_->setUoPausedOn();//UO[2]=on
        state_machine_ptr_->setUoProgramRunOff();//UO[4]=off
    }
    else
    {
        state_machine_ptr_->setPauseFlag(true);
    }
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
        recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/resume"));
        return;
    }
    
    rs_data_ptr->data.data = motion_control_ptr_->restartMove();
    if (rs_data_ptr->data.data == SUCCESS)
    {
    //    controller_client_ptr_->resume(); 
		InterpreterControl intprt_ctrl ;
		intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_RESUME ;
		parseCtrlComand(intprt_ctrl, "");
		
        state_machine_ptr_->transferRobotStateToRunning();
        state_machine_ptr_->setUoPausedOff();//UO[2]=off
        state_machine_ptr_->setUoProgramRunOn();//UO[4]=on
    }
    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/resume"));
}

// "/rpc/interpreter/abort"
void ControllerRpc::handleRpc0x000086F4(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->abortMove();
    if (rs_data_ptr->data.data == SUCCESS)
    {
    //    controller_client_ptr_->abort(); 
		InterpreterControl intprt_ctrl ;
		intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_ABORT ;
		parseCtrlComand(intprt_ctrl, "");
		
        state_machine_ptr_->setUoProgramRunOff();//UO[4]=off
    }

    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/interpreter/abort"));
}

