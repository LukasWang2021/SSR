#include "controller_rpc.h"
#include "base_datatype.h"


using namespace fst_ctrl;

// "/rpc/motion_control/stop"
void ControllerRpc::handleRpc0x00001E70(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->stopGroup();
}

// "/rpc/motion_control/reset"
void ControllerRpc::handleRpc0x00001D14(void* request_data_ptr, void* response_data_ptr)
{

}

// "/rpc/motion_control/axis_group/setManualMode"
void ControllerRpc::handleRpc0x00012FB5(void* request_data_ptr, void* response_data_ptr)
{

}

// "/rpc/motion_control/axis_group/setManualFrame"
void ControllerRpc::handleRpc0x00009D05(void* request_data_ptr, void* response_data_ptr)
{

}

// "/rpc/motion_control/axis_group/doStepManualMove"
void ControllerRpc::handleRpc0x000085D5(void* request_data_ptr, void* response_data_ptr)
{

}

// "/rpc/motion_control/axis_group/doContinusManualMove"
void ControllerRpc::handleRpc0x000173B5(void* request_data_ptr, void* response_data_ptr)
{

}

// "/rpc/motion_control/axis_group/doGotoPointManualMove"
void ControllerRpc::handleRpc0x00009B75(void* request_data_ptr, void* response_data_ptr)
{

}

// "/rpc/motion_control/axis_group/getJointFeedBack"
void ControllerRpc::handleRpc0x0000DFBB(void* request_data_ptr, void* response_data_ptr)
{

}

