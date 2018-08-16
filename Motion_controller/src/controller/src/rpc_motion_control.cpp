#include "controller_rpc.h"
#include "base_datatype.h"

using namespace fst_ctrl;
using namespace fst_mc;

// "/rpc/motion_control/stop"
void ControllerRpc::handleRpc0x00001E70(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    //ErrorCode error_code = motion_control_ptr_->stopGroup();
    ErrorCode error_code = 0;
    if(error_code == 0)
    {
        rs_data_ptr->data.data = true;
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

// "/rpc/motion_control/reset"
void ControllerRpc::handleRpc0x00001D14(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    //ErrorCode error_code = motion_control_ptr_->resetGroup();
    ErrorCode error_code = 0;
    if(error_code == 0)
    {
        rs_data_ptr->data.data = true;
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

// "/rpc/motion_control/axis_group/setManualFrame"
void ControllerRpc::handleRpc0x00009D05(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    //ErrorCode error_code = motion_control_ptr_->setManualFrame(rq_data_ptr->data.data[1]);
    ErrorCode error_code = 0;
    if(error_code == 0)
    {
        rs_data_ptr->data.data = true;
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

// "/rpc/motion_control/axis_group/doStepManualMove"
void ControllerRpc::handleRpc0x000085D5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    //ErrorCode error_code = motion_control_ptr_->manualMove(rq_data_ptr->data.data[1]);
    ErrorCode error_code = 0;
    if(error_code == 0)
    {
        rs_data_ptr->data.data = true;
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

// "/rpc/motion_control/axis_group/doContinuousManualMove"
void ControllerRpc::handleRpc0x0000D3F5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    //ErrorCode error_code = motion_control_ptr_->manualMove(rq_data_ptr->data.data[1]);
    ErrorCode error_code = 0;
    if(error_code == 0)
    {
        rs_data_ptr->data.data = true;
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

// "/rpc/motion_control/axis_group/doGotoPointManualMove"
void ControllerRpc::handleRpc0x000056B5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Bool* rs_data_ptr = static_cast<ResponseMessageType_Bool*>(response_data_ptr);

    //ErrorCode error_code = motion_control_ptr_->manualMove(rq_data_ptr->data.data[1]);
    ErrorCode error_code = 0;
    if(error_code == 0)
    {
        rs_data_ptr->data.data = true;
    }
    else
    {
        rs_data_ptr->data.data = false;
    }
}

// "/rpc/motion_control/axis_group/getJointFeedBack"
void ControllerRpc::handleRpc0x0000DFBB(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Bool_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Bool_DoubleList*>(response_data_ptr);

    Joint joint = motion_control_ptr_->getServoJoint();
    rs_data_ptr->data.data[0] = joint[0];
    rs_data_ptr->data.data[1] = joint[1];
    rs_data_ptr->data.data[2] = joint[2];
    rs_data_ptr->data.data[3] = joint[3];
    rs_data_ptr->data.data[4] = joint[4];
    rs_data_ptr->data.data[5] = joint[5];
    rs_data_ptr->data.data[6] = joint[6];
    rs_data_ptr->data.data[7] = joint[7];
    rs_data_ptr->data.data[8] = joint[8];
    rs_data_ptr->success.data = true;
}

// "/rpc/motion_control/axis_group/setUserSoftLimit"
void ControllerRpc::handleRpc0x000114A4(void* request_data_ptr, void* response_data_ptr)
{

}

// "/rpc/motion_control/axis_group/getUserSoftLimit"
void ControllerRpc::handleRpc0x0000C764(void* request_data_ptr, void* response_data_ptr)
{

}

// "/rpc/motion_control/axis_group/setManuSoftLimit"
void ControllerRpc::handleRpc0x000108E4(void* request_data_ptr, void* response_data_ptr)
{

}

// "/rpc/motion_control/axis_group/getManuSoftLimit"
void ControllerRpc::handleRpc0x0000C244(void* request_data_ptr, void* response_data_ptr)
{

}


