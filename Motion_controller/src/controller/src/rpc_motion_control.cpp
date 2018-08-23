#include "controller_rpc.h"
#include "base_datatype.h"
#include "motion_control_datatype.h"
#include "error_code.h"

using namespace fst_ctrl;
using namespace fst_mc;

// "/rpc/motion_control/stop"
void ControllerRpc::handleRpc0x00001E70(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    
    rs_data_ptr->data.data = motion_control_ptr_->stopGroup();
}

// "/rpc/motion_control/reset"
void ControllerRpc::handleRpc0x00001D14(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->resetGroup();
}

// "/rpc/motion_control/setGlobalVelRatio"
void ControllerRpc::handleRpc0x000005EF(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Double* rq_data_ptr = static_cast<RequestMessageType_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = SUCCESS;
}

// "/rpc/motion_control/getGlobalVelRatio"
void ControllerRpc::handleRpc0x0001578F(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);
    
    rs_data_ptr->data.data = SUCCESS;
}

// "/rpc/motion_control/setGlobalAccRatio"
void ControllerRpc::handleRpc0x0000271F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Double* rq_data_ptr = static_cast<RequestMessageType_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = SUCCESS;
}

// "/rpc/motion_control/getGlobalAccRatio"
void ControllerRpc::handleRpc0x00016D9F(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);
    
    rs_data_ptr->data.data = SUCCESS;
}

// "/rpc/motion_control/axis_group/setManualFrame"
void ControllerRpc::handleRpc0x00009D05(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = motion_control_ptr_->setManualFrame(rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
}

// "/rpc/motion_control/axis_group/doStepManualMove"
void ControllerRpc::handleRpc0x000085D5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data2.data_count == 9)
    {
        GroupDirection direction;
        direction.axis1 = rq_data_ptr->data2.data[0];
        direction.axis2 = rq_data_ptr->data2.data[1];
        direction.axis3 = rq_data_ptr->data2.data[2];
        direction.axis4 = rq_data_ptr->data2.data[3];
        direction.axis5 = rq_data_ptr->data2.data[4];
        direction.axis6 = rq_data_ptr->data2.data[5];
        direction.axis7 = rq_data_ptr->data2.data[6];
        direction.axis8 = rq_data_ptr->data2.data[7];
        direction.axis9 = rq_data_ptr->data2.data[8];
        rs_data_ptr->data.data = motion_control_ptr_->doStepManualMove(direction);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
}

// "/rpc/motion_control/axis_group/doContinuousManualMove"
void ControllerRpc::handleRpc0x0000D3F5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data2.data_count == 9)
    {
        GroupDirection direction;
        direction.axis1 = rq_data_ptr->data2.data[0];
        direction.axis2 = rq_data_ptr->data2.data[1];
        direction.axis3 = rq_data_ptr->data2.data[2];
        direction.axis4 = rq_data_ptr->data2.data[3];
        direction.axis5 = rq_data_ptr->data2.data[4];
        direction.axis6 = rq_data_ptr->data2.data[5];
        direction.axis7 = rq_data_ptr->data2.data[6];
        direction.axis8 = rq_data_ptr->data2.data[7];
        direction.axis9 = rq_data_ptr->data2.data[8];
        rs_data_ptr->data.data = motion_control_ptr_->doContinuousManualMove(direction);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
}

// "/rpc/motion_control/axis_group/doGotoCartesianPointManualMove"
void ControllerRpc::handleRpc0x00010C05(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data2.data_count == 6)
    {
        PoseEuler pos;
        pos.position.x = rq_data_ptr->data2.data[0];
        pos.position.y = rq_data_ptr->data2.data[1];
        pos.position.z = rq_data_ptr->data2.data[2];
        pos.orientation.a = rq_data_ptr->data2.data[3];
        pos.orientation.b = rq_data_ptr->data2.data[4];
        pos.orientation.c = rq_data_ptr->data2.data[5];
        rs_data_ptr->data.data = motion_control_ptr_->doGotoPointManualMove(pos);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
}

// "/rpc/motion_control/axis_group/doGotoJointPointManualMove"
void ControllerRpc::handleRpc0x00008075(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data2.data_count == 9)
    {
        Joint joint;
        joint.j1 = rq_data_ptr->data2.data[0];
        joint.j2 = rq_data_ptr->data2.data[1];
        joint.j3 = rq_data_ptr->data2.data[2];
        joint.j4 = rq_data_ptr->data2.data[3];
        joint.j5 = rq_data_ptr->data2.data[4];
        joint.j6 = rq_data_ptr->data2.data[5];
        joint.j7 = rq_data_ptr->data2.data[6];
        joint.j8 = rq_data_ptr->data2.data[7];
        joint.j9 = rq_data_ptr->data2.data[8];
        rs_data_ptr->data.data = motion_control_ptr_->doGotoPointManualMove(joint);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
}

// "/rpc/motion_control/axis_group/doManualStop"
void ControllerRpc::handleRpc0x0000A9A0(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->manualStop();
}

// "/rpc/motion_control/axis_group/getJointFeedBack"
void ControllerRpc::handleRpc0x0000DFBB(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);

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
    rs_data_ptr->data.data_count = 9;
    rs_data_ptr->error_code.data = SUCCESS;
}

// "/rpc/motion_control/axis_group/setUserSoftLimit"
void ControllerRpc::handleRpc0x000114A4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_JointLimit* rq_data_ptr = static_cast<RequestMessageType_Int32_JointLimit*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->limit.positive_limit.data_count == 9
        && rq_data_ptr->limit.negative_limit.data_count == 9)
    {
        JointConstraint constraint;
        constraint.upper.j1 = rq_data_ptr->limit.positive_limit.data[0];
        constraint.upper.j2 = rq_data_ptr->limit.positive_limit.data[1];
        constraint.upper.j3 = rq_data_ptr->limit.positive_limit.data[2];
        constraint.upper.j4 = rq_data_ptr->limit.positive_limit.data[3];
        constraint.upper.j5 = rq_data_ptr->limit.positive_limit.data[4];
        constraint.upper.j6 = rq_data_ptr->limit.positive_limit.data[5];
        constraint.upper.j7 = rq_data_ptr->limit.positive_limit.data[6];
        constraint.upper.j8 = rq_data_ptr->limit.positive_limit.data[7];
        constraint.upper.j9 = rq_data_ptr->limit.positive_limit.data[8];
        constraint.lower.j1 = rq_data_ptr->limit.negative_limit.data[0];
        constraint.lower.j2 = rq_data_ptr->limit.negative_limit.data[1];
        constraint.lower.j3 = rq_data_ptr->limit.negative_limit.data[2];
        constraint.lower.j4 = rq_data_ptr->limit.negative_limit.data[3];
        constraint.lower.j5 = rq_data_ptr->limit.negative_limit.data[4];
        constraint.lower.j6 = rq_data_ptr->limit.negative_limit.data[5];
        constraint.lower.j7 = rq_data_ptr->limit.negative_limit.data[6];
        constraint.lower.j8 = rq_data_ptr->limit.negative_limit.data[7];
        constraint.lower.j9 = rq_data_ptr->limit.negative_limit.data[8];
        rs_data_ptr->data.data = motion_control_ptr_->setSoftConstraint(constraint);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
}

// "/rpc/motion_control/axis_group/getUserSoftLimit"
void ControllerRpc::handleRpc0x0000C764(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimit*>(response_data_ptr);

    JointConstraint constraint;
    rs_data_ptr->error_code.data = motion_control_ptr_->getSoftConstraint(constraint);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->limit.positive_limit.data_count = 9;
        rs_data_ptr->limit.positive_limit.data[0] = constraint.upper.j1;
        rs_data_ptr->limit.positive_limit.data[1] = constraint.upper.j2;
        rs_data_ptr->limit.positive_limit.data[2] = constraint.upper.j3;
        rs_data_ptr->limit.positive_limit.data[3] = constraint.upper.j4;
        rs_data_ptr->limit.positive_limit.data[4] = constraint.upper.j5;
        rs_data_ptr->limit.positive_limit.data[5] = constraint.upper.j6;
        rs_data_ptr->limit.positive_limit.data[6] = constraint.upper.j7;
        rs_data_ptr->limit.positive_limit.data[7] = constraint.upper.j8;
        rs_data_ptr->limit.positive_limit.data[8] = constraint.upper.j9;
        rs_data_ptr->limit.negative_limit.data_count = 9;
        rs_data_ptr->limit.negative_limit.data[0] = constraint.lower.j1;
        rs_data_ptr->limit.negative_limit.data[1] = constraint.lower.j2;
        rs_data_ptr->limit.negative_limit.data[2] = constraint.lower.j3;
        rs_data_ptr->limit.negative_limit.data[3] = constraint.lower.j4;
        rs_data_ptr->limit.negative_limit.data[4] = constraint.lower.j5;
        rs_data_ptr->limit.negative_limit.data[5] = constraint.lower.j6;
        rs_data_ptr->limit.negative_limit.data[6] = constraint.lower.j7;
        rs_data_ptr->limit.negative_limit.data[7] = constraint.lower.j8;
        rs_data_ptr->limit.negative_limit.data[8] = constraint.lower.j9;
    }
}

// "/rpc/motion_control/axis_group/setManuSoftLimit"
void ControllerRpc::handleRpc0x000108E4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_JointLimit* rq_data_ptr = static_cast<RequestMessageType_Int32_JointLimit*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->limit.positive_limit.data_count == 9
        && rq_data_ptr->limit.negative_limit.data_count == 9)
    {
        JointConstraint constraint;
        constraint.upper.j1 = rq_data_ptr->limit.positive_limit.data[0];
        constraint.upper.j2 = rq_data_ptr->limit.positive_limit.data[1];
        constraint.upper.j3 = rq_data_ptr->limit.positive_limit.data[2];
        constraint.upper.j4 = rq_data_ptr->limit.positive_limit.data[3];
        constraint.upper.j5 = rq_data_ptr->limit.positive_limit.data[4];
        constraint.upper.j6 = rq_data_ptr->limit.positive_limit.data[5];
        constraint.upper.j7 = rq_data_ptr->limit.positive_limit.data[6];
        constraint.upper.j8 = rq_data_ptr->limit.positive_limit.data[7];
        constraint.upper.j9 = rq_data_ptr->limit.positive_limit.data[8];
        constraint.lower.j1 = rq_data_ptr->limit.negative_limit.data[0];
        constraint.lower.j2 = rq_data_ptr->limit.negative_limit.data[1];
        constraint.lower.j3 = rq_data_ptr->limit.negative_limit.data[2];
        constraint.lower.j4 = rq_data_ptr->limit.negative_limit.data[3];
        constraint.lower.j5 = rq_data_ptr->limit.negative_limit.data[4];
        constraint.lower.j6 = rq_data_ptr->limit.negative_limit.data[5];
        constraint.lower.j7 = rq_data_ptr->limit.negative_limit.data[6];
        constraint.lower.j8 = rq_data_ptr->limit.negative_limit.data[7];
        constraint.lower.j9 = rq_data_ptr->limit.negative_limit.data[8];
        rs_data_ptr->data.data = motion_control_ptr_->setFirmConstraint(constraint);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
}

// "/rpc/motion_control/axis_group/getManuSoftLimit"
void ControllerRpc::handleRpc0x0000C244(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimit*>(response_data_ptr);

    JointConstraint constraint;
    rs_data_ptr->error_code.data = motion_control_ptr_->getFirmConstraint(constraint);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->limit.positive_limit.data_count = 9;
        rs_data_ptr->limit.positive_limit.data[0] = constraint.upper.j1;
        rs_data_ptr->limit.positive_limit.data[1] = constraint.upper.j2;
        rs_data_ptr->limit.positive_limit.data[2] = constraint.upper.j3;
        rs_data_ptr->limit.positive_limit.data[3] = constraint.upper.j4;
        rs_data_ptr->limit.positive_limit.data[4] = constraint.upper.j5;
        rs_data_ptr->limit.positive_limit.data[5] = constraint.upper.j6;
        rs_data_ptr->limit.positive_limit.data[6] = constraint.upper.j7;
        rs_data_ptr->limit.positive_limit.data[7] = constraint.upper.j8;
        rs_data_ptr->limit.positive_limit.data[8] = constraint.upper.j9;
        rs_data_ptr->limit.negative_limit.data_count = 9;
        rs_data_ptr->limit.negative_limit.data[0] = constraint.lower.j1;
        rs_data_ptr->limit.negative_limit.data[1] = constraint.lower.j2;
        rs_data_ptr->limit.negative_limit.data[2] = constraint.lower.j3;
        rs_data_ptr->limit.negative_limit.data[3] = constraint.lower.j4;
        rs_data_ptr->limit.negative_limit.data[4] = constraint.lower.j5;
        rs_data_ptr->limit.negative_limit.data[5] = constraint.lower.j6;
        rs_data_ptr->limit.negative_limit.data[6] = constraint.lower.j7;
        rs_data_ptr->limit.negative_limit.data[7] = constraint.lower.j8;
        rs_data_ptr->limit.negative_limit.data[8] = constraint.lower.j9;
    }
}

// "/rpc/motion_control/axis_group/setHardLimit"
void ControllerRpc::handleRpc0x0000C454(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_JointLimit* rq_data_ptr = static_cast<RequestMessageType_Int32_JointLimit*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->limit.positive_limit.data_count == 9
        && rq_data_ptr->limit.negative_limit.data_count == 9)
    {
        JointConstraint constraint;
        constraint.upper.j1 = rq_data_ptr->limit.positive_limit.data[0];
        constraint.upper.j2 = rq_data_ptr->limit.positive_limit.data[1];
        constraint.upper.j3 = rq_data_ptr->limit.positive_limit.data[2];
        constraint.upper.j4 = rq_data_ptr->limit.positive_limit.data[3];
        constraint.upper.j5 = rq_data_ptr->limit.positive_limit.data[4];
        constraint.upper.j6 = rq_data_ptr->limit.positive_limit.data[5];
        constraint.upper.j7 = rq_data_ptr->limit.positive_limit.data[6];
        constraint.upper.j8 = rq_data_ptr->limit.positive_limit.data[7];
        constraint.upper.j9 = rq_data_ptr->limit.positive_limit.data[8];
        constraint.lower.j1 = rq_data_ptr->limit.negative_limit.data[0];
        constraint.lower.j2 = rq_data_ptr->limit.negative_limit.data[1];
        constraint.lower.j3 = rq_data_ptr->limit.negative_limit.data[2];
        constraint.lower.j4 = rq_data_ptr->limit.negative_limit.data[3];
        constraint.lower.j5 = rq_data_ptr->limit.negative_limit.data[4];
        constraint.lower.j6 = rq_data_ptr->limit.negative_limit.data[5];
        constraint.lower.j7 = rq_data_ptr->limit.negative_limit.data[6];
        constraint.lower.j8 = rq_data_ptr->limit.negative_limit.data[7];
        constraint.lower.j9 = rq_data_ptr->limit.negative_limit.data[8];
        rs_data_ptr->data.data = motion_control_ptr_->setHardConstraint(constraint);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
}

// "/rpc/motion_control/axis_group/getHardLimit"
void ControllerRpc::handleRpc0x00013394(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimit*>(response_data_ptr);

    JointConstraint constraint;
    rs_data_ptr->error_code.data = motion_control_ptr_->getHardConstraint(constraint);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->limit.positive_limit.data_count = 9;
        rs_data_ptr->limit.positive_limit.data[0] = constraint.upper.j1;
        rs_data_ptr->limit.positive_limit.data[1] = constraint.upper.j2;
        rs_data_ptr->limit.positive_limit.data[2] = constraint.upper.j3;
        rs_data_ptr->limit.positive_limit.data[3] = constraint.upper.j4;
        rs_data_ptr->limit.positive_limit.data[4] = constraint.upper.j5;
        rs_data_ptr->limit.positive_limit.data[5] = constraint.upper.j6;
        rs_data_ptr->limit.positive_limit.data[6] = constraint.upper.j7;
        rs_data_ptr->limit.positive_limit.data[7] = constraint.upper.j8;
        rs_data_ptr->limit.positive_limit.data[8] = constraint.upper.j9;
        rs_data_ptr->limit.negative_limit.data_count = 9;
        rs_data_ptr->limit.negative_limit.data[0] = constraint.lower.j1;
        rs_data_ptr->limit.negative_limit.data[1] = constraint.lower.j2;
        rs_data_ptr->limit.negative_limit.data[2] = constraint.lower.j3;
        rs_data_ptr->limit.negative_limit.data[3] = constraint.lower.j4;
        rs_data_ptr->limit.negative_limit.data[4] = constraint.lower.j5;
        rs_data_ptr->limit.negative_limit.data[5] = constraint.lower.j6;
        rs_data_ptr->limit.negative_limit.data[6] = constraint.lower.j7;
        rs_data_ptr->limit.negative_limit.data[7] = constraint.lower.j8;
        rs_data_ptr->limit.negative_limit.data[8] = constraint.lower.j9;
    }
}

// "/rpc/motion_control/axis_group/setCoordinate"
void ControllerRpc::handleRpc0x0000A845(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 3)
    {
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
}

// "/rpc/motion_control/axis_group/getCoordinate"
void ControllerRpc::handleRpc0x00008595(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data_count = 2;
}

// "/rpc/motion_control/axis_group/setTool"
void ControllerRpc::handleRpc0x0001581C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
}

// "/rpc/motion_control/axis_group/getTool"
void ControllerRpc::handleRpc0x0001354C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
}

// "/rpc/motion_control/axis_group/convertCartToJoint"
void ControllerRpc::handleRpc0x00010FD4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32List_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);

    if(rq_data_ptr->data1.data_count == 4
        && rq_data_ptr->data2.data_count == 6)
    {
        rs_data_ptr->data.data_count = 9;
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data_count = 9;
        rs_data_ptr->error_code.data = INVALID_PARAMETER;
    }
}

// "/rpc/motion_control/axis_group/convertJointToCart"
void ControllerRpc::handleRpc0x0000B6D4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32List_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);

    if(rq_data_ptr->data1.data_count == 4
        && rq_data_ptr->data2.data_count == 9)
    {
        rs_data_ptr->data.data_count = 6;
        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data_count = 6;
        rs_data_ptr->error_code.data = INVALID_PARAMETER;
    }
}

