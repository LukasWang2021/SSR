#include "controller_rpc.h"
#include "basic_alg_datatype.h"
#include "basic_constants.h"
#include "motion_control_datatype.h"
#include <cstring>
#include <math.h>

using namespace basic_alg;
using namespace user_space;
using namespace log_space;
using namespace group_space;


// "/rpc/motion_control/setGlobalVelRatio"
void ControllerRpc::handleRpc0x000005EF(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Double* rq_data_ptr = static_cast<RequestMessageType_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    for(size_t i = 0; i < GROUP_NUM; ++i)
    {
        rs_data_ptr->data.data = group_ptr_[i]->setGlobalVelRatio(rq_data_ptr->data.data);
        if (rs_data_ptr->data.data != SUCCESS)
        {
            LogProducer::error("rpc", "/rpc/group/setGlobalVelRatio failed, ret = %llx", rs_data_ptr->data.data);
            return;
        }
    }
    LogProducer::info("rpc", "/rpc/group/setGlobalVelRatio success");
}

// "/rpc/motion_control/getGlobalVelRatio"
void ControllerRpc::handleRpc0x0001578F(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);
    
    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = group_ptr_[0]->getGlobalVelRatio();

    LogProducer::info("rpc", "/rpc/group/getGlobalVelRatio success");
}

// "/rpc/motion_control/setGlobalAccRatio"
void ControllerRpc::handleRpc0x0000271F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Double* rq_data_ptr = static_cast<RequestMessageType_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    for(size_t i = 0; i < GROUP_NUM; ++i)
    {
        rs_data_ptr->data.data = group_ptr_[i]->setGlobalAccRatio(rq_data_ptr->data.data);
        if (rs_data_ptr->data.data != SUCCESS)
        {
            LogProducer::error("rpc", "/rpc/group/setGlobalAccRatio failed, ret = %llx", rs_data_ptr->data.data);
            return;
        }
    }
    LogProducer::info("rpc", "/rpc/group/setGlobalAccRatio success");
}

// "/rpc/motion_control/getGlobalAccRatio"
void ControllerRpc::handleRpc0x00016D9F(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);
    
    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = group_ptr_[0]->getGlobalAccRatio();

    LogProducer::info("rpc", "/rpc/group/getGlobalAccRatio success");
}

// "/rpc/motion_control/axis_group/doStepManualMove"
void ControllerRpc::handleRpc0x000085D5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data1.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/doStepManualMove input invalid params group_id = %d", group_id);
        return;
    }

    GroupStatus_e status = GROUP_STATUS_UNKNOWN;
    bool in_position = false;
    group_ptr_[group_id]->mcGroupReadStatus(status, in_position);
    if (group_ptr_[group_id]->getWorkMode() == USER_OP_MODE_AUTO
        || group_ptr_[group_id]->getWorkMode() == USER_OP_MODE_NONE
        || status != GROUP_STATUS_STANDBY)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_MOVE_STEP;
        return;
    }

    if(rq_data_ptr->data2.data_count == 9)
    {
        GroupDirection direction;
        direction.axis1 = (ManualDirection)rq_data_ptr->data2.data[0];
        direction.axis2 = (ManualDirection)rq_data_ptr->data2.data[1];
        direction.axis3 = (ManualDirection)rq_data_ptr->data2.data[2];
        direction.axis4 = (ManualDirection)rq_data_ptr->data2.data[3];
        direction.axis5 = (ManualDirection)rq_data_ptr->data2.data[4];
        direction.axis6 = (ManualDirection)rq_data_ptr->data2.data[5];
        direction.axis7 = (ManualDirection)rq_data_ptr->data2.data[6];
        direction.axis8 = (ManualDirection)rq_data_ptr->data2.data[7];
        direction.axis9 = (ManualDirection)rq_data_ptr->data2.data[8];
        rs_data_ptr->data.data = group_ptr_[group_id]->doStepManualMove(direction);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/doStepManualMove for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/doStepManualMove for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/doContinuousManualMove"
void ControllerRpc::handleRpc0x0000D3F5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data1.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/doContinuousManualMove input invalid params group_id = %d", group_id);
        return;
    }

    GroupStatus_e status = GROUP_STATUS_UNKNOWN;
    bool in_position = false;
    group_ptr_[group_id]->mcGroupReadStatus(status, in_position);
    if (group_ptr_[group_id]->getWorkMode() == USER_OP_MODE_AUTO
        || group_ptr_[group_id]->getWorkMode() == USER_OP_MODE_NONE
        || (status != GROUP_STATUS_STANDBY && status != GROUP_STATUS_MOVING))
//TODO !state_machine_ptr_->updateContinuousManualMoveRpcTime()
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_MOVE_CONTINUOUS;
        return;
    }

    if(rq_data_ptr->data2.data_count == 9)
    {
        GroupDirection direction;
        direction.axis1 = (ManualDirection)rq_data_ptr->data2.data[0];
        direction.axis2 = (ManualDirection)rq_data_ptr->data2.data[1];
        direction.axis3 = (ManualDirection)rq_data_ptr->data2.data[2];
        direction.axis4 = (ManualDirection)rq_data_ptr->data2.data[3];
        direction.axis5 = (ManualDirection)rq_data_ptr->data2.data[4];
        direction.axis6 = (ManualDirection)rq_data_ptr->data2.data[5];
        direction.axis7 = (ManualDirection)rq_data_ptr->data2.data[6];
        direction.axis8 = (ManualDirection)rq_data_ptr->data2.data[7];
        direction.axis9 = (ManualDirection)rq_data_ptr->data2.data[8];
        rs_data_ptr->data.data = group_ptr_[group_id]->doContinuousManualMove(direction);
        //TODO if(rs_data_ptr->data.data == SUCCESS)
        // {
        //     state_machine_ptr_->transferRobotStateToTeaching();
        // }
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
    
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/doContinuousManualMove for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/doContinuousManualMove for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/doGotoCartesianPointManualMove"
void ControllerRpc::handleRpc0x00010C05(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_UFTF_PoseAndPosture* rq_data_ptr = static_cast<RequestMessageType_Int32_UFTF_PoseAndPosture*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data1.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/doGotoCartesianPointManualMove input invalid params group_id = %d", group_id);
        return;
    }
    
    GroupStatus_e status = GROUP_STATUS_UNKNOWN;
    bool in_position = false;
    group_ptr_[group_id]->mcGroupReadStatus(status, in_position);
    if (group_ptr_[group_id]->getWorkMode() == USER_OP_MODE_AUTO
        || group_ptr_[group_id]->getWorkMode() == USER_OP_MODE_NONE
        || status != GROUP_STATUS_STANDBY)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_GOTO_CARTESIAN;
        return;
    }

    if(rq_data_ptr->data2.pose_and_posture.pose.data_count == 6
      && rq_data_ptr->data2.pose_and_posture.posture.turn_cycle.data_count == 9)
    {
        PoseAndPosture pose_postrue;
        pose_postrue.pose.point_.x_ = rq_data_ptr->data2.pose_and_posture.pose.data[0];
        pose_postrue.pose.point_.y_ = rq_data_ptr->data2.pose_and_posture.pose.data[1];
        pose_postrue.pose.point_.z_ = rq_data_ptr->data2.pose_and_posture.pose.data[2];
        pose_postrue.pose.euler_.a_ = rq_data_ptr->data2.pose_and_posture.pose.data[3];
        pose_postrue.pose.euler_.b_ = rq_data_ptr->data2.pose_and_posture.pose.data[4];
        pose_postrue.pose.euler_.c_ = rq_data_ptr->data2.pose_and_posture.pose.data[5];

        pose_postrue.posture.arm = rq_data_ptr->data2.pose_and_posture.posture.arm_back_front;
        pose_postrue.posture.elbow = rq_data_ptr->data2.pose_and_posture.posture.arm_up_down;
        pose_postrue.posture.wrist = rq_data_ptr->data2.pose_and_posture.posture.wrist_flip;
        pose_postrue.posture.flip = 0;

        memcpy(&(pose_postrue.turn), rq_data_ptr->data2.pose_and_posture.posture.turn_cycle.data, 9*sizeof(int));

        int user_frame_id = rq_data_ptr->data2.uf_id.data;
        int tool_frame_id = rq_data_ptr->data2.tf_id.data;
        rs_data_ptr->data.data = group_ptr_[group_id]->doGotoPointManualMove(pose_postrue, user_frame_id, tool_frame_id);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/doGotoCartesianPointManualMove for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/doGotoCartesianPointManualMove for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/doGotoJointPointManualMove"
void ControllerRpc::handleRpc0x00008075(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data1.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/doGotoJointPointManualMove input invalid params group_id = %d", group_id);
        return;
    }

    GroupStatus_e status = GROUP_STATUS_UNKNOWN;
    bool in_position = false;
    group_ptr_[group_id]->mcGroupReadStatus(status, in_position);
    if (group_ptr_[group_id]->getWorkMode() == USER_OP_MODE_AUTO
        || group_ptr_[group_id]->getWorkMode() == USER_OP_MODE_NONE
        || status != GROUP_STATUS_STANDBY)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_GOTO_JOINT;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/doGotoJointPointManualMove  workmode=%d,status= %d", group_ptr_[group_id]->getWorkMode(),status);
        return;
    }

    if(rq_data_ptr->data2.data_count == 9)
    {
        Joint joint;
        joint.j1_ = rq_data_ptr->data2.data[0];
        joint.j2_ = rq_data_ptr->data2.data[1];
        joint.j3_ = rq_data_ptr->data2.data[2];
        joint.j4_ = rq_data_ptr->data2.data[3];
        joint.j5_ = rq_data_ptr->data2.data[4];
        joint.j6_ = rq_data_ptr->data2.data[5];
        joint.j7_ = rq_data_ptr->data2.data[6];
        joint.j8_ = rq_data_ptr->data2.data[7];
        joint.j9_ = rq_data_ptr->data2.data[8];
        rs_data_ptr->data.data = group_ptr_[group_id]->doGotoPointManualMove(joint);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
    
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/doGotoJointPointManualMove for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/doGotoJointPointManualMove for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/doManualStop"
void ControllerRpc::handleRpc0x0000A9A0(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    
    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/doManualStop input invalid params group_id = %d", group_id);
        return;
    }

    GroupStatus_e status = GROUP_STATUS_UNKNOWN;
    bool in_position = false;
    group_ptr_[group_id]->mcGroupReadStatus(status, in_position);
    if (group_ptr_[group_id]->getWorkMode() == USER_OP_MODE_AUTO
        || group_ptr_[group_id]->getWorkMode() == USER_OP_MODE_NONE
        || status == GROUP_STATUS_DISABLED || status == GROUP_STATUS_ERROR_STOP)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_MANUAL_STOP;
        return;
    }

    rs_data_ptr->data.data = group_ptr_[group_id]->pauseMove();

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/doManualStop for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/doManualStop for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/getJointFeedBack"
void ControllerRpc::handleRpc0x0000DFBB(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);
    
    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getJointFeedBack input invalid params group_id = %d", group_id);
        return;
    }

    Joint joint = group_ptr_[group_id]->getServoJoint();
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

    LogProducer::info("rpc", "/rpc/motion_control/axis_group/getJointFeedBack for group[%d] success", group_id);
}

// "/rpc/motion_control/axis_group/setUserSoftLimit"
void ControllerRpc::handleRpc0x000114A4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_JointLimit* rq_data_ptr = static_cast<RequestMessageType_Int32_JointLimit*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setUserSoftLimit input invalid params group_id = %d", group_id);
        return;
    }

    if(rq_data_ptr->limit.positive_limit.data_count == 9
        && rq_data_ptr->limit.negative_limit.data_count == 9)
    {
        JointConstraint constraint;
        constraint.upper.j1_ = rq_data_ptr->limit.positive_limit.data[0];
        constraint.upper.j2_ = rq_data_ptr->limit.positive_limit.data[1];
        constraint.upper.j3_ = rq_data_ptr->limit.positive_limit.data[2];
        constraint.upper.j4_ = rq_data_ptr->limit.positive_limit.data[3];
        constraint.upper.j5_ = rq_data_ptr->limit.positive_limit.data[4];
        constraint.upper.j6_ = rq_data_ptr->limit.positive_limit.data[5];
        constraint.upper.j7_ = rq_data_ptr->limit.positive_limit.data[6];
        constraint.upper.j8_ = rq_data_ptr->limit.positive_limit.data[7];
        constraint.upper.j9_ = rq_data_ptr->limit.positive_limit.data[8];
        constraint.lower.j1_ = rq_data_ptr->limit.negative_limit.data[0];
        constraint.lower.j2_ = rq_data_ptr->limit.negative_limit.data[1];
        constraint.lower.j3_ = rq_data_ptr->limit.negative_limit.data[2];
        constraint.lower.j4_ = rq_data_ptr->limit.negative_limit.data[3];
        constraint.lower.j5_ = rq_data_ptr->limit.negative_limit.data[4];
        constraint.lower.j6_ = rq_data_ptr->limit.negative_limit.data[5];
        constraint.lower.j7_ = rq_data_ptr->limit.negative_limit.data[6];
        constraint.lower.j8_ = rq_data_ptr->limit.negative_limit.data[7];
        constraint.lower.j9_ = rq_data_ptr->limit.negative_limit.data[8];
        rs_data_ptr->data.data = group_ptr_[group_id]->setSoftConstraint(constraint);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setUserSoftLimit for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setUserSoftLimit for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/getUserSoftLimit"
void ControllerRpc::handleRpc0x0000C764(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimit*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getUserSoftLimit input invalid params group_id = %d", group_id);
        return;
    }

    JointConstraint constraint;
    rs_data_ptr->error_code.data = group_ptr_[group_id]->getSoftConstraint(constraint);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->limit.positive_limit.data_count = 9;
        rs_data_ptr->limit.positive_limit.data[0] = constraint.upper.j1_;
        rs_data_ptr->limit.positive_limit.data[1] = constraint.upper.j2_;
        rs_data_ptr->limit.positive_limit.data[2] = constraint.upper.j3_;
        rs_data_ptr->limit.positive_limit.data[3] = constraint.upper.j4_;
        rs_data_ptr->limit.positive_limit.data[4] = constraint.upper.j5_;
        rs_data_ptr->limit.positive_limit.data[5] = constraint.upper.j6_;
        rs_data_ptr->limit.positive_limit.data[6] = constraint.upper.j7_;
        rs_data_ptr->limit.positive_limit.data[7] = constraint.upper.j8_;
        rs_data_ptr->limit.positive_limit.data[8] = constraint.upper.j9_;
        rs_data_ptr->limit.negative_limit.data_count = 9;
        rs_data_ptr->limit.negative_limit.data[0] = constraint.lower.j1_;
        rs_data_ptr->limit.negative_limit.data[1] = constraint.lower.j2_;
        rs_data_ptr->limit.negative_limit.data[2] = constraint.lower.j3_;
        rs_data_ptr->limit.negative_limit.data[3] = constraint.lower.j4_;
        rs_data_ptr->limit.negative_limit.data[4] = constraint.lower.j5_;
        rs_data_ptr->limit.negative_limit.data[5] = constraint.lower.j6_;
        rs_data_ptr->limit.negative_limit.data[6] = constraint.lower.j7_;
        rs_data_ptr->limit.negative_limit.data[7] = constraint.lower.j8_;
        rs_data_ptr->limit.negative_limit.data[8] = constraint.lower.j9_;
    }

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/getUserSoftLimit for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getUserSoftLimit for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->error_code.data);

}

// "/rpc/motion_control/axis_group/setManuSoftLimit"
void ControllerRpc::handleRpc0x000108E4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_JointLimit* rq_data_ptr = static_cast<RequestMessageType_Int32_JointLimit*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setManuSoftLimit input invalid params group_id = %d", group_id);
        return;
    }

    if(rq_data_ptr->limit.positive_limit.data_count == 9
        && rq_data_ptr->limit.negative_limit.data_count == 9)
    {
        JointConstraint constraint;
        constraint.upper.j1_ = rq_data_ptr->limit.positive_limit.data[0];
        constraint.upper.j2_ = rq_data_ptr->limit.positive_limit.data[1];
        constraint.upper.j3_ = rq_data_ptr->limit.positive_limit.data[2];
        constraint.upper.j4_ = rq_data_ptr->limit.positive_limit.data[3];
        constraint.upper.j5_ = rq_data_ptr->limit.positive_limit.data[4];
        constraint.upper.j6_ = rq_data_ptr->limit.positive_limit.data[5];
        constraint.upper.j7_ = rq_data_ptr->limit.positive_limit.data[6];
        constraint.upper.j8_ = rq_data_ptr->limit.positive_limit.data[7];
        constraint.upper.j9_ = rq_data_ptr->limit.positive_limit.data[8];
        constraint.lower.j1_ = rq_data_ptr->limit.negative_limit.data[0];
        constraint.lower.j2_ = rq_data_ptr->limit.negative_limit.data[1];
        constraint.lower.j3_ = rq_data_ptr->limit.negative_limit.data[2];
        constraint.lower.j4_ = rq_data_ptr->limit.negative_limit.data[3];
        constraint.lower.j5_ = rq_data_ptr->limit.negative_limit.data[4];
        constraint.lower.j6_ = rq_data_ptr->limit.negative_limit.data[5];
        constraint.lower.j7_ = rq_data_ptr->limit.negative_limit.data[6];
        constraint.lower.j8_ = rq_data_ptr->limit.negative_limit.data[7];
        constraint.lower.j9_ = rq_data_ptr->limit.negative_limit.data[8];
        rs_data_ptr->data.data = group_ptr_[group_id]->setFirmConstraint(constraint);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setManuSoftLimit for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setManuSoftLimit for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/getManuSoftLimit"
void ControllerRpc::handleRpc0x0000C244(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimit*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getManuSoftLimit input invalid params group_id = %d", group_id);
        return;
    }
    
    JointConstraint constraint;
    rs_data_ptr->error_code.data = group_ptr_[group_id]->getFirmConstraint(constraint);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->limit.positive_limit.data_count = 9;
        rs_data_ptr->limit.positive_limit.data[0] = constraint.upper.j1_;
        rs_data_ptr->limit.positive_limit.data[1] = constraint.upper.j2_;
        rs_data_ptr->limit.positive_limit.data[2] = constraint.upper.j3_;
        rs_data_ptr->limit.positive_limit.data[3] = constraint.upper.j4_;
        rs_data_ptr->limit.positive_limit.data[4] = constraint.upper.j5_;
        rs_data_ptr->limit.positive_limit.data[5] = constraint.upper.j6_;
        rs_data_ptr->limit.positive_limit.data[6] = constraint.upper.j7_;
        rs_data_ptr->limit.positive_limit.data[7] = constraint.upper.j8_;
        rs_data_ptr->limit.positive_limit.data[8] = constraint.upper.j9_;
        rs_data_ptr->limit.negative_limit.data_count = 9;
        rs_data_ptr->limit.negative_limit.data[0] = constraint.lower.j1_;
        rs_data_ptr->limit.negative_limit.data[1] = constraint.lower.j2_;
        rs_data_ptr->limit.negative_limit.data[2] = constraint.lower.j3_;
        rs_data_ptr->limit.negative_limit.data[3] = constraint.lower.j4_;
        rs_data_ptr->limit.negative_limit.data[4] = constraint.lower.j5_;
        rs_data_ptr->limit.negative_limit.data[5] = constraint.lower.j6_;
        rs_data_ptr->limit.negative_limit.data[6] = constraint.lower.j7_;
        rs_data_ptr->limit.negative_limit.data[7] = constraint.lower.j8_;
        rs_data_ptr->limit.negative_limit.data[8] = constraint.lower.j9_;
    }

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/getManuSoftLimit for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getManuSoftLimit for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->error_code.data);

}

// "/rpc/motion_control/axis_group/setHardLimit"
void ControllerRpc::handleRpc0x0000C454(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_JointLimit* rq_data_ptr = static_cast<RequestMessageType_Int32_JointLimit*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setHardLimit input invalid params group_id = %d", group_id);
        return;
    }

    if(rq_data_ptr->limit.positive_limit.data_count == 9
        && rq_data_ptr->limit.negative_limit.data_count == 9)
    {
        JointConstraint constraint;
        constraint.upper.j1_ = rq_data_ptr->limit.positive_limit.data[0];
        constraint.upper.j2_ = rq_data_ptr->limit.positive_limit.data[1];
        constraint.upper.j3_ = rq_data_ptr->limit.positive_limit.data[2];
        constraint.upper.j4_ = rq_data_ptr->limit.positive_limit.data[3];
        constraint.upper.j5_ = rq_data_ptr->limit.positive_limit.data[4];
        constraint.upper.j6_ = rq_data_ptr->limit.positive_limit.data[5];
        constraint.upper.j7_ = rq_data_ptr->limit.positive_limit.data[6];
        constraint.upper.j8_ = rq_data_ptr->limit.positive_limit.data[7];
        constraint.upper.j9_ = rq_data_ptr->limit.positive_limit.data[8];
        constraint.lower.j1_ = rq_data_ptr->limit.negative_limit.data[0];
        constraint.lower.j2_ = rq_data_ptr->limit.negative_limit.data[1];
        constraint.lower.j3_ = rq_data_ptr->limit.negative_limit.data[2];
        constraint.lower.j4_ = rq_data_ptr->limit.negative_limit.data[3];
        constraint.lower.j5_ = rq_data_ptr->limit.negative_limit.data[4];
        constraint.lower.j6_ = rq_data_ptr->limit.negative_limit.data[5];
        constraint.lower.j7_ = rq_data_ptr->limit.negative_limit.data[6];
        constraint.lower.j8_ = rq_data_ptr->limit.negative_limit.data[7];
        constraint.lower.j9_ = rq_data_ptr->limit.negative_limit.data[8];
        rs_data_ptr->data.data = group_ptr_[group_id]->setHardConstraint(constraint);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setHardLimit for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setHardLimit for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/getHardLimit"
void ControllerRpc::handleRpc0x00013394(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimit*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getHardLimit input invalid params group_id = %d", group_id);
        return;
    }

    JointConstraint constraint;
    rs_data_ptr->error_code.data = group_ptr_[group_id]->getHardConstraint(constraint);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->limit.positive_limit.data_count = 9;
        rs_data_ptr->limit.positive_limit.data[0] = constraint.upper.j1_;
        rs_data_ptr->limit.positive_limit.data[1] = constraint.upper.j2_;
        rs_data_ptr->limit.positive_limit.data[2] = constraint.upper.j3_;
        rs_data_ptr->limit.positive_limit.data[3] = constraint.upper.j4_;
        rs_data_ptr->limit.positive_limit.data[4] = constraint.upper.j5_;
        rs_data_ptr->limit.positive_limit.data[5] = constraint.upper.j6_;
        rs_data_ptr->limit.positive_limit.data[6] = constraint.upper.j7_;
        rs_data_ptr->limit.positive_limit.data[7] = constraint.upper.j8_;
        rs_data_ptr->limit.positive_limit.data[8] = constraint.upper.j9_;
        rs_data_ptr->limit.negative_limit.data_count = 9;
        rs_data_ptr->limit.negative_limit.data[0] = constraint.lower.j1_;
        rs_data_ptr->limit.negative_limit.data[1] = constraint.lower.j2_;
        rs_data_ptr->limit.negative_limit.data[2] = constraint.lower.j3_;
        rs_data_ptr->limit.negative_limit.data[3] = constraint.lower.j4_;
        rs_data_ptr->limit.negative_limit.data[4] = constraint.lower.j5_;
        rs_data_ptr->limit.negative_limit.data[5] = constraint.lower.j6_;
        rs_data_ptr->limit.negative_limit.data[6] = constraint.lower.j7_;
        rs_data_ptr->limit.negative_limit.data[7] = constraint.lower.j8_;
        rs_data_ptr->limit.negative_limit.data[8] = constraint.lower.j9_;
    }

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/getHardLimit for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getHardLimit for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->error_code.data);

}

// "/rpc/motion_control/axis_group/setCoordinate"
void ControllerRpc::handleRpc0x0000A845(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setCoordinate input invalid data.");
        return;
    }

    int32_t group_id = rq_data_ptr->data.data[0];
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setCoordinate input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->data.data = group_ptr_[group_id]->setManualFrame((group_space::ManualFrame)rq_data_ptr->data.data[1]);
    
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setCoordinate for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setCoordinate for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/getCoordinate"
void ControllerRpc::handleRpc0x00008595(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getCoordinate input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = (int32_t)group_ptr_[group_id]->getManualFrame();

    LogProducer::info("rpc", "/rpc/motion_control/axis_group/getCoordinate for group[%d] success", group_id);

}

// "/rpc/motion_control/axis_group/setUserCoordId"
void ControllerRpc::handleRpc0x00005CF4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setUserCoordId input invalid data.");
        return;
    }

    int32_t group_id = rq_data_ptr->data.data[0];
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setUserCoordId input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->data.data = group_ptr_[group_id]->setUserFrame(rq_data_ptr->data.data[1]);

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setUserCoordId for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setUserCoordId for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/getUserCoordId"
void ControllerRpc::handleRpc0x00005BB4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getUserCoordId input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->error_code.data = SUCCESS;
    group_ptr_[group_id]->getUserFrame(rs_data_ptr->data.data);

    LogProducer::info("rpc", "/rpc/motion_control/axis_group/getUserCoordId for group[%d] success", group_id);

}

// "/rpc/motion_control/axis_group/setTool"
void ControllerRpc::handleRpc0x0001581C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setTool input invalid data.");
        return;
    }

    int32_t group_id = rq_data_ptr->data.data[0];
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setTool input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->data.data = group_ptr_[group_id]->setToolFrame(rq_data_ptr->data.data[1]);
 
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setTool for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setTool for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/getTool"
void ControllerRpc::handleRpc0x0001354C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getTool input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->error_code.data = SUCCESS;
    group_ptr_[group_id]->getToolFrame(rs_data_ptr->data.data);

    LogProducer::info("rpc", "/rpc/motion_control/axis_group/getTool for group[%d] success", group_id);

}


// "/rpc/motion_control/axis_group/convertCartToJoint"
void ControllerRpc::handleRpc0x00010FD4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_UFTF_PoseAndPosture* rq_data_ptr = static_cast<RequestMessageType_Int32_UFTF_PoseAndPosture*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data1.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/convertCartToJoint input invalid params group_id = %d", group_id);
        return;
    }

    if(rq_data_ptr->data2.pose_and_posture.pose.data_count == 6
      && rq_data_ptr->data2.pose_and_posture.posture.turn_cycle.data_count == 9)
    {
        PoseAndPosture pos;

        pos.pose.point_.x_ = rq_data_ptr->data2.pose_and_posture.pose.data[0];
        pos.pose.point_.y_ = rq_data_ptr->data2.pose_and_posture.pose.data[1];
        pos.pose.point_.z_ = rq_data_ptr->data2.pose_and_posture.pose.data[2];
        pos.pose.euler_.a_ = rq_data_ptr->data2.pose_and_posture.pose.data[3];
        pos.pose.euler_.b_ = rq_data_ptr->data2.pose_and_posture.pose.data[4];
        pos.pose.euler_.c_ = rq_data_ptr->data2.pose_and_posture.pose.data[5];
        
        pos.posture.arm = rq_data_ptr->data2.pose_and_posture.posture.arm_back_front;
        pos.posture.elbow = rq_data_ptr->data2.pose_and_posture.posture.arm_up_down;
        pos.posture.wrist = rq_data_ptr->data2.pose_and_posture.posture.wrist_flip;
        pos.posture.flip = 0;

        memcpy(&(pos.turn), rq_data_ptr->data2.pose_and_posture.posture.turn_cycle.data, 9*sizeof(int));

        int user_frame_id = rq_data_ptr->data2.uf_id.data;
        int tool_frame_id = rq_data_ptr->data2.tf_id.data;

        Joint joint;
        memset(&joint, 0, sizeof(joint));
        rs_data_ptr->error_code.data = group_ptr_[group_id]->convertCartToJoint(pos, user_frame_id, tool_frame_id, joint);
        rs_data_ptr->data.data_count = 9;
        if(rs_data_ptr->error_code.data == SUCCESS)
        {
            rs_data_ptr->data.data[0] = joint.j1_;
            rs_data_ptr->data.data[1] = joint.j2_;
            rs_data_ptr->data.data[2] = joint.j3_;
            rs_data_ptr->data.data[3] = joint.j4_;
            rs_data_ptr->data.data[4] = joint.j5_;
            rs_data_ptr->data.data[5] = joint.j6_;
            rs_data_ptr->data.data[6] = joint.j7_;
            rs_data_ptr->data.data[7] = joint.j8_;
            rs_data_ptr->data.data[8] = joint.j9_;
        }
    }
    else
    {
        rs_data_ptr->data.data_count = 9;
        rs_data_ptr->error_code.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/convertCartToJoint for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/convertCartToJoint for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->error_code.data);

}

// "/rpc/motion_control/axis_group/convertJointToCart"
void ControllerRpc::handleRpc0x0000B6D4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32List_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64_PoseAndPosture* rs_data_ptr = static_cast<ResponseMessageType_Uint64_PoseAndPosture*>(response_data_ptr);

    if(rq_data_ptr->data1.data_count != 3 || rq_data_ptr->data2.data_count != 9)
    {
        rs_data_ptr->data.pose.data_count = 6;
        rs_data_ptr->data.posture.turn_cycle.data_count = 9; 
        rs_data_ptr->error_code.data = INVALID_PARAMETER;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/convertJointToCart input invalid data.");
        return;
    }

    int32_t group_id = rq_data_ptr->data1.data[0];
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/convertJointToCart input invalid params group_id = %d", group_id);
        return;
    }

    Joint joint;
    PoseEuler pos;
    memset(&pos, 0, sizeof(pos));
    joint.j1_ = rq_data_ptr->data2.data[0];
    joint.j2_ = rq_data_ptr->data2.data[1];
    joint.j3_ = rq_data_ptr->data2.data[2];
    joint.j4_ = rq_data_ptr->data2.data[3];
    joint.j5_ = rq_data_ptr->data2.data[4];
    joint.j6_ = rq_data_ptr->data2.data[5];
    joint.j7_ = rq_data_ptr->data2.data[6];
    joint.j8_ = rq_data_ptr->data2.data[7];
    joint.j9_ = rq_data_ptr->data2.data[8];   
    int user_frame_id = rq_data_ptr->data1.data[2];
    int tool_frame_id = rq_data_ptr->data1.data[1]; 
    rs_data_ptr->error_code.data = group_ptr_[group_id]->convertJointToCart(joint, user_frame_id, tool_frame_id, pos);
    rs_data_ptr->data.pose.data_count = 6;
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.pose.data[0] = pos.point_.x_;
        rs_data_ptr->data.pose.data[1] = pos.point_.y_;
        rs_data_ptr->data.pose.data[2] = pos.point_.z_;
        rs_data_ptr->data.pose.data[3] = pos.euler_.a_;
        rs_data_ptr->data.pose.data[4] = pos.euler_.b_;
        rs_data_ptr->data.pose.data[5] = pos.euler_.c_;
    }

    Posture posture = group_ptr_[group_id]->getPostureFromJoint(joint);

    rs_data_ptr->data.posture.wrist_flip = posture.wrist;
    rs_data_ptr->data.posture.arm_up_down = posture.elbow;
    rs_data_ptr->data.posture.arm_back_front = posture.arm;
    rs_data_ptr->data.posture.arm_left_right = 0;

    Turn turn = group_ptr_[group_id]->getTurnFromJoint(joint);
    rs_data_ptr->data.posture.turn_cycle.data_count = 9; 
    memcpy(rs_data_ptr->data.posture.turn_cycle.data, &turn, 9*sizeof(int));

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/convertJointToCart for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/convertJointToCart for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->error_code.data);

}

// "/rpc/motion_control/axis_group/ignoreLostZeroError"
void ControllerRpc::handleRpc0x00014952(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    
    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/ignoreLostZeroError input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->data.data = group_ptr_[group_id]->maskOffsetLostError();

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/ignoreLostZeroError for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/ignoreLostZeroError for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/setSingleZeroPointOffset"
void ControllerRpc::handleRpc0x00012404(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_Double* rq_data_ptr = static_cast<RequestMessageType_Int32List_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data1.data_count != 2)
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setSingleZeroPointOffset input invalid data");
        return;
    }

    int32_t group_id = rq_data_ptr->data1.data[0];
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setSingleZeroPointOffset input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->data.data = group_ptr_[group_id]->setOffset(rq_data_ptr->data1.data[1], rq_data_ptr->data2.data);//data1.data[0] is groupID, data1.data[1] is axis index.

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setSingleZeroPointOffset for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setSingleZeroPointOffset for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/setAllZeroPointOffsets"
void ControllerRpc::handleRpc0x00011853(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data1.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setAllZeroPointOffsets input invalid params group_id = %d", group_id);
        return;
    }

    if(rq_data_ptr->data2.data_count == 9)
    {
        double offset[NUM_OF_JOINT];
        memcpy(&offset[0], &rq_data_ptr->data2.data[0], NUM_OF_JOINT*sizeof(double));
        rs_data_ptr->data.data = group_ptr_[group_id]->setOffset(offset);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setAllZeroPointOffsets for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setAllZeroPointOffsets for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/getAllZeroPointOffsets"
void ControllerRpc::handleRpc0x00012353(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getAllZeroPointOffsets input invalid params group_id = %d", group_id);
        return;
    }

    double offset[NUM_OF_JOINT];
    group_ptr_[group_id]->getOffset(offset);
    rs_data_ptr->data.data_count = 9;
    memcpy(&rs_data_ptr->data.data[0], &offset[0], NUM_OF_JOINT*sizeof(double));
    rs_data_ptr->error_code.data = SUCCESS;

    LogProducer::info("rpc", "/rpc/motion_control/axis_group/getAllZeroPointOffsets for group[%d] success", group_id);   

}

// "/rpc/motion_control/axis_group/getAllZeroErrorMaskStatus"
void ControllerRpc::handleRpc0x0000C183(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getAllZeroErrorMaskStatus input invalid params group_id = %d", group_id);
        return;
    }

    OffsetMask error_mask_state[NUM_OF_JOINT];
    group_ptr_[group_id]->getOffsetMask(error_mask_state);
    rs_data_ptr->data.data_count = 9;
    memcpy(&rs_data_ptr->data.data[0], &error_mask_state[0], NUM_OF_JOINT*sizeof(int));
    rs_data_ptr->error_code.data = SUCCESS;

    LogProducer::info("rpc", "/rpc/motion_control/axis_group/getAllZeroErrorMaskStatus for group[%d] success", group_id);   

}

// "/rpc/motion_control/axis_group/setSingleZeroPointStatus"
void ControllerRpc::handleRpc0x00010E43(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count != 3)
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setSingleZeroPointStatus input invalid data");
        return;
    }

    int32_t group_id = rq_data_ptr->data.data[0];
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setSingleZeroPointStatus input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->data.data = group_ptr_[group_id]->setOffsetState((size_t)rq_data_ptr->data.data[1], (OffsetState)rq_data_ptr->data.data[2]);
    
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setSingleZeroPointStatus for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setSingleZeroPointStatus for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/getAllZeroPointStatus"
void ControllerRpc::handleRpc0x000102F3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getAllZeroPointStatus input invalid params group_id = %d", group_id);
        return;
    }

    OffsetState offset_state[NUM_OF_JOINT];
    group_ptr_[group_id]->getOffsetState(offset_state);  
    rs_data_ptr->data.data_count = NUM_OF_JOINT;
    for(int i=0; i<NUM_OF_JOINT; ++i)
    {
        rs_data_ptr->data.data[i] = (int32_t)offset_state[i];
    }
    rs_data_ptr->error_code.data = SUCCESS;

    LogProducer::info("rpc", "/rpc/motion_control/axis_group/getAllZeroPointStatus for group[%d] success", group_id);   
}

//"/rpc/motion_control/axis_group/setJointManualStep"	
void ControllerRpc::handleRpc0x00018470(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data1.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setJointManualStep input invalid params group_id = %d", group_id);
        return;
    }

    double steps[NUM_OF_JOINT] = {0};
    
    int num = (rq_data_ptr->data2.data_count < NUM_OF_JOINT ? rq_data_ptr->data2.data_count : NUM_OF_JOINT);
    for (int i = 0;i < num; ++i)
    {
        steps[i] = rq_data_ptr->data2.data[i];
    }
    rs_data_ptr->data.data = group_ptr_[group_id]->setAxisManualStep(steps);

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setJointManualStep for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setJointManualStep for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}
//"/rpc/motion_conrtol/axis_group/setOnlineTrajectoryData"
void ControllerRpc::handleRpc0x00008A31(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_TransMatrixList* rq_data_ptr = static_cast<RequestMessageType_TransMatrixList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    int recv_matrixLen=0;
    recv_matrixLen = rq_data_ptr->data.matrices_count;
    //暂定最多接收10个矩阵
    int matrix_state[10];
    double matrix_data[160];
    
    #if 0 
    LogProducer::info("rpc-8A31","recv_matrixLen=%d",recv_matrixLen);
    for(int i=0;i<recv_matrixLen;i++)
    {
        LogProducer::info("handleRpc0x8A31","state=%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", 
        rq_data_ptr->data.matrices[i].state,
        rq_data_ptr->data.matrices[i].matrix[0],rq_data_ptr->data.matrices[i].matrix[1],rq_data_ptr->data.matrices[i].matrix[2],rq_data_ptr->data.matrices[i].matrix[3],
        rq_data_ptr->data.matrices[i].matrix[4],rq_data_ptr->data.matrices[i].matrix[5],rq_data_ptr->data.matrices[i].matrix[6],rq_data_ptr->data.matrices[i].matrix[7],
        rq_data_ptr->data.matrices[i].matrix[8],rq_data_ptr->data.matrices[i].matrix[9],rq_data_ptr->data.matrices[i].matrix[10],rq_data_ptr->data.matrices[i].matrix[11],
        rq_data_ptr->data.matrices[i].matrix[12],rq_data_ptr->data.matrices[i].matrix[13],rq_data_ptr->data.matrices[i].matrix[14],rq_data_ptr->data.matrices[i].matrix[15]);
    }
    #endif
    group_ptr_[0]->moveOnlineTrajectory();//检查运控状态是否处于ONLINE状态,如果不是则切换到ONLINE状态并初始化
    rs_data_ptr->data.data = SUCCESS;
    if (group_ptr_[0]->getWorkMode() != USER_OP_MODE_ONLINE)//检查控制器工作模式
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }
    for(int i=0;i<recv_matrixLen;i++)
    {
        matrix_state[i] = rq_data_ptr->data.matrices[i].state;
        memcpy(&matrix_data[i*16],&rq_data_ptr->data.matrices[i].matrix[0], 16*sizeof(double));
    }
    if(group_ptr_[0]->checkOnlineMoveError(0))
    {
        rs_data_ptr->data.data = group_ptr_[0]->checkOnlineMoveError(0);
        //group_ptr_[0]->OnlineMove_exceedJointLimit_pause();
        //TrjPoint endPoint_xyzabc =  group_ptr_[0]->getOnlineMoveLastWithinPoint();
        //group_ptr_[0]->OnlineMove_exceedJointLimit_pause2(endPoint_xyzabc);
        group_ptr_[0]->OnlineMove_exceedJointLimit_pause();//checkOnlineMoveError(1);//重置成功
    }
    else
    {
        rs_data_ptr->data.data = group_ptr_[0]->setOnlineVpointCache(recv_matrixLen, matrix_state, matrix_data);
    }
    #if 0
    if (rs_data_ptr->data.data == SUCCESS)
    {
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setOnlineTrajectoryData for group[0] success");
    } 
    else
    {
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setOnlineTrajectoryData for group[0] failed. Error = 0x%llx", rs_data_ptr->data.data);
    }
    #endif
}
//rpc/motion_conrtol/axis_group/setOnlineTrajectoryRatio  : 设置在线轨迹运动比例系数2022-0414
void ControllerRpc::handleRpc0x0000B35F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Double* rq_data_ptr = static_cast<RequestMessageType_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    rs_data_ptr->data.data = group_ptr_[0]->setOnlineTrajectoryRatio(rq_data_ptr->data.data);
    if (rs_data_ptr->data.data != SUCCESS)
    {
        LogProducer::error("rpc", "/rpc/group/setOnlineTrajectoryRatio failed, ret = %llx", rs_data_ptr->data.data);
        return;
    }
    LogProducer::info("rpc", "/rpc/group/setOnlineTrajectoryRatio success");
}
//"/rpc/motion_conrtol/axis_group/getOnlineTrajectoryRatio  : 获取在线轨迹运动比例系数
void ControllerRpc::handleRpc0x00004DEF(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);
    
    rs_data_ptr->error_code.data = SUCCESS;
    //rs_data_ptr->data.data = group_ptr_[0]->getGlobalAccRatio();

    LogProducer::info("rpc", "/rpc/group/getGlobalAccRatio success");
}

//"/rpc/motion_control/axis_group/getJointManualStep"	
void ControllerRpc::handleRpc0x00006D10(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);
    
    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getJointManualStep input invalid params group_id = %d", group_id);
        return;
    }
    
    double steps[NUM_OF_JOINT] = {0};
    group_ptr_[group_id]->getAxisManualStep(steps);
    rs_data_ptr->data.data_count = group_ptr_[group_id]->getNumberOfAxis();
    rs_data_ptr->data.data[0] = steps[0];
    rs_data_ptr->data.data[1] = steps[1];
    rs_data_ptr->data.data[2] = steps[2];
    rs_data_ptr->data.data[3] = steps[3];
    rs_data_ptr->data.data[4] = steps[4];
    rs_data_ptr->data.data[5] = steps[5];
    rs_data_ptr->data.data[6] = steps[6];
    rs_data_ptr->data.data[7] = steps[7];
    rs_data_ptr->data.data[8] = steps[8];
    rs_data_ptr->error_code.data = SUCCESS;

    LogProducer::info("rpc", "/rpc/motion_control/axis_group/getJointManualStep for group[%d] success", group_id);

}

// "/rpc/motion_control/axis_group/setCartesianManualStep"
void ControllerRpc::handleRpc0x0000A420(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Double* rq_data_ptr = static_cast<RequestMessageType_Int32_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data1.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setCartesianManualStep input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->data.data = group_ptr_[group_id]->setPositionManualStep(rq_data_ptr->data2.data);

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setCartesianManualStep for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setCartesianManualStep for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/getCartesianManualStep"
void ControllerRpc::handleRpc0x0000EAC0(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getCartesianManualStep input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = group_ptr_[group_id]->getPositionManualStep();

    LogProducer::info("rpc", "/rpc/motion_control/axis_group/getCartesianManualStep for group[%d] success", group_id);
}

// "/rpc/motion_control/axis_group/setOrientationManualStep"
void ControllerRpc::handleRpc0x00002940(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Double* rq_data_ptr = static_cast<RequestMessageType_Int32_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data1.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setOrientationManualStep input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->data.data = group_ptr_[group_id]->setOrientationManualStep(rq_data_ptr->data2.data);

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setOrientationManualStep for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setOrientationManualStep for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->data.data);

}

// "/rpc/motion_control/axis_group/getOrientationManualStep"
void ControllerRpc::handleRpc0x00016D20(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getOrientationManualStep input invalid params group_id = %d", group_id);
        return;
    }

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = group_ptr_[group_id]->getOrientationManualStep();

    LogProducer::info("rpc", "/rpc/motion_control/axis_group/getOrientationManualStep for group[%d] success", group_id);

}

//"/rpc/motion_control/axis_group/getFcpBasePose"	
void ControllerRpc::handleRpc0x000016B5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getFcpBasePose input invalid params group_id = %d", group_id);
        return;
    }

    Joint joint_feedback = group_ptr_[group_id]->getServoJoint();
    PoseEuler pose;
    memset(&pose, 0, sizeof(pose));
    rs_data_ptr->error_code.data = group_ptr_[group_id]->convertJointToCart(joint_feedback, 0, 0, pose);
    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data[0] = pose.point_.x_;
        rs_data_ptr->data.data[1] = pose.point_.y_;
        rs_data_ptr->data.data[2] = pose.point_.z_;
        rs_data_ptr->data.data[3] = pose.euler_.a_; 
        rs_data_ptr->data.data[4] = pose.euler_.b_;
        rs_data_ptr->data.data[5] = pose.euler_.c_; 
    }
    rs_data_ptr->data.data_count = 6; 

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/getFcpBasePose for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getFcpBasePose for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->error_code.data);

}

//"/rpc/motion_control/axis_group/getTcpCurrentPose"	
void ControllerRpc::handleRpc0x00003B45(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);

    if(rq_data_ptr->data.data_count != 2)
    {
        rs_data_ptr->error_code.data = INVALID_PARAMETER;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getTcpCurrentPose input invalid data.");
        return;
    }

    int32_t group_id = rq_data_ptr->data.data[0];
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getTcpCurrentPose input invalid params group_id = %d", group_id);
        return;
    }

    Joint joint_feedback = group_ptr_[group_id]->getServoJoint();
    PoseEuler pose;
    memset(&pose, 0, sizeof(pose));

    int user_frame_id = rq_data_ptr->data.data[1];//data[1] is uf, data[0] is group id.
    int tool_frame_id = 0;
    //motion_control_ptr_->getUserFrame(user_frame_id);
    group_ptr_[group_id]->getToolFrame(tool_frame_id);//default

    rs_data_ptr->error_code.data = group_ptr_[group_id]->convertJointToCart(joint_feedback, user_frame_id, tool_frame_id, pose);
    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data[0] = pose.point_.x_;
        rs_data_ptr->data.data[1] = pose.point_.y_;
        rs_data_ptr->data.data[2] = pose.point_.z_;
        rs_data_ptr->data.data[3] = pose.euler_.a_; 
        rs_data_ptr->data.data[4] = pose.euler_.b_;
        rs_data_ptr->data.data[5] = pose.euler_.c_; 
    }
    rs_data_ptr->data.data_count = 6;      
 
    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/getTcpCurrentPose for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getTcpCurrentPose for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->error_code.data);

}

//"/rpc/motion_control/getPostureByJoint"
void ControllerRpc::handleRpc0x0000EC64(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64_Posture* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Posture*>(response_data_ptr);

    int32_t group_id = rq_data_ptr->data1.data;
    if(group_id >= GROUP_NUM || group_id < 0)
    {
        rs_data_ptr->error_code.data = RPC_PARAM_INVALID;
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getPostureByJoint input invalid params group_id = %d", group_id);
        return;
    }

    if (rq_data_ptr->data2.data_count == 9)
    {
        Joint joint;
        joint.j1_ = rq_data_ptr->data2.data[0];
        joint.j2_ = rq_data_ptr->data2.data[1];
        joint.j3_ = rq_data_ptr->data2.data[2];
        joint.j4_ = rq_data_ptr->data2.data[3];
        joint.j5_ = rq_data_ptr->data2.data[4];
        joint.j6_ = rq_data_ptr->data2.data[5];
        joint.j7_ = rq_data_ptr->data2.data[6];
        joint.j8_ = rq_data_ptr->data2.data[7];
        joint.j9_ = rq_data_ptr->data2.data[8];
        Posture posture = group_ptr_[group_id]->getPostureFromJoint(joint);

        rs_data_ptr->data.wrist_flip = posture.wrist;
        rs_data_ptr->data.arm_up_down = posture.elbow;
        rs_data_ptr->data.arm_back_front = posture.arm;
        rs_data_ptr->data.arm_left_right = 0;

        Turn turn = group_ptr_[group_id]->getTurnFromJoint(joint);
        rs_data_ptr->data.turn_cycle.data_count = 9; 
        memcpy(rs_data_ptr->data.turn_cycle.data, &turn, 9*sizeof(int));

        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.turn_cycle.data_count = 9; 
        rs_data_ptr->error_code.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/getPostureByJoint for group[%d] success", group_id);
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getPostureByJoint for group[%d] failed. Error = 0x%llx", group_id, rs_data_ptr->error_code.data);

}

//"/rpc/motion_control/axis_group/convertEulerTraj2JointFile" 将欧拉位置姿态轨迹文件转换为轴角轨迹文件
void ControllerRpc::handleRpc0x0000E375(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    std::string path = rq_data_ptr->data.data;
    rs_data_ptr->data.data = group_ptr_[0]->convertEulerTraj2JointTraj(path);

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/convertEulerTraj2JointFile for group[0] success");
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/convertEulerTraj2JointFile for group[0] failed. Error = 0x%llx", rs_data_ptr->data.data);
}

//"/rpc/motion_control/axis_group/setOfflineTrajectoryFile"	
void ControllerRpc::handleRpc0x00011275(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    std::string path = rq_data_ptr->data.data;
    rs_data_ptr->data.data = group_ptr_[0]->setOfflineTrajectory(path);

    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/setOfflineTrajectoryFile for group[0] success");
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/setOfflineTrajectoryFile for group[0] failed. Error = 0x%llx", rs_data_ptr->data.data);

}

//"/rpc/motion_control/axis_group/PrepareOfflineTrajectory"	
void ControllerRpc::handleRpc0x000051E9(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    GroupStatus_e status = GROUP_STATUS_UNKNOWN;
    bool in_position = false;
    group_ptr_[0]->mcGroupReadStatus(status, in_position);
    
    if (group_ptr_[0]->getWorkMode() != USER_OP_MODE_AUTO
        || group_ptr_[0]->getWorkMode() == USER_OP_MODE_NONE
        || status != GROUP_STATUS_STANDBY)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        LogProducer::error("rpc", "prepare offline invalid option mode=%d, status=%d", group_ptr_[0]->getWorkMode(), status);
        return;
    }

    rs_data_ptr->data.data = group_ptr_[0]->prepareOfflineTrajectory();
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/PrepareOfflineTrajectory for group[0] success");
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/PrepareOfflineTrajectory for group[0] failed. Error = 0x%llx", rs_data_ptr->data.data);

}

//"/rpc/motion_control/axis_group/moveOfflineTrajectory"	
void ControllerRpc::handleRpc0x0000C4D9(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr); 

    GroupStatus_e status = GROUP_STATUS_UNKNOWN;
    bool in_position = false;
    group_ptr_[0]->mcGroupReadStatus(status, in_position);
    if (group_ptr_[0]->getWorkMode() != USER_OP_MODE_AUTO
        || group_ptr_[0]->getWorkMode() == USER_OP_MODE_NONE
        || status != GROUP_STATUS_STANDBY)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }

    rs_data_ptr->data.data = group_ptr_[0]->moveOfflineTrajectory();
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/moveOfflineTrajectory for group[0] success");
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/moveOfflineTrajectory for group[0] failed. Error = 0x%llx", rs_data_ptr->data.data);

}

//"/rpc/motion_control/axis_group/sendViaPoints"	
void ControllerRpc::handleRpc0x0000A063(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr); 

    PoseEuler pos;
    vector<PoseEuler> vps; // via points
    LogProducer::info("rpc", "recieved via point count %d", rq_data_ptr->data2.data_count);
    for(uint32_t i = 0; i < rq_data_ptr->data2.data_count; i+=6)
    {
        // pos.point_.x_ = rq_data_ptr->data2.data[i+0];
        // pos.point_.y_ = rq_data_ptr->data2.data[i+1];
        // pos.point_.z_ = rq_data_ptr->data2.data[i+2];
        // pos.euler_.a_ = rq_data_ptr->data2.data[i+3];
        // pos.euler_.b_ = rq_data_ptr->data2.data[i+4];
        // pos.euler_.c_ = rq_data_ptr->data2.data[i+5];
        LogProducer::info("rpc", "pos point(%lf,%lf,%lf,%lf,%lf,%lf)", 
        pos.point_.x_ = rq_data_ptr->data2.data[i+0], pos.point_.y_ = rq_data_ptr->data2.data[i+1], 
        pos.point_.z_ = rq_data_ptr->data2.data[i+2], pos.euler_.a_ = rq_data_ptr->data2.data[i+3], 
        pos.euler_.b_ = rq_data_ptr->data2.data[i+4], pos.euler_.c_ = rq_data_ptr->data2.data[i+5]);
        
        vps.push_back(pos);
    }
    bool is_new = rq_data_ptr->data1.data == 1 ? true : false;
    LogProducer::info("rpc", "new traj via points %s", is_new ? "true" : "false");
    rs_data_ptr->data.data = group_ptr_[0]->setOfflineViaPoints(vps, is_new);
    if (rs_data_ptr->data.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/sendViaPoints for group[0] success");
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/sendViaPoints for group[0] failed. Error = 0x%llx", rs_data_ptr->data.data);
}

//"/rpc/motion_control/axis_group/viaPointsToTrajWithGivenVelocity"	
void ControllerRpc::handleRpc0x0000E479(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String_Double* rq_data_ptr = static_cast<RequestMessageType_String_Double*>(request_data_ptr);
    ResponseMessageType_Uint64_String* rs_data_ptr = static_cast<ResponseMessageType_Uint64_String*>(response_data_ptr); 

    rs_data_ptr->error_code.data = group_ptr_[0]->planOfflineTrajectory(rq_data_ptr->data1.data, rq_data_ptr->data2.data);

    if (rs_data_ptr->error_code.data == SUCCESS)
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/viaPointsToTrajWithGivenVelocity for group[0] success");
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/viaPointsToTrajWithGivenVelocity for group[0] failed. Error = 0x%llx", rs_data_ptr->error_code.data);
}

//"/rpc/motion_control/axis_group/getDH"	
void ControllerRpc::handleRpc0x00016078(void* request_data_ptr, void* response_data_ptr)
{
    // RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr); 
    DH base_dh, arm_dh[6];
    rs_data_ptr->error_code.data = group_ptr_[0]->getDH(base_dh, arm_dh);
    rs_data_ptr->data.data_count = 4 * 7;
    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data[0] = base_dh.d;
        rs_data_ptr->data.data[1] = base_dh.a;
        rs_data_ptr->data.data[2] = base_dh.alpha;
        rs_data_ptr->data.data[3] = base_dh.offset;

        for(size_t i = 0; i < 6; ++i)
        {
            rs_data_ptr->data.data[i * 4 + 4] = arm_dh[i].d;
            rs_data_ptr->data.data[i * 4 + 5] = arm_dh[i].a;
            rs_data_ptr->data.data[i * 4 + 6] = arm_dh[i].alpha;
            rs_data_ptr->data.data[i * 4 + 7] = arm_dh[i].offset;
        }

        // memcpy(rs_data_ptr->data.data, &base_dh, sizeof(DH) * 7);
        LogProducer::info("rpc", "/rpc/motion_control/axis_group/getDH for group[0] success");
        LogProducer::info("rpc", "base:%lf,%lf,%lf,%lf", base_dh.d, base_dh.a, base_dh.alpha, base_dh.offset);
        LogProducer::info("rpc", "arm1:%lf,%lf,%lf,%lf", arm_dh[0].d, arm_dh[0].a, arm_dh[0].alpha, arm_dh[0].offset);
        LogProducer::info("rpc", "arm2:%lf,%lf,%lf,%lf", arm_dh[1].d, arm_dh[1].a, arm_dh[1].alpha, arm_dh[1].offset);
        LogProducer::info("rpc", "arm3:%lf,%lf,%lf,%lf", arm_dh[2].d, arm_dh[2].a, arm_dh[2].alpha, arm_dh[2].offset);
        LogProducer::info("rpc", "arm4:%lf,%lf,%lf,%lf", arm_dh[3].d, arm_dh[3].a, arm_dh[3].alpha, arm_dh[3].offset);
        LogProducer::info("rpc", "arm5:%lf,%lf,%lf,%lf", arm_dh[4].d, arm_dh[4].a, arm_dh[4].alpha, arm_dh[4].offset);
        LogProducer::info("rpc", "arm6:%lf,%lf,%lf,%lf", arm_dh[5].d, arm_dh[5].a, arm_dh[5].alpha, arm_dh[5].offset);
    }
    else
        LogProducer::error("rpc", "/rpc/motion_control/axis_group/getDH for group[0] failed. Error = 0x%llx", rs_data_ptr->error_code.data);
}
