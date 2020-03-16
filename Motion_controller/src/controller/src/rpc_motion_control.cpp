#include "controller_rpc.h"
#include "basic_alg_datatype.h"
#include "basic_constants.h"
#include "motion_control_datatype.h"
#include "error_code.h"
#include <cstring>
#include <math.h>

using namespace fst_ctrl;
using namespace fst_mc;
using namespace basic_alg;

// "/rpc/motion_control/stop"
void ControllerRpc::handleRpc0x00001E70(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    
    rs_data_ptr->data.data = motion_control_ptr_->stopGroup();

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/stop"));
}

// "/rpc/motion_control/reset"
void ControllerRpc::handleRpc0x00001D14(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->resetGroup();

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/reset"));    
}

// "/rpc/motion_control/setGlobalVelRatio"
void ControllerRpc::handleRpc0x000005EF(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Double* rq_data_ptr = static_cast<RequestMessageType_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        && state_machine_ptr_->getParam()->enable_set_vel_in_auto_ == false)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_SET_VEL;
    }

    rs_data_ptr->data.data = motion_control_ptr_->setGlobalVelRatio(rq_data_ptr->data.data);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/setGlobalVelRatio"));
}

// "/rpc/motion_control/getGlobalVelRatio"
void ControllerRpc::handleRpc0x0001578F(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);
    
    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = motion_control_ptr_->getGlobalVelRatio();

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/getGlobalVelRatio"));
}

// "/rpc/motion_control/setGlobalAccRatio"
void ControllerRpc::handleRpc0x0000271F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Double* rq_data_ptr = static_cast<RequestMessageType_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->setGlobalAccRatio(rq_data_ptr->data.data);

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/setGlobalAccRatio"));
}

// "/rpc/motion_control/getGlobalAccRatio"
void ControllerRpc::handleRpc0x00016D9F(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);
    
    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = motion_control_ptr_->getGlobalAccRatio();

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/getGlobalAccRatio"));
}

// "/rpc/motion_control/getAxisGroupInfoList"
void ControllerRpc::handleRpc0x00010F54(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_AxisGroupInfoList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_AxisGroupInfoList*>(response_data_ptr);

    std::string model_name = motion_control_ptr_->getModelName();
    size_t num = motion_control_ptr_->getNumberOfAxis();
    AxisType type[9] = {ROTARY_AXIS};
    motion_control_ptr_->getTypeOfAxis(&type[0]);

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.info_list_count = 1;
    rs_data_ptr->data.info_list[0].group_id = 1;
    rs_data_ptr->data.info_list[0].group_type = 0;
    strncpy(&rs_data_ptr->data.info_list[0].model_name[0], model_name.c_str(), 31);//model name
    rs_data_ptr->data.info_list[0].model_name[31] = 0;
    rs_data_ptr->data.info_list[0].axis_number = num;//axis number
    rs_data_ptr->data.info_list[0].axis_info_list_count = 9;

    size_t index = 0;
    for (index = 0; index < num; ++index)
    {
        rs_data_ptr->data.info_list[0].axis_info_list[index].axis_id = index + 1;
        rs_data_ptr->data.info_list[0].axis_info_list[index].axis_type = type[index];
    }
    for (; index < 9; ++index)
    {
        rs_data_ptr->data.info_list[0].axis_info_list[index].axis_id = -1;
        rs_data_ptr->data.info_list[0].axis_info_list[index].axis_type = -1;
    }
    
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/getAxisGroupInfoList"));
}

// "/rpc/motion_control/axis_group/doStepManualMove"
void ControllerRpc::handleRpc0x000085D5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED
        || state_machine_ptr_->getServoState() != SERVO_IDLE
        || state_machine_ptr_->getRobotState() != ROBOT_IDLE
        || state_machine_ptr_->getInterpreterState() == INTERPRETER_EXECUTE)
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
        rs_data_ptr->data.data = motion_control_ptr_->doStepManualMove(direction);
        if(rs_data_ptr->data.data == SUCCESS)
        {
            state_machine_ptr_->transferRobotStateToTeaching();
        }
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/doStepManualMove"));
}

// "/rpc/motion_control/axis_group/doContinuousManualMove"
void ControllerRpc::handleRpc0x0000D3F5(void* request_data_ptr, void* response_data_ptr)
{
    FST_INFO("/rpc/motion_control/axis_group/doContinuousManualMove");
    RequestMessageType_Int32_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED
        || !state_machine_ptr_->updateContinuousManualMoveRpcTime()
        || state_machine_ptr_->getInterpreterState() == INTERPRETER_EXECUTE)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_MOVE_CONTINUOUS;
        FST_ERROR("Invalid state to manual move");
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
        rs_data_ptr->data.data = motion_control_ptr_->doContinuousManualMove(direction);
        if(rs_data_ptr->data.data == SUCCESS)
        {
            state_machine_ptr_->transferRobotStateToTeaching();
        }
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
    
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/doContinuousManualMove"));
}

// "/rpc/motion_control/axis_group/doGotoCartesianPointManualMove"
void ControllerRpc::handleRpc0x00010C05(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_UFTF_PoseAndPosture* rq_data_ptr = static_cast<RequestMessageType_Int32_UFTF_PoseAndPosture*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED
        || state_machine_ptr_->getInterpreterState() == INTERPRETER_EXECUTE)
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
        rs_data_ptr->data.data = motion_control_ptr_->doGotoPointManualMove(pose_postrue, user_frame_id, tool_frame_id);
        if(rs_data_ptr->data.data == SUCCESS)
        {
            state_machine_ptr_->transferRobotStateToTeaching();
        }
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/doGotoCartesianPointManualMove"));
}

// "/rpc/motion_control/axis_group/doGotoJointPointManualMove"
void ControllerRpc::handleRpc0x00008075(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED
        || state_machine_ptr_->getInterpreterState() == INTERPRETER_EXECUTE)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_GOTO_JOINT;
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
        rs_data_ptr->data.data = motion_control_ptr_->doGotoPointManualMove(joint);
        if(rs_data_ptr->data.data == SUCCESS)
        {
            state_machine_ptr_->transferRobotStateToTeaching();
        }
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/doGotoJointPointManualMove"));
}

// "/rpc/motion_control/axis_group/doManualStop"
void ControllerRpc::handleRpc0x0000A9A0(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() == USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION_MANUAL_STOP;
        return;
    }

    rs_data_ptr->data.data = motion_control_ptr_->manualStop();
    
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/doManualStop"));
}

// "/rpc/motion_control/axis_group/getJointFeedBack"
void ControllerRpc::handleRpc0x0000DFBB(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
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

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getJointFeedBack"));
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
        rs_data_ptr->data.data = motion_control_ptr_->setSoftConstraint(constraint);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setUserSoftLimit"));
}

// "/rpc/motion_control/axis_group/getUserSoftLimit"
void ControllerRpc::handleRpc0x0000C764(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimit*>(response_data_ptr);

    JointConstraint constraint;
    rs_data_ptr->error_code.data = motion_control_ptr_->getSoftConstraint(constraint);
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

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getUserSoftLimit"));
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
        rs_data_ptr->data.data = motion_control_ptr_->setFirmConstraint(constraint);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setManuSoftLimit"));
}

// "/rpc/motion_control/axis_group/getManuSoftLimit"
void ControllerRpc::handleRpc0x0000C244(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimit*>(response_data_ptr);

    JointConstraint constraint;
    rs_data_ptr->error_code.data = motion_control_ptr_->getFirmConstraint(constraint);
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

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getManuSoftLimit"));
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
        rs_data_ptr->data.data = motion_control_ptr_->setHardConstraint(constraint);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setHardLimit"));
}

// "/rpc/motion_control/axis_group/getHardLimit"
void ControllerRpc::handleRpc0x00013394(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimit*>(response_data_ptr);

    JointConstraint constraint;
    rs_data_ptr->error_code.data = motion_control_ptr_->getHardConstraint(constraint);
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

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getHardLimit"));
}

// "/rpc/motion_control/axis_group/setCoordinate"
void ControllerRpc::handleRpc0x0000A845(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = motion_control_ptr_->setManualFrame((fst_mc::ManualFrame)rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setCoordinate"));
}

// "/rpc/motion_control/axis_group/getCoordinate"
void ControllerRpc::handleRpc0x00008595(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = (int32_t)motion_control_ptr_->getManualFrame();

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getCoordinate"));
}

// "/rpc/motion_control/axis_group/setUserCoordId"
void ControllerRpc::handleRpc0x00005CF4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = motion_control_ptr_->setUserFrame(rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setUserCoordId"));
}

// "/rpc/motion_control/axis_group/getUserCoordId"
void ControllerRpc::handleRpc0x00005BB4(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    motion_control_ptr_->getUserFrame(rs_data_ptr->data.data);

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getUserCoordId"));
}

// "/rpc/motion_control/axis_group/setTool"
void ControllerRpc::handleRpc0x0001581C(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = motion_control_ptr_->setToolFrame(rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setTool"));
}

// "/rpc/motion_control/axis_group/getTool"
void ControllerRpc::handleRpc0x0001354C(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    motion_control_ptr_->getToolFrame(rs_data_ptr->data.data);

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getTool"));
}

// "/rpc/motion_control/axis_group/convertCartToJoint"
void ControllerRpc::handleRpc0x00010FD4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_UFTF_PoseAndPosture* rq_data_ptr = static_cast<RequestMessageType_Int32_UFTF_PoseAndPosture*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);

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
        rs_data_ptr->error_code.data = motion_control_ptr_->convertCartToJoint(pos, user_frame_id, tool_frame_id, joint);
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

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/convertCartToJoint"));
}

// "/rpc/motion_control/axis_group/convertJointToCart"
void ControllerRpc::handleRpc0x0000B6D4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32List_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64_PoseAndPosture* rs_data_ptr = static_cast<ResponseMessageType_Uint64_PoseAndPosture*>(response_data_ptr);

    if(rq_data_ptr->data1.data_count == 3
        && rq_data_ptr->data2.data_count == 9)
    {
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
        rs_data_ptr->error_code.data = motion_control_ptr_->convertJointToCart(joint, user_frame_id, tool_frame_id, pos);
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

        Posture posture = motion_control_ptr_->getPostureFromJoint(joint);

        rs_data_ptr->data.posture.wrist_flip = posture.wrist;
        rs_data_ptr->data.posture.arm_up_down = posture.elbow;
        rs_data_ptr->data.posture.arm_back_front = posture.arm;
        rs_data_ptr->data.posture.arm_left_right = 0;

        Turn turn = motion_control_ptr_->getTurnFromJoint(joint);
        rs_data_ptr->data.posture.turn_cycle.data_count = 9; 
        memcpy(rs_data_ptr->data.posture.turn_cycle.data, &turn, 9*sizeof(int));
    }
    else
    {
        rs_data_ptr->data.pose.data_count = 6;
        rs_data_ptr->data.posture.turn_cycle.data_count = 9; 
        rs_data_ptr->error_code.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/convertJointToCart"));
}

// "/rpc/motion_control/axis_group/ignoreLostZeroError"
void ControllerRpc::handleRpc0x00014952(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->maskOffsetLostError();

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/ignoreLostZeroError"));
}

// "/rpc/motion_control/axis_group/setSingleZeroPointOffset"
void ControllerRpc::handleRpc0x00012404(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List_Double* rq_data_ptr = static_cast<RequestMessageType_Int32List_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data1.data_count == 2)
    {
        rs_data_ptr->data.data = motion_control_ptr_->setOffset(rq_data_ptr->data1.data[1], rq_data_ptr->data2.data);//data1.data[0] is groupID, data1.data[1] is axis index.
        // rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setSingleZeroPointOffset"));
}

// "/rpc/motion_control/axis_group/setAllZeroPointOffsets"
void ControllerRpc::handleRpc0x00008AB4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data2.data_count == 9)
    {
        double offset[NUM_OF_JOINT];
        memcpy(&offset[0], &rq_data_ptr->data2.data[0], NUM_OF_JOINT*sizeof(double));
        rs_data_ptr->data.data = motion_control_ptr_->setOffset(offset);
        // rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setAllZeroPointOffsets"));
}

// "/rpc/motion_control/axis_group/getAllZeroPointOffsets"
void ControllerRpc::handleRpc0x00012353(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);

    double offset[NUM_OF_JOINT];
    motion_control_ptr_->getOffset(offset);
    rs_data_ptr->data.data_count = 9;
    memcpy(&rs_data_ptr->data.data[0], &offset[0], NUM_OF_JOINT*sizeof(double));
    rs_data_ptr->error_code.data = SUCCESS;

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getAllZeroPointOffsets"));
}

// "/rpc/motion_control/axis_group/getAllZeroErrorMaskStatus"
void ControllerRpc::handleRpc0x0000C183(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    OffsetMask error_mask_state[NUM_OF_JOINT];
    motion_control_ptr_->getOffsetMask(error_mask_state);
    rs_data_ptr->data.data_count = 9;
    memcpy(&rs_data_ptr->data.data[0], &error_mask_state[0], NUM_OF_JOINT*sizeof(int));
    rs_data_ptr->error_code.data = SUCCESS;

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getAllZeroErrorMaskStatus"));
}

// "/rpc/motion_control/axis_group/saveAllZeroPointOffsets"
void ControllerRpc::handleRpc0x000171D3(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->saveOffset();

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/saveAllZeroPointOffsets"));
}

// "/rpc/motion_control/axis_group/setSingleZeroPointStatus"
void ControllerRpc::handleRpc0x00010E43(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 3)
    {
        rs_data_ptr->data.data = motion_control_ptr_->setOffsetState((size_t)rq_data_ptr->data.data[1], (OffsetState)rq_data_ptr->data.data[2]);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setSingleZeroPointStatus"));
}

// "/rpc/motion_control/axis_group/getAllZeroPointStatus"
void ControllerRpc::handleRpc0x000102F3(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    OffsetState offset_state[NUM_OF_JOINT];
    motion_control_ptr_->getOffsetState(offset_state);  
    rs_data_ptr->data.data_count = NUM_OF_JOINT;
    for(int i=0; i<NUM_OF_JOINT; ++i)
    {
        rs_data_ptr->data.data[i] = (int32_t)offset_state[i];
    }
    rs_data_ptr->error_code.data = SUCCESS;
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getAllZeroPointStatus"));
}

// "/rpc/motion_control/axis_group/calibrateAllZeroPointOffsets"
void ControllerRpc::handleRpc0x00011B03(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->calibrateOffset();

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/calibrateAllZeroPointOffsets"));
}

// "/rpc/motion_control/axis_group/calibrateSingleZeroPointOffset"
void ControllerRpc::handleRpc0x000131D4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = motion_control_ptr_->calibrateOffset((size_t)rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/calibrateSingleZeroPointOffset"));
}

// "/rpc/motion_control/axis_group/calibrateZeroPointOffsets"
void ControllerRpc::handleRpc0x00005AE3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data2.data_count == 9)
    {
        size_t axis_index[9];
        size_t size = 0;
        for(int i = 0; i < 9; ++i)
        {
            if(rq_data_ptr->data2.data[i] != -1)
            {
                axis_index[size] = (size_t)rq_data_ptr->data2.data[i];
                ++size;
            }
        }
        rs_data_ptr->data.data = motion_control_ptr_->calibrateOffset(&axis_index[0], size);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/calibrateZeroPointOffsets"));
}

// "/rpc/motion_control/axis_group/isReferencePointExist"
void ControllerRpc::handleRpc0x0000D344(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Bool* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Bool*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = motion_control_ptr_->isReferenceAvailable();

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/isReferencePointExist"));
}

// "/rpc/motion_control/axis_group/deleteReferencePoint"
void ControllerRpc::handleRpc0x00008744(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->deleteReference();

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/deleteReferencePoint"));
}

// "/rpc/motion_control/axis_group/saveReferencePoint"
void ControllerRpc::handleRpc0x00006744(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->saveReference();

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/saveReferencePoint"));
}

// "/rpc/motion_control/axis_group/fastCalibrateAllZeroPointOffsets"
void ControllerRpc::handleRpc0x0000E913(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->fastCalibrate();

    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/fastCalibrateAllZeroPointOffsets"));
}

// "/rpc/motion_control/axis_group/fastCalibrateSingleZeroPointOffset"
void ControllerRpc::handleRpc0x00004754(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = motion_control_ptr_->fastCalibrate((size_t)rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/fastCalibrateSingleZeroPointOffset"));
}

// "/rpc/motion_control/axis_group/fastCalibrateZeroPointOffsets"
void ControllerRpc::handleRpc0x00007EC3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data2.data_count == 9)
    {
        size_t axis_index[9];
        size_t size = 0;
        for(int i = 0; i < 9; ++i)
        {
            if(rq_data_ptr->data2.data[i] != -1)
            {
                axis_index[size] = (size_t)rq_data_ptr->data2.data[i];
                ++size;
            }
        }
        rs_data_ptr->data.data = motion_control_ptr_->fastCalibrate(&axis_index[0], size);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/fastCalibrateZeroPointOffsets"));
}

// "/rpc/motion_control/axis_group/getUserSoftLimitWithUnit"
void ControllerRpc::handleRpc0x00008ED4(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimitWithUnit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimitWithUnit*>(response_data_ptr);

    AxisType type[9] = {ROTARY_AXIS};
    motion_control_ptr_->getTypeOfAxis(&type[0]);

    JointConstraint constraint;
    rs_data_ptr->error_code.data = motion_control_ptr_->getSoftConstraint(constraint);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.positive_list_count = 9;
        rs_data_ptr->data.positive_list[0].data = constraint.upper.j1_;
        rs_data_ptr->data.positive_list[1].data = constraint.upper.j2_;
        rs_data_ptr->data.positive_list[2].data = constraint.upper.j3_;
        rs_data_ptr->data.positive_list[3].data = constraint.upper.j4_;
        rs_data_ptr->data.positive_list[4].data = constraint.upper.j5_;
        rs_data_ptr->data.positive_list[5].data = constraint.upper.j6_;
        rs_data_ptr->data.positive_list[6].data = constraint.upper.j7_;
        rs_data_ptr->data.positive_list[7].data = constraint.upper.j8_;
        rs_data_ptr->data.positive_list[8].data = constraint.upper.j9_;
        for(size_t i = 0; i < rs_data_ptr->data.positive_list_count; ++i)
        {
            if (type[i] == ROTARY_AXIS)
            {
                strncpy(rs_data_ptr->data.positive_list[i].unit, "rad", 31); rs_data_ptr->data.positive_list[i].unit[31] = 0;
            }
            else
            {
                strncpy(rs_data_ptr->data.positive_list[i].unit, "mm", 31); rs_data_ptr->data.positive_list[i].unit[31] = 0;
            }
        }
        
        rs_data_ptr->data.negative_list_count = 9;
        rs_data_ptr->data.negative_list[0].data = constraint.lower.j1_;
        rs_data_ptr->data.negative_list[1].data = constraint.lower.j2_;
        rs_data_ptr->data.negative_list[2].data = constraint.lower.j3_;
        rs_data_ptr->data.negative_list[3].data = constraint.lower.j4_;
        rs_data_ptr->data.negative_list[4].data = constraint.lower.j5_;
        rs_data_ptr->data.negative_list[5].data = constraint.lower.j6_;
        rs_data_ptr->data.negative_list[6].data = constraint.lower.j7_;
        rs_data_ptr->data.negative_list[7].data = constraint.lower.j8_;
        rs_data_ptr->data.negative_list[8].data = constraint.lower.j9_;
        for(size_t i = 0; i < rs_data_ptr->data.positive_list_count; ++i)
        {
            if (type[i] == ROTARY_AXIS)
            {
                strncpy(rs_data_ptr->data.negative_list[i].unit, "rad", 31); rs_data_ptr->data.negative_list[i].unit[31] = 0;
            }
            else
            {
                strncpy(rs_data_ptr->data.negative_list[i].unit, "mm", 31); rs_data_ptr->data.negative_list[i].unit[31] = 0;
            }
        }
        
    }
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getUserSoftLimit"));
}

// "/rpc/motion_control/axis_group/getManuSoftLimitWithUnit"
void ControllerRpc::handleRpc0x000124E4(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimitWithUnit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimitWithUnit*>(response_data_ptr);

    AxisType type[9] = {ROTARY_AXIS};
    motion_control_ptr_->getTypeOfAxis(&type[0]);

    JointConstraint constraint;
    rs_data_ptr->error_code.data = motion_control_ptr_->getFirmConstraint(constraint);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.positive_list_count = 9;
        rs_data_ptr->data.positive_list[0].data = constraint.upper.j1_;
        rs_data_ptr->data.positive_list[1].data = constraint.upper.j2_;
        rs_data_ptr->data.positive_list[2].data = constraint.upper.j3_;
        rs_data_ptr->data.positive_list[3].data = constraint.upper.j4_;
        rs_data_ptr->data.positive_list[4].data = constraint.upper.j5_;
        rs_data_ptr->data.positive_list[5].data = constraint.upper.j6_;
        rs_data_ptr->data.positive_list[6].data = constraint.upper.j7_;
        rs_data_ptr->data.positive_list[7].data = constraint.upper.j8_;
        rs_data_ptr->data.positive_list[8].data = constraint.upper.j9_;
        for(size_t i = 0; i < rs_data_ptr->data.positive_list_count; ++i)
        {
            if (type[i] == ROTARY_AXIS)
            {
                strncpy(rs_data_ptr->data.positive_list[i].unit, "rad", 31); rs_data_ptr->data.positive_list[i].unit[31] = 0;
            }
            else
            {
                strncpy(rs_data_ptr->data.positive_list[i].unit, "mm", 31); rs_data_ptr->data.positive_list[i].unit[31] = 0;
            }
        }
        
        rs_data_ptr->data.negative_list_count = 9;
        rs_data_ptr->data.negative_list[0].data = constraint.lower.j1_;
        rs_data_ptr->data.negative_list[1].data = constraint.lower.j2_;
        rs_data_ptr->data.negative_list[2].data = constraint.lower.j3_;
        rs_data_ptr->data.negative_list[3].data = constraint.lower.j4_;
        rs_data_ptr->data.negative_list[4].data = constraint.lower.j5_;
        rs_data_ptr->data.negative_list[5].data = constraint.lower.j6_;
        rs_data_ptr->data.negative_list[6].data = constraint.lower.j7_;
        rs_data_ptr->data.negative_list[7].data = constraint.lower.j8_;
        rs_data_ptr->data.negative_list[8].data = constraint.lower.j9_;
        for(size_t i = 0; i < rs_data_ptr->data.positive_list_count; ++i)
        {
            if (type[i] == ROTARY_AXIS)
            {
                strncpy(rs_data_ptr->data.negative_list[i].unit, "rad", 31); rs_data_ptr->data.negative_list[i].unit[31] = 0;
            }
            else
            {
                strncpy(rs_data_ptr->data.negative_list[i].unit, "mm", 31); rs_data_ptr->data.negative_list[i].unit[31] = 0;
            }
        }
        
    }
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getManuSoftLimitWithUnit"));
}

// "/rpc/motion_control/axis_group/getHardLimitWithUnit"
void ControllerRpc::handleRpc0x000092B4(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_JointLimitWithUnit* rs_data_ptr = static_cast<ResponseMessageType_Uint64_JointLimitWithUnit*>(response_data_ptr);

    AxisType type[9] = {ROTARY_AXIS};
    motion_control_ptr_->getTypeOfAxis(&type[0]);

    JointConstraint constraint;
    rs_data_ptr->error_code.data = motion_control_ptr_->getHardConstraint(constraint);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.positive_list_count = 9;
        rs_data_ptr->data.positive_list[0].data = constraint.upper.j1_;
        rs_data_ptr->data.positive_list[1].data = constraint.upper.j2_;
        rs_data_ptr->data.positive_list[2].data = constraint.upper.j3_;
        rs_data_ptr->data.positive_list[3].data = constraint.upper.j4_;
        rs_data_ptr->data.positive_list[4].data = constraint.upper.j5_;
        rs_data_ptr->data.positive_list[5].data = constraint.upper.j6_;
        rs_data_ptr->data.positive_list[6].data = constraint.upper.j7_;
        rs_data_ptr->data.positive_list[7].data = constraint.upper.j8_;
        rs_data_ptr->data.positive_list[8].data = constraint.upper.j9_;
        for(size_t i = 0; i < rs_data_ptr->data.positive_list_count; ++i)
        {
            if (type[i] == ROTARY_AXIS)
            {
                strncpy(rs_data_ptr->data.positive_list[i].unit, "rad", 31); rs_data_ptr->data.positive_list[i].unit[31] = 0;
            }
            else
            {
                strncpy(rs_data_ptr->data.positive_list[i].unit, "mm", 31); rs_data_ptr->data.positive_list[i].unit[31] = 0;
            }
        }
        
        rs_data_ptr->data.negative_list_count = 9;
        rs_data_ptr->data.negative_list[0].data = constraint.lower.j1_;
        rs_data_ptr->data.negative_list[1].data = constraint.lower.j2_;
        rs_data_ptr->data.negative_list[2].data = constraint.lower.j3_;
        rs_data_ptr->data.negative_list[3].data = constraint.lower.j4_;
        rs_data_ptr->data.negative_list[4].data = constraint.lower.j5_;
        rs_data_ptr->data.negative_list[5].data = constraint.lower.j6_;
        rs_data_ptr->data.negative_list[6].data = constraint.lower.j7_;
        rs_data_ptr->data.negative_list[7].data = constraint.lower.j8_;
        rs_data_ptr->data.negative_list[8].data = constraint.lower.j9_;
        for(size_t i = 0; i < rs_data_ptr->data.positive_list_count; ++i)
        {
            if (type[i] == ROTARY_AXIS)
            {
                strncpy(rs_data_ptr->data.negative_list[i].unit, "rad", 31); rs_data_ptr->data.negative_list[i].unit[31] = 0;
            }
            else
            {
                strncpy(rs_data_ptr->data.negative_list[i].unit, "mm", 31); rs_data_ptr->data.negative_list[i].unit[31] = 0;
            }
        }
        
    }
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getHardLimitWithUnit"));
}

//todo delete
// "/rpc/motion_control/axis_group/setRotateManualStep"
void ControllerRpc::handleRpc0x00005290(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32_Double* rq_data_ptr = static_cast<RequestMessageType_Int32_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    //rs_data_ptr->data.data = motion_control_ptr_->setRotateManualStep(rq_data_ptr->data2.data);
    rs_data_ptr->data.data = CONTROLLER_INVALID_ARG;
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setRotateManualStep"));
}

//todo delete
// "/rpc/motion_control/axis_group/getRotateManualStep"
void ControllerRpc::handleRpc0x00003000(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);

    rs_data_ptr->error_code.data = CONTROLLER_INVALID_ARG;
    //rs_data_ptr->data.data = motion_control_ptr_->getRotateManualStep();
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getRotateManualStep"));
}

//todo delete
// "/rpc/motion_control/axis_group/setPrismaticManualStep"
void ControllerRpc::handleRpc0x0000B640(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32_Double* rq_data_ptr = static_cast<RequestMessageType_Int32_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    //rs_data_ptr->data.data = motion_control_ptr_->setPrismaticManualStep(rq_data_ptr->data2.data);
    rs_data_ptr->data.data = CONTROLLER_INVALID_ARG;
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setPrismaticManualStep"));
}

//todo delete
// "/rpc/motion_control/axis_group/getPrismaticManualStep"
void ControllerRpc::handleRpc0x0000FCE0(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);

    rs_data_ptr->error_code.data = CONTROLLER_INVALID_ARG;
    //rs_data_ptr->data.data = motion_control_ptr_->getPrismaticManualStep();
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getPrismaticManualStep"));
}

//"/rpc/motion_control/axis_group/setJointManualStep"	
void ControllerRpc::handleRpc0x00018470(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    double steps[NUM_OF_JOINT] = {0};
    
    int num = (rq_data_ptr->data2.data_count < NUM_OF_JOINT ? rq_data_ptr->data2.data_count : NUM_OF_JOINT);
    for (int i = 0;i < num; ++i)
    {
        steps[i] = rq_data_ptr->data2.data[i];
    }
    rs_data_ptr->data.data = motion_control_ptr_->setAxisManualStep(steps);
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setJointManualStep"));
}

//"/rpc/motion_control/axis_group/getJointManualStep"	
void ControllerRpc::handleRpc0x00006D10(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);
    double steps[NUM_OF_JOINT] = {0};
    motion_control_ptr_->getAxisManualStep(steps);
    rs_data_ptr->data.data_count = motion_control_ptr_->getNumberOfAxis();
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

    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getJointManualStep"));
}

// "/rpc/motion_control/axis_group/setCartesianManualStep"
void ControllerRpc::handleRpc0x0000A420(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Double* rq_data_ptr = static_cast<RequestMessageType_Int32_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->setPositionManualStep(rq_data_ptr->data2.data);
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setCartesianManualStep"));
}

// "/rpc/motion_control/axis_group/getCartesianManualStep"
void ControllerRpc::handleRpc0x0000EAC0(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = motion_control_ptr_->getPositionManualStep();
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getCartesianManualStep"));
}

// "/rpc/motion_control/axis_group/setOrientationManualStep"
void ControllerRpc::handleRpc0x00002940(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Double* rq_data_ptr = static_cast<RequestMessageType_Int32_Double*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->setOrientationManualStep(rq_data_ptr->data2.data);
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setOrientationManualStep"));
}

// "/rpc/motion_control/axis_group/getOrientationManualStep"
void ControllerRpc::handleRpc0x00016D20(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Double* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Double*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = motion_control_ptr_->getOrientationManualStep();
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getOrientationManualStep"));
}

//"/rpc/motion_control/axis_group/getFcpBasePose"	
void ControllerRpc::handleRpc0x000016B5(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);

    Joint joint_feedback = motion_control_ptr_->getServoJoint();
    PoseEuler pose;
    memset(&pose, 0, sizeof(pose));
    rs_data_ptr->error_code.data = motion_control_ptr_->convertJointToCart(joint_feedback, 0, 0, pose);
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
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getFcpBasePose"));
}

//"/rpc/motion_control/axis_group/getTcpCurrentPose"	
void ControllerRpc::handleRpc0x00003B45(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64_DoubleList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_DoubleList*>(response_data_ptr);

    Joint joint_feedback = motion_control_ptr_->getServoJoint();
    PoseEuler pose;
    memset(&pose, 0, sizeof(pose));

    if(rq_data_ptr->data.data_count == 2)
    {
        int user_frame_id = rq_data_ptr->data.data[1];//data[1] is uf, data[0] is group id.
        int tool_frame_id = 0;
        //motion_control_ptr_->getUserFrame(user_frame_id);
        motion_control_ptr_->getToolFrame(tool_frame_id);//default

        rs_data_ptr->error_code.data = motion_control_ptr_->convertJointToCart(joint_feedback, user_frame_id, tool_frame_id, pose);
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
    }
    else
    {
        rs_data_ptr->error_code.data = CONTROLLER_INVALID_ARG;
    }
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getTcpCurrentPose"));
}

//"/rpc/motion_control/getPostureByJoint"
void ControllerRpc::handleRpc0x0000EC64(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_DoubleList* rq_data_ptr = static_cast<RequestMessageType_Int32_DoubleList*>(request_data_ptr);
    ResponseMessageType_Uint64_Posture* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Posture*>(response_data_ptr);

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
        Posture posture = motion_control_ptr_->getPostureFromJoint(joint);

        rs_data_ptr->data.wrist_flip = posture.wrist;
        rs_data_ptr->data.arm_up_down = posture.elbow;
        rs_data_ptr->data.arm_back_front = posture.arm;
        rs_data_ptr->data.arm_left_right = 0;

        Turn turn = motion_control_ptr_->getTurnFromJoint(joint);
        rs_data_ptr->data.turn_cycle.data_count = 9; 
        memcpy(rs_data_ptr->data.turn_cycle.data, &turn, 9*sizeof(int));

        rs_data_ptr->error_code.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.turn_cycle.data_count = 9; 
        rs_data_ptr->error_code.data = INVALID_PARAMETER;
    }
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/getPostureByJoint"));
}


//"/rpc/motion_control/axis_group/getCurrentPayload"	
void ControllerRpc::handleRpc0x000180B4(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    motion_control_ptr_->getPayload(rs_data_ptr->data.data);
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getCurrentPayload"));
}

//"/rpc/motion_control/axis_group/setCurrentPayload"	
void ControllerRpc::handleRpc0x00014094(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = motion_control_ptr_->setPayload(rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = INVALID_PARAMETER;
    }
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setCurrentPayload"));
}

//"/rpc/motion_control/axis_group/addPayload"	
void ControllerRpc::handleRpc0x000178A4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_PayloadInfo* rq_data_ptr = static_cast<RequestMessageType_PayloadInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/motion_control/axis_group/addPayload can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
        return;
    }

    if (rq_data_ptr->data.mass_center.data_count == 3 && rq_data_ptr->data.inertia_moment.data_count == 3)
    {
        PayloadInfo info;
        info.id = rq_data_ptr->data.id;
        //memcpy(info.comment, rq_data_ptr->data.comment, strlen(rq_data_ptr->data.comment) + 1);
        strncpy(info.comment, rq_data_ptr->data.comment, 255);
        info.comment[255] = 0;
        info.is_valid = false; 
        info.m_load = rq_data_ptr->data.weight;
        info.lcx_load = rq_data_ptr->data.mass_center.data[0];
        info.lcy_load = rq_data_ptr->data.mass_center.data[1];
        info.lcz_load = rq_data_ptr->data.mass_center.data[2];
        info.Ixx_load = rq_data_ptr->data.inertia_moment.data[0];
        info.Iyy_load = rq_data_ptr->data.inertia_moment.data[1];
        info.Izz_load = rq_data_ptr->data.inertia_moment.data[2];
        rs_data_ptr->data.data = motion_control_ptr_->addPayload(info);
    }
    else
    {
        rs_data_ptr->data.data = DYNAMIC_PAYLOAD_INVALID_ARG;
    }
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/addPayload"));
    
}

//"/rpc/motion_control/axis_group/deletePayload"	
void ControllerRpc::handleRpc0x00014F84(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = motion_control_ptr_->deletePayload(rq_data_ptr->data.data);
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/deletePayload"));
}

//"/rpc/motion_control/axis_group/updatePayload"	
void ControllerRpc::handleRpc0x00017074(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_PayloadInfo* rq_data_ptr = static_cast<RequestMessageType_PayloadInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (false == state_machine_ptr_->getState())
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_INFO("/rpc/motion_control/axis_group/updatePayload can't run when backup/restore, ret = %llx\n", rs_data_ptr->data.data);
        return;
    }
    
    if (rq_data_ptr->data.mass_center.data_count == 3 && rq_data_ptr->data.inertia_moment.data_count == 3)
    { 
        PayloadInfo info;
        info.id = rq_data_ptr->data.id;
        strncpy(info.comment, rq_data_ptr->data.comment, 255);
        info.comment[255] = 0;
        info.is_valid = false;
        info.m_load = rq_data_ptr->data.weight;
        info.lcx_load = rq_data_ptr->data.mass_center.data[0];
        info.lcy_load = rq_data_ptr->data.mass_center.data[1];
        info.lcz_load = rq_data_ptr->data.mass_center.data[2];
        info.Ixx_load = rq_data_ptr->data.inertia_moment.data[0];
        info.Iyy_load = rq_data_ptr->data.inertia_moment.data[1];
        info.Izz_load = rq_data_ptr->data.inertia_moment.data[2];
  
        rs_data_ptr->data.data = motion_control_ptr_->updatePayload(info);

        //set to activate the param.
        int current_id = 0;
        motion_control_ptr_->getPayload(current_id);

        if (current_id == (int)rq_data_ptr->data.id && rs_data_ptr->data.data == SUCCESS)
        {
            rs_data_ptr->data.data = motion_control_ptr_->setPayload(current_id);
        }
    }
    else
    {
        rs_data_ptr->data.data = DYNAMIC_PAYLOAD_INVALID_ARG;
    }
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/updatePayload"));
    
}

//"/rpc/motion_control/axis_group/movePayload"	
void ControllerRpc::handleRpc0x00006CE4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = motion_control_ptr_->movePayload(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    }
    else
    {
        rs_data_ptr->data.data = DYNAMIC_PAYLOAD_INVALID_ARG;
    }
    if (rs_data_ptr->data.data != SUCCESS)
    recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/movePayload"));
}

//"/rpc/motion_control/axis_group/getPayloadInfoById"	
void ControllerRpc::handleRpc0x00010C34(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_PayloadInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_PayloadInfo*>(response_data_ptr);

    PayloadInfo info;
    rs_data_ptr->error_code.data = motion_control_ptr_->getPayloadInfoById(rq_data_ptr->data.data, info);
    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.id = info.id;
        strncpy(rs_data_ptr->data.comment, info.comment, 255);
        rs_data_ptr->data.comment[255] = 0;
        rs_data_ptr->data.weight = info.m_load;
        rs_data_ptr->data.mass_center.data_count = 3;
        rs_data_ptr->data.mass_center.data[0] = info.lcx_load;
        rs_data_ptr->data.mass_center.data[1] = info.lcy_load;
        rs_data_ptr->data.mass_center.data[2] = info.lcz_load;
        rs_data_ptr->data.inertia_moment.data_count = 3;
        rs_data_ptr->data.inertia_moment.data[0] = info.Ixx_load;
        rs_data_ptr->data.inertia_moment.data[1] = info.Iyy_load;
        rs_data_ptr->data.inertia_moment.data[2] = info.Izz_load;
    }
    if (rs_data_ptr->error_code.data != SUCCESS)
    recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getPayloadInfoById"));
    
}

//"/rpc/motion_control/axis_group/getAllValidPayloadSummaryInfo"	
void ControllerRpc::handleRpc0x00010C8F(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_PayloadSummaryList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_PayloadSummaryList*>(response_data_ptr);

    std::vector<PayloadSummaryInfo> info_list;
    motion_control_ptr_->getAllValidPayloadSummaryInfo(info_list);
    for(unsigned int i = 0; i < info_list.size(); ++i)
    {
        rs_data_ptr->data.payload_summary_info[i].id = info_list[i].id;
        strncpy(rs_data_ptr->data.payload_summary_info[i].comment, info_list[i].comment.c_str(), 255);
        rs_data_ptr->data.payload_summary_info[i].comment[255] = 0;
    }
    rs_data_ptr->data.payload_summary_info_count = info_list.size();    
    rs_data_ptr->error_code.data = SUCCESS;
    
    if (rs_data_ptr->error_code.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->error_code.data, std::string("/rpc/motion_control/axis_group/getAllValidPayloadSummaryInfo"));
    
}

//"/rpc/motion_control/axis_group/setOfflineTrajectoryFile"	
void ControllerRpc::handleRpc0x00011275(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_String* rq_data_ptr = static_cast<RequestMessageType_String*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    std::string path = rq_data_ptr->data.data;
    rs_data_ptr->data.data = motion_control_ptr_->setOfflineTrajectory(path);
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/setOfflineTrajectoryFile"));
}

//"/rpc/motion_control/axis_group/PrepareOfflineTrajectory"	
void ControllerRpc::handleRpc0x000051E9(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if(state_machine_ptr_->getUserOpMode() != USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED
        || state_machine_ptr_->getInterpreterState() != INTERPRETER_IDLE)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/PrepareOfflineTrajectory"));
        return;
    }

    rs_data_ptr->data.data = motion_control_ptr_->prepairOfflineTrajectory();
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/PrepareOfflineTrajectory"));
}

//"/rpc/motion_control/axis_group/moveOfflineTrajectory"	
void ControllerRpc::handleRpc0x0000C4D9(void* request_data_ptr, void* response_data_ptr)
{
    //RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr); 
    if(state_machine_ptr_->getUserOpMode() != USER_OP_MODE_AUTO
        || state_machine_ptr_->getUserOpMode() == USER_OP_MODE_NONE
        || state_machine_ptr_->getCtrlState() != CTRL_ENGAGED
        || state_machine_ptr_->getInterpreterState() != INTERPRETER_IDLE)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/moveOfflineTrajectory"));
        return;
    }

    rs_data_ptr->data.data = motion_control_ptr_->moveOfflineTrajectory();
    if (rs_data_ptr->data.data != SUCCESS)
        recordLog(MOTION_CONTROL_LOG, rs_data_ptr->data.data, std::string("/rpc/motion_control/axis_group/moveOfflineTrajectory"));
}

