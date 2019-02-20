#include "controller_ipc.h"
#include "interpreter_common.h"
#include <unistd.h>
#include <stdlib.h>
#include <cstring>

using namespace fst_ctrl;
using namespace std;

void ControllerIpc::handleIpcSetInstruction(void* request_data_ptr, void* response_data_ptr)
{
    Instruction* rq_data_ptr = static_cast<Instruction*>(request_data_ptr);
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);
    
    state_machine_ptr_->getNewInstruction(rq_data_ptr);
    *rs_data_ptr = true;
}

void ControllerIpc::handleIpcIsNextInstructionNeeded(void* request_data_ptr, void* response_data_ptr)
{
    bool* rs_data_ptr = static_cast<bool*>(response_data_ptr);

    *rs_data_ptr = state_machine_ptr_->isNextInstructionNeeded();
}

void ControllerIpc::handleIpcGetJoint(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    Joint* rs_data_ptr = static_cast<Joint*>(response_data_ptr);

    *rs_data_ptr = motion_control_ptr_->getServoJoint();
}

void ControllerIpc::handleIpcGetCart(void* request_data_ptr, void* response_data_ptr)
{
    int* rq_data_ptr = static_cast<int*>(request_data_ptr);
    PoseEuler* rs_data_ptr = static_cast<PoseEuler*>(response_data_ptr);

    *rs_data_ptr = motion_control_ptr_->getCurrentPose();
}

void ControllerIpc::handleIpcCartToJoint(void* request_data_ptr, void* response_data_ptr)
{
    PoseEuler* rq_data_ptr = static_cast<PoseEuler*>(request_data_ptr);
    Joint* rs_data_ptr = static_cast<Joint*>(response_data_ptr);

    int user_frame_id = 0;
    int tool_frame_id = 0;
    motion_control_ptr_->getUserFrame(user_frame_id);
    motion_control_ptr_->getToolFrame(tool_frame_id);
    Joint joint;

    ErrorCode result = motion_control_ptr_->convertCartToJoint(*rq_data_ptr, user_frame_id, tool_frame_id, joint);
    if (result != SUCCESS)
    {
        for(int i = 0; i < 9; i++)
        {
            joint[i] = DBL_MAX;
        }
    }
    *rs_data_ptr = joint;
}

void ControllerIpc::handleIpcJointToCart(void* request_data_ptr, void* response_data_ptr)
{
    Joint* rq_data_ptr = static_cast<Joint*>(request_data_ptr);
    PoseEuler* rs_data_ptr = static_cast<PoseEuler*>(response_data_ptr);

    int user_frame_id = 0;
    int tool_frame_id = 0;
    motion_control_ptr_->getUserFrame(user_frame_id);
    motion_control_ptr_->getToolFrame(tool_frame_id);
    PoseEuler pos;

    ErrorCode result = motion_control_ptr_->convertJointToCart(*rq_data_ptr, user_frame_id, tool_frame_id, pos);
    if (result != SUCCESS)
    {
        pos.position.x = DBL_MAX;
        pos.position.y = DBL_MAX;
        pos.position.z = DBL_MAX;
        pos.orientation.a = DBL_MAX;
        pos.orientation.b = DBL_MAX;
        pos.orientation.c = DBL_MAX;
    }
    *rs_data_ptr = pos;
}


