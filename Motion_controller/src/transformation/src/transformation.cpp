#include "transformation.h"

using namespace std;
using namespace basic_alg;

Transformation::Transformation():
    kinematics_ptr_(NULL)
{

}

Transformation::~Transformation()
{

}

bool Transformation::init(Kinematics* kinematics_ptr)
{
    if(kinematics_ptr == NULL
        || !kinematics_ptr->isValid())
    {
        return false;
    }
    else
    {
        kinematics_ptr_ = kinematics_ptr;
        return true;
    }
}

bool Transformation::getTcpByBase(const Joint& joint, const PoseEuler& tool_frame, PoseEuler& pose_tcp_by_base)
{
    TransMatrix trans_fcp_by_base;
    kinematics_ptr_->doFK(joint, trans_fcp_by_base);
    TransMatrix trans_tool_frame, trans_tcp_by_base;
    tool_frame.convertToTransMatrix(trans_tool_frame);    
    trans_fcp_by_base.rightMultiply(trans_tool_frame, trans_tcp_by_base);
    trans_tcp_by_base.convertToPoseEuler(pose_tcp_by_base);
    return true;
}

bool Transformation::getFcpByBase(const Joint& joint, PoseEuler& pose_fcp_by_base)
{
    kinematics_ptr_->doFK(joint, pose_fcp_by_base);
    return true;
}

bool Transformation::getTcpByUser(const Joint& joint, const PoseEuler& user_frame, const PoseEuler& tool_frame, PoseEuler& pose_tcp_by_user)
{
    TransMatrix trans_user_frame_inverse;
    if(!getInverse(user_frame, trans_user_frame_inverse))
    {
        return false;
    }    
    PoseEuler pose_tcp_by_base;
    getTcpByBase(joint, tool_frame, pose_tcp_by_base);
    TransMatrix trans_tcp_by_base, trans_tcp_by_user;
    pose_tcp_by_base.convertToTransMatrix(trans_tcp_by_base);
    trans_user_frame_inverse.rightMultiply(trans_tcp_by_base, trans_tcp_by_user);
    trans_tcp_by_user.convertToPoseEuler(pose_tcp_by_user);
    return true;
}

bool Transformation::getFcpByUser(const Joint& joint, const PoseEuler& user_frame, PoseEuler& pose_fcp_by_user)
{
    TransMatrix trans_user_frame_inverse;
    if(!getInverse(user_frame, trans_user_frame_inverse))
    {
        return false;
    }  
    TransMatrix trans_fcp_by_base;
    kinematics_ptr_->doFK(joint, trans_fcp_by_base);
    TransMatrix trans_fcp_by_user;
    trans_user_frame_inverse.rightMultiply(trans_fcp_by_base, trans_fcp_by_user);
    trans_fcp_by_user.convertToPoseEuler(pose_fcp_by_user);
    return true;
}

bool Transformation::convertPoseFromBaseToUser(const PoseEuler& pose_by_base, const PoseEuler& user_frame, PoseEuler& pose_by_user)
{
    TransMatrix trans_user_frame_inverse;
    if(!getInverse(user_frame, trans_user_frame_inverse))
    {
        return false;
    }
    TransMatrix trans_pose_by_base, trans_pose_by_user;
    pose_by_base.convertToTransMatrix(trans_pose_by_base);
    trans_user_frame_inverse.rightMultiply(trans_pose_by_base, trans_pose_by_user);
    trans_pose_by_user.convertToPoseEuler(pose_by_user);
    return true;
}

bool Transformation::convertPoseFromUserToBase(const PoseEuler& pose_by_user, const PoseEuler& user_frame, PoseEuler& pose_by_base)
{
    TransMatrix trans_pose_by_user, trans_user_frame;
    //printf("xzc_debug_step3\n");
    pose_by_user.convertToTransMatrix(trans_pose_by_user);
    user_frame.convertToTransMatrix(trans_user_frame);
    TransMatrix trans_pose_by_base;
    trans_user_frame.rightMultiply(trans_pose_by_user, trans_pose_by_base);
    trans_pose_by_base.convertToPoseEuler(pose_by_base);
    return true;
}

bool Transformation::convertPoseFromBaseToUser(const PoseQuaternion& pose_by_base, const PoseEuler& user_frame, PoseQuaternion& pose_by_user)
{
    TransMatrix trans_user_frame_inverse;
    if(!getInverse(user_frame, trans_user_frame_inverse))
    {
        return false;
    }
    TransMatrix trans_pose_by_base, trans_pose_by_user;
    pose_by_base.convertToTransMatrix(trans_pose_by_base);
    trans_user_frame_inverse.rightMultiply(trans_pose_by_base, trans_pose_by_user);
    trans_pose_by_user.convertToPoseQuaternion(pose_by_user);
    return true;
}

bool Transformation::convertPoseFromUserToBase(const PoseQuaternion& pose_by_user, const PoseEuler& user_frame, PoseQuaternion& pose_by_base)
{
    TransMatrix trans_pose_by_user, trans_user_frame;
    pose_by_user.convertToTransMatrix(trans_pose_by_user);
    user_frame.convertToTransMatrix(trans_user_frame);
    TransMatrix trans_pose_by_base;
    trans_user_frame.rightMultiply(trans_pose_by_user, trans_pose_by_base);
    trans_pose_by_base.convertToPoseQuaternion(pose_by_base);
    return true;
}



bool Transformation::convertPoseFromBaseToTool(const PoseEuler& pose_by_base, const PoseEuler& pose_tcp_by_base, PoseEuler& pose_by_tool)
{
    TransMatrix trans_pose_tcp_by_base_inverse;
    if(!getInverse(pose_tcp_by_base, trans_pose_tcp_by_base_inverse))
    {
        return false;
    }
    TransMatrix trans_pose_by_base, trans_pose_by_tool;
    pose_by_base.convertToTransMatrix(trans_pose_by_base);
    trans_pose_tcp_by_base_inverse.rightMultiply(trans_pose_by_base, trans_pose_by_tool);
    trans_pose_by_tool.convertToPoseEuler(pose_by_tool);
    return true;
}

bool Transformation::convertPoseFromToolToBase(const PoseEuler& pose_by_tool, const PoseEuler& pose_tcp_by_base, PoseEuler& pose_by_base)
{
    TransMatrix trans_pose_tcp_by_base;
    pose_tcp_by_base.convertToTransMatrix(trans_pose_tcp_by_base);
    TransMatrix trans_pose_by_tool;
    pose_by_tool.convertToTransMatrix(trans_pose_by_tool);
    TransMatrix trans_pose_by_base;
    trans_pose_tcp_by_base.rightMultiply(trans_pose_by_tool, trans_pose_by_base);
    trans_pose_by_base.convertToPoseEuler(pose_by_base);
    return true;
}

bool Transformation::convertTcpToFcp(const PoseEuler& pose_tcp, const PoseEuler& tool_frame, PoseEuler& pose_fcp)
{
    TransMatrix trans_tool_frame_inverse;
    if(!getInverse(tool_frame, trans_tool_frame_inverse))
    {
        return false;
    }
    TransMatrix trans_pose_tcp;
    pose_tcp.convertToTransMatrix(trans_pose_tcp);
    TransMatrix trans_pose_fcp;
    trans_pose_tcp.rightMultiply(trans_tool_frame_inverse, trans_pose_fcp);
    trans_pose_fcp.convertToPoseEuler(pose_fcp);
    return true;
}

bool Transformation::convertFcpToTcp(const PoseEuler& pose_fcp, const PoseEuler& tool_frame, PoseEuler& pose_tcp)
{
    TransMatrix trans_pose_fcp;
    pose_fcp.convertToTransMatrix(trans_pose_fcp);
    TransMatrix trans_tool_frame;
    tool_frame.convertToTransMatrix(trans_tool_frame);
    TransMatrix trans_pose_tcp;
    trans_pose_fcp.rightMultiply(trans_tool_frame, trans_pose_tcp);
    trans_pose_tcp.convertToPoseEuler(pose_tcp);
    return true;
}

bool Transformation::convertTcpToFcp(const PoseQuaternion& pose_tcp, const PoseEuler& tool_frame, PoseQuaternion& pose_fcp)
{
    TransMatrix trans_tool_frame_inverse;
    if(!getInverse(tool_frame, trans_tool_frame_inverse))
    {
        return false;
    }
    TransMatrix trans_pose_tcp;
    pose_tcp.convertToTransMatrix(trans_pose_tcp);
    TransMatrix trans_pose_fcp;
    trans_pose_tcp.rightMultiply(trans_tool_frame_inverse, trans_pose_fcp);
    trans_pose_fcp.convertToPoseQuaternion(pose_fcp);
    return true;

}

bool Transformation::convertFcpToTcp(const PoseQuaternion& pose_fcp, const PoseEuler& tool_frame, PoseQuaternion& pose_tcp)
{
    TransMatrix trans_pose_fcp;
    pose_fcp.convertToTransMatrix(trans_pose_fcp);
    TransMatrix trans_tool_frame;
    tool_frame.convertToTransMatrix(trans_tool_frame);
    TransMatrix trans_pose_tcp;
    trans_pose_fcp.rightMultiply(trans_tool_frame, trans_pose_tcp);
    trans_pose_tcp.convertToPoseQuaternion(pose_tcp);
    return true;
}

bool Transformation::addToolOffsetForTcp(const PoseEuler& pose_tcp, const PoseEuler& tool_offset, PoseEuler& new_pose_tcp)
{
    TransMatrix trans_tool_offset_inverse;
    if(!getInverse(tool_offset, trans_tool_offset_inverse))
    {
        return false;
    }
    TransMatrix trans_pose_tcp;
    pose_tcp.convertToTransMatrix(trans_pose_tcp);
    TransMatrix trans_new_pose_tcp;
    trans_tool_offset_inverse.rightMultiply(trans_pose_tcp, trans_new_pose_tcp);
    trans_new_pose_tcp.convertToPoseEuler(new_pose_tcp);
    return true;
}

bool Transformation::deleteToolOffsetForTcp(const PoseEuler& pose_tcp, const PoseEuler& tool_offset, PoseEuler& new_pose_tcp)
{
    TransMatrix trans_tool_offset;
    tool_offset.convertToTransMatrix(trans_tool_offset);
    TransMatrix trans_pose_tcp;
    pose_tcp.convertToTransMatrix(trans_pose_tcp);
    TransMatrix trans_new_pose_tcp;
    trans_tool_offset.rightMultiply(trans_pose_tcp, trans_new_pose_tcp);
    trans_new_pose_tcp.convertToPoseEuler(new_pose_tcp);
    return true;
}

bool Transformation::addToolOffsetForTcp(const PoseQuaternion& pose_tcp, const PoseEuler& tool_offset, PoseQuaternion& new_pose_tcp)
{
    TransMatrix trans_tool_offset_inverse;
    if(!getInverse(tool_offset, trans_tool_offset_inverse))
    {
        return false;
    }
    TransMatrix trans_pose_tcp;
    pose_tcp.convertToTransMatrix(trans_pose_tcp);
    TransMatrix trans_new_pose_tcp;
    trans_tool_offset_inverse.rightMultiply(trans_pose_tcp, trans_new_pose_tcp);
    trans_new_pose_tcp.convertToPoseQuaternion(new_pose_tcp);
    return true;
}

bool Transformation::deleteToolOffsetForTcp(const PoseQuaternion& pose_tcp, const PoseEuler& tool_offset, PoseQuaternion& new_pose_tcp)
{
    TransMatrix trans_tool_offset;
    tool_offset.convertToTransMatrix(trans_tool_offset);
    TransMatrix trans_pose_tcp;
    pose_tcp.convertToTransMatrix(trans_pose_tcp);
    TransMatrix trans_new_pose_tcp;
    trans_tool_offset.rightMultiply(trans_pose_tcp, trans_new_pose_tcp);
    trans_new_pose_tcp.convertToPoseQuaternion(new_pose_tcp);
    return true;
}

bool Transformation::addOffset(const PoseEuler& pose, const PoseEuler& offset, PoseEuler& new_pose)
{
    return addToolOffsetForTcp(pose, offset, new_pose);
}

bool Transformation::deleteOffset(const PoseEuler& pose, const PoseEuler& offset, PoseEuler& new_pose)
{
    return deleteToolOffsetForTcp(pose, offset, new_pose);
}

bool Transformation::addOffset(const PoseQuaternion& pose, const PoseEuler& offset, PoseQuaternion& new_pose)
{
    return addToolOffsetForTcp(pose, offset, new_pose);
}

bool Transformation::deleteOffset(const PoseQuaternion& pose, const PoseEuler& offset, PoseQuaternion& new_pose)
{
    return deleteToolOffsetForTcp(pose, offset, new_pose);
}

bool Transformation::getInverse(const PoseEuler& pose, TransMatrix& inverse_trans)
{
    TransMatrix trans;
    pose.convertToTransMatrix(trans);
    if(trans.inverse(inverse_trans))
    {
        return true;
    }  
    else
    {
        return false;
    }
}


