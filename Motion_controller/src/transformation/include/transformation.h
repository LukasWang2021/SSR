#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include "basic_alg_datatype.h"
#include "kinematics.h"

namespace basic_alg
{
class Transformation
{
public:
    Transformation();
    ~Transformation();

    bool init(Kinematics* kinematics_ptr);

    bool getTcpByBase(const Joint& joint, const PoseEuler& tool_frame, PoseEuler& pose_tcp_by_base);   
    bool getFcpByBase(const Joint& joint, PoseEuler& pose_fcp_by_base);  
    bool getTcpByUser(const Joint& joint, const PoseEuler& user_frame, const PoseEuler& tool_frame, PoseEuler& pose_tcp_by_user);
    bool getFcpByUser(const Joint& joint, const PoseEuler& user_frame, PoseEuler& pose_fcp_by_user);

    bool convertPoseFromBaseToUser(const PoseEuler& pose_by_base, const PoseEuler& user_frame, PoseEuler& pose_by_user);
    bool convertPoseFromUserToBase(const PoseEuler& pose_by_user, const PoseEuler& user_frame, PoseEuler& pose_by_base);
    bool convertPoseFromBaseToUser(const PoseQuaternion& pose_by_base, const PoseEuler& user_frame, PoseQuaternion& pose_by_user);
    bool convertPoseFromUserToBase(const PoseQuaternion& pose_by_user, const PoseEuler& user_frame, PoseQuaternion& pose_by_base);


    bool convertPoseFromBaseToTool(const PoseEuler& pose_by_base, const PoseEuler& pose_fcp_by_base, const PoseEuler& tool_frame, PoseEuler& pose_by_tool);
    bool convertPoseFromToolToBase(const PoseEuler& pose_by_tool, const PoseEuler& pose_fcp_by_base, const PoseEuler& tool_frame, PoseEuler& pose_by_base);

    bool convertTcpToFcp(const PoseEuler& pose_tcp, const PoseEuler& tool_frame, PoseEuler& pose_fcp);
    bool convertFcpToTcp(const PoseEuler& pose_fcp, const PoseEuler& tool_frame, PoseEuler& pose_tcp);
    bool convertTcpToFcp(const PoseQuaternion& pose_tcp, const PoseEuler& tool_frame, PoseQuaternion& pose_fcp);
    bool convertFcpToTcp(const PoseQuaternion& pose_fcp, const PoseEuler& tool_frame, PoseQuaternion& pose_tcp);    

    bool addToolOffsetForTcp(const PoseEuler& pose_tcp, const PoseEuler& tool_offset, PoseEuler& new_pose_tcp);
    bool deleteToolOffsetForTcp(const PoseEuler& pose_tcp, const PoseEuler& tool_offset, PoseEuler& new_pose_tcp);
    bool addToolOffsetForTcp(const PoseQuaternion& pose_tcp, const PoseEuler& tool_offset, PoseQuaternion& new_pose_tcp);
    bool deleteToolOffsetForTcp(const PoseQuaternion& pose_tcp, const PoseEuler& tool_offset, PoseQuaternion& new_pose_tcp);

    bool addOffset(const PoseEuler& pose, const PoseEuler& offset, PoseEuler& new_pose);
    bool deleteOffset(const PoseEuler& pose, const PoseEuler& offset, PoseEuler& new_pose);
    bool addOffset(const PoseQuaternion& pose, const PoseEuler& offset, PoseQuaternion& new_pose);
    bool deleteOffset(const PoseQuaternion& pose, const PoseEuler& offset, PoseQuaternion& new_pose);

private:
    Kinematics* kinematics_ptr_;

    bool getInverse(const PoseEuler& pose, TransMatrix& inverse_trans);
};

}


#endif
