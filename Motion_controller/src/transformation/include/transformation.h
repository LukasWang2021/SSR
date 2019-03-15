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

    bool getTcpByBase(Joint& joint, PoseEuler& tool_frame, PoseEuler& pose_tcp_by_base);   
    bool getFcpByBase(Joint& joint, PoseEuler& pose_fcp_by_base);  
    bool getTcpByUser(Joint& joint, PoseEuler& user_frame, PoseEuler& tool_frame, PoseEuler& pose_tcp_by_user);
    bool getFcpByUser(Joint& joint, PoseEuler& user_frame, PoseEuler& pose_fcp_by_user);

    bool convertPoseFromBaseToUser(PoseEuler& pose_by_base, PoseEuler& user_frame, PoseEuler& pose_by_user);
    bool convertPoseFromUserToBase(PoseEuler& pose_by_user, PoseEuler& user_frame, PoseEuler& pose_by_base);

    bool convertPoseFromBaseToTool(PoseEuler& pose_by_base, PoseEuler& pose_fcp_by_base, PoseEuler& tool_frame, PoseEuler& pose_by_tool);
    bool convertPoseFromToolToBase(PoseEuler& pose_by_tool, PoseEuler& pose_fcp_by_base, PoseEuler& tool_frame, PoseEuler& pose_by_base);

    bool convertTcpToFcp(PoseEuler& pose_tcp, PoseEuler& tool_frame, PoseEuler& pose_fcp);
    bool convertFcpToTcp(PoseEuler& pose_fcp, PoseEuler& tool_frame, PoseEuler& pose_tcp);

private:
    Kinematics* kinematics_ptr_;

    bool getInverse(PoseEuler& pose, TransMatrix& inverse_trans);
};

}


#endif
