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

    bool convertPoseFromBaseToTool(const PoseEuler& pose_by_base, const PoseEuler& pose_fcp_by_base, const PoseEuler& tool_frame, PoseEuler& pose_by_tool);
    bool convertPoseFromToolToBase(const PoseEuler& pose_by_tool, const PoseEuler& pose_fcp_by_base, const PoseEuler& tool_frame, PoseEuler& pose_by_base);

    bool convertTcpToFcp(const PoseEuler& pose_tcp, const PoseEuler& tool_frame, PoseEuler& pose_fcp);
    bool convertFcpToTcp(const PoseEuler& pose_fcp, const PoseEuler& tool_frame, PoseEuler& pose_tcp);

private:
    Kinematics* kinematics_ptr_;

    bool getInverse(const PoseEuler& pose, TransMatrix& inverse_trans);
};

}


#endif
