#ifndef KINEMATICS_RTM_H
#define KINEMATICS_RTM_H

#include "joint.h"
#include "pose_euler.h"
#include "pose_quaternion.h"
#include "trans_matrix.h"

namespace basic_alg
{

typedef struct
{
    double d;
    double a;
    double alpha;
    double offset;
}DH;

typedef struct
{
    int arm;    // 1: right arm, -1:left arm
    int elbow;  // 1: elbow above wrist, -1:elbow below wrist
    int wrist;  // 1: wrist down, -1: wrist up
    int flip;   // 0: not flip wrist, 1: flip wrist
}PostureRTM;

class KinematicsRTM
{
public:
    KinematicsRTM(DH& base_dh, DH arm_dh[6], bool is_flip = false);
    ~KinematicsRTM();

    void doFK(const Joint& joint, PoseEuler& pose_euler, size_t from_joint_index = 0, size_t to_joint_index = 6);
    void doFK(const Joint& joint, PoseQuaternion& pose_quaternion, size_t from_joint_index = 0, size_t to_joint_index = 6);
    void doFK(const Joint& joint, TransMatrix& trans_matrix, size_t from_joint_index = 0, size_t to_joint_index = 6);

    /*find the solution according to posture, return false when singularity or out-of-range happens or invalid posture*/
    bool doIK(const PoseEuler& pose_euler, const PostureRTM& posture, Joint& joint, double valve = 0.001);
    bool doIK(const PoseQuaternion& pose_quaternion, const PostureRTM& posture, Joint& joint, double valve = 0.001);
    bool doIK(const TransMatrix& trans_matrix, const PostureRTM& posture, Joint& joint, double valve = 0.001);

    /*find the solution accroding to posture implied by the ref point, return false when singularity or out-of-range happens*/
    bool doIK(const PoseEuler& pose_euler, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    bool doIK(const PoseQuaternion& pose_quaternion, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    bool doIK(const TransMatrix& trans_matrix, const Joint& ref_joint, Joint& joint, double valve = 0.001);

    /*apply the ref joint when singularity happens, can always find a solution in range, return false when out-of-range happens or invalid posture*/
    bool doIK(const PoseEuler& pose_euler, const PostureRTM& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    bool doIK(const PoseQuaternion& pose_quaternion, const PostureRTM& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    bool doIK(const TransMatrix& trans_matrix, const PostureRTM& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001);

    /*get posture by joint*/
    PostureRTM getPostureByJoint(const Joint& joint, double valve = 0.001);

private:
    KinematicsRTM();
    inline void scaleResultJoint(double& angle);
    inline bool isPostureValid(const PostureRTM& posture);

    DH base_dh_;
    DH arm_dh_[6];
    TransMatrix matrix_base_;
    TransMatrix matrix_base_inv_;
    int flip_;

};


}


#endif

