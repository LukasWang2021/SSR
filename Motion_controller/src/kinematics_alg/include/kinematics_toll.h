#ifndef KINEMATICS_TOLL_H
#define KINEMATICS_TOLL_H

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
    int arm;   // 1: right arm, -1:left arm
}PostureToll;


class KinematicsToll
{
public:
    KinematicsToll(DH& base_dh, DH arm_dh[4], bool is_left = false);
    ~KinematicsToll();

    void doFK(const Joint& joint, PoseEuler& pose_euler, size_t from_joint_index = 0, size_t to_joint_index = 4);
    void doFK(const Joint& joint, PoseQuaternion& pose_quaternion, size_t from_joint_index = 0, size_t to_joint_index = 4);
    void doFK(const Joint& joint, TransMatrix& trans_matrix, size_t from_joint_index = 0, size_t to_joint_index = 4);

    /*find the solution according to posture, return false when out-of-range happens or invalid posture*/
    bool doIK(const PoseEuler& pose_euler, const PostureToll& posture, Joint& joint, double valve = 0.001);
    bool doIK(const PoseQuaternion& pose_quaternion, const PostureToll& posture, Joint& joint, double valve = 0.001);
    bool doIK(const TransMatrix& trans_matrix, const PostureToll& posture, Joint& joint, double valve = 0.001);

    /*find the solution accroding to posture implied by the ref point, return false when out-of-range happens*/
    bool doIK(const PoseEuler& pose_euler, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    bool doIK(const PoseQuaternion& pose_quaternion, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    bool doIK(const TransMatrix& trans_matrix, const Joint& ref_joint, Joint& joint, double valve = 0.001);

    /*find the solution according to posture, return false when out-of-range happens or invalid posture*/
    bool doIK(const PoseEuler& pose_euler, const PostureToll& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    bool doIK(const PoseQuaternion& pose_quaternion, const PostureToll& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    bool doIK(const TransMatrix& trans_matrix, const PostureToll& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001);

    /*get posture by default*/
    PostureToll getPostureByJoint(const Joint& joint, double valve = 0.001);

private:
    KinematicsToll();
    inline void scaleResultJoint(double& angle);
    inline bool isPostureValid(const PostureToll& posture);

    DH base_dh_;
    DH arm_dh_[6];
    TransMatrix matrix_base_;
    TransMatrix matrix_base_inv_;
    int arm_; // 1: right arm, -1:left arm

};


}


#endif

