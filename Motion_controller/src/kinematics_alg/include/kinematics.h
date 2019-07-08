#ifndef KINEMATICS_H
#define KINEMATICS_H

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
}Posture;

typedef struct
{
    int   j1_;
    int   j2_;
    int   j3_;
    int   j4_;
    int   j5_;
    int   j6_;
    int   j7_;
    int   j8_;
    int   j9_;
}Turn;

class Kinematics
{
public:
    virtual ~Kinematics(){}

    virtual bool isValid() = 0;

    virtual bool getDH(DH& base_dh, DH arm_dh[6]) = 0;
    virtual bool setDH(DH& base_dh, DH arm_dh[6]) = 0;

    virtual void doFK(const Joint& joint, PoseEuler& pose_euler, size_t from_joint_index = 0, size_t to_joint_index = 6) = 0;
    virtual void doFK(const Joint& joint, PoseQuaternion& pose_quaternion, size_t from_joint_index = 0, size_t to_joint_index = 6) = 0;
    virtual void doFK(const Joint& joint, TransMatrix& trans_matrix, size_t from_joint_index = 0, size_t to_joint_index = 6) = 0;

    /*find the solution according to posture, return false when singularity or out-of-range happens or invalid posture*/
    virtual bool doIK(const PoseEuler& pose_euler, const Posture& posture, Joint& joint, double valve = 0.001) = 0;
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, Joint& joint, double valve = 0.001) = 0;
    virtual bool doIK(const TransMatrix& trans_matrix, const Posture& posture, Joint& joint, double valve = 0.001) = 0;

    /*find the solution accroding to posture implied by the ref point, return false when singularity or out-of-range happens*/
    virtual bool doIK(const PoseEuler& pose_euler, const Joint& ref_joint, Joint& joint, double valve = 0.001) = 0;
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Joint& ref_joint, Joint& joint, double valve = 0.001) = 0;
    virtual bool doIK(const TransMatrix& trans_matrix, const Joint& ref_joint, Joint& joint, double valve = 0.001) = 0;

    /*apply the ref joint when singularity happens, can always find a solution in range, return false when out-of-range happens or invalid posture*/
    virtual bool doIK(const PoseEuler& pose_euler, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001) = 0;
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001) = 0;
    virtual bool doIK(const TransMatrix& trans_matrix, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001) = 0;

    virtual Posture getPostureByJoint(const Joint& joint, double valve = 0.001) = 0;
};

}

#endif



