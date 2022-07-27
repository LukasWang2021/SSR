#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "joint.h"
#include "pose_euler.h"
#include "pose_quaternion.h"
#include "trans_matrix.h"
#include "basic_alg_datatype.h"

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
    int arm;    // 1: face front, -1: face back
    int elbow;  // 1: elbow above wrist, -1:elbow below wrist
    int wrist;  // 1: wrist down, -1: wrist up
    int flip;   // 0: not flip wrist, 1: flip wrist, not used.
}Posture;

typedef struct
{
    int   j1;
    int   j2;
    int   j3;
    int   j4;
    int   j5;
    int   j6;
    int   j7;
    int   j8;
    int   j9;
}Turn;

class Kinematics
{
public:
    virtual ~Kinematics(){}

    virtual bool isValid() = 0;

    virtual bool getDH(DH& base_dh, DH arm_dh[6]) = 0;
    //virtual bool setDH(DH& base_dh, DH arm_dh[6]) = 0;
    
    virtual void doFK(const Joint& joint, PoseEuler& pose_euler, size_t from_joint_index = 0, size_t to_joint_index = 6) = 0;
    virtual void doFK(const Joint& joint, PoseQuaternion& pose_quaternion, size_t from_joint_index = 0, size_t to_joint_index = 6) = 0;
    virtual void doFK(const Joint& joint, TransMatrix& trans_matrix, size_t from_joint_index = 0, size_t to_joint_index = 6) = 0;

    /*find the solution according to posture, return false when singularity or out-of-range happens or invalid posture*/
    virtual bool doIK(const PoseEuler& pose_euler, const Posture& posture, Joint& joint, double valve = 0.000001) = 0;
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, Joint& joint, double valve = 0.000001) = 0;
    virtual bool doIK(const TransMatrix& trans_matrix, const Posture& posture, Joint& joint, double valve = 0.000001) = 0;

    /*find the solution according to posture and turns, return false when singularity or out-of-range happens or invalid posture*/
    virtual bool doIK(const PoseEuler& pose_euler, const Posture& posture, const Turn& turn, Joint& joint, double valve = 0.000001) = 0;
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, const Turn& turn, Joint& joint, double valve = 0.000001) = 0;
    virtual bool doIK(const TransMatrix& trans_matrix, const Posture& posture, const Turn& turn, Joint& joint, double valve = 0.000001) = 0;

    /*find the solution accroding to posture implied by the ref point, return false when singularity or out-of-range happens*/
    virtual bool doIK(const PoseEuler& pose_euler, const Joint& ref_joint, Joint& joint, double valve = 0.001) = 0;
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Joint& ref_joint, Joint& joint, double valve = 0.001) = 0;
    virtual bool doIK(const TransMatrix& trans_matrix, const Joint& ref_joint, Joint& joint, double valve = 0.001) = 0;

    virtual Posture getPostureByJoint(const Joint& joint, double valve = 0.001) = 0;
    virtual Turn getTurnByJoint(const Joint& joint) = 0;
    virtual Joint getGeometryJointByJoint(const Joint& joint) = 0;
    virtual Joint getJointByGeometryJointAndTurn(const Joint& joint, const Turn& turn) = 0;
    virtual Joint getJointByGeometryJointAndRefJointAndTurn(const Joint& geom_joint, const Joint& ref_joint, const Turn& turn) = 0;
    virtual bool nearSingularPosition(const Joint& joint) = 0;
};

}

#endif



