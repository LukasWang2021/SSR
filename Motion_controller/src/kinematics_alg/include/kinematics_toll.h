#ifndef KINEMATICS_TOLL_H
#define KINEMATICS_TOLL_H

#include "joint.h"
#include "pose_euler.h"
#include "pose_quaternion.h"
#include "trans_matrix.h"
#include "kinematics.h"
#include "parameter_manager/parameter_manager_param_group.h"


namespace basic_alg
{

class KinematicsToll : public Kinematics
{
public:
    KinematicsToll(DH& base_dh, DH arm_dh[4], bool is_left = false);
    KinematicsToll(std::string file_path, bool is_flip = false);
    ~KinematicsToll();

    virtual bool isValid();

    virtual void doFK(const Joint& joint, PoseEuler& pose_euler, size_t from_joint_index = 0, size_t to_joint_index = 4);
    virtual void doFK(const Joint& joint, PoseQuaternion& pose_quaternion, size_t from_joint_index = 0, size_t to_joint_index = 4);
    virtual void doFK(const Joint& joint, TransMatrix& trans_matrix, size_t from_joint_index = 0, size_t to_joint_index = 4);

    /*find the solution according to posture, return false when out-of-range happens or invalid posture*/
    virtual bool doIK(const PoseEuler& pose_euler, const Posture& posture, Joint& joint, double valve = 0.001);
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, Joint& joint, double valve = 0.001);
    virtual bool doIK(const TransMatrix& trans_matrix, const Posture& posture, Joint& joint, double valve = 0.001);

    /*find the solution accroding to posture implied by the ref point, return false when out-of-range happens*/
    virtual bool doIK(const PoseEuler& pose_euler, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    virtual bool doIK(const TransMatrix& trans_matrix, const Joint& ref_joint, Joint& joint, double valve = 0.001);

    /*find the solution according to posture, return false when out-of-range happens or invalid posture*/
    virtual bool doIK(const PoseEuler& pose_euler, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    virtual bool doIK(const TransMatrix& trans_matrix, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve = 0.001);

    /*get posture by default*/
    virtual Posture getPostureByJoint(const Joint& joint, double valve = 0.001);

private:
    KinematicsToll();
    inline void scaleResultJoint(double& angle);
    inline bool isPostureValid(const Posture& posture);

    DH base_dh_;
    DH arm_dh_[4];
    TransMatrix matrix_base_;
    TransMatrix matrix_base_inv_;
    Posture posture_; // 1,1,1
    fst_parameter::ParamGroup param_;
    bool is_valid_;
};


}


#endif

