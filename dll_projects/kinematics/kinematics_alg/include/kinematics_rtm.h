#ifndef KINEMATICS_RTM_H
#define KINEMATICS_RTM_H


#include "kinematics.h"
//#include "yaml_help.h"
#include <string>

namespace basic_alg
{

class KinematicsRTM : public Kinematics
{
public:
    KinematicsRTM(DH& base_dh, DH arm_dh[6], bool is_flip = false);
    //KinematicsRTM(std::string file_path, bool is_flip = false);
    ~KinematicsRTM();

    virtual bool isValid();

    virtual bool getDH(DH& base_dh, DH arm_dh[6]);
    virtual bool setDH(DH& base_dh, DH arm_dh[6]);

    virtual void doFK(const Joint& joint, PoseEuler& pose_euler, size_t from_joint_index = 0, size_t to_joint_index = 6);
    virtual void doFK(const Joint& joint, PoseQuaternion& pose_quaternion, size_t from_joint_index = 0, size_t to_joint_index = 6);
    virtual void doFK(const Joint& joint, TransMatrix& trans_matrix, size_t from_joint_index = 0, size_t to_joint_index = 6);

    /*find the solution according to posture, return false when singularity or out-of-range happens or invalid posture*/
    virtual bool doIK(const PoseEuler& pose_euler, const Posture& posture, Joint& joint, double valve = 0.000001);
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, Joint& joint, double valve = 0.000001);
    virtual bool doIK(const TransMatrix& trans_matrix, const Posture& posture, Joint& joint, double valve = 0.000001);

    /*find the solution according to posture and turns, return false when singularity or out-of-range happens or invalid posture*/
    virtual bool doIK(const PoseEuler& pose_euler, const Posture& posture, const Turn& turn, Joint& joint, double valve = 0.000001);
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, const Turn& turn, Joint& joint, double valve = 0.000001);
    virtual bool doIK(const TransMatrix& trans_matrix, const Posture& posture, const Turn& turn, Joint& joint, double valve = 0.000001);

    /*find the most close solution accroding to the ref point, return false when out-of-range happens*/
    virtual bool doIK(const PoseEuler& pose_euler, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    virtual bool doIK(const PoseQuaternion& pose_quaternion, const Joint& ref_joint, Joint& joint, double valve = 0.001);
    virtual bool doIK(const TransMatrix& trans_matrix, const Joint& ref_joint, Joint& joint, double valve = 0.001);

    /*get posture by joint*/
    virtual Posture getPostureByJoint(const Joint& joint, double valve = 0.001);
    /*get turn by joint*/
    virtual Turn getTurnByJoint(const Joint& joint);
    /*get geometry joint [-PI~PI] by joint*/
    virtual Joint getGeometryJointByJoint(const Joint& joint);
    /*get joint by geometry joint and turn*/
    virtual Joint getJointByGeometryJointAndTurn(const Joint& geom_joint, const Turn& turn);
    virtual Joint getJointByGeometryJointAndRefJointAndTurn(const Joint& geom_joint, const Joint& ref_joint, const Turn& turn);
    virtual bool nearSingularPosition(const Joint& joint);
private:
    KinematicsRTM();
    inline void scaleResultJoint(double& angle);
    inline bool isPostureValid(const Posture& posture);
    inline double getMostCloseJoint(double joint1, double joint2, double ref_joint, int& posture1);
    double getSingleJointByGeometryJointAndRefJointAndTurn(const double& geom_single_joint, const double& ref_single_joint, const int& single_turn);

    DH base_dh_;
    DH arm_dh_[6];
    TransMatrix matrix_base_;
    TransMatrix matrix_base_inv_;
    int flip_;
    // base_space::YamlHelp param_;
    bool is_valid_;
    std::string file_path_;
};


}


#endif

