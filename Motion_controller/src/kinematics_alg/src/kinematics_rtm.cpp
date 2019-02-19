#include "kinematics_rtm.h"
#include <iostream>
#include <math.h>


using namespace std;
using namespace basic_alg;


KinematicsRTM::KinematicsRTM()
{

}

KinematicsRTM::KinematicsRTM(DH& base_dh, DH arm_dh[6], bool is_flip):    
    matrix_base_(base_dh.d, base_dh.a, base_dh.alpha, base_dh.offset)
{
    base_dh_.d = base_dh.d;
    base_dh_.a = base_dh.a;
    base_dh_.alpha = base_dh.alpha;
    base_dh_.offset = base_dh.offset;   
    
    for(int i = 0; i < 6; ++i)
    {
        arm_dh_[i].d = arm_dh[i].d;
        arm_dh_[i].a = arm_dh[i].a;
        arm_dh_[i].alpha = arm_dh[i].alpha;
        arm_dh_[i].offset = arm_dh[i].offset;
    }

    matrix_base_.inverse(matrix_base_inv_);
    if(is_flip)
    {
        flip_ = 1;
    }
    else
    {
        flip_ = 0;
    }
}

KinematicsRTM::~KinematicsRTM()
{

}

void KinematicsRTM::doFK(const Joint& joint, PoseEuler& pose_euler, size_t from_joint_index, size_t to_joint_index)
{
    TransMatrix result_matrix;
    if(from_joint_index == 0)
    {
        result_matrix = matrix_base_;
    }
    else
    {
        size_t first_dh_index = from_joint_index - 1;
        TransMatrix matrix_first(arm_dh_[first_dh_index].d, arm_dh_[first_dh_index].a, arm_dh_[first_dh_index].alpha, joint[first_dh_index] + arm_dh_[first_dh_index].offset);
        result_matrix = matrix_first;       
    }
    for(size_t i = from_joint_index; i < to_joint_index; ++i)
    {
        TransMatrix matrix(arm_dh_[i].d, arm_dh_[i].a, arm_dh_[i].alpha, joint[i] + arm_dh_[i].offset);
        result_matrix.rightMultiply(matrix);
    }
    result_matrix.convertToPoseEuler(pose_euler);
}

void KinematicsRTM::doFK(const Joint& joint, PoseQuaternion& pose_quaternion, size_t from_joint_index, size_t to_joint_index)
{
    TransMatrix result_matrix;
    if(from_joint_index == 0)
    {
        result_matrix = matrix_base_;
    }
    else
    {
        size_t first_dh_index = from_joint_index - 1;
        TransMatrix matrix_first(arm_dh_[first_dh_index].d, arm_dh_[first_dh_index].a, arm_dh_[first_dh_index].alpha, joint[first_dh_index] + arm_dh_[first_dh_index].offset);
        result_matrix = matrix_first;       
    }
    for(size_t i = from_joint_index; i < to_joint_index; ++i)
    {
        TransMatrix matrix(arm_dh_[i].d, arm_dh_[i].a, arm_dh_[i].alpha, joint[i] + arm_dh_[i].offset);
        result_matrix.rightMultiply(matrix);
    }
    result_matrix.convertToPoseQuaternion(pose_quaternion);
}

void KinematicsRTM::doFK(const Joint& joint, TransMatrix& trans_matrix, size_t from_joint_index, size_t to_joint_index)
{
    TransMatrix result_matrix;
    if(from_joint_index == 0)
    {
        result_matrix = matrix_base_;
    }
    else
    {
        size_t first_dh_index = from_joint_index - 1;
        TransMatrix matrix_first(arm_dh_[first_dh_index].d, arm_dh_[first_dh_index].a, arm_dh_[first_dh_index].alpha, joint[first_dh_index] + arm_dh_[first_dh_index].offset);        
        result_matrix = matrix_first;       
    }

    for(size_t i = from_joint_index; i < to_joint_index; ++i)
    {
        TransMatrix matrix(arm_dh_[i].d, arm_dh_[i].a, arm_dh_[i].alpha, joint[i] + arm_dh_[i].offset);
        result_matrix.rightMultiply(matrix);
    }
    trans_matrix = result_matrix;
}

bool KinematicsRTM::doIK(const PoseEuler& pose_euler, const PostureRTM& posture, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_euler.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, joint, valve);
}

bool KinematicsRTM::doIK(const PoseQuaternion& pose_quaternion, const PostureRTM& posture, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_quaternion.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, joint, valve);
}

bool KinematicsRTM::doIK(const TransMatrix& trans_matrix, const PostureRTM& posture, Joint& joint, double valve)
{
    if(!isPostureValid(posture))
    {
        return false;
    }

    TransMatrix trans_matrix_1to6;
    TransMatrix* trans_matrix_ptr = const_cast<TransMatrix*>(&trans_matrix);
    trans_matrix_ptr->leftMultiply(matrix_base_inv_, trans_matrix_1to6);
    Point vector_n, vector_s, vector_a;
    trans_matrix_1to6.rotation_matrix_.getVectorN(vector_n);
    trans_matrix_1to6.rotation_matrix_.getVectorS(vector_s);
    trans_matrix_1to6.rotation_matrix_.getVectorA(vector_a);   
    Point p_vector = trans_matrix_1to6.trans_vector_ - vector_a * arm_dh_[5].d;
    
    // compute q1
    if(fabs(p_vector.x_) > valve 
        || fabs(p_vector.y_) > valve)
    {
        joint.j1_ = atan2(posture.arm * p_vector.y_, posture.arm * p_vector.x_) - arm_dh_[0].offset;
    }
    else
    {
        return false;
    }
    scaleResultJoint(joint.j1_);

    // compute q2
    double R = p_vector.norm();
    double r = sqrt(p_vector.x_ * p_vector.x_ + p_vector.y_ * p_vector.y_);    
    double sin_alpha = fabs(p_vector.z_) / R;
    double cos_alpha = r / R; 
    double cos_beta = (arm_dh_[1].a * arm_dh_[1].a + R * R - arm_dh_[3].d * arm_dh_[3].d - arm_dh_[2].a * arm_dh_[2].a) / (2.0 * arm_dh_[1].a * R);
    if(cos_beta < -1 || cos_beta > 1)
    {
        return false;
    }
    double sin_beta = sqrt(1 - cos_beta * cos_beta);
    double sin_j, cos_j;
    if(posture.arm == 1)
    {
        sin_j = sin_beta * cos_alpha + posture.elbow * cos_beta * sin_alpha;
        cos_j = cos_beta * cos_alpha - posture.elbow * sin_beta * sin_alpha;
    }
    else
    {
        sin_j = posture.elbow * (sin_beta * cos_alpha - cos_beta * sin_alpha);
        cos_j = posture.elbow * (cos_beta * cos_alpha + sin_beta * sin_alpha);
    }  
    joint.j2_ = atan2(sin_j, cos_j) - arm_dh_[1].offset;
    scaleResultJoint(joint.j2_);

    // compute q3
    double d4_a3 = sqrt(arm_dh_[3].d * arm_dh_[3].d + arm_dh_[2].a * arm_dh_[2].a);
    cos_alpha = (arm_dh_[1].a * arm_dh_[1].a + d4_a3 * d4_a3 - R * R) / (2.0 * arm_dh_[1].a * d4_a3);
    if(cos_alpha < -1 || cos_alpha > 1)
    {
        return false;
    }
    sin_alpha = sqrt(1 - cos_alpha * cos_alpha);
    sin_beta = arm_dh_[3].d / d4_a3;
    cos_beta = fabs(arm_dh_[2].a) / d4_a3;
    if(posture.arm == 1)
    {
        sin_j = sin_alpha * cos_beta - cos_alpha * sin_beta;
        cos_j = cos_alpha * cos_beta + sin_alpha * sin_beta; 
    }
    else
    {
        sin_j = -posture.elbow * sin_alpha * cos_beta - cos_alpha * sin_beta;
        cos_j = -posture.elbow * cos_alpha * cos_beta + sin_alpha * sin_beta;
    }
    joint.j3_ = atan2(sin_j, cos_j) - arm_dh_[2].offset;
    scaleResultJoint(joint.j3_);

    // compute q4
    TransMatrix trans_matrix_1to3;
    doFK(joint, trans_matrix_1to3, 1, 3);    // trans_matrix_tmp is from joint 1 to joint 3
    Point z3, z4, x_vector, y_vector;
    trans_matrix_1to3.rotation_matrix_.getVectorA(z3);  
    z3.crossProduct(vector_a, z4);
    if(!z4.normalize()
        || z4.isParallel(vector_a))
    {
        return false;
    }
    double omega = vector_s.dotProduct(z4);
    if(fabs(omega) < valve)
    {
        omega = vector_n.dotProduct(z4);
    } 
    trans_matrix_1to3.rotation_matrix_.getVectorN(x_vector);
    trans_matrix_1to3.rotation_matrix_.getVectorS(y_vector);
    if(omega >= 0)
    {
        sin_j = -posture.wrist * z4.dotProduct(x_vector);
        cos_j = posture.wrist * z4.dotProduct(y_vector);
    }
    else
    {
        sin_j = posture.wrist * z4.dotProduct(x_vector);
        cos_j = -posture.wrist * z4.dotProduct(y_vector);
    }
    if(posture.flip == 0)
    {
        joint.j4_ = atan2(sin_j, cos_j) - arm_dh_[3].offset;
    }
    else
    {
        joint.j4_ = atan2(sin_j, cos_j) - arm_dh_[3].offset + M_PI;
    }
    scaleResultJoint(joint.j4_);

    // compute q5
    TransMatrix trans_matrix_1to4;
    doFK(joint, trans_matrix_1to4, 1, 4);    
    trans_matrix_1to4.rotation_matrix_.getVectorN(x_vector);
    trans_matrix_1to4.rotation_matrix_.getVectorS(y_vector);
    sin_j = vector_a.dotProduct(x_vector);
    cos_j = -vector_a.dotProduct(y_vector);     
    joint.j5_ = atan2(sin_j, cos_j) - arm_dh_[4].offset;
    scaleResultJoint(joint.j5_);

    // compute q6
    TransMatrix trans_matrix_1to5;
    doFK(joint, trans_matrix_1to5, 1, 5);
    trans_matrix_1to5.rotation_matrix_.getVectorS(y_vector);    
    sin_j = vector_n.dotProduct(y_vector);
    cos_j = vector_s.dotProduct(y_vector);
    joint.j6_ = atan2(sin_j, cos_j) - arm_dh_[5].offset;
    scaleResultJoint(joint.j6_);

    return true;
}

bool KinematicsRTM::doIK(const PoseEuler& pose_euler, const Joint& ref_joint, Joint& joint, double valve)
{
    PostureRTM posture = getPostureByJoint(ref_joint);
    return doIK(pose_euler, posture, joint, valve);
}

bool KinematicsRTM::doIK(const PoseQuaternion& pose_quaternion, const Joint& ref_joint, Joint& joint, double valve)
{
    PostureRTM posture = getPostureByJoint(ref_joint);
    return doIK(pose_quaternion, posture, joint, valve);
}

bool KinematicsRTM::doIK(const TransMatrix& trans_matrix, const Joint& ref_joint, Joint& joint, double valve)
{
    PostureRTM posture = getPostureByJoint(ref_joint);
    return doIK(trans_matrix, posture, joint, valve);
}

bool KinematicsRTM::doIK(const PoseEuler& pose_euler, const PostureRTM& posture, const Joint& ref_joint, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_euler.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, ref_joint, joint, valve);
}

bool KinematicsRTM::doIK(const PoseQuaternion& pose_quaternion, const PostureRTM& posture, const Joint& ref_joint, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_quaternion.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, ref_joint, joint, valve);
}

bool KinematicsRTM::doIK(const TransMatrix& trans_matrix, const PostureRTM& posture, const Joint& ref_joint, Joint& joint, double valve)
{
    if(!isPostureValid(posture))
    {
        return false;
    }

    TransMatrix trans_matrix_1to6;
    TransMatrix* trans_matrix_ptr = const_cast<TransMatrix*>(&trans_matrix);
    trans_matrix_ptr->leftMultiply(matrix_base_inv_, trans_matrix_1to6);
    Point vector_n, vector_s, vector_a;
    trans_matrix_1to6.rotation_matrix_.getVectorN(vector_n);
    trans_matrix_1to6.rotation_matrix_.getVectorS(vector_s);
    trans_matrix_1to6.rotation_matrix_.getVectorA(vector_a);   
    Point p_vector = trans_matrix_1to6.trans_vector_ - vector_a * arm_dh_[5].d;
    
    // compute q1
    if(fabs(p_vector.x_) > valve 
        || fabs(p_vector.y_) > valve)
    {
        joint.j1_ = atan2(posture.arm * p_vector.y_, posture.arm * p_vector.x_) - arm_dh_[0].offset;
        scaleResultJoint(joint.j1_);
    }
    else
    {
        joint.j1_ = ref_joint.j1_;
    }    

    // compute q2
    double R = p_vector.norm();
    double r = sqrt(p_vector.x_ * p_vector.x_ + p_vector.y_ * p_vector.y_);    
    double sin_alpha = fabs(p_vector.z_) / R;
    double cos_alpha = r / R; 
    double cos_beta = (arm_dh_[1].a * arm_dh_[1].a + R * R - arm_dh_[3].d * arm_dh_[3].d - arm_dh_[2].a * arm_dh_[2].a) / (2.0 * arm_dh_[1].a * R);
    if(cos_beta < -1 || cos_beta > 1)
    {
        return false;
    }
    double sin_beta = sqrt(1 - cos_beta * cos_beta);
    double sin_j, cos_j;
    if(posture.arm == 1)
    {
        sin_j = sin_beta * cos_alpha + posture.elbow * cos_beta * sin_alpha;
        cos_j = cos_beta * cos_alpha - posture.elbow * sin_beta * sin_alpha;
    }
    else
    {
        sin_j = posture.elbow * (sin_beta * cos_alpha - cos_beta * sin_alpha);
        cos_j = posture.elbow * (cos_beta * cos_alpha + sin_beta * sin_alpha);
    }  
    joint.j2_ = atan2(sin_j, cos_j) - arm_dh_[1].offset;
    scaleResultJoint(joint.j2_);

    // compute q3
    double d4_a3 = sqrt(arm_dh_[3].d * arm_dh_[3].d + arm_dh_[2].a * arm_dh_[2].a);
    cos_alpha = (arm_dh_[1].a * arm_dh_[1].a + d4_a3 * d4_a3 - R * R) / (2.0 * arm_dh_[1].a * d4_a3);
    if(cos_alpha < -1 || cos_alpha > 1)
    {
        return false;
    }
    sin_alpha = sqrt(1 - cos_alpha * cos_alpha);
    sin_beta = arm_dh_[3].d / d4_a3;
    cos_beta = fabs(arm_dh_[2].a) / d4_a3;
    if(posture.arm == 1)
    {
        sin_j = sin_alpha * cos_beta - cos_alpha * sin_beta;
        cos_j = cos_alpha * cos_beta + sin_alpha * sin_beta; 
    }
    else
    {
        sin_j = -posture.elbow * sin_alpha * cos_beta - cos_alpha * sin_beta;
        cos_j = -posture.elbow * cos_alpha * cos_beta + sin_alpha * sin_beta;
    }
    joint.j3_ = atan2(sin_j, cos_j) - arm_dh_[2].offset;
    scaleResultJoint(joint.j3_);

    // compute q4
    TransMatrix trans_matrix_1to3;
    doFK(joint, trans_matrix_1to3, 1, 3);    // trans_matrix_tmp is from joint 1 to joint 3
    Point z3, z4, x_vector, y_vector;
    double omega;
    trans_matrix_1to3.rotation_matrix_.getVectorA(z3);
    z3.crossProduct(vector_a, z4);
    if(!z4.normalize()
        || z4.isParallel(vector_a))
    {
        joint.j4_ = ref_joint.j4_;
        goto IK_Q5;
    }
    omega = vector_s.dotProduct(z4);
    if(fabs(omega) < valve)
    {
        omega = vector_n.dotProduct(z4);
    } 
    trans_matrix_1to3.rotation_matrix_.getVectorN(x_vector);
    trans_matrix_1to3.rotation_matrix_.getVectorS(y_vector);
    if(omega >= 0)
    {
        sin_j = -posture.wrist * z4.dotProduct(x_vector);
        cos_j = posture.wrist * z4.dotProduct(y_vector);
    }
    else
    {
        sin_j = posture.wrist * z4.dotProduct(x_vector);
        cos_j = -posture.wrist * z4.dotProduct(y_vector);
    }
    if(posture.flip == 0)
    {
        joint.j4_ = atan2(sin_j, cos_j) - arm_dh_[3].offset;
    }
    else
    {
        joint.j4_ = atan2(sin_j, cos_j) - arm_dh_[3].offset + M_PI;
    }
    scaleResultJoint(joint.j4_);

IK_Q5:
    // compute q5
    TransMatrix trans_matrix_1to4;
    doFK(joint, trans_matrix_1to4, 1, 4);    
    trans_matrix_1to4.rotation_matrix_.getVectorN(x_vector);
    trans_matrix_1to4.rotation_matrix_.getVectorS(y_vector);
    sin_j = vector_a.dotProduct(x_vector);
    cos_j = -vector_a.dotProduct(y_vector);     
    joint.j5_ = atan2(sin_j, cos_j) - arm_dh_[4].offset;
    scaleResultJoint(joint.j5_);

    // compute q6
    TransMatrix trans_matrix_1to5;
    doFK(joint, trans_matrix_1to5, 1, 5);
    trans_matrix_1to5.rotation_matrix_.getVectorS(y_vector);    
    sin_j = vector_n.dotProduct(y_vector);
    cos_j = vector_s.dotProduct(y_vector);
    joint.j6_ = atan2(sin_j, cos_j) - arm_dh_[5].offset;
    scaleResultJoint(joint.j6_);

    return true;
}

PostureRTM KinematicsRTM::getPostureByJoint(const Joint& joint, double valve)
{
    PostureRTM posture;

    // arm
    TransMatrix trans_matrix_1to6;
    doFK(joint, trans_matrix_1to6, 1, 6);
    Point vector_pxy;
    vector_pxy = trans_matrix_1to6.trans_vector_;
    vector_pxy.z_ = 0;
    TransMatrix trans_matrix_1to2(arm_dh_[0].d, arm_dh_[0].a, arm_dh_[0].alpha, joint.j1_ + arm_dh_[0].offset);
    Point z1;
    trans_matrix_1to2.rotation_matrix_.getVectorA(z1);
    Point cross_product_z1_pxy;
    z1.crossProduct(vector_pxy, cross_product_z1_pxy);
    if(cross_product_z1_pxy.z_ >= 0)
    {
        posture.arm = 1;
    }
    else
    {
        posture.arm = -1;
    }
    
    // elbow
    TransMatrix trans_matrix_2to4;
    doFK(joint, trans_matrix_2to4, 2, 4);
    if(trans_matrix_2to4.trans_vector_.y_ >= 0)
    {
        posture.elbow = posture.arm;
    }
    else
    {
        posture.elbow = -posture.arm;
    }

    // wrist
    TransMatrix trans_matrix_1to4;
    doFK(joint, trans_matrix_1to4, 1, 4);
    Point z4;
    trans_matrix_1to4.rotation_matrix_.getVectorA(z4);
    Point vector_s;
    trans_matrix_1to6.rotation_matrix_.getVectorS(vector_s);
    double tmp = vector_s.dotProduct(z4);
    if(tmp < valve)
    {
        Point vector_n;
        trans_matrix_1to6.rotation_matrix_.getVectorN(vector_n);
        tmp = vector_n.dotProduct(z4);
    }
    if(tmp >= 0)
    {
        posture.wrist = 1;
    }
    else
    {
        posture.wrist = -1;
    }    

    posture.flip = flip_;
    
    return posture;
}

inline void KinematicsRTM::scaleResultJoint(double& angle)
{
    if(angle > M_PI)
    {
        angle = angle - (2 * M_PI); 
    }
    else if(angle < -M_PI)
    {
        angle = angle + (2 * M_PI);
    }
}

inline bool KinematicsRTM::isPostureValid(const PostureRTM& posture)
{
    if((posture.arm != 1 && posture.arm != -1)
        || (posture.elbow != 1 && posture.elbow != -1)
        || (posture.wrist != 1 && posture.wrist != -1)
        || (posture.flip != 0 && posture.flip != 1))
    {
        return false;
    }
    else
    {
        return true;
    }
}


