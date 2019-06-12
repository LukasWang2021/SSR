#include "kinematics_rtm.h"
#include <iostream>
#include <math.h>


using namespace std;
using namespace basic_alg;


KinematicsRTM::KinematicsRTM():
    is_valid_(false)
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
    is_valid_ = true;
}

KinematicsRTM::KinematicsRTM(std::string file_path, bool is_flip)
{
    file_path_ = file_path + "arm_dh.yaml";
    if (param_.loadParamFile(file_path_))
    {
        if (param_.getParam("base_dh/d", base_dh_.d) &&
            param_.getParam("base_dh/a", base_dh_.a) &&
            param_.getParam("base_dh/alpha", base_dh_.alpha) &&
            param_.getParam("base_dh/offset", base_dh_.offset) &&
            param_.getParam("arm_dh/axis-0/d", arm_dh_[0].d) &&
            param_.getParam("arm_dh/axis-0/a", arm_dh_[0].a) &&
            param_.getParam("arm_dh/axis-0/alpha", arm_dh_[0].alpha) &&
            param_.getParam("arm_dh/axis-0/offset", arm_dh_[0].offset) &&
            param_.getParam("arm_dh/axis-1/d", arm_dh_[1].d) &&
            param_.getParam("arm_dh/axis-1/a", arm_dh_[1].a) &&
            param_.getParam("arm_dh/axis-1/alpha", arm_dh_[1].alpha) &&
            param_.getParam("arm_dh/axis-1/offset", arm_dh_[1].offset) &&
            param_.getParam("arm_dh/axis-2/d", arm_dh_[2].d) &&
            param_.getParam("arm_dh/axis-2/a", arm_dh_[2].a) &&
            param_.getParam("arm_dh/axis-2/alpha", arm_dh_[2].alpha) &&
            param_.getParam("arm_dh/axis-2/offset", arm_dh_[2].offset) &&
            param_.getParam("arm_dh/axis-3/d", arm_dh_[3].d) &&
            param_.getParam("arm_dh/axis-3/a", arm_dh_[3].a) &&
            param_.getParam("arm_dh/axis-3/alpha", arm_dh_[3].alpha) &&
            param_.getParam("arm_dh/axis-3/offset", arm_dh_[3].offset) &&
            param_.getParam("arm_dh/axis-4/d", arm_dh_[4].d) &&
            param_.getParam("arm_dh/axis-4/a", arm_dh_[4].a) &&
            param_.getParam("arm_dh/axis-4/alpha", arm_dh_[4].alpha) &&
            param_.getParam("arm_dh/axis-4/offset", arm_dh_[4].offset) &&
            param_.getParam("arm_dh/axis-5/d", arm_dh_[5].d) &&
            param_.getParam("arm_dh/axis-5/a", arm_dh_[5].a) &&
            param_.getParam("arm_dh/axis-5/alpha", arm_dh_[5].alpha) &&
            param_.getParam("arm_dh/axis-5/offset", arm_dh_[5].offset))
        {
            TransMatrix matrix_base(base_dh_.d, base_dh_.a, base_dh_.alpha, base_dh_.offset);
            matrix_base_ = matrix_base;
            matrix_base_.inverse(matrix_base_inv_);
            if(is_flip)
            {
                flip_ = 1;
            }
            else
            {
                flip_ = 0;
            }
            is_valid_ = true;          
        }
        else
        {
            is_valid_ = false;
        }
    }
    else
    {
        is_valid_ = false;
    }    
}

KinematicsRTM::~KinematicsRTM()
{

}

bool KinematicsRTM::isValid()
{
    return is_valid_;
}

bool KinematicsRTM::getDH(DH& base_dh, DH arm_dh[6])
{
    base_dh.d = base_dh_.d;
    base_dh.a = base_dh_.a;
    base_dh.alpha = base_dh_.alpha;
    base_dh.offset = base_dh_.offset;

    for(size_t i = 0; i < 6; ++i)
    {
        arm_dh[i].d = arm_dh_[i].d;
        arm_dh[i].a = arm_dh_[i].a;
        arm_dh[i].alpha = arm_dh_[i].alpha;
        arm_dh[i].offset = arm_dh_[i].offset;
    }
    return true;
}

bool KinematicsRTM::setDH(DH& base_dh, DH arm_dh[6])
{
    if (param_.setParam("base_dh/d", base_dh.d) &&
        param_.setParam("base_dh/a", base_dh.a) &&
        param_.setParam("base_dh/alpha", base_dh.alpha) &&
        param_.setParam("base_dh/offset", base_dh.offset) &&
        param_.setParam("arm_dh/axis-0/d", arm_dh[0].d) &&
        param_.setParam("arm_dh/axis-0/a", arm_dh[0].a) &&
        param_.setParam("arm_dh/axis-0/alpha", arm_dh[0].alpha) &&
        param_.setParam("arm_dh/axis-0/offset", arm_dh[0].offset) &&
        param_.setParam("arm_dh/axis-1/d", arm_dh[1].d) &&
        param_.setParam("arm_dh/axis-1/a", arm_dh[1].a) &&
        param_.setParam("arm_dh/axis-1/alpha", arm_dh[1].alpha) &&
        param_.setParam("arm_dh/axis-1/offset", arm_dh[1].offset) &&
        param_.setParam("arm_dh/axis-2/d", arm_dh[2].d) &&
        param_.setParam("arm_dh/axis-2/a", arm_dh[2].a) &&
        param_.setParam("arm_dh/axis-2/alpha", arm_dh[2].alpha) &&
        param_.setParam("arm_dh/axis-2/offset", arm_dh[2].offset) &&
        param_.setParam("arm_dh/axis-3/d", arm_dh[3].d) &&
        param_.setParam("arm_dh/axis-3/a", arm_dh[3].a) &&
        param_.setParam("arm_dh/axis-3/alpha", arm_dh[3].alpha) &&
        param_.setParam("arm_dh/axis-3/offset", arm_dh[3].offset) &&
        param_.setParam("arm_dh/axis-4/d", arm_dh[4].d) &&
        param_.setParam("arm_dh/axis-4/a", arm_dh[4].a) &&
        param_.setParam("arm_dh/axis-4/alpha", arm_dh[4].alpha) &&
        param_.setParam("arm_dh/axis-4/offset", arm_dh[4].offset) &&
        param_.setParam("arm_dh/axis-5/d", arm_dh[5].d) &&
        param_.setParam("arm_dh/axis-5/a", arm_dh[5].a) &&
        param_.setParam("arm_dh/axis-5/alpha", arm_dh[5].alpha) &&
        param_.setParam("arm_dh/axis-5/offset", arm_dh[5].offset))
    {
        if(!param_.dumpParamFile(file_path_))
        {
            return false;
        }        
        base_dh_.d = base_dh.d;
        base_dh_.a = base_dh.a;
        base_dh_.alpha = base_dh.alpha;
        base_dh_.offset = base_dh.offset;
        for(size_t i = 0; i < 6; ++i)
        {
            arm_dh_[i].d = arm_dh[i].d;
            arm_dh_[i].a = arm_dh[i].a;
            arm_dh_[i].alpha = arm_dh[i].alpha;
            arm_dh_[i].offset = arm_dh[i].offset;
        }        
    
        TransMatrix matrix_base(base_dh_.d, base_dh_.a, base_dh_.alpha, base_dh_.offset);
        matrix_base_ = matrix_base;
        matrix_base_.inverse(matrix_base_inv_);
        return true;
    }
    else
    {
        return false;
    }
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

bool KinematicsRTM::doIK(const PoseEuler& pose_euler, const Posture& posture, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_euler.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, joint, valve);
}

bool KinematicsRTM::doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_quaternion.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, joint, valve);
}

bool KinematicsRTM::doIK(const TransMatrix& trans_matrix, const Posture& posture, Joint& joint, double valve)
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
    double r = sqrt(p_vector.x_ * p_vector.x_ + p_vector.y_ * p_vector.y_) - (posture.arm * arm_dh_[0].a);
    double R = sqrt(r * r + p_vector.z_ * p_vector.z_);
    double sin_alpha = p_vector.z_ / R;
    double cos_alpha = posture.arm * r / R; 
    double alpha = atan2(sin_alpha, cos_alpha);    
    double cos_beta = (arm_dh_[1].a * arm_dh_[1].a + R * R - arm_dh_[3].d * arm_dh_[3].d - arm_dh_[2].a * arm_dh_[2].a) / (2.0 * arm_dh_[1].a * R);
    if(cos_beta < -1 || cos_beta > 1)
    {
        return false;
    }
    double sin_beta = sqrt(1 - cos_beta * cos_beta);
    double beta = atan2(sin_beta, cos_beta);
    double k = posture.arm * posture.elbow;
    joint.j2_ = alpha + k * beta - arm_dh_[1].offset;
    scaleResultJoint(joint.j2_);

    // compute q3
    double d4_a3 = sqrt(arm_dh_[3].d * arm_dh_[3].d + arm_dh_[2].a * arm_dh_[2].a);
    cos_alpha = (arm_dh_[1].a * arm_dh_[1].a + d4_a3 * d4_a3 - R * R) / (2.0 * arm_dh_[1].a * d4_a3);
    if(cos_alpha < -1 || cos_alpha > 1)
    {
        return false;
    }
    sin_alpha = -k * sqrt(1 - cos_alpha * cos_alpha);
    sin_beta = arm_dh_[3].d / d4_a3;
    alpha = atan2(sin_alpha, cos_alpha);
    sin_beta = arm_dh_[3].d / d4_a3;
    cos_beta = fabs(arm_dh_[2].a) / d4_a3;
    beta = atan2(sin_beta, cos_beta);
    double sin_j = -sin(beta - alpha);
    double cos_j = -cos(beta - alpha);
    joint.j3_ = atan2(sin_j, cos_j) - arm_dh_[2].offset;
    scaleResultJoint(joint.j3_);

    // compute q4
    TransMatrix trans_matrix_1to3;
    doFK(joint, trans_matrix_1to3, 1, 3);    // trans_matrix_tmp is from joint 1 to joint 3
    Point x3, y3, z3, x4, y4, z4;
    trans_matrix_1to3.rotation_matrix_.getVectorN(x3);
    trans_matrix_1to3.rotation_matrix_.getVectorS(y3);
    trans_matrix_1to3.rotation_matrix_.getVectorA(z3);  
    z3.crossProduct(vector_a, z4);  
    if(!z4.normalize()
        || z4.isParallel(vector_a))
    {
        return false;
    }
    if(posture.wrist < 0)
    {
        z4.reverse();
    }
    double sin_q4, cos_q4;
    sin_q4 = -z4.dotProduct(x3);
    cos_q4 = z4.dotProduct(y3);
    joint.j4_ = atan2(sin_q4, cos_q4) - arm_dh_[3].offset;
    scaleResultJoint(joint.j4_);

    // compute q5
    TransMatrix trans_matrix_1to4;
    doFK(joint, trans_matrix_1to4, 1, 4);
    trans_matrix_1to4.rotation_matrix_.getVectorN(x4);
    trans_matrix_1to4.rotation_matrix_.getVectorS(y4);
    double sin_q5, cos_q5;
    sin_q5 = vector_a.dotProduct(x4);
    cos_q5 = -vector_a.dotProduct(y4);
    joint.j5_ = atan2(sin_q5, cos_q5) - arm_dh_[4].offset;
    scaleResultJoint(joint.j5_);

    // compute q6
    TransMatrix trans_matrix_1to5;
    doFK(joint, trans_matrix_1to5, 1, 5);
    Point y6;
    trans_matrix_1to5.rotation_matrix_.getVectorS(y6);    
    sin_j = vector_n.dotProduct(y6);
    cos_j = vector_s.dotProduct(y6);
    joint.j6_ = atan2(sin_j, cos_j) - arm_dh_[5].offset;
    scaleResultJoint(joint.j6_);

    return true;
}

bool KinematicsRTM::doIK(const PoseEuler& pose_euler, const Joint& ref_joint, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_euler.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, ref_joint, joint, valve);
}

bool KinematicsRTM::doIK(const PoseQuaternion& pose_quaternion, const Joint& ref_joint, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_quaternion.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, ref_joint, joint, valve);
}

bool KinematicsRTM::doIK(const TransMatrix& trans_matrix, const Joint& ref_joint, Joint& joint, double valve)
{
    Posture posture;
    posture.arm = 1;
    posture.elbow = 1;
    posture.wrist = 1;
    posture.flip = flip_;

    TransMatrix trans_matrix_1to6;
    TransMatrix* trans_matrix_ptr = const_cast<TransMatrix*>(&trans_matrix);
    trans_matrix_ptr->leftMultiply(matrix_base_inv_, trans_matrix_1to6);
    Point vector_n, vector_s, vector_a;
    double joint_tmp1, joint_tmp2;
    trans_matrix_1to6.rotation_matrix_.getVectorN(vector_n);
    trans_matrix_1to6.rotation_matrix_.getVectorS(vector_s);
    trans_matrix_1to6.rotation_matrix_.getVectorA(vector_a);   
    Point p_vector = trans_matrix_1to6.trans_vector_ - vector_a * arm_dh_[5].d;
    
    // compute q1
    if(fabs(p_vector.x_) > valve 
        || fabs(p_vector.y_) > valve)
    {
        joint_tmp1 = atan2(posture.arm * p_vector.y_, posture.arm * p_vector.x_) - arm_dh_[0].offset;
        scaleResultJoint(joint_tmp1);
        joint_tmp2 = atan2(-posture.arm * p_vector.y_, -posture.arm * p_vector.x_) - arm_dh_[0].offset;
        scaleResultJoint(joint_tmp2);
        joint.j1_ = getMostCloseJoint(joint_tmp1, joint_tmp2, ref_joint.j1_, posture.arm);
    }
    else
    {
        joint.j1_ = ref_joint.j1_;
    }

    // compute q2
    double r = sqrt(p_vector.x_ * p_vector.x_ + p_vector.y_ * p_vector.y_) - (posture.arm * arm_dh_[0].a);
    double R = sqrt(r * r + p_vector.z_ * p_vector.z_);
    double sin_alpha = p_vector.z_ / R;
    double cos_alpha = posture.arm * r / R; 
    double alpha = atan2(sin_alpha, cos_alpha);    
    double cos_beta = (arm_dh_[1].a * arm_dh_[1].a + R * R - arm_dh_[3].d * arm_dh_[3].d - arm_dh_[2].a * arm_dh_[2].a) / (2.0 * arm_dh_[1].a * R);
    if(cos_beta < -1 || cos_beta > 1)
    {
        return false;
    }
    double sin_beta = sqrt(1 - cos_beta * cos_beta);
    double beta = atan2(sin_beta, cos_beta);
    double k = posture.arm * posture.elbow;
    joint_tmp1 = alpha + k * beta - arm_dh_[1].offset;
    scaleResultJoint(joint_tmp1);
    joint_tmp2 = alpha - k * beta - arm_dh_[1].offset;
    scaleResultJoint(joint_tmp2);
    joint.j2_ = getMostCloseJoint(joint_tmp1, joint_tmp2, ref_joint.j2_, posture.elbow);

    // compute q3
    double d4_a3 = sqrt(arm_dh_[3].d * arm_dh_[3].d + arm_dh_[2].a * arm_dh_[2].a);
    cos_alpha = (arm_dh_[1].a * arm_dh_[1].a + d4_a3 * d4_a3 - R * R) / (2.0 * arm_dh_[1].a * d4_a3);
    if(cos_alpha < -1 || cos_alpha > 1)
    {
        return false;
    }
    sin_alpha = -k * sqrt(1 - cos_alpha * cos_alpha);
    sin_beta = arm_dh_[3].d / d4_a3;
    alpha = atan2(sin_alpha, cos_alpha);
    sin_beta = arm_dh_[3].d / d4_a3;
    cos_beta = fabs(arm_dh_[2].a) / d4_a3;
    beta = atan2(sin_beta, cos_beta);
    double sin_j = -sin(beta - alpha);
    double cos_j = -cos(beta - alpha);
    joint.j3_ = atan2(sin_j, cos_j) - arm_dh_[2].offset;
    scaleResultJoint(joint.j3_);

    // compute q4
    double sin_q4, cos_q4;
    TransMatrix trans_matrix_1to3;
    doFK(joint, trans_matrix_1to3, 1, 3);    // trans_matrix_tmp is from joint 1 to joint 3
    Point x3, y3, z3, x4, y4, z4, z4_tmp1, z4_tmp2;
    trans_matrix_1to3.rotation_matrix_.getVectorN(x3);
    trans_matrix_1to3.rotation_matrix_.getVectorS(y3);
    trans_matrix_1to3.rotation_matrix_.getVectorA(z3);  
    z3.crossProduct(vector_a, z4);  
    if(!z4.normalize()
        || z4.isParallel(vector_a))
    {
        joint.j4_ = ref_joint.j4_;
        goto IK_Q5;
    }

    // wrist == -1
    z4_tmp1 = z4;
    z4_tmp1.reverse();
    sin_q4 = -z4_tmp1.dotProduct(x3);
    cos_q4 = z4_tmp1.dotProduct(y3);
    joint_tmp1 = atan2(sin_q4, cos_q4) - arm_dh_[3].offset;
    scaleResultJoint(joint_tmp1);
    // wrist ==1
    z4_tmp2 = z4;
    sin_q4 = -z4_tmp2.dotProduct(x3);
    cos_q4 = z4_tmp2.dotProduct(y3);
    joint_tmp2 = atan2(sin_q4, cos_q4) - arm_dh_[3].offset;
    scaleResultJoint(joint_tmp2);
    // get the most close joint
    joint.j4_ = getMostCloseJoint(joint_tmp1, joint_tmp2, ref_joint.j4_, posture.wrist);    // wrist is not used later

IK_Q5:
    // compute q5
    TransMatrix trans_matrix_1to4;
    doFK(joint, trans_matrix_1to4, 1, 4);
    trans_matrix_1to4.rotation_matrix_.getVectorN(x4);
    trans_matrix_1to4.rotation_matrix_.getVectorS(y4);
    double sin_q5, cos_q5;
    sin_q5 = vector_a.dotProduct(x4);
    cos_q5 = -vector_a.dotProduct(y4);
    joint.j5_ = atan2(sin_q5, cos_q5) - arm_dh_[4].offset;
    scaleResultJoint(joint.j5_);

    // compute q6
    TransMatrix trans_matrix_1to5;
    doFK(joint, trans_matrix_1to5, 1, 5);
    Point y6;
    trans_matrix_1to5.rotation_matrix_.getVectorS(y6);    
    sin_j = vector_n.dotProduct(y6);
    cos_j = vector_s.dotProduct(y6);
    joint.j6_ = atan2(sin_j, cos_j) - arm_dh_[5].offset;
    scaleResultJoint(joint.j6_);
    
    return true;
}

bool KinematicsRTM::doIK(const PoseEuler& pose_euler, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_euler.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, ref_joint, joint, valve);
}

bool KinematicsRTM::doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_quaternion.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, ref_joint, joint, valve);
}

bool KinematicsRTM::doIK(const TransMatrix& trans_matrix, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve)
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
        joint.j1_ = ref_joint.j1_;
    }
    scaleResultJoint(joint.j1_);

    // compute q2
    double r = sqrt(p_vector.x_ * p_vector.x_ + p_vector.y_ * p_vector.y_) - (posture.arm * arm_dh_[0].a);
    double R = sqrt(r * r + p_vector.z_ * p_vector.z_);
    double sin_alpha = p_vector.z_ / R;
    double cos_alpha = posture.arm * r / R; 
    double alpha = atan2(sin_alpha, cos_alpha);    
    double cos_beta = (arm_dh_[1].a * arm_dh_[1].a + R * R - arm_dh_[3].d * arm_dh_[3].d - arm_dh_[2].a * arm_dh_[2].a) / (2.0 * arm_dh_[1].a * R);
    if(cos_beta < -1 || cos_beta > 1)
    {
        return false;
    }
    double sin_beta = sqrt(1 - cos_beta * cos_beta);
    double beta = atan2(sin_beta, cos_beta);
    double k = posture.arm * posture.elbow;
    joint.j2_ = alpha + k * beta - arm_dh_[1].offset;
    scaleResultJoint(joint.j2_);

    // compute q3
    double d4_a3 = sqrt(arm_dh_[3].d * arm_dh_[3].d + arm_dh_[2].a * arm_dh_[2].a);
    cos_alpha = (arm_dh_[1].a * arm_dh_[1].a + d4_a3 * d4_a3 - R * R) / (2.0 * arm_dh_[1].a * d4_a3);
    if(cos_alpha < -1 || cos_alpha > 1)
    {
        return false;
    }
    sin_alpha = -k * sqrt(1 - cos_alpha * cos_alpha);
    sin_beta = arm_dh_[3].d / d4_a3;
    alpha = atan2(sin_alpha, cos_alpha);
    sin_beta = arm_dh_[3].d / d4_a3;
    cos_beta = fabs(arm_dh_[2].a) / d4_a3;
    beta = atan2(sin_beta, cos_beta);
    double sin_j = -sin(beta - alpha);
    double cos_j = -cos(beta - alpha);
    joint.j3_ = atan2(sin_j, cos_j) - arm_dh_[2].offset;
    scaleResultJoint(joint.j3_);

    // compute q4
    double sin_q4, cos_q4;
    TransMatrix trans_matrix_1to3;
    doFK(joint, trans_matrix_1to3, 1, 3);    // trans_matrix_tmp is from joint 1 to joint 3
    Point x3, y3, z3, x4, y4, z4;
    trans_matrix_1to3.rotation_matrix_.getVectorN(x3);
    trans_matrix_1to3.rotation_matrix_.getVectorS(y3);
    trans_matrix_1to3.rotation_matrix_.getVectorA(z3);  
    z3.crossProduct(vector_a, z4);  
    if(!z4.normalize()
        || z4.isParallel(vector_a))
    {
        joint.j4_ = ref_joint.j4_;
        goto IK_Q5;
    }
    if(posture.wrist < 0)
    {
        z4.reverse();
    }
    
    sin_q4 = -z4.dotProduct(x3);
    cos_q4 = z4.dotProduct(y3);
    joint.j4_ = atan2(sin_q4, cos_q4) - arm_dh_[3].offset;
    scaleResultJoint(joint.j4_);
IK_Q5:
    // compute q5
    TransMatrix trans_matrix_1to4;
    doFK(joint, trans_matrix_1to4, 1, 4);
    trans_matrix_1to4.rotation_matrix_.getVectorN(x4);
    trans_matrix_1to4.rotation_matrix_.getVectorS(y4);
    double sin_q5, cos_q5;
    sin_q5 = vector_a.dotProduct(x4);
    cos_q5 = -vector_a.dotProduct(y4);
    joint.j5_ = atan2(sin_q5, cos_q5) - arm_dh_[4].offset;
    scaleResultJoint(joint.j5_);

    // compute q6
    TransMatrix trans_matrix_1to5;
    doFK(joint, trans_matrix_1to5, 1, 5);
    Point y6;
    trans_matrix_1to5.rotation_matrix_.getVectorS(y6);    
    sin_j = vector_n.dotProduct(y6);
    cos_j = vector_s.dotProduct(y6);
    joint.j6_ = atan2(sin_j, cos_j) - arm_dh_[5].offset;
    scaleResultJoint(joint.j6_);

    return true;
}

Posture KinematicsRTM::getPostureByJoint(const Joint& joint, double valve)
{
    Posture posture;

    // arm
    TransMatrix trans_matrix_1to6;
    doFK(joint, trans_matrix_1to6, 1, 6);
    Point vector_a6;
    trans_matrix_1to6.rotation_matrix_.getVectorA(vector_a6); 
    Point p4_vector = trans_matrix_1to6.trans_vector_ - vector_a6 * arm_dh_[5].d;
    TransMatrix trans_matrix_1to2(arm_dh_[0].d, arm_dh_[0].a, arm_dh_[0].alpha, joint.j1_ + arm_dh_[0].offset);
    Point z1;
    trans_matrix_1to2.rotation_matrix_.getVectorA(z1);
    Point cross_product_z1_p4;
    z1.crossProduct(p4_vector, cross_product_z1_p4);
    if(cross_product_z1_p4.z_ >= 0)
    {
        posture.arm = 1;
    }
    else
    {
        posture.arm = -1;
    }
    
    // elbow
    double elbow = -arm_dh_[3].d * cos(joint.j3_) + arm_dh_[2].a * sin(joint.j3_);
    if(elbow >= 0)
    {
        posture.elbow = -posture.arm;
    }
    else
    {
        posture.elbow = posture.arm;
    }

    // wrist
    TransMatrix trans_matrix_1to3;
    doFK(joint, trans_matrix_1to3, 1, 3);
    Point y3;
    trans_matrix_1to3.rotation_matrix_.getVectorS(y3);
    TransMatrix trans_matrix_1to4;
    doFK(joint, trans_matrix_1to4, 1, 4);
    Point x4, z4;
    trans_matrix_1to4.rotation_matrix_.getVectorN(x4);
    trans_matrix_1to4.rotation_matrix_.getVectorA(z4);
    double cos_q4 = z4.dotProduct(y3);
    double sin_q5 = vector_a6.dotProduct(x4);
    if(cos_q4*sin_q5 >= 0)
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

inline bool KinematicsRTM::isPostureValid(const Posture& posture)
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

inline double KinematicsRTM::getMostCloseJoint(double joint1, double joint2, double ref_joint, int& posture1)
{
    double delta1 = fabs(joint1 - ref_joint);
    if(delta1 > M_PI)
    {
        delta1 = fabs(delta1 - 2 * M_PI);
    }
    double delta2 = fabs(joint2 - ref_joint);
    if(delta2 > M_PI)
    {
        delta2 = fabs(delta2 - 2 * M_PI);
    }
    if(delta1 > delta2)
    {
        posture1 = -posture1;
        return joint2;
    }
    else
    {
        return joint1;
    }
}

