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
    // double k = posture.arm * posture.elbow;
    joint.j2_ = alpha + posture.elbow * beta - arm_dh_[1].offset;
    scaleResultJoint(joint.j2_);

    // compute q3
    double d4_a3 = sqrt(arm_dh_[3].d * arm_dh_[3].d + arm_dh_[2].a * arm_dh_[2].a);
    cos_alpha = (arm_dh_[1].a * arm_dh_[1].a + d4_a3 * d4_a3 - R * R) / (2.0 * arm_dh_[1].a * d4_a3);
    if(cos_alpha < -1 || cos_alpha > 1)
    {
        return false;
    }
    sin_alpha = -1 * posture.elbow * sqrt(1 - cos_alpha * cos_alpha);
    alpha = atan2(sin_alpha, cos_alpha);

    sin_beta = arm_dh_[3].d / d4_a3;
    cos_beta = fabs(arm_dh_[2].a) / d4_a3;
    beta = atan2(sin_beta, cos_beta);
    double sin_j = -sin(beta - alpha);
    double cos_j = -cos(beta - alpha);
    joint.j3_ = atan2(sin_j, cos_j) - arm_dh_[2].offset;
    scaleResultJoint(joint.j3_);
    // compute q4
    double sign_ang5;
    TransMatrix trans_matrix_1to3;
    doFK(joint, trans_matrix_1to3, 1, 3);    // trans_matrix_tmp is from joint 1 to joint 3
    Point x3, y3, z3, x4, y4, cross_product_z3_a6;
    trans_matrix_1to3.rotation_matrix_.getVectorN(x3);
    trans_matrix_1to3.rotation_matrix_.getVectorS(y3);
    trans_matrix_1to3.rotation_matrix_.getVectorA(z3);  
    z3.crossProduct(vector_a, cross_product_z3_a6);
    if(!cross_product_z3_a6.normalize())
    {        
        return false;
    }
    if(cross_product_z3_a6.isParallel(vector_a))
    {
        return false;
    }

    double z3_a6_dot_x3, z3_a6_dot_y3;
    z3_a6_dot_x3 = cross_product_z3_a6.dotProduct(x3);
    z3_a6_dot_y3 = cross_product_z3_a6.dotProduct(y3);

    sign_ang5 = cross_product_z3_a6.y_;
    double sin_q4, cos_q4;
    if (sign_ang5 >= 0)
    {
        sign_ang5 = 1;
        if (posture.wrist == -1)
        {
            sin_q4 = -z3_a6_dot_x3;
            cos_q4 = z3_a6_dot_y3;
        }
        else 
        {
            sin_q4 = z3_a6_dot_x3;
            cos_q4 = -z3_a6_dot_y3;
        }
    } 
    else
    {
        sign_ang5 = -1;
        if (posture.wrist == -1)
        {
            sin_q4 = sign_ang5 * z3_a6_dot_x3;
            cos_q4 = -1 * sign_ang5 * z3_a6_dot_y3;
        }
        else 
        {
            sin_q4 = z3_a6_dot_x3;
            cos_q4 = -z3_a6_dot_y3;
        }
    }
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

bool KinematicsRTM::doIK(const PoseEuler& pose_euler, const Posture& posture, const Turn& turn, Joint& joint, double valve)
{
    if(doIK(pose_euler, posture, joint))
    {
        joint = getJointByGeometryJointAndTurn(joint, turn);
        return true;
    }
    else
    {
        return false;
    }
}

bool KinematicsRTM::doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, const Turn& turn, Joint& joint, double valve)
{
    if(doIK(pose_quaternion, posture, joint))
    {
        joint = getJointByGeometryJointAndTurn(joint, turn);
        return true;
    }
    else
    {
        return false;
    }
}

bool KinematicsRTM::doIK(const TransMatrix& trans_matrix, const Posture& posture, const Turn& turn, Joint& joint, double valve)
{
    if(doIK(trans_matrix, posture, joint))
    {
        joint = getJointByGeometryJointAndTurn(joint, turn);
        return true;
    }
    else
    {
        return false;
    }
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

#if 0
bool KinematicsRTM::doIK(const TransMatrix& trans_matrix, const Joint& ref_joint, Joint& joint, double valve)
{
    Posture posture;
    posture.arm = 1;
    posture.elbow = 1;
    posture.wrist = 1;
    posture.flip = flip_;

    Turn ref_turn = getTurnByJoint(ref_joint);
    Joint ref_joint_geom = getGeometryJointByJoint(ref_joint);

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
        joint.j1_ = getMostCloseJoint(joint_tmp1, joint_tmp2, ref_joint_geom.j1_, posture.arm);
    }
    else
    {
        joint.j1_ = ref_joint_geom.j1_;
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
    joint.j2_ = getMostCloseJoint(joint_tmp1, joint_tmp2, ref_joint_geom.j2_, posture.elbow);

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
    double omega;
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
        joint.j4_ = ref_joint_geom.j4_;
        goto IK_Q5;
    }
    omega = vector_s.dotProduct(z4);
    double M;
    // wrist == -1
    if(omega >= 0)
    {
        M = -1;
    }
    else
    {
        M = 1;
    }        
    sin_q4 = -M * z4.dotProduct(x3);
    cos_q4 = M * z4.dotProduct(y3); 
    joint_tmp1 = atan2(sin_q4, cos_q4) - arm_dh_[3].offset;
    scaleResultJoint(joint_tmp1);
    // wrist == 1
    if(omega >= 0)
    {
        M = 1;
    }
    else
    {
        M = -1;
    }        
    sin_q4 = -M * z4.dotProduct(x3);
    cos_q4 = M * z4.dotProduct(y3);
    joint_tmp2 = atan2(sin_q4, cos_q4) - arm_dh_[3].offset;
    scaleResultJoint(joint_tmp2);
    // get the most close joint
    joint.j4_ = getMostCloseJoint(joint_tmp1, joint_tmp2, ref_joint_geom.j4_, posture.wrist);    // wrist is not used later

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

    joint = getJointByGeometryJointAndRefJointAndTurn(joint, ref_joint, ref_turn);
    
    return true;
}
#endif

#if 1
bool KinematicsRTM::doIK(const TransMatrix& trans_matrix, const Joint& ref_joint, Joint& joint, double valve)
{
    Posture posture;
    posture.arm = 1;
    posture.elbow = 1;
    posture.wrist = 1;
    posture.flip = flip_;

    Turn ref_turn = getTurnByJoint(ref_joint);
    Joint ref_joint_geom = getGeometryJointByJoint(ref_joint);

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
        joint.j1_ = getMostCloseJoint(joint_tmp1, joint_tmp2, ref_joint_geom.j1_, posture.arm);
    }
    else
    {
        joint.j1_ = ref_joint_geom.j1_;
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
    // double k = posture.arm * posture.elbow;
    joint_tmp1 = alpha + posture.elbow * beta - arm_dh_[1].offset;
    scaleResultJoint(joint_tmp1);
    joint_tmp2 = alpha - posture.elbow * beta - arm_dh_[1].offset;
    scaleResultJoint(joint_tmp2);
    joint.j2_ = getMostCloseJoint(joint_tmp1, joint_tmp2, ref_joint_geom.j2_, posture.elbow);

    // compute q3
    double d4_a3 = sqrt(arm_dh_[3].d * arm_dh_[3].d + arm_dh_[2].a * arm_dh_[2].a);
    cos_alpha = (arm_dh_[1].a * arm_dh_[1].a + d4_a3 * d4_a3 - R * R) / (2.0 * arm_dh_[1].a * d4_a3);
    if(cos_alpha < -1 || cos_alpha > 1)
    {
        return false;
    }
    sin_alpha = -posture.elbow * sqrt(1 - cos_alpha * cos_alpha);
    alpha = atan2(sin_alpha, cos_alpha);

    sin_beta = arm_dh_[3].d / d4_a3;    
    cos_beta = fabs(arm_dh_[2].a) / d4_a3;
    beta = atan2(sin_beta, cos_beta);
    double sin_j = -sin(beta - alpha);
    double cos_j = -cos(beta - alpha);
    joint.j3_ = atan2(sin_j, cos_j) - arm_dh_[2].offset;
    scaleResultJoint(joint.j3_);

    // compute q4
    double sign_ang5;
    TransMatrix trans_matrix_1to3;
    doFK(joint, trans_matrix_1to3, 1, 3);    // trans_matrix_tmp is from joint 1 to joint 3
    Point x3, y3, z3, x4, y4, cross_product_z3_a6;
    trans_matrix_1to3.rotation_matrix_.getVectorN(x3);
    trans_matrix_1to3.rotation_matrix_.getVectorS(y3);
    trans_matrix_1to3.rotation_matrix_.getVectorA(z3);  
    z3.crossProduct(vector_a, cross_product_z3_a6);  
    if(!cross_product_z3_a6.normalize()
        || cross_product_z3_a6.isParallel(vector_a))
    {
        joint.j4_ = ref_joint_geom.j4_;
        goto IK_Q5;
    }

    double z3_a6_dot_x3, z3_a6_dot_y3;
    z3_a6_dot_x3 = cross_product_z3_a6.dotProduct(x3);
    z3_a6_dot_y3 = cross_product_z3_a6.dotProduct(y3);

    sign_ang5 = cross_product_z3_a6.y_;
    double sin_q4, cos_q4;
    sign_ang5 = sign_ang5 >= 0?1:-1;

    sin_q4 = -sign_ang5 * z3_a6_dot_x3;
    cos_q4 = sign_ang5 * z3_a6_dot_y3;
    joint_tmp1 = atan2(sin_q4, cos_q4) - arm_dh_[3].offset;
    scaleResultJoint(joint_tmp1);

    sin_q4 = sign_ang5 * z3_a6_dot_x3;
    cos_q4 = -sign_ang5 * z3_a6_dot_y3;
    joint_tmp2 = atan2(sin_q4, cos_q4) - arm_dh_[3].offset;
    scaleResultJoint(joint_tmp2);

    // get the most close joint
    joint.j4_ = getMostCloseJoint(joint_tmp1, joint_tmp2, ref_joint_geom.j4_, posture.wrist);    // wrist is not used later

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

    joint = getJointByGeometryJointAndRefJointAndTurn(joint, ref_joint, ref_turn);
    
    return true;
}
#endif

Posture KinematicsRTM::getPostureByJoint(const Joint& joint, double valve)
{
    Posture posture;

    // arm
    TransMatrix trans_matrix_1to6;
    doFK(joint, trans_matrix_1to6, 1, 6);
    Point vector_n6, vector_s6, vector_a6;
    trans_matrix_1to6.rotation_matrix_.getVectorN(vector_n6); 
    trans_matrix_1to6.rotation_matrix_.getVectorS(vector_s6); 
    trans_matrix_1to6.rotation_matrix_.getVectorA(vector_a6); 
    Point p4_vector = trans_matrix_1to6.trans_vector_ - vector_a6 * arm_dh_[5].d;

    TransMatrix trans_matrix_1to2;
    doFK(joint, trans_matrix_1to2, 1, 2);
    // TransMatrix trans_matrix_1to2(arm_dh_[0].d, arm_dh_[0].a, arm_dh_[0].alpha, joint.j1_ + arm_dh_[0].offset); // to test

    Point z0;
    z0.x_ = 0; z0.y_ = 0; z0.z_ = 1;

    Point z1;
    trans_matrix_1to2.rotation_matrix_.getVectorA(z1);
    Point cross_product_z1_p4;
    z1.crossProduct(p4_vector, cross_product_z1_p4);

    double g_theta = z0.dotProduct(cross_product_z1_p4);

    if(g_theta >= 0)
    {
        posture.arm = 1;
    }
    else
    {
        posture.arm = -1;
    }
    
    // elbow
    TransMatrix trans_matrix_3to4;
    doFK(joint, trans_matrix_3to4, 3, 4);
    double point_y = trans_matrix_3to4.trans_vector_.y_;

    // double elbow = -arm_dh_[3].d * cos(joint.j3_) + arm_dh_[2].a * sin(joint.j3_);
    if(point_y >= 0)
    {
        posture.elbow = -1;
    }
    else
    {
        posture.elbow = 1;
    }

    // wrist 
    TransMatrix trans_matrix_1to3;
    doFK(joint, trans_matrix_1to3, 1, 3);
    Point z3;
    trans_matrix_1to3.rotation_matrix_.getVectorA(z3); 

    Point cross_product_w_vector;
    z3.crossProduct(vector_a6, cross_product_w_vector);



    TransMatrix trans_matrix_1to4;
    doFK(joint, trans_matrix_1to4, 1, 4);

    Point z4;
    trans_matrix_1to4.rotation_matrix_.getVectorA(z4);

    double w_vector_dot_z4 = cross_product_w_vector.dotProduct(z4);

    if(w_vector_dot_z4 > 0)
    {
        posture.wrist = -1;
    }
    else
    {
        posture.wrist = 1;
    }

    posture.flip = flip_;
    return posture;
}

Turn KinematicsRTM::getTurnByJoint(const Joint& joint)
{
    Turn turn;
    if(joint.j1_ >= 0)
    {
        turn.j1 = floor((fabs(joint.j1_) + M_PI) / (2 * M_PI)); 
    }
    else
    {
        turn.j1 = -floor((fabs(joint.j1_) + M_PI) / (2 * M_PI)); 
    }
    if(joint.j2_ >= 0)
    {
        turn.j2 = floor((fabs(joint.j2_) + M_PI) / (2 * M_PI)); 
    }
    else
    {
        turn.j2 = -floor((fabs(joint.j2_) + M_PI) / (2 * M_PI)); 
    }
    if(joint.j3_ >= 0)
    {
        turn.j3 = floor((fabs(joint.j3_) + M_PI) / (2 * M_PI)); 
    }
    else
    {
        turn.j3 = -floor((fabs(joint.j3_) + M_PI) / (2 * M_PI)); 
    }
    if(joint.j4_ >= 0)
    {
        turn.j4 = floor((fabs(joint.j4_) + M_PI) / (2 * M_PI)); 
    }
    else
    {
        turn.j4 = -floor((fabs(joint.j4_) + M_PI) / (2 * M_PI)); 
    }    
    if(joint.j5_ >= 0)
    {
        turn.j5 = floor((fabs(joint.j5_) + M_PI) / (2 * M_PI)); 
    }
    else
    {
        turn.j5 = -floor((fabs(joint.j5_) + M_PI) / (2 * M_PI)); 
    }
    if(joint.j6_ >= 0)
    {
        turn.j6 = floor((fabs(joint.j6_) + M_PI) / (2 * M_PI)); 
    }
    else
    {
        turn.j6 = -floor((fabs(joint.j6_) + M_PI) / (2 * M_PI)); 
    }
    turn.j7 = 0;
    turn.j8 = 0;
    turn.j9 = 0;
    return turn;
}

Joint KinematicsRTM::getGeometryJointByJoint(const Joint& joint)
{
    Joint gemo_joint;
    Turn turn = getTurnByJoint(joint);
    gemo_joint.j1_ = joint.j1_ - turn.j1 * 2 * M_PI;
    gemo_joint.j2_ = joint.j2_ - turn.j2 * 2 * M_PI;
    gemo_joint.j3_ = joint.j3_ - turn.j3 * 2 * M_PI;
    gemo_joint.j4_ = joint.j4_ - turn.j4 * 2 * M_PI;
    gemo_joint.j5_ = joint.j5_ - turn.j5 * 2 * M_PI;
    gemo_joint.j6_ = joint.j6_ - turn.j6 * 2 * M_PI;
    return gemo_joint;
}

Joint KinematicsRTM::getJointByGeometryJointAndTurn(const Joint& geom_joint, const Turn& turn)
{
    Joint joint;
    joint.j1_ = geom_joint.j1_ + turn.j1 * 2 * M_PI;
    joint.j2_ = geom_joint.j2_ + turn.j2 * 2 * M_PI;
    joint.j3_ = geom_joint.j3_ + turn.j3 * 2 * M_PI;
    joint.j4_ = geom_joint.j4_ + turn.j4 * 2 * M_PI;
    joint.j5_ = geom_joint.j5_ + turn.j5 * 2 * M_PI;
    joint.j6_ = geom_joint.j6_ + turn.j6 * 2 * M_PI;
    return joint;
}

double KinematicsRTM::getSingleJointByGeometryJointAndRefJointAndTurn(const double& geom_single_joint, const double& ref_single_joint, const int& single_turn)
{
    double turn_joint[4], delta_joint[4], single_joint;
    turn_joint[0] = geom_single_joint;
    turn_joint[1] = geom_single_joint + single_turn * 2 * M_PI;
    turn_joint[2] = turn_joint[1] - 2 * M_PI;
    turn_joint[3] = turn_joint[1] + 2 * M_PI;

    delta_joint[0] = fabs(turn_joint[0] - ref_single_joint);
    delta_joint[1] = fabs(turn_joint[1] - ref_single_joint);
    delta_joint[2] = fabs(turn_joint[2] - ref_single_joint);
    delta_joint[3] = fabs(turn_joint[3] - ref_single_joint);

    single_joint = turn_joint[0];
    double min_delta_joint = delta_joint[0];
    for (int i = 1; i != 4; ++i)
    {
        if (delta_joint[i] < min_delta_joint)
        {
            single_joint = turn_joint[i];
            min_delta_joint = delta_joint[i];
        } 
    }

    return single_joint;
}

Joint KinematicsRTM::getJointByGeometryJointAndRefJointAndTurn(const Joint& geom_joint, const Joint& ref_joint, const Turn& turn)
{
    Joint joint;
    joint.j1_ = getSingleJointByGeometryJointAndRefJointAndTurn(geom_joint.j1_, ref_joint.j1_, turn.j1);
    joint.j2_ = getSingleJointByGeometryJointAndRefJointAndTurn(geom_joint.j2_, ref_joint.j2_, turn.j2);
    joint.j3_ = getSingleJointByGeometryJointAndRefJointAndTurn(geom_joint.j3_, ref_joint.j3_, turn.j3);
    joint.j4_ = getSingleJointByGeometryJointAndRefJointAndTurn(geom_joint.j4_, ref_joint.j4_, turn.j4);
    joint.j5_ = getSingleJointByGeometryJointAndRefJointAndTurn(geom_joint.j5_, ref_joint.j5_, turn.j5);
    joint.j6_ = getSingleJointByGeometryJointAndRefJointAndTurn(geom_joint.j6_, ref_joint.j6_, turn.j6);
    return joint;
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

