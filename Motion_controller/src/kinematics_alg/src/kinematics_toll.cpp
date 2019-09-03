#include "kinematics_toll.h"
#include <iostream>
#include <math.h>


using namespace std;
using namespace basic_alg;


KinematicsToll::KinematicsToll():
    is_valid_(false)
{

}

KinematicsToll::KinematicsToll(DH& base_dh, DH arm_dh[4], bool is_left):    
    matrix_base_(base_dh.d, base_dh.a, base_dh.alpha, base_dh.offset)
{
    // base is the matrix from base to z0
    base_dh_.d = base_dh.d;
    base_dh_.a = base_dh.a;
    base_dh_.alpha = base_dh.alpha;
    base_dh_.offset = base_dh.offset;   
    
    for(int i = 0; i < 4; ++i)
    {
        arm_dh_[i].d = arm_dh[i].d;
        arm_dh_[i].a = arm_dh[i].a;
        arm_dh_[i].alpha = arm_dh[i].alpha;
        arm_dh_[i].offset = arm_dh[i].offset;
    }

    matrix_base_.inverse(matrix_base_inv_);
    if(is_left)
    {
        posture_.arm = -1;
    }
    else
    {
        posture_.arm = 1;
    }
    posture_.elbow = 0;
    posture_.wrist = 0;
    posture_.flip = 0;
    is_valid_ = true;
}

KinematicsToll::KinematicsToll(std::string file_path, bool is_left)
{
    file_path_ = file_path + "toll_dh.yaml";
    if (param_.loadParamFile(file_path_))
    {
        if (param_.getParam("base_dh/d", base_dh_.d) &&
            param_.getParam("base_dh/a", base_dh_.a) &&
            param_.getParam("base_dh/alpha", base_dh_.alpha) &&
            param_.getParam("base_dh/offset", base_dh_.offset) &&
            param_.getParam("toll_dh/axis-0/d", arm_dh_[0].d) &&
            param_.getParam("toll_dh/axis-0/a", arm_dh_[0].a) &&
            param_.getParam("toll_dh/axis-0/alpha", arm_dh_[0].alpha) &&
            param_.getParam("toll_dh/axis-0/offset", arm_dh_[0].offset) &&
            param_.getParam("toll_dh/axis-1/d", arm_dh_[1].d) &&
            param_.getParam("toll_dh/axis-1/a", arm_dh_[1].a) &&
            param_.getParam("toll_dh/axis-1/alpha", arm_dh_[1].alpha) &&
            param_.getParam("toll_dh/axis-1/offset", arm_dh_[1].offset) &&
            param_.getParam("toll_dh/axis-2/d", arm_dh_[2].d) &&
            param_.getParam("toll_dh/axis-2/a", arm_dh_[2].a) &&
            param_.getParam("toll_dh/axis-2/alpha", arm_dh_[2].alpha) &&
            param_.getParam("toll_dh/axis-2/offset", arm_dh_[2].offset) &&
            param_.getParam("toll_dh/axis-3/d", arm_dh_[3].d) &&
            param_.getParam("toll_dh/axis-3/a", arm_dh_[3].a) &&
            param_.getParam("toll_dh/axis-3/alpha", arm_dh_[3].alpha) &&
            param_.getParam("toll_dh/axis-3/offset", arm_dh_[3].offset))
        {
            TransMatrix matrix_base(base_dh_.d, base_dh_.a, base_dh_.alpha, base_dh_.offset);
            matrix_base_ = matrix_base;
            matrix_base_.inverse(matrix_base_inv_);
            if(is_left)
            {
                posture_.arm = -1;
            }
            else
            {
                posture_.arm = 1;
            }
            posture_.elbow = 0;
            posture_.wrist = 0;
            posture_.flip = 0;
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

KinematicsToll::~KinematicsToll()
{

}

bool KinematicsToll::isValid()
{
    return is_valid_;
}

bool KinematicsToll::getDH(DH& base_dh, DH arm_dh[6])
{
    base_dh.d = base_dh_.d;
    base_dh.a = base_dh_.a;
    base_dh.alpha = base_dh_.alpha;
    base_dh.offset = base_dh_.offset;

    for(size_t i = 0; i < 4; ++i)
    {
        arm_dh[i].d = arm_dh_[i].d;
        arm_dh[i].a = arm_dh_[i].a;
        arm_dh[i].alpha = arm_dh_[i].alpha;
        arm_dh[i].offset = arm_dh_[i].offset;
    }
    return true;
}

bool KinematicsToll::setDH(DH& base_dh, DH arm_dh[6])
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
        param_.setParam("arm_dh/axis-3/offset", arm_dh[3].offset))
    {
        if(!param_.dumpParamFile(file_path_))
        {
            return false;
        }        
        base_dh_.d = base_dh.d;
        base_dh_.a = base_dh.a;
        base_dh_.alpha = base_dh.alpha;
        base_dh_.offset = base_dh.offset;
        for(size_t i = 0; i < 4; ++i)
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

void KinematicsToll::doFK(const Joint& joint, PoseEuler& pose_euler, size_t from_joint_index, size_t to_joint_index)
{
    TransMatrix result_matrix;
    doFK(joint, result_matrix, from_joint_index, to_joint_index);
    result_matrix.convertToPoseEuler(pose_euler);
}

void KinematicsToll::doFK(const Joint& joint, PoseQuaternion& pose_quaternion, size_t from_joint_index, size_t to_joint_index)
{
    TransMatrix result_matrix;
    doFK(joint, result_matrix, from_joint_index, to_joint_index);
    result_matrix.convertToPoseQuaternion(pose_quaternion);
}

void KinematicsToll::doFK(const Joint& joint, TransMatrix& trans_matrix, size_t from_joint_index, size_t to_joint_index)
{
    from_joint_index = (from_joint_index >= 4 ? 4 : from_joint_index);
    to_joint_index = (to_joint_index >= 4 ? 4 : to_joint_index);

    TransMatrix result_matrix;
    if(from_joint_index == 0)
    {
        result_matrix = matrix_base_;
        //result_matrix.print("1 in KinematicsToll::doFK: first matrix=");
    }
    else if (from_joint_index == 1)
    {
        size_t first_dh_index = from_joint_index - 1;
        //the first joint is prismatic for toll scara.
        TransMatrix matrix_first(arm_dh_[first_dh_index].d + joint[0], arm_dh_[first_dh_index].a, arm_dh_[first_dh_index].alpha, 0 + arm_dh_[first_dh_index].offset);
        result_matrix = matrix_first; 
    }
    else
    {
        size_t first_dh_index = from_joint_index - 1;
        TransMatrix matrix_first(arm_dh_[first_dh_index].d, arm_dh_[first_dh_index].a, arm_dh_[first_dh_index].alpha, joint[first_dh_index] + arm_dh_[first_dh_index].offset);        
        result_matrix = matrix_first;       
    }
    //printf("from index=%d, to index=%d\n", from_joint_index, to_joint_index);
    for(size_t i = from_joint_index; i < to_joint_index; ++i)
    {
        if (i == 0)
        {
            //the first joint is prismatic for toll scara.
            TransMatrix matrix(arm_dh_[i].d + joint[0], arm_dh_[i].a, arm_dh_[i].alpha, 0 + arm_dh_[i].offset);
            result_matrix.rightMultiply(matrix);
            //result_matrix.print("2 in KinematicsToll::doFK: second matrix=");
            continue;
        }
        TransMatrix matrix(arm_dh_[i].d, arm_dh_[i].a, arm_dh_[i].alpha, joint[i] + arm_dh_[i].offset);
        result_matrix.rightMultiply(matrix);
        //result_matrix.print(" in KinematicsToll::doFK: sequence matrix=");
    }
    trans_matrix = result_matrix;
    //trans_matrix.print("3 in KinematicsToll::doFK: last matrix=");
}

//being called
bool KinematicsToll::doIK(const PoseEuler& pose_euler, const Posture& posture, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_euler.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, joint, valve);
}

bool KinematicsToll::doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_quaternion.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, joint, valve);
}

bool KinematicsToll::doIK(const TransMatrix& trans_matrix, const Posture& posture, Joint& joint, double valve)
{
    if(!isPostureValid(posture))
    {
        return false;
    }

    // compute q1
    TransMatrix trans_matrix_0to4;// matrix from z0 to z4
    TransMatrix* trans_matrix_ptr = const_cast<TransMatrix*>(&trans_matrix);
    trans_matrix_ptr->leftMultiply(matrix_base_inv_, trans_matrix_0to4);

    joint.j1_ = trans_matrix_0to4.trans_vector_.z_ - arm_dh_[1].d - arm_dh_[3].d - arm_dh_[0].offset;

    // compute q2+q3+q4
    TransMatrix trans_matrix_1to4;//matrix from z1 to z4
    TransMatrix matrix_0to1(joint.j1_, arm_dh_[0].a, arm_dh_[0].alpha, 0 + arm_dh_[0].offset);
    TransMatrix matrix_0to1_inv;
    matrix_0to1.inverse(matrix_0to1_inv);
    trans_matrix_ptr->leftMultiply(matrix_0to1_inv, trans_matrix_1to4);

    double sin_j234 = trans_matrix_1to4.rotation_matrix_.matrix_[1][0];
    double cos_j234 = trans_matrix_1to4.rotation_matrix_.matrix_[0][0];
    double j234 = atan2(sin_j234, cos_j234) - arm_dh_[1].offset - arm_dh_[2].offset - arm_dh_[3].offset;
    //printf("j234=%f\n",j234);

    //the 2nd version kinematics-20190222
/*    //compute x4,y4
    double x4 = trans_matrix_1to4.trans_vector_.x_;
    double y4 = trans_matrix_1to4.trans_vector_.y_;

    //compute x3,y3
    double beta = M_PI/2 - j234;
    double x3 = x4 - arm_dh_[3].a * sin(beta);
    double y3 = y4 - arm_dh_[3].a * cos(beta);

    //compute x2,y2
    double gamma = acos(((x3 * x3 + y3 * y3) + arm_dh_[2].a * arm_dh_[2].a - arm_dh_[1].a * arm_dh_[1].a)
                       /(2 * (sqrt(x3 * x3 + y3* y3)) * arm_dh_[2].a));
    double alpha = atan2(x3, y3);
    if (posture.elbow == 1)
    {
        beta = alpha - gamma;
    } else
    {
        beta = alpha + gamma;
    }
    double x2 = x3 - arm_dh_[2].a * sin(beta);
    double y2 = y3 - arm_dh_[2].a * cos(beta);

    printf("x2=%f,y2=%f,x3=%f, y3=%f, x4=%f,y4=%f\n",x2,y2,x3,y3,x4,y4);

    //compute q2
    Point v1, v2; 
    v1.x_ = 1; 
    v1.y_ = 0; 
    v1.z_ = 0;
    double norm = sqrt(x2 * x2 + y2 * y2);
    v2.x_ = (x2 - v1.x_) / norm;
    v2.y_ = y2 / norm;
    v2.z_ = 0;
    double cos_j2 = v1.dotProduct(v2);

    if(cos_j2 < -1 || cos_j2 > 1)
    {
        return false;
    }
    double sin_j2 = sqrt(1 - cos_j2 * cos_j2); 
    joint.j2_ = atan2(sin_j2, cos_j2);
    printf("cos_j2=%f, sin_j2=%f, joint.j2_=%f\n",cos_j2, sin_j2, joint.j2_);
    if (posture.arm == -1)
    {
        joint.j2_ = - joint.j2_;
    } 
    printf("joint.j2__=%f\n",joint.j2_);

    //compute q3
    Point v3; 
    norm = sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));
    v3.x_ = (x3 - x2) / norm; 
    v3.y_ = (y3 - y2) / norm; 
    v3.z_ = 0;
    double cos_j3 = v2.dotProduct(v3);
    if(cos_j3 < -1 || cos_j3 > 1)
    {
        return false;
    }
    double sin_j3 = sqrt(1 - cos_j3 * cos_j3);
    joint.j3_ = atan2(sin_j3, cos_j3);
    printf("cos_j3=%f, sin_j3=%f, joint.j3_=%f\n",cos_j3, sin_j3, joint.j3_);
    if (posture.elbow == -1)
    {
        joint.j3_ = - joint.j3_;
    }
    printf("joint.j3__=%f\n",joint.j3_);

    //compute q4
    Point v4; 
    norm = sqrt((x4 - x3) * (x4 - x3) + (y4 - y3) * (y4 - y3));
    v4.x_ = (x4 - x3) / norm; 
    v4.y_ = (y4 - y3) / norm; 
    v4.z_ = 0;
    double cos_j4 = v3.dotProduct(v4);
    if(cos_j4 < -1 || cos_j4 > 1)
    {
        return false;
    }
    double sin_j4 = sqrt(1 - cos_j4 * cos_j4);
    joint.j4_ = atan2(sin_j4, cos_j4);
    printf("cos_j4=%f, sin_j4=%f, joint.j4_=%f\n",cos_j4, sin_j4, joint.j4_);
    if (posture.wrist == -1)
    {
        joint.j4_ = - joint.j4_;
    }
    printf("joint.j4__=%f\n",joint.j4_);
    */

//  the 1st version kinematics-20190215

    // compute q3
    double x4 = trans_matrix_1to4.trans_vector_.x_;
    double y4 = trans_matrix_1to4.trans_vector_.y_;
    double r_square = (x4 - arm_dh_[3].a * cos_j234) * (x4 - arm_dh_[3].a * cos_j234) 
               + (y4 - arm_dh_[3].a * sin_j234) * (y4 - arm_dh_[3].a * sin_j234);
    double cos_j3 = (r_square - arm_dh_[1].a * arm_dh_[1].a - arm_dh_[2].a * arm_dh_[2].a)/(2 * arm_dh_[1].a * arm_dh_[2].a);
    if(cos_j3 <= -1 || cos_j3 > 1)
    {
        return false;
    }
    double sin_j3 = sqrt(1 - cos_j3 * cos_j3);
    joint.j3_ = atan2(sin_j3, cos_j3);
    scaleResultJoint(joint.j3_);

    if (posture.arm == 1)
    {
        joint.j3_ = fabs(joint.j3_);
    }
    else
    {
        joint.j3_ = - fabs(joint.j3_);
    }
    cos_j3 = cos(joint.j3_);
    sin_j3 = sin(joint.j3_);

    // compute q2
    double m1 = arm_dh_[2].a * sin_j3;
    double n1 = arm_dh_[1].a + arm_dh_[2].a * cos_j3;
    double x2 = m1 * (y4 - arm_dh_[3].a * sin_j234) + n1 * (x4 - arm_dh_[3].a * cos_j234);
    double y2 = - m1 * (x4 - arm_dh_[3].a * cos_j234) + n1 * (y4 - arm_dh_[3].a * sin_j234);

    joint.j2_ = atan2(y2, x2);
    scaleResultJoint(joint.j2_);

    // compute q1
    joint.j4_ = j234 - joint.j2_ - joint.j3_;
    scaleResultJoint(joint.j4_); 

    return true;
}


//being called.
bool KinematicsToll::doIK(const PoseEuler& pose_euler, const Joint& ref_joint, Joint& joint, double valve)
{
    Posture posture = getPostureByJoint(ref_joint);
    return doIK(pose_euler, posture, joint, valve);
}

bool KinematicsToll::doIK(const PoseQuaternion& pose_quaternion, const Joint& ref_joint, Joint& joint, double valve)
{
    Posture posture = getPostureByJoint(ref_joint);
    return doIK(pose_quaternion, posture, joint, valve);
}

bool KinematicsToll::doIK(const TransMatrix& trans_matrix, const Joint& ref_joint, Joint& joint, double valve)
{
    Posture posture = getPostureByJoint(ref_joint);
    return doIK(trans_matrix, posture, joint, valve);
}

bool KinematicsToll::doIK(const PoseEuler& pose_euler, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_euler.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, ref_joint, joint, valve);
}

bool KinematicsToll::doIK(const PoseQuaternion& pose_quaternion, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve)
{
    TransMatrix trans_matrix;
    pose_quaternion.convertToTransMatrix(trans_matrix);
    return doIK(trans_matrix, posture, ref_joint, joint, valve);
}

bool KinematicsToll::doIK(const TransMatrix& trans_matrix, const Posture& posture, const Joint& ref_joint, Joint& joint, double valve)
{
    return doIK(trans_matrix, posture, joint, valve);
}

Posture KinematicsToll::getPostureByJoint(const Joint& joint, double valve)
{
    Posture posture = posture_;

    if (joint.j3_ >= 0)
    {
        posture.arm = 1;
    }
    else 
    {
        posture.arm = -1;
    }
 
 /* 2nd version
    if (joint.j2_ >= 0)
    {
        posture.arm = 1;
    }
    else 
    {
        posture.arm = -1;
    }

    if (joint.j3_ >= 0)
    {
        posture.elbow = 1;
    }
    else 
    {
        posture.elbow = -1;
    }

    if (joint.j4_ >= 0)
    {
        posture.wrist = 1;
    }
    else 
    {
        posture.wrist = -1;
    }
*/
    return posture;
}

inline void KinematicsToll::scaleResultJoint(double& angle)
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

inline bool KinematicsToll::isPostureValid(const Posture& posture)
{
    //2nd version
    //if((posture.arm != 1 && posture.arm != -1)
    //    || (posture.elbow != 1 && posture.elbow != -1)
    //    || (posture.wrist != 1 && posture.wrist != -1))
    if (posture.arm != 1 && posture.arm != -1)
    {
        return false;
    }
    else
    {
        return true;
    }
}


