#include "trans_matrix.h"
#include <math.h>
#include <cstring>
#include <iostream>


using namespace std;
using namespace basic_alg;


TransMatrix::TransMatrix()
{

}

TransMatrix::TransMatrix(double d, double a, double alpha, double theta)
{
    rotation_matrix_.initByStandardDh(alpha, theta);
    trans_vector_.x_ = a * cos(theta);
    trans_vector_.y_ = a * sin(theta);
    trans_vector_.z_ = d;
}

TransMatrix::~TransMatrix()
{

}

bool TransMatrix::isEqual(const TransMatrix& matrix, double valve) const
{
    if(trans_vector_.isEqual(matrix.trans_vector_, valve)
        && rotation_matrix_.isEqual(matrix.rotation_matrix_, valve))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void TransMatrix::convertToPoseEuler(PoseEuler& pose_euler) const
{
    pose_euler.point_ = trans_vector_;
    rotation_matrix_.convertToEuler(pose_euler.euler_);
}

void TransMatrix::convertToPoseQuaternion(PoseQuaternion& pose_quaternion) const
{
    pose_quaternion.point_ = trans_vector_;
    rotation_matrix_.convertToQuaternion(pose_quaternion.quaternion_);
}

TransMatrix& TransMatrix::leftMultiply(const TransMatrix& left_matrix)
{
    TransMatrix tmp_matrix;
    multiply(left_matrix, *this, tmp_matrix);
    *this = tmp_matrix;
    return *this;
}

void TransMatrix::leftMultiply(const TransMatrix& left_matrix, TransMatrix& result_matrix)
{
    if(&result_matrix != this)
    {
        multiply(left_matrix, *this, result_matrix);
    }
    else
    {
        TransMatrix tmp_matrix;
        multiply(left_matrix, *this, tmp_matrix);
        *this = tmp_matrix;
    }
}

TransMatrix& TransMatrix::rightMultiply(const TransMatrix& right_matrix)
{
    TransMatrix tmp_matrix;
    multiply(*this, right_matrix, tmp_matrix);
    *this = tmp_matrix;
    return *this;
}

void TransMatrix::rightMultiply(const TransMatrix& right_matrix, TransMatrix& result_matrix)
{
    if(&result_matrix != this)
    {
        multiply(*this, right_matrix, result_matrix);
    }
    else
    {
        TransMatrix tmp_matrix;
        multiply(*this, right_matrix, tmp_matrix);
        *this = tmp_matrix;
    }
}

bool TransMatrix::inverse(double valve)
{
    return inverse(*this, valve);
}

bool TransMatrix::inverse(TransMatrix& result_matrix, double valve)
{
    double det = rotation_matrix_.matrix_[0][0] * (rotation_matrix_.matrix_[1][1] * rotation_matrix_.matrix_[2][2] - rotation_matrix_.matrix_[1][2] * rotation_matrix_.matrix_[2][1])
            - rotation_matrix_.matrix_[0][1] * (rotation_matrix_.matrix_[1][0] * rotation_matrix_.matrix_[2][2] - rotation_matrix_.matrix_[1][2] * rotation_matrix_.matrix_[2][0])
            + rotation_matrix_.matrix_[0][2] * (rotation_matrix_.matrix_[1][0] * rotation_matrix_.matrix_[2][1] - rotation_matrix_.matrix_[1][1] * rotation_matrix_.matrix_[2][0]);
    if(det < valve)
    {
        return false;
    }

    double a00 = rotation_matrix_.matrix_[1][1] * rotation_matrix_.matrix_[2][2] - rotation_matrix_.matrix_[1][2] * rotation_matrix_.matrix_[2][1];
    double a01 = rotation_matrix_.matrix_[1][0] * rotation_matrix_.matrix_[2][2] - rotation_matrix_.matrix_[1][2] * rotation_matrix_.matrix_[2][0];
    double a02 = rotation_matrix_.matrix_[1][0] * rotation_matrix_.matrix_[2][1] - rotation_matrix_.matrix_[1][1] * rotation_matrix_.matrix_[2][0];

    double a10 = rotation_matrix_.matrix_[0][1] * rotation_matrix_.matrix_[2][2] - rotation_matrix_.matrix_[0][2] * rotation_matrix_.matrix_[2][1];
    double a11 = rotation_matrix_.matrix_[0][0] * rotation_matrix_.matrix_[2][2] - rotation_matrix_.matrix_[0][2] * rotation_matrix_.matrix_[2][0];
    double a12 = rotation_matrix_.matrix_[0][0] * rotation_matrix_.matrix_[2][1] - rotation_matrix_.matrix_[0][1] * rotation_matrix_.matrix_[2][0];
    
    double a20 = rotation_matrix_.matrix_[0][1] * rotation_matrix_.matrix_[1][2] - rotation_matrix_.matrix_[0][2] * rotation_matrix_.matrix_[1][1];
    double a21 = rotation_matrix_.matrix_[0][0] * rotation_matrix_.matrix_[1][2] - rotation_matrix_.matrix_[0][2] * rotation_matrix_.matrix_[1][0];
    double a22 = rotation_matrix_.matrix_[0][0] * rotation_matrix_.matrix_[1][1] - rotation_matrix_.matrix_[0][1] * rotation_matrix_.matrix_[1][0];

    double a30 = rotation_matrix_.matrix_[0][1] * (rotation_matrix_.matrix_[1][2] * trans_vector_.z_ - rotation_matrix_.matrix_[2][2] * trans_vector_.y_) 
                - rotation_matrix_.matrix_[0][2] * (rotation_matrix_.matrix_[1][1] * trans_vector_.z_ - rotation_matrix_.matrix_[2][1] * trans_vector_.y_) 
                + trans_vector_.x_ * (rotation_matrix_.matrix_[1][1] * rotation_matrix_.matrix_[2][2] - rotation_matrix_.matrix_[2][1] * rotation_matrix_.matrix_[1][2]);
    double a31 = rotation_matrix_.matrix_[0][0] * (rotation_matrix_.matrix_[1][2] * trans_vector_.z_ - rotation_matrix_.matrix_[2][2] * trans_vector_.y_) 
                - rotation_matrix_.matrix_[0][2] * (rotation_matrix_.matrix_[1][0] * trans_vector_.z_ - rotation_matrix_.matrix_[2][0] * trans_vector_.y_) 
                + trans_vector_.x_ * (rotation_matrix_.matrix_[1][0] * rotation_matrix_.matrix_[2][2] - rotation_matrix_.matrix_[2][0] * rotation_matrix_.matrix_[1][2]);
    double a32 = rotation_matrix_.matrix_[0][0] * (rotation_matrix_.matrix_[1][1] * trans_vector_.z_ - rotation_matrix_.matrix_[2][1] * trans_vector_.y_) 
                - rotation_matrix_.matrix_[0][1] * (rotation_matrix_.matrix_[1][0] * trans_vector_.z_ - rotation_matrix_.matrix_[2][0] * trans_vector_.y_) 
                + trans_vector_.x_ * (rotation_matrix_.matrix_[1][0] * rotation_matrix_.matrix_[2][1] - rotation_matrix_.matrix_[2][0] * rotation_matrix_.matrix_[1][1]);    

    result_matrix.rotation_matrix_.matrix_[0][0] =  a00 / det; 
    result_matrix.rotation_matrix_.matrix_[0][1] = -a10 / det; 
    result_matrix.rotation_matrix_.matrix_[0][2] =  a20 / det;     
    result_matrix.rotation_matrix_.matrix_[1][0] = -a01 / det; 
    result_matrix.rotation_matrix_.matrix_[1][1] =  a11 / det; 
    result_matrix.rotation_matrix_.matrix_[1][2] = -a21 / det;     
    result_matrix.rotation_matrix_.matrix_[2][0] =  a02 / det; 
    result_matrix.rotation_matrix_.matrix_[2][1] = -a12 / det; 
    result_matrix.rotation_matrix_.matrix_[2][2] =  a22 / det;
    result_matrix.trans_vector_.x_ = -a30 / det;
    result_matrix.trans_vector_.y_ =  a31 / det;
    result_matrix.trans_vector_.z_ = -a32 / det;    

    return true;
}

void TransMatrix::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    rotation_matrix_.print("rotation matrix:");
    trans_vector_.print("trans vector:");
}

void TransMatrix::multiply(const TransMatrix& left_matrix, const TransMatrix& right_matrix, TransMatrix& result_matrix)
{
    Point result_trans_vector;
    left_matrix.rotation_matrix_.rightMultiply(right_matrix.rotation_matrix_, result_matrix.rotation_matrix_);
    left_matrix.rotation_matrix_.multiplyByTransVector(right_matrix.trans_vector_, result_trans_vector);
    result_matrix.trans_vector_ = result_trans_vector + left_matrix.trans_vector_;
}


