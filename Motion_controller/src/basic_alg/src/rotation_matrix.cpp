#include "rotation_matrix.h"
#include <assert.h>
#include <math.h>
#include <cstring>


using namespace std;
using namespace basic_alg;


RotationMatrix::RotationMatrix()
{

}

RotationMatrix::~RotationMatrix()
{

}

void RotationMatrix::initByStandardDh(double alpha, double theta)
{
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    double sin_alpha = sin(alpha);
    double cos_alpha = cos(alpha);

    matrix_[0][0] = cos_theta;
    matrix_[0][1] = -sin_theta*cos_alpha;
    matrix_[0][2] = sin_theta*sin_alpha;
    matrix_[1][0] = sin_theta;
    matrix_[1][1] = cos_theta* cos_alpha;
    matrix_[1][2] = -cos_theta*sin_alpha;
    matrix_[2][0] = 0;
    matrix_[2][1] = sin_alpha;
    matrix_[2][2] = cos_alpha;
}

void RotationMatrix::convertToQuaternion(basic_alg::Quaternion& quaternion) const
{
    quaternion.x_ = sqrt(fabs(matrix_[0][0] - matrix_[1][1] - matrix_[2][2] + 1)) / 2;
    quaternion.y_ = sqrt(fabs(matrix_[1][1] - matrix_[0][0] - matrix_[2][2] + 1)) / 2;
    int max_id;
    double tmp;
    if(quaternion.y_ > quaternion.x_)
    {
        tmp = quaternion.y_;
        max_id = 1;
    }
    else
    {
        tmp = quaternion.x_;
        max_id = 0;
    }
    quaternion.z_ = sqrt(fabs(matrix_[2][2] - matrix_[0][0] - matrix_[1][1] + 1)) / 2;
    if(quaternion.z_ > tmp)
    {
        tmp = quaternion.z_;
        max_id = 2;
    }
    quaternion.w_ = sqrt(fabs(matrix_[0][0] + matrix_[1][1] + matrix_[2][2] + 1)) / 2;
    if(quaternion.w_ > tmp)
    {
        max_id = 3;
    }

    switch(max_id)
    {
        case 0:
            if(matrix_[1][0] + matrix_[0][1] < 0) quaternion.y_ = -quaternion.y_;
            if(matrix_[2][0] + matrix_[0][2] < 0) quaternion.z_ = -quaternion.z_;
            if(matrix_[2][1] - matrix_[1][2] < 0) quaternion.w_ = -quaternion.w_;
            return;
        case 1:
            if(matrix_[1][0] + matrix_[0][1] < 0) quaternion.x_ = -quaternion.x_;
            if(matrix_[2][1] + matrix_[1][2] < 0) quaternion.z_ = -quaternion.z_;
            if(matrix_[0][2] - matrix_[2][0] < 0) quaternion.w_ = -quaternion.w_;
            return;            
        case 2:
            if(matrix_[2][0] + matrix_[0][2] < 0) quaternion.x_ = -quaternion.x_;
            if(matrix_[2][1] + matrix_[1][2] < 0) quaternion.y_ = -quaternion.y_;
            if(matrix_[1][0] - matrix_[0][1] < 0) quaternion.w_ = -quaternion.w_;
            return;
        case 3:
            if(matrix_[2][1] - matrix_[1][2] < 0) quaternion.x_ = -quaternion.x_;
            if(matrix_[0][2] - matrix_[2][0] < 0) quaternion.y_ = -quaternion.y_;
            if(matrix_[1][0] - matrix_[0][1] < 0) quaternion.z_ = -quaternion.z_;
            return;            
    }
}

void RotationMatrix::convertToEuler(basic_alg::Euler& euler) const
{
    double tmp = matrix_[2][1] * matrix_[2][1] + matrix_[2][2] * matrix_[2][2];
    euler.b_ = atan2(-matrix_[2][0], sqrt(tmp));
    if(tmp > 0.001)
    {
        euler.a_ = atan2(matrix_[1][0], matrix_[0][0]);
        euler.c_ = atan2(matrix_[2][1], matrix_[2][2]);
    }
    else
    {
        euler.a_ = atan2(-matrix_[0][1], matrix_[1][1]);
        euler.c_ = 0;
    }
}

void RotationMatrix::multiplyByTransVector(Point& trans_vector, Point& result_vector) const
{
    if(&trans_vector != &result_vector)
    {
        result_vector.x_ = matrix_[0][0] * trans_vector.x_ + matrix_[0][1] * trans_vector.y_ + matrix_[0][2] * trans_vector.z_;
        result_vector.y_ = matrix_[1][0] * trans_vector.x_ + matrix_[1][1] * trans_vector.y_ + matrix_[1][2] * trans_vector.z_;
        result_vector.z_ = matrix_[2][0] * trans_vector.x_ + matrix_[2][1] * trans_vector.y_ + matrix_[2][2] * trans_vector.z_;
    }
    else
    {
        Point tmp_vector;
        tmp_vector.x_ = matrix_[0][0] * trans_vector.x_ + matrix_[0][1] * trans_vector.y_ + matrix_[0][2] * trans_vector.z_;
        tmp_vector.y_ = matrix_[1][0] * trans_vector.x_ + matrix_[1][1] * trans_vector.y_ + matrix_[1][2] * trans_vector.z_;
        tmp_vector.z_ = matrix_[2][0] * trans_vector.x_ + matrix_[2][1] * trans_vector.y_ + matrix_[2][2] * trans_vector.z_;
        result_vector = tmp_vector;
    }
}

void RotationMatrix::getVectorN(Point& n_vector) const
{
    n_vector.x_ = matrix_[0][0];
    n_vector.y_ = matrix_[1][0];
    n_vector.z_ = matrix_[2][0];
}

void RotationMatrix::getVectorS(Point& s_vector) const
{
    s_vector.x_ = matrix_[0][1];
    s_vector.y_ = matrix_[1][1];
    s_vector.z_ = matrix_[2][1];
}

void RotationMatrix::getVectorA(Point& a_vector) const
{
    a_vector.x_ = matrix_[0][2];
    a_vector.y_ = matrix_[1][2];
    a_vector.z_ = matrix_[2][2];
}


