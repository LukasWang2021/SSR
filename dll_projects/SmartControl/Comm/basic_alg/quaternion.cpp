#include "quaternion.h"
#include <assert.h>
#include <math.h>
#include <iostream>
#include "basic_constants.h"

using namespace std;
using namespace basic_alg;

void Quaternion::zero()
{
    x_ = 0;
    y_ = 0;
    z_ = 0;
    w_ = 1;
}

bool Quaternion::isValid(double valve) const
{
    if(fabs(x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_ - 1.0) < valve)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Quaternion::isEqual(Quaternion& quaternion, double valve) const
{
    if(fabs(x_ - quaternion.x_) < valve
        && fabs(y_ - quaternion.y_) < valve
        && fabs(z_ - quaternion.z_) < valve
        && fabs(w_ - quaternion.w_) < valve)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Quaternion::reverse()
{
    x_ = -x_;
    y_ = -y_;
    z_ = -z_;
    w_ = -w_;
}

void Quaternion::convertToEuler(Euler& euler) const
{
    RotationMatrix matrix;
    convertToRotationMatrix(matrix);
    matrix.convertToEuler(euler);
}

void Quaternion::convertToRotationMatrix(RotationMatrix& matrix) const
{
    double xy = 2 * x_ * y_;
    double yz = 2 * y_ * z_;
    double zx = 2 * z_ * x_;
    double wx = 2 * w_ * x_;
    double wy = 2 * w_ * y_;
    double wz = 2 * w_ * z_;
    double ww = w_ * w_;

    matrix.matrix_[0][0] = 2 * (ww + x_ * x_) - 1; // r[0][0] = 2*(w^2+x^2)-1
    matrix.matrix_[0][1] = xy - wz;  // r[0][1] = 2*(xy-wz)
    matrix.matrix_[0][2] = wy + zx;  // r[0][2] = 2*(wy+zx)
    matrix.matrix_[1][0] = xy + wz;  // r[1][0] = 2*(xy+wz)-1
    matrix.matrix_[1][1] = 2 * (ww + y_ * y_) - 1; // r[1][1] = 2*(w^2+y^2)-1
    matrix.matrix_[1][2] = yz - wx;  // r[1][2] = 2*(yz-wx)
    matrix.matrix_[2][0] = zx - wy;  // r[2][0] = 2*(zx-wy)
    matrix.matrix_[2][1] = wx + yz;  // r[2][1] = 2*(wx+yz)
    matrix.matrix_[2][2] = 2 * (ww + z_ * z_) - 1; // r[2][2] = 2*(w^2+z^2)-1
}

double Quaternion::getIncludedAngle(Quaternion& quaternion) const
{
    double dot_product = x_ * quaternion.x_ + y_ * quaternion.y_ + z_ * quaternion.z_ + w_ * quaternion.w_;
    if(dot_product >= 1)
    {
        return 0;
    }
    else if(dot_product <= -1)
    {
        return M_PI;
    }
    else
    {
        return acos(dot_product);
    }
}

Quaternion Quaternion::getQuaternionBetween(Quaternion& end_quaternion, double angle_to_start)
{
    Quaternion result;
    double include_angle = getIncludedAngle(end_quaternion);
    if(angle_to_start < 0 
        || angle_to_start > include_angle 
        || include_angle == 0)
    {
        result.zero();
        return result;
    }
    double angle_percent = angle_to_start / include_angle;
    double k1, k2;    
    if(include_angle > (5 * M_PI / 180L))    // > 5 degree, apply slerp method
    {
        double tmp = sin(include_angle);
        k1 = sin((1 - angle_percent) * include_angle) / tmp;
        k2 = sin(angle_percent * include_angle) / tmp;        
    }
    else    // <= 5 degree, apply linear method
    {
        k1 = 1 - angle_percent;
        k2 = angle_percent;
    }
    result.x_ = k1 * x_ + k2 * end_quaternion.x_;
    result.y_ = k1 * y_ + k2 * end_quaternion.y_;
    result.z_ = k1 * z_ + k2 * end_quaternion.z_;
    result.w_ = k1 * w_ + k2 * end_quaternion.w_;
    return result;
}

double& Quaternion::operator[](size_t index) 
{
    assert(index < 4); 
    return *(&x_ + index);
}

const double& Quaternion::operator[](size_t index) const 
{
    assert(index < 4); 
    return *(&x_ + index);
}

void Quaternion::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    std::cout<<" x = "<<x_
             <<" y = "<<y_
             <<" z = "<<z_
             <<" w = "<<w_<<std::endl;
}


