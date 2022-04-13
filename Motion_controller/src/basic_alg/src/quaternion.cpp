#include "quaternion.h"
#include <assert.h>
#include <math.h>
#include <iostream>


using namespace std;
using namespace basic_alg;

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

bool Quaternion::isEqual(const Quaternion& quaternion, double valve) const
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

bool Quaternion::isEquivalent(const Euler& euler, double valve) const
{
    return fabs(getIncludedAngle(euler)) < valve;
}

bool Quaternion::isEquivalent(const Quaternion& quaternion, double valve) const
{
    return fabs(getIncludedAngle(quaternion)) < valve;
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
#if 0
    r = [   1-2*(y^2+z^2)   2*(x*y-w*z) 2*(x*z+w*y)
        2*(x*y+w*z) 1-2*(x^2+z^2)   2*(y*z-w*x)
        2*(x*z-w*y) 2*(y*z+w*x) 1-2*(x^2+y^2)   ];
#endif

    double xy = 2 * x_ * y_;
    double yz = 2 * y_ * z_;
    double zx = 2 * z_ * x_;
    double wx = 2 * w_ * x_;
    double wy = 2 * w_ * y_;
    double wz = 2 * w_ * z_;
    double yy = y_ * y_;
    double zz = z_ * z_;
    double xx = x_ * x_;

    matrix.matrix_[0][0] = 1 - 2 * (yy + zz);
    matrix.matrix_[0][1] = xy - wz;  // r[0][1] = 2*(xy-wz)
    matrix.matrix_[0][2] = wy + zx;  // r[0][2] = 2*(wy+zx)
    matrix.matrix_[1][0] = xy + wz;  // r[1][0] = 2*(xy+wz)-1
    matrix.matrix_[1][1] = 1 - 2 * (xx + zz); // r[1][1] = 2*(w^2+y^2)-1
    matrix.matrix_[1][2] = yz - wx;  // r[1][2] = 2*(yz-wx)
    matrix.matrix_[2][0] = zx - wy;  // r[2][0] = 2*(zx-wy)
    matrix.matrix_[2][1] = wx + yz;  // r[2][1] = 2*(wx+yz)
    matrix.matrix_[2][2] = 1 - 2 * (xx + yy); // r[2][2] = 2*(w^2+z^2)-1
}

double Quaternion::getIncludedAngle(const Euler& euler) const
{
    Quaternion quaternion;
    euler.convertToQuaternion(quaternion);
    return getIncludedAngle(quaternion);
}

double Quaternion::getIncludedAngle(const Quaternion& quaternion) const
{
    double dot_product = x_ * quaternion.x_ + y_ * quaternion.y_ + z_ * quaternion.z_ + w_ * quaternion.w_;

    if(dot_product < 0)
    {
        dot_product = (-x_) * quaternion.x_ + (-y_) * quaternion.y_ + (-z_) * quaternion.z_ + (-w_) * quaternion.w_;
    }

    if(dot_product > 1)
    {
        dot_product = 1;
    }

    return acos(dot_product);
}
double Quaternion::norm()
{
    //cout << "w="<<this->w_<<" x="<<this->x_<<" y="<<this->y_<<" z="<<this->z_<<endl;

    return sqrt(this->w_*this->w_ + this->x_*this->x_ + this->y_*this->y_ + this->z_*this->z_);
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
const Quaternion Quaternion::operator*(double value)
{
    Quaternion q_res;
    q_res.w_ = value * w_;
    q_res.x_ = value * x_;
    q_res.y_ = value * y_;
    q_res.z_ = value * z_;
    return q_res;
}
const Quaternion Quaternion::operator/(double value)
{
    Quaternion q_res;
    if(value != 0)
    {
        q_res.w_ =w_/ value;
        q_res.x_ =x_/ value;
        q_res.y_ =y_/ value;
        q_res.z_ =z_/ value;
    }
    else
    {
        q_res.w_ = 0;
        q_res.x_ = 0;
        q_res.y_ = 0;
        q_res.z_ = 0;
    }
    return q_res;
}
const Quaternion Quaternion::operator+(const Quaternion& q)
{
    Quaternion result;
    result.w_ = w_ + q.w_;
    result.x_ = x_ + q.x_;
    result.y_ = y_ + q.y_;
    result.z_ = z_ + q.z_;
    return result;
}
const Quaternion Quaternion::operator-(const Quaternion& q)
{
    Quaternion result;
    result.w_ = w_ - q.w_;
    result.x_ = x_ - q.x_;
    result.y_ = y_ - q.y_;
    result.z_ = z_ - q.z_;
    return result;
}

void Quaternion::print(std::string comment) const
{
    std::cout<<comment;//<<std::endl;
    std::cout<<" w="<<w_
             <<" x="<<x_
             <<" y="<<y_
             <<" z="<<z_<<std::endl;
}


