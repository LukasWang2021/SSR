#include "euler.h"
#include <assert.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;


bool Euler::isEqual(const Euler& euler, double valve) const
{
    if(fabs(a_ - euler.a_) < valve
        && fabs(b_ - euler.b_) < valve
        && fabs(c_ - euler.c_) < valve)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Euler::convertToQuaternion(Quaternion& quaternion) const
{
    RotationMatrix matrix;
    convertToRotationMatrix(matrix);
    matrix.convertToQuaternion(quaternion);
}

void Euler::convertToRotationMatrix(RotationMatrix& matrix) const
{
    double sin_z = sin(a_);
    double cos_z = cos(a_);
    double sin_y = sin(b_);
    double cos_y = cos(b_);
    double sin_x = sin(c_);
    double cos_x = cos(c_);

    matrix.matrix_[0][0] = cos_z*cos_y;
    matrix.matrix_[0][1] = cos_z*sin_y*sin_x-cos_x*sin_z;
    matrix.matrix_[0][2] = cos_z*cos_x*sin_y+sin_z*sin_x;
    matrix.matrix_[1][0] = cos_y*sin_z;
    matrix.matrix_[1][1] = cos_z*cos_x+sin_z*sin_y*sin_x;
    matrix.matrix_[1][2] = cos_x*sin_z*sin_y-cos_z*sin_x;
    matrix.matrix_[2][0] = -sin_y;
    matrix.matrix_[2][1] = cos_y*sin_x;
    matrix.matrix_[2][2] = cos_y*cos_x;    

}

double& Euler::operator[](size_t index) 
{
    assert(index < 3); 
    return *(&a_ + index);
}

const double& Euler::operator[](size_t index) const 
{
    assert(index < 3); 
    return *(&a_ + index);
}

void Euler::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    std::cout<<" a = "<<a_
             <<" b = "<<b_
             <<" c = "<<c_<<std::endl;
}

