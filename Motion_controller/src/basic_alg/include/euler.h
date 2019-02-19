#ifndef EULER_H
#define EULER_H


#include <stddef.h>
#include "rotation_matrix.h"
#include "quaternion.h"
#include <string>

namespace basic_alg
{
class Quaternion;
class RotationMatrix;

class Euler
{
public:
    double a_;
    double b_;
    double c_;

    Euler();
    ~Euler();

    bool isEqual(Euler& euler, double valve = 0.001) const;
    void convertToQuaternion(basic_alg::Quaternion& quaternion) const;
    void convertToRotationMatrix(basic_alg::RotationMatrix& matrix) const;
    
    double& operator[](size_t index);
    const double& operator[](size_t index) const;

    void print(std::string comment = "") const;
};


}

#endif

