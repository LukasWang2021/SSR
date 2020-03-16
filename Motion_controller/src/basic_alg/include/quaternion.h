#ifndef QUATERNION_H
#define QUATERNION_H

#include <stddef.h>
#include "euler.h"
#include "rotation_matrix.h"
#include <string>

namespace basic_alg
{
class Euler;
class RotationMatrix;

class Quaternion
{
public:
    double x_;
    double y_;
    double z_;
    double w_;

    bool isValid(double valve = 0.001) const;
    bool isEqual(const Quaternion& quaternion, double valve = 0.001) const;
    void reverse();
    void convertToEuler(basic_alg::Euler& euler) const;
    void convertToRotationMatrix(basic_alg::RotationMatrix& matrix) const;
    double getIncludedAngle(const Quaternion& quaternion) const;
    
    double& operator[](size_t index);
    const double& operator[](size_t index) const;

    void print(std::string comment = "") const;
};

}



#endif
