#ifndef POSE_QUATERNION_H
#define POSE_QUATERNION_H

#include "point.h"
#include "quaternion.h"
#include "pose_euler.h"
#include "trans_matrix.h"
#include <string>

namespace basic_alg
{
class PoseEuler;
class TransMatrix;

class PoseQuaternion
{
public:
    Point point_;
    Quaternion quaternion_;

    bool isEqual(const PoseQuaternion& pose_quaternion, double valve = 0.001) const;
    void convertToPoseEuler(basic_alg::PoseEuler& pose_euler) const;
    void convertToTransMatrix(basic_alg::TransMatrix& matrix) const;  

    void print(std::string comment = "") const;
};

}


#endif

