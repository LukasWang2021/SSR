#ifndef POSE_EULER_H
#define POSE_EULER_H

#include "point.h"
#include "euler.h"
#include "pose_quaternion.h"
#include "trans_matrix.h"

namespace basic_alg
{
class PoseQuaternion;
class TransMatrix;

class PoseEuler
{
public:
    Point point_;
    Euler euler_;

    PoseEuler();
    ~PoseEuler();

    bool isEqual(PoseEuler& pose_euler, double valve = 0.001) const;
    void convertToPoseQuaternion(basic_alg::PoseQuaternion& pose_quaternion) const;
    void convertToTransMatrix(basic_alg::TransMatrix& matrix) const;  

    void print(std::string comment = "") const;
};

}


#endif

