#include "pose_quaternion.h"
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;

void PoseQuaternion::zero()
{
    trans_vector_.zero();
    quaternion_.zero();
}

bool PoseQuaternion::isEqual(PoseQuaternion& pose_quaternion, double valve) const
{
    if(trans_vector_.isEqual(pose_quaternion.trans_vector_, valve)
        && quaternion_.isEqual(pose_quaternion.quaternion_, valve))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void PoseQuaternion::convertToPoseEuler(PoseEuler& pose_euler) const
{
    pose_euler.trans_vector_ = trans_vector_;
    quaternion_.convertToEuler(pose_euler.euler_);
}

void PoseQuaternion::convertToTransMatrix(TransMatrix& matrix) const
{
    matrix.trans_vector_ = trans_vector_;
    quaternion_.convertToRotationMatrix(matrix.rotation_matrix_);
}

void PoseQuaternion::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    trans_vector_.print("trans vector:");
    quaternion_.print("quaternion:");
}



