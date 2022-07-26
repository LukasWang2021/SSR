#include "pose_quaternion.h"
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;

bool PoseQuaternion::isEqual(const PoseQuaternion& pose_quaternion, double valve) const
{
    if(point_.isEqual(pose_quaternion.point_, valve)
        && quaternion_.isEqual(pose_quaternion.quaternion_, valve))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool PoseQuaternion::isEquivalent(const PoseQuaternion& pose_quaternion, double valve) const
{
    return point_.isEqual(pose_quaternion.point_, valve) && quaternion_.isEquivalent(pose_quaternion.quaternion_, valve);
}

void PoseQuaternion::convertToPoseEuler(PoseEuler& pose_euler) const
{
    pose_euler.point_ = point_;
    quaternion_.convertToEuler(pose_euler.euler_);
}

void PoseQuaternion::convertToTransMatrix(TransMatrix& matrix) const
{
    matrix.trans_vector_ = point_;
    quaternion_.convertToRotationMatrix(matrix.rotation_matrix_);
}

void PoseQuaternion::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    point_.print("point:");
    quaternion_.print("quaternion:");
}



