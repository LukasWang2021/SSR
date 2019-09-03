#include "pose_euler.h"
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;

bool PoseEuler::isEqual(const PoseEuler& pose_euler, double valve) const
{
    if(point_.isEqual(pose_euler.point_, valve)
        && euler_.isEqual(pose_euler.euler_, valve))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void PoseEuler::convertToPoseQuaternion(basic_alg::PoseQuaternion& pose_quaternion) const
{
    pose_quaternion.point_ = point_;
    euler_.convertToQuaternion(pose_quaternion.quaternion_);
}

void PoseEuler::convertToTransMatrix(basic_alg::TransMatrix& matrix) const 
{
    matrix.trans_vector_ = point_;
    euler_.convertToRotationMatrix(matrix.rotation_matrix_);
}

void PoseEuler::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    point_.print("point:");
    euler_.print("euler:");
}


