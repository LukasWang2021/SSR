#include "pose_euler.h"
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;

void PoseEuler::zero()
{
    trans_vector_.zero();
    euler_.zero();
}

bool PoseEuler::isEqual(PoseEuler& pose_euler, double valve) const
{
    if(trans_vector_.isEqual(pose_euler.trans_vector_, valve)
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
    pose_quaternion.trans_vector_ = trans_vector_;
    euler_.convertToQuaternion(pose_quaternion.quaternion_);
}

void PoseEuler::convertToTransMatrix(basic_alg::TransMatrix& matrix) const 
{
    matrix.trans_vector_ = trans_vector_;
    euler_.convertToRotationMatrix(matrix.rotation_matrix_);
}

void PoseEuler::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    trans_vector_.print("trans vector:");
    euler_.print("euler:");
}


