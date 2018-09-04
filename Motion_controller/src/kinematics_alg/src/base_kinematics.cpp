/*************************************************************************
	> File Name: base_kinematics.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年02月01日 星期四 09时25分48秒
 ************************************************************************/

#include <string.h>
#include <base_kinematics.h>

using namespace basic_alg;

namespace fst_mc
{

BaseKinematics::BaseKinematics()
{
    memset(dh_matrix_, 0, sizeof(dh_matrix_));
    user_frame_.eye();
    user_frame_.eye();
    world_frame_.eye();
    inverse_user_frame_.eye();
    inverse_tool_frame_.eye();
    inverse_world_frame_.eye();
}

BaseKinematics::~BaseKinematics()
{}

void BaseKinematics::initKinematics(double (&dh_matrix)[NUM_OF_JOINT][4])
{
    memcpy(dh_matrix_, dh_matrix, NUM_OF_JOINT * 4 * sizeof(double));
}

void BaseKinematics::forwardKinematicsInBase(const Joint &joint, Pose &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.rightMultiply(tool_frame_).toPose(pose);
}

void BaseKinematics::forwardKinematicsInBase(const Joint &joint, PoseEuler &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.rightMultiply(tool_frame_).toPoseEuler(pose);
}

void BaseKinematics::forwardKinematicsInUser(const Joint &joint, Pose &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.leftMultiply(inverse_user_frame_).rightMultiply(tool_frame_).toPose(pose);
}

void BaseKinematics::forwardKinematicsInUser(const Joint &joint, PoseEuler &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.leftMultiply(inverse_user_frame_).rightMultiply(tool_frame_).toPoseEuler(pose);
}

void BaseKinematics::forwardKinematicsInWorld(const Joint &joint, Pose &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.leftMultiply(inverse_world_frame_).rightMultiply(tool_frame_).toPose(pose);
}

void BaseKinematics::forwardKinematicsInWorld(const Joint &joint, PoseEuler &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.leftMultiply(inverse_world_frame_).rightMultiply(tool_frame_).toPoseEuler(pose);
}



ErrorCode BaseKinematics::inverseKinematicsInBase(const Pose &pose, const Joint &ref, Joint &res)
{
    Matrix matrix = Matrix(pose).rightMultiply(inverse_tool_frame_);
    return inverseKinematics(matrix, ref, res);
}

ErrorCode BaseKinematics::inverseKinematicsInBase(const PoseEuler &pose, const Joint &ref, Joint &res)
{
    Matrix matrix = Matrix(pose).rightMultiply(inverse_tool_frame_);
    return inverseKinematics(matrix, ref, res);
}

ErrorCode BaseKinematics::inverseKinematicsInUser(const Pose &pose, const Joint &ref, Joint &res)
{
    Matrix matrix = Matrix(pose).leftMultiply(user_frame_).rightMultiply(inverse_tool_frame_);
    return inverseKinematics(matrix, ref, res);
}

ErrorCode BaseKinematics::inverseKinematicsInUser(const PoseEuler &pose, const Joint &ref, Joint &res)
{
    Matrix matrix = Matrix(pose).leftMultiply(user_frame_).rightMultiply(inverse_tool_frame_);
    return inverseKinematics(matrix, ref, res);
}

ErrorCode BaseKinematics::inverseKinematicsInWorld(const Pose &pose, const Joint &ref, Joint &res)
{
    Matrix matrix = Matrix(pose).leftMultiply(world_frame_).rightMultiply(inverse_tool_frame_);
    return inverseKinematics(matrix, ref, res);
}

ErrorCode BaseKinematics::inverseKinematicsInWorld(const PoseEuler &pose, const Joint &ref, Joint &res)
{
    Matrix matrix = Matrix(pose).leftMultiply(world_frame_).rightMultiply(inverse_tool_frame_);
    return inverseKinematics(matrix, ref, res);
}

ErrorCode BaseKinematics::inverseKinematicsInTool(const Matrix &tool_coordinate, const PoseEuler &pose, const Joint &ref, Joint &res)
{
    Matrix matrix = Matrix(pose).leftMultiply(tool_coordinate).rightMultiply(inverse_tool_frame_);
    return inverseKinematics(matrix, ref, res);
}


ErrorCode BaseKinematics::setWorldFrame(const PoseEuler &wf)
{
    world_frame_ = wf;
    inverse_world_frame_ = wf;
    inverse_world_frame_.inverse();
    return SUCCESS;
}

ErrorCode BaseKinematics::setWorldFrame(const Matrix &wf)
{
    world_frame_ = wf;
    inverse_user_frame_ = wf;
    inverse_user_frame_.inverse();
    return SUCCESS;
}

ErrorCode BaseKinematics::setUserFrame(const PoseEuler &uf)
{
    user_frame_ = uf;
    inverse_user_frame_ = uf;
    inverse_user_frame_.inverse();
    return SUCCESS;
}

ErrorCode BaseKinematics::setUserFrame(const Matrix &uf)
{
    user_frame_ = uf;
    inverse_user_frame_ = uf;
    inverse_user_frame_.inverse();
    return SUCCESS;
}

ErrorCode BaseKinematics::setToolFrame(const PoseEuler &tf)
{
    tool_frame_ = tf;
    inverse_tool_frame_ = tf;
    inverse_tool_frame_.inverse();
    return SUCCESS;
}

ErrorCode BaseKinematics::setToolFrame(const Matrix &tf)
{
    tool_frame_ = tf;
    inverse_tool_frame_ = tf;
    inverse_tool_frame_.inverse();
    return SUCCESS;
}




}

