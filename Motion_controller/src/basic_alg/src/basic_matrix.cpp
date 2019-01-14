#include <string.h>
#include <iostream>
#include <basic_matrix.h>
#include <basic_alg.h>

using namespace fst_mc;

namespace basic_alg
{

Matrix::Matrix()
{
    memset(matrix_, 0, sizeof(matrix_));
}

Matrix::Matrix(const Matrix &rhs)
{
    memcpy(matrix_, rhs.matrix_, sizeof(matrix_));
}

Matrix::Matrix(const double (&mat)[4][4])
{
    memcpy(matrix_, mat, sizeof(matrix_));
}

Matrix::Matrix(double x, double y, double z)
{
    trans(x, y, z, matrix_);
}

Matrix::Matrix(const PoseEuler &pose)
{
    PoseEuler2Matrix(pose, matrix_);
}

Matrix::Matrix(const Pose &pose)
{
    Pose2Matrix(pose, matrix_);
}

Matrix& Matrix::operator=(const Matrix &rhs)
{
    if (this != &rhs)
        memcpy(matrix_, rhs.matrix_, sizeof(matrix_));
    return *this;
}

Matrix& Matrix::operator=(const double (&mat)[4][4])
{
    memcpy(matrix_, mat, sizeof(matrix_));
    return *this;
}

Matrix& Matrix::operator=(const PoseEuler pose)
{
    return operator=(Matrix(pose));
}

Matrix& Matrix::operator=(const Pose pose)
{
    return operator=(Matrix(pose));
}

double* Matrix::operator[](size_t row)
{
    assert(row < 4);
    return matrix_[row];
}

Matrix& Matrix::eye(void)
{
    basic_alg::eye(matrix_);
    return *this;
}

bool Matrix::inverse(void)
{
    return inverseMatrix(matrix_);
}

Matrix& Matrix::transpose(void)
{
    double tmp;

    tmp = matrix_[0][1]; matrix_[0][1] = matrix_[1][0]; matrix_[1][0] = tmp;
    tmp = matrix_[0][2]; matrix_[0][2] = matrix_[2][0]; matrix_[2][0] = tmp;
    tmp = matrix_[0][3]; matrix_[0][3] = matrix_[3][0]; matrix_[3][0] = tmp;
    tmp = matrix_[1][2]; matrix_[1][2] = matrix_[2][1]; matrix_[2][1] = tmp;
    tmp = matrix_[1][3]; matrix_[1][3] = matrix_[3][1]; matrix_[3][1] = tmp;
    tmp = matrix_[2][3]; matrix_[2][3] = matrix_[3][2]; matrix_[3][2] = tmp;

    return *this;
}

Matrix& Matrix::rightMultiply(const Matrix &rhs)
{
    if (this != &rhs)
    {
        rightMulMatrix2Matrix(matrix_, rhs.matrix_);
    }
    else
    {
        double tmp[4][4];
        mulMatrix2Matrix(matrix_, matrix_, tmp);
        *this = tmp;
    }

    return *this;
}

Matrix& Matrix::rightMultiply(const double (&m)[4][4])
{
    if (this->matrix_ != m)
    {
        rightMulMatrix2Matrix(matrix_, m);
    }
    else
    {
        double tmp[4][4];
        mulMatrix2Matrix(matrix_, matrix_, tmp);
        *this = tmp;
    }

    return *this;
}

Matrix& Matrix::leftMultiply(const Matrix &rhs)
{
    if (this != &rhs)
    {
        leftMulMatrix2Matrix(rhs.matrix_, matrix_);
    }
    else
    {
        double tmp[4][4];
        mulMatrix2Matrix(matrix_, matrix_, tmp);
        *this = tmp;
    }

    return *this;
}

Matrix& Matrix::leftMultiply(const double (&m)[4][4])
{
    if (this->matrix_ != m)
    {
        leftMulMatrix2Matrix(m, matrix_);
    }
    else
    {
        double tmp[4][4];
        mulMatrix2Matrix(matrix_, matrix_, tmp);
        *this = tmp;
    }

    return *this;
}

Matrix& Matrix::rotateAroundX(double angle)
{
    rotateX(angle, matrix_);
    return *this;
}

Matrix& Matrix::rotateAroundY(double angle)
{
    rotateY(angle, matrix_);
    return *this;
}

Matrix& Matrix::rotateAroundZ(double angle)
{
    rotateZ(angle, matrix_);
    return *this;
}
/*
Matrix& Matrix::rightRotateX(double angle)
{
    rotateX(angle, matrix_);
    return *this;
}

Matrix& Matrix::rightRotateY(double angle)
{
    rotateY(angle, matrix_);
    return *this;
}

Matrix& Matrix::rightRotateZ(double angle)
{
    rotateZ(angle, matrix_);
    return *this;
}
 */

Matrix& Matrix::leftTrans(double dx, double dy, double dz)
{
    matrix_[0][3] += dx;
    matrix_[1][3] += dy;
    matrix_[2][3] += dz;
    return *this;
}

Matrix& Matrix::rightTrans(double dx, double dy, double dz)
{
    matrix_[0][3] += matrix_[0][0] * dx + matrix_[0][1] * dy + matrix_[0][2] * dz;
    matrix_[1][3] += matrix_[1][0] * dx + matrix_[1][1] * dy + matrix_[1][2] * dz;
    matrix_[2][3] += matrix_[2][0] * dx + matrix_[2][1] * dy + matrix_[2][2] * dz;
    return *this;
}


Matrix& Matrix::transFromDH(const double (&dh)[4], double angle)
{
    rotateX(dh[0], matrix_);
    rightTrans(dh[1], 0, 0);
    rotateZ(dh[3] + angle, matrix_);
    rightTrans(0, 0, dh[2]);
    return *this;
}

Matrix& Matrix::fromPoseEuler(const PoseEuler &pose)
{
    PoseEuler2Matrix(pose, matrix_);
    return *this;
}

Matrix& Matrix::fromPose(const Pose &pose)
{
    Pose2Matrix(pose, matrix_);
    return *this;
}

PoseEuler Matrix::toPoseEuler(void)
{
    PoseEuler pose;
    Matrix2PoseEuler(matrix_, pose);
    return pose;
}

void Matrix::toPoseEuler(PoseEuler &pose)
{
    Matrix2PoseEuler(matrix_, pose);
}

Pose Matrix::toPose(void)
{
    Pose pose;
    Matrix2Pose(matrix_, pose);
    return pose;
}

void Matrix::toPose(Pose &pose)
{
    Matrix2Pose(matrix_, pose);
}

void Matrix::printMatrix(fst_log::Logger *log)
{
    if (log != NULL)
    {
        log->info("-------------------------------------------");
        log->info("%.6f, %.6f, %.6f, %.6f", matrix_[0][0], matrix_[0][1], matrix_[0][2], matrix_[0][3]);
        log->info("%.6f, %.6f, %.6f, %.6f", matrix_[1][0], matrix_[1][1], matrix_[1][2], matrix_[1][3]);
        log->info("%.6f, %.6f, %.6f, %.6f", matrix_[2][0], matrix_[2][1], matrix_[2][2], matrix_[2][3]);
        log->info("%.6f, %.6f, %.6f, %.6f", matrix_[3][0], matrix_[3][1], matrix_[3][2], matrix_[3][3]);
        log->info("-------------------------------------------");
    }
    else
    {
        printf("-------------------------------------------\n");
        printf("%.6f, %.6f, %.6f, %.6f\n", matrix_[0][0], matrix_[0][1], matrix_[0][2], matrix_[0][3]);
        printf("%.6f, %.6f, %.6f, %.6f\n", matrix_[1][0], matrix_[1][1], matrix_[1][2], matrix_[1][3]);
        printf("%.6f, %.6f, %.6f, %.6f\n", matrix_[2][0], matrix_[2][1], matrix_[2][2], matrix_[2][3]);
        printf("%.6f, %.6f, %.6f, %.6f\n", matrix_[3][0], matrix_[3][1], matrix_[3][2], matrix_[3][3]);
        printf("-------------------------------------------\n");
    }
}

}


