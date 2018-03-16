/*************************************************************************
	> File Name: motion_plan_matrix.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年01月30日 星期二 14时27分01秒
 ************************************************************************/

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <motion_plan_matrix.h>
#include <motion_plan_basic_function.h>


using namespace fst_controller;

namespace fst_algorithm
{

    Matrix::Matrix()
    {
        memset(matrix_, 0, sizeof(matrix_));
    }

    Matrix::Matrix(const Matrix &rhs)
    {
        memcpy(matrix_, rhs.matrix_, sizeof(matrix_));
    }

    Matrix::Matrix(const double mat[4][4])
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
        if (this != &rhs)   *this = rhs;
        return *this;
    }

    Matrix& Matrix::operator=(const double mat[4][4])
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

    Matrix& Matrix::identityMatrix(void)
    {
        fst_algorithm::eye4(matrix_);
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
        if (this != &rhs) {
            fst_algorithm::rightMultiplyMat2Mat(matrix_, rhs.matrix_);
        }
        else {
            double tmp[4][4];
            fst_algorithm::mulMat2Mat(matrix_, matrix_, tmp);
            *this = tmp;
        }

        return *this;
    }

    Matrix& Matrix::rightMultiply(const double m[4][4])
    {
        if (this->matrix_ != m) {
            fst_algorithm::rightMultiplyMat2Mat(matrix_, m);
        }
        else {
            double tmp[4][4];
            fst_algorithm::mulMat2Mat(matrix_, matrix_, tmp);
            *this = tmp;
        }

        return *this;
    }

    Matrix& Matrix::leftMultiply(const Matrix &rhs)
    {
        if (this != &rhs) {
            fst_algorithm::leftMultiplyMat2Mat(rhs.matrix_, matrix_);
        }
        else {
            double tmp[4][4];
            fst_algorithm::mulMat2Mat(matrix_, matrix_, tmp);
            *this = tmp;
        }

        return *this;
    }

    Matrix& Matrix::leftMultiply(const double m[4][4])
    {
        if (this->matrix_ != m) {
            fst_algorithm::leftMultiplyMat2Mat(m, matrix_);
        }
        else {
            double tmp[4][4];
            fst_algorithm::mulMat2Mat(matrix_, matrix_, tmp);
            *this = tmp;
        }

        return *this;
    }

    Matrix& Matrix::rightRotateX(double angle)
    {
        fst_algorithm::rightRotateX(angle, matrix_);
        return *this;
    }
    
    Matrix& Matrix::rightRotateY(double angle)
    {
        fst_algorithm::rightRotateY(angle, matrix_);
        return *this;
    }
    
    Matrix& Matrix::rightRotateZ(double angle)
    {
        fst_algorithm::rightRotateZ(angle, matrix_);
        return *this;
    }

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
    

    Matrix& Matrix::transFromDH(const double dh[4], double angle)
    {
        rightRotateX(dh[0]);
        rightTrans(dh[1], 0, 0);
        rightRotateZ(dh[3] + angle);
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
        fst_algorithm::Matrix2PoseEuler(matrix_, pose);
        return pose;
    }
    
    void Matrix::toPoseEuler(PoseEuler &pose)
    {
        fst_algorithm::Matrix2PoseEuler(matrix_, pose);
    }

    Pose Matrix::toPose(void)
    {
        Pose pose;
        fst_algorithm::Matrix2Pose(matrix_, pose);
        return pose;
    }

    void Matrix::toPose(Pose &pose)
    {
        fst_algorithm::Matrix2Pose(matrix_, pose);
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

