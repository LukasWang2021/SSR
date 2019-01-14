#ifndef BASIC_MATRIX_H
#define BASIC_MATRIX_H

#include <base_datatype.h>
#include <log_manager/log_manager_logger.h>

namespace basic_alg
{


class Matrix
{
  public:
    Matrix();

    Matrix(const Matrix &rhs);
    Matrix(const double (&mat)[4][4]);
    Matrix(double x, double y, double z);

    Matrix(const fst_mc::PoseEuler &pose);
    Matrix(const fst_mc::Pose      &pose);

    Matrix& operator=(const Matrix &rhs);
    Matrix& operator=(const double (&mat)[4][4]);

    Matrix& operator=(const fst_mc::PoseEuler pose);
    Matrix& operator=(const fst_mc::Pose      pose);

    double* operator[](size_t row);

    Matrix& eye(void);
    Matrix& transpose(void);
    bool   inverse(void);

    Matrix& rightMultiply(const Matrix &rhs);
    Matrix& rightMultiply(const double (&m)[4][4]);
    Matrix& leftMultiply(const Matrix &rhs);
    Matrix& leftMultiply(const double (&m)[4][4]);

    Matrix& rotateAroundX(double angle);
    Matrix& rotateAroundY(double angle);
    Matrix& rotateAroundZ(double angle);

    Matrix& leftTrans(double dx, double dy, double dz);
    Matrix& rightTrans(double dx, double dy, double dz);

    Matrix& transFromDH(const double (&dh)[4], double angle);

    Matrix& fromPoseEuler(const fst_mc::PoseEuler &pose);
    Matrix& fromPose(const fst_mc::Pose &pose);

    fst_mc::PoseEuler toPoseEuler(void);
    void toPoseEuler(fst_mc::PoseEuler &pose);

    fst_mc::Pose toPose(void);
    void toPose(fst_mc::Pose &pose);

    void printMatrix(fst_log::Logger *log = NULL);


  private:
    double  matrix_[4][4];
};


}

#endif


