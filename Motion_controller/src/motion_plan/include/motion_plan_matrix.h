/*************************************************************************
	> File Name: motion_plan_matrix.h
	> Author: 
	> Mail: 
	> Created Time: 2018年01月30日 星期二 14时22分13秒
 ************************************************************************/

#ifndef _MOTION_PLAN_MATRIX_H
#define _MOTION_PLAN_MATRIX_H


#include <fst_datatype.h>
#include <log_manager/log_manager_logger.h>

namespace fst_algorithm
{

class Matrix
{
  public:
    Matrix();
    
    Matrix(const Matrix &rhs);
    Matrix(const double mat[4][4]);
    Matrix(double x, double y, double z);

    Matrix(const fst_controller::PoseEuler &pose);
    Matrix(const fst_controller::Pose      &pose);

    Matrix& operator=(const Matrix &rhs);
    Matrix& operator=(const double mat[4][4]);
    
    Matrix& operator=(const fst_controller::PoseEuler pose);
    Matrix& operator=(const fst_controller::Pose      pose);
    
    double* operator[](size_t row);

    Matrix& identityMatrix(void);
    
    Matrix& transpose(void);
    bool   inverse(void);

    Matrix& rightMultiply(const Matrix &rhs);
    Matrix& rightMultiply(const double m[4][4]);
    Matrix& leftMultiply(const Matrix &rhs);
    Matrix& leftMultiply(const double m[4][4]);

    Matrix& rightRotateX(double angle);
    Matrix& rightRotateY(double angle);
    Matrix& rightRotateZ(double angle);

    Matrix& leftTrans(double dx, double dy, double dz);
    Matrix& rightTrans(double dx, double dy, double dz);
    
    Matrix& transFromDH(const double dh[4], double angle);

    Matrix& fromPoseEuler(const fst_controller::PoseEuler &pose);
    Matrix& fromPose(const fst_controller::Pose &pose);

    fst_controller::PoseEuler toPoseEuler(void);
    void toPoseEuler(fst_controller::PoseEuler &pose);
    
    fst_controller::Pose toPose(void);
    void toPose(fst_controller::Pose &pose);

    void printMatrix(fst_log::Logger *log = NULL);


  private:
    double  matrix_[4][4];
};

}


#endif
