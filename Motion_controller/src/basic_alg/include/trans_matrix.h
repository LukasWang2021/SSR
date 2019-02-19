#ifndef TRANS_MATRIX_H
#define TRANS_MATRIX_H

#include "rotation_matrix.h"
#include "point.h"
#include "pose_euler.h"
#include "pose_quaternion.h"
#include <string>

namespace basic_alg
{
class PoseEuler;
class PoseQuaternion;

class TransMatrix
{
public:
    RotationMatrix rotation_matrix_;
    Point trans_vector_;

    TransMatrix();
    TransMatrix(double d, double a, double alpha, double theta);
    ~TransMatrix();

    bool isEqual(TransMatrix& matrix, double valve = 0.001) const;
    void convertToPoseEuler(basic_alg::PoseEuler& pose_euler) const;
    void convertToPoseQuaternion(basic_alg::PoseQuaternion& pose_quaternion) const;    
    TransMatrix& leftMultiply(TransMatrix& left_matrix);
    void leftMultiply(TransMatrix& left_matrix, TransMatrix& result_matrix);
    TransMatrix& rightMultiply(TransMatrix& right_matrix);
    void rightMultiply(TransMatrix& right_matrix, TransMatrix& result_matrix);
    bool inverse(double valve = 0.001);
    bool inverse(TransMatrix& result_matrix, double valve = 0.001);
    void print(std::string comment = "") const;
    
private:
    void multiply(TransMatrix& left_matrix, TransMatrix& right_matrix, TransMatrix& result_matrix);
};

}

#endif
