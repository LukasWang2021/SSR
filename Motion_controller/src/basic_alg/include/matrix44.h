#ifndef MATRIX44_H
#define MATRIX44_H

#include <stddef.h>
#include <string>

namespace basic_alg
{
class Matrix44
{
public:
    double matrix_[4][4];

    bool isEqual(const Matrix44& matrix, double valve = 0.001) const;
    Matrix44& leftMultiply(const Matrix44& left_matrix);
    void leftMultiply(const Matrix44& left_matrix, Matrix44& result_matrix) const;
    Matrix44& rightMultiply(const Matrix44& right_matrix);
    void rightMultiply(const Matrix44& right_matrix, Matrix44& result_matrix) const;

    Matrix44& operator=(const Matrix44& matrix);
    void print(std::string comment = "") const;
private:    
    void multiply(const Matrix44& left_matrix, const Matrix44& right_matrix, Matrix44& result_matrix) const;
};

}


#endif

