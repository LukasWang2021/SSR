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

    Matrix44();
    ~Matrix44();

    bool isEqual(Matrix44& matrix, double valve = 0.001) const;
    Matrix44& leftMultiply(Matrix44& left_matrix);
    void leftMultiply(Matrix44& left_matrix, Matrix44& result_matrix);
    Matrix44& rightMultiply(Matrix44& right_matrix);
    void rightMultiply(Matrix44& right_matrix, Matrix44& result_matrix);

    Matrix44& operator=(const Matrix44& matrix);
    void print(std::string comment = "") const;
private:    
    void multiply(Matrix44& left_matrix, Matrix44& right_matrix, Matrix44& result_matrix);
};

}


#endif

