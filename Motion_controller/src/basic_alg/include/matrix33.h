#ifndef MATRIX33_H
#define MATRIX33_H

#include <stddef.h>
#include <string>

namespace basic_alg
{
class Matrix33
{
public:
    double matrix_[3][3];

    bool isEqual(const Matrix33& matrix, double valve = 0.001) const;
    Matrix33& leftMultiply(const Matrix33& left_matrix);
    void leftMultiply(const Matrix33& left_matrix, Matrix33& result_matrix);
    Matrix33& rightMultiply(const Matrix33& right_matrix);
    void rightMultiply(const Matrix33& right_matrix, Matrix33& result_matrix);

    Matrix33& operator=(const Matrix33& matrix);

    void print(std::string comment = "") const;
private:    
    void multiply(const Matrix33& left_matrix, const Matrix33& right_matrix, Matrix33& result_matrix);
};

}


#endif

