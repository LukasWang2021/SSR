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

    Matrix33();
    ~Matrix33();

    bool isEqual(Matrix33& matrix, double valve = 0.001) const;
    Matrix33& leftMultiply(Matrix33& left_matrix);
    void leftMultiply(Matrix33& left_matrix, Matrix33& result_matrix);
    Matrix33& rightMultiply(Matrix33& right_matrix);
    void rightMultiply(Matrix33& right_matrix, Matrix33& result_matrix);

    Matrix33& operator=(const Matrix33& matrix);

    void print(std::string comment = "") const;
private:    
    void multiply(Matrix33& left_matrix, Matrix33& right_matrix, Matrix33& result_matrix);
};

}


#endif

