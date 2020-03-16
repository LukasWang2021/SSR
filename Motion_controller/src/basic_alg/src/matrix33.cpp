#include "matrix33.h"
#include <math.h>
#include <cstring>
#include <iostream>

using namespace std;
using namespace basic_alg;


bool Matrix33::isEqual(const Matrix33& matrix, double valve) const
{
    if(fabs(matrix_[0][0] - matrix.matrix_[0][0]) < valve
        && fabs(matrix_[0][1] - matrix.matrix_[0][1]) < valve
        && fabs(matrix_[0][2] - matrix.matrix_[0][2]) < valve
        && fabs(matrix_[1][0] - matrix.matrix_[1][0]) < valve
        && fabs(matrix_[1][1] - matrix.matrix_[1][1]) < valve
        && fabs(matrix_[1][2] - matrix.matrix_[1][2]) < valve
        && fabs(matrix_[2][0] - matrix.matrix_[2][0]) < valve
        && fabs(matrix_[2][1] - matrix.matrix_[2][1]) < valve
        && fabs(matrix_[2][2] - matrix.matrix_[2][2]) < valve)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Matrix33::eye(void)
{
    matrix_[0][0] = 1; matrix_[0][1] = 0; matrix_[0][2] = 0;
    matrix_[1][0] = 0; matrix_[1][1] = 1; matrix_[1][2] = 0;
    matrix_[2][0] = 0; matrix_[2][1] = 0; matrix_[2][2] = 1;
}

Matrix33& Matrix33::leftMultiply(const Matrix33& left_matrix)
{
    Matrix33 tmp_matrix;
    multiply(left_matrix, *this, tmp_matrix);
    *this = tmp_matrix;
    return *this;
}

void Matrix33::leftMultiply(const Matrix33& left_matrix, Matrix33& result_matrix) const
{
    multiply(left_matrix, *this, result_matrix);
}

Matrix33& Matrix33::rightMultiply(const Matrix33& right_matrix)
{
    Matrix33 tmp_matrix;
    multiply(*this, right_matrix, tmp_matrix);
    *this = tmp_matrix;
    return *this;
}

void Matrix33::rightMultiply(const Matrix33& right_matrix, Matrix33& result_matrix) const
{
    multiply(*this, right_matrix, result_matrix);
}

Matrix33& Matrix33::operator=(const Matrix33& matrix)
{
    memcpy(&matrix_, &matrix.matrix_, sizeof(matrix_));
    return *this;
}

void Matrix33::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
            std::cout<<matrix_[i][j]<<" ";
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
}

void Matrix33::multiply(const Matrix33& left_matrix, const Matrix33& right_matrix, Matrix33& result_matrix) const
{
    result_matrix.matrix_[0][0] = left_matrix.matrix_[0][0] * right_matrix.matrix_[0][0] 
                                + left_matrix.matrix_[0][1] * right_matrix.matrix_[1][0]
                                + left_matrix.matrix_[0][2] * right_matrix.matrix_[2][0];

    result_matrix.matrix_[0][1] = left_matrix.matrix_[0][0] * right_matrix.matrix_[0][1] 
                                + left_matrix.matrix_[0][1] * right_matrix.matrix_[1][1]
                                + left_matrix.matrix_[0][2] * right_matrix.matrix_[2][1];  

    result_matrix.matrix_[0][2] = left_matrix.matrix_[0][0] * right_matrix.matrix_[0][2] 
                                + left_matrix.matrix_[0][1] * right_matrix.matrix_[1][2]
                                + left_matrix.matrix_[0][2] * right_matrix.matrix_[2][2];  

    result_matrix.matrix_[1][0] = left_matrix.matrix_[1][0] * right_matrix.matrix_[0][0] 
                                + left_matrix.matrix_[1][1] * right_matrix.matrix_[1][0]
                                + left_matrix.matrix_[1][2] * right_matrix.matrix_[2][0];

    result_matrix.matrix_[1][1] = left_matrix.matrix_[1][0] * right_matrix.matrix_[0][1] 
                                + left_matrix.matrix_[1][1] * right_matrix.matrix_[1][1]
                                + left_matrix.matrix_[1][2] * right_matrix.matrix_[2][1];  
    
    result_matrix.matrix_[1][2] = left_matrix.matrix_[1][0] * right_matrix.matrix_[0][2] 
                                + left_matrix.matrix_[1][1] * right_matrix.matrix_[1][2]
                                + left_matrix.matrix_[1][2] * right_matrix.matrix_[2][2];  

    result_matrix.matrix_[2][0] = left_matrix.matrix_[2][0] * right_matrix.matrix_[0][0] 
                                + left_matrix.matrix_[2][1] * right_matrix.matrix_[1][0]
                                + left_matrix.matrix_[2][2] * right_matrix.matrix_[2][0];

    result_matrix.matrix_[2][1] = left_matrix.matrix_[2][0] * right_matrix.matrix_[0][1] 
                                + left_matrix.matrix_[2][1] * right_matrix.matrix_[1][1]
                                + left_matrix.matrix_[2][2] * right_matrix.matrix_[2][1];  
    
    result_matrix.matrix_[2][2] = left_matrix.matrix_[2][0] * right_matrix.matrix_[0][2] 
                                + left_matrix.matrix_[2][1] * right_matrix.matrix_[1][2]
                                + left_matrix.matrix_[2][2] * right_matrix.matrix_[2][2];                                 
}

