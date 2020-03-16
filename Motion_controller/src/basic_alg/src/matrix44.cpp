#include "matrix44.h"
#include <math.h>
#include <cstring>
#include <iostream>

using namespace std;
using namespace basic_alg;


bool Matrix44::isEqual(const Matrix44& matrix, double valve) const
{
    if(fabs(matrix_[0][0] - matrix.matrix_[0][0]) < valve
        && fabs(matrix_[0][1] - matrix.matrix_[0][1]) < valve
        && fabs(matrix_[0][2] - matrix.matrix_[0][2]) < valve
        && fabs(matrix_[0][3] - matrix.matrix_[0][3]) < valve
        && fabs(matrix_[1][0] - matrix.matrix_[1][0]) < valve
        && fabs(matrix_[1][1] - matrix.matrix_[1][1]) < valve
        && fabs(matrix_[1][2] - matrix.matrix_[1][2]) < valve
        && fabs(matrix_[1][3] - matrix.matrix_[1][3]) < valve
        && fabs(matrix_[2][0] - matrix.matrix_[2][0]) < valve
        && fabs(matrix_[2][1] - matrix.matrix_[2][1]) < valve
        && fabs(matrix_[2][2] - matrix.matrix_[2][2]) < valve
        && fabs(matrix_[2][3] - matrix.matrix_[2][3]) < valve
        && fabs(matrix_[3][0] - matrix.matrix_[3][0]) < valve
        && fabs(matrix_[3][1] - matrix.matrix_[3][1]) < valve
        && fabs(matrix_[3][2] - matrix.matrix_[3][2]) < valve
        && fabs(matrix_[3][3] - matrix.matrix_[3][3]) < valve)
    {
        return true;
    }
    else
    {
        return false;
    }
}

Matrix44& Matrix44::leftMultiply(const Matrix44& left_matrix)
{
    Matrix44 tmp_matrix;
    multiply(left_matrix, *this, tmp_matrix);
    *this = tmp_matrix;
    return *this;   
}

void Matrix44::leftMultiply(const Matrix44& left_matrix, Matrix44& result_matrix) const
{
    multiply(left_matrix, *this, result_matrix);
}

Matrix44& Matrix44::rightMultiply(const Matrix44& right_matrix)
{
    Matrix44 tmp_matrix;
    multiply(*this, right_matrix, tmp_matrix);
    *this = tmp_matrix;
    return *this;
}

void Matrix44::rightMultiply(const Matrix44& right_matrix, Matrix44& result_matrix) const
{
    multiply(*this, right_matrix, result_matrix);
}

Matrix44& Matrix44::operator=(const Matrix44& matrix)
{
    memcpy(&matrix_, &matrix.matrix_, sizeof(matrix_));
    return *this;
}

void Matrix44::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    for(int i = 0; i < 4; ++i)
    {
        for(int j = 0; j < 4; ++j)
            std::cout<<matrix_[i][j]<<" ";
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
}

void Matrix44::multiply(const Matrix44& left_matrix, const Matrix44& right_matrix, Matrix44& result_matrix) const
{
    result_matrix.matrix_[0][0] = left_matrix.matrix_[0][0] * right_matrix.matrix_[0][0] 
                                + left_matrix.matrix_[0][1] * right_matrix.matrix_[1][0]
                                + left_matrix.matrix_[0][2] * right_matrix.matrix_[2][0]
                                + left_matrix.matrix_[0][3] * right_matrix.matrix_[3][0];

    result_matrix.matrix_[0][1] = left_matrix.matrix_[0][0] * right_matrix.matrix_[0][1] 
                                + left_matrix.matrix_[0][1] * right_matrix.matrix_[1][1]
                                + left_matrix.matrix_[0][2] * right_matrix.matrix_[2][1]
                                + left_matrix.matrix_[0][3] * right_matrix.matrix_[3][1];

    result_matrix.matrix_[0][2] = left_matrix.matrix_[0][0] * right_matrix.matrix_[0][2] 
                                + left_matrix.matrix_[0][1] * right_matrix.matrix_[1][2]
                                + left_matrix.matrix_[0][2] * right_matrix.matrix_[2][2]
                                + left_matrix.matrix_[0][3] * right_matrix.matrix_[3][2];

    result_matrix.matrix_[0][3] = left_matrix.matrix_[0][0] * right_matrix.matrix_[0][3] 
                                + left_matrix.matrix_[0][1] * right_matrix.matrix_[1][3]
                                + left_matrix.matrix_[0][2] * right_matrix.matrix_[2][3]
                                + left_matrix.matrix_[0][3] * right_matrix.matrix_[3][3];                                

    result_matrix.matrix_[1][0] = left_matrix.matrix_[1][0] * right_matrix.matrix_[0][0] 
                                + left_matrix.matrix_[1][1] * right_matrix.matrix_[1][0]
                                + left_matrix.matrix_[1][2] * right_matrix.matrix_[2][0]
                                + left_matrix.matrix_[1][3] * right_matrix.matrix_[3][0];

    result_matrix.matrix_[1][1] = left_matrix.matrix_[1][0] * right_matrix.matrix_[0][1] 
                                + left_matrix.matrix_[1][1] * right_matrix.matrix_[1][1]
                                + left_matrix.matrix_[1][2] * right_matrix.matrix_[2][1]
                                + left_matrix.matrix_[1][3] * right_matrix.matrix_[3][1];

    result_matrix.matrix_[1][2] = left_matrix.matrix_[1][0] * right_matrix.matrix_[0][2] 
                                + left_matrix.matrix_[1][1] * right_matrix.matrix_[1][2]
                                + left_matrix.matrix_[1][2] * right_matrix.matrix_[2][2]
                                + left_matrix.matrix_[1][3] * right_matrix.matrix_[3][2];

    result_matrix.matrix_[1][3] = left_matrix.matrix_[1][0] * right_matrix.matrix_[0][3] 
                                + left_matrix.matrix_[1][1] * right_matrix.matrix_[1][3]
                                + left_matrix.matrix_[1][2] * right_matrix.matrix_[2][3]
                                + left_matrix.matrix_[1][3] * right_matrix.matrix_[3][3];

    result_matrix.matrix_[2][0] = left_matrix.matrix_[2][0] * right_matrix.matrix_[0][0] 
                                + left_matrix.matrix_[2][1] * right_matrix.matrix_[1][0]
                                + left_matrix.matrix_[2][2] * right_matrix.matrix_[2][0]
                                + left_matrix.matrix_[2][3] * right_matrix.matrix_[3][0];

    result_matrix.matrix_[2][1] = left_matrix.matrix_[2][0] * right_matrix.matrix_[0][1] 
                                + left_matrix.matrix_[2][1] * right_matrix.matrix_[1][1]
                                + left_matrix.matrix_[2][2] * right_matrix.matrix_[2][1]
                                + left_matrix.matrix_[2][3] * right_matrix.matrix_[3][1];

    result_matrix.matrix_[2][2] = left_matrix.matrix_[2][0] * right_matrix.matrix_[0][2] 
                                + left_matrix.matrix_[2][1] * right_matrix.matrix_[1][2]
                                + left_matrix.matrix_[2][2] * right_matrix.matrix_[2][2]
                                + left_matrix.matrix_[2][3] * right_matrix.matrix_[3][2];

    result_matrix.matrix_[2][3] = left_matrix.matrix_[2][0] * right_matrix.matrix_[0][3] 
                                + left_matrix.matrix_[2][1] * right_matrix.matrix_[1][3]
                                + left_matrix.matrix_[2][2] * right_matrix.matrix_[2][3]
                                + left_matrix.matrix_[2][3] * right_matrix.matrix_[3][3];

    result_matrix.matrix_[3][0] = left_matrix.matrix_[3][0] * right_matrix.matrix_[0][0] 
                                + left_matrix.matrix_[3][1] * right_matrix.matrix_[1][0]
                                + left_matrix.matrix_[3][2] * right_matrix.matrix_[2][0]
                                + left_matrix.matrix_[3][3] * right_matrix.matrix_[3][0];

    result_matrix.matrix_[3][1] = left_matrix.matrix_[3][0] * right_matrix.matrix_[0][1] 
                                + left_matrix.matrix_[3][1] * right_matrix.matrix_[1][1]
                                + left_matrix.matrix_[3][2] * right_matrix.matrix_[2][1]
                                + left_matrix.matrix_[3][3] * right_matrix.matrix_[3][1];

    result_matrix.matrix_[3][2] = left_matrix.matrix_[3][0] * right_matrix.matrix_[0][2] 
                                + left_matrix.matrix_[3][1] * right_matrix.matrix_[1][2]
                                + left_matrix.matrix_[3][2] * right_matrix.matrix_[2][2]
                                + left_matrix.matrix_[3][3] * right_matrix.matrix_[3][2];

    result_matrix.matrix_[3][3] = left_matrix.matrix_[3][0] * right_matrix.matrix_[0][3] 
                                + left_matrix.matrix_[3][1] * right_matrix.matrix_[1][3]
                                + left_matrix.matrix_[3][2] * right_matrix.matrix_[2][3]
                                + left_matrix.matrix_[3][3] * right_matrix.matrix_[3][3];                                
}

