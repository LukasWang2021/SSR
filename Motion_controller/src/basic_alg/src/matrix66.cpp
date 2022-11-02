#include "matrix66.h"
#include <math.h>
#include <cstring>
#include <iostream>

using namespace std;
using namespace basic_alg;

Matrix66& Matrix66::eye(double gain)
{
	memset(&matrix_[0][0], 0, 36*sizeof(double));
	matrix_[0][0] = gain;
	matrix_[1][1] = gain;
	matrix_[2][2] = gain;
	matrix_[3][3] = gain;
	matrix_[4][4] = gain;
	matrix_[5][5] = gain;
	return *this;
}

Matrix66& Matrix66::leftMultiply(const Matrix66& left_matrix)
{
    Matrix66 tmp_matrix;
    multiply(left_matrix, *this, tmp_matrix);
    *this = tmp_matrix;
    return *this;   
}

void Matrix66::leftMultiply(const Matrix66& left_matrix, Matrix66& result_matrix) const
{
    multiply(left_matrix, *this, result_matrix);
}

Matrix66& Matrix66::rightMultiply(const Matrix66& right_matrix)
{
    Matrix66 tmp_matrix;
    multiply(*this, right_matrix, tmp_matrix);
    *this = tmp_matrix;
    return *this;
}

void Matrix66::rightMultiply(const Matrix66& right_matrix, Matrix66& result_matrix) const
{
    multiply(*this, right_matrix, result_matrix);
}

Matrix66& Matrix66::operator=(const Matrix66& matrix)
{
    memcpy(&matrix_, &matrix.matrix_, sizeof(matrix_));
    return *this;
}

Matrix66& Matrix66::sum(const Matrix66& mtx, int eye)
{
	if(eye == 0)
	{
		for(int i=0;i<6;i++)
		{
			for(int j=0;j<6;j++)
			{
				matrix_[i][j] += mtx.matrix_[i][j];
			}
		}
	
	}else if(eye == 1){

		for(int i=0;i<6;i++)
		{
			matrix_[i][i] += mtx.matrix_[i][i];
		}
	}
	return *this;
}


void Matrix66::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    for(int i = 0; i < 6; ++i)
    {
        for(int j = 0; j < 6; ++j)
            std::cout<<matrix_[i][j]<<" ";
        std::cout<<std::endl;
    }
    //std::cout<<std::endl;
}

void Matrix66::multiply(const Matrix66& left_matrix, const Matrix66& right_matrix, Matrix66& result_matrix) const
{
	int i, j, k;
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			result_matrix.matrix_[i][j] = 0.0;
			for(k = 0; k < 6; k++)
			{
				result_matrix.matrix_[i][j] += left_matrix.matrix_[i][k]*right_matrix.matrix_[k][j];
			}
		}
	}                         
}

void Matrix66::transpose(void)
{
	Matrix66 res_mtx;
	transpose(res_mtx);
	*this = res_mtx;
}

void Matrix66::transpose(Matrix66& res_mtx)
{
	for(int i=0; i<6; i++)
	{
		res_mtx.matrix_[i][0] = matrix_[0][i];
		res_mtx.matrix_[i][1] = matrix_[1][i];
		res_mtx.matrix_[i][2] = matrix_[2][i];
		res_mtx.matrix_[i][3] = matrix_[3][i];
		res_mtx.matrix_[i][4] = matrix_[4][i];
		res_mtx.matrix_[i][5] = matrix_[5][i];
	}
}

void Matrix66::multiplyVect(double (&src_vect)[6], double (&res_vect)[6])
{
	for(int i=0;i<6;i++)
	{
		res_vect[i] = 0.0;
		res_vect[i] += matrix_[i][0]*src_vect[0];
		res_vect[i] += matrix_[i][1]*src_vect[1];
		res_vect[i] += matrix_[i][2]*src_vect[2];
		res_vect[i] += matrix_[i][3]*src_vect[3];
		res_vect[i] += matrix_[i][4]*src_vect[4];
		res_vect[i] += matrix_[i][5]*src_vect[5];
	}
}

void Matrix66::multiplyVect(const Joint src_vect, Joint& res_vect)
{
	for(int i=0;i<6;i++)
	{
		res_vect[i] = 0.0;
		res_vect[i] += matrix_[i][0]*src_vect[0];
		res_vect[i] += matrix_[i][1]*src_vect[1];
		res_vect[i] += matrix_[i][2]*src_vect[2];
		res_vect[i] += matrix_[i][3]*src_vect[3];
		res_vect[i] += matrix_[i][4]*src_vect[4];
		res_vect[i] += matrix_[i][5]*src_vect[5];
	}
}


