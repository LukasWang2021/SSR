// 基础函数

#include <math.h>
#include <lib_controller/trajplan.h>

//符号函数
int fst_controller::TrajPlan::Sign(double b)
{
	int a = 0;
	if (b > 0)
	{
		a = 1;
	}
	else if (b < 0)
	{
		a = -1;
	};
	return a;
}
//矩阵乘以向量

int fst_controller::TrajPlan::MULTIPLY_MV(double M[4][4], double V[4], double U[4])
{
	for (int i = 0; i < 4; i++)
	{
		U[i] = 0;
			for (int j = 0; j < 4; j++)
			{
				U[i] += M[i][j] * V[j];
			}
	}
	return 0;
}

// 4*4矩阵相乘
int fst_controller::TrajPlan::MULTIPLY_M(double M1[4][4], double M2[4][4], double M[4][4])		//矩阵乘法
{
	double T[4][4];
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			T[i][j] = 0;
			for (int k = 0; k < 4; k++)
			{
				T[i][j] += M1[i][k] * M2[k][j];

			}
			/*			cout << M[i][j] << "	";	*/
		}
		/*		cout << endl;	*/
	}
	/*	cout << endl;	*/
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			M[i][j] = T[i][j];
		}
	}
	return 0;
}





// 求向量的模
double fst_controller::TrajPlan::norm(double Vector[3])
{
	double n = 0;
	for (int i = 0; i < 3; i++)
	{
		n += Vector[i] * Vector[i];
	}
	n = sqrt(n);
	return n;
}

//叉乘

int fst_controller::TrajPlan::Cross(double u[3], double v[3], double w[3])
{
	w[0] = u[1] * v[2] - v[1] * u[2];
	w[1] = u[2] * v[0] - v[2] * u[0];
	w[2] = u[0] * v[1] - u[1] * v[0];
	return 0;
}

//内积

double fst_controller::TrajPlan::InnerProduct(double u[3], double v[3])
{
	return u[0] * v[0]+u[1] * v[1] + u[2] * v[2];
}

//常数与向量相乘

int fst_controller::TrajPlan::nV3(double n, double V[3], double nV[3])
{
	for (int i = 0; i < 3; i++)
	{
		nV[i] = n*V[i];
	}
	return 0;
}


//相加
int fst_controller::TrajPlan::add3(double u[3], double v[3], double w[3])
{
	for (int i = 0; i < 3; i++)
	{
		w[i] = u[i] + v[i];
	}
	return 0;
}


int fst_controller::TrajPlan::Assign3(double u[3], double v[3])
{
	for (int i = 0; i < 3; i++)
	{
		v[i] = u[i] ;
	}
	return 0;
}
int fst_controller::TrajPlan::Assign4(double u[4], double v[4])
{
	for (int i = 0; i < 4; i++)
	{
		v[i] = u[i];
	}
	return 0;
}



int fst_controller::TrajPlan::print4(double V[4])
{
	cout << endl;
	cout << V[0] << "	" << V[1] << "	" << V[2] << "	" << V[3] << endl;
	return 0;
}

int fst_controller::TrajPlan::print3(double V[3])
{
	cout << endl;
	cout << V[0] << "	" << V[1] << "	" << V[2] << endl;
	return 0;
}

int fst_controller::TrajPlan::print6(double V[6])
{
	cout << endl;
	cout << V[0] << "  " << V[1] << "  " << V[2] << "  " << V[3] << "  " << V[4] << "  " << V[5] << endl;
	return 0;
}
