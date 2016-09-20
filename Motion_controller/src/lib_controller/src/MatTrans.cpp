
#include <lib_controller/trajplan.h>


int  fst_controller::TrajPlan::RotX(double t, double M[4][4])
{
	double T[4][4] = { { 1, 0, 0, 0 }, { 0, cos(t), -sin(t), 0 }, { 0, sin(t), cos(t), 0 }, { 0, 0, 0, 1 } };
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			M[i][j] = T[i][j];
		}
	}
	return 0;
}

int  fst_controller::TrajPlan::RotY(double t, double M[4][4])
{
	double T[4][4] = { { cos(t), 0, sin(t), 0 }, { 0, 1, 0, 0 }, { -sin(t), 0, cos(t), 0 }, { 0, 0, 0, 1 } };
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			M[i][j] = T[i][j];
		}
	}
	return 0;
}

int  fst_controller::TrajPlan::RotZ(double t, double M[4][4])
{
	double T[4][4] = { { cos(t), -sin(t), 0, 0 }, { sin(t), cos(t), 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			M[i][j] = T[i][j];
		}
	}
	return 0;
}

int  fst_controller::TrajPlan::Trans(double x, double y, double z, double M[4][4])
{
	double T[4][4] = { { 1,0,0, x }, { 0,1,0,y }, { 0, 0, 1, z }, { 0, 0, 0, 1 } };
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			M[i][j] = T[i][j];
		}
	}
	return 0;
}

int  fst_controller::TrajPlan::Euler2Matrix(double x, double y, double z, double a, double b, double c, double Pose_M[4][4])
{
	double tmp1[4][4] = {};
	double tmp2[4][4] = {};
	Trans(x, y, z, tmp1);
	RotZ(a, tmp2);
	MULTIPLY_M(tmp1, tmp2, tmp1);
	RotY(b, tmp2);
	MULTIPLY_M(tmp1, tmp2, tmp1);
	RotX(c, tmp2);
	MULTIPLY_M(tmp1, tmp2, Pose_M);
	return 0;
}

int  fst_controller::TrajPlan::Matrix2Euler(double R[4][4],double& x, double &y, double &z, double &a, double &b, double &c)
{
	double nx = R[0][0]; double ny = R[1][0]; double nz = R[2][0];
	double ox = R[0][1]; double oy = R[1][1];double oz = R[2][1]; 
	double az = R[2][2];
	x = R[0][3]; y = R[1][3]; z = R[2][3];
	b = atan2(-nz, sqrt(nx *nx + ny*ny));
	if ((oz *oz+ az*az)>0.000000001)
	{

		a = atan2(ny / cos(b), nx / cos(b));
		c = atan2(oz / cos(b), az / cos(b));
	}
	else
	{
		a = atan2(-ox, oy);
		c = 0;
	}
	return 0;
}

int  fst_controller::TrajPlan::DHMatrix(double L[4], double q, double M[4][4])
{
	double tmp1[4][4] = {}, tmp2[4][4] = {};
	RotX(L[0], &tmp1[0]);
	Trans(L[1],0,0, &tmp2[0]);
	MULTIPLY_M(tmp1, tmp2, &tmp1[0]);
	RotZ(L[3] + q, &tmp2[0]);
	MULTIPLY_M(tmp1, tmp2, &tmp1[0]);
	Trans( 0, 0, L[2],&tmp2[0]);
	MULTIPLY_M(tmp1, tmp2, &tmp1[0]);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			M[i][j] = tmp1[i][j];
		}
	}

	return 0;

}

#define N 4
double getA(double arcs[N][N], int n)
{
	if (n == 1)
	{
		return arcs[0][0];
	}
	double ans = 0;
	double temp[N][N] = { 0.0 };
	int i, j, k;
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n - 1; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				temp[j][k] = arcs[j + 1][(k >= i) ? k + 1 : k];

			}
		}
		double t = getA(temp, n - 1);
		if (i % 2 == 0)
		{
			ans += arcs[0][i] * t;
		}
		else
		{
			ans -= arcs[0][i] * t;
		}
	}
	return ans;
}

//计算每一行每一列的每个元素所对应的余子式，组成A*
void  getAStar(double arcs[N][N], int n, double ans[N][N])
{
	if (n == 1)
	{
		ans[0][0] = 1;
		return;
	}
	int i, j, k, t;
	double temp[N][N];
	for (i = 0; i<n; i++)
	{
		for (j = 0; j<n; j++)
		{
			for (k = 0; k<n - 1; k++)
			{
				for (t = 0; t<n - 1; t++)
				{
					temp[k][t] = arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
				}
			}

			ans[j][i] = getA(temp, n - 1);
			if ((i + j) % 2 == 1)
			{
				ans[j][i] = -ans[j][i];
			}
		}
	}
}

//得到给定矩阵src的逆矩阵保存到des中。
bool  fst_controller::TrajPlan::InverseMatrix(double src[N][N], double des[N][N])
{
	int n;
	n = N;
	double flag = getA(src, n);
	double t[N][N];
	if (flag == 0)
	{
		return false;
	}
	else
	{
		getAStar(src, n, t);
		for (int i = 0; i<n; i++)
		{
			for (int j = 0; j<n; j++)
			{
				des[i][j] = t[i][j] / flag;
			}

		}
	}

	return true;

}
