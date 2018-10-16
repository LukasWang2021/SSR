#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "base_datatype.h"
#include "error_code.h"
#include "trajectory_alg.h"
#include "dynamics_interface.h"

#define MAXAXES 6
#define minimt 1e-9
#define minimq 1e-6
#define minimv 1e-7
#define pi 3.141592625
#define printf

extern fst_algorithm::DynamicsInterface g_dynamics_interface;

using namespace fst_mc;
using namespace std;




int Gauss(double A[2][2], double B[2][2],int N)
{
	int i, j, k;
	double max, temp;
	double t[2][2];

	for (i = 0; i < N; i++)
	{
		for (j = 0; j < N; j++)
		{
			t[i][j] = A[i][j];
		}
	}
	//printf("gauss111\n");
	for (i = 0; i < N; i++)
	{
		for (j = 0; j < N; j++)
		{
			B[i][j] = (i == j) ? (float)1 : 0;
		}
	}
	//printf("gauss222\n");
	for (i = 0; i < N; i++)
	{

		max = t[i][i];
		k = i;
		for (j = i + 1; j < N; j++)
		{
			if (fabs(t[j][i]) > fabs(max))
			{
				max = t[j][i];
				k = j;
			}
		}

		if (k != i)
		{
			for (j = 0; j < N; j++)
			{
				temp = t[i][j];
				t[i][j] = t[k][j];
				t[k][j] = temp;

				temp = B[i][j];
				B[i][j] = B[k][j];
				B[k][j] = temp;
			}
		}

		if (t[i][i] ==0)
		{
			//cout << "There is no inverse matrix!";
			return 0;
		}

		temp = t[i][i];
		for (j = 0; j < N; j++)
		{
			t[i][j] = t[i][j] / temp;
			B[i][j] = B[i][j] / temp;
		}
		for (j = 0; j < N; j++)
		{
			if (j != i)
			{
				temp = t[j][i];
				for (k = 0; k < N; k++)
				{
					t[j][k] = t[j][k] - t[i][k] * temp;
					B[j][k] = B[j][k] - B[i][k] * temp;
				}
			}
		}
	}

	//printf("gauss end\n");
	return 1;
}
void getInverseM(double M[2][2], double IM[2][2], int N)
{

	Gauss(M, IM, N);

}
void getMtxMulVec(double IM[2][2], double V[2], double Res[2], int N)
{
	int i, j;

	for (i = 0; i<N; i++)
	{
		Res[i] = 0;
		for (j = 0; j<N; j++)
		{
			Res[i] = Res[i] + IM[i][j] * V[j];
		}

	}
}
/*
getrootofquadratic：求一元三次方程的根
输入参数：a：三次项的系数 b：二次项的系数 c：一次项的系数 d：常数项的系数
输出参数：Sv[3]  为方程的三个根 某些特殊情况下 仅仅有一个根 有两个复根（忽略）
*/
ErrorCode getrootsofquadratic(double a, double b, double c, double d, double Sv[3],int *svnum)
{
	if (a==0)
	{
		printf("coefficiant is error");
		return 0x002100B00A40001; //三次项求根系数错误
	}
	b = b / a;
	c = c / a;
	d = d / a;


	double q = (3.0*c - (b*b)) / 9.0;
	double r = -(27.0*d) + b*(9.0*c - 2.0*(b*b));
	r = r / 54.0;
	double disc = q*q*q + r*r;
	//The first root is always real.
	double term1 = (b / 3.0);
	double s = 0;

	if (disc > 0) // one root real, two are complex
	{
		s = r + sqrt(disc);

		if (s < 0)
		{
			s = -pow((-s), (1.0 / 3.0));
		}
		else
		{
			s = pow(s, (1.0 / 3.0));
		}
		//s = ((s < 0) ? -Math.pow(-s, (1.0 / 3.0)) : Math.pow(s, (1.0 / 3.0)));
		double t = r - sqrt(disc);

		if (t < 0)
		{
			t = -pow((-t), (1.0 / 3.0));
		}
		else
		{
			t = pow(t, (1.0 / 3.0));
		}
		//t = ((t < 0) ? -Math.pow(-t, (1.0 / 3.0)) : Math.pow(t, (1.0 / 3.0)));
		Sv[0] = -term1 + s + t;
		*svnum = 1;
		return 1;
		//%         term1 = term1 + (s + t) / 2.0;
		//%         dataForm.x3Re.value = dataForm.x2Re.value = -term1;
		//%         term1 = Math.sqrt(3.0)*(-t + s) / 2;
		//%         dataForm.x2Im.value = term1;
		//%         dataForm.x3Im.value = -term1;
	}

	//The remaining options are all real
	double r13 = 0;
	if (disc ==0) // All roots real, at least two are equal.
	{
		if (r < 0)
		{
			r13 = -pow((-r), (1.0 / 3.0));
		}
		else
		{
			r13 = pow(r, (1.0 / 3.0));
		}
		//r13 = ((r < 0) ? -Math.pow(-r, (1.0 / 3.0)) : Math.pow(r, (1.0 / 3.0)));
		Sv[0] = -term1 + 2.0*r13;
		Sv[1] = -(r13 + term1);
		Sv[2] = Sv[1];
		*svnum = 3;
		return 1;
	}
	//Only option left is that all roots are real and unequal(to get here, q < 0)
	q = -q;
	double dum1 = q*q*q;
	dum1 = acos(r / sqrt(dum1));
	r13 = 2.0*sqrt(q);
	Sv[0] = -term1 + r13*cos(dum1 / 3.0);
	Sv[1] = -term1 + r13*cos((dum1 + 2.0*pi) / 3.0);
	Sv[2] = -term1 + r13*cos((dum1 + 4.0*pi) / 3.0);
	*svnum = 3;
	return 0;
}
void phase_coeff_tran(double orcoeff[4], double t, double modcoeff[4])
{

	double a[4];
	for(int i=0;i<4;i++)
		a[i]= orcoeff[i];
	
	modcoeff[0] = a[3]*pow(t,3) + a[2]*pow(t,2) + a[1]*t + a[0];
	modcoeff[1] = 3 * a[3]*pow(t,2) + 2 * a[2]*t + a[1];
	modcoeff[2] = a[2] + 3 * a[3]*t;
	modcoeff[3] = a[3];

}
void coefficient_transform(double orcoeff_matrix[4][4], double t_matrix[4],double coeff_matrix[4][4])
{

	double crm[4],res[4];
	for(int  i = 0;i<4;i++)
	{
		for (int j = 0; j < 4; j++)
		{
			crm[j] = orcoeff_matrix[3 - i][j];
		}
		phase_coeff_tran(crm, t_matrix[3 - i],res);

		for (int j = 0; j < 4; j++)
		{
			coeff_matrix[i][j] = res[j];
		}
	}
}
ErrorCode calculateParam(double q0[MAXAXES], double dq0[MAXAXES], double ddq0[MAXAXES], double qf[MAXAXES], double maxdq[MAXAXES], double maxddq[MAXAXES],double minddq[MAXAXES], double Jm[MAXAXES], double Ti[MAXAXES],const double old_coeff[MAXAXES][4][4],const double old_Ta[MAXAXES][4], const double old_tqf[MAXAXES], const double old_dqf[MAXAXES],const double old_ddqf[MAXAXES], int type, double Ta[MAXAXES][4], double coeff[MAXAXES][4][4], double *maxT, double tqf[MAXAXES], double dqf[MAXAXES], double ddqf[MAXAXES])
{
	//	calculateParam是逐点加减速各项系数的计算函数 包括时间项系数  一元三次方程各项系数，各轴同步时间计算
	//   入参：
	//	q0：无论是加减速都是上一点的关节角值 6×1数组
	//	dq0：对于加速是上一点的关节速度值 对于减速是下一点的关节速度值 6×1数组
	//	ddq0：对于加速是上一点的关节加速度值  对于减速是下一点的关节加速度值 6×1数组
	//	qf：无论对于加减速 都是下一点的关节角值 6×1数组
	//	maxdq：算例中计算的平均速度值 6×1数组
	//	maxddq：算例中计算的动力学校核最大 / 最小加速度值 6×1数组
	//	Jm：各电机默认的Jerk值 6×1数组
	//	Ti：对于计算各轴同步时间来说 传 - 1值 6×1数组
	//	type：1表示加速 2表示减速
	//	dofn：关节数 可默认6

	//	出参：
	//	Ta：计算的时间系数，6×4数组 6为关节数 4个时间段的时间值
	//	coeff：计算的各时间段的三次方程系数值，6×4×4 数组 6为关节数 4为4段时间 4为四个系数
	//	maxT：代入Ti为 - 1 此处返回的maxT为各轴同步的时间 double值 代入Ti为同步时间 不会改变
	//	tqf：对于加速是下一点关节值 对于减速是上一点的关节值
	//	dqf：对于加速是下一点关节速度值  对于减速是上一点关节速度值
	//	ddqf：对于加速是下一点关节加速度值  对于减速是上一点关节加速度值

	//	主体程序

	//定义临时变量 ta1 ta2 ta3 ta4 表示4段时间的系数值 6×1数组
	double ta1[MAXAXES], ta2[MAXAXES], ta3[MAXAXES], ta4[MAXAXES];


	//对各段结束时刻的位置 速度 加速度进行变量内存分配
	double q1[MAXAXES], q2[MAXAXES], q3[MAXAXES], q4[MAXAXES];
	double dq1[MAXAXES], dq2[MAXAXES], dq3[MAXAXES], dq4[MAXAXES];
	double ddq1[MAXAXES], ddq2[MAXAXES], ddq3[MAXAXES], ddq4[MAXAXES];

	//定义临时变量
	double ddq_max[MAXAXES], dq_avg[MAXAXES],dq_avg_new[MAXAXES],deltav[MAXAXES]; //最大加速度 平均速度 总时间 jerk符号
	int sigmaq[MAXAXES], sigma[MAXAXES];
	double tempT[MAXAXES] = { 0 };
	double newT[2] = { 0 };
	double snewT[2] = { 0 };

	double r[3] = { 0 };
	double a0, a1, a2, a3;

	double a, b, c, lamda, slamda;
	int dofn = MAXAXES;

	if (type == 1) // 加速计算
	{
		for (int i = 0; i < dofn; i++)
		{
			//根据关节的运动方向 判断Jerk的符号变量
			if (qf[i] - q0[i] > minimq)
			{
				sigmaq[i] = 1; //正转  符号取 1
			}
			else if (qf[i] - q0[i] < -minimq)
			{
				sigmaq[i] = -1; //反转 符号取 - 1
			}
			else if ((qf[i] - q0[i] < minimq) && (qf[i] - q0[i]>-minimq))
			{
				sigmaq[i] = 0;	//静止 符号取 0
			}

			deltav[i] = maxdq[i] - dq0[i];
			if (deltav[i]> minimv)
			{
				sigma[i] = 1; //acceleration  符号取1
			}
			else if (deltav[i] <-minimv)
			{
				sigma[i] = -1; //deceleration 符号取 - 1
			}
			else if ((deltav[i] < minimv) && (deltav[i]>-minimv))
			{
				sigma[i] = 1;	//constant velocity 符号取1
				deltav[i] = 0;
			}

			if (sigmaq[i] != 0 && (Ti[i] > -1-minimt && Ti[i]<-1+minimt)) // 关节存在运动 且只需计算各轴同步运动时间 进入下面计算
			{
				if (sigma[i] == 1)
					ddq_max[i] = maxddq[i];	//加速度赋值
				else if (sigma[i] == -1)
					ddq_max[i] = minddq[i];
				else
					ddq_max[i] = maxddq[i];

				dq_avg[i] = maxdq[i];		//平均速度赋值

				ta2[i] = (deltav[i] - ddq_max[i] * ddq_max[i] / (sigma[i] * Jm[i])) / (ddq_max[i]);
				
				// ta2 doesn't exist! set ta2 = 0, recompute ddq_max by dq_avg
				if (ta2[i] < 0)
				{
					ta2[i] = 0;
					if (sigma[i]==1)
						ddq_max[i] = +sqrt((deltav[i])*sigma[i] * Jm[i]);
					else if(sigma[i]==-1)
						ddq_max[i] = -sqrt((deltav[i])*sigma[i] * Jm[i]);
				}
				// with new ddq_max recompute the ta1, ta3 and ta4

				ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				ddq2[i] = ddq1[i];
				dq2[i] = dq1[i] + ddq1[i] * ta2[i];
				q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

				ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
				ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
				dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
				q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

				ta4[i] = (qf[i] - q3[i]) / dq3[i];
				
				if (ta4[i] > 0)
				{
					ddq4[i] = 0;
					dq4[i] = dq3[i];
					q4[i] = q3[i] + ta4[i] * dq3[i];

					tqf[i] = q4[i];
					dqf[i] = dq4[i];
					ddqf[i] = ddq4[i];
				}
				// if qf < q3, ta4 doesn't exist. dq_avg cann't be reached. set ta4 == 0. 
				else if (ta4[i] < 0)
				{
					if (ta2[i] > minimt)
					{
						a = (ddq_max[i] / 2);
						b = (dq0[i] + (3 * ddq_max[i]*ddq_max[i]) / (2 * Jm[i] *sigma[i]));
						c = -qf[i] + q0[i] + pow(ddq_max[i],3) / (2 * Jm[i]*Jm[i] * sigma[i]*sigma[i]) +(ddq_max[i] *(ddq_max[i]*ddq_max[i] / (2 * Jm[i] *sigma[i]) + dq0[i])) / (Jm[i] *sigma[i]) + (ddq_max[i] *dq0[i]) / (Jm[i] *sigma[i]);
						lamda = b*b - 4 * a*c;
						// ta2 doesn't exist. set ta2 == 0, recompute ddq_max.
						if (lamda < 0)
						{
							// 给出连立ta1和ta2方程后 三次方程的各项系数  求修正后的ta1值
							a3 =1 ; //a
							a2 = 0; //b
							a1 = (2 * dq0[i]*Jm[i] *sigma[i]); //2*Jm*dq0*sigma
							a0 = (q0[i] - qf[i])*Jm[i]*Jm[i]*sigma[i]*sigma[i]; //Jm^2*q0*sigma^2 - Jm^2*qf*sigma^2

							int snum = 0;
							double tempt = 0; //获得全部的解
							getrootsofquadratic(a3, a2, a1, a0, r, &snum); //计算一元三次方程的根

							if (snum == 1)
							{
								tempt = r[0];
							}
							else
							{
								
								for (int j = 0; j < 3; j++)
								{
									if (sigma[i] == 1)
									{
										tempt = 99;
										if (r[j]<tempt && r[j]>0) //根据要求去掉负时间，取最小时间
										{
											tempt = r[j];
										}
									}
									else if (sigma[i] == -1)
									{
										tempt = -99;
										if (r[j] >tempt && r[j] <0)
										{
											tempt = r[j];
										}
									}
								}
							}
							ddq_max[i] = tempt;

							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ta2[i] = 0;
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ta4[i] = 0;
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];

							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];

						}
						// ta2 exists. 
						else
						{
							newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a); //求出的二次项根1
							newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a); //求出的二次项根2

							if ((newT[0] > 0 && newT[1] > 0)) // 求出两个正根 取短的时间代入计算
							{
								if (newT[0] > newT[1]) // ta2取时间较小的值
									ta2[i] = newT[1];
								else
									ta2[i] = newT[0];
								//重新计算ta2后 更新加速度 速度和位置
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								ta4[i] = 0;
								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] + ta4[i] * dq3[i];

								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}
							else if ((newT[0] > 0 && newT[1] < 0)) // 取正的时间值
							{
								ta2[i] = newT[0];
								//重新计算ta2后 更新加速度 速度和位置
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								ta4[i] = 0;
								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] + ta4[i] * dq3[i];

								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}
							else if ((newT[1] > 0 && newT[0] < 0)) // 取正的时间值
							{
								ta2[i] = newT[1];
								//重新计算ta2后 更新加速度 速度和位置
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								ta4[i] = 0;
								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] + ta4[i] * dq3[i];

								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}
							else if ((newT[0] < 0 && newT[1] < 0))
							{
								ta2[i] = 0;
								a3 = -1 / (pow(Jm[i],2) * pow(sigma[i],2)); //a
								a2 = 0; //b
								a1 = -(2 * dq0[i]) / (Jm[i] *sigma[i]); //2*Jm*dq0*sigma
								a0 = -q0[i] + qf[i]; //Jm^2*q0*sigma^2 - Jm^2*qf*sigma^2

								int snum = 0;
								double tempt = 0; //获得全部的解
								getrootsofquadratic(a3, a2, a1, a0, r, &snum); //计算一元三次方程的根

								if (snum == 1)
								{
									tempt = r[0];
								}
								else
								{

									for (int j = 0; j < 3; j++)
									{
										if (sigma[i] == 1)
										{
											tempt = 99;
											if (r[j]<tempt && r[j]>0) //根据要求去掉负时间，取最小时间
											{
												tempt = r[j];
											}
										}
										else if (sigma[i] == -1)
										{
											tempt = -99;
											if (r[j] > tempt && r[j] <0)
											{
												tempt = r[j];
											}
										}
									}
								}
								ddq_max[i] = tempt;

								ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

								ta2[i] = 0;
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								ta4[i] = 0;
								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] + ta4[i] * dq3[i];

								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}

						}

						
					}
					else if (ta2[i] > -minimt && ta2[i] < minimt)
					{
						a3 = 1; //a
						a2 = 0; //b
						a1 = (2 * dq0[i] * Jm[i] * sigma[i]); //2*Jm*dq0*sigma
						a0 = (q0[i] - qf[i])*Jm[i] * Jm[i] * sigma[i] * sigma[i]; //Jm^2*q0*sigma^2 - Jm^2*qf*sigma^2

						int snum = 0;
						double tempt = 0; //获得全部的解
						getrootsofquadratic(a3, a2, a1, a0, r, &snum); //计算一元三次方程的根

						if (snum == 1)
						{
							tempt = r[0];
						}
						else
						{

							for (int j = 0; j < 3; j++)
							{
								if (sigma[i] == 1)
								{
									tempt = 99;
									if (r[j]<tempt && r[j]>0) //根据要求去掉负时间，取最小时间
									{
										tempt = r[j];
									}
								}
								else if (sigma[i] == -1)
								{
									tempt = -99;
									if (r[j] > tempt && r[j] <0)
									{
										tempt = r[j];
									}
								}
							}
						}
						ddq_max[i] = tempt;

						ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
						ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
						dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
						q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

						ta2[i] = 0;
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						ta4[i] = 0;
						ddq4[i] = 0;
						dq4[i] = dq3[i];
						q4[i] = q3[i] + ta4[i] * dq3[i];

						tqf[i] = q4[i];
						dqf[i] = dq4[i];
						ddqf[i] = ddq4[i];
					}

				}
				else if (ta4[i] > -minimt && ta4[i] < minimt)
				{
					ddq4[i] = 0;
					dq4[i] = dq3[i];
					q4[i] = q3[i] + ta4[i] * dq3[i];

					tqf[i] = q4[i];
					dqf[i] = dq4[i];
					ddqf[i] = ddq4[i];
				}

				//计算总时间
				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
				if ((ta1[i] + ta2[i] + ta3[i] + ta4[i])<0)
				{
					printf("ta error\n");
					return 0x002100B00A40003; //总时间计算错误
				}
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];  //各段时间赋值

				//各段系数赋值
				if ((ta1[i] >minimt) || (ta1[i]>-minimt && ta1[i]<minimt))
				{
					coeff[i][0][0] = 1 / 6.0 * sigma[i] * Jm[i]; // 3阶
					coeff[i][0][1] = 1 / 2.0 * ddq0[i]; // 2阶
					coeff[i][0][2] = dq0[i]; // 1阶
					coeff[i][0][3] = q0[i]; // 0阶
				}
				if ((ta2[i] >minimt) || (ta2[i]>-minimt && ta2[i]<minimt))
				{
					coeff[i][1][0] = 0;
					coeff[i][1][1] = 1 / 2.0 * ddq1[i];
					coeff[i][1][2] = dq1[i];
					coeff[i][1][3] = q1[i];
				}
				if ((ta3[i] >minimt) || (ta3[i]>-minimt && ta3[i]<minimt))
				{
					coeff[i][2][0] = -1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][2][1] = 1 / 2.0 * ddq2[i];
					coeff[i][2][2] = dq2[i];
					coeff[i][2][3] = q2[i];
				}
				if((ta4[i] >minimt) || (ta4[i]>-minimt && ta4[i]<minimt))
				{
					coeff[i][3][0] = 0;
					coeff[i][3][1] = 0;
					coeff[i][3][2] = dq3[i];
					coeff[i][3][3] = q3[i];
				}
			}
			else if (sigmaq[i] != 0 && Ti[i] > 0) // 这一段是根据多轴时间同步后 计算各段的时间系数和多项式系数 并返回结束的位置速度 加速度
			{
				if (sigma[i] == 1)
					ddq_max[i] = maxddq[i];	//加速度赋值
				else if (sigma[i] == -1)
					ddq_max[i] = minddq[i];

				dq_avg[i] = maxdq[i];		//平均速度赋值
				dq_avg_new[i] = (qf[i] - q0[i]) /fabs( Ti[i]);
				if (((Ti[i]-(old_Ta[i][0]+ old_Ta[i][1]+ old_Ta[i][2]+ old_Ta[i][3]))<minimt) && ((Ti[i] - (old_Ta[i][0] + old_Ta[i][1] + old_Ta[i][2] + old_Ta[i][3]))>-minimt) )
				{

					Ta[i][0] = old_Ta[i][0]; Ta[i][1] = old_Ta[i][1]; Ta[i][2] = old_Ta[i][2]; Ta[i][3] = old_Ta[i][3];
					tempT[i] = Ta[i][0] + Ta[i][1] + Ta[i][2] + Ta[i][3];
					coeff[i][0][0] = old_coeff[i][0][0]; coeff[i][0][1] = old_coeff[i][0][1]; coeff[i][0][2] = old_coeff[i][0][2]; coeff[i][0][3] = old_coeff[i][0][3];
					coeff[i][1][0] = old_coeff[i][1][0]; coeff[i][1][1] = old_coeff[i][1][1]; coeff[i][1][2] = old_coeff[i][1][2]; coeff[i][1][3] = old_coeff[i][1][3];
					coeff[i][2][0] = old_coeff[i][2][0]; coeff[i][2][1] = old_coeff[i][2][1]; coeff[i][2][2] = old_coeff[i][2][2]; coeff[i][2][3] = old_coeff[i][2][3];
					coeff[i][3][0] = old_coeff[i][3][0]; coeff[i][3][1] = old_coeff[i][3][1]; coeff[i][3][2] = old_coeff[i][3][2]; coeff[i][3][3] = old_coeff[i][3][3];
				
					tqf[i] = old_tqf[i];
					dqf[i] = old_dqf[i];
					ddqf[i] = old_ddqf[i];
					continue;
				}
				deltav[i] = dq_avg_new[i] - dq0[i];
				if (deltav[i] > minimv)
				{
					sigma[i] = 1; //acceleration  符号取1
				}
				else if (deltav[i] <-minimv)
				{
					sigma[i] = -1; //deceleration 符号取 - 1
				}
				else if ((deltav[i] < minimv) && (deltav[i]>-minimv))
				{
					sigma[i] = 1;	//constant velocity 符号取1
					deltav[i] = 0;
				}
				if (sigma[i] == 1)
					ddq_max[i] = maxddq[i];	//加速度赋值
				else if (sigma[i] == -1)
					ddq_max[i] = minddq[i];

				if ((deltav[i] < minimv) && (deltav[i] > -minimv))
				{
					ta1[i] = 0;
					ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
					dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
					q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

					ta2[i] = 0;
					ddq2[i] = ddq1[i];
					dq2[i] = dq1[i] + ddq1[i] * ta2[i];
					q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

					ta3[i] = 0;
					ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
					dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
					q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

					ta4[i] = Ti[i] - ta1[i] - ta2[i] - ta3[i];
					ddq4[i] = 0;
					dq4[i] = dq3[i];
					q4[i] = q3[i] + ta4[i] * dq3[i];

					tqf[i] = q4[i];
					dqf[i] = dq4[i];
					ddqf[i] = ddq4[i];
				}
				// recompute ta2 with Ti = ta1 + ta2 + ta3 + ta4
				else
				{
					a = (ddq_max[i] / 2);
					b = (-ddq_max[i] * (Ti[i] - (2 * ddq_max[i]) / (Jm[i] * sigma[i])) - pow(ddq_max[i], 2) / (2 * Jm[i] * sigma[i]));
					c = qf[i] - q0[i] - (pow(ddq_max[i], 2) / (Jm[i] * sigma[i]) + dq0[i])*(Ti[i] - (2 * ddq_max[i]) / (Jm[i] * sigma[i])) - pow(ddq_max[i], 3) / (2 * pow(Jm[i], 2) * pow(sigma[i], 2)) - (ddq_max[i] * (pow(ddq_max[i], 2) / (2 * Jm[i] * sigma[i]) + dq0[i])) / (Jm[i] * sigma[i]) - (ddq_max[i] * dq0[i]) / (Jm[i] * sigma[i]);
					lamda = b*b - 4 * a*c;
					// ta2 doesn't exist. 
					if (lamda < -minimt)
					{						
						// set ta4 = 0. recompute the ddq_max.

						ta4[i] = 0;
						a = (Ti[i]) / (2 * Jm[i] * sigma[i]);
						b = -(pow(Ti[i], 2)) / 2;
						c = -dq0[i] * Ti[i] - q0[i] + qf[i];
						slamda = b*b - 4 * a*c;

						if (slamda >minimt)
						{
							snewT[0] = -b / (2 * a) + sqrt(slamda) / (2 * a);
							snewT[1] = -b / (2 * a) - sqrt(slamda) / (2 * a);
							if ((snewT[0] > 0 && snewT[1] > 0))
							{
								// 首先取解1 
								if (sigma[i] == 1)
								{
									if (snewT[0] > snewT[1])
										ddq_max[i] = snewT[1];
									else
										ddq_max[i] = snewT[0];
								}
							}
							else if ((snewT[0] < 0 && snewT[1] < 0))
							{
								// 首先取解1 
								if (sigma[i] == -1)
								{
									if (snewT[0] > snewT[1])
										ddq_max[i] = snewT[0];
									else
										ddq_max[i] = snewT[1];
								}
							}
							else if (snewT[0] > 0 && snewT[1] < 0)
							{
								if (sigma[i] == 1)
								{
									ddq_max[i] = snewT[0];
								}
								else if (sigma[i] == -1)
								{
									ddq_max[i] = snewT[1];
								}
							}
							else if (snewT[1] > 0 && snewT[0] < 0)
							{
								if (sigma[i] == 1)
								{
									ddq_max[i] = snewT[1];
								}
								else if (sigma[i] == -1)
								{
									ddq_max[i] = snewT[0];
								}
							}
							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);

							ta2[i] = Ti[i] - ta1[i] - ta3[i] - ta4[i];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];

							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
						else if ((slamda < -minimt) || (slamda > -minimt && slamda < minimt))
						{
							// 给出连立ta1和ta2方程后 三次方程的各项系数  求修正后的ta1值
					
							Jm[i] = -(qf[i] - q0[i] - Ti[i] *dq0[i])/((-(pow(Ti[i],3) * sigma[i]) / 8));
							ddq_max[i]= Ti[i] *sigma[i] *Jm[i] / 2;
							
							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);

							ta2[i] = 0;
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ta4[i] = 0;
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];

							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];

						}
						
					}
					// ta2 exists. compute ta2
					else
					{

						newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a); //求出的二次项根1
						newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a); //求出的二次项根2

						if ((newT[0] >0 && newT[1] >0)) // 求出两个正根 取短的时间代入计算
						{
							if (newT[0] > newT[1]) // ta2取时间较小的值
								ta2[i] = newT[1];
							else
								ta2[i] = newT[0];
							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ta4[i] = Ti[i] - ta1[i] - ta2[i] - ta3[i];
						}
						else if ((newT[0] > 0 && newT[1] < 0)) // 取正的时间值
						{
							ta2[i] = newT[0];
							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ta4[i] = Ti[i] - ta1[i] - ta2[i] - ta3[i];
						}
						else if ((newT[1] > 0 && newT[0] < 0)) // 取正的时间值
						{
							ta2[i] = newT[1];
							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ta4[i] = Ti[i] - ta1[i] - ta2[i] - ta3[i];
						}
						else if ((newT[0] < 0 && newT[1] < 0))
						{
							ta4[i] = 0;
						}


						// ta4 doesn't exists. recompute ddq_max.
						if ((ta4[i] <-minimt) || (ta4[i]>-minimt && ta4[i]<minimt))
						{
							ta4[i] = 0;
							a = -(Ti[i]) / (2 * Jm[i] * sigma[i]);
							b = (pow(Ti[i], 2)) / 2;
							c = dq0[i] * Ti[i] + q0[i] - qf[i];
							slamda = b*b - 4 * a*c;

							snewT[0] = -b / (2 * a) + sqrt(slamda) / (2 * a);
							snewT[1] = -b / (2 * a) - sqrt(slamda) / (2 * a);
							if ((snewT[0] > 0 && snewT[1] > 0))
							{
								// 首先取解1 刷新ddq_max
								if (sigma[i] == 1)
								{
									if (snewT[0] > snewT[1])
										ddq_max[i] = snewT[1];
									else
										ddq_max[i] = snewT[0];
								}
							}
							else if ((snewT[0] < 0 && snewT[1] < 0))
							{
								// 首先取解1 刷新ddq_max
								if (sigma[i] == -1)
								{
									if (snewT[0] > snewT[1])
										ddq_max[i] = snewT[0];
									else
										ddq_max[i] = snewT[1];
								}
							}
							else if (snewT[0] > 0 && snewT[1] < 0)
							{
								if (sigma[i] == 1)
								{
									ddq_max[i] = snewT[0];
								}
								else if (sigma[i] == -1)
								{
									ddq_max[i] = snewT[1];
								}
							}
							else if (snewT[1] > 0 && snewT[0] < 0)
							{
								if (sigma[i] == 1)
								{
									ddq_max[i] = snewT[1];
								}
								else if (sigma[i] == -1)
								{
									ddq_max[i] = snewT[0];
								}
							}
							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
							
							ta2[i] = Ti[i] - ta1[i] - ta3[i] - ta4[i];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];

							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
						else //ta4 exists.
						{
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];

							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
				}
				

				//刷新总时间
				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
				if ((ta1[i] + ta2[i] + ta3[i] + ta4[i])<0)
				{
					printf("ta error\n");
					return 0x002100B00A40003; //总时间计算错误
				}
				//刷新各段时间
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
				//刷新各段时间的对应三次方程的系数
				if ((ta1[i] >minimt) || (ta1[i]>-minimt && ta1[i]<minimt))
				{
					coeff[i][0][0] = 1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][0][1] = 1 / 2.0 * ddq0[i];
					coeff[i][0][2] = dq0[i];
					coeff[i][0][3] = q0[i];
				}
				if ((ta2[i] >minimt) || (ta2[i]>-minimt && ta2[i]<minimt))
				{
					coeff[i][1][0] = 0;
					coeff[i][1][1] = 1 / 2.0 * ddq1[i];
					coeff[i][1][2] = dq1[i];
					coeff[i][1][3] = q1[i];
				}
				if ((ta3[i] >minimt) || (ta3[i]>-minimt && ta3[i]<minimt))
				{
					coeff[i][2][0] = -1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][2][1] = 1 / 2.0 * ddq2[i];
					coeff[i][2][2] = dq2[i];
					coeff[i][2][3] = q2[i];
				}
				if((ta4[i] >minimt) || (ta4[i]>-minimt && ta4[i]<minimt))
				{
					coeff[i][3][0] = 0;
					coeff[i][3][1] = 0;
					coeff[i][3][2] = dq3[i];
					coeff[i][3][3] = q3[i];
				}
			}
			// steady state.
			else if (sigmaq[i] == 0)
			{
				ta1[i] = 0; ta2[i] = 0; ta3[i] = 0; ta4[i] =Ti[i];				
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				ddq2[i] = ddq1[i];
				dq2[i] = dq1[i] + ddq1[i] * ta2[i];
				q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

				ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
				dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
				q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

				ddq4[i] = 0;
				dq4[i] = dq3[i];
				q4[i] = q3[i] + ta4[i] * dq3[i];
				tqf[i] = q4[i];
				dqf[i] = dq4[i];
				ddqf[i] = ddq4[i];
				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];

				//刷新各段时间
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
				//刷新各段时间的对应三次方程的系数

				if ((ta1[i] >minimt) || (ta1[i]>-minimt && ta1[i]<minimt))
				{
					coeff[i][0][0] = 1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][0][1] = 1 / 2.0 * ddq0[i];
					coeff[i][0][2] = dq0[i];
					coeff[i][0][3] = q0[i];
				}
				if ((ta2[i] >minimt) || (ta2[i]>-minimt && ta2[i]<minimt))
				{
					coeff[i][1][0] = 0;
					coeff[i][1][1] = 1 / 2.0 * ddq1[i];
					coeff[i][1][2] = dq1[i];
					coeff[i][1][3] = q1[i];
				}
				if ((ta3[i] >minimt) || (ta3[i]>-minimt && ta3[i]<minimt))
				{
					coeff[i][2][0] = -1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][2][1] = 1 / 2.0 * ddq2[i];
					coeff[i][2][2] = dq2[i];
					coeff[i][2][3] = q2[i];
				}
				if ((ta4[i] >minimt) || (ta4[i]>-minimt && ta4[i]<minimt))
				{
					coeff[i][3][0] = 0;
					coeff[i][3][1] = 0;
					coeff[i][3][2] = dq3[i];
					coeff[i][3][3] = q3[i];
				}

			}
		}
		//maxT = max(tempT); //求tempT数组中的最大值 并返回
		*maxT = -1;
		for (int i = 0; i < 6; i++)
		{
			if (*maxT < tempT[i])
			{
				*maxT = tempT[i];
			}

		}
	}
	else if (type == 2)// slowdown  逐点减速段
	{
		for (int i = 0; i < dofn; i++)
		{
			//根据关节的运动方向 判断Jerk的符号变量
			if (qf[i] - q0[i] > minimq)
			{
				sigmaq[i] = 1; //正转  符号取1
			}
			else if (qf[i] - q0[i] < -minimq)
			{
				sigmaq[i] = -1; //反转 符号取 - 1
			}
			else if ((qf[i] - q0[i] < minimq) && (qf[i] - q0[i]>-minimq))
			{
				sigmaq[i] = 0;	//静止 符号取0
			}
			deltav[i] = dq0[i] - maxdq[i];
			if (deltav[i]> minimv)
			{
				sigma[i] = -1; //减速 符号取 - 1
			}
			else if (deltav[i]   < -minimv)
			{
				sigma[i] = 1; //加速  符号取1
			}
			else if ((deltav[i] < minimv) && (deltav[i]>-minimv))
			{
				deltav[i] = 0;
				sigma[i] = -1;	//匀速 符号取0
			}

			if (sigmaq[i] != 0 && (Ti[i] >1 - minimt && Ti[i]<1 + minimt)) // 关节存在运动 且只需计算各轴同步运动时间 进入下面计算
			{
				if (sigma[i] == -1)
				{
					ddq_max[i] = maxddq[i];	//加速度赋值
				}
				else if (sigma[i] == 1)
				{
					ddq_max[i] = minddq[i];
				}

				
				dq_avg[i] = maxdq[i]; //平均速度赋值


				ta2[i] = (-deltav[i] - ddq_max[i] * ddq_max[i] / (sigma[i] * Jm[i])) / (ddq_max[i]);

				// ta2 doesn't exist! set ta2 = 0, recompute ddq_max by dq_avg
				if (ta2[i] > 0)
				{
					ta2[i] = 0;
					if (sigma[i] == 1)
						ddq_max[i] = -sqrt((-deltav[i])*sigma[i] * Jm[i]);
					else if (sigma[i] == -1)
						ddq_max[i] = sqrt((-deltav[i])*sigma[i] * Jm[i]);
				}
				// with new ddq_max recompute the ta1, ta3 and ta4

				ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				ddq2[i] = ddq1[i];
				dq2[i] = dq1[i] + ddq1[i] * ta2[i];
				q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

				ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
				ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
				dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
				q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

				ta4[i] = (q0[i] - q3[i]) / dq3[i];

				if (ta4[i] < 0)
				{
					ddq4[i] = 0;
					dq4[i] = dq3[i];
					q4[i] = q3[i] + ta4[i] * dq3[i];

					tqf[i] = q4[i];
					dqf[i] = dq4[i];
					ddqf[i] = ddq4[i];
				}
				// if qf < q3, ta4 doesn't exist. dq_avg cann't be reached. set ta4 == 0. 
				else if (ta4[i] > 0)
				{
					if (ta2[i] < minimt)
					{
						a = (ddq_max[i] / 2);
						b = (dq0[i] + (3 * ddq_max[i] * ddq_max[i]) / (2 * Jm[i] * sigma[i]));
						c = -q0[i] + qf[i] + pow(ddq_max[i], 3) / (2 * Jm[i] * Jm[i] * sigma[i] * sigma[i]) + (ddq_max[i] * (ddq_max[i] * ddq_max[i] / (2 * Jm[i] * sigma[i]) + dq0[i])) / (Jm[i] * sigma[i]) + (ddq_max[i] * dq0[i]) / (Jm[i] * sigma[i]);
						lamda = b*b - 4 * a*c;
						// ta2 doesn't exist. set ta2 == 0, recompute ddq_max.
						if (lamda < 0)
						{
							// 给出连立ta1和ta2方程后 三次方程的各项系数  求修正后的ta1值
							a3 = 1; //a
							a2 = 0; //b
							a1 = (2 * dq0[i] * Jm[i] * sigma[i]); //2*Jm*dq0*sigma
							a0 = (qf[i] - q0[i])*Jm[i] * Jm[i] * sigma[i] * sigma[i]; //Jm^2*q0*sigma^2 - Jm^2*qf*sigma^2

							int snum = 0;
							double tempt = 0; //获得全部的解
							getrootsofquadratic(a3, a2, a1, a0, r, &snum); //计算一元三次方程的根

							if (snum == 1)
							{
								tempt = r[0];
							}
							else
							{

								for (int j = 0; j < 3; j++)
								{
									if (sigma[i] ==- 1)
									{
										tempt = 99;
										if (r[j]<tempt && r[j]>0) //根据要求去掉负时间，取最小时间
										{
											tempt = r[j];
										}
									}
									else if (sigma[i] == 1)
									{
										tempt = -99;
										if (r[j] >tempt && r[j] <0)
										{
											tempt = r[j];
										}
									}
								}
							}
							ddq_max[i] = tempt;

							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ta2[i] = 0;
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ta4[i] = 0;
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];

							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];

						}
						// ta2 exists. 
						else
						{
							newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a); //求出的二次项根1
							newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a); //求出的二次项根2

							if ((newT[0] < 0 && newT[1] < 0)) // 求出两个正根 取短的时间代入计算
							{
								if (newT[0] > newT[1]) // ta2取时间较小的值
									ta2[i] = newT[0];
								else
									ta2[i] = newT[1];

								//重新计算ta2后 更新加速度 速度和位置
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								ta4[i] = 0;
								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] + ta4[i] * dq3[i];

								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}
							else if ((newT[0] > 0 && newT[1] < 0)) // take negative value
							{
								ta2[i] = newT[1];
								//重新计算ta2后 更新加速度 速度和位置
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								ta4[i] = 0;
								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] + ta4[i] * dq3[i];

								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}
							else if ((newT[1] > 0 && newT[0] < 0)) // take negative value
							{
								ta2[i] = newT[0];
								//重新计算ta2后 更新加速度 速度和位置
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								ta4[i] = 0;
								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] + ta4[i] * dq3[i];

								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}
							else if ((newT[1] > 0 && newT[0] > 0))
							{
								ta2[i] = 0;
								a3 = -1/ (pow(Jm[i],2) * pow(sigma[i],2)); //a
								a2 = 0; //b
								a1 = -(2 * dq0[i]) / (Jm[i] *sigma[i]); //2*Jm*dq0*sigma
								a0 = q0[i] - qf[i]; //Jm^2*q0*sigma^2 - Jm^2*qf*sigma^2

								int snum = 0;
								double tempt = 0; //获得全部的解
								getrootsofquadratic(a3, a2, a1, a0, r, &snum); //计算一元三次方程的根

								if (snum == 1)
								{
									tempt = r[0];
								}
								else
								{

									for (int j = 0; j < 3; j++)
									{
										if (sigma[i] == -1)
										{
											tempt = 999;
											if (r[j]<tempt && r[j]>0) //根据要求去掉负时间，取最小时间
											{
												tempt = r[j];
											}
										}
										else if (sigma[i] == 1)
										{
											tempt = -999;
											if (r[j] > tempt && r[j] <0)
											{
												tempt = r[j];
											}
										}
									}
								}
								ddq_max[i] = tempt;

								ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

								ta2[i] = 0;
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								ta4[i] = 0;
								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] + ta4[i] * dq3[i];

								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}

						}


					}
					else if (ta2[i] > -minimt && ta2[i] < minimt)
					{
						a3 = 1; //a
						a2 = 0; //b
						a1 = (2 * dq0[i] * Jm[i] * sigma[i]); //2*Jm*dq0*sigma
						a0 = (qf[i] - q0[i])*Jm[i] * Jm[i] * sigma[i] * sigma[i]; //Jm^2*q0*sigma^2 - Jm^2*qf*sigma^2

						int snum = 0;
						double tempt = 0; //获得全部的解
						getrootsofquadratic(a3, a2, a1, a0, r, &snum); //计算一元三次方程的根

						if (snum == 1)
						{
							tempt = r[0];
						}
						else
						{

							for (int j = 0; j < 3; j++)
							{
								if (sigma[i] == -1)
								{
									tempt = 99;
									if (r[j]<tempt && r[j]>0) //根据要求去掉负时间，取最小时间
									{
										tempt = r[j];
									}
								}
								else if (sigma[i] == 1)
								{
									tempt = -99;
									if (r[j] > tempt && r[j] <0)
									{
										tempt = r[j];
									}
								}
							}
						}
						ddq_max[i] = tempt;

						ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
						ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
						dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
						q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

						ta2[i] = 0;
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						ta4[i] = 0;
						ddq4[i] = 0;
						dq4[i] = dq3[i];
						q4[i] = q3[i] + ta4[i] * dq3[i];

						tqf[i] = q4[i];
						dqf[i] = dq4[i];
						ddqf[i] = ddq4[i];
					}

				}
				else if (ta4[i] > -minimt && ta4[i] < minimt)
				{
					ddq4[i] = 0;
					dq4[i] = dq3[i];
					q4[i] = q3[i] + ta4[i] * dq3[i];

					tqf[i] = q4[i];
					dqf[i] = dq4[i];
					ddqf[i] = ddq4[i];
				}


				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
				//计算总时间
				//各段时间赋值
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
				if ((ta1[i] + ta2[i] + ta3[i] + ta4[i]) > minimt)
				{
					printf("ta error\n");
					return 0x002100B00A40003; //总时间计算错误
				}
				//各段系数赋值
				if ((ta1[i] <-minimt) || (ta1[i]>-minimt && ta1[i]<minimt))
				{
					coeff[i][0][0] = 1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][0][1] = 1 / 2.0 * ddq0[i];
					coeff[i][0][2] = dq0[i];
					coeff[i][0][3] = qf[i];
				}

				if ((ta2[i] <-minimt) || (ta2[i]>-minimt && ta2[i]<minimt))
				{
					coeff[i][1][0] = 0;
					coeff[i][1][1] = 1 / 2.0 * ddq1[i];
					coeff[i][1][2] = dq1[i];
					coeff[i][1][3] = q1[i];
				}
				if ((ta3[i] <-minimt) || (ta3[i]>-minimt && ta3[i]<minimt))
				{
					coeff[i][2][0] = -1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][2][1] = 1 / 2.0 * ddq2[i];
					coeff[i][2][2] = dq2[i];
					coeff[i][2][3] = q2[i];
				}
				if ((ta4[i] <-minimt) || (ta4[i]>-minimt && ta4[i]<minimt))
				{
					coeff[i][3][0] = 0;
					coeff[i][3][1] = 0;
					coeff[i][3][2] = dq3[i];
					coeff[i][3][3] = q3[i];
				}
			}
			else if (sigmaq[i] != 0 && Ti[i] < -minimt)// 这一段是根据多轴时间同步后 计算各段的时间系数和多项式系数 并返回结束的位置速度 加速度
			{
				if (sigma[i] == 1)
					ddq_max[i] = minddq[i];	//加速度赋值
				else if (sigma[i] == -1)
					ddq_max[i] = maxddq[i];


				dq_avg[i] = maxdq[i]; //平均速度赋值

				if (((Ti[i] - (old_Ta[i][0] + old_Ta[i][1] + old_Ta[i][2] + old_Ta[i][3]))<minimt) && ((Ti[i] - (old_Ta[i][0] + old_Ta[i][1] + old_Ta[i][2] + old_Ta[i][3]))>-minimt))
				{

					Ta[i][0] = old_Ta[i][0]; Ta[i][1] = old_Ta[i][1]; Ta[i][2] = old_Ta[i][2]; Ta[i][3] = old_Ta[i][3];
					tempT[i] = Ta[i][0] + Ta[i][1] + Ta[i][2] + Ta[i][3];
					coeff[i][0][0] = old_coeff[i][0][0]; coeff[i][0][1] = old_coeff[i][0][1]; coeff[i][0][2] = old_coeff[i][0][2]; coeff[i][0][3] = old_coeff[i][0][3];
					coeff[i][1][0] = old_coeff[i][1][0]; coeff[i][1][1] = old_coeff[i][1][1]; coeff[i][1][2] = old_coeff[i][1][2]; coeff[i][1][3] = old_coeff[i][1][3];
					coeff[i][2][0] = old_coeff[i][2][0]; coeff[i][2][1] = old_coeff[i][2][1]; coeff[i][2][2] = old_coeff[i][2][2]; coeff[i][2][3] = old_coeff[i][2][3];
					coeff[i][3][0] = old_coeff[i][3][0]; coeff[i][3][1] = old_coeff[i][3][1]; coeff[i][3][2] = old_coeff[i][3][2]; coeff[i][3][3] = old_coeff[i][3][3];

					tqf[i] = old_tqf[i];
					dqf[i] = old_dqf[i];
					ddqf[i] = old_ddqf[i];
					continue;
				}

				dq_avg_new[i] = (qf[i] - q0[i]) / fabs(Ti[i]);

				deltav[i] = dq0[i] - dq_avg_new[i];
				if (deltav[i]> 0)
				{
					sigma[i] = -1; //减速 符号取 - 1
				}
				else if (deltav[i]  < 0)
				{
					sigma[i] = 1; //加速  符号取1
				}
				else if ((deltav[i]< minimv) && (deltav[i]>-minimv))
				{
					deltav[i] = 0;
					sigma[i] = 1;	//匀速 符号取0

				}

				if (sigma[i] == 1)
					ddq_max[i] = minddq[i];	//加速度赋值
				else if (sigma[i] == -1)
					ddq_max[i] = maxddq[i];


				if ((deltav[i] < minimv) && (deltav[i] > -minimv))
				{
					ta1[i] = 0;
					ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
					dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
					q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

					ta2[i] = 0;
					ddq2[i] = ddq1[i];
					dq2[i] = dq1[i] + ddq1[i] * ta2[i];
					q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

					ta3[i] = 0;
					ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
					dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
					q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

					ta4[i] = Ti[i] - ta1[i] - ta2[i] - ta3[i];
					ddq4[i] = 0;
					dq4[i] = dq3[i];
					q4[i] = q3[i] + ta4[i] * dq3[i];

					tqf[i] = q4[i];
					dqf[i] = dq4[i];
					ddqf[i] = ddq4[i];
				}
				// recompute ta2 with Ti = ta1 + ta2 + ta3 + ta4
				else
				{
					a = (ddq_max[i] / 2);
					b = (-ddq_max[i] * (Ti[i] - (2 * ddq_max[i]) / (Jm[i] * sigma[i])) - pow(ddq_max[i], 2) / (2 * Jm[i] * sigma[i]));
					c = q0[i] - qf[i] - (pow(ddq_max[i], 2) / (Jm[i] * sigma[i]) + dq0[i])*(Ti[i] - (2 * ddq_max[i]) / (Jm[i] * sigma[i])) - pow(ddq_max[i], 3) / (2 * pow(Jm[i], 2) * pow(sigma[i], 2)) - (ddq_max[i] * (pow(ddq_max[i], 2) / (2 * Jm[i] * sigma[i]) + dq0[i])) / (Jm[i] * sigma[i]) - (ddq_max[i] * dq0[i]) / (Jm[i] * sigma[i]);
					lamda = b*b - 4 * a*c;
					// ta2 doesn't exist. 
					if (lamda < 0)
					{
						// 给出连立ta1和ta2方程后 三次方程的各项系数  求修正后的ta1值
						// set ta4 = 0. recompute the ddq_max.
						ta4[i] = 0;
						a = -(Ti[i]) / (2 * Jm[i] * sigma[i]);
						b = (pow(Ti[i], 2)) / 2;
						c = dq0[i] * Ti[i] + qf[i] - q0[i];
						slamda = b*b - 4 * a*c;

						if ((slamda > minimt) || ( slamda>-minimt && slamda<minimt))
						{
							snewT[0] = -b / (2 * a) + sqrt(slamda) / (2 * a);
							snewT[1] = -b / (2 * a) - sqrt(slamda) / (2 * a);
							if ((snewT[0] > 0 && snewT[1] > 0))
							{
								// 首先取解1 
								if (sigma[i] == -1)
								{
									if (snewT[0] > snewT[1])
										ddq_max[i] = snewT[1];
									else
										ddq_max[i] = snewT[0];
								}
							}
							else if ((snewT[0] < 0 && snewT[1] < 0))
							{
								// 首先取解1 
								if (sigma[i] == 1)
								{
									if (snewT[0] > snewT[1])
										ddq_max[i] = snewT[0];
									else
										ddq_max[i] = snewT[1];
								}
							}
							else if (snewT[0] > 0 && snewT[1] < 0)
							{
								if (sigma[i] == -1)
								{
									ddq_max[i] = snewT[0];
								}
								else if (sigma[i] == 1)
								{
									ddq_max[i] = snewT[1];
								}
							}
							else if (snewT[1] > 0 && snewT[0] < 0)
							{
								if (sigma[i] == -1)
								{
									ddq_max[i] = snewT[1];
								}
								else if (sigma[i] == 1)
								{
									ddq_max[i] = snewT[0];
								}
							}
							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);

							ta2[i] = Ti[i] - ta1[i] - ta3[i] - ta4[i];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];

							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
						else
						{

							// 给出连立ta1和ta2方程后 三次方程的各项系数  求修正后的ta1值

							Jm[i] = -(q0[i] - qf[i] - Ti[i] * dq0[i]) / ((-(pow(Ti[i], 3) * sigma[i]) / 8));
							ddq_max[i] = Ti[i] * sigma[i] * Jm[i] / 2;

							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);

							ta2[i] = 0;
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ta4[i] = 0;
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];

							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];

						}

					}
					// ta2 exists. compute ta2
					else
					{

						newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a); //求出的二次项根1
						newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a); //求出的二次项根2

						if ((newT[0] <0 && newT[1] <0)) // 求出两个正根 取短的时间代入计算
						{
							if (newT[0] > newT[1]) // ta2取时间较小的值
								ta2[i] = newT[0];
							else
								ta2[i] = newT[1];
							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ta4[i] = Ti[i] - ta1[i] - ta2[i] - ta3[i];
						}
						else if ((newT[0] > 0 && newT[1] < 0)) // 取正的时间值
						{
							ta2[i] = newT[1];
							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ta4[i] = Ti[i] - ta1[i] - ta2[i] - ta3[i];
						}
						else if ((newT[1] > 0 && newT[0] < 0)) // 取正的时间值
						{
							ta2[i] = newT[0];
							ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
							ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
							dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
							q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							ta4[i] = Ti[i] - ta1[i] - ta2[i] - ta3[i];
						}
						else if ((newT[0] > 0 && newT[1] > 0)) // 求出两个正根 取短的时间代入计算
						{
							ta4[i] = 0;
						}
						
						// ta4 doesn't exists. recompute ddq_max.
						if ((ta4[i] > 0) || (ta4[i]>-minimt && ta4[i]<minimt))
						{
							ta4[i] = 0;
							a = -(Ti[i]) / (2 * Jm[i] * sigma[i]);
							b = (pow(Ti[i], 2)) / 2;
							c = dq0[i] * Ti[i] + qf[i] - q0[i];
							slamda = b*b - 4 * a*c;

							if (slamda > 0)
							{
								snewT[0] = -b / (2 * a) + sqrt(slamda) / (2 * a);
								snewT[1] = -b / (2 * a) - sqrt(slamda) / (2 * a);
								if ((snewT[0] > 0 && snewT[1] > 0))
								{
									// 首先取解1 刷新ddq_max
									if (sigma[i] == -1)
									{
										if (snewT[0] > snewT[1])
											ddq_max[i] = snewT[1];
										else
											ddq_max[i] = snewT[0];
									}
								}
								else if ((snewT[0] < 0 && snewT[1] < 0))
								{
									// 首先取解1 刷新ddq_max
									if (sigma[i] == 1)
									{
										if (snewT[0] > snewT[1])
											ddq_max[i] = snewT[0];
										else
											ddq_max[i] = snewT[1];
									}
								}
								else if (snewT[0] > 0 && snewT[1] < 0)
								{
									if (sigma[i] == -1)
									{
										ddq_max[i] = snewT[0];
									}
									else if (sigma[i] == 1)
									{
										ddq_max[i] = snewT[1];
									}
								}
								else if (snewT[1] > 0 && snewT[0] < 0)
								{
									if (sigma[i] == -1)
									{
										ddq_max[i] = snewT[1];
									}
									else if (sigma[i] == 1)
									{
										ddq_max[i] = snewT[0];
									}
								}
								ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

								ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);

								ta2[i] = Ti[i] - ta1[i] - ta3[i] - ta4[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] + ta4[i] * dq3[i];

								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}
							else
							{
								Jm[i] = -(q0[i] - qf[i] - Ti[i] *dq0[i]) / (-pow(Ti[i],3) * sigma[i] / 8);
								ddq_max[i] = Ti[i] *sigma[i] *Jm[i] / 2;

								ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

								ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);

								ta2[i] = 0;
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] + ta4[i] * dq3[i];

								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}
						}
						else //ta4 exists.
						{
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];

							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
				}
				
				
				//刷新总时间
				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
				//刷新各段时间
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
				
				if ((ta1[i] + ta2[i] + ta3[i] + ta4[i]) > minimt)
				{
					printf("ta error\n");
					return 0x002100B00A40003; //总时间计算错误
				}
				//刷新各段时间的对应三次方程的系数
				if ((ta1[i] <-minimt) || (ta1[i]>-minimt && ta1[i]<minimt))
				{
					coeff[i][0][0] = 1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][0][1] = 1 / 2.0 * ddq0[i];
					coeff[i][0][2] = dq0[i];
					coeff[i][0][3] = qf[i];
				}
				if ((ta2[i] <-minimt) || (ta2[i]>-minimt && ta2[i]<minimt))
				{
					coeff[i][1][0] = 0;
					coeff[i][1][1] = 1 / 2.0 * ddq1[i];
					coeff[i][1][2] = dq1[i];
					coeff[i][1][3] = q1[i];
				}
				if ((ta3[i] <-minimt) || (ta3[i]>-minimt && ta3[i]<minimt))
				{
					coeff[i][2][0] = -1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][2][1] = 1 / 2.0 * ddq2[i];
					coeff[i][2][2] = dq2[i];
					coeff[i][2][3] = q2[i];
				}
				if ((ta4[i] <-minimt) || (ta4[i]>-minimt && ta4[i]<minimt))
				{
					coeff[i][3][0] = 0;
					coeff[i][3][1] = 0;
					coeff[i][3][2] = dq3[i];
					coeff[i][3][3] = q3[i];
				}
			}
			else if (sigmaq[i] == 0)
			{
				ta1[i] = 0; ta2[i] = 0; ta3[i] = 0; ta4[i] = Ti[i];
				sigma[i] = 0;
				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = qf[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				ddq2[i] = ddq1[i];
				dq2[i] = dq1[i] + ddq1[i] * ta2[i];
				q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

				ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
				dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
				q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);
				
				ddq4[i] = 0;
				dq4[i] = dq3[i];
				q4[i] = q3[i] + ta4[i] * dq3[i];
				tqf[i] = q4[i];
				dqf[i] = dq4[i];
				ddqf[i] = ddq4[i];
				
				//刷新各段时间
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
				//刷新各段时间的对应三次方程的系数

				if ((ta1[i] <-minimt) || (ta1[i]>-minimt && ta1[i]<minimt))
				{
					coeff[i][0][0] = 1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][0][1] = 1 / 2.0 * ddq0[i];
					coeff[i][0][2] = dq0[i];
					coeff[i][0][3] = qf[i];
				}
				if ((ta2[i] <-minimt) || (ta2[i]>-minimt && ta2[i]<minimt))
				{
					coeff[i][1][0] = 0;
					coeff[i][1][1] = 1 / 2.0 * ddq1[i];
					coeff[i][1][2] = dq1[i];
					coeff[i][1][3] = q1[i];
				}
				if ((ta3[i] <-minimt) || (ta3[i]>-minimt && ta3[i]<minimt))
				{
					coeff[i][2][0] = -1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][2][1] = 1 / 2.0 * ddq2[i];
					coeff[i][2][2] = dq2[i];
					coeff[i][2][3] = q2[i];
				}
				if ((ta4[i] <-minimt) || (ta4[i]>-minimt && ta4[i]<minimt))
				{
					coeff[i][3][0] = 0;
					coeff[i][3][1] = 0;
					coeff[i][3][2] = dq3[i];
					coeff[i][3][3] = q3[i];
				}

			}

		}
		*maxT = 1;
		for (int i = 0; i < 6; i++)
		{
			if (*maxT > tempT[i])
			{
				*maxT = tempT[i];
			}

		}
	}

	return 0;
}

//------------------------------------------------------
// 名称：forwardCycle
// 功能：逐点加速正向迭代计算；
// 输入：start - 起始点的位置、速度、加速度
//       target - 终末点的位置
//       exp_duration - 期望运行时间
//       alpha_upper - 正向角加速度最大值
//       alpha_lower - 反向角加速度最大值
// 输出：segment - 本段轨迹的分段表达式系数及时间
//------------------------------------------------------
ErrorCode forwardCycle(const fst_mc::JointPoint &start, const fst_mc::Joint &target, double exp_duration,
	const fst_mc::Joint &alpha_upper, const fst_mc::Joint &alpha_lower, const fst_mc::Joint &jerk,
	fst_mc::TrajSegment(&segment)[NUM_OF_JOINT])
{
	double J0[6] = { 0 };// { -0.266252049150925, -0.600252808094322, 0.645899775801551, 0, -1.61644329450212, -0.266252049150925 };
	double J1[6] = { 0 };// { -0.258175866447821, -0.595379359200683, 0.638347237960886, 0, -1.6137642055551, -0.258175866447821 };

	double dq_avg[6] = {0};
	double ddq_max[6] = { 0 };// {
	double ddq_min[6] = { 0 };
	double dq_lim[6] = { -5.8119,
		-4.6600,
		-5.8119,
		-7.8540,
		-7.0686,
		-10.5592 };
	double time_adj[6] = { 0 };
	double maxduration = -1.0;

	J0[0] = start.angle.j1; J0[1] = start.angle.j2; J0[2] = start.angle.j3; J0[3] = start.angle.j4; J0[4] = start.angle.j5; J0[5] = start.angle.j6;
	J1[0] = target.j1; J1[1] = target.j2; J1[2] = target.j3; J1[3] = target.j4; J1[4] = target.j5; J1[5] = target.j6;
	for (int i = 0; i < 6; i++)
	{

		if ((J1[i] - J0[i]) > -minimq && (J1[i] - J0[i]) < minimq)
			dq_avg[i] = 0;
		else
			dq_avg[i] = (J1[i] - J0[i]) / exp_duration;

		if (fabs(dq_avg[i]) > fabs(dq_lim[i]))
		{
			time_adj[i] = fabs((J1[i] - J0[i]) / dq_lim[i]);
		}
		else
		{
			time_adj[i] = exp_duration;
		}

		if (time_adj[i] > maxduration)
		{
			maxduration = time_adj[i];
		}
	}

	for (int i = 0; i < 6; i++)
	{

		if ((J1[i] - J0[i]) > -minimq && (J1[i] - J0[i]) < minimq)
			dq_avg[i] = 0;
		else
			dq_avg[i] = (J1[i] - J0[i]) / maxduration;

	}

	double dq0[6] = { 0 };
	double ddq0[6] = { 0 };
	double jk[6] = { 0 };
	for (int i = 0; i < 6; i++)
	{
			switch (i)
			{
			case 0:
				ddq_max[i] = alpha_upper.j1;
				ddq_min[i] = alpha_lower.j1;
				dq0[i] = start.omega.j1;
				ddq0[i] = start.alpha.j1;
				jk[i] = jerk.j1;
				break;
			case 1:
				ddq_max[i] = alpha_upper.j2;
				ddq_min[i] = alpha_lower.j2;
				dq0[i] = start.omega.j2;
				ddq0[i] = start.alpha.j2;
				jk[i] = jerk.j2;
				break;
			case 2:
				ddq_max[i] = alpha_upper.j3;
				ddq_min[i] = alpha_lower.j3;
				dq0[i] = start.omega.j3;
				ddq0[i] = start.alpha.j3;
				jk[i] = jerk.j3;
				break;
			case 3:
				ddq_max[i] = alpha_upper.j4;
				ddq_min[i] = alpha_lower.j4;
				dq0[i] = start.omega.j4;
				ddq0[i] = start.alpha.j4;
				jk[i] = jerk.j4;
				break;
			case 4:
				ddq_max[i] = alpha_upper.j5;
				ddq_min[i] = alpha_lower.j5;
				dq0[i] = start.omega.j5;
				ddq0[i] = start.alpha.j5;
				jk[i] = jerk.j5;
				break;
			case 5:
				ddq_max[i] = alpha_upper.j6;
				ddq_min[i] = alpha_lower.j6;
				dq0[i] = start.omega.j6;
				ddq0[i] = start.alpha.j6;
				jk[i] = jerk.j6;
				break;

			}
	}


	double Ti[6] = { -1,-1,-1,-1,-1,-1 };
	double Ta[6][4] = { 0 };
	double coeff[6][4][4] = { 0 };
	double oTa[6][4] = { 0 };
	double ocoeff[6][4][4] = { 0 };
	double maxT = 0;
	double tqf[6] = { 0 };
	double dqf[6] = { 0 };
	double ddqf[6] = { 0 };
	double otqf[6] = { 0 };
	double odqf[6] = { 0 };
	double oddqf[6] = { 0 };
	ErrorCode ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max, ddq_min,jk, Ti,ocoeff,oTa,otqf,odqf,oddqf, 1, Ta, coeff, &maxT, tqf, dqf, ddqf);

	printf("maxT=%f\n", maxT);

	for (int i = 0; i < 6; i++)
	{
		Ti[i] = maxT;
		oTa[i][0] = Ta[i][0]; oTa[i][1] = Ta[i][1]; oTa[i][2] = Ta[i][2]; oTa[i][3] = Ta[i][3];
		ocoeff[i][0][0] = coeff[i][0][0]; ocoeff[i][0][1] = coeff[i][0][1]; ocoeff[i][0][2] = coeff[i][0][2]; ocoeff[i][0][3] = coeff[i][0][3];
		ocoeff[i][1][0] = coeff[i][1][0]; ocoeff[i][1][1] = coeff[i][1][1]; ocoeff[i][1][2] = coeff[i][1][2]; ocoeff[i][1][3] = coeff[i][1][3];
		ocoeff[i][2][0] = coeff[i][2][0]; ocoeff[i][2][1] = coeff[i][2][1]; ocoeff[i][2][2] = coeff[i][2][2]; ocoeff[i][2][3] = coeff[i][2][3];
		ocoeff[i][3][0] = coeff[i][3][0]; ocoeff[i][3][1] = coeff[i][3][1]; ocoeff[i][3][2] = coeff[i][3][2]; ocoeff[i][3][3] = coeff[i][3][3];
		otqf[i] = tqf[i];
		odqf[i] = dqf[i];
		oddqf[i] = ddqf[i];
	}
	ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max,ddq_min, jk, Ti,ocoeff,oTa,otqf,odqf,oddqf, 1, Ta, coeff, &maxT, tqf, dqf, ddqf);

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j<4; j++)
		{
			segment[i].duration[j] = Ta[i][j];
			for (int k = 0; k < 4; k++)
			{
				segment[i].coeff[j][k] = coeff[i][j][3-k];
			}
		}


		//printf("joint:%d Ta=%f %f %f %f\n", i, Ta[i][0], Ta[i][1], Ta[i][2], Ta[i][3]);
		//printf("coeff:%f %f %f %f -- %f %f %f %f -- %f %f %f %f -- %f %f %f %f\n", coeff[i][0][0], coeff[i][0][1], coeff[i][0][2], coeff[i][0][3], coeff[i][1][0], coeff[i][1][1], coeff[i][1][2], coeff[i][1][3], coeff[i][2][0], coeff[i][2][1], coeff[i][2][2], coeff[i][2][3], coeff[i][3][0], coeff[i][3][1], coeff[i][3][2], coeff[i][3][3]);
		//printf("each joint time:%f\n", maxT);
		//printf("dest status(q,dq,ddq):%f %f %f\n", tqf[i], dqf[i], ddqf[i]);
	}

	return ret;
}

//------------------------------------------------------
// 名称：backwardCycle
// 功能：逐点加速反向迭代计算；
// 输入：start - 起始点的位置
//       target - 终末点的位置、速度、加速度
//       exp_duration - 期望运行时间
//       alpha_upper - 正向角加速度最大值
//       alpha_lower - 反向角加速度最大值
// 输出：segment - 本段轨迹的分段表达式系数及时间
//------------------------------------------------------
ErrorCode backwardCycle(const fst_mc::Joint &start, const fst_mc::JointPoint &target, double exp_duration,
	const fst_mc::Joint &alpha_upper, const fst_mc::Joint &alpha_lower, const fst_mc::Joint &jerk,
	fst_mc::TrajSegment(&segment)[NUM_OF_JOINT])
{
	double J0[6] = { 0 };// { -0.266252049150925, -0.600252808094322, 0.645899775801551, 0, -1.61644329450212, -0.266252049150925 };
	double J1[6] = { 0 };// { -0.258175866447821, -0.595379359200683, 0.638347237960886, 0, -1.6137642055551, -0.258175866447821 };

	double dq_avg[6];
	double ddq_max[6] = { 0 };// {
	double ddq_min[6] = { 0 };
	double dq_lim[6] = { -5.8119,
		- 4.6600,
		- 5.8119,
		- 7.8540,
		- 7.0686,
		- 10.5592 };
	double time_adj[6] = { 0 };
	double maxduration = -1.0;
	J1[0] = target.angle.j1; J1[1] = target.angle.j2; J1[2] = target.angle.j3; J1[3] = target.angle.j4; J1[4] = target.angle.j5; J1[5] = target.angle.j6;
	J0[0] = start.j1; J0[1] = start.j2; J0[2] = start.j3; J0[3] = start.j4; J0[4] = start.j5; J0[5] = start.j6;
	for (int i = 0; i < 6; i++)
	{

		if ((J1[i] - J0[i]) > -minimq && (J1[i] - J0[i]) < minimq)
			dq_avg[i] = 0;
		else
			dq_avg[i] = (J1[i] - J0[i]) / exp_duration;

		if (fabs(dq_avg[i]) > fabs(dq_lim[i]))
		{
			time_adj[i] =fabs( (J1[i] - J0[i]) / dq_lim[i]);
		}
		else
		{
			time_adj[i] = exp_duration;
		}

		if (time_adj[i] > maxduration)
		{
			maxduration = time_adj[i];
		}
	}

	for (int i = 0; i < 6; i++)
	{

		if ((J1[i] - J0[i]) > -minimq && (J1[i] - J0[i]) < minimq)
			dq_avg[i] = 0;
		else
			dq_avg[i] = (J1[i] - J0[i]) / maxduration;

	}

	double dq0[6] = { 0 };
	double ddq0[6] = { 0 };
	double jk[6] = { 0 };
	for (int i = 0; i < 6; i++)
	{
			switch (i)
			{
			case 0:
				ddq_max[i] = alpha_upper.j1;
				ddq_min[i] = alpha_lower.j1;
				dq0[i] = target.omega.j1;
				ddq0[i] = target.alpha.j1;
				jk[i] = jerk.j1;
				break;
			case 1:
				ddq_max[i] = alpha_upper.j2;
				ddq_min[i] = alpha_lower.j2;
				dq0[i] = target.omega.j2;
				ddq0[i] = target.alpha.j2;
				jk[i] = jerk.j2;
				break;
			case 2:
				ddq_max[i] = alpha_upper.j3;
				ddq_min[i] = alpha_lower.j3;
				dq0[i] = target.omega.j3;
				ddq0[i] = target.alpha.j3;
				jk[i] = jerk.j3;
				break;
			case 3:
				ddq_max[i] = alpha_upper.j4;
				ddq_min[i] = alpha_lower.j4;
				dq0[i] = target.omega.j4;
				ddq0[i] = target.alpha.j4;
				jk[i] = jerk.j4;
				break;
			case 4:
				ddq_max[i] = alpha_upper.j5;
				ddq_min[i] = alpha_lower.j5;
				dq0[i] = target.omega.j5;
				ddq0[i] = target.alpha.j5;
				jk[i] = jerk.j5;
				break;
			case 5:
				ddq_max[i] = alpha_upper.j6;
				ddq_min[i] = alpha_lower.j6;
				dq0[i] = target.omega.j6;
				ddq0[i] = target.alpha.j6;
				jk[i] = jerk.j6;
				break;

			}
	}

	double Ti[6] = { 1,1,1,1,1,1 };
	double Ta[6][4] = { 0 };
	double coeff[6][4][4] = { 0 };
	double oTa[6][4] = { 0 };
	double ocoeff[6][4][4] = { 0 };
	double maxT = 0;
	double tqf[6] = { 0 };
	double dqf[6] = { 0 };
	double ddqf[6] = { 0 };
	double otqf[6] = { 0 };
	double odqf[6] = { 0 };
	double oddqf[6] = { 0 };
	ErrorCode ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max,ddq_min, jk, Ti,ocoeff,oTa,otqf,odqf,oddqf, 2, Ta, coeff, &maxT, tqf, dqf, ddqf);

	printf("maxT=%f\n", maxT);

	for (int i = 0; i < 6; i++)
	{
		Ti[i] = maxT;
		oTa[i][0] = Ta[i][0]; oTa[i][1] = Ta[i][1]; oTa[i][2] = Ta[i][2]; oTa[i][3] = Ta[i][3];
		ocoeff[i][0][0] = coeff[i][0][0]; ocoeff[i][0][1] = coeff[i][0][1]; ocoeff[i][0][2] = coeff[i][0][2]; ocoeff[i][0][3] = coeff[i][0][3];
		ocoeff[i][1][0] = coeff[i][1][0]; ocoeff[i][1][1] = coeff[i][1][1]; ocoeff[i][1][2] = coeff[i][1][2]; ocoeff[i][1][3] = coeff[i][1][3];
		ocoeff[i][2][0] = coeff[i][2][0]; ocoeff[i][2][1] = coeff[i][2][1]; ocoeff[i][2][2] = coeff[i][2][2]; ocoeff[i][2][3] = coeff[i][2][3];
		ocoeff[i][3][0] = coeff[i][3][0]; ocoeff[i][3][1] = coeff[i][3][1]; ocoeff[i][3][2] = coeff[i][3][2]; ocoeff[i][3][3] = coeff[i][3][3];
	
		otqf[i] = tqf[i];
		odqf[i] = dqf[i];
		oddqf[i] = ddqf[i];
	
	}
	ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max,ddq_min, jk, Ti,ocoeff,oTa,otqf,odqf,oddqf, 2, Ta, coeff, &maxT, tqf, dqf, ddqf);

	double cef[4][4];
	double newcoeff[4][4];
	double tm[4];
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			segment[i].duration[j] = fabs(Ta[i][3 - j]);
			tm[j] = Ta[i][j];
			for (int k = 0; k < 4; k++)
			{
				cef[j][k] = coeff[i][j][3 - k];
			}
		}
		coefficient_transform(cef, tm, newcoeff);
		for (int j = 0; j < 4; j++)
		{
			for (int k = 0; k < 4; k++)
			{
				segment[i].coeff[j][k] = newcoeff[j][ k];
			}
		}
		//printf("joint:%d Ta=%f %f %f %f\n", i, Ta[i][0], Ta[i][1], Ta[i][2], Ta[i][3]);
		//printf("coeff:%f %f %f %f -- %f %f %f %f -- %f %f %f %f -- %f %f %f %f\n", coeff[i][0][0], coeff[i][0][1], coeff[i][0][2], coeff[i][0][3], coeff[i][1][0], coeff[i][1][1], coeff[i][1][2], coeff[i][1][3], coeff[i][2][0], coeff[i][2][1], coeff[i][2][2], coeff[i][2][3], coeff[i][3][0], coeff[i][3][1], coeff[i][3][2], coeff[i][3][3]);
		//printf("each joint time:%f\n", maxT);
		//printf("dest status(q,dq,ddq):%f %f %f\n", tqf[i], dqf[i], ddqf[i]);
	}

	return ret;

}
ErrorCode smoothPoint2Point(const JointPoint &start, const JointPoint &target, double exp_duration,
	const Joint &alpha_upper, const Joint &alpha_lower, const Joint &jerk,
	TrajSegment(&segment)[NUM_OF_JOINT])
{
	double J0[6] = { 0 };// { -0.266252049150925, -0.600252808094322, 0.645899775801551, 0, -1.61644329450212, -0.266252049150925 };
	double J1[6] = { 0 };// { -0.258175866447821, -0.595379359200683, 0.638347237960886, 0, -1.6137642055551, -0.258175866447821 };

	double dq_avg[6];
	double ddq_max[6] = { 0 };// {
	double ddq_min[6] = { 0 };

	J0[0] = start.angle.j1; J0[1] = start.angle.j2; J0[2] = start.angle.j3; J0[3] = start.angle.j4; J0[4] = start.angle.j5; J0[5] = start.angle.j6;
	J1[0] = target.angle.j1; J1[1] = target.angle.j2; J1[2] = target.angle.j3; J1[3] = target.angle.j4; J1[4] = target.angle.j5; J1[5] = target.angle.j6;
	for (int i = 0; i < 6; i++)
	{
		if ((J1[i] - J0[i]) > -minimq && (J1[i] - J0[i]) < minimq)
			dq_avg[i] = 0;
		else
			dq_avg[i] = (J1[i] - J0[i]) / exp_duration;
	}

	double dq0[6] = { 0 };
	//double ddq0[6] = { 0 };
	double dqf[6] = { 0 };
	double ddqf[6] = { 0 };
	double jk[6] = { 0 };

	for (int i = 0; i < 6; i++)
	{
		switch (i)
		{
		case 0:
			ddq_max[i] = alpha_upper.j1;
			ddq_min[i] = alpha_lower.j1;
			dq0[i] = start.omega.j1;
			dqf[i] = target.omega.j1;
			jk[i] = jerk.j1;
			break;
		case 1:
			ddq_max[i] = alpha_upper.j2;
			ddq_min[i] = alpha_lower.j2;
			dq0[i] = start.omega.j2;
			dqf[i] = target.omega.j2;
			jk[i] = jerk.j2;
			break;
		case 2:
			ddq_max[i] = alpha_upper.j3;
			ddq_min[i] = alpha_lower.j3;
			dq0[i] = start.omega.j3;
			dqf[i] = target.omega.j3;
			jk[i] = jerk.j3;
			break;
		case 3:
			ddq_max[i] = alpha_upper.j4;
			ddq_min[i] = alpha_lower.j4;
			dq0[i] = start.omega.j4;
			dqf[i] = target.omega.j4;
			jk[i] = jerk.j4;
			break;
		case 4:
			ddq_max[i] = alpha_upper.j5;
			ddq_min[i] = alpha_lower.j5;
			dq0[i] = start.omega.j5;
			dqf[i] = target.omega.j5;
			jk[i] = jerk.j5;
			break;
		case 5:
			ddq_max[i] = alpha_upper.j6;
			ddq_min[i] = alpha_lower.j6;
			dq0[i] = start.omega.j6;
			dqf[i] = target.omega.j6;
			jk[i] = jerk.j6;
			break;

		}
	}


	double maxT = exp_duration;

	ErrorCode ret = 0;

	double a0[MAXAXES];
	double a1[MAXAXES];
	double a2[MAXAXES];
	double a3[MAXAXES];

	int i;

	for (i = 0; i<MAXAXES; i++)
	{
		a0[i] = J0[i]; //6*1 vector
		a1[i] = dq0[i];
		a2[i] = 3 * (J1[i] - J0[i]) / pow(maxT, 2) - (2 * dq0[i] + dqf[i]) / maxT;
		a3[i] = 2 * (J0[i] - J1[i]) / pow(maxT, 3) + (dq0[i] + dqf[i]) / pow(maxT, 2);
	}

	for (int i = 0; i < 6; i++)
	{
		memset(segment[i].duration, 0, sizeof(segment[i].duration));
		memset(segment[i].coeff, 0, sizeof(segment[i].coeff));
	}
	for (int i = 0; i < 6; i++)
	{
		segment[i].duration[0] = maxT;
		segment[i].coeff[0][0] = a0[i];
		segment[i].coeff[0][1] = a1[i];
		segment[i].coeff[0][2] = a2[i];
		segment[i].coeff[0][3] = a3[i];
	}

	return ret;


}





























































































































































                          
ErrorCode computeDynamics(const fst_mc::Joint &angle, const fst_mc::Joint &omega,
  fst_mc::Joint &alpha_upper, fst_mc::Joint &alpha_lower, fst_mc::DynamicsProduct &product)
{
  FP tq[MAXAXES];
  FP tdq[MAXAXES];
  FP tddq[2][MAXAXES];
  FP tcg[MAXAXES];
  
  for (int i = 0; i < MAXAXES; i++)
  {
      switch (i)
      {
      case 0:
          tq[i] = angle.j1;
          tdq[i] = omega.j1;

          break;
      case 1:
          tq[i] = angle.j2;
          tdq[i] = omega.j2;

          break;
      case 2:
          tq[i] = angle.j3;
          tdq[i] = omega.j3;

          break;
      case 3:
          tq[i] = angle.j4;
          tdq[i] = omega.j4;

          break;
      case 4:
          tq[i] = angle.j5;
          tdq[i] = omega.j5;

          break;
      case 5:
          tq[i] = angle.j6;
          tdq[i] = omega.j6;

          break;

      }
  }
  g_dynamics_interface.computeAccMax(tq,tdq, tddq);
  //getInertia(tq, tddq, tma, tcg);

  //rne_tau(tq, tdq, tddq, tau);
  FP mq[6][6], mdq[6][6], mddq[6][6];
  FP M[6][6],C[6][6];

  for (int i = 0; i<6; i++)
  {
      for (int j = 0; j<6; j++)
      {
          mq[j][i] = tq[j];

          mdq[j][i] = 0;
          if (i == j)
              mddq[j][i] = 1;
          else
              mddq[j][i] = 0;

      }
      //printf("%f %f %f %f %f %f\n", mddq[0][i], mddq[1][i], mddq[2][i], mddq[3][i], mddq[4][i], mddq[5][i]);
  }
  g_dynamics_interface.rne_M(mq, mdq, mddq, M);


  g_dynamics_interface.rne_C(tq, tdq, C);

  FP GRV[3] = { 0,0,9.81 };

  //printf("iq : %f %f %f %f %f %f\n",iq[0],iq[1],iq[2],iq[3],iq[4],iq[5]);
  g_dynamics_interface.rne_G(tq, GRV, tcg);

  for (int i = 0; i < MAXAXES; i++)
  {
      for (int j = 0; j < MAXAXES; j++)
      {
          product.m[i][j] = M[i][j];
          product.c[i][j] = C[i][j];
          
      }
      product.g[i] = tcg[i];
  }


  alpha_upper.j1 = tddq[0][0] > 6 ? tddq[0][0] : 6;
  alpha_upper.j2 = tddq[0][1] > 5 ? tddq[0][1] : 5;
  alpha_upper.j3 = tddq[0][2] > 8 ? tddq[0][2] : 8;
  alpha_upper.j4 = tddq[0][3] > 11 ? tddq[0][3] : 11;
  alpha_upper.j5 = tddq[0][4] > 14 ? tddq[0][4] : 14;
  alpha_upper.j6 = tddq[0][5] > 21 ? tddq[0][5] : 21;

  alpha_lower.j1 = tddq[1][0] < -6 ? tddq[1][0] : -6;
  alpha_lower.j2 = tddq[1][1] < -5 ? tddq[1][1] : -5;
  alpha_lower.j3 = tddq[1][2] < -8 ? tddq[1][2] : -8;
  alpha_lower.j4 = tddq[1][3] < -11 ? tddq[1][3] : -11;
  alpha_lower.j5 = tddq[1][4] < -14 ? tddq[1][4] : -14;
  alpha_lower.j6 = tddq[1][5] < -21 ? tddq[1][5] : -21;

  //alpha_upper.j2 *= 3;
  //alpha_upper.j3 *= 3;
  //alpha_lower.j2 *= 3;
  //alpha_lower.j3 *= 3;
  if (alpha_lower.j5 > -20) alpha_lower.j5 *= 10;
  if (alpha_upper.j5 < 20)  alpha_upper.j5 *= 10;

   return SUCCESS;
}
                          
#if 0
int main(void)
{
	int i = 0;

	double J0[6] = { 0.09,0.09,0.09,0.09,0.09,0.09 };
	double J1[6] = { 0.1,0.1,0.1,0.1,0.1,0.1 };

	double dq_avg[6];
	double ddq_max[6] = {
		124.238852,
		27.405693,
		16.749410,
		115.950513,
		341.373255,
		678.835059 };

	for (int i = 0; i < 6; i++)
	{
		dq_avg[i] = (J1[i] - J0[i]) / 0.055059;
	}

	double dq0[6] = { 0 };
	double ddq0[6] = { 0 };
	double Ti[6] = { -1,-1,-1,-1,-1,-1 };
	double Ta[6][4] = { 0 };
	double coeff[6][4][4] = { 0 };
	double maxT = 0;
	double tqf[6] = { 0 };
	double dqf[6] = { 0 };
	double ddqf[6] = { 0 };
	double jerk[6] = { 5.0*0.5 / 1.3 * pow(10, 4) * 81,
	3.3*0.4 / 0.44 * pow(10, 4) * 101,
	3.3*0.4 / 0.44 * pow(10, 4) * 81,
	1.7*0.39 / 0.18 * pow(10, 4) * 60,
	1.7*0.25 / 0.17 * pow(10, 4) * 66.66667,
	1.7*0.25 / 0.17 * pow(10, 4) * 44.64286 };

	int ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max, jerk, Ti, 2, Ta, coeff, &maxT, tqf, dqf, ddqf);

	printf("maxT=%f\n", maxT);

	for (int i = 0; i < 6; i++)
		Ti[i] = maxT;
	ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max, jerk, Ti, 2, Ta, coeff, &maxT, tqf, dqf, ddqf);

	//for (int i = 0; i < 6; i++)
	//{
	//printf("joint:%d Ta=%f %f %f %f\n", i, Ta[i][0], Ta[i][1], Ta[i][2], Ta[i][3]);
	//printf("coeff:%f %f %f %f -- %f %f %f %f -- %f %f %f %f -- %f %f %f %f\n", coeff[i][0][0], coeff[i][0][1], coeff[i][0][2], coeff[i][0][3], coeff[i][1][0], coeff[i][1][1], coeff[i][1][2], coeff[i][1][3], coeff[i][2][0], coeff[i][2][1], coeff[i][2][2], coeff[i][2][3], coeff[i][3][0], coeff[i][3][1], coeff[i][3][2], coeff[i][3][3]);
	//printf("each joint time:%f\n", maxT);
	//printf("dest status(q,dq,ddq):%f %f %f\n", tqf[i], dqf[i], ddqf[i]);

	//}
	printf("start-angle: %f\n", J0[0]);
	printf("ending-angle:%f\n", J1[0]);
	printf("ending-omega:%f\n", dq0[0]);
	printf("ending-alpha:%f\n", ddq0[0]);
	printf("duration = %f %f %f %f\n", Ta[0][0], Ta[0][1], Ta[0][2], Ta[0][3]);
	printf("coeff=%f %f %f %f\n", coeff[0][0][0], coeff[0][0][1], coeff[0][0][2], coeff[0][0][3]);
	printf("coeff=%f %f %f %f\n", coeff[0][1][0], coeff[0][1][1], coeff[0][1][2], coeff[0][1][3]);
	printf("coeff=%f %f %f %f\n", coeff[0][2][0], coeff[0][2][1], coeff[0][2][2], coeff[0][2][3]);
	printf("coeff=%f %f %f %f\n", coeff[0][3][0], coeff[0][3][1], coeff[0][3][2], coeff[0][3][3]);
	getchar();
	return 0;
}

#endif
