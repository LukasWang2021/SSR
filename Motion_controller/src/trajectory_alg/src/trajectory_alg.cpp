#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "base_datatype.h"
#include "error_code.h"
#include "trajectory_alg.h"
#include "dynamics_interface.h"

extern fst_algorithm::DynamicsInterface g_dynamics_interface;

ErrorCode forwardCycle(const fst_mc::JointPoint &start, const fst_mc::Joint &target, double exp_duration,
	const fst_mc::Joint &alpha_upper, const fst_mc::Joint &alpha_lower, const fst_mc::Joint &jerk,
	fst_mc::TrajSegment(&segment)[NUM_OF_JOINT]);
ErrorCode backwardCycle(const fst_mc::Joint &start, const fst_mc::JointPoint &target, double exp_duration,
	const fst_mc::Joint &alpha_upper, const fst_mc::Joint &alpha_lower, const fst_mc::Joint &jerk,
	fst_mc::TrajSegment(&segment)[NUM_OF_JOINT]);

using namespace fst_mc;
using namespace std;

/*
getrootofquadratic：求一元三次方程的根
输入参数：a：三次项的系数 b：二次项的系数 c：一次项的系数 d：常数项的系数
输出参数：Sv[3]  为方程的三个根 某些特殊情况下 仅仅有一个根 有两个复根（忽略）
*/
int getrootsofquadratic(double a, double b, double c, double d, double Sv[3])
{
	double A, B, C, L, K, T, rad;
	double GS1, GS2, Y1, Y2, P1, P2;
	double X1, X2, X3;

	A = b*b - 3 * a*c;
	B = b*c - 9 * a*d;
	C = c*c - 3 * b*d;

	L = B*B - 4 * A*C;

	if (A == 0 && B == 0)
	{
		X1 = X2 = X3 = -b / (3 * a);
		Sv[0] = X1; Sv[1] = X2; Sv[2] = X3;
	}
	else
	{
		if (L > 0)
		{
			Y1 = A*b + (3 * a*(-B + sqrt(L))) / 2;
			Y2 = A*b + (3 * a*(-B - sqrt(L))) / 2;

			if (Y1 > 0)
			{
				GS1 = pow(Y1, 1.0 / 3);
			}
			else
			{
				GS1 = -pow(fabs(Y1), 1.0 / 3);
			}

			if (Y2 > 0)
			{
				GS2 = pow(Y2, (1.0 / 3));
			}
			else
			{
				GS2 = -pow(fabs(Y2), (1.0 / 3));
			}
			P1 = (-2 * b + GS1 + GS2) / (6 * a);
			P2 = sqrt(3)*(GS1 - GS2) / (6 * a);
			X1 = (-b - GS1 - GS2) / (3 * a);
			Sv[0] = X1;
			printf("X1=%.3f\n", X1);
			printf("X2=%.3f+%.3fi\n", P1, P2);
			printf("X3=%.3f-%.3fi\n", P1, P2);
		}
		else if (L == 0)
		{
			K = B / A;
			X1 = -b / a + K;
			X2 = X3 = -K / 2;
			Sv[0] = X1; Sv[1] = X2; Sv[2] = X3;
		}
		else
		{
			T = (2 * A*b - 3 * a*B) / (2 * pow(A, 1.5));
			rad = acos(T);
			X1 = (-b - 2 * sqrt(A)*cos(rad / 3)) / (3 * a);
			X2 = (-b + sqrt(A)*(cos(rad / 3) + sqrt(3)*sin(rad / 3))) / (3 * a);
			X3 = (-b + sqrt(A)*(cos(rad / 3) - sqrt(3)*sin(rad / 3))) / (3 * a);
			Sv[0] = X1; Sv[1] = X2; Sv[2] = X3;
		}
	}

	return 0;
}
int calculateParam(double q0[MAXAXES], double dq0[MAXAXES], double ddq0[MAXAXES], double qf[MAXAXES], 
                        double maxdq[MAXAXES], double maxddq[MAXAXES], double Jm[MAXAXES], double Ti[MAXAXES], 
                        int type, double Ta[MAXAXES][4], double coeff[MAXAXES][4][4], double *maxT, double tqf[MAXAXES], 
                        double dqf[MAXAXES], double ddqf[MAXAXES])
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
	double ddq_max[MAXAXES], dq_avg[MAXAXES], sigma[MAXAXES]; //最大加速度 平均速度 总时间 jerk符号
	double tempT[MAXAXES] = { 0 };
	double newT[2] = { 0 };
	double r[3] = { 0 };

	double a, b, c, lamda, a3, a2, a1, a0, tempt, ta41;
	int dofn = MAXAXES;

	if (type == 1) // 加速计算
	{
		for (int i = 0; i < dofn; i++)
		{
			//根据关节的运动方向 判断Jerk的符号变量
			if (qf[i] - q0[i] > minim)
			{
				sigma[i] = 1; //正转  符号取1
			}
			else if (qf[i] - q0[i] < -minim)
			{
				sigma[i] = -1; //反转 符号取 - 1
			}
			else
			{
				sigma[i] = 0;	//静止 符号取0
			}

			if (sigma[i] != 0 && Ti[i] == -1) // 关节存在运动 且只需计算各轴同步运动时间 进入下面计算
			{
				ddq_max[i] = maxddq[i];	//加速度赋值
				dq_avg[i] = maxdq[i];		//平均速度赋值

											//time adjust
											//stage 1  -- - acc - acc
											//第一段计算  计算加加速段求加加速的时间ta1
											//求加加速的时间ta1 加速度ddq1 速度dq1 结束时刻位置 q1
				ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				//stage 2 -- - quansi - acc
				//第二段计算  计算匀加速段求匀加速的时间ta2
				//求匀加速的时间ta2 加速度ddq2 速度dq2 结束时刻位置 q2
				//需要提前计算第三段所需的时间ta3 以加到平均速度dq_avg作为条件来代入计算ta2
				ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
				ta2[i] = (dq_avg[i] - ddq_max[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2) - dq1[i]) / ddq_max[i];
				ddq2[i] = ddq1[i];
				dq2[i] = dq1[i] + ddq1[i] * ta2[i];
				q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

				//第二段计算的结束时刻位置如果超过下一点的位置 需要重新计算ta2
				if ((sigma[i] == 1 && q2[i] > qf[i]) || (sigma[i] == -1 && q2[i] < qf[i]))
				{
					// 给出修正二次项的各系数
					a = 1 / 2.0 * ddq1[i];
					b = dq1[i];
					c = -(qf[i] - q1[i]);
					lamda = pow(b, 2) - 4 * a*c;
					if (lamda < 0)
					{
						printf("signification erreur"); //二次项系数有误 表示给定的路径插值时间太短 至少小于900纳秒 报错
						return -1;
					}

					newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a); //求出的二次项根1
					newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a); //求出的二次项根2

					if (newT[0] > minim && newT[1] > minim) // 求出两个正根 取短的时间代入计算
					{
						if (newT[0] > newT[1]) // ta2取时间较小的值
						{
							ta2[i] = newT[1];
						}
						else
						{
							ta2[i] = newT[0];
						}
					}
					else if (newT[0] > minim && newT[1] < -minim) // 取正的时间值
					{
						ta2[i] = newT[0];
					}
					else if (newT[1] > minim && newT[0] < -minim) // 取正的时间值
					{
						ta2[i] = newT[1];
					}
					//重新计算ta2后 更新加速度 速度和位置
					ddq2[i] = ddq1[i];
					dq2[i] = dq1[i] + ddq1[i] * ta2[i];
					q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

					//更新出参
					tqf[i] = q2[i];
					dqf[i] = dq2[i];
					ddqf[i] = ddq2[i];

					//不存在第三和第四时间段
					ta3[i] = 0;
					ta4[i] = 0;

					//计算总时间
					tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];

					Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];  //各段时间赋值

																								 //各段系数赋值
					if (ta1[i] > 0)
					{
						coeff[i][0][0] = 1 / 6.0 * sigma[i] * Jm[i]; // 3阶
						coeff[i][0][1] = 1 / 2.0 * ddq0[i]; // 2阶
						coeff[i][0][2] = dq0[i]; // 1阶
						coeff[i][0][3] = q0[i]; // 0阶
					}
					if (ta2[i] > 0)
					{
						coeff[i][1][0] = 0;
						coeff[i][1][1] = 1 / 2.0 * ddq1[i];
						coeff[i][1][2] = dq1[i];
						coeff[i][1][3] = q1[i];
					}
					if (ta3[i] > 0)
					{
						coeff[i][2][0] = -1 / 6.0 * sigma[i] * Jm[i];
						coeff[i][2][1] = 1 / 2.0 * ddq2[i];
						coeff[i][2][2] = dq2[i];
						coeff[i][2][3] = q2[i];
					}
					if (ta4[i] > 0)
					{
						coeff[i][3][0] = 0;
						coeff[i][3][1] = 0;
						coeff[i][3][2] = dq3[i];
						coeff[i][3][3] = q3[i];
					}
					continue;
				}

				//stage 3 -- - dcc - acc
				//计算减加速段
				//减加速段的时间ta3在匀加速时已计算，刷新加速度，速度和位置值
				ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
				dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
				q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

				//判断减加速结束后的位置是否超过下一点位置 如果超出 重新计算时间ta3
				if ((sigma[i] == 1 && q3[i] > qf[i]) || (sigma[i] == -1 && q3[i] < qf[i]))
				{
					// 根据减加速段的一元三次方程计算调整后的根
					//首先获得各项的系数
					a3 = -1 / 6.0 * sigma[i] * Jm[i]; //a
					a2 = 1 / 2.0 * ddq2[i]; //b
					a1 = dq2[i]; //c
					a0 = -(qf[i] - q2[i]); //d

					getrootsofquadratic(a3, a2, a1, a0, r); //计算一元三次方程的根
					double tempt = 99; //获得全部的解
					for (int j = 0; j < 3; j++)
					{
						if (r[j]<tempt && r[j]>minim) //根据要求去掉负时间，取最小时间
						{
							tempt = r[j];
						}
					}

					ta3[i] = tempt;

					//重新刷新ta3的位置 速度 和加速度值
					ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
					dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
					q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

					//不存在第四时间段
					ta4[i] = 0;

					//更新该关节的结束的位置 速度 加速度
					tqf[i] = q3[i];
					dqf[i] = dq3[i];
					ddqf[i] = ddq3[i];

					//得到总时间
					tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
					Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];  //各项时间系数的刷新
																								 //各项三次项系数的刷新
					if (ta1[i] > 0)
					{
						coeff[i][0][0] = 1 / 6.0 * sigma[i] * Jm[i];
						coeff[i][0][1] = 1 / 2.0 * ddq0[i];
						coeff[i][0][2] = dq0[i];
						coeff[i][0][3] = q0[i];
					}
					if (ta2[i] > 0)
					{
						coeff[i][1][0] = 0;
						coeff[i][1][1] = 1 / 2.0 * ddq1[i];
						coeff[i][1][2] = dq1[i];
						coeff[i][1][3] = q1[i];
					}
					if (ta3[i] > 0)
					{
						coeff[i][2][0] = -1 / 6.0 * sigma[i] * Jm[i];
						coeff[i][2][1] = 1 / 2.0 * ddq2[i];
						coeff[i][2][2] = dq2[i];
						coeff[i][2][3] = q2[i];
					}
					if (ta4[i] > 0)
					{
						coeff[i][3][0] = 0;
						coeff[i][3][1] = 0;
						coeff[i][3][2] = dq3[i];
						coeff[i][3][3] = q3[i];
					}
					continue;
				}

				//stage 4 -- - quansi - velocity
				//第四段 匀速计算
				//计算匀速的ta4 以及位置速度 加速度值
				//首先判断减加速结束点是否小于目标位置（下一点位置） 小于 则存在匀速段
				if ((sigma[i] == 1 && q3[i] < qf[i]) || (sigma[i] == -1 && q3[i] > qf[i]))
				{
					// 得到匀速段的ta4 位置和速度 加速度值
					ta4[i] = (qf[i] - q3[i]) / dq3[i];
					ddq4[i] = 0;
					dq4[i] = dq3[i];
					q4[i] = q3[i] + ta4[i] * dq3[i];

					//更新返回的位置速度加速度值
					tqf[i] = q4[i];
					dqf[i] = dq4[i];
					ddqf[i] = ddq4[i];
				}

				//更新总时间
				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];

				//更新各项时间系数
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
				//更新各项的多项式系数
				if (ta1[i] > 0)
				{
					coeff[i][0][0] = 1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][0][1] = 1 / 2.0 * ddq0[i];
					coeff[i][0][2] = dq0[i];
					coeff[i][0][3] = q0[i];
				}
				if (ta2[i] > 0)
				{
					coeff[i][1][0] = 0;
					coeff[i][1][1] = 1 / 2.0 * ddq1[i];
					coeff[i][1][2] = dq1[i];
					coeff[i][1][3] = q1[i];
				}
				if (ta3[i] > 0)
				{
					coeff[i][2][0] = -1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][2][1] = 1 / 2.0 * ddq2[i];
					coeff[i][2][2] = dq2[i];
					coeff[i][2][3] = q2[i];
				}
				if (ta4[i] > 0)
				{
					coeff[i][3][0] = 0;
					coeff[i][3][1] = 0;
					coeff[i][3][2] = dq3[i];
					coeff[i][3][3] = q3[i];
				}
			}
			else if (sigma[i] != 0 && Ti[i] > 0) // 这一段是根据多轴时间同步后 计算各段的时间系数和多项式系数 并返回结束的位置速度 加速度
			{
				ddq_max[i] = maxddq[i];	//加速度赋值
				dq_avg[i] = maxdq[i];		//平均速度赋值

											//time adjust
											//stage 1  -- - acc - acc
											//计算加加速段求加加速的时间ta1 刷新位置速度加速度
				ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				//计算匀加速段ta2前 需要计算减加速时间ta3
				ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);

				//首先根据时间Ti的约束 求ta4的合适值
				//下面给出连立方程的二次项各项系数
				a = -1 / 2.0 * ddq1[i];
				b = 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2) + ddq1[i] * (Ti[i] - ta1[i] - ta3[i]);

				c = (Ti[i] - ta1[i] - ta3[i])*(ddq1[i] * ta3[i] + dq1[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2)) + q1[i] + dq1[i] * ta3[i] + 1 / 2.0 * ddq1[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3) - qf[i];
				lamda = pow(b, 2) - 4 * a*c; //lamda值

				if (lamda <= 0) // lamda值小于0 表示ta2无解 需要根据ta1 ta3 求解
				{
					//重新计算ta2 刷新位置速度加速度
					ta2[i] = Ti[i] - ta1[i] - ta3[i];
					ddq2[i] = ddq1[i];
					dq2[i] = dq1[i] + ddq1[i] * ta2[i];
					q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
					//如果匀加速后位置超出下一点位置 利用三次方程重新计算ta2的时间
					if ((sigma[i] == 1 && q2[i] > qf[i]) || (sigma[i] == -1 && q2[i] < qf[i]))
					{
						// 给出连立ta1和ta2方程后 三次方程的各项系数  求修正后的ta1值
						a3 = -1 / 3.0 * sigma[i] * Jm[i]; //a
						a2 = -1 / 2.0 * sigma[i] * Jm[i] * Ti[i] - 1 / 2.0 * ddq0[i]; //b
						a1 = 1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
						a0 = 1 / 2.0 * ddq0[i] * pow(Ti[i], 2) + dq0[i] * Ti[i] + q0[i] - qf[i]; //d

						getrootsofquadratic(a3, a2, a1, a0, r); //求三次方程的根
						tempt = 99;
						for (int j = 0; j < 3; j++)
						{
							if (r[j] < tempt && r[j] > minim) //求时间最小且为正的根
							{
								tempt = r[j];
							}
						}

						//根据方程的根 刷新ta1 以及位置速度 加速度
						ta1[i] = tempt;
						ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
						dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
						q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

						//重新计算ta2 刷新位置速度加速度
						ta2[i] = Ti[i] - ta1[i];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//不存在第三和第四时间段
						ta3[i] = 0;
						ta4[i] = 0;

						//更新出参
						tqf[i] = q2[i];
						dqf[i] = dq2[i];
						ddqf[i] = ddq2[i];
					}

					else //存在第三段ta3 q2小于qf
					{

						//刷新减加速结束后的位置速度加速度
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//不存在第四段时间
						ta4[i] = 0;
						//更新出参
						tqf[i] = q3[i];
						dqf[i] = dq3[i];
						ddqf[i] = ddq3[i];
					}
				}
				else //存在第四段 ta2  lamda大于0
				{
					newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a); //求出ta4的解1
					newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a); //求出ta4的解2

																	//如果两个时间解都为正 依次进行计算 并判断
					if (newT[0] > minim && newT[1] > minim)
					{

						//首先取解1 刷新ta2 刷新位置速度加速度
						ta2[i] = newT[0];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//根据ta3 刷新位置速度加速度
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//刷新ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];

						//如果计算的ta4 为负 表示取得解不合适 换解2 重新计算
						if (ta4[i] < 0)
						{
							// 换解2 计算位置速度加速度
							ta2[i] = newT[1];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							//计算ta3位置速度加速度
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							//计算ta4
							ta41 = Ti[i] - ta2[i] - ta1[i] - ta3[i];

							//如果两个解都不存在ta4 则去掉第四时间段 重新调整ta2
							if (ta41 < 0)
							{
								// 重新计算ta2 刷新位置速度加速度
								ta2[i] = Ti[i] - ta1[i] - ta3[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
								//如果q2的位置大于下一点位置 需要根据三次方程重新计算ta1和ta2
								if ((sigma[i] == 1 && q2[i] > qf[i]) || (sigma[i] == -1 && q2[i] < qf[i]))
								{
									// 建立三次方程
									a3 = -1 / 3.0 * sigma[i] * Jm[i]; //a
									a2 = -1 / 2.0 * sigma[i] * Jm[i] * Ti[i] - 1 / 2.0 * ddq0[i]; //b
									a1 = 1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
									a0 = 1 / 2.0 * ddq0[i] * pow(Ti[i], 2) + dq0[i] * Ti[i] + q0[i] - qf[i]; //d

									getrootsofquadratic(a3, a2, a1, a0, r); //求三次方程的根
									tempt = 99;
									for (int j = 0; j < 3; j++)
									{
										if (r[j]<tempt && r[j]>minim) 	//取最小的正时间的根
										{
											tempt = r[j];
										}
									}

									//刷新ta1 以及位置速度加速度
									ta1[i] = tempt;
									ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
									dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
									q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

									//刷新ta2 以及位置速度加速度
									ta2[i] = Ti[i] - ta1[i];
									ddq2[i] = ddq1[i];
									dq2[i] = dq1[i] + ddq1[i] * ta2[i];
									q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

									//不存在第三和第四时间段
									ta3[i] = 0;
									ta4[i] = 0;

									//刷新出参
									tqf[i] = q2[i];
									dqf[i] = dq2[i];
									ddqf[i] = ddq2[i];
								}
								else //匀加速后的位置不超过下一点位置 计算减加速段
								{
									//更新ta3段的位置速度加速度值
									ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
									dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
									q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

									//不存在第四时间段
									ta4[i] = 0;
									//更新出参
									tqf[i] = q3[i];
									dqf[i] = dq3[i];
									ddqf[i] = ddq3[i];
								}
							}
							else //ta4大于0存在第四段 刷新位置速度加速度 刷新出参
							{
								ta4[i] = ta41;
								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] + ta41*dq3[i];
								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}
						}
						else //ta4大于0存在第四段 刷新位置速度加速度 刷新出参
						{

							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];
							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
					else if (newT[0] > minim && newT[1] < -minim) // 解1 为正 解2为负 取解1
					{
						//刷新ta2 刷新位置速度加速度
						ta2[i] = newT[0];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//刷新ta3 和位置速度加速度
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//计算ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];
						if (ta4[i] < 0) // 如果ta4小于0 表示前面给的解不合适 重新计算
						{
							//去掉ta4 重新计算ta2 刷新位置速度加速度
							ta2[i] = Ti[i] - ta1[i] - ta3[i];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
							//如果q2的位置大于下一点位置 联立q1和q2的方程组 得到一元三次方程
							if ((sigma[i] == 1 && q2[i] > qf[i]) || (sigma[i] == -1 && q2[i] < qf[i]))
							{
								// 给出一元三次方程的系数
								a3 = -1 / 3.0 * sigma[i] * Jm[i]; //a
								a2 = -1 / 2.0 * sigma[i] * Jm[i] * Ti[i] - 1 / 2.0 * ddq0[i]; //b
								a1 = 1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
								a0 = 1 / 2.0 * ddq0[i] * pow(Ti[i], 2) + dq0[i] * Ti[i] + q0[i] - qf[i]; //d

								getrootsofquadratic(a3, a2, a1, a0, r); //求一元三次方程的根
								tempt = 99;
								for (int j = 0; j < 3; j++)
								{
									if (r[j] < tempt && r[j] > minim)	//求最小的正时间
									{
										tempt = r[j];
									}
								}
								//刷新ta1 刷新位置速度加速度
								ta1[i] = tempt;
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

								//刷新ta2 刷新位置速度加速度
								ta2[i] = Ti[i] - ta1[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								//不存在第三和第四时间段
								ta3[i] = 0;
								ta4[i] = 0;
								//刷新出参
								tqf[i] = q2[i];
								dqf[i] = dq2[i];
								ddqf[i] = ddq2[i];
							}
							else //q2小于qf 存在第三段
							{
								//                     ta3[i] = Ti[i] - ta1[i] - ta2[i];
								//刷新位置速度加速度
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								//不存在第四段时间
								ta4[i] = 0;
								//刷新出参
								tqf[i] = q3[i];
								dqf[i] = dq3[i];
								ddqf[i] = ddq3[i];
							}
						}
						else //存在第四段
						{
							//刷新位置速度加速度
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];
							//刷新出参
							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
					else if (newT[1] > 1e-6 && newT[0] < -1e-6) // 解2 为正 解1为负 取解2
					{
						//刷新ta2 刷新位置速度加速度
						ta2[i] = newT[1];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//刷新ta3 和位置速度加速度
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//计算ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];
						if (ta4[i] < 0) // 如果ta4小于0 表示前面给的解不合适 重新计算
						{
							//去掉ta4 重新计算ta2 刷新位置速度加速度
							ta2[i] = Ti[i] - ta1[i] - ta3[i];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
							//如果q2的位置大于下一点位置 联立q1和q2的方程组 得到一元三次方程
							if ((sigma[i] == 1 && q2[i] > qf[i]) || (sigma[i] == -1 && q2[i] < qf[i]))
							{
								// 给出一元三次方程的系数
								a3 = -1 / 3.0 * sigma[i] * Jm[i]; //a
								a2 = -1 / 2.0 * sigma[i] * Jm[i] * Ti[i] - 1 / 2.0 * ddq0[i]; //b
								a1 = 1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
								a0 = 1 / 2.0 * ddq0[i] * pow(Ti[i], 2) + dq0[i] * Ti[i] + q0[i] - qf[i]; //d

								getrootsofquadratic(a3, a2, a1, a0, r); //求一元三次方程的根
								tempt = 99;
								for (int j = 0; j < 3; j++)
								{
									if (r[j]<tempt && r[j]>minim)  //求最小的正时间
									{
										tempt = r[j];
									}
								}

								//刷新ta1 刷新位置速度加速度
								ta1[i] = tempt;
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

								//刷新ta2 刷新位置速度加速度
								ta2[i] = Ti[i] - ta1[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								//不存在第三和第四时间段
								ta3[i] = 0;
								ta4[i] = 0;
								//刷新出参
								tqf[i] = q2[i];
								dqf[i] = dq2[i];
								ddqf[i] = ddq2[i];
							}
							else	//q2小于qf 存在第三段
							{
								//刷新位置速度加速度
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);
								//不存在第四段时间
								ta4[i] = 0;
								//刷新出参
								tqf[i] = q3[i];
								dqf[i] = dq3[i];
								ddqf[i] = ddq3[i];
							}
						}
						else//存在第四段
						{
							//刷新位置速度加速度
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];
							//刷新出参
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
				//刷新各段时间的对应三次方程的系数
				if (ta1[i] > 0)
				{
					coeff[i][0][0] = 1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][0][1] = 1 / 2.0 * ddq0[i];
					coeff[i][0][2] = dq0[i];
					coeff[i][0][3] = q0[i];
				}
				if (ta2[i] > 0)
				{
					coeff[i][1][0] = 0;
					coeff[i][1][1] = 1 / 2.0 * ddq1[i];
					coeff[i][1][2] = dq1[i];
					coeff[i][1][3] = q1[i];
				}
				if (ta3[i] > 0)
				{
					coeff[i][2][0] = -1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][2][1] = 1 / 2.0 * ddq2[i];
					coeff[i][2][2] = dq2[i];
					coeff[i][2][3] = q2[i];
				}
				if (ta4[i] > 0)
				{
					coeff[i][3][0] = 0;
					coeff[i][3][1] = 0;
					coeff[i][3][2] = dq3[i];
					coeff[i][3][3] = q3[i];
				}
			}
			else if (sigma[i] == 0) // 静止的处理
			{
				ta1[i] = 0;
				ta2[i] = 0;
				ta3[i] = 0;
				ta4[i] = 0;
				tqf[i] = q0[i];
				dqf[i] = 0;
				ddqf[i] = 0;
				coeff[i][0][0] = 0;
				coeff[i][0][1] = 0;
				coeff[i][0][2] = 0;
				coeff[i][0][3] = q0[i];
				coeff[i][1][0] = 0;
				coeff[i][1][1] = 0;
				coeff[i][1][2] = 0;
				coeff[i][1][3] = q0[i];
				coeff[i][2][0] = 0;
				coeff[i][2][1] = 0;
				coeff[i][2][2] = 0;
				coeff[i][2][3] = q0[i];
				coeff[i][3][0] = 0;
				coeff[i][3][1] = 0;
				coeff[i][3][2] = 0;
				coeff[i][3][3] = q0[i];
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
			if (qf[i] - q0[i] > minim)  //正转  符号取1
			{
				sigma[i] = 1;
			}
			else if (qf[i] - q0[i] < -minim) //反转 符号取 - 1
			{
				sigma[i] = -1;
			}
			else
			{
				sigma[i] = 0; //静止 符号取0
			}
			if (sigma[i] != 0 && Ti[i] == -1) // 关节存在运动 且只需计算各轴同步运动时间 进入下面计算
			{
				ddq_max[i] = maxddq[i]; //加速度赋值
				dq_avg[i] = maxdq[i]; //平均速度赋值

									  //time adjust
									  //stage 1  -- - acc - acc
									  //第一段计算  计算加加速段求加加速的时间ta1
									  //求加加速的时间ta1 加速度ddq1 速度dq1 结束时刻位置 q1

				ta1[i] = (ddq0[i] - ddq_max[i]) / -(sigma[i] * Jm[i]);
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				//stage 2 -- - quansi - acc
				//第二段计算  计算匀加速段求匀加速的时间ta2
				//求匀加速的时间ta2 加速度ddq2 速度dq2 结束时刻位置 q2
				//需要提前计算第三段所需的时间ta3 以加到平均速度dq_avg作为条件来代入计算ta2

				ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
				ta2[i] = (dq_avg[i] - ddq_max[i] * ta3[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2) + dq1[i]) / ddq_max[i];
				ddq2[i] = ddq1[i];
				dq2[i] = dq1[i] + ddq1[i] * ta2[i];
				q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

				//第二段计算的结束时刻位置如果小于初始点的位置 需要重新计算ta2
				if ((sigma[i] == 1 && q2[i] < q0[i]) || (sigma[i] == -1 && q2[i] > q0[i]))
				{
					// 给出修正二次项的各系数
					a = -1 / 2.0 * ddq1[i];
					b = -dq1[i];
					c = (q1[i] - q0[i]);
					lamda = b*b - 4 * a*c;
					if (lamda < 0)
					{
						printf("signification erreur"); //二次项系数有误 表示给定的路径插值时间太短 至少小于900纳秒 报错
						return -1;
					}

					newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a); //求出的二次项根1
					newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a); //求出的二次项根2

					if (newT[0] > minim && newT[1] > minim) // 求出两个正根 取短的时间代入计算
					{
						if (newT[0] > newT[1]) // ta2取时间较小的值
						{
							ta2[i] = newT[0];
						}
						else
						{
							ta2[i] = newT[1];
						}
					}
					else if (newT[0] > minim && newT[1] < -minim) // 取正的时间值
					{
						ta2[i] = newT[0];
					}
					else if (newT[1] > 1e-6 && newT[0] < -1e-6) // 取正的时间值
					{
						ta2[i] = newT[1];
					}
					//重新计算ta2后 更新加速度 速度和位置
					ddq2[i] = ddq1[i];
					dq2[i] = dq1[i] + ddq1[i] * ta2[i];
					q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

					//不存在第三和第四时间段
					ta3[i] = 0;
					ta4[i] = 0;
					//更新出参
					tqf[i] = q2[i];
					dqf[i] = dq2[i];
					ddqf[i] = ddq2[i];
					tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
					//计算总时间
					//各段时间赋值
					Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
					//各段系数赋值
					if (ta1[i] > 0)
					{
						coeff[i][0][0] = -1 / 6.0 * sigma[i] * Jm[i];
						coeff[i][0][1] = -1 / 2.0 * ddq0[i];
						coeff[i][0][2] = -dq0[i];
						coeff[i][0][3] = qf[i];
					}

					if (ta2[i] > 0)
					{
						coeff[i][1][0] = 0;
						coeff[i][1][1] = -1 / 2.0 * ddq1[i];
						coeff[i][1][2] = -dq1[i];
						coeff[i][1][3] = q1[i];
					}
					if (ta3[i] > 0)
					{
						coeff[i][2][0] = 1 / 6.0 * sigma[i] * Jm[i];
						coeff[i][2][1] = -1 / 2.0 * ddq2[i];
						coeff[i][2][2] = -dq2[i];
						coeff[i][2][3] = q2[i];
					}
					if (ta4[i] > 0)
					{
						coeff[i][3][0] = 0;
						coeff[i][3][1] = 0;
						coeff[i][3][2] = -dq3[i];
						coeff[i][3][3] = q3[i];
					}
					continue;
				}

				//stage 3 -- - dcc - acc
				//计算减加速段
				//减加速段的时间ta3在匀加速时已计算，刷新加速度，速度和位置值

				ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
				dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
				q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

				//判断减加速结束后的位置是否小于初始点位置 如果小于 重新计算时间ta3
				if ((sigma[i] == 1 && q3[i] < q0[i]) || (sigma[i] == -1 && q3[i] > q0[i]))
				{
					// 根据减加速段的一元三次方程计算调整后的根
					//首先获得各项的系数

					a3 = 1 / 6.0 * sigma[i] * Jm[i]; //a
					a2 = -1 / 2.0 * ddq2[i]; //b
					a1 = -dq2[i]; //c
					a0 = q2[i] - q0[i]; //d

										//                     r = roots([a3 a2 a1 a0]);
					getrootsofquadratic(a3, a2, a1, a0, r);  //计算一元三次方程
					tempt = 99; //获得全部的解
					for (int j = 0; j < 3; j++)
					{
						if (r[j]<tempt && r[j]>minim) //根据要求去掉负时间，取最小时间
						{
							tempt = r[j];
						}
					}
					ta3[i] = tempt;

					//重新刷新ta3的位置 速度 和加速度值
					ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
					dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
					q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);
					//不存在第四时间段
					ta4[i] = 0;
					//更新该关节的结束的位置 速度 加速度
					tqf[i] = q3[i];
					dqf[i] = dq3[i];
					ddqf[i] = ddq3[i];
					//得到总时间
					tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
					//各项时间系数的刷新
					Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
					//各项三次项系数的刷新
					if (ta1[i] > 0)
					{
						coeff[i][0][0] = -1 / 6.0 * sigma[i] * Jm[i];
						coeff[i][0][1] = -1 / 2.0 * ddq0[i];
						coeff[i][0][2] = -dq0[i];
						coeff[i][0][3] = qf[i];
					}
					if (ta2[i] > 0)
					{
						coeff[i][1][0] = 0;
						coeff[i][1][1] = -1 / 2.0 * ddq1[i];
						coeff[i][1][2] = -dq1[i];
						coeff[i][1][3] = q1[i];
					}
					if (ta3[i] > 0)
					{
						coeff[i][2][0] = 1 / 6.0 * sigma[i] * Jm[i];
						coeff[i][2][1] = -1 / 2.0 * ddq2[i];
						coeff[i][2][2] = -dq2[i];
						coeff[i][2][3] = q2[i];
					}
					if (ta4[i] > 0)
					{
						coeff[i][3][0] = 0;
						coeff[i][3][1] = 0;
						coeff[i][3][2] = -dq3[i];
						coeff[i][3][3] = q3[i];
					}
					continue;
				}

				//stage 4 -- - quansi - velocity
				//第四段 匀速计算
				//计算匀速的ta4 以及位置速度 加速度值
				//首先判断减加速结束点是否大于目标位置（初始点位置） 大于 则存在匀速段

				if ((sigma[i] == 1 && q3[i] > q0[i]) || (sigma[i] == -1 && q3[i] < q0[i]))
				{
					// 得到匀速段的ta4 位置和速度 加速度值
					ta4[i] = (q3[i] - q0[i]) / dq3[i];
					ddq4[i] = 0;
					dq4[i] = dq3[i];
					q4[i] = q3[i] - ta4[i] * dq3[i];
					//更新返回的位置速度加速度值
					tqf[i] = q4[i];
					dqf[i] = dq4[i];
					ddqf[i] = ddq4[i];
				}

				//更新总时间
				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];

				//更新各项时间系数
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
				//更新各项的多项式系数
				if (ta1[i] > 0)
				{
					coeff[i][0][0] = -1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][0][1] = -1 / 2.0 * ddq0[i];
					coeff[i][0][2] = -dq0[i];
					coeff[i][0][3] = qf[i];
				}
				if (ta2[i] > 0)
				{
					coeff[i][1][0] = 0;
					coeff[i][1][1] = -1 / 2.0 * ddq1[i];
					coeff[i][1][2] = -dq1[i];
					coeff[i][1][3] = q1[i];
				}
				if (ta3[i] > 0)
				{
					coeff[i][2][0] = 1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][2][1] = -1 / 2.0 * ddq2[i];
					coeff[i][2][2] = -dq2[i];
					coeff[i][2][3] = q2[i];
				}
				if (ta4[i] > 0)
				{
					coeff[i][3][0] = 0;
					coeff[i][3][1] = 0;
					coeff[i][3][2] = -dq3[i];
					coeff[i][3][3] = q3[i];
				}
			}
			else if (sigma[i] != 0 && Ti[i] > 0)// 这一段是根据多轴时间同步后 计算各段的时间系数和多项式系数 并返回结束的位置速度 加速度
			{
				ddq_max[i] = maxddq[i]; //加速度赋值
				dq_avg[i] = maxdq[i]; //平均速度赋值

									  //time adjust
									  //stage 1  -- - acc - acc
									  //计算加加速段求加加速的时间ta1 刷新位置速度加速度
				ta1[i] = (ddq0[i] - ddq_max[i]) / -(sigma[i] * Jm[i]);
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				//计算匀加速段ta2前 需要计算减加速时间ta3
				ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);

				//首先根据时间Ti的约束 求ta4的合适值
				//下面给出连立方程的二次项各项系数

				a = 1 / 2.0 * ddq1[i];
				b = -1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2) - ddq1[i] * (Ti[i] - ta1[i] - ta3[i]);
				//                 b = 1 / 2 * sigma[i]*Jm[i]*ta3[i] ^ 2 + ddq1[i]*Ti[i];
				c = q1[i] - q0[i] - dq1[i] * ta3[i] - 1 / 2.0 * ddq1[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3) - dq1[i] * (Ti[i] - ta1[i] - ta3[i]) - ddq1[i] * ta3[i] * (Ti[i] - ta1[i] - ta3[i]) + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2) * (Ti[i] - ta1[i] - ta3[i]);
				//                 c = q0[i] + dq0[i]*ta1[i] + ta3[i]*((sigma[i]*Jm[i]*ta1[i] ^ 2) / 2 + ddq0[i]*ta1[i] + dq0[i]) + ta3[i] ^ 2 * (ddq0[i] / 2 + (sigma[i]*Jm[i]*ta1[i]) / 2) + (ddq0[i]*ta1[i] ^ 2) / 2 + (sigma[i]*Jm[i]*ta1[i] ^ 3) / 6 - (sigma[i]*Jm[i]*ta3[i] ^ 3) / 6 + Ti[i]*(dq0[i] + ddq0[i]*ta1[i] + (sigma[i]*Jm[i]*ta1[i] ^ 2) / 2 - (sigma[i]*Jm[i]*ta3[i] ^ 2) / 2 + ta3[i]*(ddq0[i] + sigma[i]*Jm[i]*ta1[i])) - qf[i];
				lamda = b*b - 4 * a*c;
				if (lamda <= 0)// lamda值小于0 表示ta2无解 需要根据ta1 ta3 求解
				{
					//重新计算ta2 刷新位置速度加速度
					ta2[i] = Ti[i] - ta1[i] - ta3[i];
					ddq2[i] = ddq1[i];
					dq2[i] = dq1[i] + ddq1[i] * ta2[i];
					q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
					//如果匀加速后位置超出下一点位置 利用三次方程重新计算ta2的时间
					if ((sigma[i] == 1 && q2[i] < q0[i]) || (sigma[i] == -1 && q2[i] > q0[i]))
					{
						// 给出连立ta1和ta2方程后 三次方程的各项系数  求修正后的ta1值
						a3 = 1 / 3.0 * sigma[i] * Jm[i]; //a
						a2 = 1 / 2.0 * sigma[i] * Jm[i] * Ti[i] + 1 / 2.0 * ddq0[i]; //b
						a1 = -1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
						a0 = -1 / 2.0 * ddq0[i] * pow(Ti[i], 2) - dq0[i] * Ti[i] + qf[i] - q0[i]; //d

																								  //                         r = roots([a3 a2 a1 a0]);
						getrootsofquadratic(a3, a2, a1, a0, r); //求三次方程的根
						tempt = 99;
						for (int j = 0; j < 3; j++)
						{
							if (r[j]<tempt && r[j]>minim) //求时间最小且为正的根
							{
								tempt = r[j];
							}
						}
						//根据方程的根 刷新ta1 以及位置速度 加速度
						ta1[i] = tempt;
						ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
						dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
						q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);
						//重新计算ta2 刷新位置速度加速度
						ta2[i] = Ti[i] - ta1[i];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
						//不存在第三和第四时间段
						ta3[i] = 0;
						ta4[i] = 0;
						//更新出参
						tqf[i] = q2[i];
						dqf[i] = dq2[i];
						ddqf[i] = ddq2[i];
					}
					else//存在第三段ta3 q2小于qf
					{
						//                     ta3[i] = Ti[i] - ta1[i] - ta2[i];
						//刷新减加速结束后的位置速度加速度
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//不存在第四段时间
						ta4[i] = 0;
						//更新出参
						tqf[i] = q3[i];
						dqf[i] = dq3[i];
						ddqf[i] = ddq3[i];
					}
				}
				else //存在第四段 ta2  lamda大于0
				{
					newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a);
					newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a);

					//如果两个时间解都为正 依次进行计算 并判断
					if ((newT[0] > minim && newT[1] > minim))
					{
						// if newT(1)>newT(2)
						// ta2[i] = newT(2);
						//                         else
						//首先取解1 刷新ta2 刷新位置速度加速度
						ta2[i] = newT[0];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//根据ta3 刷新位置速度加速度
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//刷新ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];
						//如果计算的ta4 为负 表示取得解不合适 换解2 重新计算
						if (ta4[i] < 0)
						{
							// 换解2 计算位置速度加速度
							ta2[i] = newT[1];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							//计算ta3位置速度加速度
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							//计算ta4
							ta41 = Ti[i] - ta2[i] - ta1[i] - ta3[i];

							//如果两个解都不存在ta4 则去掉第四时间段 重新调整ta2
							if (ta41 < 0)
							{
								// 重新计算ta2 刷新位置速度加速度
								ta2[i] = Ti[i] - ta1[i] - ta3[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
								//如果q2的位置大于下一点位置 需要根据三次方程重新计算ta1和ta2
								if ((sigma[i] == 1 && q2[i] < q0[i]) || (sigma[i] == -1 && q2[i] > q0[i]))
								{
									// 建立三次方程
									a3 = 1 / 3.0 * sigma[i] * Jm[i]; //a
									a2 = 1 / 2.0 * sigma[i] * Jm[i] * Ti[i] + 1 / 2.0 * ddq0[i]; //b
									a1 = -1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
									a0 = -1 / 2.0 * ddq0[i] * pow(Ti[i], 2) - dq0[i] * Ti[i] + qf[i] - q0[i]; //d


									getrootsofquadratic(a3, a2, a1, a0, r); //求三次方程的根
									tempt = 99;
									for (int j = 0; j < 3; j++)
									{
										if (r[j]<tempt && r[j]>minim) //取最小的正时间的根
										{
											tempt = r[j];
										}
									}

									//刷新ta1 以及位置速度加速度
									ta1[i] = tempt;
									ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
									dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
									q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

									//刷新ta2 以及位置速度加速度
									ta2[i] = Ti[i] - ta1[i];
									ddq2[i] = ddq1[i];
									dq2[i] = dq1[i] + ddq1[i] * ta2[i];
									q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

									//不存在第三和第四时间段
									ta3[i] = 0;
									ta4[i] = 0;
									//刷新出参
									tqf[i] = q2[i];
									dqf[i] = dq2[i];
									ddqf[i] = ddq2[i];
								}
								else//匀加速后的位置不超过下一点位置 计算减加速段
								{

									//更新ta3段的位置速度加速度值
									ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
									dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
									q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

									//不存在第四时间段
									ta4[i] = 0;
									//更新出参
									tqf[i] = q3[i];
									dqf[i] = dq3[i];
									ddqf[i] = ddq3[i];
								}
							}
							else //ta4大于0存在第四段 刷新位置速度加速度 刷新出参
							{
								ta4[i] = ta41;
								ddq4[i] = 0;
								dq4[i] = dq3[i];
								q4[i] = q3[i] - ta41*dq3[i];
								tqf[i] = q4[i];
								dqf[i] = dq4[i];
								ddqf[i] = ddq4[i];
							}
						}
						else //ta4大于0存在第四段 刷新位置速度加速度 刷新出参
						{
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] - ta4[i] * dq3[i];
							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
					else if (newT[0] > minim && newT[1] < -minim) // 解1 为正 解2为负 取解1
					{
						//刷新ta2 刷新位置速度加速度
						ta2[i] = newT[0];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//刷新ta3 和位置速度加速度
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//计算ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];
						//如果ta4小于0 表示前面给的解不合适 重新计算
						if (ta4[i] < 0)
						{
							// 去掉ta4 重新计算ta2 刷新位置速度加速度
							ta2[i] = Ti[i] - ta1[i] - ta3[i];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
							//如果q2的位置大于下一点位置 联立q1和q2的方程组 得到一元三次方程
							if ((sigma[i] == 1 && q2[i] < q0[i]) || (sigma[i] == -1 && q2[i] > q0[i]))
							{
								// 给出一元三次方程的系数
								a3 = 1 / 3.0 * sigma[i] * Jm[i]; //a
								a2 = 1 / 2.0 * sigma[i] * Jm[i] * Ti[i] + 1 / 2.0 * ddq0[i]; //b
								a1 = -1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
								a0 = -1 / 2.0 * ddq0[i] * pow(Ti[i], 2) - dq0[i] * Ti[i] + qf[i] - q0[i]; //d


								getrootsofquadratic(a3, a2, a1, a0, r); //求一元三次方程的根
								tempt = 99;
								for (int j = 0; j < 3; j++)
								{
									if (r[j] < tempt && r[j] > minim) //求最小的正时间
									{
										tempt = r[j];
									}
								}
								//刷新ta1 刷新位置速度加速度
								ta1[i] = tempt;
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

								//刷新ta2 刷新位置速度加速度
								ta2[i] = Ti[i] - ta1[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								//不存在第三和第四时间段
								ta3[i] = 0;
								ta4[i] = 0;
								//刷新出参
								tqf[i] = q2[i];
								dqf[i] = dq2[i];
								ddqf[i] = ddq2[i];
							}
							else//q2小于qf 存在第三段
							{
								//                     ta3[i] = Ti[i] - ta1[i] - ta2[i];
								//刷新位置速度加速度
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								//不存在第四段时间
								ta4[i] = 0;
								//刷新出参
								tqf[i] = q3[i];
								dqf[i] = dq3[i];
								ddqf[i] = ddq3[i];
							}
						}
						else//存在第四段
						{
							//刷新位置速度加速度
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] - ta4[i] * dq3[i];
							//刷新出参
							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
					else if (newT[1] > minim && newT[0] < -minim) // 解2 为正 解1为负 取解2
					{
						//刷新ta2 刷新位置速度加速度
						ta2[i] = newT[1];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//刷新ta3 和位置速度加速度
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//计算ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];
						//如果ta4小于0 表示前面给的解不合适 重新计算
						if (ta4[i] < 0)
						{
							// 去掉ta4 重新计算ta2 刷新位置速度加速度
							ta2[i] = Ti[i] - ta1[i] - ta3[i];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
							//如果q2的位置大于下一点位置 联立q1和q2的方程组 得到一元三次方程
							if ((sigma[i] == 1 && q2[i] < q0[i]) || (sigma[i] == -1 && q2[i] > q0[i]))
							{
								// 给出一元三次方程的系数
								a3 = 1 / 3.0 * sigma[i] * Jm[i]; //a
								a2 = 1 / 2.0 * sigma[i] * Jm[i] * Ti[i] + 1 / 2.0 * ddq0[i]; //b
								a1 = -1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
								a0 = -1 / 2.0 * ddq0[i] * pow(Ti[i], 2) - dq0[i] * Ti[i] + qf[i] - q0[i]; //d

																										  //                                 r = roots([a3 a2 a1 a0]);
								getrootsofquadratic(a3, a2, a1, a0, r); //求一元三次方程的根
								tempt = 99;
								for (int j = 0; j < 3; j++)
								{
									if (r[j]<tempt && r[j]>minim) //求最小的正时间
									{
										tempt = r[j];
									}
								}
								//刷新ta1 刷新位置速度加速度
								ta1[i] = tempt;
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);
								//刷新ta2 刷新位置速度加速度
								ta2[i] = Ti[i] - ta1[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
								//不存在第三和第四时间段
								ta3[i] = 0;
								ta4[i] = 0;
								//                                 continue;
								//刷新出参
								tqf[i] = q2[i];
								dqf[i] = dq2[i];
								ddqf[i] = ddq2[i];
							}
							else //q2小于qf 存在第三段
							{
								//                     ta3[i] = Ti[i] - ta1[i] - ta2[i];
								//刷新位置速度加速度
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);
								//不存在第四段时间
								ta4[i] = 0;
								//刷新出参
								tqf[i] = q3[i];
								dqf[i] = dq3[i];
								ddqf[i] = ddq3[i];
							}
						}
						else//存在第四段
						{
							//刷新位置速度加速度
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] - ta4[i] * dq3[i];
							//刷新出参
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
				//刷新各段时间的对应三次方程的系数
				if (ta1[i] > 0)
				{
					coeff[i][0][0] = -1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][0][1] = -1 / 2.0 * ddq0[i];
					coeff[i][0][2] = -dq0[i];
					coeff[i][0][3] = qf[i];
				}
				if (ta2[i] > 0)
				{
					coeff[i][1][0] = 0;
					coeff[i][1][1] = -1 / 2.0 * ddq1[i];
					coeff[i][1][2] = -dq1[i];
					coeff[i][1][3] = q1[i];
				}
				if (ta3[i] > 0)
				{
					coeff[i][2][0] = 1 / 6.0 * sigma[i] * Jm[i];
					coeff[i][2][1] = -1 / 2.0 * ddq2[i];
					coeff[i][2][2] = -dq2[i];
					coeff[i][2][3] = q2[i];
				}
				if (ta4[i] > 0)
				{
					coeff[i][3][0] = 0;
					coeff[i][3][1] = 0;
					coeff[i][3][2] = -dq3[i];
					coeff[i][3][3] = q3[i];
				}
			}
			else if (sigma[i] == 0) // 静止的处理
			{

				ta1[i] = 0;
				ta2[i] = 0;
				ta3[i] = 0;
				ta4[i] = 0;
				tqf[i] = qf[i];
				dqf[i] = 0;
				ddqf[i] = 0;
				coeff[i][0][0] = 0;
				coeff[i][0][1] = 0;
				coeff[i][0][2] = 0;
				coeff[i][0][3] = qf[i];
				coeff[i][1][0] = 0;
				coeff[i][1][1] = 0;
				coeff[i][1][2] = 0;
				coeff[i][1][3] = qf[i];
				coeff[i][2][0] = 0;
				coeff[i][2][1] = 0;
				coeff[i][2][2] = 0;
				coeff[i][2][3] = qf[i];
				coeff[i][3][0] = 0;
				coeff[i][3][1] = 0;
				coeff[i][3][2] = 0;
				coeff[i][3][3] = qf[i];
			}
		}
		*maxT = -1;
		for (int i = 0; i < 6; i++)
		{
			if (*maxT < tempT[i])
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

	double dq_avg[6];
	double ddq_max[6] = { 0 };// {

	J0[0] = start.alpha.j1; J0[1] = start.alpha.j2; J0[2] = start.alpha.j3; J0[3] = start.alpha.j4; J0[4] = start.alpha.j5; J0[5] = start.alpha.j6;
	J1[0] = target.j1; J1[1] = target.j2; J1[2] = target.j3; J1[3] = target.j4; J1[4] = target.j5; J1[5] = target.j6;
	for (int i = 0; i < 6; i++)
	{

		dq_avg[i] = (J1[i] - J0[i]) / exp_duration;
	}

	double dq0[6] = { 0 };
	double ddq0[6] = { 0 };
	double jk[6] = { 0 };
	for (int i = 0; i < 6; i++)
	{
		if (J1[i] > J0[i])
		{
			switch (i)
			{
			case 0:
				ddq_max[i] = alpha_upper.j1;
				dq0[i] = start.angle.j1;
				ddq0[i] = start.omega.j1;
				jk[i] = jerk.j1;
				break;
			case 1:
				ddq_max[i] = alpha_upper.j2;
				dq0[i] = start.angle.j2;
				ddq0[i] = start.omega.j2;
				jk[i] = jerk.j2;
				break;
			case 2:
				ddq_max[i] = alpha_upper.j3;
				dq0[i] = start.angle.j3;
				ddq0[i] = start.omega.j3;
				jk[i] = jerk.j3;
				break;
			case 3:
				ddq_max[i] = alpha_upper.j4;
				dq0[i] = start.angle.j4;
				ddq0[i] = start.omega.j4;
				jk[i] = jerk.j4;
				break;
			case 4:
				ddq_max[i] = alpha_upper.j5;
				dq0[i] = start.angle.j5;
				ddq0[i] = start.omega.j5;
				jk[i] = jerk.j5;
				break;
			case 5:
				ddq_max[i] = alpha_upper.j6;
				dq0[i] = start.angle.j6;
				ddq0[i] = start.omega.j6;
				jk[i] = jerk.j6;
				break;

			}

		}

		else
		{
			switch (i)
			{
			case 0:
				ddq_max[i] = alpha_lower.j1;
				dq0[i] = start.angle.j1;
				ddq0[i] = start.omega.j1;
				jk[i] = jerk.j1;
				break;
			case 1:
				ddq_max[i] = alpha_lower.j2;
				dq0[i] = start.angle.j2;
				ddq0[i] = start.omega.j2;
				jk[i] = jerk.j2;
				break;
			case 2:
				ddq_max[i] = alpha_lower.j3;
				dq0[i] = start.angle.j3;
				ddq0[i] = start.omega.j3;
				jk[i] = jerk.j3;
				break;
			case 3:
				ddq_max[i] = alpha_lower.j4;
				dq0[i] = start.angle.j4;
				ddq0[i] = start.omega.j4;
				jk[i] = jerk.j4;
				break;
			case 4:
				ddq_max[i] = alpha_lower.j5;
				dq0[i] = start.angle.j5;
				ddq0[i] = start.omega.j5;
				jk[i] = jerk.j5;
				break;
			case 5:
				ddq_max[i] = alpha_lower.j6;
				dq0[i] = start.angle.j6;
				ddq0[i] = start.omega.j6;
				jk[i] = jerk.j6;
				break;

			}

		}
	}


	double Ti[6] = { -1,-1,-1,-1,-1,-1 };
	double Ta[6][4] = { 0 };
	double coeff[6][4][4] = { 0 };
	double maxT = 0;
	double tqf[6] = { 0 };
	double dqf[6] = { 0 };
	double ddqf[6] = { 0 };
	int ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max, jk, Ti, 2, Ta, coeff, &maxT, tqf, dqf, ddqf);

	printf("maxT=%f\n", maxT);

	for (int i = 0; i < 6; i++)
		Ti[i] = maxT;
	ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max, jk, Ti, 2, Ta, coeff, &maxT, tqf, dqf, ddqf);

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j<4; j++)
		{
			segment[i].duration[j] = Ta[i][j];
			for (int k = 0; k < 4; k++)
			{
				segment[i].coeff[j][k] = coeff[i][j][k];
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

	J1[0] = target.alpha.j1; J1[1] = target.alpha.j2; J1[2] = target.alpha.j3; J1[3] = target.alpha.j4; J1[4] = target.alpha.j5; J1[5] = target.alpha.j6;
	J0[0] = start.j1; J0[1] = start.j2; J0[2] = start.j3; J0[3] = start.j4; J0[4] = start.j5; J0[5] = start.j6;
	for (int i = 0; i < 6; i++)
	{

		dq_avg[i] = (J1[i] - J0[i]) / exp_duration;
	}

	double dq0[6] = { 0 };
	double ddq0[6] = { 0 };
	double jk[6] = { 0 };
	for (int i = 0; i < 6; i++)
	{
		if (J1[i] > J0[i])
		{
			switch (i)
			{
			case 0:
				ddq_max[i] = alpha_upper.j1;
				dq0[i] = target.angle.j1;
				ddq0[i] = target.omega.j1;
				jk[i] = jerk.j1;
				break;
			case 1:
				ddq_max[i] = alpha_upper.j2;
				dq0[i] = target.angle.j2;
				ddq0[i] = target.omega.j2;
				jk[i] = jerk.j2;
				break;
			case 2:
				ddq_max[i] = alpha_upper.j3;
				dq0[i] = target.angle.j3;
				ddq0[i] = target.omega.j3;
				jk[i] = jerk.j3;
				break;
			case 3:
				ddq_max[i] = alpha_upper.j4;
				dq0[i] = target.angle.j4;
				ddq0[i] = target.omega.j4;
				jk[i] = jerk.j4;
				break;
			case 4:
				ddq_max[i] = alpha_upper.j5;
				dq0[i] = target.angle.j5;
				ddq0[i] = target.omega.j5;
				jk[i] = jerk.j5;
				break;
			case 5:
				ddq_max[i] = alpha_upper.j6;
				dq0[i] = target.angle.j6;
				ddq0[i] = target.omega.j6;
				jk[i] = jerk.j6;
				break;

			}

		}

		else
		{
			switch (i)
			{
			case 0:
				ddq_max[i] = alpha_lower.j1;
				dq0[i] = target.angle.j1;
				ddq0[i] = target.omega.j1;
				jk[i] = jerk.j1;
				break;
			case 1:
				ddq_max[i] = alpha_lower.j2;
				dq0[i] = target.angle.j2;
				ddq0[i] = target.omega.j2;
				jk[i] = jerk.j2;
				break;
			case 2:
				ddq_max[i] = alpha_lower.j3;
				dq0[i] = target.angle.j3;
				ddq0[i] = target.omega.j3;
				jk[i] = jerk.j3;
				break;
			case 3:
				ddq_max[i] = alpha_lower.j4;
				dq0[i] = target.angle.j4;
				ddq0[i] = target.omega.j4;
				jk[i] = jerk.j4;
				break;
			case 4:
				ddq_max[i] = alpha_lower.j5;
				dq0[i] = target.angle.j5;
				ddq0[i] = target.omega.j5;
				jk[i] = jerk.j5;
				break;
			case 5:
				ddq_max[i] = alpha_lower.j6;
				dq0[i] = target.angle.j6;
				ddq0[i] = target.omega.j6;
				jk[i] = jerk.j6;
				break;

			}

		}
	}


	double Ti[6] = { -1,-1,-1,-1,-1,-1 };
	double Ta[6][4] = { 0 };
	double coeff[6][4][4] = { 0 };
	double maxT = 0;
	double tqf[6] = { 0 };
	double dqf[6] = { 0 };
	double ddqf[6] = { 0 };
	int ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max, jk, Ti, 2, Ta, coeff, &maxT, tqf, dqf, ddqf);

	printf("maxT=%f\n", maxT);

	for (int i = 0; i < 6; i++)
		Ti[i] = maxT;
	ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max, jk, Ti, 2, Ta, coeff, &maxT, tqf, dqf, ddqf);

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j<4; j++)
		{
			segment[i].duration[j] = Ta[i][j];
			for (int k = 0; k < 4; k++)
			{
				segment[i].coeff[j][k] = coeff[i][j][k];
			}
		}
		//printf("joint:%d Ta=%f %f %f %f\n", i, Ta[i][0], Ta[i][1], Ta[i][2], Ta[i][3]);
		//printf("coeff:%f %f %f %f -- %f %f %f %f -- %f %f %f %f -- %f %f %f %f\n", coeff[i][0][0], coeff[i][0][1], coeff[i][0][2], coeff[i][0][3], coeff[i][1][0], coeff[i][1][1], coeff[i][1][2], coeff[i][1][3], coeff[i][2][0], coeff[i][2][1], coeff[i][2][2], coeff[i][2][3], coeff[i][3][0], coeff[i][3][1], coeff[i][3][2], coeff[i][3][3]);
		//printf("each joint time:%f\n", maxT);
		//printf("dest status(q,dq,ddq):%f %f %f\n", tqf[i], dqf[i], ddqf[i]);
	}

	return ret;

}
                          
ErrorCode computeDynamics(const fst_mc::Joint &angle, const fst_mc::Joint &omega,
  fst_mc::Joint &alpha_upper, fst_mc::Joint &alpha_lower, fst_mc::DynamicsProduct &product)
{
  double tq[MAXAXES];
  double tdq[MAXAXES];
  double tddq[2][MAXAXES];
  double tcg[MAXAXES];
  
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
  double mq[6][6], mdq[6][6], mddq[6][6];
  double M[6][6],C[6][6];

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
      printf("%f %f %f %f %f %f\n", mddq[0][i], mddq[1][i], mddq[2][i], mddq[3][i], mddq[4][i], mddq[5][i]);
  }
  g_dynamics_interface.rne_M(mq, mdq, mddq, M);


  g_dynamics_interface.rne_C(tq, tdq, C);

  double GRV[3] = { 0,0,9.81 };

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
  
  alpha_upper.j1 = tddq[0][0];
  alpha_upper.j2 = tddq[0][1];
  alpha_upper.j3 = tddq[0][2];
  alpha_upper.j4 = tddq[0][3];
  alpha_upper.j5 = tddq[0][4];
  alpha_upper.j6 = tddq[0][5];

  alpha_lower.j1 = tddq[1][0];
  alpha_lower.j2 = tddq[1][1];
  alpha_lower.j3 = tddq[1][2];
  alpha_lower.j4 = tddq[1][3];
  alpha_lower.j5 = tddq[1][4];
  alpha_lower.j6 = tddq[1][5];
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
