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
getrootofquadratic����һԪ���η��̵ĸ�
���������a���������ϵ�� b���������ϵ�� c��һ�����ϵ�� d���������ϵ��
���������Sv[3]  Ϊ���̵������� ĳЩ��������� ������һ���� ���������������ԣ�
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
	//	calculateParam�����Ӽ��ٸ���ϵ���ļ��㺯�� ����ʱ����ϵ��  һԪ���η��̸���ϵ��������ͬ��ʱ�����
	//   ��Σ�
	//	q0�������ǼӼ��ٶ�����һ��Ĺؽڽ�ֵ 6��1����
	//	dq0�����ڼ�������һ��Ĺؽ��ٶ�ֵ ���ڼ�������һ��Ĺؽ��ٶ�ֵ 6��1����
	//	ddq0�����ڼ�������һ��Ĺؽڼ��ٶ�ֵ  ���ڼ�������һ��Ĺؽڼ��ٶ�ֵ 6��1����
	//	qf�����۶��ڼӼ��� ������һ��Ĺؽڽ�ֵ 6��1����
	//	maxdq�������м����ƽ���ٶ�ֵ 6��1����
	//	maxddq�������м���Ķ���ѧУ����� / ��С���ٶ�ֵ 6��1����
	//	Jm�������Ĭ�ϵ�Jerkֵ 6��1����
	//	Ti�����ڼ������ͬ��ʱ����˵ �� - 1ֵ 6��1����
	//	type��1��ʾ���� 2��ʾ����
	//	dofn���ؽ��� ��Ĭ��6

	//	���Σ�
	//	Ta�������ʱ��ϵ����6��4���� 6Ϊ�ؽ��� 4��ʱ��ε�ʱ��ֵ
	//	coeff������ĸ�ʱ��ε����η���ϵ��ֵ��6��4��4 ���� 6Ϊ�ؽ��� 4Ϊ4��ʱ�� 4Ϊ�ĸ�ϵ��
	//	maxT������TiΪ - 1 �˴����ص�maxTΪ����ͬ����ʱ�� doubleֵ ����TiΪͬ��ʱ�� ����ı�
	//	tqf�����ڼ�������һ��ؽ�ֵ ���ڼ�������һ��Ĺؽ�ֵ
	//	dqf�����ڼ�������һ��ؽ��ٶ�ֵ  ���ڼ�������һ��ؽ��ٶ�ֵ
	//	ddqf�����ڼ�������һ��ؽڼ��ٶ�ֵ  ���ڼ�������һ��ؽڼ��ٶ�ֵ

	//	�������

	//������ʱ���� ta1 ta2 ta3 ta4 ��ʾ4��ʱ���ϵ��ֵ 6��1����
	double ta1[MAXAXES], ta2[MAXAXES], ta3[MAXAXES], ta4[MAXAXES];


	//�Ը��ν���ʱ�̵�λ�� �ٶ� ���ٶȽ��б����ڴ����
	double q1[MAXAXES], q2[MAXAXES], q3[MAXAXES], q4[MAXAXES];
	double dq1[MAXAXES], dq2[MAXAXES], dq3[MAXAXES], dq4[MAXAXES];
	double ddq1[MAXAXES], ddq2[MAXAXES], ddq3[MAXAXES], ddq4[MAXAXES];

	//������ʱ����
	double ddq_max[MAXAXES], dq_avg[MAXAXES], sigma[MAXAXES]; //�����ٶ� ƽ���ٶ� ��ʱ�� jerk����
	double tempT[MAXAXES] = { 0 };
	double newT[2] = { 0 };
	double r[3] = { 0 };

	double a, b, c, lamda, a3, a2, a1, a0, tempt, ta41;
	int dofn = MAXAXES;

	if (type == 1) // ���ټ���
	{
		for (int i = 0; i < dofn; i++)
		{
			//���ݹؽڵ��˶����� �ж�Jerk�ķ��ű���
			if (qf[i] - q0[i] > minim)
			{
				sigma[i] = 1; //��ת  ����ȡ1
			}
			else if (qf[i] - q0[i] < -minim)
			{
				sigma[i] = -1; //��ת ����ȡ - 1
			}
			else
			{
				sigma[i] = 0;	//��ֹ ����ȡ0
			}

			if (sigma[i] != 0 && Ti[i] == -1) // �ؽڴ����˶� ��ֻ��������ͬ���˶�ʱ�� �����������
			{
				ddq_max[i] = maxddq[i];	//���ٶȸ�ֵ
				dq_avg[i] = maxdq[i];		//ƽ���ٶȸ�ֵ

											//time adjust
											//stage 1  -- - acc - acc
											//��һ�μ���  ����Ӽ��ٶ���Ӽ��ٵ�ʱ��ta1
											//��Ӽ��ٵ�ʱ��ta1 ���ٶ�ddq1 �ٶ�dq1 ����ʱ��λ�� q1
				ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				//stage 2 -- - quansi - acc
				//�ڶ��μ���  �����ȼ��ٶ����ȼ��ٵ�ʱ��ta2
				//���ȼ��ٵ�ʱ��ta2 ���ٶ�ddq2 �ٶ�dq2 ����ʱ��λ�� q2
				//��Ҫ��ǰ��������������ʱ��ta3 �Լӵ�ƽ���ٶ�dq_avg��Ϊ�������������ta2
				ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
				ta2[i] = (dq_avg[i] - ddq_max[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2) - dq1[i]) / ddq_max[i];
				ddq2[i] = ddq1[i];
				dq2[i] = dq1[i] + ddq1[i] * ta2[i];
				q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

				//�ڶ��μ���Ľ���ʱ��λ�����������һ���λ�� ��Ҫ���¼���ta2
				if ((sigma[i] == 1 && q2[i] > qf[i]) || (sigma[i] == -1 && q2[i] < qf[i]))
				{
					// ��������������ĸ�ϵ��
					a = 1 / 2.0 * ddq1[i];
					b = dq1[i];
					c = -(qf[i] - q1[i]);
					lamda = pow(b, 2) - 4 * a*c;
					if (lamda < 0)
					{
						printf("signification erreur"); //������ϵ������ ��ʾ������·����ֵʱ��̫�� ����С��900���� ����
						return -1;
					}

					newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a); //����Ķ������1
					newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a); //����Ķ������2

					if (newT[0] > minim && newT[1] > minim) // ����������� ȡ�̵�ʱ��������
					{
						if (newT[0] > newT[1]) // ta2ȡʱ���С��ֵ
						{
							ta2[i] = newT[1];
						}
						else
						{
							ta2[i] = newT[0];
						}
					}
					else if (newT[0] > minim && newT[1] < -minim) // ȡ����ʱ��ֵ
					{
						ta2[i] = newT[0];
					}
					else if (newT[1] > minim && newT[0] < -minim) // ȡ����ʱ��ֵ
					{
						ta2[i] = newT[1];
					}
					//���¼���ta2�� ���¼��ٶ� �ٶȺ�λ��
					ddq2[i] = ddq1[i];
					dq2[i] = dq1[i] + ddq1[i] * ta2[i];
					q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

					//���³���
					tqf[i] = q2[i];
					dqf[i] = dq2[i];
					ddqf[i] = ddq2[i];

					//�����ڵ����͵���ʱ���
					ta3[i] = 0;
					ta4[i] = 0;

					//������ʱ��
					tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];

					Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];  //����ʱ�丳ֵ

																								 //����ϵ����ֵ
					if (ta1[i] > 0)
					{
						coeff[i][0][0] = 1 / 6.0 * sigma[i] * Jm[i]; // 3��
						coeff[i][0][1] = 1 / 2.0 * ddq0[i]; // 2��
						coeff[i][0][2] = dq0[i]; // 1��
						coeff[i][0][3] = q0[i]; // 0��
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
				//��������ٶ�
				//�����ٶε�ʱ��ta3���ȼ���ʱ�Ѽ��㣬ˢ�¼��ٶȣ��ٶȺ�λ��ֵ
				ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
				dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
				q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

				//�жϼ����ٽ������λ���Ƿ񳬹���һ��λ�� ������� ���¼���ʱ��ta3
				if ((sigma[i] == 1 && q3[i] > qf[i]) || (sigma[i] == -1 && q3[i] < qf[i]))
				{
					// ���ݼ����ٶε�һԪ���η��̼��������ĸ�
					//���Ȼ�ø����ϵ��
					a3 = -1 / 6.0 * sigma[i] * Jm[i]; //a
					a2 = 1 / 2.0 * ddq2[i]; //b
					a1 = dq2[i]; //c
					a0 = -(qf[i] - q2[i]); //d

					getrootsofquadratic(a3, a2, a1, a0, r); //����һԪ���η��̵ĸ�
					double tempt = 99; //���ȫ���Ľ�
					for (int j = 0; j < 3; j++)
					{
						if (r[j]<tempt && r[j]>minim) //����Ҫ��ȥ����ʱ�䣬ȡ��Сʱ��
						{
							tempt = r[j];
						}
					}

					ta3[i] = tempt;

					//����ˢ��ta3��λ�� �ٶ� �ͼ��ٶ�ֵ
					ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
					dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
					q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

					//�����ڵ���ʱ���
					ta4[i] = 0;

					//���¸ùؽڵĽ�����λ�� �ٶ� ���ٶ�
					tqf[i] = q3[i];
					dqf[i] = dq3[i];
					ddqf[i] = ddq3[i];

					//�õ���ʱ��
					tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
					Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];  //����ʱ��ϵ����ˢ��
																								 //����������ϵ����ˢ��
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
				//���Ķ� ���ټ���
				//�������ٵ�ta4 �Լ�λ���ٶ� ���ٶ�ֵ
				//�����жϼ����ٽ������Ƿ�С��Ŀ��λ�ã���һ��λ�ã� С�� ��������ٶ�
				if ((sigma[i] == 1 && q3[i] < qf[i]) || (sigma[i] == -1 && q3[i] > qf[i]))
				{
					// �õ����ٶε�ta4 λ�ú��ٶ� ���ٶ�ֵ
					ta4[i] = (qf[i] - q3[i]) / dq3[i];
					ddq4[i] = 0;
					dq4[i] = dq3[i];
					q4[i] = q3[i] + ta4[i] * dq3[i];

					//���·��ص�λ���ٶȼ��ٶ�ֵ
					tqf[i] = q4[i];
					dqf[i] = dq4[i];
					ddqf[i] = ddq4[i];
				}

				//������ʱ��
				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];

				//���¸���ʱ��ϵ��
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
				//���¸���Ķ���ʽϵ��
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
			else if (sigma[i] != 0 && Ti[i] > 0) // ��һ���Ǹ��ݶ���ʱ��ͬ���� ������ε�ʱ��ϵ���Ͷ���ʽϵ�� �����ؽ�����λ���ٶ� ���ٶ�
			{
				ddq_max[i] = maxddq[i];	//���ٶȸ�ֵ
				dq_avg[i] = maxdq[i];		//ƽ���ٶȸ�ֵ

											//time adjust
											//stage 1  -- - acc - acc
											//����Ӽ��ٶ���Ӽ��ٵ�ʱ��ta1 ˢ��λ���ٶȼ��ٶ�
				ta1[i] = (ddq_max[i] - ddq0[i]) / (sigma[i] * Jm[i]);
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				//�����ȼ��ٶ�ta2ǰ ��Ҫ���������ʱ��ta3
				ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);

				//���ȸ���ʱ��Ti��Լ�� ��ta4�ĺ���ֵ
				//��������������̵Ķ��������ϵ��
				a = -1 / 2.0 * ddq1[i];
				b = 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2) + ddq1[i] * (Ti[i] - ta1[i] - ta3[i]);

				c = (Ti[i] - ta1[i] - ta3[i])*(ddq1[i] * ta3[i] + dq1[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2)) + q1[i] + dq1[i] * ta3[i] + 1 / 2.0 * ddq1[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3) - qf[i];
				lamda = pow(b, 2) - 4 * a*c; //lamdaֵ

				if (lamda <= 0) // lamdaֵС��0 ��ʾta2�޽� ��Ҫ����ta1 ta3 ���
				{
					//���¼���ta2 ˢ��λ���ٶȼ��ٶ�
					ta2[i] = Ti[i] - ta1[i] - ta3[i];
					ddq2[i] = ddq1[i];
					dq2[i] = dq1[i] + ddq1[i] * ta2[i];
					q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
					//����ȼ��ٺ�λ�ó�����һ��λ�� �������η������¼���ta2��ʱ��
					if ((sigma[i] == 1 && q2[i] > qf[i]) || (sigma[i] == -1 && q2[i] < qf[i]))
					{
						// ��������ta1��ta2���̺� ���η��̵ĸ���ϵ��  ���������ta1ֵ
						a3 = -1 / 3.0 * sigma[i] * Jm[i]; //a
						a2 = -1 / 2.0 * sigma[i] * Jm[i] * Ti[i] - 1 / 2.0 * ddq0[i]; //b
						a1 = 1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
						a0 = 1 / 2.0 * ddq0[i] * pow(Ti[i], 2) + dq0[i] * Ti[i] + q0[i] - qf[i]; //d

						getrootsofquadratic(a3, a2, a1, a0, r); //�����η��̵ĸ�
						tempt = 99;
						for (int j = 0; j < 3; j++)
						{
							if (r[j] < tempt && r[j] > minim) //��ʱ����С��Ϊ���ĸ�
							{
								tempt = r[j];
							}
						}

						//���ݷ��̵ĸ� ˢ��ta1 �Լ�λ���ٶ� ���ٶ�
						ta1[i] = tempt;
						ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
						dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
						q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

						//���¼���ta2 ˢ��λ���ٶȼ��ٶ�
						ta2[i] = Ti[i] - ta1[i];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//�����ڵ����͵���ʱ���
						ta3[i] = 0;
						ta4[i] = 0;

						//���³���
						tqf[i] = q2[i];
						dqf[i] = dq2[i];
						ddqf[i] = ddq2[i];
					}

					else //���ڵ�����ta3 q2С��qf
					{

						//ˢ�¼����ٽ������λ���ٶȼ��ٶ�
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//�����ڵ��Ķ�ʱ��
						ta4[i] = 0;
						//���³���
						tqf[i] = q3[i];
						dqf[i] = dq3[i];
						ddqf[i] = ddq3[i];
					}
				}
				else //���ڵ��Ķ� ta2  lamda����0
				{
					newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a); //���ta4�Ľ�1
					newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a); //���ta4�Ľ�2

																	//�������ʱ��ⶼΪ�� ���ν��м��� ���ж�
					if (newT[0] > minim && newT[1] > minim)
					{

						//����ȡ��1 ˢ��ta2 ˢ��λ���ٶȼ��ٶ�
						ta2[i] = newT[0];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//����ta3 ˢ��λ���ٶȼ��ٶ�
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//ˢ��ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];

						//��������ta4 Ϊ�� ��ʾȡ�ýⲻ���� ����2 ���¼���
						if (ta4[i] < 0)
						{
							// ����2 ����λ���ٶȼ��ٶ�
							ta2[i] = newT[1];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							//����ta3λ���ٶȼ��ٶ�
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							//����ta4
							ta41 = Ti[i] - ta2[i] - ta1[i] - ta3[i];

							//��������ⶼ������ta4 ��ȥ������ʱ��� ���µ���ta2
							if (ta41 < 0)
							{
								// ���¼���ta2 ˢ��λ���ٶȼ��ٶ�
								ta2[i] = Ti[i] - ta1[i] - ta3[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
								//���q2��λ�ô�����һ��λ�� ��Ҫ�������η������¼���ta1��ta2
								if ((sigma[i] == 1 && q2[i] > qf[i]) || (sigma[i] == -1 && q2[i] < qf[i]))
								{
									// �������η���
									a3 = -1 / 3.0 * sigma[i] * Jm[i]; //a
									a2 = -1 / 2.0 * sigma[i] * Jm[i] * Ti[i] - 1 / 2.0 * ddq0[i]; //b
									a1 = 1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
									a0 = 1 / 2.0 * ddq0[i] * pow(Ti[i], 2) + dq0[i] * Ti[i] + q0[i] - qf[i]; //d

									getrootsofquadratic(a3, a2, a1, a0, r); //�����η��̵ĸ�
									tempt = 99;
									for (int j = 0; j < 3; j++)
									{
										if (r[j]<tempt && r[j]>minim) 	//ȡ��С����ʱ��ĸ�
										{
											tempt = r[j];
										}
									}

									//ˢ��ta1 �Լ�λ���ٶȼ��ٶ�
									ta1[i] = tempt;
									ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
									dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
									q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

									//ˢ��ta2 �Լ�λ���ٶȼ��ٶ�
									ta2[i] = Ti[i] - ta1[i];
									ddq2[i] = ddq1[i];
									dq2[i] = dq1[i] + ddq1[i] * ta2[i];
									q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

									//�����ڵ����͵���ʱ���
									ta3[i] = 0;
									ta4[i] = 0;

									//ˢ�³���
									tqf[i] = q2[i];
									dqf[i] = dq2[i];
									ddqf[i] = ddq2[i];
								}
								else //�ȼ��ٺ��λ�ò�������һ��λ�� ��������ٶ�
								{
									//����ta3�ε�λ���ٶȼ��ٶ�ֵ
									ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
									dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
									q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

									//�����ڵ���ʱ���
									ta4[i] = 0;
									//���³���
									tqf[i] = q3[i];
									dqf[i] = dq3[i];
									ddqf[i] = ddq3[i];
								}
							}
							else //ta4����0���ڵ��Ķ� ˢ��λ���ٶȼ��ٶ� ˢ�³���
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
						else //ta4����0���ڵ��Ķ� ˢ��λ���ٶȼ��ٶ� ˢ�³���
						{

							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];
							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
					else if (newT[0] > minim && newT[1] < -minim) // ��1 Ϊ�� ��2Ϊ�� ȡ��1
					{
						//ˢ��ta2 ˢ��λ���ٶȼ��ٶ�
						ta2[i] = newT[0];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//ˢ��ta3 ��λ���ٶȼ��ٶ�
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//����ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];
						if (ta4[i] < 0) // ���ta4С��0 ��ʾǰ����Ľⲻ���� ���¼���
						{
							//ȥ��ta4 ���¼���ta2 ˢ��λ���ٶȼ��ٶ�
							ta2[i] = Ti[i] - ta1[i] - ta3[i];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
							//���q2��λ�ô�����һ��λ�� ����q1��q2�ķ����� �õ�һԪ���η���
							if ((sigma[i] == 1 && q2[i] > qf[i]) || (sigma[i] == -1 && q2[i] < qf[i]))
							{
								// ����һԪ���η��̵�ϵ��
								a3 = -1 / 3.0 * sigma[i] * Jm[i]; //a
								a2 = -1 / 2.0 * sigma[i] * Jm[i] * Ti[i] - 1 / 2.0 * ddq0[i]; //b
								a1 = 1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
								a0 = 1 / 2.0 * ddq0[i] * pow(Ti[i], 2) + dq0[i] * Ti[i] + q0[i] - qf[i]; //d

								getrootsofquadratic(a3, a2, a1, a0, r); //��һԪ���η��̵ĸ�
								tempt = 99;
								for (int j = 0; j < 3; j++)
								{
									if (r[j] < tempt && r[j] > minim)	//����С����ʱ��
									{
										tempt = r[j];
									}
								}
								//ˢ��ta1 ˢ��λ���ٶȼ��ٶ�
								ta1[i] = tempt;
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

								//ˢ��ta2 ˢ��λ���ٶȼ��ٶ�
								ta2[i] = Ti[i] - ta1[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								//�����ڵ����͵���ʱ���
								ta3[i] = 0;
								ta4[i] = 0;
								//ˢ�³���
								tqf[i] = q2[i];
								dqf[i] = dq2[i];
								ddqf[i] = ddq2[i];
							}
							else //q2С��qf ���ڵ�����
							{
								//                     ta3[i] = Ti[i] - ta1[i] - ta2[i];
								//ˢ��λ���ٶȼ��ٶ�
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								//�����ڵ��Ķ�ʱ��
								ta4[i] = 0;
								//ˢ�³���
								tqf[i] = q3[i];
								dqf[i] = dq3[i];
								ddqf[i] = ddq3[i];
							}
						}
						else //���ڵ��Ķ�
						{
							//ˢ��λ���ٶȼ��ٶ�
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];
							//ˢ�³���
							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
					else if (newT[1] > 1e-6 && newT[0] < -1e-6) // ��2 Ϊ�� ��1Ϊ�� ȡ��2
					{
						//ˢ��ta2 ˢ��λ���ٶȼ��ٶ�
						ta2[i] = newT[1];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//ˢ��ta3 ��λ���ٶȼ��ٶ�
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//����ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];
						if (ta4[i] < 0) // ���ta4С��0 ��ʾǰ����Ľⲻ���� ���¼���
						{
							//ȥ��ta4 ���¼���ta2 ˢ��λ���ٶȼ��ٶ�
							ta2[i] = Ti[i] - ta1[i] - ta3[i];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
							//���q2��λ�ô�����һ��λ�� ����q1��q2�ķ����� �õ�һԪ���η���
							if ((sigma[i] == 1 && q2[i] > qf[i]) || (sigma[i] == -1 && q2[i] < qf[i]))
							{
								// ����һԪ���η��̵�ϵ��
								a3 = -1 / 3.0 * sigma[i] * Jm[i]; //a
								a2 = -1 / 2.0 * sigma[i] * Jm[i] * Ti[i] - 1 / 2.0 * ddq0[i]; //b
								a1 = 1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
								a0 = 1 / 2.0 * ddq0[i] * pow(Ti[i], 2) + dq0[i] * Ti[i] + q0[i] - qf[i]; //d

								getrootsofquadratic(a3, a2, a1, a0, r); //��һԪ���η��̵ĸ�
								tempt = 99;
								for (int j = 0; j < 3; j++)
								{
									if (r[j]<tempt && r[j]>minim)  //����С����ʱ��
									{
										tempt = r[j];
									}
								}

								//ˢ��ta1 ˢ��λ���ٶȼ��ٶ�
								ta1[i] = tempt;
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = q0[i] + dq0[i] * ta1[i] + 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

								//ˢ��ta2 ˢ��λ���ٶȼ��ٶ�
								ta2[i] = Ti[i] - ta1[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] + dq1[i] * ta2[i] + 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								//�����ڵ����͵���ʱ���
								ta3[i] = 0;
								ta4[i] = 0;
								//ˢ�³���
								tqf[i] = q2[i];
								dqf[i] = dq2[i];
								ddqf[i] = ddq2[i];
							}
							else	//q2С��qf ���ڵ�����
							{
								//ˢ��λ���ٶȼ��ٶ�
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] + dq2[i] * ta3[i] + 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);
								//�����ڵ��Ķ�ʱ��
								ta4[i] = 0;
								//ˢ�³���
								tqf[i] = q3[i];
								dqf[i] = dq3[i];
								ddqf[i] = ddq3[i];
							}
						}
						else//���ڵ��Ķ�
						{
							//ˢ��λ���ٶȼ��ٶ�
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] + ta4[i] * dq3[i];
							//ˢ�³���
							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
				}
				//ˢ����ʱ��
				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
				//ˢ�¸���ʱ��
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
				//ˢ�¸���ʱ��Ķ�Ӧ���η��̵�ϵ��
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
			else if (sigma[i] == 0) // ��ֹ�Ĵ���
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
		//maxT = max(tempT); //��tempT�����е����ֵ ������
		*maxT = -1;
		for (int i = 0; i < 6; i++)
		{
			if (*maxT < tempT[i])
			{
				*maxT = tempT[i];
			}

		}
	}
	else if (type == 2)// slowdown  �����ٶ�
	{
		for (int i = 0; i < dofn; i++)
		{
			//���ݹؽڵ��˶����� �ж�Jerk�ķ��ű���
			if (qf[i] - q0[i] > minim)  //��ת  ����ȡ1
			{
				sigma[i] = 1;
			}
			else if (qf[i] - q0[i] < -minim) //��ת ����ȡ - 1
			{
				sigma[i] = -1;
			}
			else
			{
				sigma[i] = 0; //��ֹ ����ȡ0
			}
			if (sigma[i] != 0 && Ti[i] == -1) // �ؽڴ����˶� ��ֻ��������ͬ���˶�ʱ�� �����������
			{
				ddq_max[i] = maxddq[i]; //���ٶȸ�ֵ
				dq_avg[i] = maxdq[i]; //ƽ���ٶȸ�ֵ

									  //time adjust
									  //stage 1  -- - acc - acc
									  //��һ�μ���  ����Ӽ��ٶ���Ӽ��ٵ�ʱ��ta1
									  //��Ӽ��ٵ�ʱ��ta1 ���ٶ�ddq1 �ٶ�dq1 ����ʱ��λ�� q1

				ta1[i] = (ddq0[i] - ddq_max[i]) / -(sigma[i] * Jm[i]);
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				//stage 2 -- - quansi - acc
				//�ڶ��μ���  �����ȼ��ٶ����ȼ��ٵ�ʱ��ta2
				//���ȼ��ٵ�ʱ��ta2 ���ٶ�ddq2 �ٶ�dq2 ����ʱ��λ�� q2
				//��Ҫ��ǰ��������������ʱ��ta3 �Լӵ�ƽ���ٶ�dq_avg��Ϊ�������������ta2

				ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);
				ta2[i] = (dq_avg[i] - ddq_max[i] * ta3[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2) + dq1[i]) / ddq_max[i];
				ddq2[i] = ddq1[i];
				dq2[i] = dq1[i] + ddq1[i] * ta2[i];
				q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

				//�ڶ��μ���Ľ���ʱ��λ�����С�ڳ�ʼ���λ�� ��Ҫ���¼���ta2
				if ((sigma[i] == 1 && q2[i] < q0[i]) || (sigma[i] == -1 && q2[i] > q0[i]))
				{
					// ��������������ĸ�ϵ��
					a = -1 / 2.0 * ddq1[i];
					b = -dq1[i];
					c = (q1[i] - q0[i]);
					lamda = b*b - 4 * a*c;
					if (lamda < 0)
					{
						printf("signification erreur"); //������ϵ������ ��ʾ������·����ֵʱ��̫�� ����С��900���� ����
						return -1;
					}

					newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a); //����Ķ������1
					newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a); //����Ķ������2

					if (newT[0] > minim && newT[1] > minim) // ����������� ȡ�̵�ʱ��������
					{
						if (newT[0] > newT[1]) // ta2ȡʱ���С��ֵ
						{
							ta2[i] = newT[0];
						}
						else
						{
							ta2[i] = newT[1];
						}
					}
					else if (newT[0] > minim && newT[1] < -minim) // ȡ����ʱ��ֵ
					{
						ta2[i] = newT[0];
					}
					else if (newT[1] > 1e-6 && newT[0] < -1e-6) // ȡ����ʱ��ֵ
					{
						ta2[i] = newT[1];
					}
					//���¼���ta2�� ���¼��ٶ� �ٶȺ�λ��
					ddq2[i] = ddq1[i];
					dq2[i] = dq1[i] + ddq1[i] * ta2[i];
					q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

					//�����ڵ����͵���ʱ���
					ta3[i] = 0;
					ta4[i] = 0;
					//���³���
					tqf[i] = q2[i];
					dqf[i] = dq2[i];
					ddqf[i] = ddq2[i];
					tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
					//������ʱ��
					//����ʱ�丳ֵ
					Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
					//����ϵ����ֵ
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
				//��������ٶ�
				//�����ٶε�ʱ��ta3���ȼ���ʱ�Ѽ��㣬ˢ�¼��ٶȣ��ٶȺ�λ��ֵ

				ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
				dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
				q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

				//�жϼ����ٽ������λ���Ƿ�С�ڳ�ʼ��λ�� ���С�� ���¼���ʱ��ta3
				if ((sigma[i] == 1 && q3[i] < q0[i]) || (sigma[i] == -1 && q3[i] > q0[i]))
				{
					// ���ݼ����ٶε�һԪ���η��̼��������ĸ�
					//���Ȼ�ø����ϵ��

					a3 = 1 / 6.0 * sigma[i] * Jm[i]; //a
					a2 = -1 / 2.0 * ddq2[i]; //b
					a1 = -dq2[i]; //c
					a0 = q2[i] - q0[i]; //d

										//                     r = roots([a3 a2 a1 a0]);
					getrootsofquadratic(a3, a2, a1, a0, r);  //����һԪ���η���
					tempt = 99; //���ȫ���Ľ�
					for (int j = 0; j < 3; j++)
					{
						if (r[j]<tempt && r[j]>minim) //����Ҫ��ȥ����ʱ�䣬ȡ��Сʱ��
						{
							tempt = r[j];
						}
					}
					ta3[i] = tempt;

					//����ˢ��ta3��λ�� �ٶ� �ͼ��ٶ�ֵ
					ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
					dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
					q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);
					//�����ڵ���ʱ���
					ta4[i] = 0;
					//���¸ùؽڵĽ�����λ�� �ٶ� ���ٶ�
					tqf[i] = q3[i];
					dqf[i] = dq3[i];
					ddqf[i] = ddq3[i];
					//�õ���ʱ��
					tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
					//����ʱ��ϵ����ˢ��
					Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
					//����������ϵ����ˢ��
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
				//���Ķ� ���ټ���
				//�������ٵ�ta4 �Լ�λ���ٶ� ���ٶ�ֵ
				//�����жϼ����ٽ������Ƿ����Ŀ��λ�ã���ʼ��λ�ã� ���� ��������ٶ�

				if ((sigma[i] == 1 && q3[i] > q0[i]) || (sigma[i] == -1 && q3[i] < q0[i]))
				{
					// �õ����ٶε�ta4 λ�ú��ٶ� ���ٶ�ֵ
					ta4[i] = (q3[i] - q0[i]) / dq3[i];
					ddq4[i] = 0;
					dq4[i] = dq3[i];
					q4[i] = q3[i] - ta4[i] * dq3[i];
					//���·��ص�λ���ٶȼ��ٶ�ֵ
					tqf[i] = q4[i];
					dqf[i] = dq4[i];
					ddqf[i] = ddq4[i];
				}

				//������ʱ��
				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];

				//���¸���ʱ��ϵ��
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
				//���¸���Ķ���ʽϵ��
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
			else if (sigma[i] != 0 && Ti[i] > 0)// ��һ���Ǹ��ݶ���ʱ��ͬ���� ������ε�ʱ��ϵ���Ͷ���ʽϵ�� �����ؽ�����λ���ٶ� ���ٶ�
			{
				ddq_max[i] = maxddq[i]; //���ٶȸ�ֵ
				dq_avg[i] = maxdq[i]; //ƽ���ٶȸ�ֵ

									  //time adjust
									  //stage 1  -- - acc - acc
									  //����Ӽ��ٶ���Ӽ��ٵ�ʱ��ta1 ˢ��λ���ٶȼ��ٶ�
				ta1[i] = (ddq0[i] - ddq_max[i]) / -(sigma[i] * Jm[i]);
				ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
				dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
				q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

				//�����ȼ��ٶ�ta2ǰ ��Ҫ���������ʱ��ta3
				ta3[i] = ddq_max[i] / (sigma[i] * Jm[i]);

				//���ȸ���ʱ��Ti��Լ�� ��ta4�ĺ���ֵ
				//��������������̵Ķ��������ϵ��

				a = 1 / 2.0 * ddq1[i];
				b = -1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2) - ddq1[i] * (Ti[i] - ta1[i] - ta3[i]);
				//                 b = 1 / 2 * sigma[i]*Jm[i]*ta3[i] ^ 2 + ddq1[i]*Ti[i];
				c = q1[i] - q0[i] - dq1[i] * ta3[i] - 1 / 2.0 * ddq1[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3) - dq1[i] * (Ti[i] - ta1[i] - ta3[i]) - ddq1[i] * ta3[i] * (Ti[i] - ta1[i] - ta3[i]) + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2) * (Ti[i] - ta1[i] - ta3[i]);
				//                 c = q0[i] + dq0[i]*ta1[i] + ta3[i]*((sigma[i]*Jm[i]*ta1[i] ^ 2) / 2 + ddq0[i]*ta1[i] + dq0[i]) + ta3[i] ^ 2 * (ddq0[i] / 2 + (sigma[i]*Jm[i]*ta1[i]) / 2) + (ddq0[i]*ta1[i] ^ 2) / 2 + (sigma[i]*Jm[i]*ta1[i] ^ 3) / 6 - (sigma[i]*Jm[i]*ta3[i] ^ 3) / 6 + Ti[i]*(dq0[i] + ddq0[i]*ta1[i] + (sigma[i]*Jm[i]*ta1[i] ^ 2) / 2 - (sigma[i]*Jm[i]*ta3[i] ^ 2) / 2 + ta3[i]*(ddq0[i] + sigma[i]*Jm[i]*ta1[i])) - qf[i];
				lamda = b*b - 4 * a*c;
				if (lamda <= 0)// lamdaֵС��0 ��ʾta2�޽� ��Ҫ����ta1 ta3 ���
				{
					//���¼���ta2 ˢ��λ���ٶȼ��ٶ�
					ta2[i] = Ti[i] - ta1[i] - ta3[i];
					ddq2[i] = ddq1[i];
					dq2[i] = dq1[i] + ddq1[i] * ta2[i];
					q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
					//����ȼ��ٺ�λ�ó�����һ��λ�� �������η������¼���ta2��ʱ��
					if ((sigma[i] == 1 && q2[i] < q0[i]) || (sigma[i] == -1 && q2[i] > q0[i]))
					{
						// ��������ta1��ta2���̺� ���η��̵ĸ���ϵ��  ���������ta1ֵ
						a3 = 1 / 3.0 * sigma[i] * Jm[i]; //a
						a2 = 1 / 2.0 * sigma[i] * Jm[i] * Ti[i] + 1 / 2.0 * ddq0[i]; //b
						a1 = -1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
						a0 = -1 / 2.0 * ddq0[i] * pow(Ti[i], 2) - dq0[i] * Ti[i] + qf[i] - q0[i]; //d

																								  //                         r = roots([a3 a2 a1 a0]);
						getrootsofquadratic(a3, a2, a1, a0, r); //�����η��̵ĸ�
						tempt = 99;
						for (int j = 0; j < 3; j++)
						{
							if (r[j]<tempt && r[j]>minim) //��ʱ����С��Ϊ���ĸ�
							{
								tempt = r[j];
							}
						}
						//���ݷ��̵ĸ� ˢ��ta1 �Լ�λ���ٶ� ���ٶ�
						ta1[i] = tempt;
						ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
						dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
						q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);
						//���¼���ta2 ˢ��λ���ٶȼ��ٶ�
						ta2[i] = Ti[i] - ta1[i];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
						//�����ڵ����͵���ʱ���
						ta3[i] = 0;
						ta4[i] = 0;
						//���³���
						tqf[i] = q2[i];
						dqf[i] = dq2[i];
						ddqf[i] = ddq2[i];
					}
					else//���ڵ�����ta3 q2С��qf
					{
						//                     ta3[i] = Ti[i] - ta1[i] - ta2[i];
						//ˢ�¼����ٽ������λ���ٶȼ��ٶ�
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//�����ڵ��Ķ�ʱ��
						ta4[i] = 0;
						//���³���
						tqf[i] = q3[i];
						dqf[i] = dq3[i];
						ddqf[i] = ddq3[i];
					}
				}
				else //���ڵ��Ķ� ta2  lamda����0
				{
					newT[0] = -b / (2 * a) + sqrt(lamda) / (2 * a);
					newT[1] = -b / (2 * a) - sqrt(lamda) / (2 * a);

					//�������ʱ��ⶼΪ�� ���ν��м��� ���ж�
					if ((newT[0] > minim && newT[1] > minim))
					{
						// if newT(1)>newT(2)
						// ta2[i] = newT(2);
						//                         else
						//����ȡ��1 ˢ��ta2 ˢ��λ���ٶȼ��ٶ�
						ta2[i] = newT[0];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//����ta3 ˢ��λ���ٶȼ��ٶ�
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//ˢ��ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];
						//��������ta4 Ϊ�� ��ʾȡ�ýⲻ���� ����2 ���¼���
						if (ta4[i] < 0)
						{
							// ����2 ����λ���ٶȼ��ٶ�
							ta2[i] = newT[1];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

							//����ta3λ���ٶȼ��ٶ�
							ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
							dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
							q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

							//����ta4
							ta41 = Ti[i] - ta2[i] - ta1[i] - ta3[i];

							//��������ⶼ������ta4 ��ȥ������ʱ��� ���µ���ta2
							if (ta41 < 0)
							{
								// ���¼���ta2 ˢ��λ���ٶȼ��ٶ�
								ta2[i] = Ti[i] - ta1[i] - ta3[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
								//���q2��λ�ô�����һ��λ�� ��Ҫ�������η������¼���ta1��ta2
								if ((sigma[i] == 1 && q2[i] < q0[i]) || (sigma[i] == -1 && q2[i] > q0[i]))
								{
									// �������η���
									a3 = 1 / 3.0 * sigma[i] * Jm[i]; //a
									a2 = 1 / 2.0 * sigma[i] * Jm[i] * Ti[i] + 1 / 2.0 * ddq0[i]; //b
									a1 = -1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
									a0 = -1 / 2.0 * ddq0[i] * pow(Ti[i], 2) - dq0[i] * Ti[i] + qf[i] - q0[i]; //d


									getrootsofquadratic(a3, a2, a1, a0, r); //�����η��̵ĸ�
									tempt = 99;
									for (int j = 0; j < 3; j++)
									{
										if (r[j]<tempt && r[j]>minim) //ȡ��С����ʱ��ĸ�
										{
											tempt = r[j];
										}
									}

									//ˢ��ta1 �Լ�λ���ٶȼ��ٶ�
									ta1[i] = tempt;
									ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
									dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
									q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

									//ˢ��ta2 �Լ�λ���ٶȼ��ٶ�
									ta2[i] = Ti[i] - ta1[i];
									ddq2[i] = ddq1[i];
									dq2[i] = dq1[i] + ddq1[i] * ta2[i];
									q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

									//�����ڵ����͵���ʱ���
									ta3[i] = 0;
									ta4[i] = 0;
									//ˢ�³���
									tqf[i] = q2[i];
									dqf[i] = dq2[i];
									ddqf[i] = ddq2[i];
								}
								else//�ȼ��ٺ��λ�ò�������һ��λ�� ��������ٶ�
								{

									//����ta3�ε�λ���ٶȼ��ٶ�ֵ
									ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
									dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
									q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

									//�����ڵ���ʱ���
									ta4[i] = 0;
									//���³���
									tqf[i] = q3[i];
									dqf[i] = dq3[i];
									ddqf[i] = ddq3[i];
								}
							}
							else //ta4����0���ڵ��Ķ� ˢ��λ���ٶȼ��ٶ� ˢ�³���
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
						else //ta4����0���ڵ��Ķ� ˢ��λ���ٶȼ��ٶ� ˢ�³���
						{
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] - ta4[i] * dq3[i];
							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
					else if (newT[0] > minim && newT[1] < -minim) // ��1 Ϊ�� ��2Ϊ�� ȡ��1
					{
						//ˢ��ta2 ˢ��λ���ٶȼ��ٶ�
						ta2[i] = newT[0];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//ˢ��ta3 ��λ���ٶȼ��ٶ�
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//����ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];
						//���ta4С��0 ��ʾǰ����Ľⲻ���� ���¼���
						if (ta4[i] < 0)
						{
							// ȥ��ta4 ���¼���ta2 ˢ��λ���ٶȼ��ٶ�
							ta2[i] = Ti[i] - ta1[i] - ta3[i];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
							//���q2��λ�ô�����һ��λ�� ����q1��q2�ķ����� �õ�һԪ���η���
							if ((sigma[i] == 1 && q2[i] < q0[i]) || (sigma[i] == -1 && q2[i] > q0[i]))
							{
								// ����һԪ���η��̵�ϵ��
								a3 = 1 / 3.0 * sigma[i] * Jm[i]; //a
								a2 = 1 / 2.0 * sigma[i] * Jm[i] * Ti[i] + 1 / 2.0 * ddq0[i]; //b
								a1 = -1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
								a0 = -1 / 2.0 * ddq0[i] * pow(Ti[i], 2) - dq0[i] * Ti[i] + qf[i] - q0[i]; //d


								getrootsofquadratic(a3, a2, a1, a0, r); //��һԪ���η��̵ĸ�
								tempt = 99;
								for (int j = 0; j < 3; j++)
								{
									if (r[j] < tempt && r[j] > minim) //����С����ʱ��
									{
										tempt = r[j];
									}
								}
								//ˢ��ta1 ˢ��λ���ٶȼ��ٶ�
								ta1[i] = tempt;
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);

								//ˢ��ta2 ˢ��λ���ٶȼ��ٶ�
								ta2[i] = Ti[i] - ta1[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

								//�����ڵ����͵���ʱ���
								ta3[i] = 0;
								ta4[i] = 0;
								//ˢ�³���
								tqf[i] = q2[i];
								dqf[i] = dq2[i];
								ddqf[i] = ddq2[i];
							}
							else//q2С��qf ���ڵ�����
							{
								//                     ta3[i] = Ti[i] - ta1[i] - ta2[i];
								//ˢ��λ���ٶȼ��ٶ�
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

								//�����ڵ��Ķ�ʱ��
								ta4[i] = 0;
								//ˢ�³���
								tqf[i] = q3[i];
								dqf[i] = dq3[i];
								ddqf[i] = ddq3[i];
							}
						}
						else//���ڵ��Ķ�
						{
							//ˢ��λ���ٶȼ��ٶ�
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] - ta4[i] * dq3[i];
							//ˢ�³���
							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
					else if (newT[1] > minim && newT[0] < -minim) // ��2 Ϊ�� ��1Ϊ�� ȡ��2
					{
						//ˢ��ta2 ˢ��λ���ٶȼ��ٶ�
						ta2[i] = newT[1];
						ddq2[i] = ddq1[i];
						dq2[i] = dq1[i] + ddq1[i] * ta2[i];
						q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);

						//ˢ��ta3 ��λ���ٶȼ��ٶ�
						ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
						dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
						q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);

						//����ta4
						ta4[i] = Ti[i] - ta2[i] - ta1[i] - ta3[i];
						//���ta4С��0 ��ʾǰ����Ľⲻ���� ���¼���
						if (ta4[i] < 0)
						{
							// ȥ��ta4 ���¼���ta2 ˢ��λ���ٶȼ��ٶ�
							ta2[i] = Ti[i] - ta1[i] - ta3[i];
							ddq2[i] = ddq1[i];
							dq2[i] = dq1[i] + ddq1[i] * ta2[i];
							q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
							//���q2��λ�ô�����һ��λ�� ����q1��q2�ķ����� �õ�һԪ���η���
							if ((sigma[i] == 1 && q2[i] < q0[i]) || (sigma[i] == -1 && q2[i] > q0[i]))
							{
								// ����һԪ���η��̵�ϵ��
								a3 = 1 / 3.0 * sigma[i] * Jm[i]; //a
								a2 = 1 / 2.0 * sigma[i] * Jm[i] * Ti[i] + 1 / 2.0 * ddq0[i]; //b
								a1 = -1 / 2.0 * sigma[i] * Jm[i] * pow(Ti[i], 2); //c
								a0 = -1 / 2.0 * ddq0[i] * pow(Ti[i], 2) - dq0[i] * Ti[i] + qf[i] - q0[i]; //d

																										  //                                 r = roots([a3 a2 a1 a0]);
								getrootsofquadratic(a3, a2, a1, a0, r); //��һԪ���η��̵ĸ�
								tempt = 99;
								for (int j = 0; j < 3; j++)
								{
									if (r[j]<tempt && r[j]>minim) //����С����ʱ��
									{
										tempt = r[j];
									}
								}
								//ˢ��ta1 ˢ��λ���ٶȼ��ٶ�
								ta1[i] = tempt;
								ddq1[i] = ddq0[i] + sigma[i] * Jm[i] * ta1[i];
								dq1[i] = dq0[i] + ddq0[i] * ta1[i] + 1 / 2.0 * sigma[i] * Jm[i] * pow(ta1[i], 2);
								q1[i] = qf[i] - dq0[i] * ta1[i] - 1 / 2.0 * ddq0[i] * pow(ta1[i], 2) - 1 / 6.0 * sigma[i] * Jm[i] * pow(ta1[i], 3);
								//ˢ��ta2 ˢ��λ���ٶȼ��ٶ�
								ta2[i] = Ti[i] - ta1[i];
								ddq2[i] = ddq1[i];
								dq2[i] = dq1[i] + ddq1[i] * ta2[i];
								q2[i] = q1[i] - dq1[i] * ta2[i] - 1 / 2.0 * ddq1[i] * pow(ta2[i], 2);
								//�����ڵ����͵���ʱ���
								ta3[i] = 0;
								ta4[i] = 0;
								//                                 continue;
								//ˢ�³���
								tqf[i] = q2[i];
								dqf[i] = dq2[i];
								ddqf[i] = ddq2[i];
							}
							else //q2С��qf ���ڵ�����
							{
								//                     ta3[i] = Ti[i] - ta1[i] - ta2[i];
								//ˢ��λ���ٶȼ��ٶ�
								ddq3[i] = ddq2[i] - sigma[i] * Jm[i] * ta3[i];
								dq3[i] = dq2[i] + ddq2[i] * ta3[i] - 1 / 2.0 * sigma[i] * Jm[i] * pow(ta3[i], 2);
								q3[i] = q2[i] - dq2[i] * ta3[i] - 1 / 2.0 * ddq2[i] * pow(ta3[i], 2) + 1 / 6.0 * sigma[i] * Jm[i] * pow(ta3[i], 3);
								//�����ڵ��Ķ�ʱ��
								ta4[i] = 0;
								//ˢ�³���
								tqf[i] = q3[i];
								dqf[i] = dq3[i];
								ddqf[i] = ddq3[i];
							}
						}
						else//���ڵ��Ķ�
						{
							//ˢ��λ���ٶȼ��ٶ�
							ddq4[i] = 0;
							dq4[i] = dq3[i];
							q4[i] = q3[i] - ta4[i] * dq3[i];
							//ˢ�³���
							tqf[i] = q4[i];
							dqf[i] = dq4[i];
							ddqf[i] = ddq4[i];
						}
					}
				}
				//ˢ����ʱ��
				tempT[i] = ta1[i] + ta2[i] + ta3[i] + ta4[i];
				//ˢ�¸���ʱ��
				Ta[i][0] = ta1[i]; Ta[i][1] = ta2[i]; Ta[i][2] = ta3[i]; Ta[i][3] = ta4[i];
				//ˢ�¸���ʱ��Ķ�Ӧ���η��̵�ϵ��
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
			else if (sigma[i] == 0) // ��ֹ�Ĵ���
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
// ���ƣ�forwardCycle
// ���ܣ�����������������㣻
// ���룺start - ��ʼ���λ�á��ٶȡ����ٶ�
//       target - ��ĩ���λ��
//       exp_duration - ��������ʱ��
//       alpha_upper - ����Ǽ��ٶ����ֵ
//       alpha_lower - ����Ǽ��ٶ����ֵ
// �����segment - ���ι켣�ķֶα��ʽϵ����ʱ��
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
// ���ƣ�backwardCycle
// ���ܣ������ٷ���������㣻
// ���룺start - ��ʼ���λ��
//       target - ��ĩ���λ�á��ٶȡ����ٶ�
//       exp_duration - ��������ʱ��
//       alpha_upper - ����Ǽ��ٶ����ֵ
//       alpha_lower - ����Ǽ��ٶ����ֵ
// �����segment - ���ι켣�ķֶα��ʽϵ����ʱ��
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
