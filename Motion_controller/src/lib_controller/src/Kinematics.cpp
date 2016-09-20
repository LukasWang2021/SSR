//#include <iostream>
//#include <math.h>
#include <lib_controller/trajplan.h>


//模拟输入：分别为位置、姿态（四元数）和当前位置；

//double POSITION[3] = { 353.8925, 3.5161, 980.1559 };
//double ORIENTATION[4] = { 0.4545, -0.2325, 0.2044, -0.8352 };
//double JOINTS_INI[6] = { 0, 0, 0, 0, 0, 0 };

//机器人条件,也做出接口，供修改；保留默认值，无修改则取默认值；
//尺寸参数（DH参数），从TCP到腕关节中心的变换矩阵（工具坐标系），各轴限位；


/*namespace fst_controller
{
	//  extern double pi ;
	double DH[6][4] = { { 0, 0, 330.0, 0 }, { pi / 2, 50, 0, pi / 2 }, { 0, 330, 0, 0 }, { pi / 2, 35, 330, 0 }, { -pi / 2, 0, 0, 0 }, { pi / 2, 0, 77.5, 0 } };

double a1 = DH[1][1];
double a2 = DH[2][1];
double a3 = DH[3][1];
double d1 = DH[0][2];
double d3 = DH[2][2];
double d4 = DH[3][2];

double offset1 = DH[0][3];
double offset2 = DH[1][3];
double offset3 = DH[2][3];
double offset4 = DH[3][3];
double offset5 = DH[4][3];
double offset6 = DH[5][3];

double UserFrame[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
double InvUF[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
double ToolFrame[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 93 }, { 0, 0, 0, 1 } };
double InvTF[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, -93 }, { 0, 0, 0, 1 } };
double InvT66[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, -DH[5][2] }, { 0, 0, 0, 1 } };
double AXIS_LIMIT[2][6] = { { -2.9671, -2.2689, -1.1694, -3.3161, -1.8500, -6.2832 }, { 2.9671, 1.3090, 3.2289, 3.3161, 1.8500, 6.2832 } };

}*/

fst_controller::TrajPlan::TrajPlan()
{
	 pi = 3.1415926535897932384626433832795;
	 acc = 5000;
	 tc = 0.001;
	 limit_scale = 1;
	 overshoot = 0.25;
	 errorangle = 0.15;
	 accelerationoverload = 1;

	 double DH_0[6][4] = { { 0, 0, 330.0, 0 }, { pi / 2, 50, 0, pi / 2 }, { 0, 330, 0, 0 }, { pi / 2, 35, 330, 0 }, { -pi / 2, 0, 0, 0 }, { pi / 2, 0, 77.5, 0 } };
//	 double DH_0[6][4] = { { 0, 0, 330.0, 0 }, { pi / 2, 50, 0, pi / 2 }, { 0, 330, 0, 0 }, { pi / 2, 35, 330, 0 }, { -pi / 2, 0, 0, 0 }, { pi / 2, 0, 84.5, 0 } };
	 memcpy(DH, DH_0, 24 * sizeof(double));
	 a1 = DH[1][1];
	 a2 = DH[2][1];
	 a3 = DH[3][1];
	 d1 = DH[0][2];
	 d3 = DH[2][2];
	 d4 = DH[3][2];

	 offset1 = DH[0][3];
	 offset2 = DH[1][3];
	 offset3 = DH[2][3];
	 offset4 = DH[3][3];
	 offset5 = DH[4][3];
	 offset6 = DH[5][3];

	 double UserFrame_0[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
	 memcpy(UserFrame, UserFrame_0, 16 * sizeof(double));
	 double InvUF_0[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
	 memcpy(InvUF, InvUF_0, 16 * sizeof(double));
	 double  ToolFrame_0[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
	 memcpy(ToolFrame, ToolFrame_0, 16 * sizeof(double));
	 double   InvTF_0[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1,0 }, { 0, 0, 0, 1 } };
	 memcpy(InvTF, InvTF_0, 16 * sizeof(double));
	 double  InvT66_0[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, -DH[5][2] }, { 0, 0, 0, 1 } };
	 memcpy(InvT66, InvT66_0, 16 * sizeof(double));
	 double  AXIS_LIMIT_0[4][6] = { { -2.9671, -2.2689, -1.1694, -3.3161, -1.8500, -6.2832 },
	 { 2.9671, 1.3090, 3.2289, 3.3161, 1.8500, 6.2832 },
	 //	 {5.1,4.6,5.8,8.7,6.2,11}, {6.4, 5.8, 10, 16,15, 25} };
	 { 5.5851, 6.2832, 6.9813, 6.2832, 4.1888, 12.5664 }, {	 6.9813, 7.8540, 13.9626, 12.5664, 10.4720, 31.4159} };
	 memcpy(AXIS_LIMIT, AXIS_LIMIT_0, 24 * sizeof(double));
}

void fst_controller::TrajPlan::setDH(double _alp0, double _a0, double _d1, double _os1, double _alp1, double _a1, double _d2, double _os2, double _alp2, double _a2, double _d3, double _os3, double _alp3, double _a3, double _d4, double _os4, double _alp4, double _a4, double _d5, double _os5, double _alp5, double _a5, double _d6, double _os6)
{
	DH[0][0] = _alp0; DH[0][1] = _a0; DH[0][2] = _d1; DH[0][3] = _os1;
	DH[1][0] = _alp1; DH[1][1] = _a1; DH[1][2] = _d2; DH[1][3] = _os2;
	DH[2][0] = _alp2; DH[2][1] = _a2; DH[2][2] = _d3; DH[2][3] = _os3;
	DH[3][0] = _alp3; DH[3][1] = _a3; DH[3][2] = _d4; DH[3][3] = _os4;
	DH[4][0] = _alp4; DH[4][1] = _a4; DH[4][2] = _d5; DH[4][3] = _os5;
	DH[5][0] = _alp5; DH[5][1] = _a5; DH[5][2] = _d6; DH[5][3] = _os6;

	d1 = _d1;
	d4 = _d4;
	d3 = _d3;
	a1 =	_a1;
	a2 = _a2;
	a3 = _a3;
	offset2 = _os2;
}

void fst_controller::TrajPlan::setToolFrame(const fst_controller::Point &position, const fst_controller::Euler &orientation)
{
	
	Euler2Matrix(position.x, position.y, position.z, orientation.a, orientation.b, orientation.c, ToolFrame);
	InverseMatrix(ToolFrame, InvTF);
}

void fst_controller::TrajPlan::setUserFrame(const fst_controller::Point &position, const fst_controller::Euler &orientation)
{
	Euler2Matrix(position.x, position.y, position.z, orientation.a, orientation.b, orientation.c, UserFrame);
	InverseMatrix(UserFrame, InvUF);
}

void fst_controller::TrajPlan::setAxisLimit(const fst_controller::JointConstraints &limit)
{
	AXIS_LIMIT[0][0] = limit.j1.lower; AXIS_LIMIT[1][0] = limit.j1.upper; 
	AXIS_LIMIT[2][0] = limit.j1.max_omega; AXIS_LIMIT[3][0] = limit.j1.max_alpha;
	AXIS_LIMIT[0][1] = limit.j2.lower; AXIS_LIMIT[1][1] = limit.j2.upper;
	AXIS_LIMIT[2][1] = limit.j2.max_omega; AXIS_LIMIT[3][1] = limit.j2.max_alpha;
	AXIS_LIMIT[0][2] = limit.j3.lower; AXIS_LIMIT[1][2] = limit.j3.upper;
	AXIS_LIMIT[2][2] = limit.j3.max_omega; AXIS_LIMIT[3][2] = limit.j3.max_alpha;
	AXIS_LIMIT[0][3] = limit.j4.lower; AXIS_LIMIT[1][3] = limit.j4.upper;
	AXIS_LIMIT[2][3] = limit.j4.max_omega; AXIS_LIMIT[3][3] = limit.j4.max_alpha;
	AXIS_LIMIT[0][4] = limit.j5.lower; AXIS_LIMIT[1][4] = limit.j5.upper;
	AXIS_LIMIT[2][4] = limit.j5.max_omega; AXIS_LIMIT[3][4] = limit.j5.max_alpha;
	AXIS_LIMIT[0][5] = limit.j6.lower; AXIS_LIMIT[1][5] = limit.j6.upper;
	AXIS_LIMIT[2][5] = limit.j6.max_omega; AXIS_LIMIT[3][5] = limit.j6.max_alpha;
}
// 各轴变化限制，与轴的最高速度成正比,取值0~1
void fst_controller::TrajPlan::setLimitScale(double _limit_scale)
{
	limit_scale = _limit_scale;
}

//设定各关节最大允许超调量；
void fst_controller::TrajPlan::setOvershoot(double _overshoot)
{
	overshoot= _overshoot;
}

//设定关节空间规划时允许接近限位的角度
void fst_controller::TrajPlan::seterrorangle(double _errorangle)
{
	errorangle= _errorangle;
}

//设定关节加速度过载系数
void fst_controller::TrajPlan::setoverload(double _accelerationoverload)
{
	accelerationoverload = _accelerationoverload;
}

//修正关节变量，使其位于-pi~pi 范围内
double fst_controller::TrajPlan::REVISE_JOINT(double t)
{
	t = t - round(t / pi / 2) * 2 * pi;
	return t;
}

// 判断关节是否超过限位
int fst_controller::TrajPlan::JUDGE_AXIS(double THETA[6])		//判断关节是否超出限位；
{
	int k = 0;
	for (int i = 0; i < 6; i++)
	{
		if ((THETA[i] <= AXIS_LIMIT[0][i]) | (THETA[i] >= AXIS_LIMIT[1][i]))
		{
			k = 1;
			break;
		}
	}
	return k;
}


int fst_controller::TrajPlan::ForwardKinematics(const fst_controller::JointValues &joint,fst_controller::Pose &pose)
{
	double JOINTS[6] = { joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6 };
	int flag = JUDGE_AXIS(JOINTS);
	if (flag==1)
	{
		return 1011;
	}
	else
	{
		double Pose_M[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
		double tmp1[4][4] = {};
		double L[4];
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				L[j] = DH[i][j];
			}

			DHMatrix(L, JOINTS[i], &tmp1[0]);
			MULTIPLY_M(Pose_M, tmp1, &Pose_M[0]);
		}

		MULTIPLY_M(InvUF, Pose_M, &Pose_M[0]);
		MULTIPLY_M(Pose_M, ToolFrame, &Pose_M[0]);
		double pst[3] = {};
		double ort[4] = {};
		MATRIX2POSE(Pose_M, &pst[0], &ort[0]);
		pose.position.x = pst[0]; pose.position.y = pst[1]; pose.position.z = pst[2];
		pose.orientation.w = ort[0]; pose.orientation.x = ort[1]; pose.orientation.y = ort[2]; pose.orientation.z = ort[3];

		return 0;
	}
	
}



int fst_controller::TrajPlan::InverseKinematics(const fst_controller::Pose &pose,const fst_controller::JointValues &joint_reference, fst_controller::JointValues &joint_result)
{
	double POSE_TCP[4][4] = {};
	double POSE_WRIST[4][4] = {};		//腕关节处位姿矩阵，
	double POSITION[3] = { pose.position.x, pose.position.y, pose.position.z };
	double ORIENTATION[4] = { pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z };
	double JOINTS_INI[6] = { joint_reference.j1, joint_reference.j2, joint_reference.j3, joint_reference.j4, joint_reference.j5, joint_reference.j6 };
	POSE2MATRIX(POSITION, ORIENTATION, POSE_TCP);
	MULTIPLY_M(UserFrame, POSE_TCP, POSE_WRIST);
	MULTIPLY_M(POSE_WRIST, InvTF, POSE_WRIST);
	MULTIPLY_M(POSE_WRIST, InvT66, POSE_WRIST);
	double SOL_OUT[6] = {};

	double nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz;
	nx = POSE_WRIST[0][0];
	ny = POSE_WRIST[1][0];
	nz = POSE_WRIST[2][0];
	ox = POSE_WRIST[0][1];
	oy = POSE_WRIST[1][1];
	oz = POSE_WRIST[2][1];
	ax = POSE_WRIST[0][2];
	ay = POSE_WRIST[1][2];
	az = POSE_WRIST[2][2];
	px = POSE_WRIST[0][3];
	py = POSE_WRIST[1][3];
	pz = POSE_WRIST[2][3];


	double t1, t2, t3, t4, t5, t6;
	double c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6;
	double s23, c23, c4s5, s4s5, t31, t32, t41, t42;
	double k1, k2, k, b1, b2;
	double JOINTS_SOL[8][6] = {};


	double mn1 = px*px + py*py;
	double mn2 = d3*d3;
	int i = 0;			//设立标志i，如果i为0，表明无解。

	if (mn2 > mn1)
	{
		return 1001;		//t1无解，结束计算
	}

	else
	{
		double t11 = atan2(d3, (mn1 - mn2) / 2) + atan2(py, px);
		double t12 = atan2(d3, -(mn1 - mn2) / 2) + atan2(py, px);
		double THETA1[2] = { t11, t12 };
		for (int i1 = 0; i1 <= 1; ++i1)
		{

			t1 = THETA1[i1];

			c1 = cos(t1);
			s1 = sin(t1);
			k1 = pz - d1;
			k2 = px*c1 + py*s1 - a1;
			k = (k1*k1 + k2*k2 - a2*a2 - a3*a3 - d4*d4) / a2 / 2;

			double mp1 = a3*a3 + d4*d4;
			double mp2 = k*k;
			if (mp2> mp1)
			{
				continue;		//	t3无解，进行下一次循环
			};
			double mp3 = atan2(k, pow((mp1 - mp2), 0.5));
			double mp4 = atan2(a3, d4);
			double mp5 = atan2(k, -pow((mp1 - mp2), 0.5));
			t31 = mp3 - mp4;
			t32 = mp5 - mp4;
			double THETA3[2] = { t31, t32 };
			
			for (int i2 = 0; i2 <= 1; ++i2)
			{

				t3 = THETA3[i2];
				c3 = cos(t3);
				s3 = sin(t3);
				b1 = a3 + c3*a2;
				b2 = d4 + s3*a2;
				s23 = (k1*b1 + k2*b2) / (k1*k1 + k2*k2);
				c23 = (-k1*b2 + k2*b1) / (k1*k1 + k2*k2);
				t2 = atan2(s23, c23) - t3;
				c2 = cos(t2); s2 = sin(t2);
				c5 = c1*s23*ax + s1*s23*ay - c23*az;
				if (c5<0.999999&&c5>-0.999999)
				{
					c4s5 = c1*c23*ax + c23*s1*ay + s23*az;
					s4s5 = s1*ax - c1*ay;
					t41 = atan2(s4s5, c4s5);
					t42 = atan2(-s4s5, -c4s5);
				} 
				else
				{
					t41 = 0;
					t42 = 0;
				}

				double THETA4[2] = { t41, t42 };

				
				for (int i4 = 0; i4 <= 1; ++i4)
				{
					t4 = THETA4[i4];
					s4 = sin(t4);
					c4 = cos(t4);
					s5 = (c1*c23*c4 + s1*s4)*ax + (s1*c2*c3*c4 - s1*s2*s3*c4 - c1*s4)*ay + s23*c4*az;
					t5 = atan2(s5, c5);
					s6 = (c4*s1 - c1*c23*s4)*nx - (c1*c4 + c23*s1*s4)*ny - s23*s4*nz;
					c6 = (c4*s1 - c1*c23*s4)*ox - (c1*c4 + c23*s1*s4)*oy - s23*s4*oz;
					t6 = atan2(s6, c6);
					if (fabs(s5) < 1e-15)
					{
						t4 = (t4 + t6) / 2;
						t6 = t4;
					}
					double J[6] = { REVISE_JOINT(t1-offset1), REVISE_JOINT(t2 - offset2), REVISE_JOINT(t3 - offset3), REVISE_JOINT(t4 - offset4), REVISE_JOINT(t5 - offset5), REVISE_JOINT(t6 - offset6) };
					int flag = JUDGE_AXIS(J);

					if (flag==0)
					{
						JOINTS_SOL[i][0] = t1 - offset1;
						JOINTS_SOL[i][1] = t2 - offset2;
						JOINTS_SOL[i][2] = t3 - offset3;
						JOINTS_SOL[i][3] = t4 - offset4;
						JOINTS_SOL[i][4] = t5 - offset5;
						JOINTS_SOL[i][5] = t6 - offset6;
//					cout << "Solution " << i + 1 << ":	";
//						cout.precision(3);
//						cout.setf(ios::fixed);
					for (int j = 0; j <= 5; j++)
						{
							JOINTS_SOL[i][j] = J[j];
//						cout << JOINTS_SOL[i][j] << "	";
						};
//					cout << endl;
						i++;
					}

				};

			};

		};

	};
//double SOL_OUT[6] = {};
//	i;

	if (i)				//有解时，通过与当前姿态的距离取最优解；
						//i作为输出值，i!=0，有解，且个数为i;i=0,无解;
	{
		int minsol = 0;

		double dist0 = 24 * pow(pi,2);
		for (int k = 0; k < i; k++)
		{
			double dist = 0;
			for (int j = 0; j <= 5; j++)
			{
				dist = dist + pow((JOINTS_SOL[k][j] - JOINTS_INI[j]), 2);
			}
			if (dist < dist0)
			{
				minsol = k;
				dist0 = dist;
			};
		};
//	cout << "Nearest Solution:" << endl;
		double d_Sol[6] = {};
		int flag1 = 0;
		for (int i = 0; i <= 5; i++)
		{
			SOL_OUT[i] = JOINTS_SOL[minsol][i];			//最终给出的结果
			d_Sol[i] = fabs(SOL_OUT[i] - JOINTS_INI[i]);
			if (d_Sol[i]>AXIS_LIMIT[2][i]*tc*limit_scale)
			{
				flag1 = 1;
			} 
//		cout << SOL_OUT[i] << '	';
		};
		
//	cout << endl;
		if (flag1)
		{
			joint_result.j1 = SOL_OUT[0];
			joint_result.j2 = SOL_OUT[1];
			joint_result.j3 = SOL_OUT[2];
			joint_result.j4 = SOL_OUT[3];
			joint_result.j5 = SOL_OUT[4];
			joint_result.j6 = SOL_OUT[5];
			return 1003;														// 根据速度判断点距，点距过大
		}
		else
		{
			joint_result.j1 = SOL_OUT[0];
			joint_result.j2 = SOL_OUT[1];
			joint_result.j3 = SOL_OUT[2];
			joint_result.j4 = SOL_OUT[3];
			joint_result.j5 = SOL_OUT[4];
			joint_result.j6 = SOL_OUT[5];
			return 0;
		}
		
	}
	else     //无解。
	{
//		cout << "No Solution!" << endl;

		for (int i = 0; i <= 5; i++)
		{
			SOL_OUT[i] = JOINTS_INI[i];				//无解情况返回机器人当前的关节向量
	//		cout << SOL_OUT[i] << '	';
		};
	//	cout << endl;
		return 1002;													//无解在轴限制之内
	};


};



