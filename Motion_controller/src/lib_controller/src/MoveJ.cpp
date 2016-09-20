#include <lib_controller/trajplan.h>

double  fst_controller::TrajPlan::Polyn2_joint(double d, double v0, double v_obj, double v1, double a)
{
	double t_constspeed = 0;
	double d_ct = (d - Sign(v_obj - v0)*(v_obj *v_obj - v0 *v0) / 2 / a - Sign(v1 - v_obj)*(v1 *v1 - v_obj *v_obj) / 2 / a);
	if (fabs(v_obj)>0.00000001)
	{
		t_constspeed = d_ct / v_obj;
	}
	
	double t0(0), t1(0), t_all(0), t(0);
	if (t_constspeed>=0)
	{
		t0 = fabs((v_obj - v0) / a);
		t1 = fabs((v_obj - v1) / a);
		t_all = t0 + t_constspeed + t1;
	}
	else if ((fabs((v0*v0 - v1 *v1) / 2 / a - d)<0.000001 && (v0>v1)) || (fabs((v1 *v1 - v0 *v0) / 2 / a - d)<0.000001 && (v0<v1)))
	{
		t_all = fabs(v0 - v1) / a;
	}
	else
	{
		double vu0 = sqrt((2 * a*d + v0 *v0 + v1 *v1) / 2);
		t0= 3600;
		if (vu0 > v0&&vu0 > v1)
		{
			t0 = (fabs(vu0 - v0) + fabs(vu0 - v1)) / a;
		} 
		
		double vu = -sqrt((2 * a*d + v0 * v0 + v1 *v1) / 2);

		if (vu > v0&&vu > v1)
		{
			t = (fabs(vu - v0) + fabs(vu - v1)) / a;
			if (t<t0)
			{
				t0= t;
				vu0 = vu;
			} 
			
		}
		
		vu = sqrt((-2 * a*d + v0 *v0 + v1*v1) / 2);
		if (vu < v0&&vu < v1)
		{
			t = (fabs(vu - v0) + fabs(vu - v1)) / a;
			if (t<t0)
			{
				 t0 =t;
				 vu0 = vu;
			}
		}

			vu = -sqrt((-2 * a*d + v0 *v0 + v1 *v1) / 2);
			if (vu<v0&&vu<v1)
			{
				t = (fabs(vu - v0) + fabs(vu - v1)) / a;
				if (t<t0)
				{
					t0 = t;
					vu0 = vu;
				}
			}

		t_all = t0;
	}
	t_all = ceil(t_all / tc)*tc;
	return t_all;

}

int fst_controller::TrajPlan::Polyn2_joint_t(double d, double v0, double v1, double t, double a, double T[4], double P[3][4])
{
	int n = 0;
	double B = v0 + v1 + a*t;
	double C = v0 *v0 + v1*v1 + 2 * a*d;
	double VU[5] = {};
	if (B*B-2*C>=0)
	{
		double S[2] = {};
		S[0] = (B +pow (B*B - 2 *C ,0.5)) / 2;
		S[1] = (B -pow (B*B- 2 * C,0.5)) / 2;
		for (int i = 0; i < 2; i++)
		{
			if (S[i] >= v0&&S[i] >= v1 && (fabs(S[i] - v0) + fabs(S[i]- v1)) / a <= t)
			{
				VU[n] = S[i];
				n = n + 1;
			}
		}
	}

	B = v0 + v1 - a*t;
	C = v0 *v0 + v1*v1 - 2 * a*d;
	if (B*B - 2 * C >= 0)
	{
		double S[2] = {};
		S[0] = (B + pow(B*B - 2 * C, 0.5)) / 2;
		S[1] = (B - pow(B*B - 2 * C, 0.5)) / 2;
		for (int i = 0; i < 2; i++)
		{
			if (S[i] <= v0&&S[i] <= v1 && (fabs(S[i] - v0) + fabs(S[i] - v1)) / a <= t)
			{
				VU[n] = S[i];
				n = n + 1;
			}
		}
	}

	double u0 = v0; double u1 = v1;
	if (v0>=v1)
	{
		u1 = v0;
		u0 = v1;
	}

	double vu = (2 * a*d + u0 *u0 - u1 *u1) / (u0 - u1 + a*t) / 2;
	if  (vu >= u0&&vu <= u1 && (fabs(vu - v0) + fabs(vu - v1)) / a <= t)
	{
		VU[n] = vu;
		n = n + 1;
	} 


	double flag = 1000;
	int k = 1;

	if (n==0)
	{
		T[0] = 0; T[1] = t; T[2] = 0; T[3] = 0;
		double P_tmp[4] = {};
		Interp_singlejoint_t(d, v0, v1, t, P_tmp);
		for (int j = 0; j < 4; j++)
		{
			P[0][j] = P_tmp[j];
		}
	}

	else
	{
		for (int i = 0; i < n; i++)
		{
			double flag_new = pow((v1 - VU[i]) , 2 )+pow( (v0 - VU[i]) ,2);
			if (flag_new<flag)
			{
				flag = flag_new;
				k = i;
			}
		}
		double vu = VU[k];
		double t0 = fabs((vu - v0) / a);
		double t1 = fabs((vu - v1) / a);
		T[0] = 0; T[1] = t0; T[2] = t-t1; T[3] = t;
		
		double P0[3][4] = { { 0, Sign(vu - v0)*a / 2, v0, 0 },
		{ 0, 0, vu, Sign(vu - v0)*a / 2 * t0 *t0 + v0*t0 - vu*t0 },
		{ 0, Sign(v1 - vu)*a / 2, v1 - Sign(v1 - vu)*a*t, d - Sign(v1 - vu)*a / 2 * t *t - (v1 - Sign(v1 - vu)*a*t)*t } };
		memcpy(P, P0, 12 * sizeof(double));
		n = 3;

	}
	return n;

}

int fst_controller::TrajPlan::Interp_singlejoint_t(double d, double v0, double v1, double t, double P[4])
{
	double T[4][4] = { { 0, 0, 1, 0 }, { 3 * t*t, 2 * t, 1, 0 }, { t*t*t, t*t, t }, { 0, 0, 0, 1 } };
	double v[4] = { v0, v1, d, 0 };
	double InvT[4][4];
	InverseMatrix(T, InvT);
	MULTIPLY_MV(InvT, v, P);
	return 0;

}


int fst_controller::TrajPlan::MoveJ(	const JointPoint &J0,				//当前位置和速度
															const JointPoint &J1,				//目标位置和速度
															int v_percentage,												//速度百分比
															vector< JointValues> &Joint_diff)			//结果
{	
	
//	double c1 = 1;
//	double overshoot = 0.25;
//	double errorangle = 0.15;

	double p =(double)v_percentage / 100;
	double t[6] = { 0, 0, 0, 0, 0, 0 };
	double JointValue_sta[6] = { J0.joints.j1, J0.joints.j2, J0.joints.j3, J0.joints.j4, J0.joints.j5, J0.joints.j6 };
	double JointValue_end[6] ={ J1.joints.j1, J1.joints.j2, J1.joints.j3, J1.joints.j4, J1.joints.j5, J1.joints.j6 };
	double JointOmg_sta[6] = { J0.omegas.j1, J0.omegas.j2, J0.omegas.j3, J0.omegas.j4, J0.omegas.j5, J0.omegas.j6 };
	double JointOmg_end[6] = { J1.omegas.j1, J1.omegas.j2, J1.omegas.j3, J1.omegas.j4, J1.omegas.j5, J1.omegas.j6 };
	double d[6] = {};
	
	double t_max = 0;
	for (int i = 0; i < 6; i++)
	{
		d[i] = JointValue_end[i] - JointValue_sta[i];
		double v_obj = 0;
		if (fabs(d[i])<0.0000001)
		{
			v_obj = -Sign(JointOmg_sta[i] + JointOmg_end[i])*p*AXIS_LIMIT[2][i];
		}
		else
		{
			v_obj = Sign(d[i])*p*AXIS_LIMIT[2][i];
		}
		t[i] = Polyn2_joint(d[i], JointOmg_sta[i], v_obj, JointOmg_end[i], AXIS_LIMIT[3][i] * accelerationoverload);
		if (t[i]>t_max)
		{
			t_max = t[i];
		}
		
	}
	int n_time = round(t_max / tc);
	
	double delta=0;
	double J = 0;
	int flag = 0;
	
	Joint_diff.resize(n_time);
	for (int i = 0; i < 6; i++)
	{
		double T[4] = {};
		double P[3][4] = {};
		int n1 =Polyn2_joint_t(d[i], JointOmg_sta[i], JointOmg_end[i], t_max, AXIS_LIMIT[3][i] *accelerationoverload, T, P);
		double ti;
		for (int j = 0; j < n_time; j++)
		{
			ti= (j + 1)*tc;
			int kk = 0;
			for (int k = kk; k < n1+1; k++)
			{
				int flag1 = (ti >= (T[k] - tc / 10) && ti <= (T[k + 1] + tc / 10));
//				if ((ti >= (T[k]-tc/10)) && (ti <= (T[k+1]+tc/10)))
				if (flag1)
				{
					delta = P[k][0] * pow(ti, 3) + P[k][1] * pow(ti, 2) + P[k][2] * ti+ P[k][3];
					J = delta + JointValue_sta[i];

					if (d[i]>=0)
					{
						flag = (delta<-overshoot) || (delta>(d[i] + overshoot));
					} 
					else
					{
						flag = (delta<(d[i] - overshoot)) || (delta>overshoot);
					}

					if (flag)
					{
//						cout << "Error" << endl;
						return 1031;
					}
					else if (J>(AXIS_LIMIT[1][i] - errorangle )|| J<(AXIS_LIMIT[0][i] + errorangle))
					{
//						cout << "Error" << endl;
						return 1032;
					}
					else
					{
						switch (i)
						{
						case 0:	{
							Joint_diff[j].j1 = J;	
//							cout << "J1" <<"  "<< j <<"  "<< J << endl;
							break; }
						case 1:	{
							Joint_diff[j].j2 = J;	
//							cout << "J2" << "  " << j << "  " << J << endl;
							break; }
						case 2: {
							Joint_diff[j].j3 = J;	
//							cout << "J3" << "  " << j << "  " << J << endl;
							break; }
						case 3:{
							Joint_diff[j].j4 = J;		
//							cout << "J4" << "  " << j << "  " << J << endl;
							break; }
						case 4:{
							Joint_diff[j].j5 = J;		
//							cout << "J5" << "  " << j << "  " << J << endl;
							break; }
						case 5:{
							Joint_diff[j].j6 = J;		
//						cout << "J6" << "  " << j << "  " << J << endl;
							break; }
						}						
					}
					kk=k;
					break;

				}
			}
		}
	}

	return 0;

}

int fst_controller::TrajPlan::MoveJ2J_Cspace(const fst_controller::JointPoint &J0,				//当前位置和速度	
	const fst_controller::Pose &P,				//目标位置
	int v_percentage, int cnt,									//目标速度（百分比，取值：1-100）和cnt	（取值0-100）
	vector<fst_controller::JointValues> &Joint_diff,		//返回结果
	fst_controller::JointPoint &J_out)								//返回下条指令的初值
{
	JointPoint J1;
	InverseKinematics(P, J0.joints, J1.joints);
	double p = (double)v_percentage / 100;
	double p2 = (double)cnt / 100;
	double JointValue_sta[6] = { J0.joints.j1, J0.joints.j2, J0.joints.j3, J0.joints.j4, J0.joints.j5, J0.joints.j6 };
	double JointValue_end[6] = { J1.joints.j1, J1.joints.j2, J1.joints.j3, J1.joints.j4, J1.joints.j5, J1.joints.j6 };
	double JointOmg_sta[6] = { J0.omegas.j1, J0.omegas.j2, J0.omegas.j3, J0.omegas.j4, J0.omegas.j5, J0.omegas.j6 };
	double JointOmg_end[6] = { 0, 0, 0, 0, 0, 0 };
	double d[6] = {};

	double t_max = 0;
	for (int i = 0; i < 6; i++)
	{
		d[i] = JointValue_end[i] - JointValue_sta[i];
		double v_obj = 0;
		if (fabs(d[i]) < 0.0000001)
		{
			v_obj = -Sign(JointOmg_sta[i])*p*AXIS_LIMIT[2][i];
		}
		else
		{
			v_obj = Sign(d[i])*p*AXIS_LIMIT[2][i];
		}
		switch (i)
		{
		case 0:	{
			J1.omegas.j1 = v_obj*p2;			break; }
		case 1:	{
			J1.omegas.j2 = v_obj*p2;			break; }
		case 2: {
			J1.omegas.j3 = v_obj*p2;			break; }
		case 3:{
			J1.omegas.j4 = v_obj*p2;			break; }
		case 4:{
			J1.omegas.j5 = v_obj*p2;			break; }
		case 5:{
			J1.omegas.j6 = v_obj*p2;			break; }
		}
	}
	int n = MoveJ(J0, J1, v_percentage, Joint_diff);
	J_out = J1;
	return n;
}


int fst_controller::TrajPlan::MoveJ2J_Jspace(const fst_controller::JointPoint &J0,				//当前位置和速度	
	fst_controller::JointPoint &J1,				//目标位置
	int v_percentage, int cnt,									//目标速度（百分比，取值：1-100）和cnt	（取值0-100）
	vector<fst_controller::JointValues> &Joint_diff,		//返回结果
	fst_controller::JointPoint &J_out)								//返回下条指令的初值
{

	double p = (double)v_percentage / 100;
	double p2 = (double)cnt / 100;
	double JointValue_sta[6] = { J0.joints.j1, J0.joints.j2, J0.joints.j3, J0.joints.j4, J0.joints.j5, J0.joints.j6 };
	double JointValue_end[6] = { J1.joints.j1, J1.joints.j2, J1.joints.j3, J1.joints.j4, J1.joints.j5, J1.joints.j6 };
	double JointOmg_sta[6] = { J0.omegas.j1, J0.omegas.j2, J0.omegas.j3, J0.omegas.j4, J0.omegas.j5, J0.omegas.j6 };
	double JointOmg_end[6] = { 0, 0, 0, 0, 0, 0 };
	double d[6] = {};

	double t_max = 0;
	for (int i = 0; i < 6; i++)
	{
		d[i] = JointValue_end[i] - JointValue_sta[i];
		double v_obj = 0;
		if (fabs(d[i]) < 0.000001)
		{
			v_obj = -Sign(JointOmg_sta[i])*p*AXIS_LIMIT[2][i];
		}
		else
		{
			v_obj = Sign(d[i])*p*AXIS_LIMIT[2][i];
		}
		switch (i)
		{
		case 0:	{
			J1.omegas.j1 = v_obj*p2;			break; }
		case 1:	{
			J1.omegas.j2 = v_obj*p2;			break; }
		case 2: {
			J1.omegas.j3 = v_obj*p2;			break; }
		case 3:{
			J1.omegas.j4 = v_obj*p2;			break; }
		case 4:{
			J1.omegas.j5 = v_obj*p2;			break; }
		case 5:{
			J1.omegas.j6 = v_obj*p2;			break; }
		}
	}
	int n = MoveJ(J0, J1, v_percentage, Joint_diff);
	J_out = J1;
	return n;
}

//后接MoveL指令的MoveJ，给定目标为笛卡尔空间位姿
int fst_controller::TrajPlan::MoveJ2L_Cspace(const JointPoint &J0,				//当前位置和速度	
	const Pose &P1,				//目标位置
	const int v_percentage, const int cnt,									//目标速度和cnt	
	const Pose P2,									//预读下一点位姿
	const double vl,														//直线轨迹速度
	vector<JointValues> &Joint_diff,		//返回结果
	Pose &P0,									//返回下条指令的初值
	double &v0, double &vu0,
	Pose &P_prv)
{
	JointPoint  J1, J_tc;
	InverseKinematics(P1, J0.joints, J1.joints);

	Point PST1, PST2, PST_tc;
	PST1 = P1.position;
	PST2 = P2.position;
	Quaternion Quatern1 = P1.orientation, Quatern2 = P2.orientation, Quatern_tc;

	double dist_v[3] = { PST2.x - PST1.x, PST2.y - PST1.y, PST2.z - PST1.z };
	double dist_l = norm(dist_v);
	double tl = dist_l / vl;
	double c1 = (double)tc / tl*cnt / 100;
	PST_tc.x = (1 - c1)*PST1.x + c1*PST2.x;
	PST_tc.y = (1 - c1)*PST1.y + c1*PST2.y;
	PST_tc.z = (1 - c1)*PST1.z + c1*PST2.z;
	double Qt1[4] = { Quatern1.w, Quatern1.x, Quatern1.y, Quatern1.z };
	double Qt2[4] = { Quatern2.w, Quatern2.x, Quatern2.y, Quatern2.z };
	double Qt_tc[4];
	Slerp(Qt1, Qt2, c1, Qt_tc);
	Quatern_tc.w = Qt_tc[0]; Quatern_tc.x = Qt_tc[1]; Quatern_tc.y = Qt_tc[2]; Quatern_tc.z = Qt_tc[3];
	Pose P_tc;
	P_tc.position = PST_tc; P_tc.orientation = Quatern_tc;
	InverseKinematics(P_tc, J1.joints, J_tc.joints);

	J1.omegas.j1 = (J_tc.joints.j1 - J1.joints.j1) / tc;
	J1.omegas.j2 = (J_tc.joints.j2 - J1.joints.j2) / tc;
	J1.omegas.j3 = (J_tc.joints.j3 - J1.joints.j3) / tc;
	J1.omegas.j4 = (J_tc.joints.j4 - J1.joints.j4) / tc;
	J1.omegas.j5 = (J_tc.joints.j5 - J1.joints.j5) / tc;
	J1.omegas.j6 = (J_tc.joints.j6 - J1.joints.j6) / tc;

	int n = MoveJ(J0, J1, v_percentage, Joint_diff);
	P0 = P1;
	v0 = (double)vl*cnt / 100;
	double tmp1[4] = { Quatern1.w, Quatern1.x, Quatern1.y, Quatern1.z };
	double tmp2[4] = { Quatern2.w, Quatern2.x, Quatern2.y, Quatern2.z };
	double tmp3[4];
	Quaternprv(tmp1, tmp2, tmp3);
	Quaternion Q_prv = P_prv.orientation;
	Q_prv.w = tmp3[0];
	Q_prv.x = tmp3[1];
	Q_prv.y = tmp3[2];
	Q_prv.z = tmp3[3];
	vu0 = cnt/100/tl;
	return n;
}


//后接MoveL指令的MoveJ，给定目标为关节空间位
int fst_controller::TrajPlan::MoveJ2L_Jspace(const JointPoint &J0,				//当前位置和速度	
	const JointValues &J1_value,				//目标位置
	const int v_percentage, const int cnt,									//目标速度和cnt	
	const Pose P2,									//预读下一点位姿
	const double vl,														//直线轨迹速度
	vector<JointValues> &Joint_diff,		//返回结果
	Pose &P0,									//返回下条指令的初值
	double &v0, double &vu0,
	Pose &P_prv)
{
	JointPoint  J1, J_tc;
//	InverseKinematics(P1, J0.joints, J1.joints);
	J1.joints = J1_value;
	Pose P1;
	ForwardKinematics(J1_value, P1);
	Point PST1, PST2, PST_tc;
	PST1 = P1.position;
	PST2 = P2.position;
	Quaternion Quatern1 = P1.orientation, Quatern2 = P2.orientation, Quatern_tc;

	double dist_v[3] = { PST2.x - PST1.x, PST2.y - PST1.y, PST2.z - PST1.z };
	double dist_l = norm(dist_v);
	double tl = dist_l / vl;
	double c1 = (double)tc / tl*cnt / 100;
	PST_tc.x = (1 - c1)*PST1.x + c1*PST2.x;
	PST_tc.y = (1 - c1)*PST1.y + c1*PST2.y;
	PST_tc.z = (1 - c1)*PST1.z + c1*PST2.z;
	double Qt1[4] = { Quatern1.w, Quatern1.x, Quatern1.y, Quatern1.z };
	double Qt2[4] = { Quatern2.w, Quatern2.x, Quatern2.y, Quatern2.z };
	double Qt_tc[4];
	Slerp(Qt1, Qt2, c1, Qt_tc);
	Quatern_tc.w = Qt_tc[0]; Quatern_tc.x = Qt_tc[1]; Quatern_tc.y = Qt_tc[2]; Quatern_tc.z = Qt_tc[3];
	Pose P_tc;
	P_tc.position = PST_tc; P_tc.orientation = Quatern_tc;
	InverseKinematics(P_tc, J1.joints, J_tc.joints);
	

	J1.omegas.j1 = (J_tc.joints.j1 - J1.joints.j1) / tc;
	J1.omegas.j2 = (J_tc.joints.j2 - J1.joints.j2) / tc;
	J1.omegas.j3 = (J_tc.joints.j3 - J1.joints.j3) / tc;
	J1.omegas.j4 = (J_tc.joints.j4 - J1.joints.j4) / tc;
	J1.omegas.j5 = (J_tc.joints.j5 - J1.joints.j5) / tc;
	J1.omegas.j6 = (J_tc.joints.j6 - J1.joints.j6) / tc;

	int n = MoveJ(J0, J1, v_percentage, Joint_diff);
	P0 = P1;
	v0 = (double)vl*cnt / 100;
	double tmp1[4] = { Quatern1.w, Quatern1.x, Quatern1.y, Quatern1.z };
	double tmp2[4] = { Quatern2.w, Quatern2.x, Quatern2.y, Quatern2.z };
	double tmp3[4];
	Quaternprv(tmp1, tmp2, tmp3);
	Quaternion Q_prv = P_prv.orientation;
	Q_prv.w = tmp3[0];
	Q_prv.x = tmp3[1];
	Q_prv.y = tmp3[2];
	Q_prv.z = tmp3[3];
	vu0 = cnt / 100 / tl;
	return n;
}

/*
int fst_controller::TrajPlan::MoveJ2C_Cspace(const JointPoint &J0,				//当前位置和速度	
	const Pose &P1,				//目标位置
	int v_percentage, int cnt,									//目标速度（百分比，取值：1-100）和cnt	（取值0-100）
	const Pose &P2, const Pose &P3, double vc,
	vector<JointValues> &Joint_diff,		//返回结果
	Pose  &P0,double *v0)						//返回下条指令的初值
{
	double PST0[3] = { P1.position.x, P1.position.y, P1.position.z };
	double PST1[3] = { P2.position.x, P2.position.y, P2.position.z };
	double PST2[3] = { P3.position.x, P3.position.y, P3.position.z };
	double T[4][4];
	double r;
	double dm;
	double d;
	double O[3];
	Arc_3points(PST0, PST1, PST2, T, &r, &d, &dm, O);
	double di = tc*d*r / vc*cnt / 100 * Sign(d);
	Pose PST_tc;
	PST_tc.position.x = T[0][0] * r*cos(di) + T[0][1] * r* sin(di) + T[0][3];
	PST_tc.position.y = T[1][0] * r*cos(di) + T[1][1] * r* sin(di) + T[1][3];
	PST_tc.position.z = T[2][0] * r*cos(di) + T[2][1] * r* sin(di) + T[2][3];
	PST_tc.orientation = P1.orientation;
	JointPoint J1,J1_tc;
	int n = 0;
	n=InverseKinematics(P1, J0.joints, J1.joints);
	if (n != 0 && n != 1003)
	{
		return n;
	}
	n = InverseKinematics(PST_tc, J1.joints, J1_tc.joints);
	if (n != 0 )
	{
		return n;
	}
	J1.omegas.j1 = (J1_tc.joints.j1 - J1.joints.j1) / tc;
	J1.omegas.j2 = (J1_tc.joints.j2 - J1.joints.j2) / tc;
	J1.omegas.j3 = (J1_tc.joints.j3 - J1.joints.j3) / tc;
	J1.omegas.j4 = (J1_tc.joints.j4 - J1.joints.j4) / tc;
	J1.omegas.j5 = (J1_tc.joints.j5 - J1.joints.j5) / tc;
	J1.omegas.j6 = (J1_tc.joints.j6 - J1.joints.j6) / tc;
	n=MoveJ(J0, J1, v_percentage, Joint_diff);
	if (n != 0)
	{
		return n;
	}
	P0 = P1;
	*v0 = vc*(double)cnt / 100;
	return 0;
}*/




	
