#include <lib_controller/trajplan.h> 


void fst_controller::TrajPlan::setAcceleration(double _acc)
{
	acc = _acc;
}
void fst_controller::TrajPlan::setCycleTime(double _tc)
{
	tc= _tc;
}



int fst_controller::TrajPlan::Smt_arc(double P0[3], double P1[3], double P2[3], double r, double B1[3], double B2[3], double O[3], double *angle, double T[4][4])
{
	double pi = 3.1415926535897932384626433832795;
	double BA[3] = {};
	double BC[3] = {};
	double X[3] = {};
	double Y[3] = {};
	double Z[3] = {};
	double t1(0), t2(0), t3(0);
	double n1 = 0;
	double p00 = P0[0];
	double p01 = P0[1];
	double p02 = P0[2];
	double tmp1[3], tmp2[3];
	double uBA[3];
	double uBC[3];


	for (int i = 0; i < 3; i++)
	{

		BA[i] = P0[i] - P1[i];
		BC[i] = P2[i] - P1[i];
		t1 += BA[i] * BC[i];
		t2 += BA[i] * BA[i];
		t3 += BC[i] * BC[i];
	}
	t2 = sqrt(t2);
	t3 = sqrt(t3);

	if (t2*t3 == 0)
	{
		return 1;						// 三点中至少两点重合，报错 <未考虑定点转动情况>
	}
	else
	{
		n1 = t1 / t2 / t3;
		if (fabs(n1)>1)
		{
			n1 = n1 / fabs(n1);
		}
		double theta = acos(n1);
		double a = r / tan(theta / 2);
		double normBA = norm(BA);
		double normBC = norm(BC);
		if (a>normBA / 2 || a>normBC / 2)
		{
			return 1;										//半径相对与点距过大
		}
		double b = r / sin(theta / 2);
		nV3(1 / norm(BA), BA, uBA);
		nV3(1 / norm(BC), BC, uBC);

		nV3(a, uBA, tmp1);
		nV3(a, uBC, tmp2);
		add3(P1, tmp1, B1);
		add3(P1, tmp2, B2);

		add3(uBA, uBC, tmp1);
		nV3(b / norm(tmp1), tmp1, tmp1);
		add3(tmp1, P1, O);

		nV3(-1, O, tmp1);
		add3(B1, tmp1, X);
		add3(B2, tmp1, tmp2);

		Cross(X, tmp2, Z);

		Cross(Z, X, Y);
		double x = norm(X);
		double z = norm(Z);
		double y = norm(Y);
		nV3(1 / x, X, X);
		nV3(1 / y, Y, Y);
		nV3(1 / z, Z, Z);


		double TT[4][4] = { { X[0], Y[0], Z[0], O[0] }, { X[1], Y[1], Z[1], O[1] }, { X[2], Y[2], Z[2], O[2] }, { 0, 0, 0, 1 } };
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				T[i][j] = TT[i][j];
			}
		}
		*angle = pi - theta;

		return 0;
	}
}

int fst_controller::TrajPlan::Polyn2(double d, double v0, double v_obj, double *v_cnt1, double a, double T[4], double P[3][3])
{
	int n = 0;
	double v_cnt = *v_cnt1;
	T[0] = 0;
	if (fabs((v0 *v0 - v_cnt *v_cnt) / 2 / a) > d)
	{
		double a1 = P[0][0] = Sign(v_cnt - v0)*a / 2;
		double b1 = P[0][1] = v0;
		double c1 = P[0][2] = 0;
		double t1 = T[1] = (pow(b1 *b1 + 4 * a1*d, 0.5) - b1) / 2 / a1;
		*v_cnt1 = b1 + 2 * a1*t1;
		n = 0;
	}
	else if ((fabs(v0 * v0 - v_obj *v_obj) + fabs(v_obj*v_obj - v_cnt *v_cnt)) >= 2 * a*d)
	{
		double v22 = sqrt((2 * a*d + v0 * v0 + v_cnt *v_cnt) / 2);
		double a1 = P[0][0] = Sign(v_obj - v0)*a / 2;
		double b1 = P[0][1] = v0;
		double c1 = P[0][2] = 0;
		double a3 = P[1][0] = Sign(v_cnt - v22)*a / 2;
		double t1 = T[1] = 0;
		if (a1!=0)
		{
			t1 = T[1] = (v22 - v0) / 2 / a1;
		}
		double t2 = T[2] = 0;
		if (a3!=0)
		{
			t2 = T[2] = t1 + (v_cnt - v22) / 2 / a3;
		}
			
		double b3 = P[1][1] = v_cnt - 2 * a3*t2;
		double c3 = P[1][2] = d - a3*t2 *t2 - b3*t2;
		n = 1;
	}
	else
	{
		double a1 = P[0][0] = Sign(v_obj - v0)*a / 2;
		double b1 = P[0][1] = v0;
		double c1 = P[0][2] = 0;
		double t1 = T[1] = 0;
		if (a1 != 0)
		{
			t1 = T[1] = (v_obj - v0) / 2 / a1;
		}

		double a2 = P[1][0] = 0;
		double b2 = P[1][1] = v_obj;
		double c2 = P[1][2] = a1*t1*t1 + b1*t1 - b2*t1;
		//double a3 = P[2][0] = -a / 2;
		double a3 = P[2][0] = Sign(v_cnt - v_obj) *a / 2;
		double dt3 = 0;
		if (a3 != 0)
		{
			dt3 = (v_cnt - v_obj) / 2 / a3;
		}
		double dt2 = (d - (v_obj + v_cnt)*dt3 / 2 - (b2*t1 + c2)) / v_obj;
		double 	t2 = T[2] = t1 + dt2;
		double t3 = T[3] = t2 + dt3;
		double b3 = P[2][1] = v_obj - 2 * a3*t2;
		double c3 = P[2][2] = d - a3*t3 *t3 - b3*t3;
		n = 2;
	}
	return n;
}


int fst_controller::TrajPlan::MoveL(fst_controller::Pose &pose_start, double &v0, double &vu0,
												const fst_controller::Pose &pose_obj, const double v_obj, const int cnt_obj, 
												const fst_controller::Pose &pose_nxt, const double v_nxt, const int cnt_nxt, 
														 fst_controller::Pose &pose_prv,
														 vector<fst_controller::Pose> &Pose_diff)
{
	double  v_cnt = v_obj*cnt_obj/100;
	double k = 2;
	double radius = k*pow(v_cnt, 2) / acc;
	int flag1 = 0;
	double B1[3] = {};
	double B2[3] = {};
	double O[3] = {};
	double phi = 0;
	double ArcRot[4][4] = {};
	double PST_0[3] = { pose_start.position.x, pose_start.position.y, pose_start.position.z };
	double PST_obj[3] = { pose_obj.position.x, pose_obj.position.y, pose_obj.position.z };
	double PST_nxt[3] = { pose_nxt.position.x, pose_nxt.position.y, pose_nxt.position.z };


	if (cnt_obj ==0||cnt_obj==-1)
	{
		memcpy(B1, PST_obj, 3 * sizeof(double));
		memcpy(B2, PST_obj, 3 * sizeof(double));
		pose_prv = pose_obj;

	}
	else
	{
		
		flag1=Smt_arc(PST_0, PST_obj, PST_nxt, radius, B1, B2, O, &phi,ArcRot);
	}

	if (phi>3||flag1==1)
	{
		return 1021;							// 半径过大或夹角过于尖锐
	}
	else
	{
		double DX[3] = {};
		double DDX[3] = {};

		for (int i = 0; i < 3; i++)
		{
			DX[i] = B1[i] - PST_0[i];
		}
		double distance_l = norm(DX);
		if (distance_l==0)
		{
			return 1022;						//点距为0
		} 

		for (int i = 0; i < 3; i++)
		{
			DDX[i] = DX[i]/distance_l;
		}
		double Time[4] = { 0, 0, 0,0 };
		double P[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
		int n = Polyn2(distance_l, v0, v_obj, &v_cnt, acc, Time, P);
		double t_l=Time[n+1];
		double t_l_real = 0;
		double ti(0);
		int k = 0;
		double di = 0;
		int n_time1 = (int)(ceil(t_l / tc));
		Pose_diff.resize(n_time1);
		
		for (int i = 0; i < n_time1; i++)
		{
			ti = (i+1)*t_l / n_time1;
			for (int j = k; j <= n; j++)
			{
				if (ti <= Time[j + 1] && ti >= Time[j ])
				{
					di = P[j][0] * ti*ti + P[j][1] * ti + P[j][2];

					Pose_diff[i].position.x = di*DDX[0] + PST_0[0];
					Pose_diff[i].position.y = di*DDX[1] + PST_0[1];
					Pose_diff[i].position.z = di*DDX[2] + PST_0[2];

					k = j;
					//continue;
					break;
				} 
			}
			t_l_real = tc*n_time1;
		}
//		cout << ti << endl;
//		cout << t_l << endl;
//		cout << di << endl;
//		cout << distance_l << endl;

		double t_arc = 0;
		int n_time2 = 0;

		if (cnt_obj!=0&&cnt_obj!=-1&&phi>0.000001)
		{
			double omg = v_cnt / radius;
			t_arc = phi / omg;
			n_time2 = (int)(ceil(t_arc / tc));
			Pose_diff.resize(n_time1+n_time2);
				for (int i = 0; i < n_time2; i++)
			{
				ti = (i + 1)*t_arc / n_time2;
				Pose_diff[n_time1 + i].position.x = ArcRot[0][0] * radius*cos(omg*ti) + ArcRot[0][1] * radius*sin(omg*ti) + ArcRot[0][3];
				Pose_diff[n_time1 + i].position.y = ArcRot[1][0] * radius*cos(omg*ti) + ArcRot[1][1] * radius*sin(omg*ti) + ArcRot[1][3];
				Pose_diff[n_time1 + i].position.z = ArcRot[2][0] * radius*cos(omg*ti) + ArcRot[2][1] * radius*sin(omg*ti) + ArcRot[2][3];

			}
		}
		//cout << t_arc << endl;
		//cout << ti << endl;
		double t_arc_real = tc*n_time2;
		double t_real = t_l_real + t_arc_real;
		//cout << n_time1 << endl;
		//cout<<n_time2<<endl;
		//cout << t_real;
		double D_nxt[3];
		for (int i = 0; i < 3; i++)
		{
			D_nxt[i] = PST_nxt[i] - B2[i];
		}
		double t_nxt_estm = norm(D_nxt);

		double vu1;
		if (cnt_obj == 0 || cnt_obj == -1)
		{
			vu1 = 0;
		}
		else
		{
			vu1 = 1 / (t_real + t_nxt_estm);
		}

		double VU[2] = { vu0*tc, vu1*tc };						//时间域到离散点域
		
		double ORT_prv[4] = { pose_prv.orientation.w, pose_prv.orientation.x, pose_prv.orientation.y, pose_prv.orientation.z };
		double ORT_0[4] = { pose_start.orientation.w, pose_start.orientation.x, pose_start.orientation.y, pose_start.orientation.z };
		double ORT_obj[4] = { pose_obj.orientation.w, pose_obj.orientation.x, pose_obj.orientation.y, pose_obj.orientation.z };
		double ORT_nxt[4] = { pose_nxt.orientation.w, pose_nxt.orientation.x, pose_nxt.orientation.y, pose_nxt.orientation.z };

		InterpQ(n_time1+n_time2, ORT_prv, ORT_0, ORT_obj, ORT_nxt, VU, Pose_diff);

		vu0 = vu1;
		v0 = v_cnt;
		
		pose_prv = pose_start;
		pose_start = Pose_diff.back();


	}
	return 0;
}

int fst_controller::TrajPlan::MoveL2J(const Pose &P0, double v0, double vu0,
	const Pose &P1, double v_obj, int cnt_obj,
	Pose &P_prv,
	vector<fst_controller::Pose> &Pose_diff, JointPoint &J0)
{
	double  v_cnt = v_obj*cnt_obj / 100;
	double PST_0[3] = { P0.position.x, P0.position.y, P0.position.z };
	double PST_obj[3] = { P1.position.x, P1.position.y, P1.position.z };
	double DX[3] = {};
	double DDX[3] = {};

	for (int i = 0; i < 3; i++)
	{
		DX[i] = PST_obj[i] - PST_0[i];
	}
	double distance_l = norm(DX);
	if (distance_l == 0)
	{
		return 1022;						//点距为0
	}
	nV3(1 / distance_l, DX, DDX);
	double Time[4] = { 0, 0, 0, 0 };
	double P[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
	int n = Polyn2(distance_l, v0, v_obj, &v_cnt, acc, Time, P);
	double t_l = Time[n + 1];
	double t_l_real = 0;
	double ti(0);
	int k = 0;
	double di = 0;
	int n_time1 = (int)(ceil(t_l / tc));
	Pose_diff.resize(n_time1);

	for (int i = 0; i < n_time1; i++)
	{
		ti = (i + 1)*t_l / n_time1;
		for (int j = k; j <= n; j++)
		{
			if (ti <= Time[j + 1] && ti >= Time[j])
			{
				di = P[j][0] * ti*ti + P[j][1] * ti + P[j][2];

				Pose_diff[i].position.x = di*DDX[0] + PST_0[0];
				Pose_diff[i].position.y = di*DDX[1] + PST_0[1];
				Pose_diff[i].position.z = di*DDX[2] + PST_0[2];

				k = j;
				continue;
			}
		}
		t_l_real = tc*n_time1;
	}
	//cout << ti << endl;
	//cout << t_l << endl;
	//cout << di << endl;
	//cout << distance_l << endl;

	double vu1 = 0;
	double VU[2] = { vu0*tc, vu1*tc };						//时间域到离散点域

	double ORT_prv[4] = { P_prv.orientation.w,P_prv.orientation.x, P_prv.orientation.y, P_prv.orientation.z };
	double ORT_0[4] = { P0.orientation.w, P0.orientation.x, P0.orientation.y, P0.orientation.z };
	double ORT_obj[4] = { P1.orientation.w, P1.orientation.x, P1.orientation.y, P1.orientation.z };
	double ORT_nxt[4] = { P1.orientation.w, P1.orientation.x, P1.orientation.y, P1.orientation.z };

	InterpQ(n_time1, ORT_prv, ORT_0, ORT_obj, ORT_nxt, VU, Pose_diff);
	
	JointValues J_ref;
	J_ref.j1 = 0; J_ref.j2 = 0; J_ref.j3 = 0; J_ref.j4 = 0; J_ref.j5 = 0; J_ref.j6 = 0;
	int n2 = Pose_diff.size();
	//实际运行时，可完全反解完成后拿到关节位置和速度后，再规划下个J指令
	InverseKinematics(Pose_diff.back(), J_ref, J0.joints);
	InverseKinematics(Pose_diff[n2 - 1], J0.joints, J_ref);
	J0.omegas.j1 = (J0.joints.j1 - J_ref.j1) / tc;
	J0.omegas.j2 = (J0.joints.j2 - J_ref.j2) / tc;
	J0.omegas.j3 = (J0.joints.j3 - J_ref.j3) / tc;
	J0.omegas.j4 = (J0.joints.j4 - J_ref.j4) / tc;
	J0.omegas.j5 = (J0.joints.j5 - J_ref.j5) / tc;
	J0.omegas.j6 = (J0.joints.j6 - J_ref.j6) / tc;
	return 0;

}
