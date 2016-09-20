
//#include <math.h>
#include <lib_controller/trajplan.h>

// 将位置和姿态（四元数表示）转换为位姿矩阵
int fst_controller::TrajPlan::POSE2MATRIX(double PST1[3], double ORT1[4], double Pose_M[4][4])
{
	double x(PST1[0]), y(PST1[1]), z(PST1[2]), s(ORT1[0]), a(ORT1[1]), b(ORT1[2]), c(ORT1[3]);

	Pose_M[0][0] = 2 * (s*s + a*a) - 1;
	Pose_M[0][1] = 2 * (a*b - s*c);
	Pose_M[0][2] = 2 * (s*b + a*c);
	Pose_M[0][3] = x;
	Pose_M[1][0] = 2 * (a*b + s*c);
	Pose_M[1][1] = 2 * (s*s + b*b) - 1;
	Pose_M[1][2] = 2 * (-s*a + b*c);
	Pose_M[1][3] = y;
	Pose_M[2][0] = 2 * (-s*b + a*c);
	Pose_M[2][1] = 2 * (s*a + b*c);
	Pose_M[2][2] = 2 * (s*s + c*c) - 1;
	Pose_M[2][3] = z;
	Pose_M[3][0] = Pose_M[3][1] = Pose_M[3][2] = 0;
	Pose_M[3][3] = 1;

	/*	cout << Pose_M[0][0] << "	" << Pose_M[0][1] << "	" << Pose_M[0][2] << "	" << Pose_M[0][3] << endl;
	cout << Pose_M[1][0] << "	" << Pose_M[1][1] << "	" << Pose_M[1][2] << "	" << Pose_M[1][3] << endl;
	cout << Pose_M[2][0] << "	" << Pose_M[2][1] << "	" << Pose_M[2][2] << "	" << Pose_M[2][3] << endl;
	cout << Pose_M[3][0] << "	" << Pose_M[3][1] << "	" << Pose_M[3][2] << "	" << Pose_M[3][3] << endl << endl;	*/
	return 0;
};

// 将位置和姿态（四元数表示）转换为位姿矩阵
int fst_controller::TrajPlan::MATRIX2POSE(double Pose_M[4][4], double PST2[3], double ORT2[4])
{
	PST2[0] = Pose_M[0][3];
	PST2[1] = Pose_M[1][3];
	PST2[2] = Pose_M[2][3];
	ORT2[0] = sqrt(fabs(Pose_M[0][0] + Pose_M[1][1] + Pose_M[2][2] + 1)) / 2;
	ORT2[1] = sqrt(fabs(Pose_M[0][0] - Pose_M[1][1] - Pose_M[2][2] + 1)) / 2;
	ORT2[2] = sqrt(fabs(-Pose_M[0][0] + Pose_M[1][1] - Pose_M[2][2] + 1)) / 2;
	ORT2[3] = sqrt(fabs(-Pose_M[0][0] - Pose_M[1][1] + Pose_M[2][2] + 1)) / 2;

	int a, b, c;
	if (ORT2[0] > 0.00000001)
	{
		a = Sign(Pose_M[2][1] - Pose_M[1][2]);
		b = Sign(Pose_M[0][2] - Pose_M[2][0]);
		c = Sign(Pose_M[1][0] - Pose_M[0][1]);
	}
	else
	{
		a = 1;
		b = Sign(Pose_M[1][0]);
		c = Sign(Pose_M[2][0]);
		if (ORT2[1] < 0.00000001)
		{
			b = 1;
			c = Sign(Pose_M[2][1]);
			if (ORT2[1] < 0.00000001)
			{
				c = 1;
			}
		}

	}

	ORT2[1] = a*ORT2[1];
	ORT2[2] = b*ORT2[2];
	ORT2[3] = c*ORT2[3];
	return 0;
};

int fst_controller::TrajPlan::InvQ(double q[4], double iq[4])
{
	iq[0] = q[0];
	iq[1] = -q[1];
	iq[2] = -q[2];
	iq[3] = -q[3];
	return 0;
}

int fst_controller::TrajPlan::MultiQ(double u[4], double v[4], double w[4])
{
	
	double m[3] = { u[1], u[2], u[3] };
	double n[3] = { v[1], v[2], v[3] };
	w[0] = u[0] * v[0] - InnerProduct(m, n);
	double tmp1[3] = {};
	double tmp2[3] = {};
	double tmp3[3] = {};
	
	Cross(m, n, tmp1);
	nV3(u[0], n, tmp2);
	nV3(v[0], m, tmp3);
	add3(tmp1, tmp2, tmp1);
	add3(tmp1, tmp3, tmp1);
	w[1] = tmp1[0];
	w[2] = tmp1[1];
	w[3] = tmp1[2];
	return 0;

}

int fst_controller::TrajPlan::LogQ(double q[4], double l[4])
{
	double v[3] = { q[1], q[2], q[3] };
	double theta = acos(q[1]);
	if (theta!=0)
	{
		nV3(1 / sin(theta), v, v);
	}
	nV3(theta, v, v);
	l[0] = 0;
	l[1] = v[0];
	l[2] = v[1];
	l[3] = v[2];
	return 0;	

}

double fst_controller::TrajPlan::InnerProduct4(double u[4], double v[4])
{
	return u[0] * v[0] + u[1] * v[1] + u[2] * v[2] + u[3] * v[3];
}

int fst_controller::TrajPlan::ExpQ(double L[4], double E[4])
{
	if (InnerProduct4(L,L)==0)
	{
		double tmp[4] = { 1, 0, 0, 0 };
		Assign4(tmp, E);
	}
	else
	{
		double theta = pow(InnerProduct4(L, L), 0.5);
		double k = sin(theta) / theta;
		double tmp[4] = { cos(theta), L[1] * k, L[2] * k, L[3] * k };
		Assign4(tmp, E);
	}
	return 0;
}

int fst_controller::TrajPlan::add4(double u[4], double v[4], double w[4])
{
	for (int i = 0; i < 4; i++)
	{
		w[i] = u[i] + v[i];
	}
	return 0;
}

int fst_controller::TrajPlan::nV4(double n, double V[4], double nV[4])
{
	for (int i = 0; i < 4; i++)
	{
		nV[i] = n*V[i];
	}
	return 0;
}

int fst_controller::TrajPlan::Si(double qiq[4], double qi[4], double qih[4], double si[4])
{
	double invqi[4] = {};
	InvQ(qi, invqi);
	//print4(qi);
	//print4(invqi);
	double m1[4],m2[4];
	MultiQ(qiq, invqi,m1 );
	LogQ(m1, m1);
	MultiQ(qih, invqi, m2);
	LogQ(m2, m2);
	add4(m1, m2, m1);
	nV4(-0.25, m1, m1);
	ExpQ(m1, m2);
	MultiQ(m2, qi, m1);
	Assign4(m1, si);
	return 0;
}


//u-t 三次拟合
int fst_controller::TrajPlan::Polyn3_Q(double t, double v[2], double A[4])
{

	double M[4][4] = { { 0,0, 1, 0 }, {0,0,0, 1 }, { pow(t,3), pow(t,2), t, 1 }, { 3 * pow(t,2), 2 * t, 1, 0 } };
	double IM[4][4];
	InverseMatrix(M, IM);
	double V[4] = { v[0], 0, 1, v[1] };
	MULTIPLY_MV(IM, V, A);
	return 0;
}

int fst_controller::TrajPlan::Slerp(double q0[4], double q1[4], double  h, double qh[4])
{
//	print4(q0);
//	print4(q1);
//	nV4(Sign(InnerProduct(q0, q1)), q1, q1);
	double tmp = InnerProduct4(q0, q1);
	if (fabs(tmp)>1)
	{
		tmp = tmp/fabs(tmp);
	}
	double omg = acos(tmp);
	if (omg == 0)
	{
		Assign4(q0, qh);
	}
	else
	{
		double tmp1[4] = {}, tmp2[4] = {};
		nV4(sin((1 - h)*omg) / sin(omg), q0, tmp1);
		nV4(sin(h*omg) / sin(omg), q1, tmp2);
//		print4(tmp1);
//		print4(tmp2);
		add4(tmp1, tmp2, tmp1);
		Assign4(tmp1, qh);
//		cout << sin((1 - h)*omg) / sin(omg) << endl;
//		cout << sin(h*omg) / sin(omg) << endl;

	}
//	print4(q0);
//	print4(q1);
//	print4(qh);
	return 0;
}

int fst_controller::TrajPlan::Squad(double q1[4], double q2[4], double s1[4], double s2[4], double u, double qu[4])
{
//	print4(q1);
//	print4(q2);
//	print4(s1);
//	print4(s2);
	double tmp1[4], tmp2[4];
	Slerp(q1, q2, u, tmp1);
	/*print4(q1);
	print4(q2);
	cout << u << endl;
	print4(tmp1);*/
	Slerp(s1, s2, u, tmp2);
	Slerp(tmp1, tmp2, 2 * u*(1 - u),qu);
	return 0;
}

// 四元数球面插值，时间
/*int InterpQ(double t, double q_prv[4], double q_0[4], double q_obj[4], double q_nxt[4], double VU[2], vector<vector <double>> &Quatern_diff)
{
	double tc = 0.001;
	nV4(Sign(InnerProduct(q_0, q_obj)), q_obj, q_obj);
	nV4(Sign(InnerProduct(q_0, q_prv)), q_prv, q_prv);
	nV4(Sign(InnerProduct(q_nxt, q_obj)), q_nxt, q_nxt);
	double s1[4] = {}, s2[4] = {};
	Si(q_prv, q_0, q_obj, s1);
	Si(q_0, q_obj, q_nxt, s2);
	//print4(q_0);
	//print4(q_obj);
	//print4(s1);
	//print4(s2);


	double A[4];
	Polyn3_Q(t, VU, A);
	double ti,uu;
	double Q[4];
	for (int i = 0; i < round(t/tc); i++)
	{
		ti = (i + 1)*tc;
		uu = A[0] * pow(ti, 3) + A[1] * pow(ti, 2) + A[2] * ti + A[3];
		Squad(q_0, q_obj, s1, s2, uu, Q);
		//cout << i << endl;
		//cout << uu<<endl;
		//print4(Q);
		Quatern_diff[i][0] = Q[0];
		Quatern_diff[i][1] = Q[1];
		Quatern_diff[i][2] = Q[2];
		Quatern_diff[i][3] = Q[3];
	}
	return 0;
}*/

//时间域到离散点域

int fst_controller::TrajPlan::InterpQ(int n_time, double q_prv[4], double q_0[4], double q_obj[4], double q_nxt[4], double VU[2], vector<fst_controller::Pose> &Quatern_diff)
{
	
	nV4(Sign(InnerProduct(q_0, q_obj)), q_obj, q_obj);
	nV4(Sign(InnerProduct(q_0, q_prv)), q_prv, q_prv);
	nV4(Sign(InnerProduct(q_nxt, q_obj)), q_nxt, q_nxt);
	double s1[4] = {}, s2[4] = {};
	Si(q_prv, q_0, q_obj, s1);
	Si(q_0, q_obj, q_nxt, s2);
	//print4(q_0);
	//print4(q_obj);
	//print4(s1);
	//print4(s2);


	double A[4];
	Polyn3_Q(n_time, VU, A);
	double ti, uu;
	double Q[4];
	for (int i = 0; i < n_time; i++)
	{
		ti = (i + 1);
		uu = A[0] * pow(ti, 3) + A[1] * pow(ti, 2) + A[2] * ti + A[3];
		Squad(q_0, q_obj, s1, s2, uu, Q);
		//cout << i << endl;
		//cout << uu<<endl;
		//print4(Q);
		Quatern_diff[i].orientation.w = Q[0];
		Quatern_diff[i].orientation.x = Q[1];
		Quatern_diff[i].orientation.y= Q[2];
		Quatern_diff[i].orientation.z = Q[3];
	}
	return 0;
}


//虚拟上一点的四元数

int fst_controller::TrajPlan::Quaternprv(double Q1[4], double Q2[4], double Q_prv[4])
{
	double p1 = InnerProduct4(Q1, Q2);
	if (p1<0)
	{
		nV4(-1, Q2, Q2);
	}
	double tmp[3] = { 0, 0, 0 };
	double M1[4][4] = {};
	double M2[4][4] = {};
	double M3[4][4] = {};
	POSE2MATRIX(tmp, Q1, M1);
	POSE2MATRIX(tmp, Q1, M2);
	InverseMatrix(M1, M3);
	MULTIPLY_M(M2, M3, M3);
	InverseMatrix(M3, M3);
	MULTIPLY_M(M3, M1, M3);
	MATRIX2POSE(M3, tmp, Q_prv);
	p1 = InnerProduct(Q1, Q_prv);
	if (p1<0)
	{
		nV4(-1, Q_prv, Q_prv);
	}
	return 0;
}
