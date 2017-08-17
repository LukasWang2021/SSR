#ifndef _trajplan_h
#define _trajplan_h
#include <iostream>
#include "fst_datatype.h"
#include <cmath>
#include <vector>
#include <cstring>
using namespace std;

namespace fst_controller
{
	class TrajPlan
	{
	public:
		TrajPlan();

		// set the DH parameters(modified) of the robot in the order of  alpha, a ,d , offset of theta
		void setDH(double _alp0, double _a0, double _d1, double _os1, double _alp1, double _a1, double _d2, double _os2, double _alp2, double _a2, double _d3, double _os3, double _alp3, double _a3, double _d4, double _os4, double _alp4, double _a4, double _d5, double _os5, double _alp5, double _a5, double _d6, double _os6);
		
		//set the toolframe using a Euler-format pose
		void setToolFrame(PoseEuler &P);    
		
		//set the userframe using a Euler-format pose
		void setUserFrame(PoseEuler &P);    
		
		//set the axislimit using a 4*6 array, including the lower-limit,upper-limit,max velocity and max acceleration
		void setAxisLimit(const JointConstraints &axislimit);
		
		//set the cycle time 
		void setCycleTime(double _tc);
		
		//set the reference linear and angular velocity in Cartesian coordinates
		void setVelocityRef(double _vel_ref);
		void setOmegaRef(double _omega_ref);

		//set the max acceleration of TCP in Cartesian coordinates
		void setAcceleration(double _acc);
		
		//set the jerk of TCP
		void setJaratio(double _jerk);
		
		//Select Curve mode:1 for 2-order polynominal, 2 for 3-order polynominal
		void setCurveMode(double _curve_mode);

		//set smoothing radius coefficient
		void setRadiusCoefficient(double _k_redius);

		//set the allowable overshoot of each joint, in radian 
		void setOvershoot(double _overshoot);

		//set the allowable approaching angle to limit , in radian 
		void seterrorangle(double _errorangle);

		//set the allowable acceleration overload ratio of joints
		void setoverload(double _accelerationoverload);

		//set the velocity limit ratio related to the max velocity, normally between 0~1 
		void setLimitScale(double _limit_scale);

		void setEstime_coefficient(double _estime_coefficient);
		

		int ForwardKinematics(const JointValues &joint, Pose &pose);

		int InverseKinematics(const Pose &pose, const JointValues &joint_reference,JointValues &joint_result);

		
		int MoveL(	Pose &P0, double &v0, double &vu0,
							const Pose &P1, const double vl1, const int cnt1,
							const Pose &P2,  const double vl2, 
							Pose &P_prv,
							vector<Pose> &Pose_diff);
		
		int MoveL2J(	const Pose &P0, double v0, double vu0,
								const Pose &P1, double vl, int cnt1,
								Pose &P_prv, 
								vector<Pose> &Pose_diff);
		int GetJointStatus(const JointValues &J_2ndlast, const JointValues &J_last, JointPoint &J_startofnxt);

		int MoveL2C_part1(Pose &P0, double &v0, double vu0,
										const Pose &P1, double vl, int cnt,
										const Pose &P2, const Pose &P3, double vc,
										Pose &P_prv,
										vector<Pose> &Pose_diff,
										Pose &P_sta_ptc);

		int MoveL2C_part2(const JointPoint &J0, const Pose &P0, const Pose &P_sta_ptc, vector <JointValues> &Joint_diff);

		int MoveJ(	const JointPoint &J0, 	
							const  JointPoint &J1, 	double v_percentage, 
							vector<JointValues> &Joint_diff);
		int MoveJ2J_Jspace(	JointPoint &J0,
											JointPoint &J1,JointPoint &J2,
											double v_percentage, int cnt,
											vector<JointValues> &Joint_diff);

		int  MoveJ2J_Cspace(	JointPoint &J0,
											const Pose &P, const Pose &P_nxt, 
											double v_percentage, int cnt,
											vector<JointValues> &Joint_diff);

		int MoveJ2L_Cspace(	const JointPoint &J0,
											const Pose &P1,double v_percentage, const int cnt,
											const Pose P2,const double vl,
											vector<JointValues> &Joint_diff,
											Pose &P0,double &v0, double &vu0,Pose &P_prv);
		int MoveJ2L_Jspace(	const JointPoint &J0,	
											const JointValues &J1,	double v_percentage, const int cnt,	
											const Pose P2,const double vl,	
											vector<JointValues> &Joint_diff,
											Pose &P0,double &v0, double &vu0,Pose &P_prv);

		int MoveJ2C_Jspace(const JointPoint &J0,
			const JointValues &P1, double v_percentage, int cnt,
			const Pose &P2, const Pose &P3, double vc,
			vector<JointValues> &Joint_diff,
			Pose &P0, double &v0);

		int MoveJ2C_Cspace(		const JointPoint &J0,
												const Pose &P1, double v_percentage, int cnt, 
												const Pose &P2,const Pose &P3,double vc,
												vector<JointValues> &Joint_diff, 
												Pose &P0,double &v0);


		int MoveC2J(	const Pose &P0, double v0,
								const Pose &P1, const Pose &P2, double vc, int cnt,
								vector<Pose> &P_diff);
	
		int MoveC2L_part1(	Pose &P0, double &v0,
											const Pose &P1, const Pose &P2, double vc, int cnt,
											const Pose &P3, double vl,
											vector<Pose> &Pose_diff, 
											Pose &P_sta_ptc,  double &vu0, Pose &P_prv);
		int MoveC2L_part2(const JointPoint &J0, const Pose &P0, const Pose &P_sta_ptc, vector <JointValues> &Joint_diff);

		int MoveC2C_part1(	Pose &P0, double &v0,
											const Pose &P1, const Pose &P2, double vc1, int cnt,
											const Pose &P3, const Pose &P4, double vc2,
											vector<Pose> &Pose_diff,
											Pose &P_sta_ptc);
		int MoveC2C_part2(	const JointPoint &J0, const Pose &P0, const Pose &P_sta_ptc, vector <JointValues> &Joint_diff);

		// 暂停与重新启动
		int Pause(		vector<JointValues> &Joint_diff);
		int Restart(	vector<JointValues> &J_diff, JointValues &J0);



		//位姿表示方法相互转化函数
		int PoseQuatern2Matrix(const Pose &P, double Pose_M[4][4]);
		int Matrix2PoseQuatern(double Pose_M[4][4], Pose &P);

		int PoseEuler2Matrix(const PoseEuler &P, double Pose_M[4][4]);
		int Matrix2PoseEuler(double Pose_M[4][4], PoseEuler &P);

		int Quatern2Euler(const Pose &P_Q, PoseEuler &P_E);
		int Euler2Quatern(const  PoseEuler &P_E, Pose &P_Q);
		int Pose2Array(const Pose &P, double PST[3], double ORT[4]);

		int CheckJointVector(const vector<JointValues> J_diff);


private:

        int Circle(	const Pose &P0, double &v0,
							const Pose &P1, const Pose &P2, double vc, int cnt,
							vector<Pose> &P_diff);

		//基础函数
		double min(double a, double b);
		double max(double a, double b);

		//变换矩阵函数
		int RotX(double t, double  M[4][4]);
		int RotY(double t, double  M[4][4]);
		int RotZ(double t, double  M[4][4]);
		int Trans(double x, double y, double z, double M[4][4]);
		int DHMatrix(double L[4], double q, double  M[4][4]);



		//矩阵、向量运算函数
		bool InverseMatrix(double src[4][4], double des[4][4]);
		double InnerProduct(double u[3], double v[3]);
		int nV3(double n, double V[3], double nV[3]);
		int add3(double u[3], double v[3], double w[3]);
		int minus3(double u[3], double v[3], double w[3]);
		int Assign3(double u[3], double v[3]);
		int Assign4(double u[4], double v[4]);
		int Sign(double b);
		double norm(double Vector[3]);
		double norm4(double Vector[4]);
		int Cross(double u[3], double v[3], double w[3]);
		int MULTIPLY_M(double M1[4][4], double M2[4][4], double M[4][4]);
		int MULTIPLY_MV(double M[4][4], double V[4], double U[4]);

		//四元数相关函数
		
		int InterpQ(double time, double q_prv[4], double q_0[4], double q_obj[4], double q_nxt[4], double VU[2], vector<Pose> &Quatern_diff);
		int Squad(double q1[4], double q2[4], double s1[4], double s2[4], double u, double qu[4]);
		int Slerp(double q0[4], double q1[4], double  h, double qh[4]);
		int Polyn3_Q(double t, double v[2], double A[4]);
		int Si(double qiq[4], double qi[4], double qih[4], double si[4]);
		int nV4(double n, double V[4], double nV[4]);
		int add4(double u[4], double v[4], double w[4]);
		int ExpQ(double L[4], double E[4]);
		double InnerProduct4(double u[4], double v[4]);
		int LogQ(double q[4], double l[4]);
		int MultiQ(double u[4], double v[4], double w[4]);
		int InvQ(double q[4], double iq[4]);
		int Quaternprv(double Q1[4], double Q2[4], double Q_prv[4]);
		int OrientSmooth(const vector<Quaternion> &Q, double t_pos[3], vector<Quaternion> &Q_out);


		//运动学函数
		int JUDGE_AXIS(double THETA[6]);
		double REVISE_JOINT(double t);


		//规划和插值
		int Smt_arc(double P0[3], double P1[3], double P2[3], double &r, double B1[3], double B2[3], double O[3], double &angle, double T[4][4]);
		//int Polyn2(double d, double v0, double v_obj, double &v_cnt1, double a, double T[4], double P[3][3]);
		
		int Interpolation_mode(double d, double v0, double v_obj, double &v_cnt1, double a, double &t, vector<double> &di);

		int Polyn2_v2(double d, double v0, double v_obj, double &v_cnt1, double a, double &t,vector<double> &di);
		int Polyn2_t(double d, double v0, double v1, double t, double a, vector<double> &d_diff);
				
		//double  Polyn2_joint(double d, double v0, double v_obj, double v1, double a);
		//int Polyn2_joint_t(double d, double v0, double v1, double t, double a, vector<double> &d_diff);
		//int Interp_3seg(double T[4], double P[3][3], vector<Pose> &Pose_diff);

		int Polyn3_7seg(double d, double v0, double v_obj, double &v_cnt1, double a, double &t, vector<double> &di);
		int Polyn3_seg(double t, double SVA[3], double P[4]);
		//int Interp_7seg(double T[8], double P[7][4], vector<Pose> &Pose_diff);

		int Interp_singlejoint_t(double d, double v0, double v1, double t, double P[4]);
		int Interp_joints(const JointPoint &J0, const JointPoint &J1, vector<JointValues> &J_diff);
		
		int Arc_3points(double P1[3], double P2[3], double P3[3], double T[4][4], double &r, double &theta3, double &theta2, double O[3]);

		//Judge the objective pose, if same with current pose ,return error code
		int CheckPoint(const Pose &P1, const Pose &P2);

		//configurable parameters
		
		double DH[6][4] ;
		double 	UserFrame[4][4] ;
		double ToolFrame[4][4];
		double AXIS_LIMIT[4][6];
		double acc;
		double ja_ratio;
		double tc;
		double limit_scale;          //速度限制，各轴参考速度的倍数
		double overshoot;
		double errorangle;
		double accelerationoverload;
		double omega_ref;
		double vel_ref;
		double k_radius;
		double minim;
		double estime_coefficient;
		int curve_mode;

		//un-configurable parameters
		double pi;
		double InvUF[4][4] ;
		double InvTF[4][4] ;
		double InvT66[4][4] ;
		double a1;
		double a2 ;
		double a3 ;
		double d1 ;
		double d3 ;
		double d4 ;
		double offset1 ;
		double offset2;
		double offset3 ;
		double offset4;
		double offset5 ;
		double offset6 ;


	};
	

}


#endif