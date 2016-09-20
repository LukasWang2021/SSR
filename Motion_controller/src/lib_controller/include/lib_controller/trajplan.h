#ifndef _trajplan_h
#define _trajplan_h
#include <iostream>
#include "fst_datatype.h"
#include <cmath>
#include <vector>
#include <cstring>
using namespace std;
// double pi=3.14;
// double DH[4][4];

namespace fst_controller
{
	class TrajPlan
	{
	public:
		TrajPlan();
//		const static double pi;

		void setDH(double _alp0, double _a0, double _d1, double _os1, double _alp1, double _a1, double _d2, double _os2, double _alp2, double _a2, double _d3, double _os3, double _alp3, double _a3, double _d4, double _os4, double _alp4, double _a4, double _d5, double _os5, double _alp5, double _a5, double _d6, double _os6);

		void setToolFrame(const fst_controller::Point &position,const fst_controller::Euler &orientation);

		void setUserFrame(const fst_controller::Point &position, const fst_controller::Euler &orientation);

		void setAxisLimit(const fst_controller::JointConstraints &axislimit);

		void setCycleTime(double _tc);

		void setAcceleration(double _acc);

		void setOvershoot(double _overshoot);

		void seterrorangle(double _errorangle);

		void setoverload(double _accelerationoverload);

		void setLimitScale(double _limit_scale);

		int InverseKinematics(const fst_controller::Pose &pose, const fst_controller::JointValues &joint_reference, fst_controller::JointValues &joint_result);

		int ForwardKinematics(const fst_controller::JointValues &joint, fst_controller::Pose &pose);

		
		int MoveL(fst_controller::Pose &pose_start, double &v0, double &vu0,
						const fst_controller::Pose &pose_obj, const double v_obj, const int cnt_obj,
						const fst_controller::Pose &pose_nxt, const double v_nxt, const int cnt_nxt,
								fst_controller::Pose &pose_prv,
							vector<fst_controller::Pose> &Pose_diff);
		int MoveL2J(const Pose &P0, double v0, double vu0,
							const Pose &P1, double v_obj, int cnt_obj,
							Pose &P_prv, 
							vector<fst_controller::Pose> &Pose_diff, JointPoint &J0);


		int MoveJ(const fst_controller::JointPoint &J0, 
						 const  fst_controller::JointPoint &J1, 
						 int v_percentage, vector<fst_controller::JointValues> &Joint_diff);

		int MoveJ2J_Jspace(const fst_controller::JointPoint &J0,
			fst_controller::JointPoint &J1,
			int v_percentage, int cnt,
			vector<fst_controller::JointValues> &Joint_diff,
			fst_controller::JointPoint &J_out);

		int  MoveJ2J_Cspace(const fst_controller::JointPoint &J0,				//当前位置和速度	
			const fst_controller::Pose &P,				//目标位置
			int v_percentage, int cnt,									//目标速度（百分比，取值：1-100）和cnt	（取值0-100）
			vector<fst_controller::JointValues> &Joint_diff,		//返回结果
			fst_controller::JointPoint &J_out);


		int MoveJ2L_Cspace(const JointPoint &J0,				//当前位置和速度	
			const Pose &P1,				//目标位置
			const int v_percentage, const int cnt,									//目标速度和cnt	
			const Pose P2,									//预读下一点位姿
			const double vl,														//直线轨迹速度
			vector<JointValues> &Joint_diff,		//返回结果
			Pose &P0,									//返回下条指令的初值
			double &v0, double &vu0,
			Pose &P_prv);

		int MoveJ2L_Jspace(
			const JointPoint &J0,				//当前位置和速度	
			const JointValues &J1,				//目标位置
			const int v_percentage, const int cnt,									//目标速度和cnt	
			const Pose P2,									//预读下一点位姿
			const double vl,														//直线轨迹速度
			vector<JointValues> &Joint_diff,		//返回结果
			Pose &P0,									//返回下条指令的初值
			double &v0, double &vu0,
			Pose &P_prv
			);

		int print4(double V[4]);
		int print3(double V[3]);
		int print6(double V[6]);
		int POSE2MATRIX(double position[3], double orientation[4], double Pose_M[4][4]);
		int MATRIX2POSE(double Pose_M[4][4], double position[3], double orientation[4]);
		int Euler2Matrix(double x, double y, double z, double a, double b, double c, double Pose_M[4][4]);
		int Matrix2Euler(double R[4][4], double& x, double &y, double &z, double &a, double &b, double &c);


private:



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
		int Assign3(double u[3], double v[3]);
		int Assign4(double u[4], double v[4]);
		int Sign(double b);
		double norm(double Vector[3]);
		int Cross(double u[3], double v[3], double w[3]);
		int MULTIPLY_M(double M1[4][4], double M2[4][4], double M[4][4]);
		int MULTIPLY_MV(double M[4][4], double V[4], double U[4]);

		//四元数相关函数
		
		int InterpQ(int n_time, double q_prv[4], double q_0[4], double q_obj[4], double q_nxt[4], double VU[2], vector<fst_controller::Pose> &Quatern_diff);
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


		//运动学函数
		int JUDGE_AXIS(double THETA[6]);
		double REVISE_JOINT(double t);


		//规划和插值
		int Smt_arc(double P0[3], double P1[3], double P2[3], double r, double B1[3], double B2[3], double O[3], double *angle, double T[4][4]);
		int Polyn2(double d, double v0, double v_obj, double *v_cnt1, double a, double T[4], double P[3][3]);
		double  Polyn2_joint(double d, double v0, double v_obj, double v1, double a);
		int Polyn2_joint_t(double d, double v0, double v1, double t, double a, double T[4], double P[3][4]);
		int Interp_singlejoint_t(double d, double v0, double v1, double t, double P[4]);

		double  pi;
		double DH[6][4] ;
		double 	UserFrame[4][4] ;
		double InvUF[4][4] ;
		double ToolFrame[4][4];
		double InvTF[4][4] ;
		double InvT66[4][4] ;
		double AXIS_LIMIT[4][6] ;
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
		double acc;
		double tc;
		double limit_scale;          //速度限制，各轴参考速度的倍数
		double overshoot;
		double errorangle;
		double accelerationoverload;
	};
	

}


#endif
