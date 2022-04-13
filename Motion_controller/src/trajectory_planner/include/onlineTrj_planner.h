#ifndef _ONLINE_TRJ_PLANNER_H_
#define _ONLINE_TRJ_PLANNER_H_
#include "basic_alg.h"
#include "quaternion.h"
#include "matrix33.h"
#include "vector3.h"
#include <algorithm>

typedef struct {
    int  status;
    double x_;
    double y_;
    double z_;
    double a_;
    double b_;
    double c_;
}TrjPoint;

using namespace std;
using namespace basic_alg;

class OnlineTrajectoryPlanner
{
public:
    //int  trjPointCnt;
    TrjPoint trj_point_buf[160];
    OnlineTrajectoryPlanner();
    ~OnlineTrajectoryPlanner();
    int sign(double x);//取double数字的符号
    double roundn(double x, int precision);//精确设置小数点位数精度
    Matrix33 rpy2r(double a, double b, double c);//欧拉角转旋转矩阵
    Vector3 rtm_rpy(Matrix33& R);//旋转矩阵转欧拉角

    Quaternion rtm_r2quat(Matrix33& R);//矩阵转四元数
    Matrix33 rtm_quat2r(Quaternion& q);//四元数转矩阵

    Matrix33 skew(Vector3& vec3);
    Quaternion quatinv(Quaternion& q);
    Quaternion quatlog(Quaternion& q);
    Quaternion quatexp(Quaternion& q);
    double quat_dot_multiply(Quaternion& q1, Quaternion& q2);//两个四元数点乘
    Quaternion quatmultiply(Quaternion& q1, Quaternion& q2);//两个四元数叉乘
    Vector3 update_wgoal(Matrix33& M, Quaternion& qtemp, Vector3& alpha, double tTmin, double ts);//更新wgoal

    Vector3 rtm_quat2abc(Quaternion& Q);
    Quaternion rtm_abc2quat(Vector3& abc);
    Vector3 rtm_abcDiff(Vector3& abc,Vector3 abc_rate);//计算当前轴角速度分别在xyz的投影，即绕xyz旋转的分量
    Quaternion rtm_Slerpt(Quaternion& q0, Quaternion& q1, double t);//
    Quaternion rtm_Squad(Quaternion& q0, Quaternion& q1, Quaternion& q2,Quaternion& q3,double t);
    void traj_on_FB(int m, int lambda, int Nstep, Vector3 VPp_point[], int NVP, Vector3 Pout[]);
    //Vector3 * traj_on_FB(int m, int lambda, int Nstep, Vector3 * VPp);
    //void traj_on_Squad(int Nstep, Vector3* VPp, Quaternion* Qnew, Vector3* abc, Quaternion* Qold);
    void traj_on_Squad(int Nstep, Vector3 VPp_abc[], int NVP, Quaternion Qnew[], Vector3 abc[], Quaternion Qold[]);
    //int traj_on_FIR_Bspline(Vector3 xyz, Vector3 abc,int status);
    int  traj_on_FIR_Bspline(Vector3 xyz, Vector3 abc,int status, int online_TrjpointBufIndex);
    void Fir_Bspline_algorithm_test(void);
    void Fir_Bspline_algorithm_test2(void);
    void TrjPointPlanning();
    void pointer_fuzhi_test(Quaternion q_in[], int num, Quaternion q_out[]);
    void function_test();
    bool FixedBaseCoordTransformation(Matrix44& T_touchm,Matrix44& resM);
    bool DynamicBaseCoordTransformation(Matrix44 T_r0_R, Matrix44 Touch_h0_v,  Matrix44 Touch_ht_v, double k,Matrix44& resM);
    void rtm_r2xyzabc(Matrix44& u,Vector3& res_xyz, Vector3& res_abc);
};

#endif