#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include "onlineTrj_planner.h"
#include <sstream>
#include "math.h"
#include <cmath>
#include <iomanip>


float VPp[1][6] = { {0.1, 0.2, 0.3, 0, 0, 0}
                    //{0.1, 0.2, 0.3, 0.2*PI, 0.3*PI, 0.3*PI},
                    //{0.2, 0.3, 0.8, 0.3*PI, 0.2*PI, 0.1*PI},
                };
float VPv[2][6] = { {0.0, 0.0, 0.0, 0, 0, 0},
                    {0.1, 0.2, 0.1, 0.1*PI, 0.2*PI, 0.3*PI},
                    //{0.3, 0.3, 0.2, 0.1*PI, 0.2*PI, 0.3*PI},
                };

//#define Tsample     0.002 //touch采样时间间隔
//#define GEN_TN      0.001 //生成轨迹间隔
//#define NstepP      5     //计算位置-取VP点的采样点间隔--->每5个采样点取一个xyz向量  范围<=5
//#define NstepQ      25    //计算姿态-取VQ的采样点间隔--->每25个采样点取一个abc向量
//#define NinterpP    ((NstepP*Tsample)/GEN_TN)  //5*0.002/0.001 = 10
//#define NinterpQ    ((NstepQ*Tsample)/GEN_TN)  //25*0.002/0.001 = 50

#define LAMBDA 10  //B样条3级串联滤波器脉冲响应系数索引值[1,10,100]
#if(LAMBDA == 1)
double hfir[6] = {0.4018,0.2424,0.0841,0.0041,-0.0174,-0.0140};
#elif(LAMBDA == 10)
double hfir[6] = {0.1952,0.1666,0.1183,0.0714,0.0350,0.0112};
#elif(LAMBDA == 100)
double hfir[6] = {0.1252,0.1191,0.1056,0.0886,0.0706,0.0535};
#endif


OnlineTrajectoryPlanner::OnlineTrajectoryPlanner()
{
    online_trajectory_algorithm_params_init();
}
OnlineTrajectoryPlanner::~OnlineTrajectoryPlanner()
{

}
//取double数字的符号
int OnlineTrajectoryPlanner::sign(double x)
{
    if(x < 0) return -1;
    else return 1;
}
/****************************************
 * 计算指定精度后的浮点数
 * *******************************************/
double OnlineTrajectoryPlanner::roundn(double x, int precision)
{
    stringstream ss;
    ss <<fixed<<setprecision(precision)<<x;
    ss >> x;
    return x;
}

//abc转旋转矩阵
Matrix33 OnlineTrajectoryPlanner::rpy2r(double a, double b, double c)
{
    Matrix33 Rx,Ry,Rz,result_R;
    double ct,st;
    ct = cos(a);
    st = sin(a);
    Rx.eye();
    Rx.matrix_[1][1] = ct;
    Rx.matrix_[2][2] = ct;
    Rx.matrix_[1][2] = -st;
    Rx.matrix_[2][1] = st;
    Ry.eye();
    ct = cos(b);
    st = sin(b);
    Ry.matrix_[0][0] = ct;
    Ry.matrix_[2][2] = ct;
    Ry.matrix_[0][2] = st;
    Ry.matrix_[2][0] = -st;
    Rz.eye();
    ct = cos(c);
    st = sin(c);
    Rz.matrix_[0][0] = ct;
    Rz.matrix_[1][1] = ct;
    Rz.matrix_[0][1] = -st;
    Rz.matrix_[1][0] = st;
    result_R = Rz.rightMultiply(Ry);
    result_R = result_R.rightMultiply(Rx);
    return result_R;
}
//旋转矩阵转四元数
Quaternion OnlineTrajectoryPlanner::rtm_r2quat(Matrix33& R)
{
    Quaternion q;
    double r11,r12,r13,r21,r22,r23,r31,r32,r33;
    r11 = R.matrix_[0][0];
    r12 = R.matrix_[0][1];
    r13 = R.matrix_[0][2];
    r21 = R.matrix_[1][0];
    r22 = R.matrix_[1][1];
    r23 = R.matrix_[1][2];
    r31 = R.matrix_[2][0];
    r32 = R.matrix_[2][1];
    r33 = R.matrix_[2][2];
    q.w_ = 0.5*sqrt(r11+r22+r33+1);
    q.x_ = 0.5*sign(r32-r23)*sqrt(r11-r22-r33+1);
    q.y_ = 0.5*sign(r13-r31)*sqrt(r22-r11-r33+1);
    q.z_ = 0.5*sign(r21-r12)*sqrt(r33-r11-r22+1);
    return q;
}
//旋转矩阵转四元数
/*
Quaternion OnlineTrajectoryPlanner::rtm_r2quat(Matrix33& R)
{
    int accurate = 8;//精确到小数点后8位
    Quaternion q;
    double r11,r12,r13,r21,r22,r23,r31,r32,r33;
    r11 = roundn(R.matrix_[0][0],accurate);
    r12 = roundn(R.matrix_[0][1],accurate);
    r13 = roundn(R.matrix_[0][2],accurate);
    r21 = roundn(R.matrix_[1][0],accurate);
    r22 = roundn(R.matrix_[1][1],accurate);
    r23 = roundn(R.matrix_[1][2],accurate);
    r31 = roundn(R.matrix_[2][0],accurate);
    r32 = roundn(R.matrix_[2][1],accurate);
    r33 = roundn(R.matrix_[2][2],accurate);
    q.w_ = 0.5*sqrt(roundn(r11+r22+r33+1,accurate));
    q.x_ = 0.5*sign(r32-r23)*sqrt(roundn(r11-r22-r33+1,accurate));
    q.y_ = 0.5*sign(r13-r31)*sqrt(roundn(r22-r11-r33+1,accurate));
    q.z_ = 0.5*sign(r21-r12)*sqrt(roundn(r33-r11-r22+1,accurate));
    return q;
}
*/

//四元数转矩阵
Matrix33 OnlineTrajectoryPlanner::rtm_quat2r(Quaternion& q)
{
    /*
    R = [1 - 2*(y.^2 + z.^2),   2*(x.*y - s.*z),   2*(x.*z + s.*y);
            2*(x.*y + s.*z), 1 - 2*(x.^2 + z.^2),   2*(y.*z - s.*x);
            2*(x.*z - s.*y),   2*(y.*z + s.*x), 1 - 2*(x.^2 + y.^2) ];
    */
    double s,x,y,z;
    Matrix33 R;
    s = q.w_;
    x = q.x_;
    y = q.y_;
    z = q.z_;
    R.matrix_[0][0] = 1-2*(pow(y,2)+pow(z,2));
    R.matrix_[0][1] = 2*(x*y - s*z);
    R.matrix_[0][2] = 2*(x*z + s*y);
    R.matrix_[1][0] = 2*(x*y + s*z);
    R.matrix_[1][1] = 1-2*(pow(x,2)+pow(z,2));
    R.matrix_[1][2] = 2*(y*z - s*x);
    R.matrix_[2][0] = 2*(x*z - s*y);
    R.matrix_[2][1] = 2*(y*z + s*x);
    R.matrix_[2][2] = 1-2*(pow(x,2)+pow(y,2));
    return R;
}
/********************************
 * 函数功能: 三维向量按照下图方式转矩阵
 * |x|         | 0   -z   y |
 * |y| ===>    | z   0   -x |
 * |z|         |-y   x    0 |
**********************************/
Matrix33 OnlineTrajectoryPlanner::skew(Vector3& vec3)
{
    Matrix33 resMat;
    resMat.matrix_[0][0] = 0;resMat.matrix_[0][1] = -vec3.z_;resMat.matrix_[0][2] = vec3.y_;
    resMat.matrix_[1][0] = vec3.z_;resMat.matrix_[1][1] = 0;resMat.matrix_[1][2] = -vec3.x_;
    resMat.matrix_[2][0] = -vec3.y_;resMat.matrix_[2][1] = vec3.x_;resMat.matrix_[2][2] = 0;
    return resMat;
}
//四元数点乘
double OnlineTrajectoryPlanner::quat_dot_multiply(Quaternion& q1, Quaternion& q2)
{
    double res;
    res = q1.w_ * q2.w_ + q1.x_ * q2.x_ + q1.y_ * q2.y_ + q1.z_ * q2.z_;
    return res;
}
//四元数叉乘
Quaternion OnlineTrajectoryPlanner::quatmultiply(Quaternion& q1, Quaternion& q2)
{
    Quaternion res_quat;
    res_quat.w_ = q1.w_*q2.w_ - q1.x_*q2.x_ - q1.y_*q2.y_ - q1.z_*q2.z_;
    res_quat.x_ = q1.w_*q2.x_ + q1.x_*q2.w_ + q1.y_*q2.z_ - q1.z_*q2.y_;
    res_quat.y_ = q1.w_*q2.y_ - q1.x_*q2.z_ + q1.y_*q2.w_ + q1.z_*q2.x_;
    res_quat.z_ = q1.w_*q2.z_ + q1.x_*q2.y_ - q1.y_*q2.x_ + q1.z_*q2.w_;
    return res_quat;
}
//四元数取逆
Quaternion OnlineTrajectoryPlanner::quatinv(Quaternion& q)
{
    double q_norm = q.norm();
    Quaternion resQ;
    if(q_norm != 0)
    {
        resQ.w_ = q.w_/q_norm;
        resQ.x_ = -q.x_/q_norm;
        resQ.y_ = -q.y_/q_norm;
        resQ.z_ = -q.z_/q_norm;
    }
    else
    {
        resQ.w_ = 0;
        resQ.x_ = 0;
        resQ.y_ = 0;
        resQ.z_ = 0;
    }
    return resQ;
}
//计算四元数的对数, 输入q的模为1
Quaternion OnlineTrajectoryPlanner::quatlog(Quaternion& q)
{
    Quaternion resQ;
    double qV_norm = sqrt(q.x_*q.x_ + q.y_*q.y_ + q.z_*q.z_);
    double fi = atan2(qV_norm, q.w_);
    if(qV_norm == 0)
    {fi =0;}
    else
    {fi = fi/qV_norm;}
    resQ.w_ = 0;
    resQ.x_ = fi*q.x_;
    resQ.y_ = fi*q.y_;
    resQ.z_ = fi*q.z_;
    return resQ;
}

//四元数以e为底的幂运算
Quaternion OnlineTrajectoryPlanner::quatexp(Quaternion& q)
{
    Quaternion resQ;
    double qV_norm = sqrt(q.x_*q.x_ + q.y_*q.y_ + q.z_*q.z_);
    resQ.w_ = cos(qV_norm)*exp(q.w_);
    if(qV_norm != 0)
    {
        resQ.x_ = q.x_ * (sin(qV_norm)/qV_norm)*exp(q.w_);
        resQ.y_ = q.y_ * (sin(qV_norm)/qV_norm)*exp(q.w_);
        resQ.z_ = q.z_ * (sin(qV_norm)/qV_norm)*exp(q.w_);
    }
    else {
        resQ.x_ = 0;
        resQ.y_ = 0;
        resQ.z_ = 0;
    }
    return resQ;
}
//旋转矩阵转欧拉角
Vector3 OnlineTrajectoryPlanner::rtm_rpy(Matrix33& u)
{
    double min_value = 1e-18;
    Vector3 angle1,angle2;
    double phi_1, phi_2, psi_1, psi_2, theta_1,theta_2;
    angle1.zero();
    angle2.zero();

    if((abs(u.matrix_[2][0]-1) < min_value)  || (abs(u.matrix_[2][0]+1) < min_value))
    {
        if(abs(u.matrix_[2][0]+1) < min_value)
        {
            theta_1 = PI/2;
            phi_1 = 0;
            psi_1 = phi_1 + atan2(u.matrix_[0][1], u.matrix_[0][2]);
        }
        else
        {
            theta_1 = -PI/2;
            phi_1 = 0;
            psi_1 = -phi_1 + atan2(-u.matrix_[0][1], -u.matrix_[0][2]);
        }
    }
    else
    {
        theta_1 = -asin(u.matrix_[2][0]);
        theta_2 = PI - theta_1;
        psi_1 = atan2(u.matrix_[2][1]/cos(theta_1), u.matrix_[2][2]/cos(theta_1));
        psi_2 = atan2(u.matrix_[2][1]/cos(theta_2), u.matrix_[2][2]/cos(theta_2));
        phi_1 = atan2(u.matrix_[1][0]/cos(theta_1), u.matrix_[0][0]/cos(theta_1));
        phi_2 = atan2(u.matrix_[1][0]/cos(theta_2), u.matrix_[0][0]/cos(theta_2));
        
        /*if(phi_1 < 0)
        {
            phi_1 = phi_1 + 2*PI;
        }
        if(phi_2 < 0)
        {
            phi_2 = phi_2 + 2*PI;
        }*/
    }
    angle1.x_ = psi_1;
    angle1.y_ = theta_1;
    angle1.z_ = phi_1;
    return angle1;
}
//四元数转方向向量
Vector3 OnlineTrajectoryPlanner::rtm_quat2abc(Quaternion& Q)
{
    Matrix33 R;
    Vector3 abc;
    double norm = Q.norm();
    if(norm != 0)
    {
        Q.w_ = Q.w_/norm;
        Q.x_ = Q.x_/norm;
        Q.y_ = Q.y_/norm;
        Q.z_ = Q.z_/norm;
    }
    R = rtm_quat2r(Q);
    abc = rtm_rpy(R);
    return abc;
}
//方向向量转四元数
Quaternion OnlineTrajectoryPlanner::rtm_abc2quat(Vector3& abc)
{
    Matrix33 mrt;
    Quaternion resq;
    mrt = rpy2r(abc.x_,abc.y_,abc.z_);
    resq = rtm_r2quat(mrt);
    return resq;
}
//wgoal=((inv(M))*(2*Qtemp(2:4)-alpha*(Tmin^2)/2)')'+alpha*ts; %这里vgoal就是论文里的vk+1_goal
Vector3 OnlineTrajectoryPlanner::update_wgoal(Matrix33& M, Quaternion& qtemp, Vector3& alpha, double tTmin, double ts)
{
    Matrix33 invM;
    Vector3 resV;
    Vector3 vtempL3;//用于暂存qtemp的后三个数字

    invM = M;
    invM.inverse(0.0);
    vtempL3.x_ = qtemp.x_*2 - alpha.x_*(pow(tTmin,2)/2);
    vtempL3.y_ = qtemp.y_*2 - alpha.y_*(pow(tTmin,2)/2);
    vtempL3.z_ = qtemp.z_*2 - alpha.z_*(pow(tTmin,2)/2);

    resV.x_ = invM.matrix_[0][0]*vtempL3.x_ + invM.matrix_[0][1]*vtempL3.y_ + invM.matrix_[0][2]*vtempL3.z_ + alpha.x_*ts;
    resV.y_ = invM.matrix_[1][0]*vtempL3.x_ + invM.matrix_[1][1]*vtempL3.y_ + invM.matrix_[1][2]*vtempL3.z_ + alpha.y_*ts;
    resV.z_ = invM.matrix_[2][0]*vtempL3.x_ + invM.matrix_[2][1]*vtempL3.y_ + invM.matrix_[2][2]*vtempL3.z_ + alpha.z_*ts;
    return resV;
}
/********************************************
 * 计算当前轴角速度分别在xyz的投影，即绕xyz旋转的分量
 * abc是当前RPY值，abc_rate是当前RPY的角微分量
 * *******************************************/
Vector3 OnlineTrajectoryPlanner::rtm_abcDiff(Vector3& abc,Vector3 abc_rate)
{
    Vector3 resV;
    resV.x_ = cos(abc.y_)*cos(abc.z_)*abc_rate.x_ - sin(abc.z_)*abc_rate.y_;
    resV.y_ = cos(abc.y_)*sin(abc.z_)*abc_rate.x_ + cos(abc.z_)*abc_rate.y_;
    resV.z_ = -sin(abc.y_)*abc_rate.x_ + abc_rate.z_;
    return resV;
}

//获取q0到q1的t时刻插补值
Quaternion OnlineTrajectoryPlanner::rtm_Slerpt(Quaternion& q0, Quaternion& q1, double t)
{
    double Q_theta,sinv,k0,k1;
    Quaternion Qt;
    if(t<0 || t>1)//检查t是否已经归一化在0到1之间
    {
        cout << "t应该归一化在0到1之间"<<endl;
    }
    double dq = q0.w_*q1.w_ + q0.x_*q1.x_ + q0.y_*q1.y_ + q0.z_*q1.z_;//四元数点积的物理意义是差值??
    //计算姿态并判断插补最短路径
    if(dq<0)
    {
        //两四元数的dot product表示两个Q之间的夹角余弦，取反后，选取最短路径
        q1.w_ = -q1.w_;
        q1.x_ = -q1.x_;
        q1.y_ = -q1.y_;
        q1.z_ = -q1.z_;
        dq = -dq;
    }
    else if(dq > 1)
    {
        dq = 1;
    }
    //计算Q之间的夹角,并取最短路径
    Q_theta = acos(dq);
    if(abs(Q_theta) > 1e-8)
    {
        sinv = 1/(sin(Q_theta));
        k0 = sin((1-t)*Q_theta);
        k1 = sin(t*Q_theta);
        Qt = q0*k0;
        Qt = Qt + q1*k1;
        Qt = Qt*sinv;
    }
    else
    {
        Qt = q1;
    }
    Qt = Qt/(Qt.norm());
    return Qt;
}

/********************************************************************
 * Q12=squad(Q0,Q1,Q2,q3,t) 从Q1到Q2进行球面插值 Q0 Q3 对应12段前后的VP点
 * 如果是初始段则为Q0 Q0 Q1 Q2 结束段为Q0 Q1 Q2 Q2
 **********************************************************************/
Quaternion OnlineTrajectoryPlanner::rtm_Squad(Quaternion& q0, Quaternion& q1, Quaternion& q2,Quaternion& q3,double t)
{
    Quaternion s1,s2,q12,qt1,qt2;
    //MATLAB  s1=quatmultiply(q1,quatexp((quatlog(quatmultiply(quatinv(q0),q1))-quatlog(quatmultiply(quatinv(q1),q2)))/4));
    qt1 = quatinv(q0);
    qt1 = quatmultiply(qt1,q1);
    qt1 = quatlog(qt1);
    qt2 = quatinv(q1);
    qt2 = quatmultiply(qt2,q2);
    qt2 = quatlog(qt2);
    s1 = qt1-qt2;
    s1 = s1/4;
    s1 = quatexp(s1);

    s1 = quatmultiply(q1,s1);
    //MATLAB  s2=quatmultiply(q2,quatexp((quatlog(quatmultiply(quatinv(q1),q2))-quatlog(quatmultiply(quatinv(q2),q3)))/4));
    qt1 = quatinv(q1);
    qt1 = quatmultiply(qt1,q2);
    qt1 = quatlog(qt1);
    qt2 = quatinv(q2);//---
    qt2 = quatmultiply(qt2,q3);//---
    qt2 = quatlog(qt2);//---
    s2 = qt1-qt2;
    s2 = s2/4;
    s2 = quatexp(s2);
    s2 = quatmultiply(q2,s2);
    //MATLAB   q12=rtm_Slerpt(rtm_Slerpt(q1,q2,t),rtm_Slerpt(s1,s2,t),2*t*(1-t));
    qt1 = rtm_Slerpt(q1,q2,t);
    qt2 = rtm_Slerpt(s1,s2,t);
    q12 = rtm_Slerpt(qt1,qt2,2*t*(1-t));

    return q12;
}

/******************************************
 * 函数功能:
 * 参数说明:
 * Nstep--输入
 * VPp_abc----输入VP点姿态数据
 * NVP---加载的VP点数量
 * Qnew[]---输出
 * abc[]----输出矩阵,n行3列
 * Qold[]---输出
 * ******************************************/
void OnlineTrajectoryPlanner::traj_on_Squad(int Nstep, Vector3 VPp_abc[], int NVP, Quaternion Qnew[], Vector3 abc[], Quaternion Qold[])
{
    double t_NinterpP = online_alg_params_.N_interp_P;//2022-6-10 pm
    Quaternion Qs[255];
    Quaternion Q0,Q1,Q2,Q3;
    double dq;

    int i=0,j=0,k=0,end;
    double s;
    for(i=0;i<NVP;i=i+Nstep)//每Nstep个点加载一个VP点
    {
        //Qs(ceil(i/Nstep),:)=rtm_abc2quat(VPp(i,4:6));
        end = ceil(i/Nstep);
        end = end-1;//c++数组下标从0开始
        Qs[end] = rtm_abc2quat(VPp_abc[i]);
        if(i>=(2*Nstep+1))//需要至少三个VP点才能开始规划
        {
            Q0 = Qs[end-1];
            Q1 = Qs[end];
            dq = Q0.w_*Q1.w_ + Q0.x_*Q1.x_ + Q0.y_*Q1.y_ + Q0.z_*Q1.z_;
            if(dq < 0)
            {
                //两四元数的dot product表示两个Q之间的夹角余弦，取反后，选取最短路径
                Qs[end].w_ = -Q1.w_;
                Qs[end].x_ = -Q1.x_;
                Qs[end].y_ = -Q1.y_;
                Qs[end].z_ = -Q1.z_;
            }
            if(j==0)//初始段
            {
                Q0 = Qs[0];  Q1 = Q0;  Q2 = Qs[2]; Q3 = Qs[3];
            }
            else if(j == ceil(NVP/Nstep))//最后一段
            {
                Q0 = Qs[end-2]; Q1 = Qs[end-1]; Q2 = Qs[end]; Q3 = Q2;
            }
            else
            {
                Q0 = Qs[j-1]; Q1 = Qs[j]; Q2 = Qs[j+1]; Q3 = Qs[j+2];
            }
            for(k=0;k<t_NinterpP;k++)
            {
                s = k/t_NinterpP;
                Qnew[static_cast<int>(k+t_NinterpP*(j-1)+1)] = rtm_Squad(Q0,Q1,Q2,Q3,s);
                abc[static_cast<int>(k+t_NinterpP*(j-1)+1)] = rtm_quat2abc(Qnew[static_cast<int>(k+t_NinterpP*(j-1)+1)]);//
            }
            j = j+1;
        }
    }
    Qold = Qs;
}


/********************
 * 函数功能:
 * 参数说明:
 * 一个采样点数据 xyz, abc
 * status: 状态 0-起点(按下按钮)  1-中间过程(hold按钮) 2-终点(松开按钮)
 * online_TrjpointBufIndex
 * 返回值: 生成结果途经点的数量
 * *************************/
int  OnlineTrajectoryPlanner::traj_on_FIR_Bspline(Vector3 xyz, Vector3 abc,int status, int online_TrjpointBufIndex)
{
    int m = 5;//B样条近似阶数,越大越接近实际值，但是滤波器延迟也会变大(m最大取值为5,可选3或4)
    static int sp_cnt = 0;//采样点计数
    static int vp_cnt = 0;//vp点计数
    static int vq_cnt = 0;//vq点计数
    bool flag_getVpFromTouch = false;
    bool flag_getVqFromTouch = false;

    static double VPx[31],VPy[31],VPz[31];
    static double cx[5],cy[5],cz[5];
    static double cx_ts[50],cy_ts[50],cz_ts[50];//实际使用长度40
    const int hfir_idx[11] = {5,4,3,2,1,0,1,2,3,4,5};
    int hfir_idxidx = 0;
    static Vector3 px_new[31];//单次xyz规划输出,实际使用31

    static Quaternion Q0,Q1,Q2,Q3;
    double dq;
    static int j=0;

    static Quaternion Qnew[50];//
    static Vector3 out_abc[50];//单次abc规划输出
    static Vector3 out_xyz_buf[255];
    static Vector3 out_abc_buf[255];
    static int out_xyz_cnt=0;
    static int out_abc_cnt=0;
    static int out_cnt=0;
    static int out_status=0;//默认输出轨迹点状态为起点
    int NP = static_cast<int>(online_alg_params_.N_interp_P);//has been static_cast to int when init().
    int res_PointCnt = 0;
    //每NstepP记录一个VP NstepP==5
//cout << "sp_cnt="<<sp_cnt<<endl;
 if(sp_cnt%online_alg_params_.N_step_P == 0) {flag_getVpFromTouch = true;} else {flag_getVpFromTouch = false;}
    //每NstepQ记录一个VQ NstepQ==25
    //if(sp_cnt%NstepQ == 0) {flag_getVqFromTouch = true;} else {flag_getVqFromTouch = false;}
    if(sp_cnt%online_alg_params_.N_step_Q == 0) {flag_getVqFromTouch = true;} else {flag_getVqFromTouch = false;}
    if(status == 0)//轨迹起点--按下按键后立即传入静止状态的机械臂末端位姿
    {
        sp_cnt = 0; vp_cnt = 0; vq_cnt = 0;j=0;
//cout <<"start point"<<endl;
        for(int i=0;i<m;i++){cx[i] = xyz.x_;cy[i] = xyz.y_;cz[i] = xyz.z_;}//cx=ones(1,m)*px(0);
        for(int i=0;i<m*NP;i++){cx_ts[i] = cx[0];cy_ts[i] = cy[0];cz_ts[i] = cz[0];}//cx_ts=ones(1,m*NinterpP)*cx(0);
        for(int i=0;i<2*m;i++)//0~2m-1
        {
            VPx[i] = xyz.x_;
            VPy[i] = xyz.y_;
            VPz[i] = xyz.z_;
        }
        flag_getVpFromTouch = true;
        flag_getVqFromTouch = true;
        out_xyz_cnt = 0;
        out_abc_cnt = 0;
        out_cnt=0;
        out_status=0;
        //sp_cnt++;//采样点计数自增
    }else if(status == 2)//终点---(松开按钮时会发送一次位置点)
    {
        flag_getVpFromTouch = true;
        flag_getVqFromTouch = true;
    }
    //采样点计数根据取VP点和VQ点间隔的公倍数清零
    if(sp_cnt >= 2*online_alg_params_.N_step_P*online_alg_params_.N_step_Q) 
    {sp_cnt = online_alg_params_.N_step_P*online_alg_params_.N_step_Q;}
    sp_cnt++;//采样点计数自增
    if(flag_getVpFromTouch)
    {
        VPx[2*m] = xyz.x_;//vp_cnt = 2*m;
        VPy[2*m] = xyz.y_;
        VPz[2*m] = xyz.z_;
        if(vp_cnt < (2*m+1)) vp_cnt++;
        double t_sum_cx=0,t_sum_cy=0,t_sum_cz=0;
        hfir_idxidx = 5-m;//确定hfir_idx的查找起始下标
        for(int i=0;i<(2*m+1);i++)
        {
            t_sum_cx += hfir[hfir_idx[hfir_idxidx]]*VPx[i];
            t_sum_cy += hfir[hfir_idx[hfir_idxidx]]*VPy[i];
            t_sum_cz += hfir[hfir_idx[hfir_idxidx]]*VPz[i];
            hfir_idxidx++;
        }
        cx[3] = t_sum_cx;
        cy[3] = t_sum_cy;
        cz[3] = t_sum_cz;
        for(int i=0;i<NP;i++)//每一个VP点要生成Ninterp个cx_ts
        {
            cx_ts[3*NP + i] = cx[3];
            cy_ts[3*NP + i] = cy[3];
            cz_ts[3*NP + i] = cz[3];
        }
        for(int i=0;i<2*m;i++)
        {
            VPx[i] = VPx[i+1];VPy[i] = VPy[i+1];VPz[i] = VPz[i+1];
        }
        //滤波函数
        if(vp_cnt == 1)
        {
            for(int i=0;i<NP;i++)
            {
                switch(i){
                    case 0:
                    {
                        px_new[0].x_ =cx[0] + (cx_ts[3*NP]-3*cx_ts[2*NP]+3*cx_ts[NP]-cx_ts[0])/pow(NP,3);
                        px_new[0].y_ =cy[0] + (cy_ts[3*NP]-3*cy_ts[2*NP]+3*cy_ts[NP]-cy_ts[0])/pow(NP,3);
                        px_new[0].z_ =cz[0] + (cz_ts[3*NP]-3*cz_ts[2*NP]+3*cz_ts[NP]-cz_ts[0])/pow(NP,3);
                    }break;
                    case 1: {
                        px_new[1].x_ = 3*px_new[0].x_ -2*cx[0] + (cx_ts[3*NP+1]-3*cx_ts[2*NP+1]+3*cx_ts[NP+1]-cx_ts[1])/pow(NP,3);
                        px_new[1].y_ = 3*px_new[0].y_ -2*cy[0] + (cy_ts[3*NP+1]-3*cy_ts[2*NP+1]+3*cy_ts[NP+1]-cy_ts[1])/pow(NP,3);
                        px_new[1].z_ = 3*px_new[0].z_ -2*cz[0] + (cz_ts[3*NP+1]-3*cz_ts[2*NP+1]+3*cz_ts[NP+1]-cz_ts[1])/pow(NP,3);
                    }break;
                    case 2: {
                        px_new[2].x_ = 3*px_new[1].x_- 3*px_new[0].x_+cx[0] + (cx_ts[3*NP+2]-3*cx_ts[2*NP+2]+3*cx_ts[NP+2]-cx_ts[2])/pow(NP,3);
                        px_new[2].y_ = 3*px_new[1].y_- 3*px_new[0].y_+cy[0] + (cy_ts[3*NP+2]-3*cy_ts[2*NP+2]+3*cy_ts[NP+2]-cy_ts[2])/pow(NP,3);
                        px_new[2].z_ = 3*px_new[1].z_- 3*px_new[0].z_+cz[0] + (cz_ts[3*NP+2]-3*cz_ts[2*NP+2]+3*cz_ts[NP+2]-cz_ts[2])/pow(NP,3);
                    }break;
                    default:
                        px_new[i].x_ = 3*px_new[i-1].x_-3*px_new[i-2].x_+px_new[i-3].x_ + (cx_ts[3*NP+i]-3*cx_ts[2*NP+i]+3*cx_ts[NP+i]-cx_ts[i])/pow(NP,3);
                        px_new[i].y_ = 3*px_new[i-1].y_-3*px_new[i-2].y_+px_new[i-3].y_ + (cy_ts[3*NP+i]-3*cy_ts[2*NP+i]+3*cy_ts[NP+i]-cy_ts[i])/pow(NP,3);
                        px_new[i].z_ = 3*px_new[i-1].z_-3*px_new[i-2].z_+px_new[i-3].z_ + (cz_ts[3*NP+i]-3*cz_ts[2*NP+i]+3*cz_ts[NP+i]-cz_ts[i])/pow(NP,3);
                    break;
                }
//px_new[i].print();//输出第一次xyz规划结果
                //out_xyz_buf[out_xyz_cnt]=px_new[i]; out_xyz_cnt++;//前10个不算输出xyz规划
            }
        }
        else
        {
            for(int i=0;i<NP; i++)//输出
            {
                px_new[NP+i].x_ = 3*px_new[NP+i-1].x_-3*px_new[NP+i-2].x_+px_new[NP+i-3].x_ + (cx_ts[3*NP+i]-3*cx_ts[2*NP+i]+3*cx_ts[1*NP+i]-cx_ts[i])/pow(NP,3);
                px_new[NP+i].y_ = 3*px_new[NP+i-1].y_-3*px_new[NP+i-2].y_+px_new[NP+i-3].y_ + (cy_ts[3*NP+i]-3*cy_ts[2*NP+i]+3*cy_ts[1*NP+i]-cy_ts[i])/pow(NP,3);
                px_new[NP+i].z_ = 3*px_new[NP+i-1].z_-3*px_new[NP+i-2].z_+px_new[NP+i-3].z_ + (cz_ts[3*NP+i]-3*cz_ts[2*NP+i]+3*cz_ts[1*NP+i]-cz_ts[i])/pow(NP,3);
//px_new[10+i].print();//输出xyz规划结果
                out_xyz_buf[out_xyz_cnt]=px_new[NP+i]; out_xyz_cnt++;
            }
            for(int i=0;i<NP;i++)//迭代
            {
                px_new[i].x_ = px_new[i+NP].x_;
                px_new[i].y_ = px_new[i+NP].y_;
                px_new[i].z_ = px_new[i+NP].z_;
            }
        }
        for(int i=0;i<3;i++){cx[i] = cx[i+1];cy[i] = cy[i+1];cz[i] = cz[i+1];}
        for(int i=0;i<3*NP;i++){cx_ts[i] = cx_ts[i+NP];cy_ts[i] = cy_ts[i+NP];cz_ts[i] = cz_ts[i+NP];}
//cout <<"-----------------------------out_xyz_cnt="<<out_xyz_cnt<<" out_abc_cnt="<<out_abc_cnt<<endl;
    }
    if(flag_getVqFromTouch)
    {
        vq_cnt++;
        Q3 = rtm_abc2quat(abc);
        #if 1
        dq = Q2.w_*Q3.w_ + Q2.x_*Q3.x_ + Q2.y_*Q3.y_ + Q2.z_*Q3.z_;
            if(dq < 0)
            {
                //两四元数的dot product表示两个Q之间的夹角余弦，取反后，选取最短路径
                Q3 = Q3*(-1);
            }
        #endif
       /*
        abc.print("input_abc=");
        Q0.print("Q0=");
        Q1.print("Q1=");
        Q2.print("Q2=");
        Q3.print("Q3=");
        */
        if(vq_cnt >= 3)//需要至少三个VP点才能开始规划
        {
            vq_cnt = 4;
            #if 0
            dq = Q2.w_*Q3.w_ + Q2.x_*Q3.x_ + Q2.y_*Q3.y_ + Q2.z_*Q3.z_;
            if(dq < 0)
            {
                //两四元数的dot product表示两个Q之间的夹角余弦，取反后，选取最短路径
                Q3 = Q3*(-1);
            }
            #endif
            if(j==0)//初始段
            {
                Q0=Q1;
            }
//cout << "***************************************************"<<endl;
            double s;
            for(int k=0;k<online_alg_params_.N_interp_Q;k++)
            {
 		s = k/online_alg_params_.N_interp_Q;
                Qnew[k] = rtm_Squad(Q0,Q1,Q2,Q3,s);
//Qnew[k].print("alg_out_Qnew");
                out_abc[k] = rtm_quat2abc(Qnew[k]);//
//out_abc[k].print_abc("alg_out_abc");//输出abc规划结果
                out_abc_buf[out_abc_cnt]=out_abc[k]; out_abc_cnt++;
            }
            if(status == 2)//终点处理
            {
                Q0=Q1;Q1=Q2;Q2=Q3;//覆盖迭代
cout << "***************************************************Ending abc planing:"<<endl;
		for(int k=0;k<online_alg_params_.N_interp_Q;k++)
                {
		    s = k/online_alg_params_.N_interp_Q;
                    Qnew[k] = rtm_Squad(Q0,Q1,Q2,Q3,s);
//Qnew[k].print("alg_out_Qnew");
                    out_abc[k] = rtm_quat2abc(Qnew[k]);//
                    out_abc_buf[out_abc_cnt]=out_abc[k]; out_abc_cnt++;
//out_abc[k].print_abc("alg_out_abc");//输出终点abc规划结果
                }
                sp_cnt = 0; vp_cnt = 0; vq_cnt = 0;
            }
//cout << "***************************************************out_xyz_cnt="<<out_xyz_cnt<<" out_abc_cnt="<<out_abc_cnt<<endl;
            j = j+1;
        }
        Q0=Q1;Q1=Q2;Q2=Q3;//覆盖迭代
    }
    //xyz与abc同步输出
    if(out_xyz_cnt > 0 && out_abc_cnt > 0)
    {
        int cha=0;
//cout <<"-------out_xyz_cnt="<<out_xyz_cnt<<"------------out_abc_cnt="<<out_abc_cnt<<"--------------"<<endl;
        if(out_xyz_cnt>=out_abc_cnt)
        {
            cha=out_xyz_cnt-out_abc_cnt;
            res_PointCnt = out_abc_cnt;
            for(int i=0;i< out_abc_cnt;i++)
            {
                trj_point_buf[i+online_TrjpointBufIndex].status = out_status;//可能是起点或中间点状态
                if(out_status == 0) {out_status = 1;}
                trj_point_buf[i+online_TrjpointBufIndex].x_ = out_xyz_buf[i].x_; trj_point_buf[i+online_TrjpointBufIndex].y_ = out_xyz_buf[i].y_; trj_point_buf[i+online_TrjpointBufIndex].z_ = out_xyz_buf[i].z_;
                trj_point_buf[i+online_TrjpointBufIndex].a_ = out_abc_buf[i].x_; trj_point_buf[i+online_TrjpointBufIndex].b_ = out_abc_buf[i].y_; trj_point_buf[i+online_TrjpointBufIndex].c_ = out_abc_buf[i].z_;
//cout << "alg_output "<<out_cnt<<" (" << out_xyz_buf[i].x_ <<"," << out_xyz_buf[i].y_ <<"," << out_xyz_buf[i].z_ <<"," << out_abc_buf[i].x_ <<"," << out_abc_buf[i].y_ <<"," << out_abc_buf[i].z_<<")"<<endl;
                out_cnt++;
            }
            for(int i=0;i<cha;i++)
            {
                out_xyz_buf[i]=out_xyz_buf[i+out_abc_cnt];
            }
            if(status == 2)//正好终点就是abc算法输入点的结束的情况
            {
                cout << "---------------------------->>>-----------------------------normal ending." <<endl;
                trj_point_buf[online_TrjpointBufIndex+out_abc_cnt-1].status = 2;//输出轨迹点状态标记为终点
            }
            out_xyz_cnt=cha;
            out_abc_cnt = 0;
        }
        else if(out_xyz_cnt < out_abc_cnt)
        {
            cha = out_abc_cnt-out_xyz_cnt;
            res_PointCnt = out_xyz_cnt;
            for(int i=0;i< out_xyz_cnt;i++)
            {
//cout << "alg_ending output "<<out_cnt<<" (" << out_xyz_buf[i].x_ <<"," << out_xyz_buf[i].y_ <<"," << out_xyz_buf[i].z_ <<"," << out_abc_buf[i].x_ <<"," << out_abc_buf[i].y_ <<"," << out_abc_buf[i].z_<<")"<<endl;
                trj_point_buf[i+online_TrjpointBufIndex].status = out_status;
                if(out_status == 0) {out_status = 1;}
                trj_point_buf[i+online_TrjpointBufIndex].x_ = out_xyz_buf[i].x_; trj_point_buf[i+online_TrjpointBufIndex].y_ = out_xyz_buf[i].y_; trj_point_buf[i+online_TrjpointBufIndex].z_ = out_xyz_buf[i].z_;
                trj_point_buf[i+online_TrjpointBufIndex].a_ = out_abc_buf[i].x_; trj_point_buf[i+online_TrjpointBufIndex].b_ = out_abc_buf[i].y_; trj_point_buf[i+online_TrjpointBufIndex].c_ = out_abc_buf[i].z_;
                out_cnt++;
            }
            for(int i=0;i<cha;i++)
            {
                out_abc_buf[i]=out_abc_buf[i+out_xyz_cnt];
            }
            if(status == 2)//考虑可能的提前结束的情况
            {
                cout << "early ending. padding xyz (" << out_xyz_buf[out_xyz_cnt-1].x_<<","<<out_xyz_buf[out_xyz_cnt-1].y_<<","<<out_xyz_buf[out_xyz_cnt-1].z_<<")"<<endl;
                for(int i=0;i<out_abc_cnt;i++)
                {
                    trj_point_buf[i+online_TrjpointBufIndex].status = out_status;
                    if(i == (out_abc_cnt-1)) {trj_point_buf[i+online_TrjpointBufIndex].status = 2;}
                    trj_point_buf[i+online_TrjpointBufIndex].x_ = out_xyz_buf[out_xyz_cnt-1].x_; trj_point_buf[i+online_TrjpointBufIndex].y_ = out_xyz_buf[out_xyz_cnt-1].y_; trj_point_buf[i+online_TrjpointBufIndex].z_ = out_xyz_buf[out_xyz_cnt-1].z_;
                    trj_point_buf[i+online_TrjpointBufIndex].a_ = out_abc_buf[i].x_; trj_point_buf[i+online_TrjpointBufIndex].b_ = out_abc_buf[i].y_; trj_point_buf[i+online_TrjpointBufIndex].c_ = out_abc_buf[i].z_;
                }
                res_PointCnt = out_abc_cnt;
            }
            out_abc_cnt=cha;
            out_xyz_cnt = 0;
        }
    }/*
    if(res_PointCnt!=0)
    {
        cout << "===>alg_output_PointCnt="<<res_PointCnt<<endl;
    }*/
    return res_PointCnt;
}

/**
* 函数功能:固定基坐标系关联法,将touch发送的T矩阵所在的坐标系转换到机械臂所在的工作空间坐标系
* 使用时具体参数待修改
*/
bool OnlineTrajectoryPlanner::FixedBaseCoordTransformation(Matrix44& T_touchm,Matrix44& resM)
{
    Matrix44 T_ms;
    T_ms.matrix_[0][0]=0;T_ms.matrix_[0][1]=1;T_ms.matrix_[0][2]=0;T_ms.matrix_[0][3]=0.172;
    T_ms.matrix_[1][0]=1;T_ms.matrix_[1][1]=0;T_ms.matrix_[1][2]=0;T_ms.matrix_[1][3]=0;
    T_ms.matrix_[2][0]=0;T_ms.matrix_[2][1]=0;T_ms.matrix_[2][2]=-1;T_ms.matrix_[2][3]=0.0645;
    T_ms.matrix_[3][0]=0;T_ms.matrix_[3][1]=0;T_ms.matrix_[3][2]=0;T_ms.matrix_[3][3]=1;
    resM = T_touchm.rightMultiply(T_ms);
    return true;
}

/**
*函数功能: 动态基坐标关联法计算机械臂当前末端磨钻的目标位姿在机械臂基坐标下的位姿矩阵
*参数说明:
* T_r0_R: 按下按钮瞬间,机械臂当前末端磨钻位姿在机械臂基坐标系下的位姿矩阵
* Touch_h0_v: 按下按钮瞬间,touch当前末端假想位姿在touch基坐标系下的位姿矩阵
* Touch_ht_v: 在规划过程中 touch当前末端假想位姿在touch基坐标系下的位姿矩阵
* k: touch移动距离与机械臂移动距离之间的比例系数
* resM: 计算结果
*/
bool OnlineTrajectoryPlanner::DynamicBaseCoordTransformation(Matrix44 T_r0_R, Matrix44 Touch_h0_v,  Matrix44 Touch_ht_v, double k,Matrix44& resM)
{
    Matrix44 T_v_h0;
    Matrix44 T_ht_h0;
    Matrix44 T_rt_r0;

    Matrix44 Tr2Ti;
    Tr2Ti.matrix_[0][0]=0;  Tr2Ti.matrix_[0][1]=-1; Tr2Ti.matrix_[0][2]=0;  Tr2Ti.matrix_[0][3]=0;
    Tr2Ti.matrix_[1][0]=-1; Tr2Ti.matrix_[1][1]=0;  Tr2Ti.matrix_[1][2]=0;  Tr2Ti.matrix_[1][3]=0;
    Tr2Ti.matrix_[2][0]=0;  Tr2Ti.matrix_[2][1]=0;  Tr2Ti.matrix_[2][2]=-1; Tr2Ti.matrix_[2][3]=0;
    Tr2Ti.matrix_[3][0]=0;  Tr2Ti.matrix_[3][1]=0;  Tr2Ti.matrix_[3][2]=0;  Tr2Ti.matrix_[3][3]=1;
    Touch_h0_v.rightMultiply(Tr2Ti);
    Touch_h0_v.transmatrix_inverse_matrix44(T_v_h0);
    Touch_ht_v.rightMultiply(Tr2Ti);
    T_ht_h0 = T_v_h0.rightMultiply(Touch_ht_v);

    T_rt_r0.matrix_[0][0] = T_ht_h0.matrix_[0][0];T_rt_r0.matrix_[0][1] = T_ht_h0.matrix_[0][1];T_rt_r0.matrix_[0][2] = T_ht_h0.matrix_[0][2];
    T_rt_r0.matrix_[1][0] = T_ht_h0.matrix_[1][0];T_rt_r0.matrix_[1][1] = T_ht_h0.matrix_[1][1];T_rt_r0.matrix_[1][2] = T_ht_h0.matrix_[1][2];
    T_rt_r0.matrix_[2][0] = T_ht_h0.matrix_[2][0];T_rt_r0.matrix_[2][1] = T_ht_h0.matrix_[2][1];T_rt_r0.matrix_[2][2] = T_ht_h0.matrix_[2][2];
    T_rt_r0.matrix_[0][3] = k*T_ht_h0.matrix_[0][3];
    T_rt_r0.matrix_[1][3] = k*T_ht_h0.matrix_[1][3];
    T_rt_r0.matrix_[2][3] = k*T_ht_h0.matrix_[2][3];
    T_rt_r0.matrix_[3][0]=0;T_rt_r0.matrix_[3][1]=0;T_rt_r0.matrix_[3][2]=0;T_rt_r0.matrix_[3][3]=1;

    resM = T_r0_R.rightMultiply(T_rt_r0);
    return true;
}

/**
*函数功能: T矩阵转xyz,abc
*参数说明:
*u: 输入的T矩阵
*res_xyz: 输出向量xyz
*res_abc: 输出向量abc
*/
void OnlineTrajectoryPlanner::rtm_r2xyzabc(Matrix44& u,Vector3& res_xyz, Vector3& res_abc)
{
    res_xyz.zero();
    res_abc.zero();
    res_xyz.x_ = u.matrix_[0][3];res_xyz.y_ = u.matrix_[1][3];res_xyz.z_ = u.matrix_[2][3];
    Matrix33 m33;
    m33.matrix_[0][0] = u.matrix_[0][0];m33.matrix_[0][1] = u.matrix_[0][1];m33.matrix_[0][2] = u.matrix_[0][2];
    m33.matrix_[1][0] = u.matrix_[1][0];m33.matrix_[1][1] = u.matrix_[1][1];m33.matrix_[1][2] = u.matrix_[1][2];
    m33.matrix_[2][0] = u.matrix_[2][0];m33.matrix_[2][1] = u.matrix_[2][1];m33.matrix_[2][2] = u.matrix_[2][2];
    res_abc = rtm_rpy(m33);
}

void OnlineTrajectoryPlanner::Fir_Bspline_algorithm_test(void)
{
    //720=6*120
    double trj_data[720]={
        /*
        0,0,0,0,0,0,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
            */
        0.243304977,0.023855873,0.156425301,2.7171521840162,0.033570305073327,2.02318187876396,
        0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
        0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
        0.243304977,0.023855873,0.156425301,2.71818368437483,0.0340165598633893,2.02408029092595,
        0.243304977,0.023855873,0.156425301,2.71818368437483,0.0340165598633893,2.02408029092595,
        0.243304977,0.023855873,0.156425301,2.71718318506849,0.0339825402024382,2.02409551343192,
        0.243304977,0.023855873,0.156425301,2.7171521840162,0.033570305073327,2.02318187876396,
        0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
        0.243304977,0.023855873,0.156425301,2.71720970135377,0.0319734474712608,2.02320395426382,
        0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
        0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
        0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
        0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
        0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
        0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
        0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
        0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
        0.243304977,0.023855873,0.156425301,2.7171521840162,0.033570305073327,2.02318187876396,
        0.243304977,0.023855873,0.156425301,2.7171521840162,0.033570305073327,2.02318187876396,
        0.243304977,0.023855873,0.156425301,2.7182112546414,0.0332181087098097,2.02409085556358,
        0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
        0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
        0.243304977,0.023855873,0.156425301,2.7182112546414,0.0332181087098097,2.02409085556358,
        0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
        0.243304977,0.023855873,0.156425301,2.7171521840162,0.033570305073327,2.02318187876396,
        0.243304977,0.023855873,0.156425301,2.7182112546414,0.0332181087098097,2.02409085556358,
        0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
        0.243304977,0.023855873,0.156425301,2.7182112546414,0.0332181087098097,2.02409085556358,
        0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
        0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
        0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
        0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
        0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
        0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
        0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
        0.243292526,0.023803957,0.156425301,2.71700989237573,0.0319154178657083,2.02411009179913,
        0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
        0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
        0.243292526,0.023803957,0.156425301,2.71692410251354,0.033101044356145,2.02317444377718,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71700989237573,0.0319154178657083,2.02411009179913,
        0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
        0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
        0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71692410251354,0.033101044356145,2.02317444377718,
        0.243292526,0.023803957,0.156425301,2.71692410251354,0.033101044356145,2.02317444377718,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.7170390663482,0.0323286310320417,2.0250227898036,
        0.243292526,0.023803957,0.156425301,2.71700989237573,0.0319154178657083,2.02411009179913,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
        0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
        0.243292526,0.023803957,0.156425301,2.71700989237573,0.0319154178657083,2.02411009179913,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71692410251354,0.033101044356145,2.02317444377718,
        0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
        0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
        0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
        0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
        0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
        0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
        0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
        0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
        0.243292526,0.023803957,0.156425301,2.71700989237573,0.0319154178657083,2.02411009179913,
        0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
        0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.7170390663482,0.0323286310320417,2.0250227898036,
        0.243292526,0.023803957,0.156425301,2.7170390663482,0.0323286310320417,2.0250227898036,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
        0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,

      };
    Vector3 Pdata[120];
    Vector3 Qdata[120];


    for(int i=0;i<120;i++)
    {
        Pdata[i].x_ = trj_data[i*6];
        Pdata[i].y_ = trj_data[i*6+1];
        Pdata[i].z_ = trj_data[i*6+2];
        Qdata[i].x_ = trj_data[i*6+3];
        Qdata[i].y_ = trj_data[i*6+4];
        Qdata[i].z_ = trj_data[i*6+5];
        if(i==0){traj_on_FIR_Bspline(Pdata[i],Qdata[i],0,0);}
        else if(i==119){traj_on_FIR_Bspline(Pdata[i],Qdata[i],2,0);}
        else {traj_on_FIR_Bspline(Pdata[i],Qdata[i],1,0);}
    }
}

void OnlineTrajectoryPlanner::pointer_fuzhi_test(Quaternion q_in[], int num, Quaternion q_out[])
{
    for(int i=0;i<num;i++)
    {
        q_out[i] = q_in[i];
    }
}

void OnlineTrajectoryPlanner::function_test()
{
    cout << "##############################################"<<endl;
    Quaternion q0,q1,q2,q3,q_res;
    q0.w_ = 0.1826;
    q0.x_ = 0.3651;
    q0.y_ = 0.5477;
    q0.z_ = 0.7303;

    q1.w_ = 0.3235;
    q1.x_ = 0.4313;
    q1.y_ = 0.5392;
    q1.z_ = 0.6470;

    q2.w_ = 0.3563;
    q2.x_ = 0.4454;
    q2.y_ = 0.5345;
    q2.z_ = 0.6236;

    q3.w_ = 0.3563;
    q3.x_ = 0.4454;
    q3.y_ = 0.5345;
    q3.z_ = 0.6236;

    q_res = rtm_Squad(q0, q1, q2, q3, 0.5);
    q_res.print("q_res=");

    q_res = rtm_Slerpt(q0,q1,0.5);
    q_res.print("q_res=rtm_Slerpt");
}


/*
sample_time : 0.002 #touch采样时间间隔
generate_traj_interval  : 0.001 #生成轨迹间隔
N_step_P : 5  #计算位置-取VP点的采样点间隔--->每5个采样点取一个xyz向量  范围<=5
N_step_Q : 25 #计算姿态-取VQ的采样点间隔--->每25个采样点取一个abc向量
trj_ratio: 1.0 #机械臂移动距离与touch移动距离的比例系数,    即机械臂移动距离=K*touch移动距离
online_receive_Tmatrix_buffPack_len : 30  #运控接收来自Touch T矩阵数据包的缓存包数量
*/
void OnlineTrajectoryPlanner::online_trajectory_algorithm_params_init()
{
    online_alg_params_.sample_time=0.002;
    online_alg_params_.generate_traj_interval=0.001;
    online_alg_params_.N_step_P=5;
    online_alg_params_.N_step_Q=25;
    online_alg_params_.trj_ratio=1.0;
    online_alg_params_.online_receive_Tmatrix_buffPack_len=30;
    online_alg_params_.N_interp_P=((online_alg_params_.N_step_P*online_alg_params_.sample_time)/online_alg_params_.generate_traj_interval); //NinterpP_ = ((N_step_P_*Tsample_)/GEN_TN_);
    online_alg_params_.N_interp_Q=((online_alg_params_.N_step_Q*online_alg_params_.sample_time)/online_alg_params_.generate_traj_interval); //NinterpQ_ = ((N_step_Q_*Tsample_)/GEN_TN_);
}

int OnlineTrajectoryPlanner::setOnlineTrjRatio(double data_ratio)
{
    //trajectory_planner_ptr_.traj_params_.trj_ratio_ = data_ratio;
    online_alg_params_.trj_ratio = data_ratio;
    return 0;
}
/**
 * 函数功能: 获取运控接收来自Touch T矩阵数据包的缓存包数量
*/
int  OnlineTrajectoryPlanner::get_onlineRecvTmatrixBuffPackLen()
{
    //return trajectory_planner_ptr_.traj_params_.OnlineRecvTmatrixBuffPackLen_;
    return 30;
}


double OnlineTrajectoryPlanner::get_online_trj_ratio()
{
    //return trajectory_planner_ptr_.traj_params_.trj_ratio_;
    return 1.0;
}










