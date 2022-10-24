#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include "onlineTrj_planner.h"
#include "common_file_path.h"
#include <sstream>
#include "math.h"
#include <cmath>
#include <iomanip>
#include  "basic_alg.h"


float VPp[1][6] = { {0.1, 0.2, 0.3, 0, 0, 0} 
                  };

float VPv[2][6] = { {0.0, 0.0, 0.0, 0, 0, 0},
                    {0.1, 0.2, 0.1, 0.1*PI, 0.2*PI, 0.3*PI},
                  };

//B-Spline 3 order FIR impulse response constant 
#define LAMBDA 1  
#if(LAMBDA == 1)
double hfir[6] = {0.401782, 0.242382, 0.084082, 0.004082, -0.01742, -0.01401818};
#elif(LAMBDA == 10)
double hfir[6] = {0.197604, 0.167796, 0.117796, 0.077796, 0.037796, 0.00001402};
#elif(LAMBDA == 100)
double hfir[6] = {0.1252, 0.1191, 0.1056, 0.0886, 0.0706, 0.0535};
#endif


OnlineTrajectoryPlanner::OnlineTrajectoryPlanner()
{
    config_OnlineMove_params_file_path_ = config_OnlineMove_params_file_path_ 
                                            + ALGORITHM_DIR 
                                            + "config_OnlineMove_params.yaml";
    load_OnlineMove_params_Config();
}


OnlineTrajectoryPlanner::~OnlineTrajectoryPlanner()
{

}

// int OnlineTrajectoryPlanner::sign(double x)
// {
//     if(x < 0) return -1;
//     else return 1;
// }


// double OnlineTrajectoryPlanner::roundn(double x, int precision)
// {
//     stringstream ss;
//     ss <<fixed<<setprecision(precision)<<x;
//     ss >> x;
//     return x;
// }


Matrix33 OnlineTrajectoryPlanner::online_turnEulerMatrix(double x, double y, double z)
{
    Matrix33 Rx,Ry,Rz,result_R;
    double ct,st;
    ct = cos(x);
    st = sin(x);
    Rx.eye();
    Rx.matrix_[1][1] = ct;
    Rx.matrix_[2][2] = ct;
    Rx.matrix_[1][2] = -st;
    Rx.matrix_[2][1] = st;
    Ry.eye();
    ct = cos(y);
    st = sin(y);
    Ry.matrix_[0][0] = ct;
    Ry.matrix_[2][2] = ct;
    Ry.matrix_[0][2] = st;
    Ry.matrix_[2][0] = -st;
    Rz.eye();
    ct = cos(z);
    st = sin(z);
    Rz.matrix_[0][0] = ct;
    Rz.matrix_[1][1] = ct;
    Rz.matrix_[0][1] = -st;
    Rz.matrix_[1][0] = st;
    result_R = Rz.rightMultiply(Ry);
    result_R = result_R.rightMultiply(Rx);
    return result_R;
}

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
    q.x_ = 0.5*basic_alg::POLAR(r32-r23)*sqrt(r11-r22-r33+1);
    q.y_ = 0.5*basic_alg::POLAR(r13-r31)*sqrt(r22-r11-r33+1);
    q.z_ = 0.5*basic_alg::POLAR(r21-r12)*sqrt(r33-r11-r22+1);
    return q;
}

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

// Matrix33 OnlineTrajectoryPlanner::skew(Vector3& vec3)
// {
//     Matrix33 resMat;
//     resMat.matrix_[0][0] = 0;resMat.matrix_[0][1] = -vec3.z_;resMat.matrix_[0][2] = vec3.y_;
//     resMat.matrix_[1][0] = vec3.z_;resMat.matrix_[1][1] = 0;resMat.matrix_[1][2] = -vec3.x_;
//     resMat.matrix_[2][0] = -vec3.y_;resMat.matrix_[2][1] = vec3.x_;resMat.matrix_[2][2] = 0;
//     return resMat;
// }


// double OnlineTrajectoryPlanner::quat_dot_multiply(Quaternion& q1, Quaternion& q2)
// {
//     double res;
//     res = q1.w_ * q2.w_ + q1.x_ * q2.x_ + q1.y_ * q2.y_ + q1.z_ * q2.z_;
//     return res;
// }


Quaternion OnlineTrajectoryPlanner::quatmultiply(Quaternion& q1, Quaternion& q2)
{
    Quaternion res_quat;
    res_quat.w_ = q1.w_*q2.w_ - q1.x_*q2.x_ - q1.y_*q2.y_ - q1.z_*q2.z_;
    res_quat.x_ = q1.w_*q2.x_ + q1.x_*q2.w_ + q1.y_*q2.z_ - q1.z_*q2.y_;
    res_quat.y_ = q1.w_*q2.y_ - q1.x_*q2.z_ + q1.y_*q2.w_ + q1.z_*q2.x_;
    res_quat.z_ = q1.w_*q2.z_ + q1.x_*q2.y_ - q1.y_*q2.x_ + q1.z_*q2.w_;
    return res_quat;
}

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

Vector3 OnlineTrajectoryPlanner::online_turnMatrixEuler(Matrix33& rotation_matrix)
{
    double min_value = 1e-18;
    Vector3 angle1,angle2;
    double phi_1, phi_2, psi_1, psi_2, theta_1,theta_2;
    angle1.zero();
    angle2.zero();

    if((abs(rotation_matrix.matrix_[2][0]-1) < min_value)  || (abs(rotation_matrix.matrix_[2][0]+1) < min_value))
    {
        if(abs(rotation_matrix.matrix_[2][0]+1) < min_value)
        {
            theta_1 = PI/2;
            phi_1 = 0;
            psi_1 = phi_1 + atan2(rotation_matrix.matrix_[0][1], rotation_matrix.matrix_[0][2]);
        }
        else
        {
            theta_1 = -PI/2;
            phi_1 = 0;
            psi_1 = -phi_1 + atan2(-rotation_matrix.matrix_[0][1], -rotation_matrix.matrix_[0][2]);
        }
    }
    else
    {
        theta_1 = -asin(rotation_matrix.matrix_[2][0]);
        theta_2 = PI - theta_1;
        psi_1 = atan2(rotation_matrix.matrix_[2][1]/cos(theta_1), rotation_matrix.matrix_[2][2]/cos(theta_1));
        psi_2 = atan2(rotation_matrix.matrix_[2][1]/cos(theta_2), rotation_matrix.matrix_[2][2]/cos(theta_2));
        phi_1 = atan2(rotation_matrix.matrix_[1][0]/cos(theta_1), rotation_matrix.matrix_[0][0]/cos(theta_1));
        phi_2 = atan2(rotation_matrix.matrix_[1][0]/cos(theta_2), rotation_matrix.matrix_[0][0]/cos(theta_2));
        
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
    abc = online_turnMatrixEuler(R);
    return abc;
}

Quaternion OnlineTrajectoryPlanner::rtm_abc2quat(Vector3& abc)
{
    Matrix33 mrt;
    Quaternion resq;
    mrt = online_turnEulerMatrix(abc.x_,abc.y_,abc.z_);
    resq = rtm_r2quat(mrt);
    return resq;
}


// wgoal=((inv(M))*(2*Qtemp(2:4)-alpha*(Tmin^2)/2)')'+alpha*ts; %这里vgoal就是论文里的vk+1_goal
// Vector3 OnlineTrajectoryPlanner::update_wgoal(Matrix33& M, Quaternion& qtemp, Vector3& alpha, double tTmin, double ts)
// {
//     Matrix33 invM;
//     Vector3 resV;
//     Vector3 vtempL3;//用于暂存qtemp的后三个数字

//     invM = M;
//     invM.inverse(0.0);
//     vtempL3.x_ = qtemp.x_*2 - alpha.x_*(pow(tTmin,2)/2);
//     vtempL3.y_ = qtemp.y_*2 - alpha.y_*(pow(tTmin,2)/2);
//     vtempL3.z_ = qtemp.z_*2 - alpha.z_*(pow(tTmin,2)/2);

//     resV.x_ = invM.matrix_[0][0]*vtempL3.x_ + invM.matrix_[0][1]*vtempL3.y_ + invM.matrix_[0][2]*vtempL3.z_ + alpha.x_*ts;
//     resV.y_ = invM.matrix_[1][0]*vtempL3.x_ + invM.matrix_[1][1]*vtempL3.y_ + invM.matrix_[1][2]*vtempL3.z_ + alpha.y_*ts;
//     resV.z_ = invM.matrix_[2][0]*vtempL3.x_ + invM.matrix_[2][1]*vtempL3.y_ + invM.matrix_[2][2]*vtempL3.z_ + alpha.z_*ts;
//     return resV;
// }


// Vector3 OnlineTrajectoryPlanner::rtm_abcDiff(Vector3& abc,Vector3 abc_rate)
// {
//     Vector3 resV;
//     resV.x_ = cos(abc.y_)*cos(abc.z_)*abc_rate.x_ - sin(abc.z_)*abc_rate.y_;
//     resV.y_ = cos(abc.y_)*sin(abc.z_)*abc_rate.x_ + cos(abc.z_)*abc_rate.y_;
//     resV.z_ = -sin(abc.y_)*abc_rate.x_ + abc_rate.z_;
//     return resV;
// }


Quaternion OnlineTrajectoryPlanner::rtm_Slerpt(Quaternion& q0, Quaternion& q1, double t)
{
    double Q_theta,sinv,k0,k1;
    Quaternion Qt;
    if(t<0 || t>1)// check whether t is between 0 and 1
    {
        cout << "OnlineTrajectoryPlanner::rtm_Splerpt() -> WARNING -> t should between 0 and 1"<<endl;
    }
    double dq = q0.w_*q1.w_ + q0.x_*q1.x_ + q0.y_*q1.y_ + q0.z_*q1.z_;

    if(dq<0)
    {
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
// void OnlineTrajectoryPlanner::traj_on_Squad(int Nstep, Vector3 VPp_abc[], int NVP, Quaternion Qnew[], Vector3 abc[], Quaternion Qold[])
// {
//     double t_NinterpP = online_alg_params_.N_interp_P;//2022-6-10 pm
//     Quaternion Qs[255];
//     Quaternion Q0,Q1,Q2,Q3;
//     double dq;

//     int i=0,j=0,k=0,end;
//     double s;
//     for(i=0;i<NVP;i=i+Nstep)//每Nstep个点加载一个VP点
//     {
//         //Qs(ceil(i/Nstep),:)=rtm_abc2quat(VPp(i,4:6));
//         end = ceil(i/Nstep);
//         end = end-1;//c++数组下标从0开始
//         Qs[end] = rtm_abc2quat(VPp_abc[i]);
//         if(i>=(2*Nstep+1))//需要至少三个VP点才能开始规划
//         {
//             Q0 = Qs[end-1];
//             Q1 = Qs[end];
//             dq = Q0.w_*Q1.w_ + Q0.x_*Q1.x_ + Q0.y_*Q1.y_ + Q0.z_*Q1.z_;
//             if(dq < 0)
//             {
//                 //两四元数的dot product表示两个Q之间的夹角余弦，取反后，选取最短路径
//                 Qs[end].w_ = -Q1.w_;
//                 Qs[end].x_ = -Q1.x_;
//                 Qs[end].y_ = -Q1.y_;
//                 Qs[end].z_ = -Q1.z_;
//             }
//             if(j==0)//初始段
//             {
//                 Q0 = Qs[0];  Q1 = Q0;  Q2 = Qs[2]; Q3 = Qs[3];
//             }
//             else if(j == ceil(NVP/Nstep))//最后一段
//             {
//                 Q0 = Qs[end-2]; Q1 = Qs[end-1]; Q2 = Qs[end]; Q3 = Q2;
//             }
//             else
//             {
//                 Q0 = Qs[j-1]; Q1 = Qs[j]; Q2 = Qs[j+1]; Q3 = Qs[j+2];
//             }
//             for(k=0;k<t_NinterpP;k++)
//             {
//                 s = k/t_NinterpP;
//                 Qnew[static_cast<int>(k+t_NinterpP*(j-1)+1)] = rtm_Squad(Q0,Q1,Q2,Q3,s);
//                 abc[static_cast<int>(k+t_NinterpP*(j-1)+1)] = rtm_quat2abc(Qnew[static_cast<int>(k+t_NinterpP*(j-1)+1)]);//
//             }
//             j = j+1;
//         }
//     }
//     Qold = Qs;
// }


int OnlineTrajectoryPlanner::traj_on_FIR_Bspline(Point xyz, Euler abc,int status, int online_TrjpointBufIndex)
{
    // test only, turn input variables into fitted type
    // Point xyz;
    // Euler abc;
    // xyz.x_ = xyz_o.x_;
    // xyz.y_ = xyz_o.y_;
    // xyz.z_ = xyz_o.z_;
    // abc.a_ = abc_o.x_;
    // abc.b_ = abc_o.y_;
    // abc.c_ = abc_o.z_;


    // B-Spline step number, larger number brings more accurate result, but delay may increase
    // suggest range 3~5
    int m = 5;
    // sampling count
    static int sp_cnt = 0;
    // via point count
    static int vp_cnt = 0;
    // via q point count
    static int vq_cnt = 0;
    // factor to replace division for further process
    static double inv_interpQ = 1 / online_alg_params_.N_interp_Q;
    // status of getting via points from touch device
    bool flag_getVpFromTouch = false;
    bool flag_getVqFromTouch = false;
    // algorithm related, for more detail please see documents
    static Point vp[14];

    Point c_xyz[7];
    Point p_fil[43];

    // xyz planning output for every process
    Point offset, vp_off[14], remainder[4], p_new[10];

    // hfir indexes
    const int hfir_idx[11] = {5,4,3,2,1,0,1,2,3,4,5};
    int hfir_idxidx = 0;

    static Quaternion Q0,Q1,Q2,Q3;
    double dq;
    static int j = 0;

    static Quaternion Qnew[50];
    // abc planning output for every process
    static Euler out_abc[50];

    static Point out_xyz_buf[255];
    static Euler out_abc_buf[255];
    static int out_xyz_cnt = 0;
    static int out_abc_cnt = 0;
    static int out_cnt = 0;
    // by default, set status as begin
    static int out_status = 0;
    // has been static_cast to int when init().
    int NP = static_cast<int>(online_alg_params_.N_interp_P);
    int Pre_seg = ceil((3 * NP - 3) / NP);

    Point v3_zero;
    v3_zero.zero();
    int res_PointCnt = 0;

    // initialization checking
    // get a via point(VP) from every 5 touch matrixes, N_step_P is 5 here, as demanded in documents
    if(sp_cnt % online_alg_params_.N_step_P == 0) 
    {
        flag_getVpFromTouch = true;
    } else {
        flag_getVpFromTouch = false;
    }

    // get a via point(VP) from every 25 touch matrixes, N_step_Q is 25 here as demanded in documents
    if(sp_cnt % online_alg_params_.N_step_Q == 0) 
    {
        flag_getVqFromTouch = true;
    } else {
        flag_getVqFromTouch = false;
    }

    // status: 0 meaning begin point, 1 means moving point, 2 means end point
    // if begin, initialize variables
    if(status == 0)
    {
        sp_cnt = 0;
        vp_cnt = 0;
        vq_cnt = 0;
        j = 0;

        for(int i = 0; i < (2 * m + Pre_seg + 1); ++i)
        {
            vp[i] = xyz;
        }
        flag_getVpFromTouch = true;
        flag_getVqFromTouch = true;
        
        Q0.zero();
        Q1.zero();
        Q2.zero();
        Q3.zero();

        memset(Qnew, 0, 50 * sizeof(Quaternion));
        memset(out_abc, 0, 50 * sizeof(Euler));
        memset(out_xyz_buf, 0, 255 * sizeof(Point));
        memset(out_abc_buf, 0, 255 * sizeof(Euler));

        out_xyz_cnt = 0;
        out_abc_cnt = 0;
        out_cnt = 0;
        out_status = 0;

    } else if(status == 2) // if end
    {
        flag_getVpFromTouch = true;
        flag_getVqFromTouch = true;
    }

    // cleaning vp_cnt and vq_cnt by common factor
    if(sp_cnt >= 2 * online_alg_params_.N_step_P * online_alg_params_.N_step_Q)
    {
        sp_cnt = online_alg_params_.N_step_P * online_alg_params_.N_step_Q;
    }
    // sp_cnt increment
    ++sp_cnt;


    // Process VP
    if(flag_getVpFromTouch)
    {
        vp[2 * m + Pre_seg] = xyz;
        // if the receving point is the beginning
        if(vp_cnt == 0)
        {
            for(int i = 0; i < (2 * m + Pre_seg); ++i)
            {
                vp[i] = vp[2 * m + Pre_seg];
            }
        }
        // get current via_point offset value
        offset = vp[0];
        for(int i = 0; i < (2 * m + Pre_seg + 1); ++i)
        {
            vp_off[i] = vp[i] - offset;
        }
        // via_point count increment
        if(vp_cnt < (2 * m + 1))
        {
            ++vp_cnt;
        }
        // hfir index depend on m
        hfir_idxidx = 5 - m;
        // initialize c_xyz
        for(int i = 0; i < 7; ++i)
        {
            c_xyz[i] = v3_zero;
        }

        for(int i = 0; i < (2 * m + 1); ++i)
        {
            c_xyz[3] += vp_off[i] * hfir[hfir_idx[hfir_idxidx]];
            c_xyz[4] += vp_off[i+1] * hfir[hfir_idx[hfir_idxidx]];
            c_xyz[5] += vp_off[i+2] * hfir[hfir_idx[hfir_idxidx]];
            c_xyz[6] += vp_off[i+3] * hfir[hfir_idx[hfir_idxidx]];
            ++hfir_idxidx;
        }

        for(int i = 0; i < 4; ++i)
        {
            double temp = 0.001; 
            remainder[i] = (c_xyz[i + 3] 
                            - (c_xyz[i + 2] * 3) 
                            + (c_xyz[i + 1] * 3) 
                            - c_xyz[i]) 
                            * temp;
        }


        p_fil[0] = p_fil[1] = p_fil[2] = v3_zero;
        int remainder_idx = 0;
        for(int i = 0; i < 40; ++i)
        {
            remainder_idx = i/10;
            p_fil[i+3] = (p_fil[i+2] * 3) - (p_fil[i+1] * 3) + p_fil[i] + remainder[remainder_idx];
        }
        // via_point count update
        if(vp_cnt != 1)
        {
            for(int i = 0; i < NP; ++i)
            {
                p_new[i] = p_fil[i + 33] + offset;
                out_xyz_buf[out_xyz_cnt] = p_new[i]; 
                ++out_xyz_cnt;
            }
        }
        // via_point iteration
        for(int i = 0; i < (2 * m + Pre_seg); ++i)
        {
            vp[i] = vp[i + 1];
        }

    }

    // Process VQ
    if(flag_getVqFromTouch)
    {
        ++vq_cnt;
        abc.convertToQuaternion(Q3);

        #if 1
        dq = Q2.w_ * Q3.w_ 
            + Q2.x_ * Q3.x_ 
            + Q2.y_ * Q3.y_ 
            + Q2.z_ * Q3.z_;

            if(dq < 0)
                {
                    Q3 = Q3 * (-1);
                }
        #endif

        // requires at least 3 via points for planning
        if(vq_cnt >= 3)
        {
            vq_cnt = 4;

            #if 0
            dq = Q2.w_*Q3.w_ + Q2.x_*Q3.x_ + Q2.y_*Q3.y_ + Q2.z_*Q3.z_;
            if(dq < 0)
            {
                Q3 = Q3*(-1);
            }
            #endif

            if(j == 0)
            {
                Q0 = Q1;
            }

            double s;
            for(int k = 0; k < online_alg_params_.N_interp_Q; ++k)
            {
                s = k * inv_interpQ;
                Qnew[k] = rtm_Squad(Q0,Q1,Q2,Q3,s);
                Qnew[k].convertToEuler(out_abc[k]);
                out_abc_buf[out_abc_cnt] = out_abc[k]; 
                ++out_abc_cnt;
            }

            if(status == 2)
            {
                Q0 = Q1;
                Q1 = Q2;
                Q2 = Q3;

                for(int k = 0; k < online_alg_params_.N_interp_Q; ++k)
                {
                    s = k * inv_interpQ;
                    Qnew[k] = rtm_Squad(Q0,Q1,Q2,Q3,s);
                    Qnew[k].convertToEuler(out_abc[k]);
                    out_abc_buf[out_abc_cnt] = out_abc[k];
                    ++out_abc_cnt;
                }

                sp_cnt = 0;
                vp_cnt = 0;
                vq_cnt = 0;
            }

            ++j;
        }

        Q0 = Q1;
        Q1 = Q2;
        Q2 = Q3;
    }

    // Synchronize and Update xyz and abc output
    if(out_xyz_cnt > 0 && out_abc_cnt > 0)
    {
        int cha = 0;
        if(out_xyz_cnt >= out_abc_cnt)
        {
            cha = out_xyz_cnt - out_abc_cnt;
            res_PointCnt = out_abc_cnt;
            for(int i = 0; i < out_abc_cnt; ++i)
            {
                trj_point_buf[i + online_TrjpointBufIndex].status = out_status;
                if(out_status == 0) 
                {
                    out_status = 1;
                }
                trj_point_buf[i + online_TrjpointBufIndex].x_ = out_xyz_buf[i].x_; 
                trj_point_buf[i + online_TrjpointBufIndex].y_ = out_xyz_buf[i].y_; 
                trj_point_buf[i + online_TrjpointBufIndex].z_ = out_xyz_buf[i].z_;
                trj_point_buf[i + online_TrjpointBufIndex].a_ = out_abc_buf[i].a_; 
                trj_point_buf[i + online_TrjpointBufIndex].b_ = out_abc_buf[i].b_; 
                trj_point_buf[i + online_TrjpointBufIndex].c_ = out_abc_buf[i].c_;
                ++out_cnt;
            }

            for(int i = 0; i < cha; ++i)
            {
                out_xyz_buf[i] = out_xyz_buf[i + out_abc_cnt];
            }

            if(status == 2)
            {
                cout << "---------------------------->>>-----------------------------normal ending." <<endl;
                trj_point_buf[online_TrjpointBufIndex + out_abc_cnt - 1].status = 2;
            }
            out_xyz_cnt = cha;
            out_abc_cnt = 0;
        } else if(out_xyz_cnt < out_abc_cnt) {

            cha = out_abc_cnt - out_xyz_cnt;
            res_PointCnt = out_xyz_cnt;

            for(int i = 0; i < out_xyz_cnt; ++i)
            {
                trj_point_buf[i + online_TrjpointBufIndex].status = out_status;
                if(out_status == 0) 
                {
                    out_status = 1;
                }
                trj_point_buf[i + online_TrjpointBufIndex].x_ = out_xyz_buf[i].x_; 
                trj_point_buf[i + online_TrjpointBufIndex].y_ = out_xyz_buf[i].y_; 
                trj_point_buf[i + online_TrjpointBufIndex].z_ = out_xyz_buf[i].z_;
                trj_point_buf[i + online_TrjpointBufIndex].a_ = out_abc_buf[i].a_; 
                trj_point_buf[i + online_TrjpointBufIndex].b_ = out_abc_buf[i].b_; 
                trj_point_buf[i + online_TrjpointBufIndex].c_ = out_abc_buf[i].c_;
                ++out_cnt;
            }

            for(int i = 0; i < cha; ++i)
            {
                out_abc_buf[i] = out_abc_buf[i + out_xyz_cnt];
            }

            if(status == 2)
            {
                // print early ending
                cout << "early ending. padding xyz (" << out_xyz_buf[out_xyz_cnt - 1].x_
                        <<","<<out_xyz_buf[out_xyz_cnt - 1].y_
                        <<","<<out_xyz_buf[out_xyz_cnt - 1].z_
                        <<")"<<endl;

                for(int i = 0; i < out_abc_cnt; ++i)
                {
                    trj_point_buf[i + online_TrjpointBufIndex].status = out_status;
                    if(i == (out_abc_cnt - 1)) 
                    {
                        trj_point_buf[i + online_TrjpointBufIndex].status = 2;
                    }
                    trj_point_buf[i + online_TrjpointBufIndex].x_ = out_xyz_buf[out_xyz_cnt - 1].x_; 
                    trj_point_buf[i + online_TrjpointBufIndex].y_ = out_xyz_buf[out_xyz_cnt - 1].y_; 
                    trj_point_buf[i + online_TrjpointBufIndex].z_ = out_xyz_buf[out_xyz_cnt - 1].z_;
                    trj_point_buf[i + online_TrjpointBufIndex].a_ = out_abc_buf[i].a_; 
                    trj_point_buf[i + online_TrjpointBufIndex].b_ = out_abc_buf[i].b_; 
                    trj_point_buf[i + online_TrjpointBufIndex].c_ = out_abc_buf[i].c_;
                }
                res_PointCnt = out_abc_cnt;
            }
            out_abc_cnt = cha;
            out_xyz_cnt = 0;
        }
    }
    
    return res_PointCnt;

}





/********************
 * 函数功能:
 * 参数说明:
 * 一个采样点数据 xyz, abc
 * status: 状态 0-起点(按下按钮)  1-中间过程(hold按钮) 2-终点(松开按钮)
 * online_TrjpointBufIndex--填入算法输出结果trj_point_buf[]的起点下标
 * 返回值: 生成结果途经点的数量
 * *************************/
// int OnlineTrajectoryPlanner::traj_on_FIR_Bspline(Vector3 xyz, Vector3 abc,int status, int online_TrjpointBufIndex)
// {
//     // B样条近似阶数,越大越接近实际值，但是滤波器延迟也会变大(m最大取值为5,可选3或4)
//     int m = 5;
//     // 采样点计数
//     static int sp_cnt = 0;
//     // vp点计数
//     static int vp_cnt = 0;
//     // vq点计数
//     static int vq_cnt = 0;

//     bool flag_getVpFromTouch = false;
//     bool flag_getVqFromTouch = false;
//     // 前13个逐个迭代
//     static Vector3 vp[14];

//     Vector3 c_xyz[7];
//     Vector3 p_fil[43];

//     // 单次xyz规划输出
//     Vector3 offset, vp_off[14], remainder[4], p_new[10];

//     // hfir下标
//     const int hfir_idx[11] = {5,4,3,2,1,0,1,2,3,4,5};
//     int hfir_idxidx = 0;

//     static Quaternion Q0,Q1,Q2,Q3;
//     double dq;
//     static int j = 0;

//     static Quaternion Qnew[50];
//     // 单次abc规划输出
//     static Vector3 out_abc[50];

//     static Vector3 out_xyz_buf[255];
//     static Vector3 out_abc_buf[255];
//     static int out_xyz_cnt=0;
//     static int out_abc_cnt=0;
//     static int out_cnt=0;
//     // 默认输出轨迹点状态为起点
//     static int out_status=0;
//     // has been static_cast to int when init().
//     int NP = static_cast<int>(online_alg_params_.N_interp_P);
//     int Pre_seg = ceil((3*NP-3)/NP);

//     Vector3 v3_zero;
//     v3_zero.zero();
//     int res_PointCnt = 0;

//     //每NstepP记录一个VP NstepP==5
//     //cout << "sp_cnt="<<sp_cnt<<endl;
//     if(sp_cnt % online_alg_params_.N_step_P == 0) 
//     {
//         flag_getVpFromTouch = true;
//     } else {
//         flag_getVpFromTouch = false;
//     }

//     // 每NstepQ记录一个VQ NstepQ==25
//     // if(sp_cnt%NstepQ == 0) {flag_getVqFromTouch = true;} else {flag_getVqFromTouch = false;}
//     if(sp_cnt%online_alg_params_.N_step_Q == 0) 
//     {
//         flag_getVqFromTouch = true;
//     } else {
//         flag_getVqFromTouch = false;
//     }

//     // 轨迹起点--按下按键后立即传入静止状态的机械臂末端位姿
//     if(status == 0)
//     {
//         sp_cnt = 0; 
//         vp_cnt = 0; 
//         vq_cnt = 0;
//         j = 0;

//         for(int i = 0; i < (2 * m + Pre_seg + 1); i++)
//         {
//             vp[i] = xyz;
//         }

//         flag_getVpFromTouch = true;
//         flag_getVqFromTouch = true;

//         Q0.w_=0;Q0.x_=0;Q0.y_=0;Q0.z_=0;
//         Q1.w_=0;Q1.x_=0;Q1.y_=0;Q1.z_=0;
//         Q2.w_=0;Q2.x_=0;Q2.y_=0;Q2.z_=0;
//         Q3.w_=0;Q3.x_=0;Q3.y_=0;Q3.z_=0;

//         memset(Qnew,0,50*sizeof(Quaternion));
//         memset(out_abc,0,50*sizeof(Vector3));
//         memset(out_xyz_buf,0,255*sizeof(Vector3));
//         memset(out_abc_buf,0,255*sizeof(Vector3));

//         out_xyz_cnt = 0;
//         out_abc_cnt = 0;
//         out_cnt=0;
//         out_status=0;

//         //sp_cnt++;//采样点计数自增
//     }else if(status == 2)//终点---(松开按钮时会发送一次位置点)
//     {
//         flag_getVpFromTouch = true;
//         flag_getVqFromTouch = true;
//     }


//     // 采样点计数根据取VP点和VQ点间隔的公倍数清零
//     if(sp_cnt >= 2*online_alg_params_.N_step_P*online_alg_params_.N_step_Q) 
//     {
//         sp_cnt = online_alg_params_.N_step_P*online_alg_params_.N_step_Q;
//     }
//     // 采样点计数自增
//     sp_cnt++;


//     if(flag_getVpFromTouch)
//     {
//         vp[2 * m + Pre_seg] = xyz;

//         if(vp_cnt == 0)
//         {
//             for(int i = 0; i < (2 * m + Pre_seg); i++)
//             {
//                 vp[i] = vp[2*m+Pre_seg];
//             }
//         }

//         offset = vp[0];

//         for(int i = 0; i < (2 * m + Pre_seg + 1); i++)
//         {
//             vp_off[i] = vp[i] - offset;
//         }

//         if(vp_cnt < (2 * m + 1)) 
//         {
//             vp_cnt++;
//         }

//         // 确定hfir_idx的查找起始下标
//         hfir_idxidx = 5-m;

//         //初始化c_xyz[]前3个为零向量
//         c_xyz[0] = c_xyz[1] = c_xyz[2] = v3_zero;
//         c_xyz[3] = c_xyz[4] = c_xyz[5] = c_xyz[6] = v3_zero;

//         for(int i = 0; i < (2 * m + 1); i++)
//         {
//             c_xyz[3].x_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i].x_;
//             c_xyz[3].y_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i].y_;
//             c_xyz[3].z_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i].z_;
//             c_xyz[4].x_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i+1].x_;
//             c_xyz[4].y_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i+1].y_;
//             c_xyz[4].z_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i+1].z_;
//             c_xyz[5].x_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i+2].x_;
//             c_xyz[5].y_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i+2].y_;
//             c_xyz[5].z_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i+2].z_;
//             c_xyz[6].x_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i+3].x_;
//             c_xyz[6].y_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i+3].y_;
//             c_xyz[6].z_ += hfir[hfir_idx[hfir_idxidx]] * vp_off[i+3].z_;
//             hfir_idxidx++;
//         }

//         // 先计算好滤波公式后半部分余项
//         for(int i = 0; i < 4; i++)
//         {
//             remainder[i].x_ = (c_xyz[i+3].x_ - 3*c_xyz[i+2].x_ + 3*c_xyz[i+1].x_ - c_xyz[i].x_)/1000;
//             remainder[i].y_ = (c_xyz[i+3].y_ - 3*c_xyz[i+2].y_ + 3*c_xyz[i+1].y_ - c_xyz[i].y_)/1000;
//             remainder[i].z_ = (c_xyz[i+3].z_ - 3*c_xyz[i+2].z_ + 3*c_xyz[i+1].z_ - c_xyz[i].z_)/1000;
//         }

//         // 滤波函数
//         // 初始化p_fil[]前3个为零向量
//         p_fil[0] = p_fil[1] = p_fil[2] = v3_zero;
//         int remainder_idx = 0;

//         for(int i=0; i<40;i++)
//         {
//             remainder_idx = round(i/10);
//             p_fil[i+3].x_ = 3*p_fil[i+2].x_ - 3*p_fil[i+1].x_ + p_fil[i].x_ + remainder[remainder_idx].x_;
//             p_fil[i+3].y_ = 3*p_fil[i+2].y_ - 3*p_fil[i+1].y_ + p_fil[i].y_ + remainder[remainder_idx].y_;
//             p_fil[i+3].z_ = 3*p_fil[i+2].z_ - 3*p_fil[i+1].z_ + p_fil[i].z_ + remainder[remainder_idx].z_;
//         }

//         if(vp_cnt != 1)
//         {
//             // 输出
//             for(int i = 0; i < NP; i++)
//             {
//                 // px_new[10+i].print(); // 输出xyz规划结果
//                 p_new[i] = p_fil[i+33] + offset;
//                 out_xyz_buf[out_xyz_cnt] = p_new[i]; 
//                 out_xyz_cnt++;
//             }
//         }

//         for(int i = 0; i < (2 * m + Pre_seg); i++)
//         {
//             vp[i] = vp[i+1];
//         }
//         //cout <<"-----------------------------out_xyz_cnt="<<out_xyz_cnt<<" out_abc_cnt="<<out_abc_cnt<<endl;
//     }

    
//     if(flag_getVqFromTouch)
//     {
//         vq_cnt++;
//         Q3 = rtm_abc2quat(abc);

//         #if 1
//         dq = Q2.w_*Q3.w_ 
//             + Q2.x_*Q3.x_ 
//             + Q2.y_*Q3.y_ 
//             + Q2.z_*Q3.z_;

//             if(dq < 0)
//             {
//                 //两四元数的dot product表示两个Q之间的夹角余弦，取反后，选取最短路径
//                 Q3 = Q3*(-1);
//             }
//         #endif

//         /*
//         abc.print("input_abc=");
//         Q0.print("Q0=");
//         Q1.print("Q1=");
//         Q2.print("Q2=");
//         Q3.print("Q3=");
//         */

//         // 需要至少三个VP点才能开始规划
//         if(vq_cnt >= 3)
//         {
//             vq_cnt = 4;

//             #if 0
//             dq = Q2.w_*Q3.w_ + Q2.x_*Q3.x_ + Q2.y_*Q3.y_ + Q2.z_*Q3.z_;
//             if(dq < 0)
//             {
//                 //两四元数的dot product表示两个Q之间的夹角余弦，取反后，选取最短路径
//                 Q3 = Q3*(-1);
//             }
//             #endif
//             // 初始段
//             if(j == 0)
//             {
//                 Q0 = Q1;
//             }

//             //cout << "***************************************************"<<endl;

//             double s;
//             for(int k=0;k<online_alg_params_.N_interp_Q;k++)
//             {
//  		        s = k / online_alg_params_.N_interp_Q;
//                 Qnew[k] = rtm_Squad(Q0,Q1,Q2,Q3,s);
//                 //Qnew[k].print("alg_out_Qnew");
//                 out_abc[k] = rtm_quat2abc(Qnew[k]);
//                 // 输出abc规划结果
//                 //out_abc[k].print_abc("alg_out_abc");
//                 out_abc_buf[out_abc_cnt]=out_abc[k]; 

//                 out_abc_cnt++;
//             }

//             //终点处理
//             if(status == 2)
//             {
//                 //覆盖迭代
//                 Q0=Q1;
//                 Q1=Q2;
//                 Q2=Q3;
//                 cout << "***************************************************Ending abc planing:"<<endl;
		        
//                 for(int k=0;k<online_alg_params_.N_interp_Q;k++)
//                 {
// 		            s = k/online_alg_params_.N_interp_Q;
//                     Qnew[k] = rtm_Squad(Q0,Q1,Q2,Q3,s);
//                     //Qnew[k].print("alg_out_Qnew");
//                     out_abc[k] = rtm_quat2abc(Qnew[k]);
//                     out_abc_buf[out_abc_cnt] = out_abc[k]; 
//                     out_abc_cnt++;
//                     //out_abc[k].print_abc("alg_out_abc");//输出终点abc规划结果
//                 }

//                 sp_cnt = 0; 
//                 vp_cnt = 0; 
//                 vq_cnt = 0;

//             }

//             //cout << "***************************************************out_xyz_cnt="<<out_xyz_cnt<<" out_abc_cnt="<<out_abc_cnt<<endl;
//             j = j+1;

//         }
//         // 覆盖迭代
//         Q0=Q1;
//         Q1=Q2;
//         Q2=Q3;
//     }


//     // xyz与abc同步输出
//     if(out_xyz_cnt > 0 && out_abc_cnt > 0)
//     {
//         int cha = 0;
//         //cout <<"-------out_xyz_cnt="<<out_xyz_cnt<<"------------out_abc_cnt="<<out_abc_cnt<<"--------------"<<endl;
//         if(out_xyz_cnt >= out_abc_cnt)
//         {
//             cha = out_xyz_cnt - out_abc_cnt;
//             res_PointCnt = out_abc_cnt;
//             for(int i = 0; i < out_abc_cnt; i++)
//             {
//                 // 可能是起点或中间点状态
//                 trj_point_buf[i+online_TrjpointBufIndex].status = out_status;
//                 if(out_status == 0) 
//                 {
//                     out_status = 1;
//                 }
//                 trj_point_buf[i+online_TrjpointBufIndex].x_ = out_xyz_buf[i].x_; trj_point_buf[i+online_TrjpointBufIndex].y_ = out_xyz_buf[i].y_; trj_point_buf[i+online_TrjpointBufIndex].z_ = out_xyz_buf[i].z_;
//                 trj_point_buf[i+online_TrjpointBufIndex].a_ = out_abc_buf[i].x_; trj_point_buf[i+online_TrjpointBufIndex].b_ = out_abc_buf[i].y_; trj_point_buf[i+online_TrjpointBufIndex].c_ = out_abc_buf[i].z_;
//                 //cout << "alg_output "<<out_cnt<<" (" << out_xyz_buf[i].x_ <<"," << out_xyz_buf[i].y_ <<"," << out_xyz_buf[i].z_ <<"," << out_abc_buf[i].x_ <<"," << out_abc_buf[i].y_ <<"," << out_abc_buf[i].z_<<")"<<endl;
//                 out_cnt++;
//             }
//             for(int i = 0; i < cha; i++)
//             {
//                 out_xyz_buf[i] = out_xyz_buf[i+out_abc_cnt];
//             }
//             // 正好终点就是abc算法输入点的结束的情况
//             if(status == 2)
//             {
//                 cout << "---------------------------->>>-----------------------------normal ending." <<endl;
//                 // 输出轨迹点状态标记为终点
//                 trj_point_buf[online_TrjpointBufIndex+out_abc_cnt-1].status = 2;
//             }
//             out_xyz_cnt=cha;
//             out_abc_cnt = 0;
//         } else if(out_xyz_cnt < out_abc_cnt) {

//             cha = out_abc_cnt-out_xyz_cnt;
//             res_PointCnt = out_xyz_cnt;

//             for(int i = 0; i < out_xyz_cnt; i++)
//             {
//                 //cout << "alg_ending output "<<out_cnt<<" (" << out_xyz_buf[i].x_ <<"," << out_xyz_buf[i].y_ <<"," << out_xyz_buf[i].z_ <<"," << out_abc_buf[i].x_ <<"," << out_abc_buf[i].y_ <<"," << out_abc_buf[i].z_<<")"<<endl;
//                 trj_point_buf[i+online_TrjpointBufIndex].status = out_status;
//                 if(out_status == 0) 
//                 {
//                     out_status = 1;
//                 }
//                 trj_point_buf[i+online_TrjpointBufIndex].x_ = out_xyz_buf[i].x_; trj_point_buf[i+online_TrjpointBufIndex].y_ = out_xyz_buf[i].y_; trj_point_buf[i+online_TrjpointBufIndex].z_ = out_xyz_buf[i].z_;
//                 trj_point_buf[i+online_TrjpointBufIndex].a_ = out_abc_buf[i].x_; trj_point_buf[i+online_TrjpointBufIndex].b_ = out_abc_buf[i].y_; trj_point_buf[i+online_TrjpointBufIndex].c_ = out_abc_buf[i].z_;
//                 out_cnt++;
//             }

//             for(int i = 0; i < cha; i++)
//             {
//                 out_abc_buf[i] = out_abc_buf[i+out_xyz_cnt];
//             }

//             // 考虑可能的提前结束的情况
//             if(status == 2)
//             {
//                 cout << "early ending. padding xyz (" << out_xyz_buf[out_xyz_cnt-1].x_<<","<<out_xyz_buf[out_xyz_cnt-1].y_<<","<<out_xyz_buf[out_xyz_cnt-1].z_<<")"<<endl;
                
//                 for(int i = 0; i < out_abc_cnt; i++)
//                 {
//                     trj_point_buf[i+online_TrjpointBufIndex].status = out_status;
//                     if(i == (out_abc_cnt-1)) 
//                     {
//                         trj_point_buf[i+online_TrjpointBufIndex].status = 2;
//                     }
//                     trj_point_buf[i+online_TrjpointBufIndex].x_ = out_xyz_buf[out_xyz_cnt-1].x_; 
//                     trj_point_buf[i+online_TrjpointBufIndex].y_ = out_xyz_buf[out_xyz_cnt-1].y_; 
//                     trj_point_buf[i+online_TrjpointBufIndex].z_ = out_xyz_buf[out_xyz_cnt-1].z_;
//                     trj_point_buf[i+online_TrjpointBufIndex].a_ = out_abc_buf[i].x_; 
//                     trj_point_buf[i+online_TrjpointBufIndex].b_ = out_abc_buf[i].y_; 
//                     trj_point_buf[i+online_TrjpointBufIndex].c_ = out_abc_buf[i].z_;
//                 }
//                 res_PointCnt = out_abc_cnt;
//             }
//             out_abc_cnt = cha;
//             out_xyz_cnt = 0;
//         }
//     }

//     /*
//     if(res_PointCnt!=0)
//     {
//         cout << "===>alg_output_PointCnt="<<res_PointCnt<<endl;
//     }*/

//     return res_PointCnt;
// }




/**
* 函数功能:固定基坐标系关联法,将touch发送的T矩阵所在的坐标系转换到机械臂所在的工作空间坐标系
* 使用时具体参数待修改
*/
// bool OnlineTrajectoryPlanner::FixedBaseCoordTransformation(Matrix44& T_touchm,Matrix44& resM)
// {
//     Matrix44 T_ms;
//     T_ms.matrix_[0][0]=0;T_ms.matrix_[0][1]=1;T_ms.matrix_[0][2]=0;T_ms.matrix_[0][3]=0.172;
//     T_ms.matrix_[1][0]=1;T_ms.matrix_[1][1]=0;T_ms.matrix_[1][2]=0;T_ms.matrix_[1][3]=0;
//     T_ms.matrix_[2][0]=0;T_ms.matrix_[2][1]=0;T_ms.matrix_[2][2]=-1;T_ms.matrix_[2][3]=0.0645;
//     T_ms.matrix_[3][0]=0;T_ms.matrix_[3][1]=0;T_ms.matrix_[3][2]=0;T_ms.matrix_[3][3]=1;
//     resM = T_touchm.rightMultiply(T_ms);
//     return true;
// }



Matrix44 OnlineTrajectoryPlanner::rtm_reorthog(Matrix44 &T)
{
    Matrix44 T_new;//result;
    Vector3 a,b,a_ort, b_ort,c_ort,a_new,b_new,c_new;
    a.x_ = T.matrix_[0][0];
    a.y_ = T.matrix_[0][1];
    a.z_ = T.matrix_[0][2];

    b.x_ = T.matrix_[1][0];
    b.y_ = T.matrix_[1][1];
    b.z_ = T.matrix_[1][2];

    double t;
    t = a.dotProduct(b)/2;
    a_ort = a - b*t;
    b_ort = b - a*t;
    a_ort.crossProduct(b_ort,c_ort);

    t = (3-a_ort.dotProduct(a_ort))/2;
    a_new = a_ort*t;

    t = (3-b_ort.dotProduct(b_ort))/2;
    b_new = b_ort*t;

    t = (3-c_ort.dotProduct(c_ort))/2;
    c_new = c_ort*t;

    T_new.matrix_[0][0] = a_new.x_;T_new.matrix_[0][1] = a_new.y_;T_new.matrix_[0][2] = a_new.z_; T_new.matrix_[0][3] = T.matrix_[0][3];
    T_new.matrix_[1][0] = b_new.x_;T_new.matrix_[1][1] = b_new.y_;T_new.matrix_[1][2] = b_new.z_; T_new.matrix_[1][3] = T.matrix_[1][3];
    T_new.matrix_[2][0] = c_new.x_;T_new.matrix_[2][1] = c_new.y_;T_new.matrix_[2][2] = c_new.z_; T_new.matrix_[2][3] = T.matrix_[2][3];
    T_new.matrix_[3][0] = T.matrix_[3][0];T_new.matrix_[3][1] =T.matrix_[3][1];T_new.matrix_[3][2] = T.matrix_[3][2]; T_new.matrix_[3][3] = T.matrix_[3][3];
    return T_new;
}



    
Matrix33 OnlineTrajectoryPlanner::rtm_reorthog(Matrix33 &T)
{
    Matrix33 T_new;//result;
    Vector3 a,b,a_ort, b_ort,c_ort,a_new,b_new,c_new;
    a.x_ = T.matrix_[0][0];
    a.y_ = T.matrix_[0][1];
    a.z_ = T.matrix_[0][2];

    b.x_ = T.matrix_[1][0];
    b.y_ = T.matrix_[1][1];
    b.z_ = T.matrix_[1][2];

    double t;
    t = a.dotProduct(b)/2;
    a_ort = a - b*t;
    b_ort = b - a*t;
    a_ort.crossProduct(b_ort,c_ort);

    t = (3-a_ort.dotProduct(a_ort))/2;
    a_new = a_ort*t;

    t = (3-b_ort.dotProduct(b_ort))/2;
    b_new = b_ort*t;

    t = (3-c_ort.dotProduct(c_ort))/2;
    c_new = c_ort*t;
    T_new.matrix_[0][0] = a_new.x_;T_new.matrix_[0][1] = a_new.y_;T_new.matrix_[0][2] = a_new.z_;
    T_new.matrix_[1][0] = b_new.x_;T_new.matrix_[1][1] = b_new.y_;T_new.matrix_[1][2] = b_new.z_;
    T_new.matrix_[2][0] = c_new.x_;T_new.matrix_[2][1] = c_new.y_;T_new.matrix_[2][2] = c_new.z_;
    return T_new;
}

RotationMatrix OnlineTrajectoryPlanner::rtm_reorthog(RotationMatrix &T)
{
    // result stores in this variable
    RotationMatrix T_new;
    Vector3 a,b,a_ort, b_ort,c_ort,a_new,b_new,c_new;
    a.x_ = T.matrix_[0][0];
    a.y_ = T.matrix_[0][1];
    a.z_ = T.matrix_[0][2];

    b.x_ = T.matrix_[1][0];
    b.y_ = T.matrix_[1][1];
    b.z_ = T.matrix_[1][2];

    double t;
    t = a.dotProduct(b)/2;
    a_ort = a - b*t;
    b_ort = b - a*t;
    a_ort.crossProduct(b_ort,c_ort);

    t = (3-a_ort.dotProduct(a_ort))/2;
    a_new = a_ort*t;

    t = (3-b_ort.dotProduct(b_ort))/2;
    b_new = b_ort*t;

    t = (3-c_ort.dotProduct(c_ort))/2;
    c_new = c_ort*t;
    T_new.matrix_[0][0] = a_new.x_;T_new.matrix_[0][1] = a_new.y_;T_new.matrix_[0][2] = a_new.z_;
    T_new.matrix_[1][0] = b_new.x_;T_new.matrix_[1][1] = b_new.y_;T_new.matrix_[1][2] = b_new.z_;
    T_new.matrix_[2][0] = c_new.x_;T_new.matrix_[2][1] = c_new.y_;T_new.matrix_[2][2] = c_new.z_;
    return T_new;
}


void OnlineTrajectoryPlanner::turnM2T(const Matrix44 ma, TransMatrix mb)
{
    // copy Rotation Matrix
    for(int i=0;i<3;++i)
    {
        for(int j=0;j<3;++j)
        {
            mb.rotation_matrix_.matrix_[i][j] = ma.matrix_[i][j];
        }
    }
    // copy xyz vector
    mb.trans_vector_.x_ = ma.matrix_[0][3];
    mb.trans_vector_.y_ = ma.matrix_[1][3];
    mb.trans_vector_.z_ = ma.matrix_[2][3];

}

void OnlineTrajectoryPlanner::turnT2M(const TransMatrix ma, Matrix44 mb)
{
    // clean mb structure
    mb.setZero();
    
    for(int i=0;i<3;++i)
    {
        for(int j=0;j<3;++j)
        {
            mb.matrix_[i][j] = ma.rotation_matrix_.matrix_[i][j];
        }
    }
    mb.matrix_[0][3] = ma.trans_vector_.x_;
    mb.matrix_[1][3] = ma.trans_vector_.y_;
    mb.matrix_[2][3] = ma.trans_vector_.z_;
    mb.matrix_[3][3] = 1;

}


bool OnlineTrajectoryPlanner::DynamicBaseCoordTransformation(TransMatrix T_r0_R, TransMatrix Touch_h0_v,  TransMatrix Touch_ht_v, double k_xyz,double k_abc, TransMatrix& resM)
{
    TransMatrix T_v_h0;
    TransMatrix T_ht_h0;
    TransMatrix T_rt_r0;

    TransMatrix Tr2Ti;
    Tr2Ti.trans_vector_.zero();
    for(int i = 0; i < 3; ++i)
    { 
        for(int j = 0; j < 3; ++j)
        {
            Tr2Ti.rotation_matrix_.matrix_[i][j] = 0;
        }
    }
    Tr2Ti.rotation_matrix_.matrix_[0][1] = -1;
    Tr2Ti.rotation_matrix_.matrix_[1][0] = -1;
    Tr2Ti.rotation_matrix_.matrix_[2][2] = -1;
  
    Touch_h0_v.rightMultiply(Tr2Ti);

    online_getMatrixInv(Touch_h0_v,T_v_h0);

    Touch_ht_v.rightMultiply(Tr2Ti);
    T_ht_h0 = T_v_h0.rightMultiply(Touch_ht_v);
    T_rt_r0.rotation_matrix_ = T_ht_h0.rotation_matrix_;
    T_rt_r0.trans_vector_ = T_ht_h0.trans_vector_ * k_xyz;


    RotationMatrix dr_temp, M33_dRr;
    Quaternion Qtemp,Qzero;

    Qzero.zero();

    Qzero.w_ = 1;

    dr_temp = T_rt_r0.rotation_matrix_;

    dr_temp = rtm_reorthog(dr_temp);

    dr_temp.convertToQuaternion(Qtemp);

    Qtemp = rtm_Slerpt(Qzero, Qtemp, k_abc);

    Qtemp.convertToRotationMatrix(M33_dRr);

    T_rt_r0.rotation_matrix_ = M33_dRr;

    resM = T_r0_R.rightMultiply(T_rt_r0);

    return true;
}





/**
*函数功能: 动态基坐标关联法计算机械臂当前末端磨钻的目标位姿在机械臂基坐标下的位姿矩阵
*参数说明:
* T_r0_R: 按下按钮瞬间,机械臂当前末端磨钻位姿在机械臂基坐标系下的位姿矩阵
* Touch_h0_v: 按下按钮瞬间,touch当前末端假想位姿在touch基坐标系下的位姿矩阵
* Touch_ht_v: 在规划过程中 touch当前末端假想位姿在touch基坐标系下的位姿矩阵
* k_xyz: touch移动距离与机械臂移动距离之间的比例系数
* k_abc: touch移动角度与机械臂移动角度之间的比例系数
* resM: 计算结果
*/
bool OnlineTrajectoryPlanner::DynamicBaseCoordTransformation(Matrix44 T_r0_R, Matrix44 Touch_h0_v,  Matrix44 Touch_ht_v, double k_xyz,double k_abc, Matrix44& resM)
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

    online_getMatrixInv(Touch_h0_v,T_v_h0);
 
    Touch_ht_v.rightMultiply(Tr2Ti);

    T_ht_h0 = T_v_h0.rightMultiply(Touch_ht_v);

    T_rt_r0.matrix_[0][0] = T_ht_h0.matrix_[0][0];
    T_rt_r0.matrix_[0][1] = T_ht_h0.matrix_[0][1];
    T_rt_r0.matrix_[0][2] = T_ht_h0.matrix_[0][2];
    T_rt_r0.matrix_[1][0] = T_ht_h0.matrix_[1][0];
    T_rt_r0.matrix_[1][1] = T_ht_h0.matrix_[1][1];
    T_rt_r0.matrix_[1][2] = T_ht_h0.matrix_[1][2];
    T_rt_r0.matrix_[2][0] = T_ht_h0.matrix_[2][0];
    T_rt_r0.matrix_[2][1] = T_ht_h0.matrix_[2][1];
    T_rt_r0.matrix_[2][2] = T_ht_h0.matrix_[2][2];
    T_rt_r0.matrix_[0][3] = k_xyz*T_ht_h0.matrix_[0][3];
    T_rt_r0.matrix_[1][3] = k_xyz*T_ht_h0.matrix_[1][3];
    T_rt_r0.matrix_[2][3] = k_xyz*T_ht_h0.matrix_[2][3];
    T_rt_r0.matrix_[3][0]=0;
    T_rt_r0.matrix_[3][1]=0;
    T_rt_r0.matrix_[3][2]=0;
    T_rt_r0.matrix_[3][3]=1;



    Matrix33 dr_temp, M33_dRr;//T_rt_r0 33
    Quaternion Qtemp,Qzero;
    Qzero.w_=1;Qzero.x_=0;Qzero.y_=0;Qzero.z_=0;
    dr_temp.matrix_[0][0] = T_rt_r0.matrix_[0][0]; 
    dr_temp.matrix_[0][1] = T_rt_r0.matrix_[0][1]; 
    dr_temp.matrix_[0][2] = T_rt_r0.matrix_[0][2];
    dr_temp.matrix_[1][0] = T_rt_r0.matrix_[1][0]; 
    dr_temp.matrix_[1][1] = T_rt_r0.matrix_[1][1]; 
    dr_temp.matrix_[1][2] = T_rt_r0.matrix_[1][2];
    dr_temp.matrix_[2][0] = T_rt_r0.matrix_[2][0]; 
    dr_temp.matrix_[2][1] = T_rt_r0.matrix_[2][1]; 
    dr_temp.matrix_[2][2] = T_rt_r0.matrix_[2][2];  
    dr_temp = rtm_reorthog(dr_temp);//正交化
    Qtemp = rtm_r2quat(dr_temp);
    Qtemp = rtm_Slerpt(Qzero,Qtemp,k_abc);
    M33_dRr = rtm_quat2r(Qtemp);

    printf("Matrix44 M33_dRr: \n");
    for(int ii = 0; ii < 3; ++ii)
    {
        for(int jj = 0; jj < 3; ++jj)
        {
            printf("%.6f\t", M33_dRr.matrix_[ii][jj]);
        }
        printf("\n");
    }

    printf("Matrix44 MT_rt_r0: \n");
    for(int ii = 0; ii < 3; ++ii)
    {
        for(int jj = 0; jj < 3; ++jj)
        {
            printf("%.6f\t", T_rt_r0.matrix_[ii][jj]);
        }
        printf("\n");
    }

    T_rt_r0.matrix_[0][0] = M33_dRr.matrix_[0][0];
    T_rt_r0.matrix_[0][1] = M33_dRr.matrix_[0][1];
    T_rt_r0.matrix_[0][2] = M33_dRr.matrix_[0][2];
    T_rt_r0.matrix_[1][0] = M33_dRr.matrix_[1][0];
    T_rt_r0.matrix_[1][1] = M33_dRr.matrix_[1][1];
    T_rt_r0.matrix_[1][2] = M33_dRr.matrix_[1][2];
    T_rt_r0.matrix_[2][0] = M33_dRr.matrix_[2][0];
    T_rt_r0.matrix_[2][1] = M33_dRr.matrix_[2][1];
    T_rt_r0.matrix_[2][2] = M33_dRr.matrix_[2][2];
    
    printf("Matrix44 T_rt_r0's after move: \n");
    for(int ii = 0; ii < 3; ++ii)
    {
        for(int jj = 0; jj < 3; ++jj)
        {
            printf("%.6f\t", T_rt_r0.matrix_[ii][jj]);
        }
        printf("\n");
    }


    resM = T_r0_R.rightMultiply(T_rt_r0);

    return true;
}


void OnlineTrajectoryPlanner::online_getMatrixInv(Matrix44 m44,Matrix44 &resT)
{

    double p_matrix[16],p_inv[16];
    p_matrix[0] = m44.matrix_[0][0];
    p_matrix[1] = m44.matrix_[0][1];
    p_matrix[2] = m44.matrix_[0][2];
    p_matrix[3] = m44.matrix_[0][3],

    p_matrix[4] = m44.matrix_[1][0],
    p_matrix[5] = m44.matrix_[1][1],
    p_matrix[6] = m44.matrix_[1][2],
    p_matrix[7] = m44.matrix_[1][3],

    p_matrix[8] = m44.matrix_[2][0],
    p_matrix[9] = m44.matrix_[2][1],
    p_matrix[10] = m44.matrix_[2][2],
    p_matrix[11] = m44.matrix_[2][3],

    p_matrix[12] = m44.matrix_[3][0],
    p_matrix[13] = m44.matrix_[3][1],
    p_matrix[14] = m44.matrix_[3][2],
    p_matrix[15] = m44.matrix_[3][3];

    basic_alg::inverse(p_matrix,4,p_inv);

    resT.matrix_[0][0] = p_inv[0];
    resT.matrix_[0][1] = p_inv[1];
    resT.matrix_[0][2] = p_inv[2];
    resT.matrix_[0][3] = p_inv[3];

    resT.matrix_[1][0] = p_inv[4];
    resT.matrix_[1][1] = p_inv[5];
    resT.matrix_[1][2] = p_inv[6];
    resT.matrix_[1][3] = p_inv[7];

    resT.matrix_[2][0] = p_inv[8];
    resT.matrix_[2][1] = p_inv[9];
    resT.matrix_[2][2] = p_inv[10];
    resT.matrix_[2][3] = p_inv[11];

    resT.matrix_[3][0] = p_inv[12];
    resT.matrix_[3][1] = p_inv[13];
    resT.matrix_[3][2] = p_inv[14];
    resT.matrix_[3][3] = p_inv[15];
}

void OnlineTrajectoryPlanner::online_getMatrixInv(TransMatrix m44,TransMatrix &resT)
{
   
    double p_matrix[16],p_inv[16];
    p_matrix[0] = m44.rotation_matrix_.matrix_[0][0];
    p_matrix[1] = m44.rotation_matrix_.matrix_[0][1];
    p_matrix[2] = m44.rotation_matrix_.matrix_[0][2];
    p_matrix[3] = m44.trans_vector_.x_;

    p_matrix[4] = m44.rotation_matrix_.matrix_[1][0];
    p_matrix[5] = m44.rotation_matrix_.matrix_[1][1];
    p_matrix[6] = m44.rotation_matrix_.matrix_[1][2];
    p_matrix[7] = m44.trans_vector_.y_;

    p_matrix[8] = m44.rotation_matrix_.matrix_[2][0];
    p_matrix[9] = m44.rotation_matrix_.matrix_[2][1];
    p_matrix[10] = m44.rotation_matrix_.matrix_[2][2];
    p_matrix[11] = m44.trans_vector_.z_;

    p_matrix[12] = 0;
    p_matrix[13] = 0;
    p_matrix[14] = 0;
    p_matrix[15] = 1;

    basic_alg::inverse(p_matrix,4,p_inv);

    resT.rotation_matrix_.matrix_[0][0] = p_inv[0];
    resT.rotation_matrix_.matrix_[0][1] = p_inv[1];
    resT.rotation_matrix_.matrix_[0][2] = p_inv[2];
    resT.trans_vector_.x_ = p_inv[3];

    resT.rotation_matrix_.matrix_[1][0] = p_inv[4];
    resT.rotation_matrix_.matrix_[1][1] = p_inv[5];
    resT.rotation_matrix_.matrix_[1][2] = p_inv[6];
    resT.trans_vector_.y_ = p_inv[7];

    resT.rotation_matrix_.matrix_[2][0] = p_inv[8];
    resT.rotation_matrix_.matrix_[2][1] = p_inv[9];
    resT.rotation_matrix_.matrix_[2][2] = p_inv[10];
    resT.trans_vector_.z_ = p_inv[11];
}


bool OnlineTrajectoryPlanner::get_increment_matrix(TransMatrix T_ck, TransMatrix T_k1, TransMatrix T_k, TransMatrix &resT)
{
    TransMatrix inv_T_k1;
    // double inv_status;
    // T_k1.inverse(inv_T_k1, inv_status);
    online_getMatrixInv(T_k1, inv_T_k1);
    T_ck.rightMultiply(inv_T_k1, resT);
    resT.rightMultiply(T_k);
    return true;
}


/*
* 函数功能: 计算获取增量后的位姿矩阵
* 参数: 
*/
bool OnlineTrajectoryPlanner::get_increment_matrix(Matrix44 T_ck,Matrix44 T_k1, Matrix44 T_k, Matrix44 &resT)
{
    Matrix44 inv_T_k1;
    //T_k1.transmatrix_inverse_matrix44(inv_T_k1);
    online_getMatrixInv(T_k1,inv_T_k1);
    //inv_T_k1.print("inv_T_k1:");

    // test - is rightmultiply the same?
    // TransMatrix T_ck_p, inv_T_k1_p, resT_p;
    // turnM2T(T_ck, T_ck_p);
    // turnM2T(inv_T_k1, inv_T_k1_p);
    // turnM2T(resT, resT_p);

    // T_ck_p.rightMultiply(inv_T_k1_p, resT_p);
    T_ck.rightMultiply(inv_T_k1,resT);

    //cout<<"right multiply check"<<endl;
    // for(int ii = 0; ii < 3; ++ii)
    //     {
    //         for(int jj = 0; jj < 3; ++jj)
    //         {
    //             if(!(fabs(resT.matrix_[ii][jj] - resT_p.rotation_matrix_.matrix_[ii][jj]) < 0.000001))
    //             {
    //                 cout<<"Ratation Matrix: the "<<ii<<"th "<<jj<<"th element of two dynamic output is different"<<endl;
    //                 cout<<"m44 & transmatrix results are: "<<resT.matrix_[ii][jj]<<"\t"<<resT_p.rotation_matrix_.matrix_[ii][jj]<<endl;
    //             }
    //         }
    //     }
    //     if(!(fabs(resT.matrix_[0][3] - resT_p.trans_vector_.x_) < 0.000001))
    //     {
    //         cout<<"XYZ Vector: the 0th 3rd element of two dynamic output is different"<<endl;
    //         cout<<"m44 & transmatrix results are : "<<resT.matrix_[0][3]<<"\t"<<resT_p.trans_vector_.x_<<endl;
    //     }
    //     if(!(fabs(resT.matrix_[1][3] - resT_p.trans_vector_.y_) < 0.000001))
    //     {
    //         cout<<"XYZ Vector: the 1th 3rd element of two dynamic output is different"<<endl;
    //         cout<<"m44 & transmatrix results are : "<<resT.matrix_[1][3]<<"\t"<<resT_p.trans_vector_.y_<<endl;
    //     }
    //     if(!(fabs(resT.matrix_[2][3] - resT_p.trans_vector_.z_) < 0.000001))
    //     {
    //         cout<<"XYZ Vector: the 2th 3rd element of two dynamic output is different"<<endl;
    //         cout<<"m44 & transmatrix results are : "<<resT.matrix_[2][3]<<"\t"<<resT_p.trans_vector_.z_<<endl;
    //     }


    //resT.print("res = T_c*inv_T_k1=");
    resT.rightMultiply(T_k);
    //resT.print("res = res*T_k=");
    return true;
}

void OnlineTrajectoryPlanner::rtm_r2xyzabc(Matrix44& u,Vector3& res_xyz, Vector3& res_abc)
{
    res_xyz.zero();
    res_abc.zero();
    res_xyz.x_ = u.matrix_[0][3];res_xyz.y_ = u.matrix_[1][3];res_xyz.z_ = u.matrix_[2][3];
    Matrix33 m33;
    m33.matrix_[0][0] = u.matrix_[0][0];m33.matrix_[0][1] = u.matrix_[0][1];m33.matrix_[0][2] = u.matrix_[0][2];
    m33.matrix_[1][0] = u.matrix_[1][0];m33.matrix_[1][1] = u.matrix_[1][1];m33.matrix_[1][2] = u.matrix_[1][2];
    m33.matrix_[2][0] = u.matrix_[2][0];m33.matrix_[2][1] = u.matrix_[2][1];m33.matrix_[2][2] = u.matrix_[2][2];
    res_abc = online_turnMatrixEuler(m33);
}

// void OnlineTrajectoryPlanner::Fir_Bspline_algorithm_test(void)
// {
//     //720=6*120
//     double trj_data[720]={
//         /*
//         0,0,0,0,0,0,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
//         1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
//             */
//         0.243304977,0.023855873,0.156425301,2.7171521840162,0.033570305073327,2.02318187876396,
//         0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
//         0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
//         0.243304977,0.023855873,0.156425301,2.71818368437483,0.0340165598633893,2.02408029092595,
//         0.243304977,0.023855873,0.156425301,2.71818368437483,0.0340165598633893,2.02408029092595,
//         0.243304977,0.023855873,0.156425301,2.71718318506849,0.0339825402024382,2.02409551343192,
//         0.243304977,0.023855873,0.156425301,2.7171521840162,0.033570305073327,2.02318187876396,
//         0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
//         0.243304977,0.023855873,0.156425301,2.71720970135377,0.0319734474712608,2.02320395426382,
//         0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
//         0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
//         0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
//         0.243304977,0.023855873,0.156425301,2.71723887803281,0.0323856608841416,2.02411754080533,
//         0.243304977,0.023855873,0.156425301,2.71718069354989,0.0327718658227053,2.02319336674902,
//         0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
//         0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
//         0.243304977,0.023855873,0.156425301,2.71721078238005,0.0331840899618763,2.02410563947698,
//         0.243304977,0.023855873,0.156425301,2.7171521840162,0.033570305073327,2.02318187876396,
//         0.243304977,0.023855873,0.156425301,2.7171521840162,0.033570305073327,2.02318187876396,
//         0.243304977,0.023855873,0.156425301,2.7182112546414,0.0332181087098097,2.02409085556358,
//         0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
//         0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
//         0.243304977,0.023855873,0.156425301,2.7182112546414,0.0332181087098097,2.02409085556358,
//         0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
//         0.243304977,0.023855873,0.156425301,2.7171521840162,0.033570305073327,2.02318187876396,
//         0.243304977,0.023855873,0.156425301,2.7182112546414,0.0332181087098097,2.02409085556358,
//         0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
//         0.243304977,0.023855873,0.156425301,2.7182112546414,0.0332181087098097,2.02409085556358,
//         0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
//         0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
//         0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
//         0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
//         0.243304977,0.023855873,0.156425301,2.71815266927753,0.0336043242601783,2.02316665580239,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
//         0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
//         0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
//         0.243292526,0.023803957,0.156425301,2.71700989237573,0.0319154178657083,2.02411009179913,
//         0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
//         0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
//         0.243292526,0.023803957,0.156425301,2.71692410251354,0.033101044356145,2.02317444377718,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71700989237573,0.0319154178657083,2.02411009179913,
//         0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
//         0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
//         0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71692410251354,0.033101044356145,2.02317444377718,
//         0.243292526,0.023803957,0.156425301,2.71692410251354,0.033101044356145,2.02317444377718,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.7170390663482,0.0323286310320417,2.0250227898036,
//         0.243292526,0.023803957,0.156425301,2.71700989237573,0.0319154178657083,2.02411009179913,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
//         0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
//         0.243292526,0.023803957,0.156425301,2.71700989237573,0.0319154178657083,2.02411009179913,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71692410251354,0.033101044356145,2.02317444377718,
//         0.243292526,0.023803957,0.156425301,2.71792407150937,0.0331350630103647,2.02315922104311,
//         0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
//         0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
//         0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
//         0.243292526,0.023803957,0.156425301,2.71795508290336,0.0335462915530697,2.02407284208891,
//         0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
//         0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
//         0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
//         0.243292526,0.023803957,0.156425301,2.71700989237573,0.0319154178657083,2.02411009179913,
//         0.243292526,0.023803957,0.156425301,2.71695261768827,0.0323016169282965,2.02318593178515,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
//         0.243292526,0.023803957,0.156425301,2.71798224763537,0.0327488534749097,2.02408340676083,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.7170390663482,0.0323286310320417,2.0250227898036,
//         0.243292526,0.023803957,0.156425301,2.7170390663482,0.0323286310320417,2.0250227898036,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,
//         0.243292526,0.023803957,0.156425301,2.71698179127107,0.0327148352534222,2.02409819046037,

//       };

//     Point Pdata[120];
//     Euler Qdata[120];
//     //Vector3 Pdata[120];
//     //Vector3 Qdata[120];
//     for(int i = 0; i < 120; ++i)
//     {
//         Pdata[i].x_ = trj_data[i*6];
//         Pdata[i].y_ = trj_data[i*6+1];
//         Pdata[i].z_ = trj_data[i*6+2];
//         Qdata[i].a_ = trj_data[i*6+3];
//         Qdata[i].b_ = trj_data[i*6+4];
//         Qdata[i].c_ = trj_data[i*6+5];
//         if(i == 0){traj_on_FIR_Bspline(Pdata[i], Qdata[i], 0, 0);}
//         else if(i == 119){traj_on_FIR_Bspline(Pdata[i], Qdata[i], 2, 0);}
//         else {traj_on_FIR_Bspline(Pdata[i], Qdata[i], 1, 0);}
//     }
// }

// void OnlineTrajectoryPlanner::pointer_fuzhi_test(Quaternion q_in[], int num, Quaternion q_out[])
// {
//     for(int i=0;i<num;i++)
//     {
//         q_out[i] = q_in[i];
//     }
// }

// void OnlineTrajectoryPlanner::function_test()
// {
//     cout << "##############################################"<<endl;
//     Quaternion q0,q1,q2,q3,q_res;
//     q0.w_ = 0.1826;
//     q0.x_ = 0.3651;
//     q0.y_ = 0.5477;
//     q0.z_ = 0.7303;

//     q1.w_ = 0.3235;
//     q1.x_ = 0.4313;
//     q1.y_ = 0.5392;
//     q1.z_ = 0.6470;

//     q2.w_ = 0.3563;
//     q2.x_ = 0.4454;
//     q2.y_ = 0.5345;
//     q2.z_ = 0.6236;

//     q3.w_ = 0.3563;
//     q3.x_ = 0.4454;
//     q3.y_ = 0.5345;
//     q3.z_ = 0.6236;

//     q_res = rtm_Squad(q0, q1, q2, q3, 0.5);
//     q_res.print("q_res=");

//     q_res = rtm_Slerpt(q0,q1,0.5);
//     q_res.print("q_res=rtm_Slerpt");
// }

bool OnlineTrajectoryPlanner::load_OnlineMove_params_Config()
{
    if (!yaml_help_.loadParamFile(config_OnlineMove_params_file_path_.c_str())
        || !yaml_help_.getParam("online_sample_time", online_alg_params_.sample_time)
        || !yaml_help_.getParam("online_generate_traj_interval", online_alg_params_.generate_traj_interval)
        || !yaml_help_.getParam("online_N_step_P", online_alg_params_.N_step_P)
        || !yaml_help_.getParam("online_N_step_Q", online_alg_params_.N_step_Q)
        || !yaml_help_.getParam("online_N_interp_P", online_alg_params_.N_interp_P)
        || !yaml_help_.getParam("online_N_interp_Q", online_alg_params_.N_interp_Q)
        || !yaml_help_.getParam("online_trj_ratio_xyz", online_alg_params_.trj_ratio_xyz)
        || !yaml_help_.getParam("online_trj_ratio_abc", online_alg_params_.trj_ratio_abc)
        || !yaml_help_.getParam("online_receive_Tmatrix_buff_len", online_alg_params_.online_receive_Tmatrix_buff_len))
    {
        std::cout << " Failed load config_OnlineMove_params.yaml " << std::endl;
        return false;
    }
    printf("\nload_OnlineMove_params_Config:\nsample_time=%lf,generate_interval=%lf,N_step_P=%d,N_step_Q=%d,N_interpP=%lf,N_interpQ=%lf,trj_ratio_xyz=%lf,trj_ratio_abc=%lf,recvTmatrix_buffLen=%d\n",
            online_alg_params_.sample_time,
            online_alg_params_.generate_traj_interval,
            online_alg_params_.N_step_P,
            online_alg_params_.N_step_Q,
            online_alg_params_.N_interp_P,
            online_alg_params_.N_interp_Q,
            online_alg_params_.trj_ratio_xyz,
            online_alg_params_.trj_ratio_abc,
            online_alg_params_.online_receive_Tmatrix_buff_len);
    return true;
}
/*
sample_time : 0.002 #touch采样时间间隔
generate_traj_interval  : 0.001 #生成轨迹间隔
N_step_P : 5  #计算位置-取VP点的采样点间隔--->每5个采样点取一个xyz向量  范围<=5
N_step_Q : 25 #计算姿态-取VQ的采样点间隔--->每25个采样点取一个abc向量
trj_ratio: 1.0 #机械臂移动距离与touch移动距离的比例系数,    即机械臂移动距离=K*touch移动距离
online_receive_Tmatrix_buff_len : 1000  #运控接收来自Touch T矩阵数据的缓存数量
*/
void OnlineTrajectoryPlanner::online_trajectory_algorithm_params_init()
{
    load_OnlineMove_params_Config();
}

int OnlineTrajectoryPlanner::setOnlineTrjRatio_xyz(double data_ratio)
{
    online_alg_params_.trj_ratio_xyz = data_ratio;
    if(!yaml_help_.setParam("online_trj_ratio_xyz", data_ratio) || !yaml_help_.dumpParamFile(config_OnlineMove_params_file_path_.c_str()))
    {
        return 1;//FAIL
    }
    return 0;//SUCCESS
}
int OnlineTrajectoryPlanner::setOnlineTrjRatio_abc(double data_ratio)
{
    online_alg_params_.trj_ratio_xyz = data_ratio;
    if(!yaml_help_.setParam("online_trj_ratio_abc", data_ratio) || !yaml_help_.dumpParamFile(config_OnlineMove_params_file_path_.c_str()))
    {
        return 1;//FAIL
    }
    return 0;//SUCCESS
}
double OnlineTrajectoryPlanner::get_online_trj_ratio_xyz()
{
    printf("\nonline_trajectory_algorithm_params_init:\nsample_time=%lf,generate_interval=%lf,N_step_P=%d,N_step_Q=%d,N_interpP=%lf,N_interpQ=%lf,trj_ratio_xyz=%lf,trj_ratio_abc=%lf,recvTmatrix_buffLen=%d\n",
            online_alg_params_.sample_time,
            online_alg_params_.generate_traj_interval,
            online_alg_params_.N_step_P,
            online_alg_params_.N_step_Q,
            online_alg_params_.N_interp_P,
            online_alg_params_.N_interp_Q,
            online_alg_params_.trj_ratio_xyz,
            online_alg_params_.trj_ratio_abc,
            online_alg_params_.online_receive_Tmatrix_buff_len);
    return online_alg_params_.trj_ratio_xyz;
}
double OnlineTrajectoryPlanner::get_online_trj_ratio_abc()
{
    return online_alg_params_.trj_ratio_abc;
}









