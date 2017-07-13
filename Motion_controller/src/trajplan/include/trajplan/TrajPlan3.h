#ifndef  _trajplan_h
#define  _trajplan_h
#include <iostream>
#include "fst_datatype.h"
#include "fst_trajdatatype.h"
#include <cmath>
#include <vector>
#include <cstring>
#include <complex>


using namespace std;
namespace fst_controller
{
    class TrajPlan
    {
    public:
        //Initialize
        TrajPlan();

        //Version of program
        string getVersion();

        //get length of vector
        int getVlength();
        void setVlength(int _Vlength);

        // set the DH parameters(modified) of the robot in the order of  alpha, a ,d , offset of theta
        void setDH(const DHGroup &_dh);

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
        void setRadiusCoefficient(double _k_radius);

        //set the allowable overshoot of each joint, in radian 
        void setOvershoot(double _overshoot);

        //set the allowable approaching angle to limit , in radian 
        void seterrorangle(double _errorangle);

        //set the allowable acceleration overload ratio of joints
        void setoverload(double _accelerationoverload);

        //set the velocity limit ratio related to the max velocity, normally between 0~1 
        void setLimitScale(double _limit_scale);

        //set the coefficient to estimate rotational times
        void setEstime_coefficient(double _estime_coefficient);

        //set rotation range of 6th axis
        void setTurns_6axis(int _turns_6axis);

        // set smoothing mode
        void setSmoothMode(int _smooth_mode);


        //Forward and Inverse Kinematics
        int ForwardKinematics(const JointValues &joint, Pose &pose);
        int InverseKinematics(const Pose &pose, const JointValues &joint_reference, JointValues &joint_result);

        //PicK points
        int PickPoint(vector<Pose> &Pose_out);
        int PickPoint(vector<JointValues> &Joint_out);
        
        int PickPoint2(vector<Pose> &Pose_out);
        int PickPoint2(vector<JointValues> &Joint_out);

        //Move Command functions
        int MoveL2L(const LInitial &L_initial, const LCommand &L_objective,
                    const LCommand &L_preread, LInitial &L_out, int &n_time);

        int MoveL2C(const LInitial &L_initial, const LCommand &L_objective,
                    const CCommand &C_preread, CInitial &C_out, int &n_time);

        int MoveL2J(const LInitial &L_initial, const LCommand &L_objective,
                    int &ntime);

        int MoveJ2J(const JointPoint &J0, const JCommand &J_obj,
                    const JCommand &J_preread, JointPoint &J_out, int &ntime);

        int MoveJ2L(const JointPoint &J0, const JCommand &J_obj,
                    const LCommand &L_preread, LInitial &L_out, int &ntime);

        int MoveJ2C(const JointPoint &J0, const JCommand &J_obj,
                    const CCommand &C_preread, CInitial &C_out, int &ntime);

        int MoveC2C(const CInitial &C_initial, const CCommand &C_objective,
                    const CCommand &C_preread, CInitial &C_out, int &n_time);

        int MoveC2L(const CInitial &C_initial, const CCommand &C_objective,
                    const LCommand &L_preread, LInitial &L_out, int &n_time);

        int MoveC2J(const CInitial &C_initial, const CCommand &C_objective, int &n_time);

 
        //get joint status for J command
        int GetJointStatus(const JointValues &J_2ndlast,
                           const JointValues &J_last,
                                 JointPoint &J_startofnxt);

        // Pause and restart
        int Pause  (vector<JointValues> &Joint_diff);
        int Restart(vector<JointValues> &J_diff, JointValues &J0);

        //Translation of different pose expressions
        int PoseQuatern2Matrix(const Pose &P, double Pose_M[4][4]);
        int Matrix2PoseQuatern(double Pose_M[4][4], Pose &P);

        int PoseEuler2Matrix(const PoseEuler &P, double Pose_M[4][4]);
        int Matrix2PoseEuler(double Pose_M[4][4], PoseEuler &P);

        int Quatern2Euler(const Pose &P_Q, PoseEuler &P_E);
        int Euler2Quatern(const PoseEuler &P_E, Pose &P_Q);

        //Translate Pose to array in Quaternion
        void Pose2Array(const Pose &P, double PST[3], double ORT[4]);
        void Array2PoseEuler(const double p[6], PoseEuler &P);
        void Array2Euler(const double p[3], Euler &P);
        void Euler2Array(const Euler P, double p[3]);
        void Array2PoseQuatern(const double p[7], Pose &P);
        void Array2Quatern(const double p[4], Quaternion &P);
        void Quatern2Array(const Quaternion P, double p[4]);
        void Array2Position(const double p[3], Point &P);
        void Position2Array(const Point P, double p[3]);
        void Joint2Array(const JointValues P, double p[6]);
        void Array2Joint(const double p[6], JointValues &P);

        //Frame and point Offset
        void Offset_frame(const PoseEuler &P_origin, const PoseEuler &P_offset, PoseEuler &P_out);
        void Offset_point(const Pose &p_before, const PoseEuler &P_offset, Pose &p_after);

        //Judge the objective pose, if same with current pose ,return error code
        int Check_coincidence_c(const Pose &P1, const Pose &P2);
        int Check_coincidence_jc(const JointValues &J1, const Pose &P2);
        int Check_coincidence_j(const JointValues &J1, const JointValues &J2);
        
        //For Test Program
        int CheckJointVector(const vector<JointValues> J_diff);

        //Print
        void PrintPose(Pose P);
        void PrintJointValues(JointValues J0);
        void PrintJointPoint(JointPoint J0);
        int IKandPrint(vector<Pose> &Pose_out, JointValues &J0, vector<JointValues>&Joint_out, string filename);
        int IKandPrint(vector<JointValues> &Pose_out, string filename);
        //Timing in Windows
        //DWORD  time_info();
        //Timing in Linux
        //void info();

    private:

        //Max and Min
        double Min(double a, double b);
        double Max(double a, double b);

        //Transformation matrix
        void RotX(double t, double  M[4][4]);
        void RotY(double t, double  M[4][4]);
        void RotZ(double t, double  M[4][4]);
        void Trans(double x, double y, double z, double M[4][4]);
        void DHMatrix(double L[4], double q, double  M[4][4]);


        //Matrix and vector compution
        bool InverseMatrix(double src[4][4], double des[4][4]);
        double InnerProduct(double u[3], double v[3]);
        void nV3(double n, double V[3], double nV[3]);
        void add3(double u[3], double v[3], double w[3]);
        void minus3(double u[3], double v[3], double w[3]);

        int Sign(double b);
        double Norm(double Vector[3]);
        double Norm4(double Vector[4]);
        void Cross(double u[3], double v[3], double w[3]);
        void Multiply_M(double M1[4][4], double M2[4][4], double M[4][4]);
        void Multiply_MV(double M[4][4], double V[4], double U[4]);

        //四元数相关函数

        int Squad(double q1[4], double q2[4], double s1[4], double s2[4], double u, double qu[4]);
        int Slerp(double q0[4], double q1[4], double  h, double qh[4]);

        void Si(double qiq[4], double qi[4], double qih[4], double si[4]);
        void nV4(double n, double V[4], double nV[4]);
        void add4(double u[4], double v[4], double w[4]);
        void ExpQ(double L[4], double E[4]);
        double InnerProduct4(double u[4], double v[4]);
        void LogQ(double q[4], double l[4]);
        void MultiQ(double u[4], double v[4], double w[4]);
        void InvQ(double q[4], double iq[4]);
        int Quaternprv(double Q1[4], double Q2[4], double Q_prv[4]);
        double Angle_Qs(double q0[4], double q1[4]);
        void ModifyQ2(const Quaternion &q1, Quaternion &q2);
        double CalUAcc(double ORT_0[4], double ORT_obj[4]);

        //运动学函数
        int JudgeAxis(double THETA[6]);
        double ReviseJoint(double t);
        int CheckJoint(const JointValues &Joint);

        //规划和插值
        int Smooth_arc(double P0[3], double P1[3], double P2[3],
                    double &r, double B1[3], double B2[3],
                    double O[3], double &angle, double T[4][4]);
  

        int Polyn3_seg(double t, double SVA[3], double P[4]);
//        int Polyn3_7seg_t(double d, double v0, double v1, double t, double a, vector<double> &di);
        int Interp_singlejoint_t(double d, double v0, double v1, double t, double P[4]);

        int Arc_3points(double P1[3], double P2[3], double P3[3], double T[4][4], double &r, double &theta3, double &theta2, double O[3]);

        
        int Interpolation_mode(double d, double v0, double v_obj, double &v_cnt1, double a, ElementSmooth &Time_Polyn);
        int Interpolation_mode_t(double d, double v0, double v1, double t, double a, ElementSmooth &Time_Polyn);
        int TrapeCurve(double d, double v0, double v_obj, double &v_cnt1, double a, double T[8], double P[7][4]);
        int TrapeCurve_t(double d, double v0, double v1, double t, double a, double T[8], double P[7][4]);
        int SCurve(double d, double v0, double v_obj, double &v_cnt1, double a, double T[8], double P[7][4]);
        int SCurve_t(double d, double v0, double v1, double t, double a, double T[8], double P[7][4]);
        void InterpTP(double T[8], double P[7][4], vector<double> in, vector<double> &out);
        void InterpArc(Arc arc, vector<double> ti_arc, vector<Point> &arc_diff);
        void InterpLine(double PST0[3], double DDX[3], vector<double> l_diff, vector<Point> &p_diff);
        void SquadCondition(vector<Quaternion> Orientation, orienttrans &orient);
        void InterpOrient(orienttrans orient, vector<double> u_diff, vector<Quaternion> &orient_out);
        int MoveJ(const JointPoint &J0, JointPoint &J1, double v_percentage, vector<ElementSmooth> &E);
        int Circle(const Pose &P0, double &v0,
                   const Pose &P1, const Pose &P2, double vc, int &cnt, 
                   ElementSmooth &Time_Polyn);
        int OrientSmooth(const vector<Quaternion> &Q, double t_pos[3], ElementSmooth &Time_Polyn);
        void Interp_pandv(double P0[3], double V0[3], double P1[3], double V1[3], double T,
            Parabola &PB);
        void InterpParabola(const Parabola &PB, vector<double> ti, vector<Point> &pst_diff);

        int CheckSmoothParameter(int cnt, double smooth_radius);


        //configurable parameters

        double DH[6][4];
        double UserFrame[4][4];
        double ToolFrame[4][4];
        double AXIS_LIMIT[4][6];
        double acc;
        double ja_ratio;
        double tc;
        double limit_scale;
        double overshoot;
        double errorangle;
        double accelerationoverload;
        double omega_ref;
        double vel_ref;
        double angle_acc;
        double k_radius;
        double minim;
        double estime_coefficient;
        int curve_mode;
        int turns_6axis;
        int Vlength;
        int smooth_mode;

        //un-configurable parameters
        double InvUF[4][4];
        double InvTF[4][4];
        double InvT66[4][4];
        double a1;
        double a2;
        double a3;
        double d1;
        double d3;
        double d4;
        double offset1;
        double offset2;
        double offset3;
        double offset4;
        double offset5;
        double offset6;
        double time_error;
        int sizeofdouble;
        int PickFlag;
        double tan_threshold_low, tan_threshold_high;
        
    private:
        int count;                        // count pick point
        
        /* Traj time and length*/
        /* For all Trajs*/
        double t_part1, t_part2;
        int n_time_part1, n_time_part2;
        
        /* For L&C */
        ElementSmooth Time_Polyn_part1,Time_Polyn_orient;
        Arc arc;  

        /* For L */
        double DDX[3], PST_linestart[3];  //unit line direction vector, line start point
        orienttrans orient;               //Orient info for squad

        /* For C */
        orienttrans orient_c1, orient_c2;        
        
        /* For J */
        vector<ElementSmooth> SixElementsJ1to6;
        JointValues Joint_start,Joint_end;
  
        /* For L2C*/
        Parabola Parabola_lc;

                 
    };

    



}


#endif
