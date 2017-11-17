#ifndef  _trajplan_h
#define  _trajplan_h
#include <iostream>
#include "fst_datatype.h"
#include <cmath>
#include <vector>
#include <cstring>
#include <complex>

//#include <windows.h>


using namespace std;
namespace fst_controller
{
    class MoveCommand;
    class TrajPlan
    {
        friend class MoveCommand;
        friend class ManualTeach;
    public:
        TrajPlan();        //Initialize

        string getVersion();        //Version of program
        int  getVlength();        //get length of vector
        void setVlength(int _Vlength);
        void setDH(const DHGroup &_dh);    // set the DH parameters(modified) of the robot : alpha, a ,d , offset of theta
        void setToolFrame(const PoseEuler &P);    //set the toolframe using a Euler-format pose
        void setUserFrame(const PoseEuler &P);     //set the userframe using a Euler-format pose
        void setAxisLimit(const JointConstraint &axislimit); //set lower-limit,upper-limit,max velocity and max acceleration
        void setCycleTime(double _tc);        //set the cycle time 
        void setVelocityRef(double _vel_ref);
        void setOmegaRef(double _omega_ref);        //set the reference linear and angular velocity in Cartesian coordinates
        void setAcceleration(double _acc); //set the max acceleration of TCP in Cartesian coordinates
        void setJaratio(double _jerk);        //set the jerk of TCP
        void setCurveMode(CurveMode _curve_mode);        //Select Curve mode:1 for 2-order polynominal, 2 for 3-order polynominal
        void setRadiusCoefficient(double _k_radius);        //set smoothing radius coefficient
        void setOvershoot(double _overshoot);        //set the allowable overshoot of each joint, in radian 
        //void seterrorangle(double _errorangle);        //set the allowable approaching angle to limit , in radian 
        void setoverload(double _accelerationoverload);        //set the allowable acceleration overload ratio of joints
        void setEstime_coefficient(double _estime_coefficient);        //set the coefficient to estimate rotational times
        void setTurns_6axis(int _turns_6axis);        //set rotation range of 6th axis
        void setSmoothMode(SmoothMode _smooth_mode);        // set smoothing mode
        void set_k_fifolength(double _k_fifolength);

        void setOverallSpeedRatio(double _speed_ratio);
        
        int ForwardKinematics(const Joint &joint, Pose &pose);
        int ForwardKinematics_world(const Joint &joint, PoseEuler &pose_flange,PoseEuler &pose_tcp);
        int InverseKinematics(const Pose &pose, const Joint &joint_reference, Joint &joint_result);//Forward and Inverse Kinematics
        
        int EstimateFIFOlength(const Joint &J_2ndlast, const Joint &J_last);

        // Pause and restart
        int Pause(const vector<JointPoint> &Joint_diff, vector<JointPoint> &Joint_out,JointPoint &J_end);
        int Pause2(const vector<JointPoint> &Joint_diff, vector<JointPoint> &Joint_out,  JointPoint &J_end);
        //int Resume(const Joint &Joint_current, const Joint &Joint_pause, MoveCommand *CurrentTraj, vector<JointPoint> &Joint_out);

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
        void Euler2Array(const Euler &P, double p[3]);
        void Array2PoseQuatern(const double p[7], Pose &P);
        void Array2Quatern(const double p[4], Quaternion &P);
        void Quatern2Array(const Quaternion &P, double p[4]);
        void Array2Position(const double p[3], Point &P);
        void Position2Array(const Point &P, double p[3]);
        void Joint2Array(const Joint &P, double p[6]);
        void Array2Joint(const double p[6], Joint &P);
        void Omega2Array(const JointOmega &P, double p[6]);
        void Array2Omega(const  double p[6], JointOmega &P);

        //Frame and point Offset
        void Offset_frame(const PoseEuler &P_origin, const PoseEuler &P_offset, PoseEuler &P_out);
        void Offset_point(const Pose &p_before, const PoseEuler &P_offset, Pose &p_after);

        //Judge the objective pose, if same with current pose ,return error code
        int Check_coincidence(const Pose &P1, const Pose &P2);
        int Check_coincidence(const Joint &J1, const Pose &P2);
        int Check_coincidence(const Pose &P2, const Joint &J1);
        int Check_coincidence(const Joint &J1, const Joint &J2);

        void PrintPose(const Pose &P);
        void PrintPoseEuler(const PoseEuler &P);
        void PrintJointValues(const Joint &J0);
        void PrintJointPoint(const JointPoint &J0);
        int IKandPrint(const vector<Pose> &Pose_out, Joint &J0, vector<Joint>&Joint_out, string filename);
        int IKandPrint(const vector<JointPoint> &Pose_out, string filename);
        int IKandPrint(const vector<PathPoint> &Point_out, Joint &J0, vector<JointPoint>&Joint_out, string filename);
        
        //DWORD  time_info();   //Timing in Windows        
        //void info();          //Timing in Linux

    private:
        void add4(double u[4], double v[4], double w[4]);
        void nV4(double n, double V[4], double nV[4]);
        double InnerProduct4(double u[4], double v[4]);
        int Slerp(double q0[4], double q1[4], double  h, double qh[4]);
        void Multiply_MV(double M[4][4], double V[4], double U[4]);

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

        //四元数相关函数

        int Squad(double q1[4], double q2[4], double s1[4], double s2[4], double u, double qu[4]);
        void Si(double qiq[4], double qi[4], double qih[4], double si[4]);
        void ExpQ(double L[4], double E[4]);
        void LogQ(double q[4], double l[4]);
        void MultiQ(double u[4], double v[4], double w[4]);
        void InvQ(double q[4], double iq[4]);
        int Quaternprv(double Q1[4], double Q2[4], double Q_prv[4]);
        double Angle_Qs(double q0[4], double q1[4]);
        void ModifyQ2(const Quaternion &q1, Quaternion &q2);
        double CalUAcc(double ORT_0[4], double ORT_obj[4]);
        //double CalUAcc(double ORT_prv[4], double ORT_0[4], double ORT_obj[4]);
        double CalUAcc(vector<Quaternion> Orientation);
        int JudgeAxis(double THETA[6]);
        void ReviseJoint(double J[6],double Joint_ref[6]);

        //Move Command functions
        int MoveL2L(const LInitial &L_initial, const LCommand &L_objective,
            const LCommand &L_preread, LInitial &L_out);
        int MoveL2C(const LInitial &L_initial, const LCommand &L_objective,
            const CCommand &C_preread, CInitial &C_out);
        int MoveL2J(const LInitial &L_initial, const LCommand &L_objective);
        int MoveJ2J(const JointPoint &J0, const JCommand &J_obj,
            const JCommand &J_preread, JointPoint &J_out);
        int MoveJ2L(const JointPoint &J0, const JCommand &J_obj,
            const LCommand &L_preread, LInitial &L_out);
        int MoveJ2C(const JointPoint &J0, const JCommand &J_obj,
            const CCommand &C_preread, CInitial &C_out);
        int MoveC2C(const CInitial &C_initial, const CCommand &C_objective,
            const CCommand &C_preread, CInitial &C_out);
        int MoveC2L(const CInitial &C_initial, const CCommand &C_objective,
            const LCommand &L_preread, LInitial &L_out);
        int MoveC2J(const CInitial &C_initial, const CCommand &C_objective);

        int Restart(JointPoint &J0, vector<JointPoint> &Joint_diff, vector<JointPoint> &Joint_out);

        int Smooth_arc(double P0[3], double P1[3], double P2[3],
            double &r, double B1[3], double B2[3],
            double O[3], double &angle, double T[4][4]);


        int Polyn3_seg(double t, double SVA[3], double P[4]);
        //        int Polyn3_7seg_t(double d, double v0, double v1, double t, double a, vector<double> &di);
        int Interp_singlejoint_t(double d, double v0, double v1, double t, double P[4]);

        int Arc_3points(double P1[3], double P2[3], double P3[3], double T[4][4], double &r, double &theta3, double &theta2, double O[3]);

        int Interpolation_mode(double d, double v0, double v_obj, double &v_cnt1, double a, ElementSmooth &Time_Polyn);
        int Interpolation_mode_t(double d, double v0, double &v1, double t, double a, ElementSmooth &Time_Polyn);
        int TrapeCurve(double d, double v0, double v_obj, double &v_cnt1, double a, double T[8], double P[7][4]);
        int TrapeCurve_t(double d, double v0, double &v1, double t, double a, double T[8], double P[7][4]);
        int SCurve(double d, double v0, double v_obj, double &v_cnt1, double a, double T[8], double P[7][4]);
        int SCurve_t(double d, double v0, double &v1, double t, double a, double T[8], double P[7][4]);
        void SquadCondition(const vector<Quaternion> &Orientation, orienttrans &orient);
        int MoveJ(const JointPoint &J0, JointPoint &J1, double v_percentage, vector<ElementSmooth> &E);
        int Circle(const Pose &P0, double &v0,const Pose &P1, const Pose &P2, double vc, int &cnt, ElementSmooth &Time_Polyn);
        int OrientSmooth(const vector<Quaternion> &Q, double t_pos[3], ElementSmooth &Time_Polyn);
        void Interp_pandv(double P0[3], double V0[3], double P1[3], double V1[3], double T,
            Parabola &PB);
        int CheckJoint(const Joint &Joint_in, Joint Joint_start, Joint Joint_end);
        int CheckSmoothParameter(int cnt, double smooth_radius);
        void ResetPickParameters();


        //configurable parameters
        string version;
        double DH[6][4];
        double UserFrame[4][4];
        double ToolFrame[4][4];
        double AXIS_LIMIT[4][6];
        double acc;
        double ja_ratio;
        double tc;
        //double limit_scale;    delete 2017.08.24
        double overshoot;
        //double errorangle;
        double accelerationoverload;
        double omega_ref;
        double vel_ref;
        double angle_acc;
        double k_radius;
        double k_fifolength;
        double estime_coefficient;        
        unsigned int turns_6axis;
        unsigned int Vlength;
        CurveMode curve_mode;
        SmoothMode smooth_mode;

        double overall_speed_ratio;
        double resume_speed_ratio;

        //un-configurable parameters
        double InvUF[4][4], InvTF[4][4], InvT66[4][4];
        double a1, a2, a3, d1, d3, d4, offset1, offset2, offset3, offset4, offset5, offset6;
        double time_error;
        double tan_threshold_low, tan_threshold_high;
        double minim; /* error tolerence*/
        double t_part1, t_part2;/* Traj time and length*/
        int n_time_part1, n_time_part2;/* For all Trajs*/
        ElementSmooth Time_Polyn_part1, Time_Polyn_orient;/* For L&C */
        Arc arc;
        double DDX[3], PST_linestart[3];  /* For L *///unit line direction vector, line start point
        orienttrans orient;               //Orient info for squad
        orienttrans orient_c1, orient_c2; /* For C */
        vector<ElementSmooth> SixElementsJ1to6;/* For J */
        Joint Joint_start, Joint_end;
        Parabola Parabola_lc;/* For L2C*/

    };

    class MoveCommand
    {
        friend class TrajPlan;
    public:

        MoveCommand();
        // MoveCommand(TrajPlan *obj, MotionTarget Target1,int ID);
        MoveCommand(TrajPlan *obj, const MotionTarget &Target1, const MotionTarget &Target2, int ID);

        MoveCommand *prev;
        MoveCommand *next;
        void LinkPrevObj(MoveCommand *prevobj);

        void setInitialJoint(Joint _Joint_wo_prev);   //For the first command, no initial status,set current joints
        int  PlanTraj();
   
        int  PickPoint_origin(vector<PathPoint> &Points_out);
        int  PickPoint(vector<PathPoint> &Points_out);
        int  PickPoint_origin(vector<PathPoint> &Points_out, int pick_length);
        int  PickPoint(vector<PathPoint> &Points_out, int pick_length);
        void GetJointStatus(const Joint &J_2ndlast, const Joint &J_last);  //After IK, call this function to get initial joints status 

        int  Resume(const Joint &Joint_current, const Joint &Joint_pause, vector<JointPoint> &Joint_out);//First segment of resumption, appliable for current command        

 
        //get functions
        int  getMotionID();        
        int  getTrajLength();
        int  getPickedlength();
        int  getPlannedFlag();
        PathPoint   getEndPoint();
        MotionType  getMotionType();
        SmoothType  getSmoothType();
        //set functions
        void setPickedlength(int _picked_traj_length);
        void JoinVectors(vector<Joint> &J1, vector <Joint> &J2); 
        void JoinVectors(vector<PathPoint> &J1, vector <PathPoint> &J2);
        void JoinVectors(vector<JointPoint> &J1, vector <JointPoint> &J2);
        bool getFinishFlag();

    private:
        TrajPlan *ins;
        void getTrajResult();
        void InterpTP(double T[8], double P[7][4], vector<double> &in, vector<double> &out);
        void InterpArc(Arc arc, vector<double> &ti_arc, vector<Point> &arc_diff);
        void InterpLine(double PST0[3], double DDX[3], vector<double> &l_diff, vector<Point> &p_diff);
        void InterpOrient(orienttrans orient, vector<double> &u_diff, vector<Quaternion> &orient_out);
        void InterpParabola(const Parabola &PB, vector<double> &ti, vector<Point> &pst_diff);
        void ConvertJP(const vector<PathPoint> &Point_out, vector<JointPoint> &Joint_out);
        void ConvertJP(const vector<JointPoint> &Point_out, vector<PathPoint> &Joint_out);
        void ConvertJP(const PathPoint &Point_out, JointPoint &Joint_out);
        void ConvertJP(const JointPoint &Joint_out, PathPoint &Point_out);
        int  Resume2(vector<JointPoint> &Joint_out);//Following segment of resumption, appliable if robot can't finish resumption in first segment

        void CalculateInitialVelocity(double T[8], double P[7][4], double &joint_vel);
        double CalculateVelVarTime();
        void ReinterpResumeVector(vector<double> ti, vector<PathPoint> &Points_out);
        double timemapping(double k1, double k2, double t_v, double t_dyn, double &k_cur);
        int InterpTraj(vector<double> ti, vector<PathPoint> &Points_out);
        void CalculateNewTimeVector(double &time_initial, double time_all, double time_vel_variation, vector<double> &ti);
        Joint Joint_resume_pick_sta;
        Joint Joint_wo_prev;
        JointPoint J_restart_initial;
        LInitial   L_initial; LCommand L_object; LCommand L_preread; LInitial   L_out;
        CInitial   C_initial; CCommand C_object; CCommand C_preread; CInitial   C_out;
        JointPoint J_initial; JCommand J_object; JCommand J_preread; JointPoint J_out;

        double tc;
        double speed_ratio;
        SmoothType NextMotionType;
        MotionType CurrentMotionType;
        int TrajLength, picked_traj_length, Vlength, MotionID;     // count pick point
        int Flag_resume_unfinish;//Planned_flag,
        bool Flag_setInitialJoint, Flag_getendjointstatus, Flag_finish;
        bool Flag_resume_vector;
       
        
        double time_section_initial,time_resume_initial;        /*time initial of vel variation section*/
        double t_part1, t_part2;                          /* Traj time and length*/
        int n_time_part1, n_time_part2;                   /* For all Trajs*/

        ElementSmooth Time_Polyn_part1, Time_Polyn_orient;/* For L&C */
        Arc arc;
        Parabola Parabola_lc;                             /* For L2C*/
        double DDX[3], PST_linestart[3];  //unit line direction vector, line start point
        orienttrans orient,orient_c1, orient_c2;               //Orient info for squad
        vector<ElementSmooth> SixElementsJ1to6;/* For J */
        Joint Joint_start, Joint_end;

        vector<JointPoint> JointVector_resume,JointVector_resume_copy;
        vector<double> ti_nxt;

    };

    class ManualTeach
    {
        friend class TrajPlan;
    public:
        ManualTeach();
        ManualTeach(TrajPlan *obj);
        void ManualCartesian(vector<int> manual_button, vector<PathPoint> &P_out);
        void ManualJoint(vector<int> manual_button, vector<PathPoint> &J_out);
        void setManualLength(unsigned int _length);
        void setCurrentStatus(Joint _joints);
        void setManualSpeedRatio(double _ratio);
        void setManualCartesianMode(ManualMode _mode);
    private:
        TrajPlan *ins;

        void ManualSinglePosition(double &p0, double &v0, double acc, double v_obj);
        
        unsigned int manual_length;
        JointPoint joint_status;     // value and omegas of robot
        PoseVel pose_status;
        double manual_speed_ratio;   // speed ratio of manual teach
        ManualMode ManualCartesianMode;


    };

}


#endif



