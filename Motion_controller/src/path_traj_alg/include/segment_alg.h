/*
Important Note: all APIs provided by this source are not support multi thread operation!
*/

#ifndef SEGMENT_ALG_H
#define SEGMENT_ALG_H

#include <math.h>
#include <cstring>
#include <string>
#include "basic_alg_datatype.h"
#include "common_enum.h"
#include "motion_control_datatype.h"
#include "coordinate_manager.h"
#include "tool_manager.h"
#include "transformation.h"
#include "kinematics.h"
#include "dynamics_interface.h"
#include "error_code.h"
#include <basic_alg.h>

#define DOUBLE_ACCURACY 1e-6
#define SQRT_DOUBLE_ACCURACY 1e-12

//typedef ErrorCode (fst_mc::BaseKinematics::*ComputeIKFunc)(const fst_mc::Pose      &pose, const fst_mc::Joint &ref, fst_mc::Joint &res);
//typedef bool (fst_algorithm::DynamicsInterface::*ComputeDynamicsFunc)(const float joint[6], const float omega[6], float alpha_max[2][6]);

typedef struct
{
    int accuracy_cartesian_factor;  // define the number of trajectory points on per 100mm cartesian path
    int accuracy_joint_factor;      // define the number of trajectory points on per PI joint path
    int max_traj_points_num;    // max traj points per trajectory, up to 20 points
    double path_interval;       // path points interval in mm
    double joint_interval;      // joint angle interval in rad
    double angle_interval;    // quatern angle interval in rad
    double angle_valve;       // valve for slerp or linear interpolation for quatern angle
    double conservative_acc;    // conservative accerl
    bool is_fake_dynamics;
    double time_factor_first;   // the time factor of the first piece
    double time_factor_last;   // the time factor of the last piece
    double max_cartesian_acc;   // mm/s^2
    fst_ctrl::CoordinateManager* coordinate_manager_ptr;
    fst_ctrl::ToolManager* tool_manager_ptr;
    basic_alg::Kinematics* kinematics_ptr;
    fst_algorithm::DynamicsInterface* dynamics_ptr;
}SegmentAlgParam;

typedef enum
{
    JOINT_TYPE_ROTATE = 0,
    JOINT_TYPE_PRISMATIC,
}JointType;

typedef enum
{
    DH_TYPE_STANDARD = 0,
    DH_TYPE_MODIFY
}DHType;

typedef struct
{
    double theta;   // link angle
    double d;       // link offset
    double alpha;   // link twist
    double a;       // link length
    double theta_offset;    // joint variable offset
    double d_offset;        // joint variable offset
    double alpha_offset;    // joint variable offset
    double a_offset;        // joint variable offset
    double qlim[2]; // joint coordinate hardware limits
    bool is_flip;      // joint moves in opposite direction
}LinkKinematic;

typedef struct
{
    double m;   // link mass
    double r[3];    // position of COM with respect to link frame (3x1)
    double inertial[3][3]; // inertia of link with respect to COM (3x3)
}LinkDynamic;

typedef struct
{
    double inertial;  // motor inertia
    double tm;  // max torque of motor
    double vm;  // max vel of motor, rpm
    double am;  // max acc of motor
    double jm;  // max jerk of motor
    double b[2]; // viscous friction
    double tb[2]; // coulomb friction
    double gear; // gear ratio, joint to motor
    int zero;  // zero position expressed in encoder pulse 
}MotorDynamic;

typedef struct
{
    JointType joint_type;
    LinkKinematic link_kinematic;
    LinkDynamic link_dynamic;
    MotorDynamic motor_dynamic;
}LinkModel;

typedef struct 
{
    std::string robot_name;
    DHType dh_type;
    double cart_vm;
    double cart_am;
    double cart_jm;
    double gravity;
    int link_num;
    LinkModel link[9];    
}ComplexAxisGroupModel;


typedef enum
{
    // basic parameters derive from robot model or algorithm
    S_RealTheta = 0,  // vector in size 9
    S_RealD = 9,  // vector in size 9
    S_RealAlpha = 18,  // vector in size 9
    S_RealA = 27,  // vector in size 9
    S_ConstraintJointVelMax = 36,    // vector in size 9 
    S_BSplineNodeVector = 45,    // vector in size 6
    S_PathCountFactorCartesian = 51,    // double
    S_PathCountFactorJoint = 52,    // double

    S_CircleAngle = 53,    // double
    S_CircleRadius = 54,    // double
    S_CircleAngleVia2In = 55,
    S_CircleAngleIn2Out = 56,

    S_CircleCenter = 60,    // double
    S_CircleVectorN = 65, //65/66/67
    S_CircleVectorO = 68, // 

    S_PauseTimeFactor = 100,    // double
    S_PausePathLengthFactor = 101,    // double
    S_PauseAccCartesian = 102,
    S_PauseAccJoint = 103,

    // basic matrix operation
    S_TransMatrix = 200,  // matrix[4][4]
    S_HomoTransMatrix = 216, // matrix[4][4]

    S_PausePointState0 = 236, // vector in size 3
    S_PausePointState1 = 239,
    S_PausePointState2 = 242,
    S_PausePointState3 = 245,
    S_PausePointState4 = 248,
    S_PausePointState5 = 251,
    S_PausePointState6 = 254,
    S_PausePointState7 = 257,
    S_PausePointState8 = 260, 

    // tmp variable, vector, matrix 
    S_TmpDouble_1 = 500,    // double
    S_TmpDouble_2 = 501,
    S_TmpDouble_3 = 502,
    S_TmpDouble_4 = 503,
    S_TmpDouble_5 = 504,
    S_TmpDouble_6 = 505,
    S_TmpDouble_7 = 506,
    S_TmpDouble_8 = 507,

    S_TmpVector3_1 = 550, // vector[3]
    S_TmpVector3_2 = 553,
    S_TmpVector3_3 = 556,
    S_TmpVector3_4 = 559,

    S_TmpVector4_1 = 600, // vector[4]
    S_TmpVector4_2 = 604,
    S_TmpVector4_3 = 608,
    S_TmpVector4_4 = 612,

    S_TmpVector6_1 = 650, // vector[6]
    S_TmpVector6_2 = 656, 
    S_TmpVector6_3 = 662, 
    S_TmpVector6_4 = 668, 

    S_TmpMatrix33_1 = 700, // matrix[3][3]
    S_TmpMatrix33_2 = 709, 
    S_TmpMatrix33_3 = 718, 
    S_TmpMatrix33_4 = 727,

    S_TmpMatrix44_1 = 750, // matrix[4][4]
    S_TmpMatrix44_2 = 766,

    S_TmpMatrix66_1 = 800, // matrix[6][6]
    S_TmpMatrix66_2 = 836,

    // BSpline computation related, support up to 1000 interpolation points
    S_BaseFunctionNik0 = 850,   // base function Nik0, double 
    S_BaseFunctionNik1 = 851,   // base function Nik1, double 
    S_BaseFunctionNik2 = 852,   // base function Nik2, double
    S_BSpLineResultXBase = 900,
    S_BSpLineResultYBase = 1900,
    S_BSpLineResultZBase = 2900,
 
    // banding matrix related, support up to 20 order of the matrix
    S_A = 3900,   // up to matrix in size 20*20
    S_B = 4400, // up to vector in size 20
    S_X = 4425, // up to vector in size 20 
    
    // compute TrajP/V/A by PathX & TrajT
    S_DeltaJointVector = 4450,   // vector in size 9, fabs(joint end -joint start)
    S_PathIndexStep_Start2End = 4460, // path index interval between two traj points   
    S_PathIndexStep_Start2Out = 4461,
    S_PathIndexStep_Out2End = 4462,
    S_PathIndexStep_Out2In = 4463,
    S_PathIndexStep_In2Out = 4464,
    S_PathIndexStep_In2End = 4465,
    
    S_TrajP0 = 4500,    // {P,V,A} of joint0 on traj points, up to 20 points
    S_TrajV0 = 4525,
    S_TrajA0 = 4550,
    S_TrajP1 = 4575,
    S_TrajV1 = 4600,
    S_TrajA1 = 4625,
    S_TrajP2 = 4650,
    S_TrajV2 = 4675,
    S_TrajA2 = 4700,
    S_TrajP3 = 4725,
    S_TrajV3 = 4750,
    S_TrajA3 = 4775,
    S_TrajP4 = 4800,
    S_TrajV4 = 4825,
    S_TrajA4 = 4850,
    S_TrajP5 = 4875,
    S_TrajV5 = 4900,
    S_TrajA5 = 4925,
    S_TrajP6 = 4950,
    S_TrajV6 = 4975,
    S_TrajA6 = 5000,
    S_TrajP7 = 5025,
    S_TrajV7 = 5050,
    S_TrajA7 = 5075,
    S_TrajP8 = 5100,
    S_TrajV8 = 5125,
    S_TrajA8 = 5150,         
    S_TrajT = 5175,
    S_TrajRescaleFactor = 5200,

    S_TrajP0_Smooth = 5225,    // {P,V,A} of joint0 on smooth traj points, up to 20 points
    S_TrajV0_Smooth = 5250,
    S_TrajA0_Smooth = 5275,
    S_TrajP1_Smooth = 5300,
    S_TrajV1_Smooth = 5325,
    S_TrajA1_Smooth = 5350,
    S_TrajP2_Smooth = 5375,
    S_TrajV2_Smooth = 5400,
    S_TrajA2_Smooth = 5425,
    S_TrajP3_Smooth = 5450,
    S_TrajV3_Smooth = 5475,
    S_TrajA3_Smooth = 5500,
    S_TrajP4_Smooth = 5525,
    S_TrajV4_Smooth = 5550,
    S_TrajA4_Smooth = 5575,
    S_TrajP5_Smooth = 5600,
    S_TrajV5_Smooth = 5625,
    S_TrajA5_Smooth = 5650,
    S_TrajP6_Smooth = 5675,
    S_TrajV6_Smooth = 5700,
    S_TrajA6_Smooth = 5725,
    S_TrajP7_Smooth = 5750,
    S_TrajV7_Smooth = 5775,
    S_TrajA7_Smooth = 5800,
    S_TrajP8_Smooth = 5825,
    S_TrajV8_Smooth = 5850,
    S_TrajA8_Smooth = 5875,         
    S_TrajT_Smooth = 5900,
    S_TrajRescaleFactor_Smooth = 5925,

    S_StartPointState0 = 5950, // vector in size 3
    S_StartPointState1 = 5953,
    S_StartPointState2 = 5956,
    S_StartPointState3 = 5959,
    S_StartPointState4 = 5962,
    S_StartPointState5 = 5965,
    S_StartPointState6 = 5968,
    S_StartPointState7 = 5971,
    S_StartPointState8 = 5974,

    S_OutPointState0 = 5977, // vector in size 3
    S_OutPointState1 = 5980,
    S_OutPointState2 = 5983,
    S_OutPointState3 = 5986,
    S_OutPointState4 = 5989,
    S_OutPointState5 = 5992,
    S_OutPointState6 = 5995,
    S_OutPointState7 = 5998,
    S_OutPointState8 = 6001,

    S_InPointState0 = 6004, // vector in size 3
    S_InPointState1 = 6007,
    S_InPointState2 = 6010,
    S_InPointState3 = 6013,
    S_InPointState4 = 6016,
    S_InPointState5 = 6019,
    S_InPointState6 = 6022,
    S_InPointState7 = 6025,
    S_InPointState8 = 6028,

    S_EndPointState0 = 6031, // vector in size 3
    S_EndPointState1 = 6034,
    S_EndPointState2 = 6037,
    S_EndPointState3 = 6040,
    S_EndPointState4 = 6043,   
    S_EndPointState5 = 6046,
    S_EndPointState6 = 6049,
    S_EndPointState7 = 6052,
    S_EndPointState8 = 6055,

    S_TrajJ0 = 6060, // vector in size 4, {J1, J2, Jn-1, Jn}
    S_TrajJ1 = 6064,
    S_TrajJ2 = 6068,
    S_TrajJ3 = 6072,
    S_TrajJ4 = 6076,
    S_TrajJ5 = 6080,
    S_TrajJ6 = 6084,
    S_TrajJ7 = 6088,
    S_TrajJ8 = 6092,
    
    // compute robot dynamics constraints, put result in ConstaintJointPos/NegAX
    S_ConstraintJointPosA0 = 6100, // positive max acc of joint0, from traj point(1) ~ point(n), up to 20 points
    S_ConstraintJointNegA0 = 6125,
    S_ConstraintJointPosA1 = 6150,
    S_ConstraintJointNegA1 = 6175,    
    S_ConstraintJointPosA2 = 6200,
    S_ConstraintJointNegA2 = 6225, 
    S_ConstraintJointPosA3 = 6250,
    S_ConstraintJointNegA3 = 6275, 
    S_ConstraintJointPosA4 = 6300,
    S_ConstraintJointNegA4 = 6325, 
    S_ConstraintJointPosA5 = 6350,
    S_ConstraintJointNegA5 = 6375,
    S_ConstraintJointPosA6 = 6400,
    S_ConstraintJointNegA6 = 6425,
    S_ConstraintJointPosA7 = 6450,
    S_ConstraintJointNegA7 = 6475,
    S_ConstraintJointPosA8 = 6500,
    S_ConstraintJointNegA8 = 6525,

    // traj piece jerk rescale factor
    S_TrajPieceJ0 = 6550,   // up to 19 in size
    S_TrajPieceJ1 = 6575,
    S_TrajPieceJ2 = 6600,
    S_TrajPieceJ3 = 6625,
    S_TrajPieceJ4 = 6650,
    S_TrajPieceJ5 = 6675,
    S_TrajPieceJ6 = 6700,
    S_TrajPieceJ7 = 6725,
    S_TrajPieceJ8 = 6750,
    // traj piece acc rescale factor
    S_TrajPieceA0 = 6775,
    S_TrajPieceA1 = 6800,
    S_TrajPieceA2 = 6825,
    S_TrajPieceA3 = 6850,
    S_TrajPieceA4 = 6875,
    S_TrajPieceA5 = 6900,
    S_TrajPieceA6 = 6925,
    S_TrajPieceA7 = 6950,
    S_TrajPieceA8 = 6975,
    // traj piece vel rescale factor
    S_TrajPieceV0 = 7000,
    S_TrajPieceV1 = 7025,
    S_TrajPieceV2 = 7050,
    S_TrajPieceV3 = 7075,
    S_TrajPieceV4 = 7100,
    S_TrajPieceV5 = 7125,
    S_TrajPieceV6 = 7150,
    S_TrajPieceV7 = 7175,
    S_TrajPieceV8 = 7200,
    // traj piece final rescale factor, up to 19 in size
    S_TrajPieceRescaleFactor = 7225,
    
    // coeff of p = A5*t^5 + A4*t^4 + A3*t^3 + A2*t^2 + A1*t + A0
    S_TrajCoeffJ0A0 = 7300,
    S_TrajCoeffJ0A1 = 7325,
    S_TrajCoeffJ0A2 = 7350,    
    S_TrajCoeffJ0A3 = 7375,
    S_TrajCoeffJ0A4 = 7400,
    S_TrajCoeffJ0A5 = 7425,

    S_TrajCoeffJ1A0 = 7450,
    S_TrajCoeffJ1A1 = 7475,
    S_TrajCoeffJ1A2 = 7500,    
    S_TrajCoeffJ1A3 = 7525,
    S_TrajCoeffJ1A4 = 7550,
    S_TrajCoeffJ1A5 = 7575,

    S_TrajCoeffJ2A0 = 7600,
    S_TrajCoeffJ2A1 = 7625,
    S_TrajCoeffJ2A2 = 7650,    
    S_TrajCoeffJ2A3 = 7675,
    S_TrajCoeffJ2A4 = 7700,
    S_TrajCoeffJ2A5 = 7725,

    S_TrajCoeffJ3A0 = 7750,
    S_TrajCoeffJ3A1 = 7775,
    S_TrajCoeffJ3A2 = 7800,    
    S_TrajCoeffJ3A3 = 7825,
    S_TrajCoeffJ3A4 = 7850,
    S_TrajCoeffJ3A5 = 7875,

    S_TrajCoeffJ4A0 = 7900,
    S_TrajCoeffJ4A1 = 7925,
    S_TrajCoeffJ4A2 = 7950,    
    S_TrajCoeffJ4A3 = 7975,
    S_TrajCoeffJ4A4 = 8000,
    S_TrajCoeffJ4A5 = 8025,

    S_TrajCoeffJ5A0 = 8050,
    S_TrajCoeffJ5A1 = 8075,
    S_TrajCoeffJ5A2 = 8100,    
    S_TrajCoeffJ5A3 = 8125,
    S_TrajCoeffJ5A4 = 8150,
    S_TrajCoeffJ5A5 = 8175,

    S_TrajCoeffJ6A0 = 8200,
    S_TrajCoeffJ6A1 = 8225,
    S_TrajCoeffJ6A2 = 8250,    
    S_TrajCoeffJ6A3 = 8300,
    S_TrajCoeffJ6A4 = 8325,
    S_TrajCoeffJ6A5 = 8350,

    S_TrajCoeffJ7A0 = 8375,
    S_TrajCoeffJ7A1 = 8400,
    S_TrajCoeffJ7A2 = 8425,    
    S_TrajCoeffJ7A3 = 8450,
    S_TrajCoeffJ7A4 = 8475,
    S_TrajCoeffJ7A5 = 8500,

    S_TrajCoeffJ8A0 = 8525,
    S_TrajCoeffJ8A1 = 8550,
    S_TrajCoeffJ8A2 = 8575,    
    S_TrajCoeffJ8A3 = 8600,
    S_TrajCoeffJ8A4 = 8625,
    S_TrajCoeffJ8A5 = 8650,

    // coeff of p_smooth = A5*t^5 + A4*t^4 + A3*t^3 + A2*t^2 + A1*t + A0
    S_TrajCoeffJ0A0_Smooth = 8675,
    S_TrajCoeffJ0A1_Smooth = 8700,
    S_TrajCoeffJ0A2_Smooth = 8725,    
    S_TrajCoeffJ0A3_Smooth = 8750,
    S_TrajCoeffJ0A4_Smooth = 8775,
    S_TrajCoeffJ0A5_Smooth = 8800,

    S_TrajCoeffJ1A0_Smooth = 8825,
    S_TrajCoeffJ1A1_Smooth = 8850,
    S_TrajCoeffJ1A2_Smooth = 8875,    
    S_TrajCoeffJ1A3_Smooth = 8900,
    S_TrajCoeffJ1A4_Smooth = 8925,
    S_TrajCoeffJ1A5_Smooth = 8950,

    S_TrajCoeffJ2A0_Smooth = 8975,
    S_TrajCoeffJ2A1_Smooth = 9000,
    S_TrajCoeffJ2A2_Smooth = 9025,    
    S_TrajCoeffJ2A3_Smooth = 9050,
    S_TrajCoeffJ2A4_Smooth = 9075,
    S_TrajCoeffJ2A5_Smooth = 9100,

    S_TrajCoeffJ3A0_Smooth = 9125,
    S_TrajCoeffJ3A1_Smooth = 9150,
    S_TrajCoeffJ3A2_Smooth = 9175,    
    S_TrajCoeffJ3A3_Smooth = 9200,
    S_TrajCoeffJ3A4_Smooth = 9225,
    S_TrajCoeffJ3A5_Smooth = 9250,

    S_TrajCoeffJ4A0_Smooth = 9275,
    S_TrajCoeffJ4A1_Smooth = 9300,
    S_TrajCoeffJ4A2_Smooth = 9325,    
    S_TrajCoeffJ4A3_Smooth = 9350,
    S_TrajCoeffJ4A4_Smooth = 9375,
    S_TrajCoeffJ4A5_Smooth = 9400,

    S_TrajCoeffJ5A0_Smooth = 9425,
    S_TrajCoeffJ5A1_Smooth = 9450,
    S_TrajCoeffJ5A2_Smooth = 9475,    
    S_TrajCoeffJ5A3_Smooth = 9500,
    S_TrajCoeffJ5A4_Smooth = 9525,
    S_TrajCoeffJ5A5_Smooth = 9550,

    S_TrajCoeffJ6A0_Smooth = 9575,
    S_TrajCoeffJ6A1_Smooth = 9600,
    S_TrajCoeffJ6A2_Smooth = 9625,    
    S_TrajCoeffJ6A3_Smooth = 9650,
    S_TrajCoeffJ6A4_Smooth = 9675,
    S_TrajCoeffJ6A5_Smooth = 9700,

    S_TrajCoeffJ7A0_Smooth = 9725,
    S_TrajCoeffJ7A1_Smooth = 9750,
    S_TrajCoeffJ7A2_Smooth = 9775,    
    S_TrajCoeffJ7A3_Smooth = 9800,
    S_TrajCoeffJ7A4_Smooth = 9825,
    S_TrajCoeffJ7A5_Smooth = 9850,

    S_TrajCoeffJ8A0_Smooth = 9875,
    S_TrajCoeffJ8A1_Smooth = 9900,
    S_TrajCoeffJ8A2_Smooth = 9925,    
    S_TrajCoeffJ8A3_Smooth = 9950,
    S_TrajCoeffJ8A4_Smooth = 9975,
    S_TrajCoeffJ8A5_Smooth = 10000,

    S_BSpLineResultJ1Base = 11000,
    S_BSpLineResultJ2Base = 12000,
    S_BSpLineResultJ3Base = 13000,
    S_BSpLineResultJ4Base = 14000,
    S_BSpLineResultJ5Base = 15000,
    S_BSpLineResultJ6Base = 16000,    
}StackIndex;

extern ComplexAxisGroupModel model;
extern double stack[20000];
extern fst_mc::AxisType seg_axis_type[9];
/***********************************************************************************************/
//void initComplexAxisGroupModel();
void initSegmentAlgParam(SegmentAlgParam* segment_alg_param_ptr, int link_num, fst_mc::AxisType axis_type[NUM_OF_JOINT], double joint_vel_max[NUM_OF_JOINT]);

ErrorCode planPathJoint(const basic_alg::Joint &start, const fst_mc::MotionTarget &end, fst_mc::PathCache &path_cache);
ErrorCode planPathLine(const basic_alg::PoseEuler &start, const fst_mc::MotionTarget &end, fst_mc::PathCache &path_cache);
ErrorCode planPathCircle(const basic_alg::PoseEuler &start, const fst_mc::MotionTarget &end, fst_mc::PathCache &path_cache);

ErrorCode planPathSmoothJoint(const basic_alg::Joint &start, const fst_mc::MotionTarget &via, const fst_mc::MotionTarget &end, fst_mc::PathCache &path_cache);
ErrorCode planPathSmoothLine(const basic_alg::PoseEuler &start, const fst_mc::MotionTarget &via, const fst_mc::MotionTarget &end, fst_mc::PathCache &path_cache);
ErrorCode planPathSmoothCircle(const basic_alg::PoseEuler &start, const fst_mc::MotionTarget &via, const fst_mc::MotionTarget &end, fst_mc::PathCache &path_cache);

ErrorCode planTrajectory(const fst_mc::PathCache &path_cache, const fst_mc::JointState &start_state, double vel_ratio, double acc_ratio, fst_mc::TrajectoryCache &traj_cache);
ErrorCode planTrajectorySmooth(const fst_mc::PathCache &path_cache, const fst_mc::JointState &start_state, const fst_mc::MotionTarget &via, double vel_ratio, double acc_ratio, fst_mc::TrajectoryCache &traj_cache);
ErrorCode planPauseTrajectory(const fst_mc::PathCache &path_cache, const fst_mc::JointState &start_state, double acc_ratio, fst_mc::TrajectoryCache &traj_cache, int &path_stop_index);

/***********************************************************************************************/
/*
Function:   getNorm
Summary:    get norm of vector 3*1 or 4*1
Input:      vector 3*1 or 4*1
Output:     norm of the vector
Stack:      NULL
*/
inline double getVector3Norm(double* vector);
inline double getVector4Norm(double* vector);

/*
Function: updateTransMatrix44
Summary: get transformation matrix by translation vector{x,y,z} and rotation vector{a,b,c}
Input:  translation vector in size 3, {X,Y,Z}; rotation vector in size 3, {A/Z axis,B/Y axis,C/X axis}
Output:  transformation matrix in size 4*4
Stack:  S_TransMatrix
*/
void updateTransMatrix44(double* rot_vector, double* trans_vector);

/*
Function:   updateHomoTransMatrix44
Summary:    update homogeneous transformation matrix from joint i-1 to joint i
Input:      target_joint_index equals to i, 
            target_joint_q is the angle of target joint in rad
Output:     homogeneous transformation matrix in 4*4 size
Stack:      S_HomoTransMatrix
*/
void updateHomoTransMatrix44(int target_joint_index, double target_joint_q);

/*
Function:   getHomoTransMatrix44
Summary:    get homogeneous transformation matrix from joint i-1 to joint i
Input:      target_joint_index equals to i, 
            target_joint_q is the angle of target joint in rad            
Output:     r is the result homogeneous transformation matrix
Stack:      NULL
*/
void getHomoTransMatrix44(int target_joint_index, double target_joint_q, double* r);

/*
Function:   getMatrix44MultiMatrix44
Summary:    get result matrix of matrix a multiply matrix b
Input:      matrix a and matrix b, all in 4 * 4 size
Output:     matrix r = matrix a * matrix b, in 4 * 4 size
Stack:      NULL
*/
void getMatrix44MultiMatrix44(double* a, double* b, double* r);

/*
Function:   getVector3CrossProduct
Summary:    get result vector of vector a cross product vector b
Input:      vector a and vector b, all in 3*1 size
Output:     vector r = vector a × vector b, in 3*1 size
Stack:      NULL
*/
void getVector3CrossProduct(double* a, double* b, double* r);

/*
Function:   getMatrix33Transpose
Summary:    get transpose of matrix
Input:      matrix in 3*3 size
Output:     matrix_t in 3*3 size
Stack:      NULL
*/
void getMatrix33Transpose(double* matrix, double* matrix_t);

/*
Function:   getMatrix33MultiVector3, getMatrix66MultiVector6
Summary:    get result vector of matrix multiply vector
Input:      matrix in 3*3 or 6*6 size, vector in 3*1 or 6*1 size
Output:     vector_r in 3*1 or 6*1 size
Stack:      NULL
*/
void getMatrix33MultiVector3(double* matrix, double* vector, double* vector_r);
void getMatrix66MultiVector6(double* matrix, double* vector, double* vector_r);

/*
Function:   getRotationMatrix33FromHomoTransMatrix44
Summary:    get rotation matrix from homogeneous transformation matrix
Input:      homogeneous transformation matrix in 4*4 size
Output:     rotation matrix in 3*3 size
Stack:      NULL
*/
void getRotationMatrix33FromHomoTransMatrix44(double* homo_trans, double* rotation);

/*
Function:   getBaseFunction
Summary:    get the value of base function 
Input:      i is the index of vector node to be compute
            k is the power number of base function
            u is the interpolation point from 0~1
Output:     value of base function
Stack:      S_NodeVector is used to stroe the node vector for computation
*/
double getBaseFunction(int i, int k, double u);

/*
Function:   updateTransitionBSpLineCartResult
Summary:    update the B Spline interpolation of transition path
Input:      k is the power number of base function
            start_pos is the start point {X,Y,Z}
            mid_pos is the middle point {X,Y,Z}
            end_pos is the middle point {X,Y,Z}
            result_count is the expected number of result points 
Output:     a list of points on transition path, not include start and end points
Stack:      S_BSpLineResult is used to stroe the result points
*/
void updateTransitionBSpLineCartResult(int k, double* start_pos, double* mid_pos, double* end_pos, int result_count);

/*
Function:   updateTransitionBSpLineJointResult
Summary:    update the B Spline interpolation of transition path
Input:      k is the power number of base function
            start_pos is the start point {J1~J6}
            mid_pos is the middle point {J1~J6}
            end_pos is the middle point {J1~J6}
            result_count is the expected number of result points 
Output:     a list of points on transition path, not include start and end points
Stack:      S_BSpLineResult is used to stroe the result points
*/
void updateTransitionBSpLineJointResult(int k, double* start_joint, double* mid_joint, double* end_joint, int result_count);

/*
Function:   getQuaternsIntersectionAngle
Summary:    return the intersection angle of two quaterns, if the inner product is negative, negation the quatern1
Input:      quatern1: quatern in form of {x,y,z,w}, it might be negatived if inner production is negative
            quatern2: quatern in form of {x,y,z,w}
Output:     intersection angle of two quaterns in rad
Stack:      S_TmpDouble_1
*/
double getQuaternsIntersectionAngle(double* quatern1, double* quatern2);

/*
Function:   getEulerToRotationMatrix33
Summary:    get rotation matrix by euler, rotate by ZYX
Input:      euler is a vector in size 3, in form of {A,B,C}
Output:     rotation is a matrix in size 3*3
Stack:      S_TmpDouble_1~S_TmpDouble_6 are used in the function
*/
void getEulerToRotationMatrix33(double* euler, double* rotation);

/*
Function:   getRotationMatrix33ToEuler
Summary:    get euler by rotation matrix, rotate by ZYX
Input:      rotation is a matrix in size 3*3      
Output:     euler is a vector in size 3, in form of {A,B,C}
Stack:      NULL
*/
void getRotationMatrix33ToEuler(double* rotation, double* euler);

/*
Function:   getQuaternToRotationMatrix33
Summary:    get rotation matrix by quatern
Input:      quatern is a vector in size 4, in form of {x,y,z,w}
Output:     rotation is a matrix in size 3*3
Stack:      S_TmpDouble_1~S_TmpDouble_7 are used in the function
*/
void getQuaternToRotationMatrix33(double* quatern, double* rotation);

/*
Function:   getRotationMatrix33ToQuatern
Summary:    get quatern by rotation matrix
Input:      rotation is a matrix in size 3*3      
Output:     quatern is a vector in size 4, in form of {x,y,z,w}
Stack:      S_TmpDouble_1 is used in the function
*/
void getRotationMatrix33ToQuatern(double* rotation, double* quatern);

/*
Function:   getEulerToQuatern
Summary:    get quatern by euler
Input:      euler is a vector in size 3, {A,B,C}      
Output:     quatern is a vector in size 4, {x,y,z,w}
Stack:      S_TmpDouble_1~S_TmpDouble_6 are used in the function
*/
void getEulerToQuatern(double* euler, double* quatern);

/*
Function:   getQuaternToEuler
Summary:    get euler by quatern
Input:      quatern is a vector in size 4, {x,y,z,w}      
Output:     euler is a vector in size 3, {A,B,C}
Stack:      S_TmpDouble_1~S_TmpDouble_7 are used in the function
*/
void getQuaternToEuler(double* quatern, double* euler);

/*
Function:   getQuaternVector4
Summary:    get quatern state by given necessary information
Input:      start quatern {x,y,z,w} is the start state
            end_quartern {x,y,z,w} is the end state
            angle is the rotation angle from start to end state
            angle_distance_to_start is the angle of expected state and start state
Output:     target_quatern is the expected state, in form of {x,y,z,w}
Stack:      S_TmpDouble_1~S_TmpDouble_3 are used in the function
*/
void getQuaternVector4(double* start_quatern, double* end_quartern, double angle, double angle_distance_to_start, double* target_quatern);

/*
Function:   getModelFk
Summary:    get forward kinematic solution according to model
Input:      model_ptr is the reference model pointer
            joints is the joint angle vector of up to 9 axes
Output:     pos_euler is a vector in size 6, {X,Y,Z,A,B,C}
Stack:      S_TmpMatrix33_1, S_TmpMatrix44_1, S_HomoTransMatrix, S_TmpDouble_1~S_TmpDouble_6,
            are used in the function
*/
void getModelFk(ComplexAxisGroupModel* model_ptr, double* joints, double* pos_euler);

void getVector3(const basic_alg::Point &start, const basic_alg::Point &end, double* vector);
void getMidPoint(const basic_alg::Point &start, const basic_alg::Point &end, double* mid_point);
void getUintVector3(const basic_alg::Point start, const basic_alg::Point end, double* uint_vector);
void getUintVector3(double* vector, double* uint_vector);
/* target_row = target_row - times * ref_row */
inline void doRowOperation(double* target_row, double* ref_row, double times, int row_size);

/*
Function:   updateMatrixA, updateMatrixB, updateEquationSolution
Summary:    compute equation solution, AX=B, support up to 100 equations
Input:      matrix_a is equation matrix A in size n*n, n is up to 100
            matrix_b is equation vector B in size n, n is up to 100
            order is n
            interp_time is the time vector with size n+1, include time duration of the 2 flexible points
            path is the joint angle vector whit size n, not include the 2 flexible points, include start and end points instead
Output:     solution is vector X stored in stack[S_X] in size n, n is up to 100 
Stack:      S_X, S_A, S_B are used in the function
*/
inline void updateMatrixA(double* traj_t, int order);
inline void updateMatrixB(double* traj_p, double* traj_t, double* start_state, double* end_state, int order);
inline void updateEquationSolution(double* matrix_a, double* matrix_b, int order);

/*
Function:   updateTrajPVA
Summary:    compute trajectory according to path P0->PN
Input:      traj_p_base is S_TrajP0 or S_TrajP0_Smooth
            traj_v_base is S_TrajV0 or S_TrajV0_Smooth
            traj_a_base is S_TrajA0 or S_TrajA0_Smooth
            traj_pva_size is the size of traj pva
            traj_t_base is S_TrajT or S_TrajT_Smooth
            traj_t_size is the size of traj t
            traj_j_base is S_TrajJ
            start_state is S_StartPointState0 or S_OutPointState0, in express {P, V, A}
            end_state is S_EndPointState0 or S_InPointState0, in express {P, V, A}
Output:     trajectory {P, V, A} for path according to interp_interval
Stack:      read: S_TrajT, S_TrajP or S_TrajT_Smooth, S_TrajP_Smooth
            write: S_TrajV, S_TrajA or S_TrajV_Smooth, S_TrajA_Smooth
*/
void updateTrajPVA(int traj_p_address, int traj_v_address, int traj_a_address, int traj_pva_size, 
                   int traj_j_address, double* traj_t_base, int traj_t_size, int start_state_address, int end_state_address);


/***********************************************************************************************/
void initStack(int link_num, double joint_vel_max[6]);
void fkToTraj(fst_mc::TrajectoryCache &traj_cache);

inline void getMoveLPathVector(const basic_alg::Point& start_point, const basic_alg::Point& end_point, double* path_vector, double& path_length);
inline double getPointsDistance(const basic_alg::Point& point1, const basic_alg::Point& point2);
inline void getMoveLPathPoint(const basic_alg::Point& start_point, double* path_vector, double distance, basic_alg::Point& target_point);
inline void getMoveEulerToQuatern(const basic_alg::Euler& euler, double* quatern);
inline void getQuaternToQuaternVector4(const basic_alg::Quaternion quatern, double* quatern_vector);
inline void getMovePointToVector3(const basic_alg::Point& point, double* pos_vector);
inline void getQuaternPoint(double* start_quatern, double* end_quartern, double angle, double angle_distance_to_start, basic_alg::Quaternion& target_quatern);
inline void packPoseByPointAndQuatern(basic_alg::Point point, double quatern[4], basic_alg::PoseQuaternion& pose);
inline void packPathBlockType(fst_mc::PointType point_type, fst_mc::MotionType motion_type, fst_mc::PathBlock& path_block);

inline void getMoveCircleCenterAngle(const basic_alg::PoseEuler &start, const fst_mc::MotionTarget &end, 
    double &angle, basic_alg::Point &circle_center_position, double &circle_radius, double* cross_vector);
inline void getCirclePoint(double &circle_radius, double &angle, double* n_vector, double* o_vector,
    basic_alg::Point &circle_center_point, basic_alg::Point &circle_point);

inline void getCircleCenterAngle(const basic_alg::PoseEuler &start, const fst_mc::MotionTarget &end, double &angle);

//dealing with uf, tf
inline ErrorCode convertJointToCartByUserFrame(const basic_alg::Joint &joint, int user_frame_id, int tool_frame_id, basic_alg::PoseEuler &pose);
inline ErrorCode convertCartToJointByUserFrame(const basic_alg::PoseEuler &pose, const basic_alg::Joint &ref_joint, 
                                               int user_frame_id, int tool_frame_id, basic_alg::Joint &joint);


inline void updateTrajPSingleItem(int traj_p_address, const basic_alg::Joint& joint);
inline void getTrajPFromPathStart2End(const fst_mc::PathCache& path_cache, double traj_piece_ideal_start2end, 
                                             int* traj_path_cache_index, int& traj_pva_out_index, int& traj_pva_size);
inline void getTrajPFromPathStart2Out2End(const fst_mc::PathCache& path_cache, double traj_piece_ideal_start2end, 
                                                   int* traj_path_cache_index, int& traj_pva_out_index, int& traj_pva_size);
inline void getTrajPFromPathIn2End(const fst_mc::PathCache& path_cache, double traj_piece_ideal_in2end, int traj_pva_in_index, 
                                          int* traj_path_cache_index_in2end, int& traj_pva_out_index, int& traj_pva_size_via2end);
inline void getTrajPFromPathIn2Out2End(const fst_mc::PathCache& path_cache, double traj_piece_ideal_in2end, int traj_pva_in_index, 
                                               int* traj_path_cache_index_in2end, int& traj_pva_out_index, int& traj_pva_size_via2end);
inline void getTrajPFromPathOut2In(const fst_mc::PathCache& path_cache, double traj_piece_ideal_out2in, 
                                          int* traj_path_cache_index_out2in, int& traj_pva_size_out2in);

inline void updateMovLTrajP(const fst_mc::PathCache& path_cache, int* traj_path_cache_index, int& traj_pva_out_index, int& traj_pva_size);
inline void updateMovJTrajP(const fst_mc::PathCache& path_cache, int* traj_path_cache_index, int& traj_pva_out_index, int& traj_pva_size);
inline void updateMovCTrajP(const fst_mc::PathCache& path_cache, int* traj_path_cache_index, int& traj_pva_out_index, int& traj_pva_size);
inline bool updateMovLVia2InTrajP(const fst_mc::PathCache& path_cache, const fst_mc::MotionTarget& via, int& traj_pva_in_index);
inline bool updateMovJVia2InTrajP(const fst_mc::PathCache& path_cache, const basic_alg::Joint &start, const fst_mc::MotionTarget& via, int& traj_pva_in_index);
inline bool updateMovCVia2InTrajP(const fst_mc::PathCache& path_cache, const fst_mc::MotionTarget& via, int& traj_pva_in_index);
inline void updateMovLIn2EndTrajP(const fst_mc::PathCache& path_cache, int traj_pva_in_index, 
                                        int* traj_path_cache_index_in2end, int& traj_pva_out_index, int& traj_pva_size_via2end);
inline void updateMovJIn2EndTrajP(const fst_mc::PathCache& path_cache, int traj_pva_in_index, 
                                        int* traj_path_cache_index_in2end, int& traj_pva_out_index, int& traj_pva_size_via2end);
inline void updateMovCIn2EndTrajP(const fst_mc::PathCache& path_cache, int traj_pva_in_index, 
                                        int* traj_path_cache_index_in2end, int& traj_pva_out_index, int& traj_pva_size_via2end);

inline bool canBePause(const fst_mc::PathCache &path_cache, const fst_mc::JointState &stop_state, const int &path_stop_index, const int &left_path_number, 
    int &path_end_index);

inline bool canBePauseStartBetweenIn2out(const fst_mc::PathCache &path_cache, const fst_mc::JointState &stop_state, const int &path_stop_index, const int &left_path_number, 
    int &path_end_index);

inline void updatePauseTrajT(const fst_mc::JointState &start_state, int &traj_pva_size, int &traj_t_size);

inline void updatePauseMovLTrajP(const fst_mc::PathCache& path_cache, int* traj_path_cache_index, 
    int& traj_pva_size, int &path_stop_index,int &path_end_index);

inline void updatePauseMovCTrajP(const fst_mc::PathCache& path_cache, int* traj_path_cache_index,int& traj_pva_size, 
    int &path_stop_index, int &path_end_index);

inline void updatePauseMovJTrajP(const fst_mc::PathCache& path_cache, int* traj_path_cache_index, int& traj_pva_size,
     int &path_stop_index,int &path_end_index);

inline void updateMovLTrajT(const fst_mc::PathCache& path_cache, double cmd_vel,
                                int* traj_path_cache_index, int traj_pva_out_index, int traj_pva_size,
                                int& traj_t_size);
inline void updateMovJTrajT(const fst_mc::PathCache& path_cache, double cmd_vel, 
                                int* traj_path_cache_index, int traj_pva_out_index, int traj_pva_size, 
                                int& traj_t_size);
inline void updateMovCTrajT(const fst_mc::PathCache& path_cache, double cmd_vel, 
                                int* traj_path_cache_index, int traj_pva_out_index, int traj_pva_size, 
                                int& traj_t_size);
inline bool updateMovLVia2EndTrajT(const fst_mc::PathCache& path_cache, const fst_mc::MotionTarget& via, double cmd_vel,
                                int* traj_path_cache_index_in2end, int traj_pva_in_index, int traj_pva_out_index, int traj_pva_size_via2end,
                                int& traj_t_size);
inline bool updateMovJVia2EndTrajT(const fst_mc::PathCache& path_cache,const basic_alg::Joint &start, const fst_mc::MotionTarget& via, double cmd_vel,
                                int* traj_path_cache_index_in2end, int traj_pva_in_index, int traj_pva_out_index, int traj_pva_size_via2end,
                                int& traj_t_size);
inline void updateMovCVia2EndTrajT(const fst_mc::PathCache& path_cache, const fst_mc::MotionTarget& via, double cmd_vel,
                                int* traj_path_cache_index_in2end, int traj_pva_in_index, int traj_pva_out_index, int traj_pva_size_via2end,
                                int& traj_t_size);

inline void updateSmoothOut2InTrajP(const fst_mc::PathCache& path_cache, const fst_mc::MotionTarget& via, const basic_alg::Joint start, int* traj_path_cache_index_out2in, int& traj_pva_size_out2in);
inline void updateSmoothOut2InTrajT(const fst_mc::PathCache& path_cache, const fst_mc::MotionTarget& via, const basic_alg::Joint start, double cmd_vel, 
                                           int* traj_path_cache_index_out2in, int traj_pva_size_out2in, 
                                           int& traj_t_size_out2in);

inline void getJerkStart(double* traj_p_base, int traj_pva_size, double* traj_t_base, int traj_t_size, double* start_state, double a2,
                             double& jerk1, double& jerk2);
inline void getJerkEnd(double* traj_p_base, int traj_pva_size, double* traj_t_base, int traj_t_size, double* end_state, double an_1,
                          double& jerkn_1, double& jerkn);
inline void updateTrajVA(double* traj_p_base, double* traj_v_base, double* traj_a_base, int traj_pva_size, 
                             double* traj_j_base, double* traj_t_base, int traj_t_size,
                             double* start_state, double* end_state);
inline void updateConstraintJoint(int traj_p_address, int traj_v_address, int traj_pva_size);
inline void updateTrajPieceA(int traj_a_address, int traj_pva_size, double acc_ratio);
inline void updateTrajPieceV(int traj_v_address, int traj_a_address, int traj_pva_size, int traj_t_address, double vel_ratio);
inline double getMaxOfAllAxes(int traj_piece_address);
inline void updateTrajPieceRescaleFactor(int traj_piece_size);
inline void updateTrajTByPieceRescaleFactor(int traj_t_address, int traj_t_size);
inline void updateOutAndInPointState(const fst_mc::JointState& out_state, int traj_pva_in_index);
inline void updatePausePointState(const fst_mc::JointState& pause_state);
inline void updateEndPointStateForPause(int traj_pva_end_index);
inline bool isRescaleNeeded(int traj_piece_size);
inline void updateTrajCoeff(int traj_p_address, int traj_v_address, int traj_a_address, int traj_pva_size, 
                                int traj_t_address, int traj_t_size, int traj_j_address, int traj_coeff_address);
inline void packTrajCache(int* traj_path_cache_index, int traj_pva_out_index, int traj_pva_size, 
                          int traj_coeff_address, int traj_t_address, int traj_t_size, fst_mc::TrajectoryCache& traj_cache);
inline void packPauseTrajCache(int* traj_path_cache_index, int traj_pva_size, 
                          int traj_coeff_address, int traj_t_address, int traj_t_size, fst_mc::TrajectoryCache& traj_cache);
inline void packTrajCacheSmooth(int* traj_path_cache_index_out2in, int traj_pva_size_out2in, int traj_coeff_address_out2in, int traj_t_address_out2in, int traj_t_size_out2in, 
                                      int* traj_path_cache_index_in2end, int traj_pva_size_via2end, int traj_coeff_address_via2end, int traj_t_address_via2end, int traj_t_size_via2end,
                                      int traj_pva_in_index, int traj_pva_out_index,
                                      fst_mc::TrajectoryCache& traj_cache);

void printTraj(fst_mc::TrajectoryCache &traj_cache, int index, double time_step, int end_segment);


#endif
