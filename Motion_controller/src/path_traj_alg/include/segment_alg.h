#ifndef SEGMENT_ALG_H
#define SEGMENT_ALG_H

#include <math.h>
#include <cstring>
#include <string>
#include "base_datatype.h"
#include "common_enum.h"
#include "motion_control_datatype.h"
#include "arm_kinematics.h"
#include "dynamics_interface.h"

#define DOUBLE_ACCURACY 1e-6
#define SQRT_DOUBLE_ACCURACY 1e-12

typedef ErrorCode (fst_mc::BaseKinematics::*ComputeIKFunc)(const fst_mc::Pose      &pose, const fst_mc::Joint &ref, fst_mc::Joint &res);
typedef bool (fst_algorithm::DynamicsInterface::*ComputeDynamicsFunc)(const float joint[6], const float omega[6], float alpha_max[2][6]);

typedef struct
{
    int accuracy_cartesian_factor;  // define the number of trajectory points on per 100mm cartesian path
    int accuracy_joint_factor;      // define the number of trajectory points on per PI joint path
    int max_traj_points_num;
    double path_interval;       // path points interval in mm
    double joint_interval;      // joint angle interval in rad
    double angle_interval;    // quatern angle interval in rad
    double angle_valve;       // valve for slerp or linear interpolation for quatern angle
    double conservative_acc;    // conservative accerl
    double jerk_ratio;  // jerk constraints ratio, [0,1]
    bool is_fake_dynamics;
    double time_factor_1;   // the first piece of time vector 
    double time_factor_2;   // the second
    double time_factor_3;   // the second last
    double time_factor_4;   // the last
    double max_rescale_factor;  
    fst_mc::BaseKinematics* kinematics_ptr;
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
    S_ConstraintJointJerkMax = 45,   // vector in size 9
    S_BSplineNodeVector = 54,    // vector in size 6
    S_PathCountFactorCartesian = 60,    // double
    S_PathCountFactorJoint = 61,    // double

    // basic matrix operation
    S_TransMatrix = 200,  // matrix[4][4]
    S_HomoTransMatrix = 216, // matrix[4][4]

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
    S_PathIndexStep = 4460, // path index interval between two traj points   
    S_PathIndexStepStart2Out = 4461,
    S_PathIndexStepOut2End = 4462,
    S_PathIndexStepOut2In = 4463,
    S_PathIndexStepIn2Out = 4464,
    S_PathIndexStepIn2End = 4465,
    S_TrajRescaleFactor = 4470,  // final rescale factor, double
    
    S_Path0 = 4475,    // traj points selected from path points, up to 20 points, not include the 2 flexible points
    S_Path1 = 4500,
    S_Path2 = 4525,
    S_Path3 = 4550,
    S_Path4 = 4575,
    S_Path5 = 4600,
    S_Path6 = 4625,
    S_Path7 = 4650,
    S_Path8 = 4675,        
    S_TrajT = 4700,    // traj initial time vector, expressed in time duration of two adjacent traj points, 
                       // up to 21 points, include the 2 flexible points
    S_TrajT_Smooth = 4725,                   
    S_TrajRescaleT = 4750,  // recale S_TrajT by S_TrajRescaleFactor, up to 20 + 20 + 1 points
    S_TrajAbsoluteT = 4800, // absolute time compute by S_TrajRescaleT
    
    S_TrajP0 = 4850,    // {P,V,A} of joint0 on traj points, up to 20 + 20 + 2 points, include the 2 flexible points
    S_TrajV0 = 4900,
    S_TrajA0 = 4950,
    S_TrajP1 = 5000,
    S_TrajV1 = 5050,
    S_TrajA1 = 5100,
    S_TrajP2 = 5150,
    S_TrajV2 = 5200,
    S_TrajA2 = 5250,
    S_TrajP3 = 5300,
    S_TrajV3 = 5350,
    S_TrajA3 = 5400,
    S_TrajP4 = 5450,
    S_TrajV4 = 5500,
    S_TrajA4 = 5550,
    S_TrajP5 = 5600,
    S_TrajV5 = 5650,
    S_TrajA5 = 5700,
    S_TrajP6 = 5750,
    S_TrajV6 = 5800,
    S_TrajA6 = 5850,
    S_TrajP7 = 5900,
    S_TrajV7 = 5950,
    S_TrajA7 = 6000,
    S_TrajP8 = 6050,
    S_TrajV8 = 6100,
    S_TrajA8 = 6150,    

    S_OutPointState0 = 6200, // vector in size 3
    S_OutPointState1 = 6203,
    S_OutPointState2 = 6206,
    S_OutPointState3 = 6209,
    S_OutPointState4 = 6212,
    S_OutPointState5 = 6215,
    S_OutPointState6 = 6218,
    S_OutPointState7 = 6221,
    S_OutPointState8 = 6224,

    S_InPointState0 = 6250, // vector in size 3
    S_InPointState1 = 6253,
    S_InPointState2 = 6256,
    S_InPointState3 = 6259,
    S_InPointState4 = 6262,
    S_InPointState5 = 6265,
    S_InPointState6 = 6268,
    S_InPointState7 = 6271,
    S_InPointState8 = 6274,

    // compute robot dynamics constraints, put result in ConstaintJointPos/NegAX
    S_ConstraintJointPosA0 = 6500, // positive max acc of joint0, from traj point(2) ~ point(n), up to 21 in size
    S_ConstraintJointNegA0 = 6525,
    S_ConstraintJointPosA1 = 6550,
    S_ConstraintJointNegA1 = 6575,    
    S_ConstraintJointPosA2 = 6600,
    S_ConstraintJointNegA2 = 6625, 
    S_ConstraintJointPosA3 = 6650,
    S_ConstraintJointNegA3 = 6675, 
    S_ConstraintJointPosA4 = 6700,
    S_ConstraintJointNegA4 = 6725, 
    S_ConstraintJointPosA5 = 6750,
    S_ConstraintJointNegA5 = 6775,
    S_ConstraintJointPosA6 = 6800,
    S_ConstraintJointNegA6 = 6825,
    S_ConstraintJointPosA7 = 6850,
    S_ConstraintJointNegA7 = 6875,
    S_ConstraintJointPosA8 = 6900,
    S_ConstraintJointNegA8 = 6925,

    // traj piece jerk rescale factor
    S_TrajPieceJ0 = 7000,   // up to 21 in size
    S_TrajPieceJ1 = 7025,
    S_TrajPieceJ2 = 7050,
    S_TrajPieceJ3 = 7075,
    S_TrajPieceJ4 = 7100,
    S_TrajPieceJ5 = 7125,
    S_TrajPieceJ6 = 7150,
    S_TrajPieceJ7 = 7175,
    S_TrajPieceJ8 = 7200,
    // traj piece acc rescale factor
    S_TrajPieceA0 = 7225,
    S_TrajPieceA1 = 7250,
    S_TrajPieceA2 = 7275,
    S_TrajPieceA3 = 7300,
    S_TrajPieceA4 = 7325,
    S_TrajPieceA5 = 7350,
    S_TrajPieceA6 = 7375,
    S_TrajPieceA7 = 7400,
    S_TrajPieceA8 = 7425,
    // traj piece vel rescale factor
    S_TrajPieceV0 = 7450,
    S_TrajPieceV1 = 7475,
    S_TrajPieceV2 = 7500,
    S_TrajPieceV3 = 7525,
    S_TrajPieceV4 = 7550,
    S_TrajPieceV5 = 7575,
    S_TrajPieceV6 = 7600,
    S_TrajPieceV7 = 7625,
    S_TrajPieceV8 = 7650,
      
    // coeff of p = A5*t^5 + A4*t^4 + A3*t^3 + A2*t^2 + A1*t + A0
    S_TrajCoeffJ0A5 = 7700,
    S_TrajCoeffJ0A4 = 7750,
    S_TrajCoeffJ0A3 = 7800,    
    S_TrajCoeffJ0A2 = 7850,
    S_TrajCoeffJ0A1 = 7900,
    S_TrajCoeffJ0A0 = 7950,

    S_TrajCoeffJ1A5 = 8000,
    S_TrajCoeffJ1A4 = 8050,
    S_TrajCoeffJ1A3 = 8100,    
    S_TrajCoeffJ1A2 = 8150,
    S_TrajCoeffJ1A1 = 8200,
    S_TrajCoeffJ1A0 = 8250,

    S_TrajCoeffJ2A5 = 8300,
    S_TrajCoeffJ2A4 = 8350,
    S_TrajCoeffJ2A3 = 8400,    
    S_TrajCoeffJ2A2 = 8450,
    S_TrajCoeffJ2A1 = 8500,
    S_TrajCoeffJ2A0 = 8550,

    S_TrajCoeffJ3A5 = 8600,
    S_TrajCoeffJ3A4 = 8650,
    S_TrajCoeffJ3A3 = 8700,    
    S_TrajCoeffJ3A2 = 8750,
    S_TrajCoeffJ3A1 = 8800,
    S_TrajCoeffJ3A0 = 8850,

    S_TrajCoeffJ4A5 = 8900,
    S_TrajCoeffJ4A4 = 8950,
    S_TrajCoeffJ4A3 = 9000,    
    S_TrajCoeffJ4A2 = 9050,
    S_TrajCoeffJ4A1 = 9100,
    S_TrajCoeffJ4A0 = 9150,

    S_TrajCoeffJ5A5 = 9200,
    S_TrajCoeffJ5A4 = 9250,
    S_TrajCoeffJ5A3 = 9300,    
    S_TrajCoeffJ5A2 = 9350,
    S_TrajCoeffJ5A1 = 9400,
    S_TrajCoeffJ5A0 = 9450,

    S_TrajCoeffJ6A5 = 9500,
    S_TrajCoeffJ6A4 = 9550,
    S_TrajCoeffJ6A3 = 9600,    
    S_TrajCoeffJ6A2 = 9650,
    S_TrajCoeffJ6A1 = 9700,
    S_TrajCoeffJ6A0 = 9750,

    S_TrajCoeffJ7A5 = 9800,
    S_TrajCoeffJ7A4 = 9850,
    S_TrajCoeffJ7A3 = 9900,    
    S_TrajCoeffJ7A2 = 9950,
    S_TrajCoeffJ7A1 = 10000,
    S_TrajCoeffJ7A0 = 10050,

    S_TrajCoeffJ8A5 = 10100,
    S_TrajCoeffJ8A4 = 10150,
    S_TrajCoeffJ8A3 = 10200,    
    S_TrajCoeffJ8A2 = 10250,
    S_TrajCoeffJ8A1 = 10300,
    S_TrajCoeffJ8A0 = 10350,

}StackIndex;

extern ComplexAxisGroupModel model;
extern double stack[12000];

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
Function:   updateTransitionBSpLineResult
Summary:    update the B Spline interpolation of transition path
Input:      k is the power number of base function
            start_pos is the start point {X,Y,Z}
            mid_pos is the middle point {X,Y,Z}
            end_pos is the middle point {X,Y,Z}
            result_count is the expected number of result points 
Output:     a list of points on transition path, not include start and end points
Stack:      S_BSpLineResult is used to stroe the result points
*/
void updateTransitionBSpLineResult(int k, double* start_pos, double* mid_pos, double* end_pos, int result_count);

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
inline void updateMatrixA(double* interp_time, int order);
inline void updateMatrixB(double* path, double* interp_time, double* init_state, double* end_state, int order);
inline void updateEquationSolution(double* matrix_a, double* matrix_b, int order);

/*
Function:   updateTrajPVA
Summary:    compute trajectory according to path P0->PN
Input:      path is the original path point, not include the 2 flexible points, include start and end points, size is noted as n
            interp_time is the time vector include the 2 flexible points, size is n+1
            init_state is the {P,V,A} of start point
            end_state is the {P,V,A} of end point
            order should be equal to the size of path
            traj_base should be S_TrajP0
Output:     trajectory {P, V, A} for path according to interp_interval
Stack:      read: S_TrajT, S_Path0~S_Path8
            write: S_TrajP, S_TrajV, S_TrajA
*/
void updateTrajPVA(int path_base, double* init_state, double* end_state, int order, int traj_base, int t_base);

/***********************************************************************************************/


void initComplexAxisGroupModel();
void initStack(ComplexAxisGroupModel* model_ptr);
void initSegmentAlgParam(fst_mc::BaseKinematics* kinematics_ptr, fst_algorithm::DynamicsInterface* dynamics_ptr);

ErrorCode planPathJoint(const fst_mc::Joint &start, const fst_mc::MotionTarget &target, fst_mc::PathCache &path_cache);
ErrorCode planPathLine(const fst_mc::PoseEuler &start, const fst_mc::MotionTarget &target, fst_mc::PathCache &path_cache);
ErrorCode planPathCircle(const fst_mc::PoseEuler &start, const fst_mc::MotionTarget &target, fst_mc::PathCache &path_cache);

ErrorCode planPathSmoothJoint(const fst_mc::Joint &start, const fst_mc::MotionTarget &via, const fst_mc::MotionTarget &target, fst_mc::PathCache &path_cache);
ErrorCode planPathSmoothLine(const fst_mc::PoseEuler &start, const fst_mc::MotionTarget &via, const fst_mc::MotionTarget &target, fst_mc::PathCache &path_cache);
ErrorCode planPathSmoothCircle(const fst_mc::PoseEuler &start, const fst_mc::MotionTarget &via, const fst_mc::MotionTarget &target, fst_mc::PathCache &path_cache);

ErrorCode planTrajectory(const fst_mc::PathCache &path_cache, const fst_mc::JointState &start_state, double vel_ratio, double acc_ratio, fst_mc::TrajectoryCache &traj_cache);
ErrorCode planTrajectorySmooth(const fst_mc::PathCache &path_cache, const fst_mc::JointState &start_state, const fst_mc::MotionTarget &via, double vel_ratio, double acc_ratio, fst_mc::TrajectoryCache &traj_cache);
ErrorCode planPauseTrajectory(const fst_mc::PathCache &path_cache, const fst_mc::JointState &start_state, double acc_ratio, fst_mc::TrajectoryCache &traj_cache, int &path_stop_index);


void getMoveLPathVector(const fst_mc::Point& start_point, const fst_mc::Point& end_point, double* path_vector, double& path_length);
double getPointsDistance(const fst_mc::Point& point1, const fst_mc::Point& point2);
void getMoveLPathPoint(const fst_mc::Point& start_point, double* path_vector, double distance, fst_mc::Point& target_point);
void getMoveEulerToQuatern(const fst_mc::Euler& euler, double* quatern);
void getMovePointToVector3(const fst_mc::Point& point, double* pos_vector);
void getQuaternPoint(double* start_quatern, double* end_quartern, double angle, double angle_distance_to_start, fst_mc::Quaternion& target_quatern);

void generateMoveJPathPoint(const fst_mc::PathCache &path_cache, int* path_index_array, int& path_index_array_size);
void generateMoveJTimeVector(double vel_ratio, int path_size, int& time_vector_size);
void generateMoveLPathPoint(const fst_mc::PathCache &path_cache, const fst_mc::JointState &start_state, 
                                    int* path_index_array, int& path_index_array_size,
                                    int& path_index_array_smooth_out_index, double& path_length_start2out, double& path_length_out2end);
void generateMoveLTimeVector(double vel, int path_size, int path_index_array_smooth_out_index, 
                                    double path_length_start2out, double path_length_out2end, int& time_vector_size);

int computeTrajSmoothInIndex(double length_out2via, double length_via2in);
void computePathIndexStepVia2End(const fst_mc::PathCache &path_cache, const fst_mc::MotionTarget &via, double length_via2in,
                                    int& path_index_in, int& path_index_out, int& path_index_end);
void generatePathPointVia2In(const fst_mc::PathCache &path_cache, const fst_mc::MotionTarget &via, double length_via2in,
                                    double* path_vector_via2in, int path_index_in);
void generatePathPointIn2End(const fst_mc::PathCache &path_cache, int path_index_in, int path_index_out, int path_index_end, int* path_index_array_in2end);
void generatePathVia2EndTimeVector(double vel, const fst_mc::PathCache &path_cache, double length_via2in, 
                                   int path_index_in, int path_index_out, int path_index_end, int& time_vector_size_via2end);
void generatePathPointOut2In(const fst_mc::PathCache &path_cache, int traj_smooth_in_index, int& path_array_size, int* path_index_array_out2in);
void generatePathOut2InTimeVector(double cmd_vel, double length_out2via, double length_via2in, int path_array_size_out2via,int& time_vector_size_out2in);


void generatePieceVectors(int traj_offset, int traj_piece_num, int t_base, double vel_ratio, double acc_ratio);
double getMaxTrajPiece(int base_address, int index);
double getMax(double value0, double value1, double value2, double value3);
void generateRescaleFactorVector(int time_vector_size, double& recale_factor);
void generateRescaleVector(int time_vector_size);
void adjustTrajT(int time_vector_size, double rescale_factor);
void adjustTrajTSmooth(int time_vector_size, double rescale_factor);
void adjustPVA(int traj_index_via, int traj_index_end, double rescale_factor_via2end);
void adjustPVA2(int traj_index_in, double rescale_factor);
void generateRescaleVectorSmooth(int time_vector_size_out2in, int time_vector_size_in2end, int time_vector_size_via2in);
void generateAbsoluteVector(int time_vector_size);
void generateCoeff(int time_vector_size);
void adjustCoeff(int traj_smooth_in_index);
void generateTrajCache(fst_mc::TrajectoryCache &traj_cache, int time_vector_size, int* path_index_array, int path_index_array_size);
void generateTrajCacheSmooth(fst_mc::TrajectoryCache &traj_cache, int time_vector_size, 
                                    int* path_index_array_out2in, int path_index_array_size_out2in,
                                    int* path_index_array_in2end, int path_index_array_size_in2end);
void generateTrajCacheSmoothTest(fst_mc::TrajectoryCache &traj_cache, int time_vector_size, 
                                    int* path_index_array_in2end, int path_index_array_size_in2end);

void printTraj(fst_mc::TrajectoryCache &traj_cache, int index, double time_step);
void printTraj2(fst_mc::TrajectoryCache &traj_cache, int index, double time_step, int end_segment);
void printAllTraj(fst_mc::TrajectoryCache &traj_cache, double time_step);


#endif

