/**********************************************************************
    Copyright:	Foresight-Robotics
    File:   	fst_datatype.h
    Author:	Feng Yun
    Data:	Aug. 1 2016
    Modify:	Jun. 1 2017
    Description:Define all datatypes used in ArmGroup
**********************************************************************/

#ifndef FST_DATATYPE_H
#define FST_DATATYPE_H

#define                 PI      3.1415926535897932384626433832795
#define       NUM_OF_JOINT      9
#define  AXIS_IN_ALGORITHM      6

namespace fst_controller {

class MotionCommand;

typedef unsigned int    Tick;
typedef unsigned int    Size;
typedef double          MotionTime;
typedef double          Angle;
typedef double          Omega;
typedef double          Alpha;



/* Define a group of values in joint space */
struct _Joint {
    Angle   j1;
    Angle   j2;
    Angle   j3;
    Angle   j4;
    Angle   j5;
    Angle   j6;
    Angle   j7;
    Angle   j8;
    Angle   j9;
};

// Define a angular velocity structure
struct _JointOmega {
    Omega   j1;
    Omega   j2;
    Omega   j3;
    Omega   j4;
    Omega   j5;
    Omega   j6;
    Omega   j7;
    Omega   j8;
    Omega   j9;
};

struct _JointAlpha {
    Alpha   j1;
    Alpha   j2;
    Alpha   j3;
    Alpha   j4;
    Alpha   j5;
    Alpha   j6;
    Alpha   j7;
    Alpha   j8;
    Alpha   j9;
};

struct _JointInertia {
    Alpha   j1;
    Alpha   j2;
    Alpha   j3;
    Alpha   j4;
    Alpha   j5;
    Alpha   j6;
    Alpha   j7;
    Alpha   j8;
    Alpha   j9;
};

struct _JointState
{
    Angle   joint[NUM_OF_JOINT];
    Omega   omega[NUM_OF_JOINT];
    Alpha   alpha[NUM_OF_JOINT];
};

struct _JointGroup {
    Tick        stamp;
    _JointState  j1;
    _JointState  j2;
    _JointState  j3;
    _JointState  j4;
    _JointState  j5;
    _JointState  j6;
    _JointState  j7;
    _JointState  j8;
    _JointState  j9;
};


struct _Spline {
    double  coeff[6];
};

struct _JointSegment {
    MotionTime  time_from_start;
    MotionTime  duration;
    _JointGroup  start;
    _JointGroup  end;
    _Spline      coeff[NUM_OF_JOINT];
};

// Define a point in cartesian space
struct _Point {
    double x;
    double y;
    double z;
};

// Define a _Quaternion
struct _Quaternion {
    double x;
    double y;
    double z;
    double w;
};

// Define an euler angle
struct _Euler {
    double a;
    double b;
    double c;
};

// Define a pose in cartesian space
struct _Pose {
    _Point       position;
    _Quaternion  orientation;
};

// Define another pose type using euler angle
struct _PoseEuler {
    _Point position;
    _Euler orientation;
};




// Define the home position, upper limit and lower limit of a joint
struct _JointLimit {
    double home;
    double upper;
    double lower;

    //double max_omega;
    //double max_alpha;
};

// Define all joint limits of a robot
struct _JointConstraint {
    _JointLimit j1;
    _JointLimit j2;
    _JointLimit j3;
    _JointLimit j4;
    _JointLimit j5;
    _JointLimit j6;
    _JointLimit j7;
    _JointLimit j8;
    _JointLimit j9;
};

// Define a transformation type
//struct Transformation {
//    Point position;
//    Euler orientation;
//};
typedef _PoseEuler Transformation;

// point level: 0x0 -> middle point
//              0x1 -> start  point
//              0x2 -> ending point
enum PointLevel {
    POINT_MIDDLE    = 0x0,
    POINT_START     = 0x1,
    POINT_ENDING    = 0x2,
    POINT_LAST      = 0x3,
    POINT_FIRST     = 0x4,
};

const unsigned int POINT_LEVEL_MASK  = 0x00000003;

// A flag indicating the type of a motion
enum MotionType {
    MOTION_NONE,
    MOTION_JOINT,
    MOTION_LINE,
    MOTION_CIRCLE,
};

// A flag indicating the smooth type between two motions
enum SmoothType {
    SMOOTH_NONE,
    SMOOTH_2J,
    SMOOTH_2L,
    SMOOTH_2C,
    SMOOTH_UNKNOWN,
};

enum SmoothMode
{
    MODE_VELOCITY,
    MODE_DISTANCE,
};

// PathPoint structure defines what does path-plan should give out.
struct PathPoint {
    // Command type, joint command or cartesian command*/
    MotionType  type;

    int         id;

    // point stamp
    Tick        stamp;

    // the point generated from which command
    MotionCommand *source;

    // value in cartesian space or joint space
    union {
        _Pose    pose;
        _Joint   joint;
    };
};

// ControlPoint structure used in trajectory-create.
struct ControlPoint {
    // point from path plan
    PathPoint   path_point;

    // time from start
    // time < 0 means this point has not been converted to a trajectory point
    MotionTime  time_from_start;

    // time duration from prev point to this point
    // duration < 0 means this point has not been converted to a trajectory point
    // command_duration = cycle_step / v_command
    // duration = actual dutation
    MotionTime  command_duration;
    MotionTime  forward_duration;
    MotionTime  backward_duration;

    
    // point is what trajectory-create should give out
    _JointState  forward_point;
    _JointState  backward_point;

    double  coeff[AXIS_IN_ALGORITHM][6];
    double  inertia[NUM_OF_JOINT];

    //bool    alpha_valid;
    //double  alpha_upper[6];
    //double  alpha_lower[6];
};

struct TrajSegment
{
    int         id;
    MotionTime  time_from_start;
    MotionTime  duration;

    //bool    quadratic;
    //bool    quintic;
    //bool    t_curve;

    double  coeff[AXIS_IN_ALGORITHM][6];
    double  inertia[NUM_OF_JOINT];
};

// Define a point structure used in joint trajectory fifo
struct JointPoint
{
    // time stamp
    int stamp;
    // point level, start/middle/ending point
    PointLevel  level;
    // the point generated from which command
    MotionCommand *source;
    // joint values of this point
    _Joint       joint;
    // angular velocity of this point
    _JointOmega  omega;
};



////////////////////////////////////////////////////////////////////////////////////////////
struct JointOutput {
    int id;
    PointLevel  level;

    _Joint           joint;
    _JointOmega      omega;
    _JointAlpha      alpha;
    _JointInertia    inertia;
};

struct JointOut {
    int id;
    PointLevel  level;

    double joint[NUM_OF_JOINT];
    double omega[NUM_OF_JOINT];
    double alpha[NUM_OF_JOINT];
    double inertia[NUM_OF_JOINT];
};

// Define a group of coordinate offset
struct CoordinateOffset {
    double alpha;
    double a;
    double d;
    double theta;
};

// Define a group of DH parameter
struct DHGroup {
    CoordinateOffset j1;
    CoordinateOffset j2;
    CoordinateOffset j3;
    CoordinateOffset j4;
    CoordinateOffset j5;
    CoordinateOffset j6;
};

// target poses of circle motion
struct CircleTarget {
    _PoseEuler pose1;
    _PoseEuler pose2;
};

// target structure used in motion command
struct MotionTarget {
    MotionType  type;

    // 0.0 - 1.0
    double cnt;

    // percent velocity in move Joint, range: 0.0-1.0
    // linear velocity in move cartesian, range 0.0-MAC_VEL
    // velocity < 0 means using default velocity
    double vel;

    int user_frame_id_;
    int tool_frame_id_;

    union {
        _PoseEuler       pose_target;
        _Joint           joint_target;
        CircleTarget    circle_target;
    };
};

// initial state of a line motion
struct LInitial {
    _Pose    pose_prv;
    _Pose    pose;
    double  velocity;
    double  vu;
};

// initial state of a circle motion
struct CInitial {
    _Pose    pose;
    double  velocity;
};

struct Arc {
    double ArcRot[4][4];
    double radius;
    double omega;
};

struct orienttrans {
    double q0[4];
    double q1[4];
    double s1[4];
    double s2[4];
    int modeflag;
    double theta_q;
    double theta_s;
};

struct ElementSmooth {
    double time[8];
    double Polynominal[7][4];
};

struct Parabola {
    double P0[3];
    double v0[3];
    double a0[3];
    double P2[3];
    double v2[3];
    double a2[3];
    double T;
    double lamda[3];
};

struct LCommand
{
    _Pose    pose;
    double  velocity;
    union
    {
        int     cnt;
        double  smooth_radius;
    };
    
};

struct CCommand
{
    _Pose    pose1;
    _Pose    pose2;
    double  velocity;
    union
    {
        int     cnt;
        double  smooth_radius;
    };

};

struct JCommand
{
    _Joint       joints;
    double      v_percentage;
    union
    {
        int         cnt;
        double      smooth_radius;
    };

};

struct VelCartesian
{
    double vx;
    double vy;
    double vz;
    double omegax;
    double omegay;
    double omegaz;
};

struct PoseVel
{
    _Pose pose;
    VelCartesian velocity;
};

enum ManualFrame
{
    JOINT,
    WORLD,
    USER,
    TOOL,
};

enum ManualMode
{
    STEP,
    CONTINUOUS,
    APOINT,
};

enum ManualDirection
{
    STANDBY  = 0,
    INCREASE = 1,
    DECREASE = 2,
};

struct ManualCoeff
{
    double duration_1;
    double duration_2;
    double duration_3;
    double alpha_1;
    double alpha_3;
};





}   // namespace fst_controller



#endif  // #ifndef FST_DATATYPE_H
