/**********************************************************************
    Copyright:	Foresight-Robotics
    File:   	fst_datatype.h
    Author:	Feng Yun
    Data:	Aug. 1 2016
    Modify:	Aug.23 2016
    Description:Define all datatypes used in ArmGroup
**********************************************************************/

#ifndef FST_DATATYPE_H
#define FST_DATATYPE_H

namespace fst_controller {

/* Define a group of values in joint space */
struct JointValues {
    double j1;
    double j2;
    double j3;
    double j4;
    double j5;
    double j6;
};

/*
// Define a point in joint space 
struct JointPoint {
    int id;
    double velocity;
    JointValues joints;
};
    
// Defines a trajectory in joint space
class Trajectory {
  public:
    std::vector<fst_controller::JointPoint> trajectory;
};
*/

// Define the home position, upper limit and lower limit of a joint
struct JointLimit {
    double home;
    double upper;
    double lower;

    double max_omega;
    double max_alpha;
};

// Define all joint limits of a robot
struct JointConstraints {
    JointLimit j1;
    JointLimit j2;
    JointLimit j3;
    JointLimit j4;
    JointLimit j5;
    JointLimit j6;
};

// Define a point in cartesian space
struct Point {
    double x;
    double y;
    double z;
};

// Define a quaternion
struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};

// Define a pose in cartesian space
struct Pose {
    Point position;
    Quaternion orientation;
};

/*
// Define a pose point in cartesian space
struct PosePoint {
    int id;
    double velocity;
    Pose pose;
};
*/

// Define a kind of euler angle
struct Euler {
    double a;
    double b;
    double c;
};

// Define another pose type using euler angle
struct PoseEuler {
    Point position;
    Euler orientation;
};

// Define a transformation type
struct Transformation {
    Point position;
    Euler orientation;
};

// An enum value to indicate the type of commands
enum CommandType {
    // enum_Type_Joint,
    // enum_Type_Cartesian
    enum_Type_MoveJ,
    enum_Type_MoveL,
    enum_Type_MoveC,
    enum_Type_Other,
};

// An enum value to indicate the type of points
enum PointLevel {
    enum_Level_Middle   = 0,
    enum_Level_Start    = 1,
    enum_Level_Ending   = 2,
};

// Define a pose/joint point structure used in planned path fifo
struct PathPoint {
    // Command ID
    int id;
    // Command type, joint command or cartesian command
    CommandType type;
    // User frame indicator
    int user_frame;
    // Tool frame indicator
    int tool_frame;
    // velocity of the point in cartesian space
    double velocity;
    // coordinate value in cartesian space or joint space
    union {
        Pose pose;
        JointValues joints;
    };
};

struct JointVelocity {
    double j1;
    double j2;
    double j3;
    double j4;
    double j5;
    double j6;
};

// Define a point structure used in joint trajectory fifo
struct JointPoint {
    int id;
    // double velocity;
    JointVelocity omegas;
    JointValues joints;
};

/*
// To record important infomations of a command
struct PointMap {
  public:
    // Command ID
    int id;
    // Command type, joint command or cartesian command
    CommandType type;
    // Number of points generated by the command
    int count;
    // User frame indicator
    int user_frame;
    // Tool frame indicator
    int tool_frame;
};
*/
}   // namespace fst_controller

/*
namespace Error {
enum ErrorCode_ {
    Success                 = 0,
    Undefined_Error         = 1,

    IK_Out_Of_Workspace     = 1001,
    IK_Joint_Out_Of_Limit   = 1002,
    IK_Excessive_Distance   = 1003,
    FK_Out_Of_Joint_Limit   = 1011,
    MoveL_Unsmoothable      = 1021,
    MoveL_Zero_Distance     = 1022,
    MoveJ_Axis_Overshoot    = 1031,
    MoveJ_Axis_Near_Limit   = 1032,

    Fail_Init_Algorithm     = 100000,
    Fail_Load_Constraint    = 100001,
    Fail_Load_Parameters    = 100002,
    Lost_Parameters         = 100003,
    Invalid_Parameters      = 100004,
    Joint_Out_Of_Limit      = 101000,
    Arm_Group_Suspended     = 101001,
    Trajectory_FIFO_Full    = 101002,
    No_Enough_Points        = 101010,
    Cartesian_Path_Exist    = 101020,
    Cmd_Parameter_Invalid   = 101030,
    Cmd_Sequence_Error      = 101031,
};  // enum ErrorCode
}   // namespace Error
typedef Error::ErrorCode_ ErrorCode;
*/

#endif  // #ifndef FST_DATATYPE_H
