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

#define PI 3.1415926535897932384626433832795

namespace fst_controller
{

    /* Define a group of values in joint space */
    struct JointValues
    {
        double j1;
        double j2;
        double j3;
        double j4;
        double j5;
        double j6;
        double j7;
        double j8;
        double j9;
    };



    // Define the home position, upper limit and lower limit of a joint
    struct JointLimit
    {
        double home;
        double upper;
        double lower;

        double max_omega;
        double max_alpha;
    };

    // Define all joint limits of a robot
    struct JointConstraints
    {
        JointLimit j1;
        JointLimit j2;
        JointLimit j3;
        JointLimit j4;
        JointLimit j5;
        JointLimit j6;
    };

    // Define a point in cartesian space
    struct Point
    {
        double x;
        double y;
        double z;
    };

    // Define a quaternion
    struct Quaternion
    {
        double x;
        double y;
        double z;
        double w;
    };

    // Define an euler angle
    struct Euler
    {
        double a;
        double b;
        double c;
    };

    // Define a pose in cartesian space
    struct Pose
    {
        Point position;
        Quaternion orientation;
    };

    // Define another pose type using euler angle
    struct PoseEuler
    {
        Point position;
        Euler orientation;
    };

    // Define a transformation type
    struct Transformation
    {
        Point position;
        Euler orientation;
    };

    enum PointLevel
    {
        POINT_MIDDLE = 0,
        POINT_START = 1,
        POINT_ENDING = 2,
    };

    const unsigned int POINT_LEVEL_MASK = 0x00000003;

    enum MotionType
    {
        MOTION_UNDEFINED,
        MOTION_JOINT,
        MOTION_LINE,
        MOTION_CIRCLE,
    };
    enum SmoothType
    {
        SMOOTH_NONE,
        SMOOTH_J2J,
        SMOOTH_J2L,
        SMOOTH_J2C,
        SMOOTH_L2J,
        SMOOTH_L2L,
        SMOOTH_L2C,
        SMOOTH_C2J,
        SMOOTH_C2L,
        SMOOTH_C2C,
    };

    // Define a pose/joint point structure used in planned path fifo
    struct PathPoint
    {
        // Command ID
        int id;
        // Command type, joint command or cartesian command
        MotionType type;
        // coordinate value in cartesian space or joint space
        union
        {
            Pose pose;
            JointValues joints;
        };
    };

    struct JointOmegas
    {
        double j1;
        double j2;
        double j3;
        double j4;
        double j5;
        double j6;
        double j7;
        double j8;
        double j9;
    };

    // Define a point structure used in joint trajectory fifo
    struct JointPoint
    {
        int id;
        // double velocity;
        JointOmegas omegas;
        JointValues joints;
    };

    struct CoordinateOffset
    {
        double alpha;
        double a;
        double d;
        double theta;
    };

    struct DHGroup
    {
        CoordinateOffset j1;
        CoordinateOffset j2;
        CoordinateOffset j3;
        CoordinateOffset j4;
        CoordinateOffset j5;
        CoordinateOffset j6;
    };



}   // namespace fst_controller



#endif  // #ifndef FST_DATATYPE_H
