/*************************************************************************
	> File Name: motion_control_datatype.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 09时26分40秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_DATATYPE_H
#define _MOTION_CONTROL_DATATYPE_H

#include <motion_control_base_type.h>

namespace fst_mc
{

struct JointConstraint
{
    Joint upper;
    Joint lower;
};

struct CircleTarget
{
    PoseEuler pose1;
    PoseEuler pose2;
};

struct MotionTarget
{
    MotionType  type;

    // 0.0 - 1.0
    double cnt;

    // percent velocity in move Joint, range: 0.0-1.0
    // linear velocity in move cartesian, range 0.0-MAC_VEL
    // velocity < 0 means using default velocity
    double vel;

    union {
        PoseEuler       pose_target;
        Joint           joint_target;
        CircleTarget    circle_target;
    };
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

struct ManualCoef
{
    MotionTime start_time;
    MotionTime stable_time;
    MotionTime brake_time;
    MotionTime stop_time;

    double  start_alpha;
    double  brake_alpha;
};

struct ManualTrajectory
{
    ManualMode      mode;
    ManualFrame     frame;
    ManualDirection direction[6];

    Joint       joint_start;
    Joint       joint_ending;

    PoseEuler   cart_start;
    PoseEuler   cart_ending;

    MotionTime      duration;
    ManualCoef      coeff[6];
};


}

#endif
