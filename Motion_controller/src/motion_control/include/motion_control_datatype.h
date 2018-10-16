/*************************************************************************
	> File Name: motion_control_datatype.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 09时26分40秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_DATATYPE_H
#define _MOTION_CONTROL_DATATYPE_H

#include <base_datatype.h>

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

    int user_frame_id;
    int tool_frame_id;

    union
    {
        PoseEuler       pose_target;
        Joint           joint_target;
        CircleTarget    circle_target;
    };
};

struct PathPoint
{
    // Command type, joint command or cartesian command*/
    MotionType  type;

    int         id;

    // point stamp
    size_t      stamp;

    // value in cartesian space or joint space
    union
    {
        Pose    pose;
        Joint   joint;
    };
};

struct TrajectorySegment
{
    // point from path plan
    PathPoint   path_point;
    JointPoint  start_state;
    JointPoint  ending_state;

    // time from start
    // time < 0 means this point has not been converted to a trajectory point
    MotionTime  time_from_start;

    // time duration from prev point to this point
    // duration < 0 means this point has not been converted to a trajectory point
    // command_duration = cycle_step / v_command
    // duration = actual dutation
    //MotionTime  command_duration;
    MotionTime  forward_duration;
    MotionTime  backward_duration;

    // point is what trajectory-create should give out
    //JointPoint  forward_point;
    //JointPoint  backward_point;

    TrajSegment  forward_coeff[NUM_OF_JOINT];
    TrajSegment  backward_coeff[NUM_OF_JOINT];

    DynamicsProduct dynamics_product;
};

struct TrajectoryCache
{
    bool    valid;
    size_t  head;
    size_t  tail;
    size_t  smooth_in_stamp;
    size_t  smooth_out_stamp;
    double  deadline;
    double  expect_duration;

    TrajectorySegment cache[MAX_PATH_SIZE];
    //TrajectoryCache *prev;
    TrajectoryCache *next;
};

struct TrajectoryItem
{
    int         id;
    MotionTime  time_from_start;
    MotionTime  duration;
    TrajSegment traj_coeff[NUM_OF_JOINT];
    DynamicsProduct dynamics_product;
};

class GroupDirection
{
  public:
    ManualDirection axis1;
    ManualDirection axis2;
    ManualDirection axis3;
    ManualDirection axis4;
    ManualDirection axis5;
    ManualDirection axis6;
    ManualDirection axis7;
    ManualDirection axis8;
    ManualDirection axis9;

    ManualDirection& operator[](size_t index) {assert(index < NUM_OF_JOINT); return *(&axis1 + index);}
    const ManualDirection& operator[](size_t index) const {assert(index < NUM_OF_JOINT); return *(&axis1 + index);}
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
    PoseEuler   tool_coordinate;

    MotionTime      duration;
    ManualCoef      coeff[6];
};

struct TrajectoryPoint
{
    Joint   angle;
    Joint   omega;
    Joint   alpha;
    Joint   ma_cv_g;
    //Joint   inertia;
    //Joint   gravity;
    PointLevel  level;
};


}

#endif
