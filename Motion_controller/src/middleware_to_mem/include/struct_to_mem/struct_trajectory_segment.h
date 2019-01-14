#ifndef MIDDLEWARE_TO_MEM_STRUCT_TRAJECTORY_SEGMENT_H_
#define MIDDLEWARE_TO_MEM_STRUCT_TRAJECTORY_SEGMENT_H_

#ifndef JOINT_NUM
#define JOINT_NUM 6
#endif

#define TS_POINT_NUM 10
#define POSITION_ONLY 1
#define POS_VEL 2
#define POS_VEL_ACC 4
#define POS_VEL_ACC_EFF 8

typedef struct 
{
    unsigned int sec;
    unsigned int nsec;
}Time;

typedef struct 
{
    double positions[JOINT_NUM];
    double velocities[JOINT_NUM];
    double accelerations[JOINT_NUM];
    double effort[JOINT_NUM];
    unsigned int valid_level;
    Time time_from_start;
}TrajectoryPoints;

typedef struct
{
    TrajectoryPoints points[TS_POINT_NUM];
    Time stamp;
    unsigned int total_points;
    unsigned int seq;  
    unsigned int last_fragment;
}TrajectorySeg;


#endif //MIDDLEWARE_TO_MEM_STRUCT_TRAJECTORY_SEGMENT_H_
