
#ifndef _STRUCT_JOINT_COMMAND_H_
#define _STRUCT_JOINT_COMMAND_H_

#ifndef JOINT_NUM
#define JOINT_NUM 6
#endif
#define JC_POINT_NUM 10

#define START_POINT 1
#define END_POINT 2
#define MID_POINT 0

typedef struct 
{
    double positions[JOINT_NUM];
    int point_position;
}Points;

typedef struct 
{
    Points points[JC_POINT_NUM];
    int total_points;
}JointCommand;


#endif
