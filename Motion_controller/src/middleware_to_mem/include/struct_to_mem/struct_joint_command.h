#ifndef MIDDLEWARE_TO_MEM_STRUCT_JOINT_COMMAND_H_
#define MIDDLEWARE_TO_MEM_STRUCT_JOINT_COMMAND_H_

#ifndef JOINT_NUM
#define JOINT_NUM 6
#endif

#define JC_POINT_NUM 10
#define START_POINT 1
#define END_POINT 2
#define MID_POINT 0

typedef struct 
{
    double angle[JOINT_NUM];
    double omega[JOINT_NUM];
    double alpha[JOINT_NUM];
    double inertia[JOINT_NUM];
    int point_position;
}Points;

typedef struct 
{
    Points points[JC_POINT_NUM];
    int total_points;
}JointCommand;


#endif //MIDDLEWARE_TO_MEM_STRUCT_JOINT_COMMAND_H_
