#ifndef STRUCT_FEEDBACK_JOINT_STATES_H_
#define STRUCT_FEEDBACK_JOINT_STATES_H_

#ifndef JOINT_NUM
#define JOINT_NUM 6
#endif

#define STATE_INIT 0
#define STATE_READY 1
#define STATE_RUNNING 2
#define STATE_ERROR 3

typedef struct
{
    double position[JOINT_NUM]; 
    double velocity[JOINT_NUM];
    double effort[JOINT_NUM];
    unsigned int state;
}FeedbackJointState;


#endif //STRUCT_FEEDBACK_JOINT_STATES_H_

