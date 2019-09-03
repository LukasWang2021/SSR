#include "kinematics_rtm.h"
#include <iostream>
// #include "joint.h"
// #include "point.h"
// #include "quaternion.h"
// #include "pose_euler.h"
// #include "pose_quaternion.h"
// #include "rotation_matrix.h"
// #include "trans_matrix.h"
// #include "matrix33.h"
#include <math.h>

using namespace std;
using namespace basic_alg;

#define SET_NUM 4

bool isCorrect(Joint& init, Joint& result)
{
    for(int i = 0; i < 6; ++i)
    {
        if(fabs(init[i] - result[i]) > 0.001)
        {
            return false;
        }
    }
    return true;
}

int main()
{
    // d a alpha offset
    DH base_dh;
    DH arm_dh[6];

    base_dh.d = 0.365;      base_dh.a = 0.03;       base_dh.alpha = 0;              base_dh.offset = 0;
    arm_dh[0].d = 0;        arm_dh[0].a = 0;        arm_dh[0].alpha = M_PI / 2;     arm_dh[0].offset = 0;
    arm_dh[1].d = 0;        arm_dh[1].a = 0.34;     arm_dh[1].alpha = 0;            arm_dh[1].offset = M_PI / 2;
    arm_dh[2].d = 0;        arm_dh[2].a = -0.035;   arm_dh[2].alpha = M_PI / 2;     arm_dh[2].offset = 0;
    arm_dh[3].d = 0.35;     arm_dh[3].a = 0;        arm_dh[3].alpha = -M_PI / 2;    arm_dh[3].offset = 0;
    arm_dh[4].d = 0;        arm_dh[4].a = 0;        arm_dh[4].alpha = M_PI / 2;     arm_dh[4].offset = 0;
    arm_dh[5].d = 0.0965;   arm_dh[5].a = 0;        arm_dh[5].alpha = 0;            arm_dh[5].offset = 0;
    
    KinematicsRTM k(base_dh, arm_dh, false);
    
    Joint joint[SET_NUM];
    joint[0].j1_ = M_PI / 4; joint[0].j2_ = 0;        joint[0].j3_ = 0;        joint[0].j4_ = 0;        joint[0].j5_ = M_PI / 4; joint[0].j6_ = 0;
    joint[1].j1_ = M_PI / 4; joint[1].j2_ = 0;        joint[1].j3_ = -M_PI / 2;        joint[1].j4_ = 0;        joint[1].j5_ = -M_PI / 2;joint[1].j6_ = M_PI / 4;
    joint[2].j1_ = M_PI / 4; joint[2].j2_ = -M_PI / 4;joint[2].j3_ = 0;        joint[2].j4_ = 0;        joint[2].j5_ = M_PI / 4; joint[2].j6_ = 0;
    joint[3].j1_ = M_PI / 4; joint[3].j2_ = -M_PI / 4;joint[3].j3_ = M_PI / 2;        joint[3].j4_ = 0;        joint[3].j5_ = M_PI / 2;joint[3].j6_ = -M_PI / 4;

    Posture posture[SET_NUM];
    for(int i = 0; i < SET_NUM; ++i)
    {
        posture[i] = k.getPostureByJoint(joint[i]);
        
    }

    PoseEuler pose_euler[SET_NUM];
    for(int i = 0; i < SET_NUM; ++i)
    {
        k.doFK(joint[i], pose_euler[i]);
    }

    Joint result_joint[SET_NUM];
    bool ret[SET_NUM];
    for(int i = 0; i < SET_NUM; ++i)
    {
        ret[i] = k.doIK(pose_euler[i], posture[i], result_joint[i]);
    }

    for(int i = 0; i < SET_NUM; ++i)
    {
        if(ret[i])
        {
            if(isCorrect(joint[i], result_joint[i]))
            {
                std::cout<<"Test set "<<i<<" is correct, ";
            }
            else
            {
                std::cout<<"Test set "<<i<<" is not correct, ";
                joint[i].print("fk input:");
                result_joint[i].print("ik output:");
            }
        }
        else
        {
            std::cout<<"Test set "<<i<<" is ik failed, ";
        }
        std::cout<<"posture is "<<posture[i].arm<<" "<<posture[i].elbow<<" "<<posture[i].wrist<<" "<<posture[i].flip<<std::endl;
    }

    return 0;
}

