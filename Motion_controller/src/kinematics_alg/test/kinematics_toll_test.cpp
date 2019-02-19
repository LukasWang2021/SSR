#include "kinematics_toll.h"
#include <iostream>
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
    DH arm_dh[4];

    base_dh.d = 0;          base_dh.a = 0;          base_dh.alpha = 0;    base_dh.offset = 0;
    arm_dh[0].d = 0;        arm_dh[0].a = 0.41267;  arm_dh[0].alpha = 0;  arm_dh[0].offset = 0;
    arm_dh[1].d = 0.06011;  arm_dh[1].a = 0.3;      arm_dh[1].alpha = 0;  arm_dh[1].offset = 0;
    arm_dh[2].d = 0;        arm_dh[2].a = 0.4;      arm_dh[2].alpha = 0;  arm_dh[2].offset = 0;
    arm_dh[3].d = 0.06998;  arm_dh[3].a = 0.19803;  arm_dh[3].alpha = 0;  arm_dh[3].offset = 0;
    
    KinematicsToll k(base_dh, arm_dh, false);
    
    Joint joint[SET_NUM];
    joint[0].j1_ = 1;     joint[0].j2_ = 0;        joint[0].j3_ = 0;         joint[0].j4_ = 0;  
    joint[1].j1_ = 0.5;   joint[1].j2_ = M_PI / 2; joint[1].j3_ = -M_PI / 2; joint[1].j4_ = 0;  
    joint[2].j1_ = 0.25;  joint[2].j2_ = -M_PI / 4;joint[2].j3_ = M_PI / 4; joint[2].j4_ = M_PI / 2;  
    joint[3].j1_ = 0.125; joint[3].j2_ = -M_PI / 4;joint[3].j3_ = -M_PI / 2;  joint[3].j4_ = -M_PI / 2;   

    PostureToll posture[SET_NUM];
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
                std::cout<<"Toll Test set "<<i<<" is correct, ";
            }
            else
            {
                std::cout<<"Toll Test set "<<i<<" is not correct, ";
                joint[i].print("fk input:");
                result_joint[i].print("ik output:");
            }
        }
        else
        {
            std::cout<<"Test set "<<i<<" is ik failed, ";
        }
        std::cout<<"posture is "<<posture[i].arm<<std::endl<<std::endl;
    }

    return 0;
}

