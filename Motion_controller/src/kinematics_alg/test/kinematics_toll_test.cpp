#include "kinematics_toll.h"
#include <iostream>
#include <math.h>
#include <string.h>

using namespace std;
using namespace basic_alg;

#define SET_NUM 6

bool isCorrect(Joint& init, Joint& result)
{
    for(int i = 0; i < 4; ++i)
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
    arm_dh[0].d = 0;        arm_dh[0].a = 412.67;  arm_dh[0].alpha = 0;  arm_dh[0].offset = 0;
    arm_dh[1].d = 60.11;  arm_dh[1].a = 300;      arm_dh[1].alpha = 0;  arm_dh[1].offset = 0;
    arm_dh[2].d = 0;        arm_dh[2].a = 400;      arm_dh[2].alpha = 0;  arm_dh[2].offset = 0;
    arm_dh[3].d = 69.98;  arm_dh[3].a = 198.03;  arm_dh[3].alpha = 0;  arm_dh[3].offset = 0;
    
    KinematicsToll k(base_dh, arm_dh, false);
    
    Joint joint[SET_NUM];
    for(int i = 0; i < SET_NUM; ++i)
    {
        memset(&joint[i], 0, sizeof(Joint));
    }
    joint[0].j1_ = 1000;     joint[0].j2_ = 0;        joint[0].j3_ = 0;         joint[0].j4_ = 0;  
    joint[1].j1_ = 500;   joint[1].j2_ = M_PI / 2; joint[1].j3_ = -M_PI / 2; joint[1].j4_ = 0;  
    joint[2].j1_ = 250;  joint[2].j2_ = -M_PI / 4;joint[2].j3_ = M_PI / 4; joint[2].j4_ = M_PI / 2;  
    joint[3].j1_ = 125; joint[3].j2_ = -M_PI / 4;joint[3].j3_ = -M_PI / 2;  joint[3].j4_ = -M_PI / 2;  
    joint[4].j1_ = 8.025487; joint[4].j2_ = 0.025782;joint[4].j3_ = 1.224100;  joint[4].j4_ = 0.06948; 
    joint[5].j1_ = 8.025487; joint[5].j2_ = 0.025782;joint[5].j3_ = 1.224100;  joint[5].j4_ = 0.06948; 

    Posture posture[SET_NUM];
    for(int i = 4; i < SET_NUM; ++i)
    {
        posture[i] = k.getPostureByJoint(joint[i]);
        if (i == 5)
        {
         //   posture[i].arm = -1;
        }
        printf("init posture[%d]=%d, %d, %d\n", i, posture[i].arm, posture[i].elbow,posture[i].wrist);
    }

    PoseEuler pose_euler[SET_NUM];
    for(int i = 4; i < SET_NUM; ++i)
    {
        k.doFK(joint[i], pose_euler[i]);
        
        if (i == 5)
        {
            pose_euler[i].point_.x_ = 942.8402;
            pose_euler[i].point_.y_ = 543.8398;
            pose_euler[i].point_.z_ = 138.0645;
            pose_euler[i].euler_.a_ = 1.1288;
            pose_euler[i].euler_.b_ = 0;
            pose_euler[i].euler_.c_ = 0;
        }
        printf("pose_euler:%f, %f, %f, %f, %f, %f\n",pose_euler[i].point_.x_,pose_euler[i].point_.y_,pose_euler[i].point_.z_,
              pose_euler[i].euler_.a_,pose_euler[i].euler_.b_,pose_euler[i].euler_.c_);
    }

    Joint result_joint[SET_NUM];
    bool ret[SET_NUM];
    for(int i = 4; i < SET_NUM; ++i)
    {
        ret[i] = k.doIK(pose_euler[i], posture[i], result_joint[i]);
        printf("\n\n result joint=%f,%f,%f,%f\n", result_joint[i].j1_,result_joint[i].j2_,result_joint[i].j3_,result_joint[i].j4_);
    }

    for(int i = 4; i < SET_NUM; ++i)
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
        std::cout<<"posture is "<<posture[i].arm<<" "<<posture[i].elbow<<" "<<posture[i].wrist<<std::endl;
    }
/*
    PoseEuler pose_euler1;
    pose_euler1.point_.x_ = 942.8402;
    pose_euler1.point_.y_ = 543.8398;
    pose_euler1.point_.z_ = 122.0645; 
    pose_euler1.euler_.a_ = 1.1288;
    pose_euler1.euler_.b_ = 0;
    pose_euler1.euler_.c_ = 0;
    Joint result_joint1;
    Posture posture1;
    posture1.arm = 1;
    posture1.elbow = 1;
    posture1.wrist = 1;
    k.doIK(pose_euler1, posture1, result_joint1);
    printf("\n\nothers:resuelt joint1=%f,%f,%f,%f\n", result_joint1.j1_,result_joint1.j2_,result_joint1.j3_,result_joint1.j4_);

    k.doFK(result_joint1, pose_euler1);
    printf("pose_euler1:%f, %f, %f, %f, %f, %f\n",pose_euler1.point_.x_,pose_euler1.point_.y_,pose_euler1.point_.z_,
              pose_euler1.euler_.a_,pose_euler1.euler_.b_,pose_euler1.euler_.c_);
*/
    return 0;
}

