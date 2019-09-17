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

#define SET_NUM 1

bool isCorrect(Joint& init, Joint& result)
{
    for(int i = 0; i < 6; ++i)
    {
        if(fabs(init[i] - result[i]) > 0.001
            && fabs(fabs(init[i] - result[i]) - 3.1415926) > 0.001)
        {
            return false;
        }
    }
    return true;
}


bool isNearestReferenceJointSingle(const double& init_single_joint, const double& ref_single_joint, const int& single_turn)
{
    double turn_joint[4], delta_joint[4], single_joint;
    turn_joint[0] = init_single_joint;
    turn_joint[1] = init_single_joint - single_turn * 2 * M_PI;
    turn_joint[2] = init_single_joint - 2 * M_PI;
    turn_joint[3] = init_single_joint + 2 * M_PI;

    delta_joint[0] = fabs(turn_joint[0] - ref_single_joint);
    delta_joint[1] = fabs(turn_joint[1] - ref_single_joint);
    delta_joint[2] = fabs(turn_joint[2] - ref_single_joint);
    delta_joint[3] = fabs(turn_joint[3] - ref_single_joint);

    double min_delta_joint = delta_joint[0];
    for (int i = 1; i != 4; ++i)
    {
        if (delta_joint[i] < min_delta_joint)
        {
            return false;
        } 
    }

    return true;
}

bool isNearestReferenceJoint(Joint& init, Joint& result, const Turn& turn)
{
    int turn_temp[9];
    turn_temp[0] = turn.j1; turn_temp[1] = turn.j2; turn_temp[2] = turn.j3; 
    turn_temp[3] = turn.j4; turn_temp[4] = turn.j5; turn_temp[5] = turn.j6; 
    for(int i = 0; i < 6; ++i)
    {
        if(!isNearestReferenceJointSingle(init[i], result[i], turn_temp[i]))
        {
            return false;
        }
    }
    return true;
}


void testJointTurn(void)
{
    Posture posture;
    PoseEuler pose;
    Joint joint_input, joint_reference, joint_output;
    KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");

    while (true)
    {
        
        joint_reference.j1_ = 0.1 * ((double)rand() / RAND_MAX);
        joint_reference.j2_ = 0.1 * ((double)rand() / RAND_MAX);
        joint_reference.j3_ = 3.1415926 * ((double)rand() / RAND_MAX);
        joint_reference.j4_ = 3.1415926 * ((double)rand() / RAND_MAX);
        joint_reference.j5_ = 0.1 * ((double)rand() / RAND_MAX);
        joint_reference.j6_ = 3.1415926 * 3 * ((double)rand() / RAND_MAX);
        joint_reference.j7_ = 0;
        joint_reference.j8_ = 0;
        joint_reference.j9_ = 0;

        joint_input.j1_ = -2.9 + 5.80 * ((double)rand() / RAND_MAX);
        joint_input.j2_ = -2.35 + 4.09 * ((double)rand() / RAND_MAX);
        joint_input.j3_ = -1.22 + 4.36 * ((double)rand() / RAND_MAX);
        joint_input.j4_ = -3.14 + 6.28 * ((double)rand() / RAND_MAX);
        joint_input.j5_ = -2.0 + 4.0 * ((double)rand() / RAND_MAX);
        joint_input.j6_ = -3.14 + 6.28 * ((double)rand() / RAND_MAX);
        joint_input.j7_ = 0;
        joint_input.j8_ = 0;
        joint_input.j9_ = 0;

        joint_reference += joint_input;

        Turn joint_turn = kinematics.getTurnByJoint(joint_reference);

        kinematics.doFK(joint_input, pose);
        posture = kinematics.getPostureByJoint(joint_input);

        if (!kinematics.doIK(pose, joint_reference, joint_output))
        {
            printf("ERROR: IK failed.\n");
            printf("  Joint input: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint_input.j1_, joint_input.j2_, joint_input.j3_, joint_input.j4_, joint_input.j5_, joint_input.j6_);
            printf("  Joint reference: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint_reference.j1_, joint_reference.j2_, joint_reference.j3_, joint_reference.j4_, joint_reference.j5_, joint_reference.j6_);
            printf("  Pose: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            continue;
        }

        joint_output.j7_ = 0;
        joint_output.j8_ = 0;
        joint_output.j9_ = 0;

        if (!isNearestReferenceJoint(joint_output, joint_reference, joint_turn))
        {
            printf("ERROR: output joint different with input\n");
            printf("  Pose: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            printf("  Posture: %d,%d,%d,%d\n", posture.arm, posture.elbow, posture.wrist, posture.flip);
            printf("  Joint turn: %d,%d,%d,%d,%d,%d\n", joint_turn.j1, joint_turn.j2, joint_turn.j3, joint_turn.j4, joint_turn.j5, joint_turn.j6);
            joint_input.print("joint input:");
            joint_output.print("joint output:");
            joint_reference.print("joint ref:");
        }
        else
        {
            printf("output joint different with input ok\n");
        }
    }
}



int main()
{
    // d a alpha offset
    DH base_dh;
    DH arm_dh[6];

    base_dh.d = 365;      base_dh.a = 0;       base_dh.alpha = 0;              base_dh.offset = 0;
    arm_dh[0].d = 0;        arm_dh[0].a = 30;        arm_dh[0].alpha = M_PI / 2;     arm_dh[0].offset = 0;
    arm_dh[1].d = 0;        arm_dh[1].a = 340.2;     arm_dh[1].alpha = 0;            arm_dh[1].offset = M_PI / 2;
    arm_dh[2].d = 0;        arm_dh[2].a = 34.85;   arm_dh[2].alpha = M_PI / 2;     arm_dh[2].offset = 0;
    arm_dh[3].d = 350.3;     arm_dh[3].a = 0;        arm_dh[3].alpha = -M_PI / 2;    arm_dh[3].offset = 0;
    arm_dh[4].d = 0;        arm_dh[4].a = 0;        arm_dh[4].alpha = M_PI / 2;     arm_dh[4].offset = 0;
    arm_dh[5].d = 96.5;   arm_dh[5].a = 0;        arm_dh[5].alpha = 0;            arm_dh[5].offset = 0;
    
    KinematicsRTM k("/root/install/share/runtime/axis_group/");
    
    Joint joint[SET_NUM];
    joint[0].j1_ = -0.7238; joint[0].j2_ = 0.759417; joint[0].j3_ = 1.01465; joint[0].j4_ = 1.05331; joint[0].j5_ = 0.126426; joint[0].j6_ = -2.89332;

    Joint ref_joint[SET_NUM];
    ref_joint[0].j1_ = -0.699972;  ref_joint[0].j2_ = 0.856481; ref_joint[0].j3_ = 3.84902;  ref_joint[0].j4_ = 3.72655;  ref_joint[0].j5_ = 0.153092; ref_joint[0].j6_ = 2.193802;

    Turn turn_joint[SET_NUM];
    Posture posture[SET_NUM];
    for(int i = 0; i < SET_NUM; ++i)
    {
        posture[i] = k.getPostureByJoint(joint[i]);
        turn_joint[i] = k.getTurnByJoint(ref_joint[i]);
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
        ret[i] = k.doIK(pose_euler[i], ref_joint[i], result_joint[i]);
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
                ref_joint[i].print("ik ref:");
                printf("turn_joint : %d,%d,%d,%d,%d,%d\n", turn_joint[i].j1, turn_joint[i].j2, turn_joint[i].j3, turn_joint[i].j4, turn_joint[i].j5, turn_joint[i].j6);

            }
        }
        else
        {
            std::cout<<"Test set "<<i<<" is ik failed, ";
        }
        std::cout<<"posture is "<<posture[i].arm<<" "<<posture[i].elbow<<" "<<posture[i].wrist<<" "<<posture[i].flip<<std::endl;
    }

    // testJointTurn();
    return 0;
}
