/*************************************************************************
	> File Name: motion_plan_demo1.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年01月09日 星期二 10时15分23秒
 ************************************************************************/

#include <iostream>
#include <time.h>
#include <string.h>
#include <fstream>

#include <parameter_manager/parameter_manager_param_group.h>

#include <motion_plan_arm_group.h>
#include <motion_plan_matrix.h>
#include <motion_plan_kinematics.h>
#include <motion_plan_variable.h>
#include <motion_plan_motion_command.h>
#include <motion_plan_traj_plan.h>
#include <motion_plan_reuse.h>
#include <motion_plan_manual_teach.h>

using namespace std;
using namespace fst_controller;
using namespace fst_algorithm;

int main(int argc, char **argv)
{
    cout << "hello" << endl;
    cout << "hello" << endl;
    cout << "hello" << endl;
    cout << "hello" << endl;
    Joint start_joint;
    start_joint.j1 = 0.0;
    start_joint.j2 = 0.0;
    start_joint.j3 = 0.0;
    start_joint.j4 = 0.0;
    start_joint.j5 = -1.5708;
    start_joint.j6 = 0.0;

    MotionTarget target1;
    target1.type = MOTION_JOINT;
    target1.cnt = 0.5;
    target1.vel = 1;
    target1.acc = 1;
    target1.joint_target.j1 = 0;
    target1.joint_target.j2 = 0;
    target1.joint_target.j3 = 0;
    target1.joint_target.j4 = 0;
    target1.joint_target.j5 = 1.57;
    target1.joint_target.j6 = 0;

    MotionTarget target2;
    target2.type = MOTION_LINE;
    target2.cnt = 0;
    target2.vel = 1500.0;
    target2.acc = 16000.0;
    target2.pose_target.position.x = 480.0;
    target2.pose_target.position.y = 160.0;
    target2.pose_target.position.z = 450.0;
    target2.pose_target.orientation.a = 0.3;
    target2.pose_target.orientation.b = 0.2;
    target2.pose_target.orientation.c = 3.0;

    MotionTarget target3;
    target3.type = MOTION_JOINT;
    target3.cnt = 0.5;
    target3.vel = 1;
    target3.acc = 1;
    target3.joint_target.j1 = 1.0;
    target3.joint_target.j2 = 0.3;
    target3.joint_target.j3 = 0.8;
    target3.joint_target.j4 = 1.40;
    target3.joint_target.j5 = -1;
    target3.joint_target.j6 = 0.8;

    ArmGroup arm;
    ErrorCode err = arm.initArmGroup();

    arm.setStartState(start_joint);
    arm.autoMove(target3, 5);
    arm.autoMove(target1, 7);

    



    cout << "end" << endl;

    return 0;
}

