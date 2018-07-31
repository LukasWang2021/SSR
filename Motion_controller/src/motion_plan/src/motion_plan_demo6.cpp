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
#include <lock_free_fifo.h>
#include <motion_plan_kinematics.h>

using namespace std;
using namespace fst_controller;
using namespace fst_algorithm;



int main(int argc, char **argv)
{
    ArmGroup arm;
    arm.initArmGroup();

    PoseEuler pose1, pose2, pose_forward1, pose_forward2;
    Joint jnt1, jnt2, jnt_ref;

    jnt_ref.j1 = 0;
    jnt_ref.j2 = 0;
    jnt_ref.j3 = 0;
    jnt_ref.j4 = 0;
    jnt_ref.j5 = -1.5708;
    jnt_ref.j6 = 0;

    pose1.position.x = 400;
    pose1.position.y = -250;
    pose1.position.z = 425;
    pose1.orientation.a = 0;
    pose1.orientation.b = 0.7;
    pose1.orientation.c = 3.4;

    pose2.position.x = 400;
    pose2.position.y = 250;
    pose2.position.z = 425;
    pose2.orientation.a = 0;
    pose2.orientation.b = 0.7;
    pose2.orientation.c = 3.4;


    inverseKinematics(pose1, jnt_ref, jnt1);
    inverseKinematics(pose2, jnt_ref, jnt2);

    forwardKinematics(jnt1, pose_forward1);
    forwardKinematics(jnt2, pose_forward2);


    FST_INFO("pose-1: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", pose1.position.x, pose1.position.y, pose1.position.z, pose1.orientation.a, pose1.orientation.b, pose1.orientation.c);
    FST_INFO("jnt-1:  %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", jnt1.j1, jnt1.j2, jnt1.j3, jnt1.j4, jnt1.j5, jnt1.j6);
    FST_INFO("pfor-1: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", pose_forward1.position.x, pose_forward1.position.y, pose_forward1.position.z, pose_forward1.orientation.a, pose_forward1.orientation.b, pose_forward1.orientation.c);

    FST_INFO("pose-2: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", pose2.position.x, pose2.position.y, pose2.position.z, pose2.orientation.a, pose2.orientation.b, pose2.orientation.c);
    FST_INFO("jnt-2:  %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", jnt2.j1, jnt2.j2, jnt2.j3, jnt2.j4, jnt2.j5, jnt2.j6);
    FST_INFO("pfor-2: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", pose_forward2.position.x, pose_forward2.position.y, pose_forward2.position.z, pose_forward2.orientation.a, pose_forward2.orientation.b, pose_forward2.orientation.c);


    return 0;
}

