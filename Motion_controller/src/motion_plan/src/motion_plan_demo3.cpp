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


using namespace std;
using namespace fst_controller;
using namespace fst_algorithm;

int main(int argc, char **argv)
{
    Joint       jnt;
    PoseEuler   pose;

    jnt.j1 = 0.1;
    jnt.j2 = -0.2;
    jnt.j3 = 0.25;
    jnt.j4 = 0.17;
    jnt.j5 = -1.5708;
    jnt.j6 = -0.38;
/////////////////////////////////////////////////////////////////////////////////////
    

    Joint start_joint;
    start_joint.j1 = 0.0;
    start_joint.j2 = 0.0;
    start_joint.j3 = 0.0;
    start_joint.j4 = 0.0;
    start_joint.j5 = -1.5708;
    start_joint.j6 = 0.0;

    MotionTarget target;
    target.type = MOTION_LINE;
    target.cnt = 0;
    target.vel = 1500.0;
    target.acc = 16000.0;
    target.pose_target.position.x = 480.0;
    target.pose_target.position.y = 160.0;
    target.pose_target.position.z = 450.0;
    target.pose_target.orientation.a = 0.3;
    target.pose_target.orientation.b = 0.2;
    target.pose_target.orientation.c = 3.0;


    ArmGroup arm;
    ErrorCode err = arm.initArmGroup();

    if (err != SUCCESS)
        return err;
    
    arm.setStartState(start_joint);
    arm.autoMove(target, 5);
    //arm.speedup();

    std::ofstream os("jout.csv");
    std::vector<JointOutput> points;

    arm.getPointFromFIFO(300, points);
    FST_INFO("get %d points", points.size());

    for (size_t i = 0; i < points.size(); i++)
    {
        //FST_INFO("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f",\
                 points[i].joint.j1,\
                 points[i].joint.j2,\
                 points[i].joint.j3,\
                 points[i].joint.j4,\
                 points[i].joint.j5,\
                 points[i].joint.j6);

        os  << i + 1 << ","\
            << points[i].joint.j1 << ","\
            << points[i].joint.j2 << ","\
            << points[i].joint.j3 << ","\
            << points[i].joint.j4 << ","\
            << points[i].joint.j5 << ","\
            << points[i].joint.j6 << endl;
            
    }


    cout << "end" << endl;

    return 0;
}

