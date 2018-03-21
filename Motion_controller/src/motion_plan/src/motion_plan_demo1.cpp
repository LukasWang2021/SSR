/*************************************************************************
	> File Name: motion_plan_demo1.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年01月09日 星期二 10时15分23秒
 ************************************************************************/

#include <iostream>
#include <time.h>
#include <string.h>
#include <motion_plan_arm_group.h>
#include <motion_plan_matrix.h>
#include <motion_plan_kinematics.h>
#include <motion_plan_variable.h>

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
    fst_algorithm::forwardKinematics(jnt, pose);
    
    cout << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ", "
         << pose.orientation.a << ", " << pose.orientation.b << ", " << pose.orientation.c << endl;

    Joint ref, res;
    ref.j1 = 0.1017;
    ref.j2 = -0.1986;
    ref.j3 = 0.2502;
    ref.j4 = 0.171;
    ref.j5 = -1.568;
    ref.j6 = -0.385;
    cout << "IK res=" << fst_algorithm::inverseKinematics(pose, ref, res) << endl;
    cout << res.j1 << ", " << res.j2 << ", " << res.j3 << ", " << res.j4 << ", " << res.j5 << ", " << res.j6 << endl;
    
    int loop = 1000000;
	clock_t start, finish;

    start = clock();
    for (int i = 0; i < loop; i++)
        fst_algorithm::forwardKinematics(jnt, pose);
    finish = clock();

    cout << "New FK alg loop = " << loop << ", time = " << (double)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms, " << (double)(finish - start) / CLOCKS_PER_SEC << "us/point" << endl;

    start = clock();
    for (int i = 0; i < loop; i++)
        fst_algorithm::inverseKinematics(pose, ref, res);
    finish = clock();

    cout << "New IK alg loop = " << loop << ", time = " << (double)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms, " << (double)(finish - start) / CLOCKS_PER_SEC << "us/point" << endl;
}

