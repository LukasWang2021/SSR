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

namespace fst_controller
{

void forwardMaximumDuration(const JointState je, const JointState js, double *alpha_min, double *alpha_max, double *t_max);
}

int main(int argc, char **argv)
{
    ArmGroup arm;
    arm.initArmGroup();

    JointState je,js;

    js.joint[0] = 0.128224;
    js.joint[1] = -0.111427;
    js.joint[2] = -0.038589;
    js.joint[3] = -0.047956;
    js.joint[4] = -1.479401;
    js.joint[5] = 0.037754;
    js.omega[0] = 0.345906;
    js.omega[1] = -0.379308;
    js.omega[2] = -0.040702;
    js.omega[3] = -0.130667;
    js.omega[4] = 0.239133;
    js.omega[5] = 0.093355;
    js.alpha[0] = 4.053574;
    js.alpha[1] = -4.445000;
    js.alpha[2] = -0.476975;
    js.alpha[3] = -1.531247;
    js.alpha[4] = 2.802335;
    js.alpha[5] = 1.094001;
    je.joint[0] = 0.142639;
    je.joint[1] = -0.128119;
    je.joint[2] = -0.039438;
    je.joint[3] = -0.053431;
    je.joint[4] = -1.469588;
    je.joint[5] = 0.041512;
    je.omega[0] = 0;
    je.omega[1] = 0;
    je.omega[2] = 0;
    je.omega[3] = 0;
    je.omega[4] = 0;
    je.omega[5] = 0;
    je.alpha[0] = 0;
    je.alpha[1] = 0;
    je.alpha[2] = 0;
    je.alpha[3] = 0;
    je.alpha[4] = 0;
    je.alpha[5] = 0;

    double t_max[6];
    double alpha_max[6];
    double alpha_min[6];
    double jerk[AXIS_IN_ALGORITHM] = {5.258, 4.445, 11.737, 197.644, 75.21, 105.61};


    for (int i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        alpha_max[i] = js.alpha[i] + jerk[i];
        alpha_min[i] = js.alpha[i] - jerk[i];
    }

    fst_controller::forwardMaximumDuration(je, js, alpha_min, alpha_max, t_max);


    FST_INFO("t-max: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", t_max[0], t_max[1], t_max[2], 
                                                     t_max[3], t_max[4], t_max[5]);

    return 0;
}

