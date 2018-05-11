/*************************************************************************
	> File Name: motion_plan_demo_constraint.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年05月11日 星期五 14时24分02秒
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
    ArmGroup    arm;
    ErrorCode   err;
    
    if (arm.initArmGroup() != SUCCESS)
    {
        cout << "initArmGroup ERROR" << endl;
        return 0;
    }

    JointConstraint hard_cons = arm.getHardConstraint();
    JointConstraint soft_cons = arm.getSoftConstraint();
    
    cout << "set J1 constraint to -1 ~ 1" << endl;
    soft_cons.j1.upper = 1.0;
    soft_cons.j1.lower = -1.0;
    err = arm.setSoftConstraint(soft_cons);

    if (err == SUCCESS)
    {
        cout << "setSoftConstraint: success!" << endl;
    }
    else
    {
        cout << "setSoftConstraint: failed" << endl;
    }

    cout << "set J2 constraint to -1 ~ 5" << endl;
    soft_cons.j2.upper = 5.0;
    soft_cons.j2.lower = -1.0;
    err = arm.setSoftConstraint(soft_cons);

    if (err == SUCCESS)
    {
        cout << "setSoftConstraint: success!" << endl;
    }
    else
    {
        cout << "setSoftConstraint: failed" << endl;
    }

    return 0;
}
