/*************************************************************************
	> File Name: motion_plan_demo4.cpp
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
    g_cycle_time = 0.001;                   // s
    g_global_vel_ratio = 1.0;
    g_global_acc_ratio = 1.0;

    g_joint_vel_default = 1;
    g_joint_acc_default = 1;

    g_user_frame.identityMatrix();
    g_tool_frame.identityMatrix();
    g_user_frame_inverse.identityMatrix();
    g_tool_frame_inverse.identityMatrix();

    fst_parameter::ParamGroup param;

    g_dh_mat[0][0] = 0;
    g_dh_mat[0][1] = 0;
    g_dh_mat[0][2] = 330;
    g_dh_mat[0][3] = 0;
    g_dh_mat[1][0] = PI/2;
    g_dh_mat[1][1] = 50;
    g_dh_mat[1][2] = 0;
    g_dh_mat[1][3] = PI/2;
    g_dh_mat[2][0] = 0;
    g_dh_mat[2][1] = 330;
    g_dh_mat[2][2] = 0;
    g_dh_mat[2][3] = 0;
    g_dh_mat[3][0] = PI/2;
    g_dh_mat[3][1] = 35;
    g_dh_mat[3][2] = 330;
    g_dh_mat[3][3] = 0;
    g_dh_mat[4][0] = -PI/2;
    g_dh_mat[4][1] = 0;
    g_dh_mat[4][2] = 0;
    g_dh_mat[4][3] = 0;
    g_dh_mat[5][0] = PI/2;
    g_dh_mat[5][1] = 0;
    g_dh_mat[5][2] = 77.5;
    g_dh_mat[5][3] = 0;

    param.loadParamFile("share/configuration/configurable/soft_constraints.yaml");

    param.getParam("soft_constraint.j1.upper", g_soft_constraint.j1.upper);
    param.getParam("soft_constraint.j1.lower", g_soft_constraint.j1.lower);
    param.getParam("soft_constraint.j2.upper", g_soft_constraint.j2.upper);
    param.getParam("soft_constraint.j2.lower", g_soft_constraint.j2.lower);
    param.getParam("soft_constraint.j3.upper", g_soft_constraint.j3.upper);
    param.getParam("soft_constraint.j3.lower", g_soft_constraint.j3.lower);
    param.getParam("soft_constraint.j4.upper", g_soft_constraint.j4.upper);
    param.getParam("soft_constraint.j4.lower", g_soft_constraint.j4.lower);
    param.getParam("soft_constraint.j5.upper", g_soft_constraint.j5.upper);
    param.getParam("soft_constraint.j5.lower", g_soft_constraint.j5.lower);
    param.getParam("soft_constraint.j6.upper", g_soft_constraint.j6.upper);
    param.getParam("soft_constraint.j6.lower", g_soft_constraint.j6.lower);

    param.getParam("soft_constraint.j1.omega_max", g_soft_constraint.j1.max_omega);
    param.getParam("soft_constraint.j1.alpha_max", g_soft_constraint.j1.max_alpha);
    param.getParam("soft_constraint.j2.omega_max", g_soft_constraint.j2.max_omega);
    param.getParam("soft_constraint.j2.alpha_max", g_soft_constraint.j2.max_alpha);
    param.getParam("soft_constraint.j3.omega_max", g_soft_constraint.j3.max_omega);
    param.getParam("soft_constraint.j3.alpha_max", g_soft_constraint.j3.max_alpha);
    param.getParam("soft_constraint.j4.omega_max", g_soft_constraint.j4.max_omega);
    param.getParam("soft_constraint.j4.alpha_max", g_soft_constraint.j4.max_alpha);
    param.getParam("soft_constraint.j5.omega_max", g_soft_constraint.j5.max_omega);
    param.getParam("soft_constraint.j5.alpha_max", g_soft_constraint.j5.max_alpha);
    param.getParam("soft_constraint.j6.omega_max", g_soft_constraint.j6.max_omega);
    param.getParam("soft_constraint.j6.alpha_max", g_soft_constraint.j6.max_alpha);

/////////////////////////////////////////////////////////////////////////////////////
    

    Joint start_joint;
    start_joint.j1 = 0.0;
    start_joint.j2 = 0.0;
    start_joint.j3 = 0.0;
    start_joint.j4 = 0.0;
    start_joint.j5 = 0.0;
    start_joint.j6 = 0.0;

    MotionTarget target;
    target.type = MOTION_JOINT;
    target.cnt = 0;
    target.vel = 1;
    target.acc = 1;
    target.joint_target.j1 = 1.57;
    target.joint_target.j2 = -3.14;
    target.joint_target.j3 = 0.78;
    target.joint_target.j4 = 0;
    target.joint_target.j5 = 0;
    target.joint_target.j6 = 0;

    ErrorCode err = SUCCESS;

    ArmGroup arm;
    arm.initArmGroup();
    
    arm.setStartState(start_joint);
    arm.autoMove(target, 5);

    std::vector<JointOutput> points;
    arm.getPointFromFIFO(500, points);
    FST_INFO("get %d points", points.size());

    /*std::ofstream os("/home/fst/myworkspace/jout.txt");
    for (size_t i = 0; i < points.size(); i++)
    {
        os  << points[i].joint.j1 << ","
            << points[i].joint.j2 << ","
            << points[i].joint.j3 << ","
            << points[i].joint.j4 << ","
            << points[i].joint.j5 << ","
            << points[i].joint.j6 << endl;      
    }
    os.close();*/
    return 0;
}




