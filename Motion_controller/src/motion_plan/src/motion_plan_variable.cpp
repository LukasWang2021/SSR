/*************************************************************************
	> File Name: motion_plan_variable.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年01月09日 星期二 10时52分06秒
 ************************************************************************/

#include <atomic>
#include <fst_datatype.h>
#include <motion_plan_matrix.h>
#include <log_manager/log_manager_logger.h>
#include <dynamics_interface.h>
//#include <motion_plan_variable.h>


#define TRAJECTORY_FIFO_CAPACITY    500

#define ITEM_USED                   1
#define ITEM_UNUSED                 0

using namespace fst_controller;


namespace fst_algorithm
{

double  g_cycle_time;
double  g_cycle_radian;
double  g_cycle_distance;

double  g_global_vel_ratio, g_global_acc_ratio;

double  g_cart_vel_default, g_cart_acc_default;
double  g_cart_vel_min,     g_cart_acc_min;
double  g_cart_vel_max,     g_cart_acc_max;

double  g_joint_vel_default;
double  g_joint_acc_default;

double  g_cart_vel_reference;
double  g_cart_acc_reference;
double  g_orientation_omega_reference;
double  g_orientation_alpha_reference;
double  g_ort_linear_polation_threshold;

double  g_manual_step_joint;
double  g_manual_step_position;
double  g_manual_step_orientation;


Matrix  g_user_frame;
Matrix  g_tool_frame;
Matrix  g_user_frame_inverse;
Matrix  g_tool_frame_inverse;

Joint   g_ik_reference;
Joint   g_start_joint;

DHGroup g_dh;
double  g_dh_mat[6][4];

double  g_omega_limit[9];
double  g_alpha_limit[9];

JointConstraint     g_soft_constraint;
JointConstraint     g_soft_constraint_limit;
JointConstraint     g_hard_constraint;

DynamicsInterface   g_dynamics_interface;
fst_log::Logger     g_log;

}

