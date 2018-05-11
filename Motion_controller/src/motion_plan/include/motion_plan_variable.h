/*************************************************************************
	> File Name: motion_plan_variable.h
	> Author: 
	> Mail: 
	> Created Time: 2018年01月09日 星期二 13时39分59秒
 ************************************************************************/

#ifndef _MOTION_PLAN_VARIABLE_H
#define _MOTION_PLAN_VARIABLE_H

#include <fst_datatype.h>
#include <motion_plan_matrix.h>
#include <log_manager/log_manager_logger.h>
#include <dynamics_interface.h>

namespace fst_algorithm
{

extern double   g_cycle_time;
extern double   g_cycle_radian;
extern double   g_cycle_distance;

extern double   g_global_vel_ratio, g_global_acc_ratio;
extern double   g_cart_vel_default, g_cart_acc_default;
extern double   g_cart_vel_min, g_cart_acc_min;
extern double   g_cart_vel_max, g_cart_acc_max;

extern double   g_joint_vel_default;
extern double   g_joint_acc_default;

extern double   g_cart_vel_reference;
extern double   g_cart_acc_reference;
extern double   g_orientation_omega_reference;
extern double   g_orientation_alpha_reference;
extern double   g_ort_linear_polation_threshold;

extern double  g_manual_step_joint;
extern double  g_manual_step_position;
extern double  g_manual_step_orientation;

extern Matrix   g_user_frame;
extern Matrix   g_tool_frame;
extern Matrix   g_user_frame_inverse;
extern Matrix   g_tool_frame_inverse;

extern fst_controller::Joint    g_ik_reference;
extern fst_controller::Joint    g_start_joint;
extern fst_controller::DHGroup  g_dh;
extern double   g_dh_mat[AXIS_IN_ALGORITHM][4];

extern double   g_omega_limit[AXIS_IN_ALGORITHM];
extern double   g_alpha_limit[AXIS_IN_ALGORITHM];

extern fst_controller::JointConstraint  g_soft_constraint;
extern fst_controller::JointConstraint  g_soft_constraint_limit;
extern fst_controller::JointConstraint  g_hard_constraint;

extern DynamicsInterface                g_dynamics_interface;
extern fst_log::Logger                  g_log;

}

#endif
