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
//#include <motion_plan_variable.h>


#define TRAJECTORY_FIFO_CAPACITY    500

#define ITEM_USED                   1
#define ITEM_UNUSED                 0

using fst_controller::Joint;
using fst_controller::JointPoint;
using fst_controller::JointConstraint;
using fst_controller::CurveMode;
using fst_controller::SmoothMode;
using fst_controller::DHGroup;


namespace fst_algorithm
{

double  g_cycle_time;
double  g_cycle_distance;

double  g_global_vel_ratio, g_global_acc_ratio;

double  g_cart_vel_default, g_cart_acc_default;
double  g_cart_vel_min,     g_cart_acc_min;
double  g_cart_vel_max,     g_cart_acc_max;

double  g_joint_vel_default;
double  g_joint_acc_default;

double  g_cart_acc_reference;
double  g_orientation_omega_reference;
double  g_ort_linear_polation_threshold;

//double  g_cart_omega_ref, g_cart_vel_ref;
//double  g_k_radius;

Matrix  g_user_frame;
Matrix  g_tool_frame;
Matrix  g_user_frame_inverse;
Matrix  g_tool_frame_inverse;

//CurveMode   g_curve_mode;
//SmoothMode  g_smooth_mode;

//size_t  g_pick_len;
Joint   g_ik_reference;

double  g_dh_mat[6][4];
DHGroup g_dh;

JointConstraint g_soft_constraint;
JointConstraint g_hard_constraint;
/*
JointPoint g_trajectory_fifo[TRAJECTORY_FIFO_CAPACITY];
bool g_trajectory_fifo_flag[TRAJECTORY_FIFO_CAPACITY];

std::atomic<size_t>  index_in;
std::atomic<size_t>  index_out;
*/
fst_log::Logger g_log;

/*
size_t getFifoCapacity(void)
{
    return TRAJECTORY_FIFO_CAPACITY;
}

size_t getFifoSize(void)
{
    size_t  in = index_in;
    size_t out = index_out;

    return in >= out ? in - out : in + TRAJECTORY_FIFO_CAPACITY - out;
}
*/



}

