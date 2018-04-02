/**********************************************************************
    Copyright:  Foresight-Robotics
    File:       fst_controller.cpp
    Author:     Feng Yun
    Data:       Aug.1  2016
    Modify:     Aug.30 2016
    Description:ArmGroup--Source code.
**********************************************************************/

#include <string.h>
#include <float.h>
#include <fstream>

#include <motion_plan_version.h>
#include <motion_plan_arm_group.h>
#include <motion_plan_variable.h>
#include <motion_plan_basic_function.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <log_manager/log_manager_logger.h>
#include <motion_plan_reuse.h>
#include <motion_plan_kinematics.h>
#include <motion_plan_traj_plan.h>

using std::vector;
using std::string;
using fst_parameter::ParamGroup;
using fst_parameter::ParamValue;


using namespace fst_algorithm;

namespace fst_controller {

// -------------------------------------public function--------------------------------------------



//------------------------------------------------------------------------------
// Function:    ArmGroup
// Summary: The constructor of class
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
ArmGroup::ArmGroup(void)
{
    free_command_list_ptr_ = NULL;
    used_command_list_ptr_ = NULL;

    traj_head_ = 0;
    traj_tail_ = 0;

    path_head_  = 0;
    path_tail_  = 0;
    
    manual_running_ = false;
}


//------------------------------------------------------------------------------
// Function:    ~ArmGroup
// Summary: The destructor of class
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
ArmGroup::~ArmGroup(void)
{

}


//------------------------------------------------------------------------------
// Function:    initArmGroup
// Summary: Initial all the resources in this class
// In:      None
// Out:     None
// Return:  error code
//------------------------------------------------------------------------------
ErrorCode ArmGroup::initArmGroup(void)
{
    if (!g_log.initLogger("motion_plan"))
    {
        FST_ERROR("initArmGroup: fail to init log client.");
        return MOTION_FAIL_IN_INIT;
    }

    //g_log.setDisplayLevel(fst_log::MSG_LEVEL_ERROR);
    FST_INFO("Initializing ArmGroup ver%d.%d.%d ...",   motion_plan_VERSION_MAJOR,\
                                                        motion_plan_VERSION_MINOR,\
                                                        motion_plan_VERSION_PATCH);

    if (MOTION_COMMAND_POOL_CAPACITY > 1)
    {
        MotionCommand *ptr = NULL;
        motion_command_pool_[0].setPrevCommandPtr(ptr);
        motion_command_pool_[0].setNextCommandPtr(&motion_command_pool_[1]);

        for (int i = 1 ; i < MOTION_COMMAND_POOL_CAPACITY - 1; i++) {
            motion_command_pool_[i].setPrevCommandPtr(&motion_command_pool_[i - 1]);
            motion_command_pool_[i].setNextCommandPtr(&motion_command_pool_[i + 1]);
        }

        ptr = &motion_command_pool_[MOTION_COMMAND_POOL_CAPACITY - 2];
        motion_command_pool_[MOTION_COMMAND_POOL_CAPACITY - 1].setPrevCommandPtr(ptr);
        motion_command_pool_[MOTION_COMMAND_POOL_CAPACITY - 1].setNextCommandPtr(NULL);
        FST_INFO("Command pool capacity: %d", MOTION_COMMAND_POOL_CAPACITY);
    }
    else
    {
        FST_ERROR("The capacity (=%d) of motion command pool should greater than 1.", MOTION_COMMAND_POOL_CAPACITY);
        return INVALID_PARAMETER;
    }

    free_command_list_ptr_ = &motion_command_pool_[0];
    used_command_list_ptr_ = NULL;

    path_head_ = 0;
    path_tail_ = 0;
    traj_head_ = 0;
    traj_tail_ = 0;

    t_head_ = 0;
    t_tail_ = 0;

    prev_traj_point_.time_from_start = 0;
    prev_traj_point_.duration = 0;
    prev_traj_point_.expect_duration = 0;
    memset(prev_traj_point_.point.joint, 0, NUM_OF_JOINT * sizeof(double));
    memset(prev_traj_point_.point.omega, 0, NUM_OF_JOINT * sizeof(double));
    memset(prev_traj_point_.point.alpha, 0, NUM_OF_JOINT * sizeof(double));
    
    memset(&g_ik_reference, 0, NUM_OF_JOINT * sizeof(double));

    g_global_vel_ratio = 1.0;
    g_global_acc_ratio = 1.0;

    g_user_frame.identityMatrix();
    g_tool_frame.identityMatrix();
    g_user_frame_inverse.identityMatrix();
    g_tool_frame_inverse.identityMatrix();

    ParamGroup param;
    if (param.loadParamFile("share/configuration/configurable/motion_plan.yaml"))
    {
        param.getParam("cycle_time", g_cycle_time);
        param.getParam("cycle_radian", g_cycle_radian);
        param.getParam("cycle_distance", g_cycle_distance);
        param.getParam("cartesian/velocity/min", g_cart_vel_min);
        param.getParam("cartesian/velocity/max", g_cart_vel_max);
        param.getParam("cartesian/velocity/default", g_cart_vel_default);
        param.getParam("cartesian/velocity/reference", g_cart_vel_reference);
        param.getParam("cartesian/acceleration/min", g_cart_acc_min);
        param.getParam("cartesian/acceleration/max", g_cart_acc_max);
        param.getParam("cartesian/acceleration/default", g_cart_acc_default);
        param.getParam("cartesian/acceleration/reference", g_cart_acc_reference);
        param.getParam("joint/omega/default", g_joint_vel_default);
        param.getParam("joint/alpha/default", g_joint_acc_default);
        param.getParam("cartesian/orientation/omega_reference", g_orientation_omega_reference);
        param.getParam("cartesian/orientation/alpha_reference", g_orientation_alpha_reference);
        param.getParam("cartesian/orientation/linear_polation_threshold", g_ort_linear_polation_threshold);

        param.getParam("manual/step_length/joint", g_manual_step_joint);
        param.getParam("manual/step_length/position", g_manual_step_position);
        param.getParam("manual/step_length/orientation", g_manual_step_orientation);
        param.getParam("manual/speed_ratio/top", g_manual_top_speed_ratio);
        param.getParam("manual/speed_ratio/sub", g_manual_sub_speed_ratio);
        param.getParam("manual/acceleration_ratio/top", g_manual_top_acc_ratio);
        param.getParam("manual/acceleration_ratio/sub", g_manual_sub_acc_ratio);


        FST_INFO("cycle: time=%.4f, radian=%.4f, distance=%.4f", g_cycle_time, g_cycle_radian, g_cycle_distance);
        FST_INFO("cartesian velocity: min=%.2f, max=%.2f, default=%.2f",\
                 g_cart_vel_min, g_cart_vel_max, g_cart_vel_default);
        FST_INFO("cartesian acceleration: min=%.2f, max=%.2f, default=%.2f",\
                 g_cart_acc_min, g_cart_acc_max, g_cart_acc_default);
        FST_INFO("cartesian acceleration reference: %.2f", g_cart_acc_reference);
        FST_INFO("cartesian orientation omega reference: %.2f", g_orientation_omega_reference);
        FST_INFO("cartesian orientation linear polation threshold: %.2f", g_ort_linear_polation_threshold);
        FST_INFO("joint: omega default=%.2f, alpha default=%.2f", g_joint_vel_default, g_joint_acc_default);

        FST_INFO("manual step length: joint=%.4f rad, position=%.4f mm, orientation=%.4f rad",
                 g_manual_step_joint, g_manual_step_position, g_manual_step_orientation);
        FST_INFO("manual speed ratio: top=%.0f%%, sub=%.0f%%",
                 g_manual_top_speed_ratio * 100, g_manual_sub_speed_ratio * 100);
        FST_INFO("manual acceleration ratio: top=%.0f%%, sub=%.0f%%",
                 g_manual_top_acc_ratio * 100, g_manual_sub_acc_ratio * 100);
    }
    else
    {
        FST_ERROR("Lost config file: motion_plan.yaml");
        return param.getLastError();
    }

    if (param.loadParamFile("share/configuration/machine/dh.yaml"))
    {
        param.getParam("DH_parameter/j1/alpha", g_dh_mat[0][0]);
        param.getParam("DH_parameter/j1/a",     g_dh_mat[0][1]);
        param.getParam("DH_parameter/j1/d",     g_dh_mat[0][2]);
        param.getParam("DH_parameter/j1/theta", g_dh_mat[0][3]);
        param.getParam("DH_parameter/j2/alpha", g_dh_mat[1][0]);
        param.getParam("DH_parameter/j2/a",     g_dh_mat[1][1]);
        param.getParam("DH_parameter/j2/d",     g_dh_mat[1][2]);
        param.getParam("DH_parameter/j2/theta", g_dh_mat[1][3]);
        param.getParam("DH_parameter/j3/alpha", g_dh_mat[2][0]);
        param.getParam("DH_parameter/j3/a",     g_dh_mat[2][1]);
        param.getParam("DH_parameter/j3/d",     g_dh_mat[2][2]);
        param.getParam("DH_parameter/j3/theta", g_dh_mat[2][3]);
        param.getParam("DH_parameter/j4/alpha", g_dh_mat[3][0]);
        param.getParam("DH_parameter/j4/a",     g_dh_mat[3][1]);
        param.getParam("DH_parameter/j4/d",     g_dh_mat[3][2]);
        param.getParam("DH_parameter/j4/theta", g_dh_mat[3][3]);
        param.getParam("DH_parameter/j5/alpha", g_dh_mat[4][0]);
        param.getParam("DH_parameter/j5/a",     g_dh_mat[4][1]);
        param.getParam("DH_parameter/j5/d",     g_dh_mat[4][2]);
        param.getParam("DH_parameter/j5/theta", g_dh_mat[4][3]);
        param.getParam("DH_parameter/j6/alpha", g_dh_mat[5][0]);
        param.getParam("DH_parameter/j6/a",     g_dh_mat[5][1]);
        param.getParam("DH_parameter/j6/d",     g_dh_mat[5][2]);
        param.getParam("DH_parameter/j6/theta", g_dh_mat[5][3]);

        FST_INFO("DH parameters:");
        FST_INFO("  J1: %.6f, %.6f, %.6f, %.6f", g_dh_mat[0][0], g_dh_mat[0][1], g_dh_mat[0][2], g_dh_mat[0][3]);
        FST_INFO("  J2: %.6f, %.6f, %.6f, %.6f", g_dh_mat[1][0], g_dh_mat[1][1], g_dh_mat[1][2], g_dh_mat[1][3]);
        FST_INFO("  J3: %.6f, %.6f, %.6f, %.6f", g_dh_mat[2][0], g_dh_mat[2][1], g_dh_mat[2][2], g_dh_mat[2][3]);
        FST_INFO("  J4: %.6f, %.6f, %.6f, %.6f", g_dh_mat[3][0], g_dh_mat[3][1], g_dh_mat[3][2], g_dh_mat[3][3]);
        FST_INFO("  J5: %.6f, %.6f, %.6f, %.6f", g_dh_mat[4][0], g_dh_mat[4][1], g_dh_mat[4][2], g_dh_mat[4][3]);
        FST_INFO("  J6: %.6f, %.6f, %.6f, %.6f", g_dh_mat[5][0], g_dh_mat[5][1], g_dh_mat[5][2], g_dh_mat[5][3]);
    }
    else
    {
        FST_ERROR("Lost config file: dh.yaml");
        return param.getLastError();
    }

    if (param.loadParamFile("share/configuration/model/hard_constraints.yaml"))
    {
        param.getParam("hard_constraint/j1/home", g_hard_constraint.j1.home);
        param.getParam("hard_constraint/j1/upper", g_hard_constraint.j1.upper);
        param.getParam("hard_constraint/j1/lower", g_hard_constraint.j1.lower);
        param.getParam("hard_constraint/j1/omega_max", g_hard_constraint.j1.max_omega);
        param.getParam("hard_constraint/j1/alpha_max", g_hard_constraint.j1.max_alpha);
        param.getParam("hard_constraint/j2/home", g_hard_constraint.j2.home);
        param.getParam("hard_constraint/j2/upper", g_hard_constraint.j2.upper);
        param.getParam("hard_constraint/j2/lower", g_hard_constraint.j2.lower);
        param.getParam("hard_constraint/j2/omega_max", g_hard_constraint.j2.max_omega);
        param.getParam("hard_constraint/j2/alpha_max", g_hard_constraint.j2.max_alpha);
        param.getParam("hard_constraint/j3/home", g_hard_constraint.j3.home);
        param.getParam("hard_constraint/j3/upper", g_hard_constraint.j3.upper);
        param.getParam("hard_constraint/j3/lower", g_hard_constraint.j3.lower);
        param.getParam("hard_constraint/j3/omega_max", g_hard_constraint.j3.max_omega);
        param.getParam("hard_constraint/j3/alpha_max", g_hard_constraint.j3.max_alpha);
        param.getParam("hard_constraint/j4/home", g_hard_constraint.j4.home);
        param.getParam("hard_constraint/j4/upper", g_hard_constraint.j4.upper);
        param.getParam("hard_constraint/j4/lower", g_hard_constraint.j4.lower);
        param.getParam("hard_constraint/j4/omega_max", g_hard_constraint.j4.max_omega);
        param.getParam("hard_constraint/j4/alpha_max", g_hard_constraint.j4.max_alpha);
        param.getParam("hard_constraint/j5/home", g_hard_constraint.j5.home);
        param.getParam("hard_constraint/j5/upper", g_hard_constraint.j5.upper);
        param.getParam("hard_constraint/j5/lower", g_hard_constraint.j5.lower);
        param.getParam("hard_constraint/j5/omega_max", g_hard_constraint.j5.max_omega);
        param.getParam("hard_constraint/j5/alpha_max", g_hard_constraint.j5.max_alpha);
        param.getParam("hard_constraint/j6/home", g_hard_constraint.j6.home);
        param.getParam("hard_constraint/j6/upper", g_hard_constraint.j6.upper);
        param.getParam("hard_constraint/j6/lower", g_hard_constraint.j6.lower);
        param.getParam("hard_constraint/j6/omega_max", g_hard_constraint.j6.max_omega);
        param.getParam("hard_constraint/j6/alpha_max", g_hard_constraint.j6.max_alpha);
    }
    else
    {
        FST_ERROR("Lost config file: hard_constraints.yaml");
        return param.getLastError();
    }

    if (param.loadParamFile("share/configuration/configurable/soft_constraints.yaml"))
    {
        param.getParam("soft_constraint/j1/home", g_soft_constraint.j1.home);
        param.getParam("soft_constraint/j1/upper", g_soft_constraint.j1.upper);
        param.getParam("soft_constraint/j1/lower", g_soft_constraint.j1.lower);
        param.getParam("soft_constraint/j1/omega_max", g_soft_constraint.j1.max_omega);
        param.getParam("soft_constraint/j1/alpha_max", g_soft_constraint.j1.max_alpha);
        param.getParam("soft_constraint/j2/home", g_soft_constraint.j2.home);
        param.getParam("soft_constraint/j2/upper", g_soft_constraint.j2.upper);
        param.getParam("soft_constraint/j2/lower", g_soft_constraint.j2.lower);
        param.getParam("soft_constraint/j2/omega_max", g_soft_constraint.j2.max_omega);
        param.getParam("soft_constraint/j2/alpha_max", g_soft_constraint.j2.max_alpha);
        param.getParam("soft_constraint/j3/home", g_soft_constraint.j3.home);
        param.getParam("soft_constraint/j3/upper", g_soft_constraint.j3.upper);
        param.getParam("soft_constraint/j3/lower", g_soft_constraint.j3.lower);
        param.getParam("soft_constraint/j3/omega_max", g_soft_constraint.j3.max_omega);
        param.getParam("soft_constraint/j3/alpha_max", g_soft_constraint.j3.max_alpha);
        param.getParam("soft_constraint/j4/home", g_soft_constraint.j4.home);
        param.getParam("soft_constraint/j4/upper", g_soft_constraint.j4.upper);
        param.getParam("soft_constraint/j4/lower", g_soft_constraint.j4.lower);
        param.getParam("soft_constraint/j4/omega_max", g_soft_constraint.j4.max_omega);
        param.getParam("soft_constraint/j4/alpha_max", g_soft_constraint.j4.max_alpha);
        param.getParam("soft_constraint/j5/home", g_soft_constraint.j5.home);
        param.getParam("soft_constraint/j5/upper", g_soft_constraint.j5.upper);
        param.getParam("soft_constraint/j5/lower", g_soft_constraint.j5.lower);
        param.getParam("soft_constraint/j5/omega_max", g_soft_constraint.j5.max_omega);
        param.getParam("soft_constraint/j5/alpha_max", g_soft_constraint.j5.max_alpha);
        param.getParam("soft_constraint/j6/home", g_soft_constraint.j6.home);
        param.getParam("soft_constraint/j6/upper", g_soft_constraint.j6.upper);
        param.getParam("soft_constraint/j6/lower", g_soft_constraint.j6.lower);
        param.getParam("soft_constraint/j6/omega_max", g_soft_constraint.j6.max_omega);
        param.getParam("soft_constraint/j6/alpha_max", g_soft_constraint.j6.max_alpha);
    }
    else
    {
        FST_ERROR("Lost config file: soft_constraints.yaml");
        return param.getLastError();
    }

    FST_INFO("Hard constraint:");
    FST_INFO("J1 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_hard_constraint.j1.lower, g_hard_constraint.j1.home, g_hard_constraint.j1.upper,\
             g_hard_constraint.j1.max_omega, g_hard_constraint.j1.max_alpha);
    FST_INFO("J2 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_hard_constraint.j2.lower, g_hard_constraint.j2.home, g_hard_constraint.j2.upper,\
             g_hard_constraint.j2.max_omega, g_hard_constraint.j2.max_alpha);
    FST_INFO("J3 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_hard_constraint.j3.lower, g_hard_constraint.j3.home, g_hard_constraint.j3.upper,\
             g_hard_constraint.j3.max_omega, g_hard_constraint.j3.max_alpha);
    FST_INFO("J4 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_hard_constraint.j4.lower, g_hard_constraint.j4.home, g_hard_constraint.j4.upper,\
             g_hard_constraint.j4.max_omega, g_hard_constraint.j4.max_alpha);
    FST_INFO("J5 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_hard_constraint.j5.lower, g_hard_constraint.j5.home, g_hard_constraint.j5.upper,\
             g_hard_constraint.j5.max_omega, g_hard_constraint.j5.max_alpha);
    FST_INFO("J6 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_hard_constraint.j6.lower, g_hard_constraint.j6.home, g_hard_constraint.j6.upper,\
             g_hard_constraint.j6.max_omega, g_hard_constraint.j6.max_alpha);

    FST_INFO("Soft constraint:");
    FST_INFO("J1 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_soft_constraint.j1.lower, g_soft_constraint.j1.home, g_soft_constraint.j1.upper,\
             g_soft_constraint.j1.max_omega, g_soft_constraint.j1.max_alpha);
    FST_INFO("J2 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_soft_constraint.j2.lower, g_soft_constraint.j2.home, g_soft_constraint.j2.upper,\
             g_soft_constraint.j2.max_omega, g_soft_constraint.j2.max_alpha);
    FST_INFO("J3 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_soft_constraint.j3.lower, g_soft_constraint.j3.home, g_soft_constraint.j3.upper,\
             g_soft_constraint.j3.max_omega, g_soft_constraint.j3.max_alpha);
    FST_INFO("J4 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_soft_constraint.j4.lower, g_soft_constraint.j4.home, g_soft_constraint.j4.upper,\
             g_soft_constraint.j4.max_omega, g_soft_constraint.j4.max_alpha);
    FST_INFO("J5 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_soft_constraint.j5.lower, g_soft_constraint.j5.home, g_soft_constraint.j5.upper,\
             g_soft_constraint.j5.max_omega, g_soft_constraint.j5.max_alpha);
    FST_INFO("J6 - lower=%.6f, home=%.6f, upper=%.6f, omega=%.6f, alpha=%.6f",\
             g_soft_constraint.j6.lower, g_soft_constraint.j6.home, g_soft_constraint.j6.upper,\
             g_soft_constraint.j6.max_omega, g_soft_constraint.j6.max_alpha);

    if (!isFirstConstraintCoveredBySecond(g_soft_constraint, g_hard_constraint))
    {
        FST_ERROR("Soft constraint out of hard constraint.");
        return INVALID_PARAMETER;
    }

    FST_INFO("ArmGroup is ready to make life easier. Have a good time!");
    FST_WARN("**********************************************************************************************");
    
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getVersion
// Summary: To get current version of motion_plan.
// In:      None
// Out:     None
// Return:  version string
//------------------------------------------------------------
string ArmGroup::getVersion(void)
{
    char buf[32];
    sprintf(buf, "ArmGroup Ver%d.%d.%d", motion_plan_VERSION_MAJOR,
                                         motion_plan_VERSION_MINOR,
                                         motion_plan_VERSION_PATCH);
    return string(buf);
}

//------------------------------------------------------------
// Function:    getCycleTime
// Summary: To get the cycle time used in interpolation.
// In:      None
// Out:     None
// Return:  cycle time
//------------------------------------------------------------
double ArmGroup::getCycleTime(void)
{
    return g_cycle_time;
}

//------------------------------------------------------------
// Function:    getCycleRadian
// Summary: To get the cycle radian used in path plan.
// In:      None
// Out:     None
// Return:  cycle radian
//------------------------------------------------------------
double ArmGroup::getCycleRadian(void)
{
    return g_cycle_radian;
}

//------------------------------------------------------------
// Function:    getCycleDistance
// Summary: To get the cycle distance used in path plan.
// In:      None
// Out:     None
// Return:  cycle distance
//------------------------------------------------------------
double ArmGroup::getCycleDistance(void)
{
    return g_cycle_distance;
}

//------------------------------------------------------------
// Function:    getCartesianAccMax
// Summary: To get max acceleration in cartesian space.
// In:      None
// Out:     None
// Return:  value of max acceleration in cartesian space
//------------------------------------------------------------
double ArmGroup::getCartesianAccMax(void)
{
    return g_cart_acc_max;
}

//------------------------------------------------------------
// Function:    getCartesianVelMax
// Summary: To get max velocity in cartesian space.
// In:      None
// Out:     None
// Return:  value of max velocity in cartesian space
//------------------------------------------------------------
double ArmGroup::getCartesianVelMax(void)
{
    return g_cart_vel_max;
}

//------------------------------------------------------------
// Function:    getGlobalVelRatio
// Summary: To get global velocity ratio in cartesian and joint space.
// In:      None
// Out:     None
// Return:  global velocity ratio in cartesian space
//          range: 0.0 ~ 1.0
//------------------------------------------------------------
double ArmGroup::getGlobalVelRatio(void)
{
    return g_global_vel_ratio;
}

//------------------------------------------------------------
// Function:    setGlobalVelRatio
// Summary: To set global velocity ratio in cartesian and joint space.
// In:      ratio -> global velocity ratio in cartesian space
//                   range: 0.0 ~ 1.0
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setGlobalVelRatio(double ratio)
{
    if (ratio < 0 || ratio > 1)
    {
        return INVALID_PARAMETER;
    }
    else
    {
        g_global_vel_ratio = ratio;
        return SUCCESS;
    }
}

//------------------------------------------------------------
// Function:    getGlobalAccRatio
// Summary: To get global acceleration ratio in cartesian and joint space.
// In:      None
// Out:     None
// Return:  global acceleration ratio in cartesian space
//          range: 0.0 ~ 3.0
//------------------------------------------------------------
double ArmGroup::getGlobalAccRatio(void)
{ 
    return g_global_acc_ratio;
}

//------------------------------------------------------------
// Function:    setGlobalAccRatio
// Summary: To set global acceleration ratio in cartesian and joint space.
// In:      ratio -> global acceleration ratio in cartesian space
//                   range: 0.0 ~ 3.0
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setGlobalAccRatio(double ratio)
{
    if (ratio < 0 || ratio > 3)
    {
        return INVALID_PARAMETER;
    }
    else
    {
        g_global_acc_ratio = ratio;
        return SUCCESS;
    }
}

//------------------------------------------------------------
// Function:    getCartesianAccDefault
// Summary: To get default acceleration in cartesian space.
// In:      None
// Out:     None
// Return:  cartesian acc, unit: mm/(s*s)
//------------------------------------------------------------
double ArmGroup::getCartesianAccDefault(void)
{
    return g_cart_acc_default;
}

//------------------------------------------------------------
// Function:    setCartesianAccDefault
// Summary: To set default acceleration in cartesian space.
// In:      acc -> cartesian acceleration, unit: mm/(s*s)
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setCartesianAccDefault(double acc)
{
    if (acc < 0 || acc > g_cart_acc_max)
    {
        return INVALID_PARAMETER;
    }
    else
    {
        g_cart_acc_default = acc;
        return SUCCESS;
    }
}

//------------------------------------------------------------
// Function:    getCartesianVelDefault
// Summary: To get default velocity in cartesian space.
// In:      None
// Out:     None
// Return:  cartesian vel, unit: mm/s
//------------------------------------------------------------
double ArmGroup::getCartesianVelDefault(void)
{
    return g_cart_vel_default;
}

//------------------------------------------------------------
// Function:    setCartesianVelDefault
// Summary: To set default velocity in cartesian space.
// In:      vel -> cartesian acceleration, unit: mm/s
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setCartesianVelDefault(double vel)
{
    if (vel < 0 || vel > g_cart_vel_max)
    {
        return INVALID_PARAMETER;
    }
    else
    {
        g_cart_vel_default = vel;
        return SUCCESS;
    }
}

//------------------------------------------------------------
// Function:    getSoftConstraint
// Summary: To get soft joint constraint from algorithm.
// In:      None
// Out:     None
// Return:  soft constraint
//------------------------------------------------------------
const JointConstraint& ArmGroup::getSoftConstraint(void)
{
    return g_soft_constraint;
}

//------------------------------------------------------------
// Function:    setSoftConstraint
// Summary: To set soft joint constraint to algorithm and config file.
// In:      cons -> soft joint constraint
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setSoftConstraint(const JointConstraint &cons)
{
    if (isFirstConstraintCoveredBySecond(cons, g_hard_constraint))
    {
        g_soft_constraint = cons;
        return SUCCESS;
    }
    else
    {
        return INVALID_PARAMETER;
    }
}

//------------------------------------------------------------
// Function:    getHardConstraint
// Summary: To get hard joint constraint from algorithm.
// In:      None
// Out:     None
// Return:  hard constraint
//------------------------------------------------------------
const JointConstraint& ArmGroup::getHardConstraint(void)
{
    return g_hard_constraint;
}

//------------------------------------------------------------
// Function:    getDH
// Summary: To get DH parameter group from algorithm.
// In:      None
// Out:     None
// Return:  DH parameter group
//------------------------------------------------------------
const DHGroup& ArmGroup::getDH(void)
{
    return g_dh;
}

//------------------------------------------------------------
// Function:    getFIFOLength
// Summary: To get the length of trajectory FIFO.
// In:      None
// Out:     None
// Return:  length of the FIFO
//------------------------------------------------------------
size_t ArmGroup::getFIFOLength(void)
{
    static size_t cnt = 0;
    size_t len = 0;

    if (auto_running_)
    {
        //cnt++;
        //if (cnt > 1000)
        //{
        //    cnt = 0;
            //FST_INFO("pick seg=%d, t_tail=%d", pick_segment_, t_tail_);
            //FST_INFO("pick time=%f, last seg time=%f", pick_time_, t_path_[t_tail_ - 1].time_from_start);
            if (pick_segment_ < t_tail_)
            {
                len = ceil((t_path_[t_tail_ - 1].time_from_start - pick_time_) / g_cycle_time);
            }
            else
            {
                len = 0;
            }
        //}
    }
    else if (manual_running_)
    {
        if (g_manual_frame == JOINT)
        {
            if (g_manual_pick_time < g_manual_joint_coeff[0].duration_3)
            {
                len = ceil((g_manual_joint_coeff[0].duration_3 - g_manual_pick_time) / g_cycle_time);
            }
            else
            {
                len = 0;
            }
        }
        else
        {
            if (g_manual_pick_time < g_manual_cartesian_coeff[0].duration_3)
            {
                //FST_INFO("cartesian: pick time=%f, duration=%f", g_manual_pick_time, g_manual_cartesian_coeff[0].duration_3);
                len = ceil((g_manual_cartesian_coeff[0].duration_3 - g_manual_pick_time) / g_cycle_time);
            }
            else
            {
                //FST_INFO("cartesian: pick time=%f, duration=%f", g_manual_pick_time, g_manual_cartesian_coeff[0].duration_3);
                len = 0;
            }
        }
    }

    if (len > 0 && len < 100)
    {
        FST_INFO("pick seg=%d, t_tail=%d", pick_segment_, t_tail_);
        FST_INFO("auto: pick time=%f, last seg time=%f, len=%d", pick_time_, t_path_[t_tail_ - 1].time_from_start, len);
        FST_INFO("manual: pick time=%f, total time=%f, len=%d", g_manual_pick_time, g_manual_joint_coeff[0].duration_3, len);
    }
    
    return len;
}

//------------------------------------------------------------
// Function:    getFIFOCapacity
// Summary: To get the capacity of trajectory FIFO.
// In:      None
// Out:     None
// Return:  capacity of the FIFO
//------------------------------------------------------------
size_t ArmGroup::getFIFOCapacity(void)
{
    // TODO
    return 0;
}

//------------------------------------------------------------
// Function:    getPointFromFIFO
// Summary: To get points from trajectory FIFO.
// In:      num -> number of joint points want to get
// Out:     points -> points from trajectory FIFO
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::getPointFromFIFO(size_t num, std::vector<JointOutput> &points)
{
    points.clear();

    FST_WARN("getPointFromFIFO: auto = %d, manul = %d", auto_running_, manual_running_);

    if (auto_running_)
    {
        return pickFromAuto(num, points);
    }
    else if (manual_running_)
    {
        return pickFromManual(num, points);
    }

    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getPointFromFIFO
// Summary: To get points from trajectory FIFO.
// In:      num -> number of joint points want to get
// Out:     points -> points from trajectory FIFO
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::getPointFromFIFO(size_t num, JointOutput *points)
{
    // TODO
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getPointFromFIFO
// Summary: To get 10 points from trajectory FIFO.
// In:      None
// Out:     points -> points from trajectory FIFO
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::getPointFromFIFO(vector<JointOutput> &points)
{
    // TODO
    return SUCCESS;
}

/*
//------------------------------------------------------------
// Function:    pickPointToFIFO
// Summary: To pick points from motion command and put the points 
//          into trajectory FIFO.
// In:      num -> number of points want to pick
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::pickPointToFIFO(size_t num)
{
    return SUCCESS;
}
*/

//------------------------------------------------------------
// Function:    getToolFrame
// Summary: To get current tool frame in algorithm.
// In:      None
// Out:     None
// Return:  current tool frame
//------------------------------------------------------------
const Transformation ArmGroup::getToolFrame(void)
{
    Transformation tf;
    g_tool_frame.toPoseEuler(tf);
    return tf;
}

//------------------------------------------------------------
// Function:    setToolFrame
// Summary: To set current tool frame.
// In:      frame -> current tool frame
// Out:     None
// Return:  None
//------------------------------------------------------------
void ArmGroup::setToolFrame(const Transformation &tf)
{
    g_tool_frame = tf;
}

//------------------------------------------------------------
// Function:    getUserFrame
// Summary: To get current user frame in algorithm.
// In:      None
// Out:     None
// Return:  current user frame
//------------------------------------------------------------
const Transformation ArmGroup::getUserFrame(void)
{
    Transformation uf;
    g_tool_frame.toPoseEuler(uf);
    return uf;
}

//------------------------------------------------------------
// Function:    setUserFrame
// Summary: To set current user frame.
// In:      frame -> current user frame
// Out:     None
// Return:  None
//------------------------------------------------------------
void ArmGroup::setUserFrame(const Transformation &uf)
{
    g_user_frame = uf;
}

//------------------------------------------------------------
// Function:    setStartState
// Summary: To set robot start state.
// In:      joint -> robot start state
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setStartState(const Joint &joint)
{
    if (traj_head_ != traj_tail_ || path_head_ != path_tail_)
    {
        FST_ERROR("setStartState: cannot set start state until ArmGroup is standing by.");
        return INVALID_SEQUENCE;
    }

    if (isJointInConstraint(joint, g_soft_constraint))
    {
        g_start_joint       = joint;
        g_ik_reference      = joint;

        memcpy(prev_traj_point_.point.joint, &joint, NUM_OF_JOINT * sizeof(double));
        memset(prev_traj_point_.point.omega, 0, NUM_OF_JOINT * sizeof(double));
        memset(prev_traj_point_.point.alpha, 0, NUM_OF_JOINT * sizeof(double));

        forwardKinematics(joint, prev_traj_point_.path_point.pose);

        FST_INFO("setStartState: joint=%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                 prev_traj_point_.point.joint[0],
                 prev_traj_point_.point.joint[1],
                 prev_traj_point_.point.joint[2],
                 prev_traj_point_.point.joint[3],
                 prev_traj_point_.point.joint[4],
                 prev_traj_point_.point.joint[5],
                 prev_traj_point_.point.joint[6],
                 prev_traj_point_.point.joint[7],
                 prev_traj_point_.point.joint[8]);
        
        FST_INFO("setStartState: omega=%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                 prev_traj_point_.point.omega[0],
                 prev_traj_point_.point.omega[1],
                 prev_traj_point_.point.omega[2],
                 prev_traj_point_.point.omega[3],
                 prev_traj_point_.point.omega[4],
                 prev_traj_point_.point.omega[5],
                 prev_traj_point_.point.omega[6],
                 prev_traj_point_.point.omega[7],
                 prev_traj_point_.point.omega[8]);

        FST_INFO("setStartState: alpha=%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                 prev_traj_point_.point.alpha[0],
                 prev_traj_point_.point.alpha[1],
                 prev_traj_point_.point.alpha[2],
                 prev_traj_point_.point.alpha[3],
                 prev_traj_point_.point.alpha[4],
                 prev_traj_point_.point.alpha[5],
                 prev_traj_point_.point.alpha[6],
                 prev_traj_point_.point.alpha[7],
                 prev_traj_point_.point.alpha[8]);

        return SUCCESS;
    }
    else
    {
        FST_ERROR("setStartState: given joint is out of soft constraint");
        
        JointConstraint &cons = g_soft_constraint;
        int res = contrastJointWithConstraint(joint, g_soft_constraint);
        
        if ((res >> 0) % 2 == 0)    FST_INFO ("  j1=%.6f - (%.6f, %.6f)", joint.j1, cons.j1.lower, cons.j1.upper);
        else                        FST_ERROR("  j1=%.6f - (%.6f, %.6f)", joint.j1, cons.j1.lower, cons.j1.upper);
        if ((res >> 1) % 2 == 0)    FST_INFO ("  j2=%.6f - (%.6f, %.6f)", joint.j2, cons.j2.lower, cons.j2.upper);
        else                        FST_ERROR("  j2=%.6f - (%.6f, %.6f)", joint.j2, cons.j2.lower, cons.j2.upper);
        if ((res >> 2) % 2 == 0)    FST_INFO ("  j3=%.6f - (%.6f, %.6f)", joint.j3, cons.j3.lower, cons.j3.upper);
        else                        FST_ERROR("  j3=%.6f - (%.6f, %.6f)", joint.j3, cons.j3.lower, cons.j3.upper);
        if ((res >> 3) % 2 == 0)    FST_INFO ("  j4=%.6f - (%.6f, %.6f)", joint.j4, cons.j4.lower, cons.j4.upper);
        else                        FST_ERROR("  j4=%.6f - (%.6f, %.6f)", joint.j4, cons.j4.lower, cons.j4.upper);
        if ((res >> 4) % 2 == 0)    FST_INFO ("  j5=%.6f - (%.6f, %.6f)", joint.j5, cons.j5.lower, cons.j5.upper);
        else                        FST_ERROR("  j5=%.6f - (%.6f, %.6f)", joint.j5, cons.j5.lower, cons.j5.upper);
        if ((res >> 5) % 2 == 0)    FST_INFO ("  j6=%.6f - (%.6f, %.6f)", joint.j6, cons.j6.lower, cons.j6.upper);
        else                        FST_ERROR("  j6=%.6f - (%.6f, %.6f)", joint.j6, cons.j6.lower, cons.j6.upper);
        
        return JOINT_OUT_OF_CONSTRAINT;
    }
}

//------------------------------------------------------------
// Function:    getJointFromPose
// Summary: To compute IK with a given pose in cartesian space,
//          without reference joint and accessibility check.
// In:      poes    -> the pose in cartesian space needed to compute IK
// Out:     joint   -> IK result
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::getJointFromPose(const PoseEuler &pose, Joint &joint)
{
    return inverseKinematics(pose, g_ik_reference, joint);
}

//------------------------------------------------------------
// Function:    getPoseFromJoint
// Summary: To compute FK with a given point in joint space.
// In:      joint   -> the point in joint space needed to compute FK.
// Out:     pose    -> FK result
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::getPoseFromJoint(const Joint &joint, PoseEuler &pose)
{
    forwardKinematics(joint, pose);
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getPoseFromJointInWorld
// Summary: To get the pose of flange and tcp from a given point
//          in joint space in world coordinate.
// In:      joint   -> the point in joint space needed to compute FK.
// Out:     flange  -> flange pose in world coordinate
//          tcp     -> tcp pose in world coordinate
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::getPoseFromJointInWorld(const Joint &joint, PoseEuler &flange, PoseEuler &tcp)
{
    // TODO
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getRemainingTime
// Summary: To get the remaining time of current motion.
// In:      None
// Out:     None
// Return:  remaining time
//------------------------------------------------------------
MotionTime ArmGroup::getRemainingTime(void)
{
    MotionTime tm = 0;

    if (auto_running_)
    {
        //FST_INFO("pick seg=%d, t_tail=%d", pick_segment_, t_tail_);
        if (pick_segment_ < t_tail_)
        {
            //FST_INFO("pick time=%f, last seg time=%f", pick_time_, t_path_[t_tail_ - 1].time_from_start);
            tm = t_path_[t_tail_ - 1].time_from_start - pick_time_;
        }
    }
    else if (manual_running_)
    {
        if (g_manual_frame == JOINT)
        {
            if (g_manual_pick_time < g_manual_joint_coeff[0].duration_3)
            {
                tm = g_manual_joint_coeff[0].duration_3 - g_manual_pick_time;
            }
        }
        else
        {
            if (g_manual_pick_time < g_manual_cartesian_coeff[0].duration_3)
            {
                //FST_INFO("cartesian: pick time=%f, duration=%f", g_manual_pick_time, g_manual_cartesian_coeff[0].duration_3);
                tm = g_manual_cartesian_coeff[0].duration_3 - g_manual_pick_time;
            }
        }
    }

    return tm;
}

//------------------------------------------------------------
// Function:    suspendMotion
// Summary: To replan a slow-down trajectory upon trajectory FIFO
//          and stop the robot motion. Used when pause event or IK
//          failure raised etc.
// In:      None
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::suspendMotion(void)
{
    //TODO
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    declareESTOP
// Summary: Declare an ESTOP event to arm_group, this function
//          should be called after the robot is stopped.
// In:      joint   -> stopped robot is standing on this joint
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::declareESTOP(const Joint &joint)
{
    //TODO
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    resumeMotion
// Summary: To replan a start-up trajectory and resume robot
//          motion from suspend or ESTOP state.
// In:      None
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::resumeMotion(void)
{
    //TODO
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    resetArmGroup
// Summary: Return wether joint is in soft constraint or not.
// In:      joint   ->
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::resetArmGroup(const Joint &joint)
{
    //TODO
    FST_INFO("resetArmGroup -------------");
    return isJointInSoftConstraint(joint) ? SUCCESS : JOINT_OUT_OF_CONSTRAINT;
}

//------------------------------------------------------------
// Function:    clearArmGroup
// Summary: To reset resources in arm group, FIFO and state etc.
// In:      None
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::clearArmGroup(void)
{
    //TODO
    FST_INFO("clearArmGroup -------------");

    return SUCCESS;
}

//------------------------------------------------------------
// Function:    isJointInConstraint
// Summary: Retrun true if given joint fall into the soft constraint
// In:      joint -> joint to be compared with soft constraint
// Out:     None
// Return:  true  -> joint in soft constraint
//          false -> joint not in soft constraint
//------------------------------------------------------------
bool ArmGroup::isJointInSoftConstraint(const Joint &joint)
{
    return isJointInConstraint(joint, g_soft_constraint);
}

//------------------------------------------------------------
// Function:    autoMove
// Summary: Plan an auto move command (moveJ/moveL/moveC) to reach
//          the target without smooth. If FIFO is empty at the
//          moment, then fill the FIFO with points from this command.
// In:      target -> motion target
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::autoMove(const MotionTarget &target, int id)
{
    ErrorCode err = SUCCESS;

    if (target.type == MOTION_JOINT)
    {
        err = autoJoint(target, id);
    }
    else if (target.type == MOTION_LINE)
    {
         err = autoLine(target, id);
    }
    else if (target.type == MOTION_CIRCLE)
    {
        //return autoMoveCircle(target, id);
        // TODO

        FST_ERROR("autoMove: moveC is unsupported now");
        err = MOTION_INTERNAL_FAULT;
    }
    else
    {
        FST_ERROR("autoMove: unsupported motion type (=%d)", target.type);
        return INVALID_PARAMETER;
    }

    if (err == SUCCESS)
    {
        auto_running_ = true;
        prev_traj_point_ = t_path_[t_tail_ - 1];
        prev_traj_point_.time_from_start = 0;
        if (prev_traj_point_.path_point.type == MOTION_JOINT)
        {
            FST_WARN("rebuild prev_traj_point");
            forwardKinematics(t_path_[t_tail_ - 1].path_point.joint, prev_traj_point_.path_point.pose);
            prev_traj_point_.path_point.type = MOTION_LINE;
        }
    }

    return err;
}

//------------------------------------------------------------
// Function:    manualMove
// Summary: Plan a manual move trajectory (Joint/Line) with given
//          direction. If FIFO is empty at the moment, fill it.
// In:      directions -> manual direction
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::manualMove(const vector<ManualDirection> &directions)
{
    FST_INFO("manual_running_=%d", manual_running_);
    if (manual_running_)
    {
        FST_WARN("manualMove: last manual command is running, waiting before its finishing ...");
        return SUCCESS;
    }

    if (directions.size() == 6)
    {
        ManualDirection dirs[6] = {directions[0], directions[1], directions[2], directions[3], directions[4], directions[5]};
        
        ErrorCode err = manual_.stepTeach(dirs);
        
        if (err == SUCCESS)
        {
            manual_running_ = true;
            return SUCCESS;
        }
        else
        {
            FST_ERROR("manualMove: failed, err=0x%llx", err);
            return err;
        }
    }
    else
    {
        FST_ERROR("manualMove: 6 manual directions needed, %d given", directions.size());
        return INVALID_PARAMETER;
    }
}

//------------------------------------------------------------
// Function:    manualMove
// Summary: Plan a manual move trajectory (Joint/Line) with given
//          point. If FIFO is empty at the moment, then fill it.
// In:      joint -> manual target in joint space
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::manualMove(const Joint &joint)
{
    if (manual_running_ == false)
    {
        ErrorCode err = manual_.stepTeach(joint);
        
        if (err == SUCCESS)
        {
            manual_running_ = true;
            return SUCCESS;
        }
        else
        {
            FST_ERROR("manualMove: failed, err=0x%x", err);
            return err;
        }
    }
    else
    {
        FST_ERROR("manualMove: last manual command is running");
        return INVALID_SEQUENCE;
    }
}

//------------------------------------------------------------
// Function:    manualMove
// Summary: Plan a manual move trajectory (Joint/Line) with given
//          point. If FIFO is empty at the moment, then fill it.
// In:      pose  -> manual target in cartesian space
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::manualMove(const PoseEuler &pose)
{
    // TODO
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    setManualFrame
// Summary: To set manual frame mode.
// In:      frame   -> manual frame mode, JOINT/WORLD/USER/TOOL
// Out:     None
// Return:  None
//------------------------------------------------------------
void ArmGroup::setManualFrame(ManualFrame frame)
{
    manual_.setFrame(frame);
}

//------------------------------------------------------------
// Function:    setManualMode
// Summary: To set manual motion mode.
// In:      motion  -> manual motion mode, STEP/CONTINUOUS/POINT
// Out:     None
// Return:  None
//------------------------------------------------------------
void ArmGroup::setManualMode(ManualMode mode)
{
    manual_.setMode(mode);
}

//------------------------------------------------------------
// Function:    setManualJointStep
// Summary: To set joint step length in manual joint step working mode.
// In:      step    -> step length, unit: rad
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setManualJointStep(double step)
{
    g_manual_step_joint = step;
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    setManualCartesianPositionStep
// Summary: To set position step length in manual cartesian step working mode.
// In:      step    -> step length, unit: mm
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setManualCartesianPositionStep(double step)
{
    g_manual_step_position = step;
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    setManualCartesianOrientationStep
// Summary: To set orientation step length in manual cartesian step working mode.
// In:      step    -> step length, unit: mm
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setManualCartesianOrientationStep(double step)
{
    g_manual_step_orientation = step;
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getManualMaxSpeedRatio
// Summary: To get manual max speed ratio to max-auto-speed.
// In:      None
// Out:     None
// Return:  manual max speed ratio, range: 0.0 ~ 1.0
//------------------------------------------------------------
double ArmGroup::getManualMaxSpeedRatio(void)
{
    return g_manual_top_speed_ratio;
}

//------------------------------------------------------------
// Function:    getManualSpeedRatio
// Summary: To get manual speed ratio to max-manual-speed.
// In:      None
// Out:     None
// Return:  manual speed ratio, range: 0.0 ~ 1.0
//------------------------------------------------------------
double ArmGroup::getManualSpeedRatio(void)
{
    return g_manual_sub_speed_ratio;
}

//------------------------------------------------------------
// Function:    setManualSpeedRatio
// Summary: To set manual speed ratio to max-manual-speed.
// In:      ratio   -> manual speed to max-manual-speed
//                      range: 0.0 ~ 1.0
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setManualSpeedRatio(double ratio)
{
    if (ratio < 0 || ratio > 1)
    {
        return INVALID_PARAMETER;
    }
    else
    {
        g_manual_sub_speed_ratio = ratio;
        return SUCCESS;
    }
}

//------------------------------------------------------------
// Function:    getManualMaxAccRatio
// Summary: To get manual max acc ratio to max-auto-acc.
// In:      None
// Out:     None
// Return:  manual max acc ratio, range: 0.0 ~ 1.0
//------------------------------------------------------------
double ArmGroup::getManualMaxAccRatio(void)
{
    return g_manual_top_acc_ratio;
}

//------------------------------------------------------------
// Function:    getManualAccRatio
// Summary: To get manual acc ratio to max-manual-acc.
// In:      None
// Out:     None
// Return:  manual acc ratio, range: 0.0 ~ 1.0
//------------------------------------------------------------
double ArmGroup::getManualAccRatio(void)
{
    return g_manual_sub_acc_ratio;
}

//------------------------------------------------------------
// Function:    setManualAccRatio
// Summary: To set manual acc ratio to max-manual-acc.
// In:      ratio   -> manual acc to max-manual-acc
//                      range: 0.0 ~ 1.0
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setManualAccRatio(double ratio)
{
    if (ratio < 0 || ratio > 1)
    {
        return INVALID_PARAMETER;
    }
    else
    {
        g_manual_sub_acc_ratio = ratio;
        return SUCCESS;
    }
}




















MotionCommand* ArmGroup::getFreeMotionCommand(void)
{
    MotionCommand *ptr = NULL;

    if (free_command_list_ptr_ != NULL)
    {
        ptr = free_command_list_ptr_;
        
        free_command_list_ptr_ = free_command_list_ptr_->getNextCommandPtr();
        if (free_command_list_ptr_ != NULL)
        {
            free_command_list_ptr_->setPrevCommandPtr(NULL);
        }
        
        ptr->setPrevCommandPtr(NULL);
        ptr->setNextCommandPtr(NULL);

        return ptr;
    }
    else {
        FST_ERROR("getFreeMotionCommand: no free item available now !");

        char buf[MOTION_COMMAND_POOL_CAPACITY * 16];
        char *str = buf;
        int  len;
        ptr = used_command_list_ptr_;
        
        while (ptr != NULL)
        {
            len = sprintf(str, "%p->", ptr);
            ptr = ptr->getNextCommandPtr();
            str += len;
        }

        sprintf(str, "NULL");
        
        FST_INFO("free_command_list_ptr_->NULL");
        FST_INFO("used_command_list_ptr_->%s", buf);

        return NULL;
    }

}

MotionCommand* ArmGroup::releaseMotionCommand(MotionCommand *cmd)
{
    if (cmd != NULL)
    {
        if (free_command_list_ptr_ != NULL)
        {
            free_command_list_ptr_->setPrevCommandPtr(cmd);
        }

        cmd->setNextCommandPtr(free_command_list_ptr_);
        cmd->setPrevCommandPtr(NULL);
        free_command_list_ptr_ = cmd;
    }
    else
    {
        FST_ERROR("releaseMotionCommand: given ptr is NULL !");
    }

    return free_command_list_ptr_;
}

//------------------------------------------------------------
// Function:    autoJoint
// Summary: Plan a path in joint space to reach the target
//          without smooth.
// In:      target -> motion target
//          id     -> motion id
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::autoJoint(const MotionTarget &target, int id)
{
    MotionTarget tar = target;

    FST_INFO("autoJoint: checking joint parameters ...");

    if (tar.vel < 0)
    {
        tar.vel = g_joint_vel_default;
    }
    else if (tar.vel > 1)
    {
        FST_ERROR("  invalid joint velocity (=%.4f), velocity should in [0.0, 1.0]", tar.vel);
        return INVALID_PARAMETER;
    }

    if (tar.acc < 0)
    {
        tar.acc = g_joint_acc_default;
    }
    else if (tar.acc > 1)
    {
        FST_ERROR("  invalid joint acceleration (=%.4f), acceleration should in [0.0, 1.0]", tar.acc);
        return INVALID_PARAMETER;
    }

    if (tar.cnt < 0)
    {
        tar.cnt = 0;
    }
    else if (tar.cnt > 1)
    {
        FST_ERROR("  invalid smooth parameter (=%.4f), smooth parameter should in [0, 1]", tar.cnt);
        return INVALID_PARAMETER;
    }

    FST_INFO("  start = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             g_start_joint.j1, g_start_joint.j2, g_start_joint.j3,
             g_start_joint.j4, g_start_joint.j5, g_start_joint.j6);
    FST_INFO("  target = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             target.joint_target.j1, target.joint_target.j2, target.joint_target.j3,
             target.joint_target.j4, target.joint_target.j5, target.joint_target.j6);
    FST_INFO("  velocity = %.0f%%, acceleration = %.0f%%, SV = %.0f%%", tar.vel * 100, tar.acc * 100, tar.cnt * 100);
    FST_INFO("Parameter check passed, planning path ...");

    MotionCommand *cmd = getFreeMotionCommand();

    if (cmd == NULL)
    {
        FST_ERROR("Cannot get a free motion object, planning abort.");
        return MOTION_INTERNAL_FAULT;
    }

    if (used_command_list_ptr_ != NULL)
    {
        MotionCommand *ptr = used_command_list_ptr_;
        
        while (ptr->getNextCommandPtr() != NULL)
        {
            ptr = ptr->getNextCommandPtr();
        }

        ptr->setNextCommandPtr(cmd);
        cmd->setPrevCommandPtr(ptr);
        cmd->setNextCommandPtr(NULL);
    }
    else
    {
        used_command_list_ptr_ = cmd;
        cmd->setPrevCommandPtr(NULL);
        cmd->setNextCommandPtr(NULL);
    }
    
    ErrorCode err = cmd->setTarget(tar, id);

    if (err == SUCCESS)
    {
        err = cmd->setStartJoint(g_start_joint);
    }
    if (err == SUCCESS)
    {
        //err = cmd->planPathAndTrajectory(t_path_, t_head_, t_tail_);
        err = cmd->planPath();
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to plan path error=%x, planning abort.", err);

        MotionCommand *ptr = cmd->getPrevCommandPtr();
        releaseMotionCommand(cmd);
        
        if (ptr != NULL)
        {
            ptr->setNextCommandPtr(NULL);
        }
        else
        {
            used_command_list_ptr_ = NULL;
        }
       
        return err;
    }

    //FST_INFO("Path plan and trajectory plan finished successfully");
    FST_INFO("Path plan finished successfully, picking point ...");
    FST_INFO("Path consisted of %d common points and %d trasition points.",
             cmd->getCommonLength(), cmd->getTransitionLength());

    vector<PathPoint> points;

    err = cmd->pickAllPoint(points);

    if (points.size() > PATH_FIFO_CAPACITY)
    {
        FST_ERROR("Path point overload, point num=%d, FIFO capacity=%d.", points.size(), PATH_FIFO_CAPACITY);
        err = MOTION_INTERNAL_FAULT;
    }

    t_head_ = 0;
    t_tail_ = 0;
    for (vector<PathPoint>::iterator it = points.begin(); it != points.end(); ++it)
    {
        FST_INFO("%.6f %.6f %.6f %.6f %.6f %.6f", it->joint.j1, it->joint.j2, it->joint.j3,
                                                  it->joint.j4, it->joint.j5, it->joint.j6);
        t_path_[t_tail_].path_point = *it;
        t_tail_++;
    }
    FST_INFO("  %d points picked out", points.size());

    FST_INFO("Create trajectory ...");
    if (t_head_ != t_tail_)
    {
        // -----------------------------------------------
        for(size_t i = 0; i < t_tail_; ++i)
        {
            t_path_[i].time_from_start = 0;
            memset(t_path_[i].point.joint, 0, NUM_OF_JOINT * sizeof(double));
            memset(t_path_[i].point.omega, 0, NUM_OF_JOINT * sizeof(double));
            memset(t_path_[i].point.alpha, 0, NUM_OF_JOINT * sizeof(double));
        }


        err = planJointTraj();

        if (err == SUCCESS)
        {
            for (size_t i = 0; i < t_tail_; ++i)
            {
                FST_INFO("%d - time_from_start:%.4f duration:%.4f",
                         i + 1, t_path_[i].time_from_start, t_path_[i].duration);
                FST_INFO("    joint: %.6f %.6f %.6f %.6f %.6f %.6f",
                         t_path_[i].point.joint[0], t_path_[i].point.joint[1], t_path_[i].point.joint[2],
                         t_path_[i].point.joint[3], t_path_[i].point.joint[4], t_path_[i].point.joint[5]);
                FST_INFO("    omega: %.6f %.6f %.6f %.6f %.6f %.6f",
                         t_path_[i].point.omega[0], t_path_[i].point.omega[1], t_path_[i].point.omega[2],
                         t_path_[i].point.omega[3], t_path_[i].point.omega[4], t_path_[i].point.omega[5]);
                FST_INFO("    alpha: %.6f %.6f %.6f %.6f %.6f %.6f",
                         t_path_[i].point.alpha[0], t_path_[i].point.alpha[1], t_path_[i].point.alpha[2],
                         t_path_[i].point.alpha[3], t_path_[i].point.alpha[4], t_path_[i].point.alpha[5]);
            }

            auto_running_ = true;
            memcpy(&g_start_joint, t_path_[t_tail_ - 1].point.joint, sizeof(Joint));
            FST_INFO("  create traj success");
        }
        else
        {
            FST_ERROR("  fail to create trajectory");
        }
    }

    return err;
}

//------------------------------------------------------------
// Function:    autoLine
// Summary: Plan a line path to reach the target given by auto move command 
//          without smooth.
// In:      target -> motion target
//          id     -> motion id
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::autoLine(const MotionTarget &target, int id)
{
    MotionTarget tar = target;

    FST_INFO("autoLine: checking line parameters ...");

    if (tar.vel < 0)
    {
        tar.vel = g_cart_vel_default;
    }
    else if (tar.vel < g_cart_vel_min || tar.vel > g_cart_vel_max)
    {
        FST_ERROR("  invalid line velocity (=%.4f), velocity should in [%.4f, %.4f]", 
                  tar.vel, g_cart_vel_min, g_cart_vel_max);
        
        return INVALID_PARAMETER;
    }

    if (tar.acc < 0)
    {
        tar.acc = g_cart_acc_default;
    }
    else if (tar.acc < g_cart_acc_min || tar.acc > g_cart_acc_max)
    {
        FST_ERROR("  invalid line acceleration (=%.4f), acceleration should in [%.4f, %.4f]",
                  tar.acc, g_cart_acc_min, g_cart_acc_max);

        return INVALID_PARAMETER;
    }

    if (tar.cnt < 0)
    {
        tar.cnt = 0;
    }
    else if (tar.cnt > 1)
    {
        FST_ERROR("  invalid smooth parameter (=%.4f), smooth parameter should in [0, 1]", tar.cnt);
        return INVALID_PARAMETER;
    }

    FST_INFO("  target pose: %f %f %f - %f %f %f",  target.pose_target.position.x,
                                                    target.pose_target.position.y,
                                                    target.pose_target.position.z,
                                                    target.pose_target.orientation.a,
                                                    target.pose_target.orientation.b,
                                                    target.pose_target.orientation.c);
    FST_INFO("  velocity=%.2f, acceleration=%.2f, cnt=%.2f", tar.vel, tar.acc, tar.cnt);

    FST_INFO("Parameter check passed, planning path ...");

    MotionCommand *cmd = getFreeMotionCommand();

    if (cmd == NULL)
    {
        FST_ERROR("Cannot get a free motion object, planning abort.");
        return MOTION_INTERNAL_FAULT;
    }

    if (used_command_list_ptr_ != NULL)
    {
        MotionCommand *ptr = used_command_list_ptr_;
        
        while (ptr->getNextCommandPtr() != NULL)
        {
            ptr = ptr->getNextCommandPtr();
        }

        ptr->setNextCommandPtr(cmd);
        cmd->setPrevCommandPtr(ptr);
        cmd->setNextCommandPtr(NULL);
    }
    else
    {
        used_command_list_ptr_ = cmd;
        cmd->setPrevCommandPtr(NULL);
        cmd->setNextCommandPtr(NULL);
    }
    
    ErrorCode err = SUCCESS;
    err = cmd->setTarget(tar, id);
    if (err == SUCCESS)
    {
        err = cmd->setStartJoint(g_start_joint);
    }
    if (err == SUCCESS)
    {
        err = cmd->planPath();
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to plan path error=%x, planning abort.", err);

        MotionCommand *ptr = cmd->getPrevCommandPtr();
        releaseMotionCommand(cmd);
        
        if (ptr != NULL)
        {
            ptr->setNextCommandPtr(NULL);
        }
        else
        {
            used_command_list_ptr_ = NULL;
        }
       
        return err;
    }

    FST_INFO("Path plan finished successfully.");
    FST_INFO("  path point num: %d common %d trasition",
             cmd->getCommonLength(), cmd->getTransitionLength());
    FST_INFO("Picking point ...");

    vector<PathPoint> points;

    err = cmd->pickAllPoint(points);

    if (points.size() > PATH_FIFO_CAPACITY)
    {
        FST_ERROR("Path point overload, point num=%d, FIFO capacity=%d.", points.size(), PATH_FIFO_CAPACITY);
        err = MOTION_INTERNAL_FAULT;
    }

    for (vector<PathPoint>::iterator it = points.begin(); it != points.end(); ++it)
    {
        FST_INFO("%d - %.4f, %.4f, %.4f - %.4f, %.4f, %.4f, %.4f",\
                 it->stamp,\
                 it->pose.position.x,\
                 it->pose.position.y,\
                 it->pose.position.z,\
                 it->pose.orientation.w,\
                 it->pose.orientation.x,\
                 it->pose.orientation.y,\
                 it->pose.orientation.z);
    }

    FST_INFO("  %d points picked out", points.size());

    t_head_ = 0;
    t_tail_ = 0;

    FST_INFO("Converting path point to joint space ...");

    for (vector<PathPoint>::iterator it = points.begin(); it != points.end(); ++it)
    {
        t_path_[t_tail_].path_point = *it;
        err = convertPath2Trajectory(t_path_[t_tail_]);

        FST_INFO("%d - %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",\
                 t_path_[t_tail_].path_point.stamp,\
                 t_path_[t_tail_].point.joint[0],\
                 t_path_[t_tail_].point.joint[1],\
                 t_path_[t_tail_].point.joint[2],\
                 t_path_[t_tail_].point.joint[3],\
                 t_path_[t_tail_].point.joint[4],\
                 t_path_[t_tail_].point.joint[5]);

        if (err != SUCCESS)
        {
            FST_ERROR("fail to convert path point to joint space: stamp=%d", t_path_[t_tail_].path_point.stamp);
            break;
        }

        t_tail_++;
    }

    FST_INFO("  %d points converted", t_tail_ - t_head_);
    FST_INFO("Create trajectory ...");

    if (t_head_ != t_tail_)
    {
        err = planTraj();
        if (err == SUCCESS)
        {
            memcpy(&g_start_joint, t_path_[t_tail_ - 1].point.joint, sizeof(Joint));
            FST_INFO("  create traj success");
        }
        else
        {
            FST_ERROR("  fail to create trajectory");
        }
    }

    return err;

/*
    size_t next_tail;

    for (vector<PathPoint>::iterator it = points.begin(); it != points.end(); ++it)
    {
        next_tail = (path_tail_ + 1) & (PATH_FIFO_CAPACITY - 1);
        if (next_tail != path_head_)
        {
            path_fifo_[path_tail_] = *it;
            path_tail_ = next_tail;
        }
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Get an error while picking point, %d points picked befor this error, err=%x.",
                  points.size(), err);
        FST_INFO("Release the motion object.");

        MotionCommand *ptr = cmd->getPrevCommandPtr();
        releaseMotionCommand(cmd);
        
        if (ptr != NULL)
        {
            ptr->setNextCommandPtr(NULL);
        }
        else
        {
            used_command_list_ptr_ = NULL;
        }

        FST_INFO("Converting points from path to trajectory ...");
    }
    else
    {
        FST_INFO("All common points picked out successfully, converting points from path to trajectory ...");
    }
    // what if the first point in path FIFO is not a pose / joint?
    ErrorCode   ik_err = SUCCESS;
    size_t      ik_cnt = 0;

    while (path_head_ != path_tail_)
    {
        next_tail = (traj_tail_ + 1) & (TRAJECTORY_FIFO_CAPACITY - 1);
        if (next_tail != traj_head_)
        {
            ik_err = chainIK(path_fifo_[path_head_].pose,\
                             g_ik_reference,\
                             traj_fifo_[traj_tail_].point.joint);
            
            if (ik_err == SUCCESS)
            {
                traj_fifo_[traj_tail_].path_point = path_fifo_[path_head_];
                path_head_ = (path_head_ + 1) & (PATH_FIFO_CAPACITY - 1);
                traj_tail_ = next_tail;
                ik_cnt++;
            }
            else
            {
                FST_ERROR("IK failed: source=%p, stamp=%d",
                          path_fifo_[path_head_].source, path_fifo_[path_head_].stamp);
                FST_INFO("Pose: (%.4f, %.4f, %.4f) - (%.4f, %.4f, %.4f, %.4f)",
                         path_fifo_[path_head_].pose.position.x,
                         path_fifo_[path_head_].pose.position.y,
                         path_fifo_[path_head_].pose.position.z,
                         path_fifo_[path_head_].pose.orientation.w,
                         path_fifo_[path_head_].pose.orientation.x,
                         path_fifo_[path_head_].pose.orientation.y,
                         path_fifo_[path_head_].pose.orientation.z);
                FST_INFO("Reference: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                         g_ik_reference.j1, g_ik_reference.j2, g_ik_reference.j3,
                         g_ik_reference.j4, g_ik_reference.j5, g_ik_reference.j6);
                FST_INFO("Joint:     %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                        traj_fifo_[traj_tail_].point.joint[0],
                        traj_fifo_[traj_tail_].point.joint[1],
                        traj_fifo_[traj_tail_].point.joint[2],
                        traj_fifo_[traj_tail_].point.joint[3],
                        traj_fifo_[traj_tail_].point.joint[4],
                        traj_fifo_[traj_tail_].point.joint[5]);
                break;
            }
        }
        else
        {
            break;
        }
    }
    

    FST_INFO("traj points:");
    for (size_t j = traj_head_; j != traj_tail_; j = (j + 1) & (TRAJECTORY_FIFO_CAPACITY - 1))
        FST_INFO("%d - %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                 traj_fifo_[j].path_point.stamp,
                 traj_fifo_[j].point.joint[0],
                 traj_fifo_[j].point.joint[1],
                 traj_fifo_[j].point.joint[2],
                 traj_fifo_[j].point.joint[3],
                 traj_fifo_[j].point.joint[4],
                 traj_fifo_[j].point.joint[5]);
    if (err == SUCCESS)
    {
        if (ik_err == SUCCESS)
        {
            if (path_head_ == path_tail_)
            {
                FST_INFO("All points inversed to joint space successfully.");
                return SUCCESS;
            }
            else
            {
                FST_INFO("%d points inversed to joint space, trajectory FIFO is full.", ik_cnt);

                return SUCCESS;
            }
        }
        else
        {
            FST_ERROR("Computing IK failed, last stamp=%d", traj_fifo_[traj_tail_].path_point.stamp);
            return ik_err;
        }
    }
    else
    {
        return err;
    }
*/
}

/*
//------------------------------------------------------------
// Function:    fillTrajectoryFIFO
// Summary: Get points from path FIFO, compute IK, fill into trajectory FIFO .
// In:      num    -> num of points to handle
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::fillTrajectoryFIFO(size_t num = TRAJECTORY_FIFO_CAPACITY)
{
    ErrorCode err = SUCCESS;
    size_t i;

    for (i = 0; i < num; i++)
    {
        size_t next_traj_tail = (traj_tail_ + 1) & (TRAJECTORY_FIFO_CAPACITY - 1);
        if (next_traj_tail != traj_head_)
        {
            if (path_head_ != path_tail_)
            {
                err = convertPath2Trajectory(path_fifo_[path_head_], traj_fifo_[traj_tail_]);

                if (err == SUCCESS)
                {
                    path_head_ = (path_head_ + 1) & (PATH_FIFO_CAPACITY - 1);
                    traj_tail_ = next_traj_tail;
                }
                else
                {
                    break;
                }
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    return err;
}
*/

ErrorCode ArmGroup::convertPath2Trajectory(ControlPoint &cp)
{
    ErrorCode err = SUCCESS;

    PathPoint  &pp = cp.path_point;
    JointState &jp = cp.point;

    if (pp.type == MOTION_JOINT)
    {
        cp.path_point = pp;
        cp.duration = -1;
        cp.expect_duration = -1;
        cp.time_from_start = -1;

        memcpy(jp.joint, &pp.joint, NUM_OF_JOINT * sizeof(double));

        g_ik_reference = pp.joint;
    }
    else if (pp.type == MOTION_LINE || pp.type == MOTION_CIRCLE)
    {
        err = chainIK(pp.pose, g_ik_reference, jp.joint);

        if (err == SUCCESS)
        {
            cp.path_point = pp;
            cp.duration = -1;
            cp.expect_duration = -1;
            cp.time_from_start = -1;
        }
        else
        {
            FST_ERROR("IK failed: source=%p, stamp=%d", pp.source, pp.stamp);
            FST_INFO("Pose: (%.4f, %.4f, %.4f) - (%.4f, %.4f, %.4f, %.4f)",
                     pp.pose.position.x, pp.pose.position.y, pp.pose.position.z,
                     pp.pose.orientation.w, pp.pose.orientation.x, pp.pose.orientation.y, pp.pose.orientation.z);
            FST_INFO("Reference: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                     g_ik_reference.j1, g_ik_reference.j2, g_ik_reference.j3,
                     g_ik_reference.j4, g_ik_reference.j5, g_ik_reference.j6);
            FST_INFO("Joint:     %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                    jp.joint[0], jp.joint[1], jp.joint[2], jp.joint[3], jp.joint[4], jp.joint[5]);
        }
    }
    else
    {
        FST_ERROR("convertPath2Trajectory: invalid point type(=%d),", pp.type);
        err = MOTION_INTERNAL_FAULT;
    }

    return err;
}

ErrorCode ArmGroup::planTraj()
{
    std::ofstream os("analysis.csv");
    ErrorCode err;

    size_t  head = t_head_;
    size_t  tail = t_tail_;

    if (head + 2 < tail)
    {
        double velocity[tail - head];
        int    fore_flg = 1;
        int    back_flg = 1;
        double expc_vel = t_path_[head].path_point.source->getCommandVelocity();
        double fore_vel = 0;
        double back_vel = 0;
        memset(t_path_[tail - 1].point.omega, 0, NUM_OF_JOINT * sizeof(double));

        while (tail > head + 2)
        {
            if (fore_vel > back_vel)
            {
                err = backCycle(t_path_[tail - 1], t_path_[tail - 2], back_flg);
                if (err == SUCCESS)
                {
                    double dis = getDistance(t_path_[tail - 2].path_point.pose, t_path_[tail - 1].path_point.pose);
                    double tim = t_path_[tail - 1].duration;
                    back_vel = dis / tim;
                    velocity[tail - 1] = back_vel;
                    double  dif_vel = expc_vel - velocity[tail - 1];

                    if (back_flg == 1 && dif_vel < expc_vel * 0.2 && dif_vel < 100)
                    {
                        //FST_INFO("<<<<<<1 -> 2<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                        back_flg = 2;
                    }
                    else if (back_flg == 2 && velocity[tail - 1]  < velocity[tail])
                    {
                        //FST_INFO("<<<<<<<2 -> 0<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                        back_flg = 0;
                    }
                    else
                    {

                    }

                    //FST_INFO("dis=%f, tim=%f, back vel=%f", dis, tim, back_vel);
                    tail--;
                }
                else
                {
                    FST_ERROR("back cycle err=%x", err);
                    break;
                }
            }
            else
            {
                err = foreCycle(prev_traj_point_, t_path_[head], fore_flg);
                if (err == SUCCESS)
                {
                    double dis = getDistance(prev_traj_point_.path_point.pose, t_path_[head].path_point.pose);
                    double tim = t_path_[head].duration;

                    fore_vel = dis / tim;
                    velocity[head] = fore_vel;
                    double dif_vel = expc_vel - velocity[head];
                    
                    if (fore_flg == 1 && dif_vel < expc_vel * 0.2 && dif_vel < 100)
                    {
                        //FST_INFO("<<<<<<1 -> 2<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                        fore_flg = 2;
                    }
                    else if (fore_flg == 2 && velocity[head] < velocity[head - 1])
                    {
                        //FST_INFO("<<<<<<<2 -> 0<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                        fore_flg = 0;
                    }
                    else
                    {
                        
                    }
                    //FST_INFO("dis=%f, tim=%f, fore vel=%f", dis, tim, velocity[head]);
                    prev_traj_point_ = t_path_[head];
                    head++;
                }
                else
                {
                    FST_ERROR("fore cycle err=%x", err);
                    break;
                }
            }
            
        }
        //FST_INFO("head=%d, tail=%d", head, tail);
        //FST_INFO("fore vel=%.6f, back vel=%.6f", velocity[head - 1], velocity[tail + 1]);

        double dduration = (t_path_[tail + 1].duration - t_path_[head - 1].duration) / 3;
        t_path_[head].duration = t_path_[head - 1].duration + dduration;
        t_path_[head].time_from_start = t_path_[head - 1].time_from_start + t_path_[head - 1].duration;
        t_path_[head + 1].duration = t_path_[head].duration + dduration;
        t_path_[head + 1].time_from_start = t_path_[head].time_from_start + t_path_[head].duration;


        JointState *ps = &t_path_[head - 1].point;
        JointState *pe = &t_path_[head].point;

        for (size_t j = 0; j < 6; j++)
        {
            pe->alpha[j] = (pe->joint[j] - ps->joint[j] - ps->omega[j] * t_path_[head].duration) * 2 / t_path_[head].duration / t_path_[head].duration;
            pe->omega[j] = ps->omega[j] + pe->alpha[j] * t_path_[head - 1].duration;
        }

        ps = &t_path_[head].point;
        pe = &t_path_[head + 1].point;

        for (size_t j = 0; j < 6; j++)
        {
            pe->alpha[j] = (pe->joint[j] - ps->joint[j] - ps->omega[j] * t_path_[head + 1].duration) * 2 / t_path_[head + 1].duration / t_path_[head + 1].duration;
            pe->omega[j] = ps->omega[j] + pe->alpha[j] * t_path_[head + 1].duration;
        }

        for (size_t j = head + 2; j < t_tail_; j++)
        {
            t_path_[j].time_from_start = t_path_[j - 1].time_from_start + t_path_[j - 1].duration;
        }

        ControlPoint *pp;
        JointState *p;

        for (size_t j = t_head_; j < t_tail_; j++)
        {
            pp = &t_path_[j];
            p = &pp->point;
            FST_WARN("stamp=%d, time_from_start=%.6f", pp->path_point.stamp, pp->time_from_start);
            //FST_INFO("duration=%.6f, exp_duration=%.6f", pp->duration, pp->expect_duration);
            //FST_INFO("j1-j6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p->joint[0], p->joint[1], p->joint[2], p->joint[3], p->joint[4], p->joint[5]);
            //FST_INFO("w1-w6: %.f, %.6f, %.6f, %.6f, %.6f, %.6f", p->omega[0], p->omega[1], p->omega[2], p->omega[3], p->omega[4], p->omega[5]);
            //FST_INFO("a1-a6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p->alpha[0], p->alpha[1], p->alpha[2], p->alpha[3], p->alpha[4], p->alpha[5]);
            
            os << pp->path_point.stamp << "," << pp->time_from_start << ","
               << pp->path_point.pose.position.x << "," << pp->path_point.pose.position.y << ","
               << pp->path_point.pose.position.z << "," << pp->path_point.pose.orientation.w << ","
               << pp->path_point.pose.orientation.x << "," << pp->path_point.pose.orientation.y << ","
               << pp->path_point.pose.orientation.z << "," << p->joint[0] << ","
               << p->joint[1] << "," << p->joint[2] << "," << p->joint[3] << "," << p->joint[4] << ","
               << p->joint[5] << "," << p->omega[0] << "," << p->omega[1] << "," << p->omega[2] << ","
               << p->omega[3] << "," << p->omega[4] << "," << p->omega[5] << "," << p->alpha[0] << ","
               << p->alpha[1] << "," << p->alpha[2] << "," << p->alpha[3] << "," << p->alpha[4] << ","
               << p->alpha[5] << std::endl;
        }
    
        pick_time_    = g_cycle_time;
        pick_segment_ = 0;
/*
        for (size_t i = head - 2; i < tail + 2; i++)
        {
            JointState &p = t_path_[i].point;
            FST_INFO("stamp=%d, duration=%f, time_from_start=%.6f", t_path_[i].path_point.stamp, t_path_[i].duration, t_path_[i].time_from_start);
            FST_INFO("j1-j6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.joint[0], p.joint[1], p.joint[2], p.joint[3], p.joint[4], p.joint[5]);
            FST_INFO("w1-w6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.omega[0], p.omega[1], p.omega[2], p.omega[3], p.omega[4], p.omega[5]);
            FST_INFO("a1-a6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p.alpha[0], p.alpha[1], p.alpha[2], p.alpha[3], p.alpha[4], p.alpha[5]);
        }
        */

    }

    return err;
}


ErrorCode ArmGroup::planJointTraj(void)
{
    ErrorCode err;

    // this should not be set here, fix it later
    pick_time_    = g_cycle_time;
    pick_segment_ = 0;
    // setting joint limits
    Omega velocity_max[AXIS_IN_ALGORITHM];
    Alpha acc_max[AXIS_IN_ALGORITHM]; //acc limits
    Alpha acc_backward[AXIS_IN_ALGORITHM];   //acc limits used by backward loop
    double vel =  t_path_[t_head_].path_point.source->getCommandVelocity();
    double acc =  t_path_[t_head_].path_point.source->getCommandAcc();

    velocity_max[0] = vel*g_soft_constraint.j1.max_omega;
    velocity_max[1] = vel*g_soft_constraint.j2.max_omega;
    velocity_max[2] = vel*g_soft_constraint.j3.max_omega;
    velocity_max[3] = vel*g_soft_constraint.j4.max_omega;
    velocity_max[4] = vel*g_soft_constraint.j5.max_omega;
    velocity_max[5] = vel*g_soft_constraint.j6.max_omega;
    acc_max[0] = acc*g_soft_constraint.j1.max_alpha;
    acc_max[1] = acc*g_soft_constraint.j2.max_alpha;
    acc_max[2] = acc*g_soft_constraint.j3.max_alpha;
    acc_max[3] = acc*g_soft_constraint.j4.max_alpha;
    acc_max[4] = acc*g_soft_constraint.j5.max_alpha;
    acc_max[5] = acc*g_soft_constraint.j6.max_alpha;  

    // trajectory generation
    double forward_duration = DBL_MAX, backward_duration = DBL_MAX;
    size_t forward_tick = t_head_, backward_tick = t_tail_ - 1;
    while(forward_tick <= backward_tick)
    {
        MotionTime duration[AXIS_IN_ALGORITHM] = {0};
        MotionTime duration_max = 0;
        Angle* start_joint_ptr = NULL;
        Angle* end_joint_ptr = NULL;
        Omega* start_omega_ptr = NULL;
        MotionTime prev_time_from_start = 0;
        size_t target_tick = 0;
        if(forward_duration >= backward_duration) // do forward integration preparation
        {
            // set start and end points and start omega
            target_tick = forward_tick;
            if(forward_tick == t_head_)
            {
                start_joint_ptr = (Angle*)&prev_traj_point_.point.joint;
                start_omega_ptr = (Omega*)&prev_traj_point_.point.omega;
                prev_time_from_start = prev_traj_point_.time_from_start;
            }
            else
            {
                start_joint_ptr = (Angle*)&t_path_[target_tick - 1].path_point.joint;
                start_omega_ptr = (Omega*)&t_path_[target_tick - 1].point.omega;
                prev_time_from_start = t_path_[target_tick - 1].time_from_start;
            }
            end_joint_ptr = (Angle*)&t_path_[target_tick].path_point.joint;
        }
        else    // do backward integration preparation
        {      
            // set start and end points and start omega
            target_tick = backward_tick - 1;
            start_joint_ptr = (Angle*)&t_path_[target_tick].path_point.joint;
            start_omega_ptr = (Omega*)&t_path_[target_tick + 1].point.omega;
            prev_time_from_start = 0;
            end_joint_ptr = (Angle*)&t_path_[target_tick + 1].path_point.joint;
        }

        // compute axes maximum durations for a piece of path
        computeDurationMax(start_joint_ptr, end_joint_ptr, start_omega_ptr, acc_max, duration_max);   
        if(forward_duration >= backward_duration)
        {       
            computeTrajectory(true, target_tick, start_joint_ptr, end_joint_ptr, start_omega_ptr, duration_max, t_path_);   
            forward_duration = duration_max;
            t_path_[target_tick].time_from_start = prev_time_from_start + duration_max;
            ++forward_tick;
        }
        else
        {          
            computeTrajectory(false, target_tick, start_joint_ptr, end_joint_ptr, start_omega_ptr, duration_max, t_path_);   
            backward_duration = duration_max;
            t_path_[backward_tick].time_from_start = duration_max;
            --backward_tick;
        }
    }

    // set the last trajectory point
    Angle* last_joint_ptr = (Angle*)&t_path_[t_tail_ - 1].path_point.joint;
    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)            
    {
        t_path_[t_tail_ - 1].point.joint[i] = last_joint_ptr[i];
        t_path_[t_tail_ - 1].point.omega[i] = 0;
    }

    // complete time_from_start in backward direction
    for(size_t i = forward_tick; i < t_tail_; ++i)
    {
         t_path_[i].time_from_start += t_path_[i - 1].time_from_start;
    }

    return SUCCESS;
}

/*
//std::ofstream tos("/home/fst/myworkspace/jout.txt");
ErrorCode ArmGroup::planJointTraj(void)
{
    ErrorCode err;

    // this should not be set here, fix it later
    pick_time_    = g_cycle_time;
    pick_segment_ = 0;
    // setting joint limits
    Omega velocity_max[AXIS_IN_ALGORITHM];
    Alpha acc_max[AXIS_IN_ALGORITHM]; //acc limits
    Alpha acc_backward[AXIS_IN_ALGORITHM];   //acc limits used by backward loop
    double vel =  t_path_[t_head_].path_point.source->getCommandVelocity();
    double acc =  t_path_[t_head_].path_point.source->getCommandAcc();
    velocity_max[0] = vel*g_soft_constraint.j1.max_omega;
    velocity_max[1] = vel*g_soft_constraint.j2.max_omega;
    velocity_max[2] = vel*g_soft_constraint.j3.max_omega;
    velocity_max[3] = vel*g_soft_constraint.j4.max_omega;
    velocity_max[4] = vel*g_soft_constraint.j5.max_omega;
    velocity_max[5] = vel*g_soft_constraint.j6.max_omega;
    acc_max[0] = acc*g_soft_constraint.j1.max_alpha;
    acc_max[1] = acc*g_soft_constraint.j2.max_alpha;
    acc_max[2] = acc*g_soft_constraint.j3.max_alpha;
    acc_max[3] = acc*g_soft_constraint.j4.max_alpha;
    acc_max[4] = acc*g_soft_constraint.j5.max_alpha;
    acc_max[5] = acc*g_soft_constraint.j6.max_alpha;  

    // trajectory generation
    double forward_duration = DBL_MAX, backward_duration = DBL_MAX;
    size_t forward_tick = t_head_, backward_tick = t_tail_ - 1;
    while(forward_tick < backward_tick)
    {
        MotionTime duration[AXIS_IN_ALGORITHM] = {0};
        MotionTime duration_max = 0;
        Angle* start_joint_ptr = NULL;
        Angle* end_joint_ptr = NULL;
        Omega* start_omega_ptr = NULL;
        MotionTime prev_time_from_start = 0;
        size_t target_tick = 0;
        if(forward_duration >= backward_duration) // do forward integration preparation
        {
            // set start and end points and start omega
            target_tick = forward_tick;
            if(forward_tick == t_head_)
            {
                start_joint_ptr = (Angle*)&prev_traj_point_.point.joint;
                start_omega_ptr = (Omega*)&prev_traj_point_.point.omega;
                prev_time_from_start = prev_traj_point_.time_from_start;
            }
            else
            {
                start_joint_ptr = (Angle*)&t_path_[target_tick - 1].path_point.joint;
                start_omega_ptr = (Omega*)&t_path_[target_tick - 1].point.omega;
                prev_time_from_start = t_path_[target_tick - 1].time_from_start;
            }
            end_joint_ptr = (Angle*)&t_path_[target_tick].path_point.joint;
        }
        else    // do backward integration preparation
        {      
            // set start and end points and start omega
            target_tick = backward_tick - 1;
            start_joint_ptr = (Angle*)&t_path_[target_tick].path_point.joint;
            start_omega_ptr = (Omega*)&t_path_[target_tick + 1].point.omega;
            prev_time_from_start = 0;
            end_joint_ptr = (Angle*)&t_path_[target_tick + 1].path_point.joint;
        }

        // compute axes maximum durations for a piece of path
        computeDurationMax(start_joint_ptr, end_joint_ptr, start_omega_ptr, acc_max, duration_max);

        if(forward_duration >= backward_duration)
        {
            computeTrajectory(true, target_tick, start_joint_ptr, end_joint_ptr, start_omega_ptr, duration_max, t_path_);   
            forward_duration = duration_max;
            t_path_[target_tick].time_from_start = prev_time_from_start + duration_max;
            ++forward_tick;
        }
        else
        {
            computeTrajectory(false, target_tick, start_joint_ptr, end_joint_ptr, start_omega_ptr, duration_max, t_path_);   
            backward_duration = duration_max;
            t_path_[backward_tick].time_from_start = duration_max;
            --backward_tick;
        }
    }
    
    // set the last trajectory point
    Angle* last_joint_ptr = (Angle*)&t_path_[t_tail_ - 1].path_point.joint;
    for(int i = 0; i < AXIS_IN_ALGORITHM; ++i)            
    {
        t_path_[t_tail_ - 1].point.joint[i] = last_joint_ptr[i];
        t_path_[t_tail_ - 1].point.omega[i] = 0;
    }

    // complete time_from_start in backward direction
    for(size_t i = forward_tick; i < t_tail_; ++i)
    {
         t_path_[i].time_from_start += t_path_[i - 1].time_from_start;
    }


    return SUCCESS;
}
*/

/*
ErrorCode ArmGroup::speedup(void)
{
    ErrorCode err;
    std::ofstream os("test.csv");

    for (int i = 0; i < traj_tail_; i++)
    //for (int i = 0; i < 70; i++)
    {
        //err = createTrajectoryFromPath(prev_traj_point_, traj_fifo_[i]);
        err = createTrajFromPath(prev_traj_point_, traj_fifo_[i]);
        if (err == SUCCESS)
        {
            prev_traj_point_ = traj_fifo_[i];
        
            os  << i\
                << traj_fifo_[i].path_point.pose.position.x << ","\
                << traj_fifo_[i].path_point.pose.position.y << ","\
                << traj_fifo_[i].path_point.pose.position.z << ","\
                << traj_fifo_[i].path_point.pose.orientation.w << ","\
                << traj_fifo_[i].path_point.pose.orientation.x << ","\
                << traj_fifo_[i].path_point.pose.orientation.y << ","\
                << traj_fifo_[i].path_point.pose.orientation.z << ","\
                << traj_fifo_[i].point.joint[0] << ","\
                << traj_fifo_[i].point.joint[1] << ","\
                << traj_fifo_[i].point.joint[2] << ","\
                << traj_fifo_[i].point.joint[3] << ","\
                << traj_fifo_[i].point.joint[4] << ","\
                << traj_fifo_[i].point.joint[5] << ","\
                << traj_fifo_[i].point.omega[0] << ","\
                << traj_fifo_[i].point.omega[1] << ","\
                << traj_fifo_[i].point.omega[2] << ","\
                << traj_fifo_[i].point.omega[3] << ","\
                << traj_fifo_[i].point.omega[4] << ","\
                << traj_fifo_[i].point.omega[5] << ","\
                << traj_fifo_[i].point.alpha[0] << ","\
                << traj_fifo_[i].point.alpha[1] << ","\
                << traj_fifo_[i].point.alpha[2] << ","\
                << traj_fifo_[i].point.alpha[3] << ","\
                << traj_fifo_[i].point.alpha[4] << ","\
            << traj_fifo_[i].point.alpha[5] << std::endl;
        }
        else
        {
            break;
        }
    }

    return err;
}
*/

ErrorCode ArmGroup::pickFromAuto(size_t num, vector<JointOutput> &points)
{
    MotionTime   tm;
    JointOutput  jout;
    JointState  *js;

    FST_WARN("pickFromAuto: pick points from auto");

    for (size_t i = 0; i < num; i++)
    {
        while (pick_segment_ < t_tail_ && pick_time_ > t_path_[pick_segment_].time_from_start)
        {
            pick_segment_++;
        }
        //FST_INFO("pick time=%f, total time=%f", pick_time_, t_path_[t_tail_ - 1].time_from_start);
        //FST_INFO("pick segment=%d, segment time from start=%f", pick_segment_, t_path_[pick_segment_].time_from_start);
        if (pick_segment_ < t_tail_ && pick_time_ < t_path_[pick_segment_].time_from_start)
        {
            //FST_WARN("pick t=%f seg=%d", pick_time_, pick_segment_);
            //FST_WARN("time_from_start=%f, duration=%f", t_path_[pick_segment_].time_from_start, t_path_[pick_segment_].duration);

            js = &t_path_[pick_segment_].point;
            tm = t_path_[pick_segment_].time_from_start - pick_time_;

            if (pick_time_ < g_cycle_time + MINIMUM_E9)
            {
                jout.level = POINT_START;
            }
            else
            {
                jout.level = POINT_MIDDLE;
            }
            jout.id = t_path_[pick_segment_].path_point.id;
            jout.joint.j1 = js->joint[0] - js->omega[0] * tm + 0.5 * js->alpha[0] * tm * tm;
            jout.joint.j2 = js->joint[1] - js->omega[1] * tm + 0.5 * js->alpha[1] * tm * tm;
            jout.joint.j3 = js->joint[2] - js->omega[2] * tm + 0.5 * js->alpha[2] * tm * tm;
            jout.joint.j4 = js->joint[3] - js->omega[3] * tm + 0.5 * js->alpha[3] * tm * tm;
            jout.joint.j5 = js->joint[4] - js->omega[4] * tm + 0.5 * js->alpha[4] * tm * tm;
            jout.joint.j6 = js->joint[5] - js->omega[5] * tm + 0.5 * js->alpha[5] * tm * tm;

            //FST_INFO("J5: j=%.6f, w=%.6f, a=%.6f, tm=%.4f, time_from_start=%.4f, duration=%.4f, res=%.6f",
            //         js->joint[4], js->omega[4], js->alpha[4],
            //         tm, t_path_[pick_segment_].time_from_start, t_path_[pick_segment_].duration,
            //         jout.joint.j5);
            FST_INFO("%d - %.4f - %f,%f,%f,%f,%f,%f", jout.level, pick_time_,
                     jout.joint.j1, jout.joint.j2, jout.joint.j3,
                     jout.joint.j4, jout.joint.j5, jout.joint.j6);
            points.push_back(jout);
            pick_time_ += g_cycle_time;
        }
        else
        {
            jout.id = t_path_[t_tail_ - 1].path_point.id;
            jout.level = POINT_ENDING;
            jout.joint.j1 = t_path_[t_tail_ - 1].point.joint[0];
            jout.joint.j2 = t_path_[t_tail_ - 1].point.joint[1];
            jout.joint.j3 = t_path_[t_tail_ - 1].point.joint[2];
            jout.joint.j4 = t_path_[t_tail_ - 1].point.joint[3];
            jout.joint.j5 = t_path_[t_tail_ - 1].point.joint[4];
            jout.joint.j6 = t_path_[t_tail_ - 1].point.joint[5];

            FST_INFO("%d - %.4f - %f,%f,%f,%f,%f,%f", jout.level, pick_time_,
                     jout.joint.j1, jout.joint.j2, jout.joint.j3,
                     jout.joint.j4, jout.joint.j5, jout.joint.j6);
            points.push_back(jout);

            FST_INFO("delete obj: %p", t_path_[t_tail_ - 1].path_point.source);
            releaseMotionCommand(t_path_[t_tail_ - 1].path_point.source);
            auto_running_ = false;
            break;
        }
    }

    return SUCCESS;
}

ErrorCode ArmGroup::pickFromManual(size_t num, vector<JointOutput> &points)
{
    FST_WARN("pickFromManual: pick points from manual");

    if (g_manual_frame == JOINT)
    {
        return pickManualJoint(num, points);
    }
    else
    {
        return pickManualCartesian(num, points);
    }

}

ErrorCode ArmGroup::pickManualJoint(size_t num, vector<JointOutput> &points)
{
    JointOutput jout;

    FST_WARN("pickManualJoint: pick point");

    if (g_manual_mode == STEP || g_manual_mode == CONTINUOUS)
    {
        if (g_manual_pick_time < g_manual_joint_coeff[0].duration_3)
        {
            size_t index;
            double *angle;
            double *start;

            for (index = 0; index < num; index++)
            {
                angle = &jout.joint.j1;
                start = &g_manual_joint_start.j1;
                g_manual_pick_time += g_cycle_time;

                jout.id = -1;
                jout.level = POINT_MIDDLE;
                if (g_manual_pick_time < g_cycle_time + MINIMUM_E9)
                {
                    jout.level = POINT_START;
                }
                
                for (size_t jnt = 0; jnt < 6; jnt++)
                {
                    if (g_manual_pick_time < g_manual_joint_coeff[jnt].duration_1)
                    {
                        *angle = *start + g_manual_joint_coeff[jnt].alpha_1 * g_manual_pick_time * g_manual_pick_time / 2;
                    }
                    else if (g_manual_pick_time < g_manual_joint_coeff[jnt].duration_2)
                    {
                        *angle = *start + g_manual_joint_coeff[jnt].alpha_1 * g_manual_joint_coeff[jnt].duration_1 * g_manual_joint_coeff[jnt].duration_1 / 2 +
                                 g_manual_joint_coeff[jnt].alpha_1 * g_manual_joint_coeff[jnt].duration_1 * (g_manual_pick_time - g_manual_joint_coeff[jnt].duration_1);
                    }
                    else if (g_manual_pick_time < g_manual_joint_coeff[jnt].duration_3)
                    {
                        *angle = *start + g_manual_joint_coeff[jnt].alpha_1 * g_manual_joint_coeff[jnt].duration_1 * g_manual_joint_coeff[jnt].duration_1 / 2 +
                                 g_manual_joint_coeff[jnt].alpha_1 * g_manual_joint_coeff[jnt].duration_1 * (g_manual_joint_coeff[jnt].duration_2 - g_manual_joint_coeff[jnt].duration_1) -
                                 g_manual_joint_coeff[jnt].alpha_3 * (g_manual_pick_time - g_manual_joint_coeff[jnt].duration_2) * (g_manual_pick_time - g_manual_joint_coeff[jnt].duration_2) / 2;
                    }
                    else
                    {
                        if (g_manual_direction[jnt] == STANDBY)
                        {
                            *angle = *start;
                        }
                        else if (g_manual_direction[jnt] == INCREASE)
                        {
                            *angle = *start + g_manual_step_joint;
                        }
                        else
                        {
                            *angle = *start - g_manual_step_joint;
                        }
                    }

                    ++ angle;
                    ++ start;
                }

                points.push_back(jout);

                if (g_manual_pick_time >= g_manual_joint_coeff[0].duration_3)
                {
                    points.back().level = POINT_ENDING;
                    FST_INFO("%d - %f %f %f %f %f %f", points.back().level, points.back().joint.j1,
                              points.back().joint.j2,  points.back().joint.j3,  points.back().joint.j4, 
                              points.back().joint.j5,  points.back().joint.j6);

                    manual_running_ = false;
                    break;
                }

                FST_INFO("%d - %f %f %f %f %f %f", points.back().level, points.back().joint.j1,
                          points.back().joint.j2,  points.back().joint.j3,  points.back().joint.j4, 
                          points.back().joint.j5,  points.back().joint.j6);
            }

            return SUCCESS;
        }
        else
        {
            FST_INFO("pick time=%f, duration=%f", g_manual_pick_time, g_manual_joint_coeff[0].duration_3);
            return SUCCESS;
        }
    }
    else
    {
        if (g_manual_pick_time < g_manual_joint_coeff[0].duration_3)
        {
            size_t index;
            double *angle;
            double *start;
            double *target;

            for (index = 0; index < num; index++)
            {
                angle = &jout.joint.j1;
                start = &g_manual_joint_start.j1;
                target = &g_manual_joint_target.j1;
                g_manual_pick_time += g_cycle_time;

                jout.id = -1;
                jout.level = POINT_MIDDLE;
                if (g_manual_pick_time < g_cycle_time + MINIMUM_E9)
                {
                    jout.level = POINT_START;
                }
                
                for (size_t jnt = 0; jnt < 6; jnt++)
                {
                    if (g_manual_pick_time < g_manual_joint_coeff[jnt].duration_1)
                    {
                        *angle = *start + g_manual_joint_coeff[jnt].alpha_1 * g_manual_pick_time * g_manual_pick_time / 2;
                    }
                    else if (g_manual_pick_time < g_manual_joint_coeff[jnt].duration_2)
                    {
                        *angle = *start + g_manual_joint_coeff[jnt].alpha_1 * g_manual_joint_coeff[jnt].duration_1 * g_manual_joint_coeff[jnt].duration_1 / 2 +
                                 g_manual_joint_coeff[jnt].alpha_1 * g_manual_joint_coeff[jnt].duration_1 * (g_manual_pick_time - g_manual_joint_coeff[jnt].duration_1);
                    }
                    else if (g_manual_pick_time < g_manual_joint_coeff[jnt].duration_3)
                    {
                        *angle = *start + g_manual_joint_coeff[jnt].alpha_1 * g_manual_joint_coeff[jnt].duration_1 * g_manual_joint_coeff[jnt].duration_1 / 2 +
                                 g_manual_joint_coeff[jnt].alpha_1 * g_manual_joint_coeff[jnt].duration_1 * (g_manual_joint_coeff[jnt].duration_2 - g_manual_joint_coeff[jnt].duration_1) -
                                 g_manual_joint_coeff[jnt].alpha_3 * (g_manual_pick_time - g_manual_joint_coeff[jnt].duration_2) * (g_manual_pick_time - g_manual_joint_coeff[jnt].duration_2) / 2;
                    }
                    else
                    {
                        *angle = *target;
                    }

                    ++ angle;
                    ++ start;
                    ++ target;
                }

                points.push_back(jout);

                if (g_manual_pick_time >= g_manual_joint_coeff[0].duration_3)
                {
                    points.back().level = POINT_ENDING;
                    FST_INFO("%d - %f %f %f %f %f %f", points.back().level, points.back().joint.j1,
                              points.back().joint.j2,  points.back().joint.j3,  points.back().joint.j4, 
                              points.back().joint.j5,  points.back().joint.j6);

                    manual_running_ = false;
                    break;
                }

                FST_INFO("%d - %f %f %f %f %f %f", points.back().level, points.back().joint.j1,
                          points.back().joint.j2,  points.back().joint.j3,  points.back().joint.j4, 
                          points.back().joint.j5,  points.back().joint.j6);
            }

            return SUCCESS;
        }
        else
        {
            FST_INFO("pick time=%f, duration=%f", g_manual_pick_time, g_manual_joint_coeff[0].duration_3);
            return SUCCESS;
        }
        return SUCCESS;
    }

}


ErrorCode ArmGroup::pickManualCartesian(size_t num, vector<JointOutput> &points)
{
    PoseEuler   pose;
    JointOutput jout;

    FST_WARN("pickManualCartesian: pick point");

    if (g_manual_mode == STEP || g_manual_mode == CONTINUOUS)
    {
        if (g_manual_pick_time < g_manual_cartesian_coeff[0].duration_3)
        {
            size_t index;
            double *value;
            double *start;

            for (index = 0; index < num; index++)
            {
                value = &pose.position.x;
                start = &g_manual_cartesian_start.position.x;
                g_manual_pick_time += g_cycle_time;

                jout.id = -1;
                jout.level = POINT_MIDDLE;
                if (g_manual_pick_time < g_cycle_time + MINIMUM_E9)
                {
                    jout.level = POINT_START;
                }
                
                for (size_t i = 0; i < 6; i++)
                {
                    if (g_manual_pick_time < g_manual_cartesian_coeff[i].duration_1)
                    {
                        *value = *start + g_manual_cartesian_coeff[i].alpha_1 * g_manual_pick_time * g_manual_pick_time / 2;
                    }
                    else if (g_manual_pick_time < g_manual_cartesian_coeff[i].duration_2)
                    {
                        *value = *start + g_manual_cartesian_coeff[i].alpha_1 * g_manual_cartesian_coeff[i].duration_1 * g_manual_cartesian_coeff[i].duration_1 / 2 +
                                 g_manual_cartesian_coeff[i].alpha_1 * g_manual_cartesian_coeff[i].duration_1 * (g_manual_pick_time - g_manual_cartesian_coeff[i].duration_1);
                    }
                    else if (g_manual_pick_time < g_manual_cartesian_coeff[i].duration_3)
                    {
                        *value = *start + g_manual_cartesian_coeff[i].alpha_1 * g_manual_cartesian_coeff[i].duration_1 * g_manual_cartesian_coeff[i].duration_1 / 2 +
                                 g_manual_cartesian_coeff[i].alpha_1 * g_manual_cartesian_coeff[i].duration_1 * (g_manual_cartesian_coeff[i].duration_2 - g_manual_cartesian_coeff[i].duration_1) -
                                 g_manual_cartesian_coeff[i].alpha_3 * (g_manual_pick_time - g_manual_cartesian_coeff[i].duration_2) * (g_manual_pick_time - g_manual_cartesian_coeff[i].duration_2) / 2;
                    }
                    else
                    {
                        if (g_manual_direction[i] == STANDBY)
                        {
                            *value = *start;
                        }
                        else if (g_manual_direction[i] == INCREASE)
                        {
                            *value = *start + (i < 3 ? g_manual_step_position : g_manual_step_orientation);
                        }
                        else
                        {
                            *value = *start - (i < 3 ? g_manual_step_position : g_manual_step_orientation);
                        }
                    }

                    ++ value;
                    ++ start;
                }

                //FST_INFO("# - %f %f %f %f %f %f", pose.position.x, pose.position.y, pose.position.z,
                //         pose.orientation.a, pose.orientation.b, pose.orientation.c);

                if (chainIK(pose, g_ik_reference, jout.joint) == SUCCESS)
                {
                    points.push_back(jout);
                }


                if (g_manual_pick_time >= g_manual_cartesian_coeff[0].duration_3)
                {
                    points.back().level = POINT_ENDING;
                    //FST_INFO("%d - %f %f %f %f %f %f", points.back().level, points.back().joint.j1,
                    //          points.back().joint.j2,  points.back().joint.j3,  points.back().joint.j4,
                    //          points.back().joint.j5,  points.back().joint.j6);

                    manual_running_ = false;
                    break;
                }

                //FST_INFO("%d - %f %f %f %f %f %f", points.back().level, points.back().joint.j1,
                //          points.back().joint.j2,  points.back().joint.j3,  points.back().joint.j4, 
                //          points.back().joint.j5,  points.back().joint.j6);
            }

            return SUCCESS;
        }
        else
        {
            FST_INFO("pick time=%f, duration=%f", g_manual_pick_time, g_manual_cartesian_coeff[0].duration_3);
            return SUCCESS;
        }
    }
    else
    {
        // TODO
        FST_INFO("mode is not STEP CONTINUOUS");
        return SUCCESS;
    }

}



}
