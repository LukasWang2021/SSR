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
#include <motion_plan_additional.h>



#define MANUAL_LOCK     pthread_mutex_lock(&manual_mutex_)
#define MANUAL_UNLOCK   pthread_mutex_unlock(&manual_mutex_)
#define AUTO_LOCK       pthread_mutex_lock(&auto_mutex_)
#define AUTO_UNLOCK     pthread_mutex_unlock(&auto_mutex_)

using std::vector;
using std::string;
using fst_parameter::ParamGroup;
using fst_parameter::ParamValue;

using namespace fst_algorithm;

//std::ofstream tos("./jout.csv");
//std::ofstream jos("./path.csv");

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
    //free_command_list_ptr_ = NULL;
    //used_command_list_ptr_ = NULL;
    motion_command_index_ = 0;

    motion_state_ = IDLE;
    speed_state_ = SPEED_KEEP;
    //auto_running_ = false;
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
    //tos.close();
    //jos.close();
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

    g_log.setDisplayLevel(fst_log::MSG_LEVEL_ERROR);
    FST_INFO("Initializing ArmGroup ver%d.%d.%d ...",   motion_plan_VERSION_MAJOR,\
                                                        motion_plan_VERSION_MINOR,\
                                                        motion_plan_VERSION_PATCH);

    for (size_t i = 0; i < MOTION_POOL_CAPACITY; i++)
    {
        path_cache_[i].head = 0;
        path_cache_[i].tail = 0;
        path_cache_[i].next = &path_cache_[(i + 1) % MOTION_POOL_CAPACITY];
        path_cache_[i].prev = &path_cache_[(i - 1) % MOTION_POOL_CAPACITY];
    }

    pick_path_ptr_ = &path_cache_[MOTION_POOL_CAPACITY - 1];

    
    memset(&g_ik_reference, 0, NUM_OF_JOINT * sizeof(double));

    g_global_vel_ratio = 0.1;
    g_global_acc_ratio = 0.1;

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
        param.getParam("cartesian/orientation/omega_reference", g_orientation_omega_reference);
        param.getParam("cartesian/orientation/alpha_reference", g_orientation_alpha_reference);
        param.getParam("cartesian/orientation/linear_polation_threshold", g_ort_linear_polation_threshold);

        param.getParam("manual/step_length/joint", g_manual_step_joint);
        param.getParam("manual/step_length/position", g_manual_step_position);
        param.getParam("manual/step_length/orientation", g_manual_step_orientation);

        param.getParam("joint/omega/default", g_joint_vel_default);
        param.getParam("joint/alpha/default", g_joint_acc_default);
        param.getParam("joint/omega/limit", AXIS_IN_ALGORITHM, g_omega_limit);
        param.getParam("joint/alpha/limit", AXIS_IN_ALGORITHM, g_alpha_limit);

        FST_INFO("cycle: time=%.4f, radian=%.4f, distance=%.4f", g_cycle_time, g_cycle_radian, g_cycle_distance);
        FST_INFO("cartesian velocity: min=%.2f, max=%.2f, default=%.2f, reference=%.2f",\
                 g_cart_vel_min, g_cart_vel_max, g_cart_vel_default, g_cart_vel_reference);
        FST_INFO("cartesian acceleration: min=%.2f, max=%.2f, default=%.2f, reference=%.2f",\
                 g_cart_acc_min, g_cart_acc_max, g_cart_acc_default, g_cart_acc_reference);
        FST_INFO("cartesian orientation omega reference: %.2f", g_orientation_omega_reference);
        FST_INFO("cartesian orientation alpha reference: %.2f", g_orientation_alpha_reference);
        FST_INFO("cartesian orientation linear polation threshold: %.2f", g_ort_linear_polation_threshold);

        FST_INFO("joint: omega default=%.2f, alpha default=%.2f", g_joint_vel_default, g_joint_acc_default);
        FST_INFO("       omega limit=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                 g_omega_limit[0], g_omega_limit[1], g_omega_limit[2],
                 g_omega_limit[3], g_omega_limit[4], g_omega_limit[5]);
        FST_INFO("       alpha limit=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                 g_alpha_limit[0], g_alpha_limit[1], g_alpha_limit[2],
                 g_alpha_limit[3], g_alpha_limit[4], g_alpha_limit[5]);

        FST_INFO("manual step length: joint=%.4f rad, position=%.4f mm, orientation=%.4f rad",
                 g_manual_step_joint, g_manual_step_position, g_manual_step_orientation);
        FST_INFO("global-speed-ratio: %.0f%%, global-acceleration-ratio: %.0f%%",
                 g_global_vel_ratio * 100, g_global_acc_ratio * 100);
    }
    else
    {
        FST_ERROR("Lost config file: motion_plan.yaml");
        return param.getLastError();
    }

    if (param.loadParamFile("share/configuration/machine/dynamics.yaml"))
    {
        vector<double> torque;
        param.getParam("torque", torque);
        if (torque.size() == 6)
        {
            double tor[6];
            tor[0] = torque[0];
            tor[1] = torque[1];
            tor[2] = torque[2];
            tor[3] = torque[3];
            tor[4] = torque[4];
            tor[5] = torque[5];
            //g_dynamics_interface.setRatedTorque(tor);
            //FST_INFO("input torque: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
            //        tor[0], tor[1], tor[2], tor[3], tor[4], tor[5]);
        }
        else
        {
            FST_ERROR("Lost torque configuration");
            return MOTION_FAIL_IN_INIT;
        }
    }
    else
    {
        FST_ERROR("Lost config file: dynamics.yaml");
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
        param.getParam("hard_constraint/j1/upper", g_hard_constraint.j1.upper);
        param.getParam("hard_constraint/j1/lower", g_hard_constraint.j1.lower);
        param.getParam("hard_constraint/j2/upper", g_hard_constraint.j2.upper);
        param.getParam("hard_constraint/j2/lower", g_hard_constraint.j2.lower);
        param.getParam("hard_constraint/j3/upper", g_hard_constraint.j3.upper);
        param.getParam("hard_constraint/j3/lower", g_hard_constraint.j3.lower);
        param.getParam("hard_constraint/j4/upper", g_hard_constraint.j4.upper);
        param.getParam("hard_constraint/j4/lower", g_hard_constraint.j4.lower);
        param.getParam("hard_constraint/j5/upper", g_hard_constraint.j5.upper);
        param.getParam("hard_constraint/j5/lower", g_hard_constraint.j5.lower);
        param.getParam("hard_constraint/j6/upper", g_hard_constraint.j6.upper);
        param.getParam("hard_constraint/j6/lower", g_hard_constraint.j6.lower);

        g_hard_constraint.j1.home = 0;
        g_hard_constraint.j2.home = 0;
        g_hard_constraint.j3.home = 0;
        g_hard_constraint.j4.home = 0;
        g_hard_constraint.j5.home = 0;
        g_hard_constraint.j6.home = 0;
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
        param.getParam("soft_constraint/j2/home", g_soft_constraint.j2.home);
        param.getParam("soft_constraint/j2/upper", g_soft_constraint.j2.upper);
        param.getParam("soft_constraint/j2/lower", g_soft_constraint.j2.lower);
        param.getParam("soft_constraint/j3/home", g_soft_constraint.j3.home);
        param.getParam("soft_constraint/j3/upper", g_soft_constraint.j3.upper);
        param.getParam("soft_constraint/j3/lower", g_soft_constraint.j3.lower);
        param.getParam("soft_constraint/j4/home", g_soft_constraint.j4.home);
        param.getParam("soft_constraint/j4/upper", g_soft_constraint.j4.upper);
        param.getParam("soft_constraint/j4/lower", g_soft_constraint.j4.lower);
        param.getParam("soft_constraint/j5/home", g_soft_constraint.j5.home);
        param.getParam("soft_constraint/j5/upper", g_soft_constraint.j5.upper);
        param.getParam("soft_constraint/j5/lower", g_soft_constraint.j5.lower);
        param.getParam("soft_constraint/j6/home", g_soft_constraint.j6.home);
        param.getParam("soft_constraint/j6/upper", g_soft_constraint.j6.upper);
        param.getParam("soft_constraint/j6/lower", g_soft_constraint.j6.lower);

        param.getParam("soft_constraint_limit/j1/upper", g_soft_constraint_limit.j1.upper);
        param.getParam("soft_constraint_limit/j1/lower", g_soft_constraint_limit.j1.lower);
        param.getParam("soft_constraint_limit/j2/upper", g_soft_constraint_limit.j2.upper);
        param.getParam("soft_constraint_limit/j2/lower", g_soft_constraint_limit.j2.lower);
        param.getParam("soft_constraint_limit/j3/upper", g_soft_constraint_limit.j3.upper);
        param.getParam("soft_constraint_limit/j3/lower", g_soft_constraint_limit.j3.lower);
        param.getParam("soft_constraint_limit/j4/upper", g_soft_constraint_limit.j4.upper);
        param.getParam("soft_constraint_limit/j4/lower", g_soft_constraint_limit.j4.lower);
        param.getParam("soft_constraint_limit/j5/upper", g_soft_constraint_limit.j5.upper);
        param.getParam("soft_constraint_limit/j5/lower", g_soft_constraint_limit.j5.lower);
        param.getParam("soft_constraint_limit/j6/upper", g_soft_constraint_limit.j6.upper);
        param.getParam("soft_constraint_limit/j6/lower", g_soft_constraint_limit.j6.lower);

        g_soft_constraint_limit.j1.home = 0;
        g_soft_constraint_limit.j2.home = 0;
        g_soft_constraint_limit.j3.home = 0;
        g_soft_constraint_limit.j4.home = 0;
        g_soft_constraint_limit.j5.home = 0;
        g_soft_constraint_limit.j6.home = 0;
    }
    else
    {
        FST_ERROR("Lost config file: soft_constraints.yaml");
        return param.getLastError();
    }

    FST_INFO("Hard constraint:");
    FST_INFO("J1 - %.6f ~ %.6f", g_hard_constraint.j1.lower, g_hard_constraint.j1.upper);
    FST_INFO("J2 - %.6f ~ %.6f", g_hard_constraint.j2.lower, g_hard_constraint.j2.upper);
    FST_INFO("J3 - %.6f ~ %.6f", g_hard_constraint.j3.lower, g_hard_constraint.j3.upper);
    FST_INFO("J4 - %.6f ~ %.6f", g_hard_constraint.j4.lower, g_hard_constraint.j4.upper);
    FST_INFO("J5 - %.6f ~ %.6f", g_hard_constraint.j5.lower, g_hard_constraint.j5.upper);
    FST_INFO("J6 - %.6f ~ %.6f", g_hard_constraint.j6.lower, g_hard_constraint.j6.upper);

    FST_INFO("Soft constraint:");
    FST_INFO("J1 - [%.6f - %.6f ~ %.6f - %.6f]",\
             g_soft_constraint_limit.j1.lower, g_soft_constraint.j1.lower,
             g_soft_constraint.j1.upper, g_soft_constraint_limit.j1.upper);
    FST_INFO("J2 - [%.6f - %.6f ~ %.6f - %.6f]",\
             g_soft_constraint_limit.j2.lower, g_soft_constraint.j2.lower,
             g_soft_constraint.j2.upper, g_soft_constraint_limit.j2.upper);
    FST_INFO("J3 - [%.6f - %.6f ~ %.6f - %.6f]",\
             g_soft_constraint_limit.j3.lower, g_soft_constraint.j3.lower,
             g_soft_constraint.j3.upper, g_soft_constraint_limit.j3.upper);
    FST_INFO("J4 - [%.6f - %.6f ~ %.6f - %.6f]",\
             g_soft_constraint_limit.j4.lower, g_soft_constraint.j4.lower,
             g_soft_constraint.j4.upper, g_soft_constraint_limit.j4.upper);
    FST_INFO("J5 - [%.6f - %.6f ~ %.6f - %.6f]",\
             g_soft_constraint_limit.j5.lower, g_soft_constraint.j5.lower,
             g_soft_constraint.j5.upper, g_soft_constraint_limit.j5.upper);
    FST_INFO("J6 - [%.6f - %.6f ~ %.6f - %.6f]",\
             g_soft_constraint_limit.j6.lower, g_soft_constraint.j6.lower,
             g_soft_constraint.j6.upper, g_soft_constraint_limit.j6.upper);

    if (!isFirstConstraintCoveredBySecond(g_soft_constraint_limit, g_hard_constraint))
    {
        FST_ERROR("Soft constraint limit out of hard constraint.");
        return INVALID_PARAMETER;
    }
    if (!isFirstConstraintCoveredBySecond(g_soft_constraint, g_soft_constraint_limit))
    {
        FST_ERROR("Soft constraint out of limit.");
        return INVALID_PARAMETER;
    }


    manual_pick_time_ = 0;
    memset(&manual_traj_, 0, sizeof(ManualTrajectory));
    pthread_mutex_init(&manual_mutex_, NULL);
    pthread_mutex_init(&auto_mutex_, NULL);

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
        FST_WARN("Set global vel ratio to: %.4f", ratio);
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
    if (ratio < 0 || ratio > 1)
    {
        return INVALID_PARAMETER;
    }
    else
    {
        FST_WARN("Set global acc ratio to: %.4f", ratio);
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
    FST_INFO("setSoftConstraint:");
    FST_INFO("  J1 - %.6f ~ %.6f", cons.j1.lower, cons.j1.upper);
    FST_INFO("  J2 - %.6f ~ %.6f", cons.j2.lower, cons.j2.upper);
    FST_INFO("  J3 - %.6f ~ %.6f", cons.j3.lower, cons.j3.upper);
    FST_INFO("  J4 - %.6f ~ %.6f", cons.j4.lower, cons.j4.upper);
    FST_INFO("  J5 - %.6f ~ %.6f", cons.j5.lower, cons.j5.upper);
    FST_INFO("  J6 - %.6f ~ %.6f", cons.j6.lower, cons.j6.upper);

    if (isFirstConstraintCoveredBySecond(cons, g_soft_constraint_limit))
    {
        g_soft_constraint = cons;
        FST_INFO("Success!");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Failed: new constraint out of limit");
        return INVALID_PARAMETER;
    }
}

//------------------------------------------------------------
// Function:    getSoftConstraintLimit
// Summary: To get soft joint constraint limit from algorithm.
// In:      None
// Out:     None
// Return:  soft constraint
//------------------------------------------------------------
const JointConstraint& ArmGroup::getSoftConstraintLimit(void)
{
    return g_soft_constraint_limit;
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
// Function:    timeBeforeDeadline
// Summary: To get remaining time before deadline of the trajectory.
// In:      None
// Out:     None
// Return:  remaining time
//------------------------------------------------------------
double ArmGroup::timeBeforeDeadline(void)
{
    double deadline = 9999999.99;
    double time = 0;

    //if (auto_running_)
    if (motion_state_ == AUTO_RUNNING || motion_state_ == AUTO_RUNNING_TO_PAUSE)
    {
        ControlPointCache *ptr = pick_path_ptr_;

        for (size_t i = 0; i < MOTION_POOL_CAPACITY; i++)
        {
            if (ptr->deadline > 0 && ptr->next->valid)
            {
                ptr = ptr->next;
            }
            else
            {
                break;
            }
        }

        if (ptr->deadline > 0)
        {
            deadline = ptr->deadline;
        }

        time = deadline > pick_time_ ? deadline - pick_time_ : 0;
    }

    //FST_LOG("time before deadline: %.6f", time);
    return time;
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

    //if (auto_running_)
    if (motion_state_ == AUTO_RUNNING || motion_state_ == AUTO_RUNNING_TO_PAUSE)
    {
        // To be completed
        len = 10;
        /*
        ControlPoint *path = pick_path_ptr_->path;
        size_t tail = pick_path_ptr_->tail;

        if (pick_segment_ < tail)
        {
            if (pick_time_ < path[tail - 1].time_from_start)
            {
                len = ceil((path[tail - 1].time_from_start - pick_time_) / g_cycle_time);
            }
            else
            {
                if (pick_path_ptr_->next->valid)
                {
                    path = pick_path_ptr_->next->path;
                    tail = pick_path_ptr_->next->tail;
                    len = ceil((path[tail - 1].time_from_start - pick_time_) / g_cycle_time);
                }
                else
                {
                    len = 1;
                }
            }
        }
        */
    }
    else if (manual_running_)
    {
        if (manual_traj_.duration > manual_pick_time_)
            len = ceil((manual_traj_.duration - manual_pick_time_) / g_cycle_time);
        else
            len = 1;
        //FST_INFO("manual-pick-time=%.4f, manual-duration=%.4f",
        //          manual_pick_time_, manual_traj_.duration);
    }

    //FST_LOG("fifo len=%d", len);
/*
    if (len > 0 && len < 20)
    {
        FST_INFO("pick seg=%d, t_tail=%d", pick_segment_, pick_path_ptr_->tail);
        FST_INFO("auto: pick time=%f, last seg time=%f, len=%d", pick_time_, pick_path_ptr_->path[pick_path_ptr_->tail - 1].time_from_start, len);
        FST_INFO("manual: pick time=%f, total time=%f, len=%d", manual_pick_time_, g_manual_joint_coeff[0].duration_3, len);
    }
    */
    
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

    //FST_WARN("getPointFromFIFO: auto = %d, manul = %d", motion_state_ == AUTO_RUNNING, manual_running_);

    if (motion_state_ == AUTO_RUNNING || motion_state_ == AUTO_RUNNING_TO_PAUSE)
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
    //if (auto_running_ || manual_running_)
    if (motion_state_ != IDLE || manual_running_)
    {
        FST_ERROR("setStartState: cannot set start state until ArmGroup is standing by.");
        return INVALID_SEQUENCE;
    }

    g_start_joint       = joint;
    g_ik_reference      = joint;

    FST_INFO("setStartState: joint=%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
             g_start_joint.j1, g_start_joint.j2, g_start_joint.j3,
             g_start_joint.j4, g_start_joint.j5, g_start_joint.j6);

    return SUCCESS;
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
    FST_INFO("clearArmGroup -------------");


    pick_time_ = 0;
    pick_segment_ = 0;

    motion_state_ = IDLE;
    speed_state_ = SPEED_KEEP;
    //auto_running_ = false;
    manual_running_ = false;
    manual_pick_time_ = 0;
    memset(&manual_traj_, 0, sizeof(ManualTrajectory));

    while (pick_path_ptr_->valid)
    {
        pick_path_ptr_->valid = false;
        pick_path_ptr_ = pick_path_ptr_->next;
    }

    traj_fifo_.clear();
    pick_segment_ = 0;

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

    if (!isJointInConstraint(g_start_joint, g_soft_constraint))
    {
        err = JOINT_OUT_OF_CONSTRAINT;
        FST_ERROR("autoMove: start-joint is out of soft constraint, autoMove is disabled");
        return err;
    }

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
        //if (auto_running_ == false)
        if (motion_state_ == IDLE)
        {
            // idle -> running
            for (size_t i = 0; i < MOTION_POOL_CAPACITY; i++)
            {
                if (pick_path_ptr_->valid == false)
                {
                    pick_path_ptr_ = pick_path_ptr_->next;
                }
                else
                {
                    break;
                }
            }

            if (pick_path_ptr_->valid)
            {
                pick_path_ptr_->path[0].time_from_start = 0;
                pick_time_ = g_cycle_time;
                pick_segment_ = 1;
                motion_state_ = AUTO_RUNNING;
                speed_state_ = SPEED_UP;
                //auto_running_ = true;
                FST_INFO("Start motion, pick-cache=%p, pick-seg=%d, pick-time=%.4f",
                         pick_path_ptr_, pick_segment_, pick_time_);
            }
            else
            {
                FST_ERROR("No valid path found in cache, fail to start motion");
                err = MOTION_INTERNAL_FAULT;
                return err;
            }
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
    ErrorCode err = SUCCESS;
    ManualDirection dirs[6];

    if (directions.size() != 6)
    {
        FST_ERROR("manualMove: 6 manual directions needed, %d given", directions.size());
        return INVALID_PARAMETER;
    }

    for (size_t i = 0; i < 6; i++)
    {
        if (directions[i] != STANDBY && directions[i] != INCREASE && directions[i] != DECREASE)
        {
            FST_ERROR("manualMove: unsupported direction, dir[%d] = %d", i, directions[i]);
            err = INVALID_PARAMETER;
        }
        else
        {
            dirs[i] = directions[i];
        }
    }

    if (err != SUCCESS)
    {
        FST_ERROR("manualMove: given directions invalid, manual abort");
        return err;
    }
    
    MANUAL_LOCK;
    if (manual_traj_.mode == CONTINUOUS)
    {
        if (manual_running_)
        {
            err = manual_.stepTeach(dirs, manual_pick_time_, manual_traj_);

            if (err != SUCCESS)
            {
                FST_ERROR("manualMove: fail to stop in manual continuous mode");
            }
        }
        else
        {
            if (dirs[0] != STANDBY || dirs[1] != STANDBY || dirs[2] != STANDBY || 
                dirs[3] != STANDBY || dirs[4] != STANDBY || dirs[5] != STANDBY)
            {
                manual_traj_.joint_start = g_start_joint;
                manual_traj_.joint_ending = g_start_joint;
                err = manual_.stepTeach(dirs, 0, manual_traj_);
                if (err == SUCCESS)
                {
                    manual_running_ = true;
                    manual_pick_time_ = g_cycle_time;
                    //FST_INFO("manual-traj: mode=%d,frame=%d,direct=%d %d %d %d %d %d, duration=%.4f",
                    //        manual_traj_.mode, manual_traj_.frame,
                    //        manual_traj_.direction[0], manual_traj_.direction[1], manual_traj_.direction[2],
                    //        manual_traj_.direction[3], manual_traj_.direction[4], manual_traj_.direction[5],
                    //        manual_traj_.duration);
                }
                else
                {
                    FST_ERROR("manualMove: failed, err=0x%llx", err);
                }
            }
        }
    }
    else if (manual_traj_.mode == STEP)
    {
        // manual step
        if (!manual_running_)
        {
            if (dirs[0] != STANDBY || dirs[1] != STANDBY || dirs[2] != STANDBY || 
                dirs[3] != STANDBY || dirs[4] != STANDBY || dirs[5] != STANDBY)
            {
                manual_traj_.joint_start = g_start_joint;
                ErrorCode err = manual_.stepTeach(dirs, 0, manual_traj_);
                
                if (err == SUCCESS)
                {
                    manual_running_ = true;
                    manual_pick_time_ = g_cycle_time;
                    //FST_INFO("manual-traj: mode=%d,frame=%d,direct=%d %d %d %d %d %d, duration=%.4f",
                    //        manual_traj_.mode, manual_traj_.frame,
                    //        manual_traj_.direction[0], manual_traj_.direction[1], manual_traj_.direction[2],
                    //        manual_traj_.direction[3], manual_traj_.direction[4], manual_traj_.direction[5],
                    //        manual_traj_.duration);
                }
                else
                {
                    FST_ERROR("manualMove: failed, err=0x%llx", err);
                }
            }
        }
        else
        {
            FST_WARN("manualMove: last manual command is running, waiting before it finish ...");
        }
    }
    else
    {
        err = INVALID_SEQUENCE;
        FST_ERROR("manualMove: manual-mode=APOINT, but given 6 directions instead of target");
    }
    MANUAL_UNLOCK;

    return err;
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
        ErrorCode err = SUCCESS;

        MANUAL_LOCK;
        if (manual_traj_.mode == APOINT && manual_traj_.frame == JOINT)
        {
            if (manual_running_ == false)
            {
                manual_traj_.joint_start = g_start_joint;
                err = manual_.stepTeach(joint, 0, manual_traj_);
                
                if (err == SUCCESS)
                {
                    manual_running_ = true;
                    manual_pick_time_ = g_cycle_time;
                }
                else
                {
                    FST_ERROR("manualMove: failed, err=0x%x", err);
                }
            }
            else
            {
                FST_ERROR("manualMove: last manual command is running");
                err = INVALID_SEQUENCE;
            }
        }
        else
        {
            FST_ERROR("manualMove: joint target given, but mode != APOINT, current-mode=%d", manual_traj_.mode);
            err = MOTION_INTERNAL_FAULT;
        }
        MANUAL_UNLOCK;

        return err;
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
    if (manual_running_ == false)
    {
        Joint jnt;
        ErrorCode err = inverseKinematics(pose, g_ik_reference, jnt);
        FST_INFO("manualMove: pose=%.2f %.2f %.2f - %.4f %.4f %.4f",
                 pose.position.x, pose.position.y, pose.position.z,
                 pose.orientation.a, pose.orientation.b, pose.orientation.c);

        if (err == SUCCESS)
        {
            FST_INFO("           joint=%.6f %.6f %.6f %.6f %.6f %.6f",
                     jnt.j1, jnt.j2, jnt.j3, jnt.j4, jnt.j5, jnt.j6);
            manual_traj_.frame = JOINT;
            //manual_traj_.mode  = APOINT;
            manual_traj_.joint_start = g_start_joint;
            err = manual_.stepTeach(jnt, 0, manual_traj_);
            if (err == SUCCESS)
            {
                manual_running_ = true;
                manual_pick_time_ = g_cycle_time;
            }
            else
            {
                FST_ERROR("manualMove: failed, err=0x%x", err);
            }
            /*
            ManualFrame frame = manual_.getFrame();
            manual_.setFrame(JOINT);
            err = manualMove(jnt);
            manual_.setFrame(frame);
            */
        }
        else
        {
            FST_ERROR("Fail to find a valid IK result with given pose");
        }

        return err;
    }
    else
    {
        FST_ERROR("manualMove: last manual command is running");
        return INVALID_SEQUENCE;
    }
}

ErrorCode ArmGroup::manualStop(void)
{
    FST_INFO("manualStop: stop request accepted.");
    MANUAL_LOCK;
    if (manual_running_)
    {
        manual_.stopTeach(manual_pick_time_, manual_traj_);
    }
    MANUAL_UNLOCK;

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
    MANUAL_LOCK;
    if (!manual_running_)
    {
        manual_traj_.frame = frame;
    }
    MANUAL_UNLOCK;
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
    MANUAL_LOCK;
    if (!manual_running_)
    {
        manual_traj_.mode = mode;
    }
    MANUAL_UNLOCK;
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
    FST_INFO("setManualCartPosStep: %.4f", step);
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

/*
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
*/
















MotionCommand* ArmGroup::getMotionCommandPtr(void)
{
    auto ptr = &motion_command_pool_[motion_command_index_];
    motion_command_index_ = (motion_command_index_ + 1) % MOTION_POOL_CAPACITY;
    return ptr;
}

/*
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

        char buf[MOTION_POOL_CAPACITY * 16];
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
*/


bool tflag = false;

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
             tar.joint_target.j1, tar.joint_target.j2, tar.joint_target.j3,
             tar.joint_target.j4, tar.joint_target.j5, tar.joint_target.j6);
    FST_INFO("  velocity = %.0f%%, acceleration = %.0f%%, SV = %.0f%%", tar.vel * 100, tar.acc * 100, tar.cnt * 100);

    if (!isJointInConstraint(g_start_joint, g_soft_constraint))
    {
        FST_ERROR("  motion start joint out of constraint");
        return INVALID_PARAMETER;
    }

    // if target joint out of constraint, we should move and stop the robot near the constraint
    /*
    if (!isJointInConstraint(tar.joint_target, g_soft_constraint))
    {
        FST_ERROR("  motion target joint out of constraint");
        return INVALID_PARAMETER;
    }
    */

    FST_INFO("Parameter check passed, planning path ...");

    MotionCommand *cmd = getMotionCommandPtr();
    cmd->setStartJoint(g_start_joint);
    cmd->setTarget(tar, id);
    ErrorCode err = cmd->planPath();

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to plan path error=%x, planning abort.", err);
        return err;
    }

    FST_INFO("Path plan finished successfully with %d points, picking point ...", cmd->getPathLength());

    vector<PathPoint> points;

    cmd->pickAllPoint(points);

    if (points.size() > PATH_FIFO_CAPACITY)
    {
        FST_ERROR("Path point overload, point num=%d, FIFO capacity=%d.", points.size(), PATH_FIFO_CAPACITY);
        err = MOTION_INTERNAL_FAULT;
        return err;
    }

    FST_INFO("  %d points picked out", points.size() - 1);

    size_t cnt = 0;
    ControlPointCache *cache_ptr = pick_path_ptr_->next;

    while (cache_ptr->valid && cnt < MOTION_POOL_CAPACITY)
    {
        cache_ptr = cache_ptr->next;
        cnt ++;
    }

    if (cache_ptr->valid)
    {
        err = MOTION_INTERNAL_FAULT;
        FST_ERROR("No enough path cache available, err=0x%llx", err);

        for (cnt = 0; cnt < MOTION_POOL_CAPACITY; cnt++)
        {
            FST_INFO("cache %d: address=0x%p busy=%d", cnt, &path_cache_[cnt], path_cache_[cnt].valid);
        }

        return err;
    }

    ControlPoint *path = cache_ptr->path;
    size_t  &head = cache_ptr->head;
    size_t  &tail = cache_ptr->tail;

    head = 0;
    tail = 0;

    MotionTime command_duration = points[0].source->getCommandDuration();
    
    for (vector<PathPoint>::iterator it = points.begin(); it != points.end(); ++it)
    {
        //FST_INFO("%.6f %.6f %.6f %.6f %.6f %.6f", it->joint.j1, it->joint.j2, it->joint.j3,
        //                                          it->joint.j4, it->joint.j5, it->joint.j6);

        path[tail].path_point = *it;
        err = convertPathPoint(path[tail]);

        if (err == SUCCESS)
        {
            path[tail].command_duration = command_duration;
            tail ++;
        }
        else
        {
            FST_ERROR("autoJoint: fail to convert path point: stamp=%d, err=0x%llx", it->stamp, err);
            break;
        }
    }

    FST_INFO("  %d points fill in the path cache.", tail);

    if (tail > 0)
    {
        err = preTrajPlan(cache_ptr);
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

    PoseEuler pose_start;
    forwardKinematics(g_start_joint, pose_start);
    FST_INFO("  start joint: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             g_start_joint.j1, g_start_joint.j2, g_start_joint.j3, 
             g_start_joint.j4, g_start_joint.j5, g_start_joint.j6);
    FST_INFO("  start pose:  %f %f %f - %f %f %f",  pose_start.position.x,
                                                    pose_start.position.y,
                                                    pose_start.position.z,
                                                    pose_start.orientation.a,
                                                    pose_start.orientation.b,
                                                    pose_start.orientation.c);
    FST_INFO("  target pose: %f %f %f - %f %f %f",  target.pose_target.position.x,
                                                    target.pose_target.position.y,
                                                    target.pose_target.position.z,
                                                    target.pose_target.orientation.a,
                                                    target.pose_target.orientation.b,
                                                    target.pose_target.orientation.c);
    FST_INFO("  velocity=%.2f, acceleration=%.2f, cnt=%.2f", tar.vel, tar.acc, tar.cnt);

    FST_INFO("Parameter check passed, planning path ...");

    MotionCommand *cmd = getMotionCommandPtr();

    cmd->setStartJoint(g_start_joint);
    cmd->setTarget(tar, id);

    ErrorCode err = cmd->planPath();

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to plan path error=%x, planning abort.", err);
        return err;
    }

    FST_INFO("Path plan finished successfully with %d points, picking point ...", cmd->getPathLength());

    vector<PathPoint> points;

    err = cmd->pickAllPoint(points);

    if (points.size() > PATH_FIFO_CAPACITY)
    {
        FST_ERROR("Path point overload, point num=%d, FIFO capacity=%d.", points.size(), PATH_FIFO_CAPACITY);
        err = MOTION_INTERNAL_FAULT;
        return err;
    }

    /*
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
    */

    FST_INFO("  %d points picked out", points.size());

    size_t cnt = 0;
    ControlPointCache *cache_ptr = pick_path_ptr_->next;

    while (cache_ptr->valid && cnt < MOTION_POOL_CAPACITY)
    {
        cache_ptr = cache_ptr->next;
        cnt ++;
    }

    if (cache_ptr->valid)
    {
        err = MOTION_INTERNAL_FAULT;
        FST_ERROR("No enough path cache available, err=0x%llx", err);

        for (cnt = 0; cnt < MOTION_POOL_CAPACITY; cnt++)
        {
            FST_INFO("cache %d: address=0x%p busy=%d", cnt, &path_cache_[cnt], path_cache_[cnt].valid);
        }

        return err;
    }

    ControlPoint *path = cache_ptr->path;
    size_t  &head = cache_ptr->head;
    size_t  &tail = cache_ptr->tail;

    head = 0;
    tail = 0;

    FST_INFO("Converting path point to joint space ...");
    MotionTime command_duration = points[0].source->getCommandDuration();

    for (vector<PathPoint>::iterator it = points.begin(); it != points.end(); ++it)
    {
        //FST_INFO("%.6f %.6f %.6f - %.6f %.6f %.6f %.6f",
        //          it->pose.position.x, it->pose.position.y, it->pose.position.z,
        //          it->pose.orientation.w, it->pose.orientation.x, it->pose.orientation.y, it->pose.orientation.z);

        path[tail].path_point = *it;
        err = convertPathPoint(path[tail]);

        if (err == SUCCESS)
        {
            path[tail].command_duration = command_duration;
            tail ++;
        }
        else
        {
            FST_ERROR("autoLine: fail to convert path point: stamp=%d, err=0x%llx", it->stamp, err);
            break;
        }

        /*
        FST_INFO("%d - %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",\
                 path[tail - 1].path_point.stamp,\
                 path[tail - 1].point.joint[0],\
                 path[tail - 1].point.joint[1],\
                 path[tail - 1].point.joint[2],\
                 path[tail - 1].point.joint[3],\
                 path[tail - 1].point.joint[4],\
                 path[tail - 1].point.joint[5]);
        */
    }

    FST_INFO("  %d points converted into joint space and filled in the path cache.", tail);

    if (tail > 0)
    {
        err = preTrajPlan(cache_ptr);
    }

    return err;
}

ErrorCode ArmGroup::preTrajPlan(ControlPointCache *cache)
{
    FST_INFO("Create first stage trajectory head=%d, tail=%d...", cache->head, cache->tail);
    ErrorCode err = SUCCESS;

    ControlPoint *path = cache->path;
    size_t head = cache->head;
    size_t tail = cache->tail;

    memset(path[0].forward_point.omega, 0, NUM_OF_JOINT * sizeof(double));
    memset(path[0].forward_point.alpha, 0, NUM_OF_JOINT * sizeof(double));
    memset(path[tail - 1].backward_point.omega, 0, NUM_OF_JOINT * sizeof(double));
    
    err = planFirstStageTraj(path, head, tail);
    
    if (err == SUCCESS)
    {
        size_t  index;
        double  cnt, smooth_duration;
        size_t  &smooth_in  = cache->smooth_in_stamp;
        size_t  &smooth_out = cache->smooth_out_stamp;
        MotionTime command_duration = path[0].path_point.source->getCommandDuration();
        smooth_in  = 1;
        smooth_out = tail - 1;

        //if (auto_running_ == true)
        if (motion_state_ == AUTO_RUNNING)
        {
            if (cache->prev->smooth_out_stamp < cache->prev->tail - 1)
            {
                // addition smooth
                FST_INFO("Start from smooth");
                //ControlPoint *prev_path = cache->prev->path;
                //size_t prev_stamp = cache->prev->smooth_out_stamp;
                //FST_INFO("  prev-smooth-out-stamp=%d, time=%.4f", prev_stamp, prev_path[prev_stamp].time_from_start);
                //FST_INFO("  prev-smooth-out-joint=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                //        prev_path[prev_stamp].point.joint[0], prev_path[prev_stamp].point.joint[1],
                //        prev_path[prev_stamp].point.joint[2], prev_path[prev_stamp].point.joint[3],
                //        prev_path[prev_stamp].point.joint[4], prev_path[prev_stamp].point.joint[5]);
                
                cnt = cache->prev->path[0].path_point.source->getCommandCNT();
                smooth_duration = cnt > MINIMUM_E6 ? command_duration / cnt : 99.99;
                FST_INFO("  last-SV=%.4f, command-duration=%.4f, smooth-in-duration=%.4f",
                         cnt, command_duration, smooth_duration);
                                                
                for (index = 1; index < tail - 1; index++)
                {
                    if (path[index + 1].forward_duration > smooth_duration)
                    {
                        continue;
                    }
                    else
                    {
                        smooth_in = index;
                        break;
                    }
                }
            }
            else
            {
                // start from Standing-By
                FST_INFO("Start from Standby");
                smooth_in = 1;
            }
        }
        else
        {
            // start from idle
            FST_INFO("Start from idle");
            smooth_in = 1;
        }

        cnt = path[0].path_point.source->getCommandCNT();
        smooth_duration = cnt > MINIMUM_E6 ? command_duration / cnt : 99.99;
        FST_INFO("  curr-SV=%.4f, command-duration=%.4f, smooth-out-duration=%.4f",
                 cnt, command_duration, smooth_duration);

        for (index = tail - 1; index > 1; index--)
        {
            if (path[index - 1].backward_duration < smooth_duration)
            {
                smooth_out = index;
                break;
            }
            else
            {
                continue;
            }
        }


        cache->deadline = -1;
        cache->valid = true;

        memcpy(&g_start_joint, path[tail - 1].forward_point.joint, sizeof(Joint));
        g_ik_reference = g_start_joint;

        FST_INFO("Success, smooth-in=%d, sommth-out=%d", smooth_in, smooth_out);

/*
        FST_INFO("autoLine Trajectory--------------------------");
        for (size_t i = 0; i < tail; i++)
        {
            if (path[i].smooth)
            {
                FST_INFO("%d - stamp:%d time_from_start:%.4f duration:%.4f - smooth",
                         i, path[i].path_point.stamp, path[i].time_from_start, path[i].duration);
                FST_INFO("    j1: %.6f %.6f %.6f %.6f %.6f %.6f",
                         path[i].coeff[0][0], path[i].coeff[0][1], path[i].coeff[0][2],
                         path[i].coeff[0][3], path[i].coeff[0][4], path[i].coeff[0][5]);
                FST_INFO("    j2: %.6f %.6f %.6f %.6f %.6f %.6f",
                         path[i].coeff[1][0], path[i].coeff[1][1], path[i].coeff[1][2],
                         path[i].coeff[1][3], path[i].coeff[1][4], path[i].coeff[1][5]);
                FST_INFO("    j3: %.6f %.6f %.6f %.6f %.6f %.6f",
                         path[i].coeff[2][0], path[i].coeff[2][1], path[i].coeff[2][2],
                         path[i].coeff[2][3], path[i].coeff[2][4], path[i].coeff[2][5]);
                FST_INFO("    j4: %.6f %.6f %.6f %.6f %.6f %.6f",
                         path[i].coeff[3][0], path[i].coeff[3][1], path[i].coeff[3][2],
                         path[i].coeff[3][3], path[i].coeff[3][4], path[i].coeff[3][5]);
                FST_INFO("    j5: %.6f %.6f %.6f %.6f %.6f %.6f",
                         path[i].coeff[4][0], path[i].coeff[4][1], path[i].coeff[4][2],
                         path[i].coeff[4][3], path[i].coeff[4][4], path[i].coeff[4][5]);
                FST_INFO("    j6: %.6f %.6f %.6f %.6f %.6f %.6f",
                         path[i].coeff[5][0], path[i].coeff[5][1], path[i].coeff[5][2],
                         path[i].coeff[5][3], path[i].coeff[5][4], path[i].coeff[5][5]);
            }
            else
            {
                FST_INFO("%d - stamp:%d time_from_start:%.4f duration:%.4f",
                         i, path[i].path_point.stamp, path[i].time_from_start, path[i].duration);
                FST_INFO("    joint: %.6f %.6f %.6f %.6f %.6f %.6f",
                         path[i].point.joint[0], path[i].point.joint[1], path[i].point.joint[2],
                         path[i].point.joint[3], path[i].point.joint[4], path[i].point.joint[5]);
                FST_INFO("    omega: %.6f %.6f %.6f %.6f %.6f %.6f",
                         path[i].point.omega[0], path[i].point.omega[1], path[i].point.omega[2],
                         path[i].point.omega[3], path[i].point.omega[4], path[i].point.omega[5]);
                FST_INFO("    alpha: %.6f %.6f %.6f %.6f %.6f %.6f",
                         path[i].point.alpha[0], path[i].point.alpha[1], path[i].point.alpha[2],
                         path[i].point.alpha[3], path[i].point.alpha[4], path[i].point.alpha[5]);
            }
        }
        */
        /*
        for (size_t i = 0; i < t_tail_; ++i)
        {
            FST_INFO("%d - time_from_start:%.4f duration:%.4f",
                     t_path_[i].path_point.stamp, t_path_[i].time_from_start, t_path_[i].duration);
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
        */
    }
    else
    {
        FST_ERROR("Fail to create first stage trajectory");
    }

    return err;
}

ErrorCode ArmGroup::smoothJoint2Joint(const JointState &ps, const JointState &pe,
                                      MotionTime smooth_time, TrajSegment &seg)
{
    //FST_INFO("ps: stamp=%d, last_stamp=%d, duration=%.6f, exp_duration=%.6f",
    //         ps.path_point.stamp, ps.path_point.source->getPathLength(), ps.duration, ps.expect_duration);
    //FST_INFO("pe: stamp=%d, duration=%.6f, exp_duration=%.6f",
    //         pe.path_point.stamp, pe.duration, pe.expect_duration);

    /*
    FST_INFO("js=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             ps.point.joint[0], ps.point.joint[1], ps.point.joint[2],
             ps.point.joint[3], ps.point.joint[4], ps.point.joint[5]);
    FST_INFO("ws=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             ps.point.omega[0], ps.point.omega[1], ps.point.omega[2],
             ps.point.omega[3], ps.point.omega[4], ps.point.omega[5]);
    FST_INFO("as=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             ps.point.alpha[0], ps.point.alpha[1], ps.point.alpha[2],
             ps.point.alpha[3], ps.point.alpha[4], ps.point.alpha[5]);
    FST_INFO("je=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             pe.point.joint[0], pe.point.joint[1], pe.point.joint[2],
             pe.point.joint[3], pe.point.joint[4], pe.point.joint[5]);
    FST_INFO("we=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             pe.point.omega[0], pe.point.omega[1], pe.point.omega[2],
             pe.point.omega[3], pe.point.omega[4], pe.point.omega[5]);
    FST_INFO("ae=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             pe.point.alpha[0], pe.point.alpha[1], pe.point.alpha[2],
             pe.point.alpha[3], pe.point.alpha[4], pe.point.alpha[5]);
    */

    double t[6] = { 1,
                    smooth_time,
                    smooth_time * smooth_time,
                    smooth_time * smooth_time * smooth_time,
                    smooth_time * smooth_time * smooth_time * smooth_time,
                    smooth_time * smooth_time * smooth_time * smooth_time * smooth_time
                  };

    for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
    {
        seg.coeff[i][0] = ps.joint[i];
        seg.coeff[i][1] = ps.omega[i];
        seg.coeff[i][2] = 0.5 * ps.alpha[i];
        seg.coeff[i][3] = ((pe.joint[i] - ps.joint[i]) * 10 - t[1] * 
                          (pe.omega[i] * 4 + ps.omega[i] * 6) + t[2] *
                          (pe.alpha[i] * 0.5 - ps.alpha[i] * 1.5)) / t[3];
        seg.coeff[i][4] = ((ps.joint[i] - pe.joint[i]) * 15 + t[1] *
                          (ps.omega[i] * 8 + pe.omega[i] * 7) + t[2] *
                          (ps.alpha[i] * 1.5 - pe.alpha[i])) / t[4];
        seg.coeff[i][5] = ((pe.joint[i] - ps.joint[i]) * 6 - t[1] * 3 *
                          (ps.omega[i] + pe.omega[i]) + t[2] * 0.5 *
                          (pe.alpha[i] - ps.alpha[i])) / t[5];
    }

    seg.duration = smooth_time;
    //seg.time_from_start = ps.time_from_start + smooth_time;

    return SUCCESS;    
}

ErrorCode ArmGroup::convertPathPoint(ControlPoint &cp)
{
    ErrorCode err = SUCCESS;

    PathPoint  &pp = cp.path_point;
    JointState &fp = cp.forward_point;
    JointState &bp = cp.backward_point;

    if (pp.type == MOTION_JOINT)
    {
        if (isJointInConstraint(pp.joint, g_soft_constraint))
        {
            cp.command_duration = -1;
            cp.time_from_start = -1;
            cp.forward_duration = -1;
            cp.backward_duration = -1;

            memcpy(fp.joint, &pp.joint, NUM_OF_JOINT * sizeof(double));
            memcpy(bp.joint, &pp.joint, NUM_OF_JOINT * sizeof(double));
        }
        else
        {
            FST_ERROR("convertPathPoint: movej path point out of soft constraint");
            FST_ERROR("  joint=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                      pp.joint.j1, pp.joint.j2, pp.joint.j3, pp.joint.j4, pp.joint.j5, pp.joint.j6);
            err = JOINT_OUT_OF_CONSTRAINT;
        }
    }
    else if (pp.type == MOTION_LINE || pp.type == MOTION_CIRCLE)
    {
        Joint jnt;
        err = chainIK(pp.pose, g_ik_reference, jnt);

        if (err == SUCCESS)
        {
            if (isJointInConstraint(jnt, g_soft_constraint))
            {
                cp.command_duration = -1;
                cp.time_from_start = -1;
                cp.forward_duration = -1;
                cp.backward_duration = -1;

                memcpy(fp.joint, &jnt, NUM_OF_JOINT * sizeof(double));
                memcpy(bp.joint, &jnt, NUM_OF_JOINT * sizeof(double));

                //FST_LOG("stamp-%d: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", cp.path_point.stamp,
                //        jnt.j1, jnt.j2, jnt.j3, jnt.j4, jnt.j5, jnt.j6);
            }
            else
            {
                FST_ERROR("convertPathPoint: movel path point (stamp=%d) out of soft constraint", cp.path_point.stamp);
                FST_ERROR("   pose=%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f",
                          pp.pose.position.x, pp.pose.position.y, pp.pose.position.z,
                          pp.pose.orientation.w, pp.pose.orientation.x, pp.pose.orientation.y, pp.pose.orientation.z);
                FST_ERROR("  joint=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                          jnt.j1, jnt.j2, jnt.j3, jnt.j4, jnt.j5, jnt.j6);
                err = JOINT_OUT_OF_CONSTRAINT;
            }
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
                    jnt.j1, jnt.j2, jnt.j3, jnt.j4, jnt.j5, jnt.j6);
        }
    }
    else
    {
        FST_ERROR("convertPathPoint: invalid point type(=%d),", pp.type);
        err = MOTION_INTERNAL_FAULT;
    }

    //jos << fp.joint[0] << "," << fp.joint[1] << "," << fp.joint[2] << ","
    //    << fp.joint[3] << "," << fp.joint[4] << "," << fp.joint[5] << endl;

    return err;
}

ErrorCode ArmGroup::planFirstStageTraj(ControlPoint *path, size_t head, size_t tail)
{
    ErrorCode err = SUCCESS;

    double  f_inertia[AXIS_IN_ALGORITHM];
    double  b_inertia[AXIS_IN_ALGORITHM];
    Alpha   f_upper[AXIS_IN_ALGORITHM];
    Alpha   f_lower[AXIS_IN_ALGORITHM];
    Alpha   b_upper[AXIS_IN_ALGORITHM];
    Alpha   b_lower[AXIS_IN_ALGORITHM];

    size_t spd_up = 0, spd_down = tail;
    size_t ind_f = head + 1;
    size_t ind_b = tail - 1;
    MotionTime  duration_f = 99.99;
    MotionTime  duration_b = 99.99;
    MotionTime  cmd_duration = path[0].command_duration;

    // trajectory generation
    computeAlphaLimit(path[0].forward_point.joint, path[0].forward_point.omega, f_upper, f_lower);
    g_dynamics_interface.getInertia(path[0].forward_point.joint, f_inertia);

    computeAlphaLimit(path[tail - 1].backward_point.joint, path[tail - 1].backward_point.omega, b_upper, b_lower);
    g_dynamics_interface.getInertia(path[tail - 1].backward_point.joint, b_inertia);

    /*
    double *joint = path[0].forward_point.joint;
    double *omega = path[0].forward_point.omega;
    FST_INFO("input-joint:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", 
             joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
    FST_INFO("input-omega:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", 
             omega[0], omega[1], omega[2], omega[3], omega[4], omega[5]);
    FST_INFO("alpha-upper:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             f_upper[0], f_upper[1], f_upper[2], f_upper[3], f_upper[4], f_upper[5]);
    FST_INFO("alpha-lower:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             f_lower[0], f_lower[1], f_lower[2], f_lower[3], f_lower[4], f_lower[5]);
    joint = path[tail - 1].backward_point.joint;
    omega = path[tail - 1].backward_point.omega;
    FST_INFO("input-joint:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
    FST_INFO("input-omega:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             omega[0], omega[1], omega[2], omega[3], omega[4], omega[5]);
    FST_INFO("alpha-upper:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             b_upper[0], b_upper[1], b_upper[2], b_upper[3], b_upper[4], b_upper[5]);
    FST_INFO("alpha-lower:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
             b_lower[0], b_lower[1], b_lower[2], b_lower[3], b_lower[4], b_lower[5]);
    */


    double  last_duration = 99.99;

    while (duration_b > cmd_duration + MINIMUM_E6 && ind_b > 0)
    {
        err = createBackwardSpeedUpTraj(path[ind_b - 1], path[ind_b], cmd_duration, b_upper, b_lower);

        if (err == SUCCESS)
        {
            duration_b = path[ind_b].backward_duration;
            memcpy(path[ind_b].inertia, b_inertia, AXIS_IN_ALGORITHM * sizeof(double));
            //FST_LOG("back-index=%d, duration=%.6f, last-duration=%.6f, cmd-duration=%.6f",
            //        ind_b, duration_b, last_duration, cmd_duration);

            if (duration_b + MINIMUM_E6 > last_duration)
            {
                break;
            }
            else
            {
                last_duration = duration_b;
            }

            if (duration_b > cmd_duration + MINIMUM_E6)
            {
                spd_down = ind_b;
                ind_b --;
            }
            else
            {
                spd_down = ind_b;
                break;
            }
        }
        else
        {
            break;
        }
    }

    last_duration = 99.99;

    while (duration_f > cmd_duration + MINIMUM_E6 && ind_f < tail)
    {
        err = createSpeedUpTraj(path[ind_f - 1], path[ind_f], cmd_duration, f_upper, f_lower);

        if (err == SUCCESS)
        {
            duration_f = path[ind_f].forward_duration;
            memcpy(path[ind_f].inertia, f_inertia, AXIS_IN_ALGORITHM * sizeof(double));
            //FST_LOG("fore-index=%d, duration=%.6f, last-duration=%.6f, cmd-duration=%.6f",
            //        ind_f, duration_f, last_duration, cmd_duration);

            if (duration_f + MINIMUM_E6 > last_duration)
            {
                break;
            }
            else
            {
                last_duration = duration_f;
            }

            if (duration_f > cmd_duration + MINIMUM_E6)
            {
                spd_up = ind_f;
                ind_f ++;
            }
            else
            {
                spd_up = ind_f;
                break;
            }
        }
        else
        {
            break;
        }
    }

    ind_f = head + 1;
    ind_b = tail - 1;
    duration_f = path[ind_f].forward_duration;
    duration_b = path[ind_b].backward_duration;

    while (ind_f != ind_b)
    {
        //FST_LOG("ind-f=%d, ind-b=%d, duration-f=%.6f, duration-nf=%.6f, duration-b=%.6f, duration-nb=%.6f",
        //        ind_f, ind_b, duration_f, path[ind_f + 1].forward_duration, duration_b, path[ind_b - 1].backward_duration);

        if (duration_f > duration_b)
        {
            if (path[ind_f + 1].forward_duration > 0)
            {
                ind_f ++;
                duration_f = path[ind_f].forward_duration;
            }
            else
            {
                break;
            }
        }
        else
        {
            if (path[ind_b - 1].backward_duration > 0)
            {
                ind_b --;
                duration_b = path[ind_b].backward_duration;
            }
            else
            {
                break;
            }
        }
    }

    if (ind_f == ind_b)
    {
        if (path[ind_f].forward_duration < path[ind_b].backward_duration)
        {
            ind_f --;
        }
        else
        {
            ind_b ++;
        }
    }

    spd_up = ind_f;
    spd_down = ind_b;

    ind_f ++;
    ind_b --;

    while (path[ind_f].forward_duration > 0 && ind_f < tail)
    {
        path[ind_f ++].forward_duration = -1;
    }

    while (path[ind_b].backward_duration > 0 && ind_b > 0)
    {
        path[ind_b --].backward_duration = -1;
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to build startup and stop trajectory, err=0x%llx", err);
    }

    FST_INFO("spd-up-index=%d, spd-down-index=%d", spd_up, spd_down);


    for (size_t i = head; i < tail; i++)
    {
        FST_LOG("stamp=%d, duration_f=%.6f, duration_b=%.6f, cmd_duration=%.6f",
                 path[i].path_point.stamp, path[i].forward_duration,
                 path[i].backward_duration, path[i].command_duration);
    }


    return err;
}


ErrorCode ArmGroup::pauseMove(size_t pick_segment)
{
    if (motion_state_ == AUTO_RUNNING)
    {
        motion_state_ = AUTO_RUNNING_TO_PAUSE;
    }
    else if (motion_state_ == AUTO_RESUME)
    {
        motion_state_ = AUTO_RESUME_TO_PAUSE;
    }
    else if (motion_state_ == AUTO_READY)
    {
        motion_state_ = AUTO_PAUSE;
    }

    return SUCCESS;
    /*
    ErrorCode err;

    // setting joint limits
    Omega velocity_max[AXIS_IN_ALGORITHM];
    Alpha acc_max[AXIS_IN_ALGORITHM]; //acc limits
    double vel = 0, acc = 0;
    if(t_path_[t_head_].path_point.type == MOTION_JOINT)
    {
        vel =  t_path_[t_head_].path_point.source->getCommandVelocity();
        acc =  t_path_[t_head_].path_point.source->getCommandAcc();
    }
    else
    {
        vel = 1;
        acc = 1;
    }
    velocity_max[0] = vel*g_omega_limit[0];
    velocity_max[1] = vel*g_omega_limit[1];
    velocity_max[2] = vel*g_omega_limit[2];
    velocity_max[3] = vel*g_omega_limit[3];
    velocity_max[4] = vel*g_omega_limit[4];
    velocity_max[5] = vel*g_omega_limit[5];
    acc_max[0] = acc*g_alpha_limit[0];
    acc_max[1] = acc*g_alpha_limit[1];
    acc_max[2] = acc*g_alpha_limit[2];
    acc_max[3] = acc*g_alpha_limit[3];
    acc_max[4] = acc*g_alpha_limit[4];
    acc_max[5] = acc*g_alpha_limit[5];

    size_t pick_current = pick_segment;
    Angle* start_joint_ptr = NULL;
    Angle* end_joint_ptr = NULL;
    Omega* start_omega_ptr = NULL;
    MotionTime duration_min = 0, last_duration_min = 0;
    bool isLastStep = false;
    while(!isLastStep)
    {
        start_joint_ptr = (Angle*)&t_path_[pick_current].point.joint;
        end_joint_ptr = (Angle*)&t_path_[pick_current + 1].point.joint;
        start_omega_ptr = (Angle*)&t_path_[pick_current].point.omega;
        computeDurationMin(start_joint_ptr, end_joint_ptr, start_omega_ptr, acc_max, velocity_max, duration_min);
        if(duration_min == DBL_MAX)
        {
            isLastStep = true;
            computeLastDurationMin(start_joint_ptr, end_joint_ptr, start_omega_ptr, duration_min);
            if(duration_min > 2 * last_duration_min)
            {
                break;
            }
        }
        computeTrajectory(true, true, pick_current + 1, start_joint_ptr, end_joint_ptr, start_omega_ptr, duration_min, acc_max, velocity_max, t_path_);
        t_path_[pick_current + 1].time_from_start = t_path_[pick_current].time_from_start + duration_min;
        last_duration_min = duration_min;
        ++pick_current;
    }
    
    if(pick_current + 1 >= t_real_start_)   // pause happened in command path
    {
        t_real_start_ = pick_current + 1;
        t_real_end_ = t_tail_;
        t_tail_ = pick_current + 1;
    }
    else    // pause happened in intermediate MOVJ path
    {
        t_tail_ = t_real_start_;
    }

    auto_running_ = true;
    memcpy(&g_start_joint, t_path_[t_tail_ - 1].point.joint, sizeof(Joint));

    FST_INFO("pauseMove Trajectory--------------------------");
    for(size_t i = 0; i < t_tail_; ++i)
    {
        FST_INFO("point[%d]: joint = %f, omega = %f, alpha = %f, time = %f", 
            i, t_path_[i].point.joint[4], t_path_[i].point.omega[4], t_path_[i].point.alpha[4], t_path_[i].time_from_start);
        tos1  << t_path_[i].point.joint[0] << " "
            << t_path_[i].point.joint[1] << " "
            << t_path_[i].point.joint[2] << " "
            << t_path_[i].point.joint[3] << " "
            << t_path_[i].point.joint[4] << " "
            << t_path_[i].point.joint[5] << " "
            << t_path_[i].point.omega[0] << " "
            << t_path_[i].point.omega[1] << " "
            << t_path_[i].point.omega[2] << " "
            << t_path_[i].point.omega[3] << " "
            << t_path_[i].point.omega[4] << " "
            << t_path_[i].point.omega[5] << " "
            << t_path_[i].point.alpha[0] << " "
            << t_path_[i].point.alpha[1] << " "
            << t_path_[i].point.alpha[2] << " "
            << t_path_[i].point.alpha[3] << " "
            << t_path_[i].point.alpha[4] << " "
            << t_path_[i].point.alpha[5] << " "
            << t_path_[i].time_from_start << std::endl;       
    }
    tos1.close();
    */
    
    return SUCCESS;
}

ErrorCode ArmGroup::pauseMove(void)
{
    return pauseMove(pick_segment_);
}


ErrorCode ArmGroup::continueMove(void)
{
    ErrorCode err;

    /*
    // setting joint limits
    Omega velocity_max[AXIS_IN_ALGORITHM];
    Alpha acc_max[AXIS_IN_ALGORITHM]; //acc limits
    double vel = 0, acc = 0;
    if(t_path_[t_real_start_].path_point.type == MOTION_JOINT)
    {
        vel =  t_path_[t_real_start_].path_point.source->getCommandVelocity();
        acc =  t_path_[t_real_start_].path_point.source->getCommandAcc();
    }
    else
    {
        vel = 1;
        acc = 1;
    }
    velocity_max[0] = vel*g_omega_limit[0];
    velocity_max[1] = vel*g_omega_limit[1];
    velocity_max[2] = vel*g_omega_limit[2];
    velocity_max[3] = vel*g_omega_limit[3];
    velocity_max[4] = vel*g_omega_limit[4];
    velocity_max[5] = vel*g_omega_limit[5];
    acc_max[0] = acc*g_alpha_limit[0];
    acc_max[1] = acc*g_alpha_limit[1];
    acc_max[2] = acc*g_alpha_limit[2];
    acc_max[3] = acc*g_alpha_limit[3];
    acc_max[4] = acc*g_alpha_limit[4];
    acc_max[5] = acc*g_alpha_limit[5];

    // set continue start point index
    size_t t_continue = t_tail_ - 1;
    // set start point
    setStartState(g_start_joint);

    // set target point
    MotionTarget target;
    target.type = MOTION_JOINT;
    target.cnt = 0;
    target.vel = vel;
    target.acc = acc;
    target.joint_target.j1 = t_path_[t_continue].point.joint[0];
    target.joint_target.j2 = t_path_[t_continue].point.joint[1];
    target.joint_target.j3 = t_path_[t_continue].point.joint[2];
    target.joint_target.j4 = t_path_[t_continue].point.joint[3];
    target.joint_target.j5 = t_path_[t_continue].point.joint[4];
    target.joint_target.j6 = t_path_[t_continue].point.joint[5];

    FST_INFO("continue start: %f, %f, %f, %f, %f, %f", g_start_joint.j1, g_start_joint.j2, g_start_joint.j3,
                g_start_joint.j4, g_start_joint.j5, g_start_joint.j6);
    FST_INFO("continue end: %f, %f, %f, %f, %f, %f", target.joint_target.j1, target.joint_target.j2, target.joint_target.j3,
                target.joint_target.j4, target.joint_target.j5, target.joint_target.j6);

    // plan MOVJ pathpoint
    MotionCommand *cmd = getMotionCommandPtr();
    cmd->setTarget(target, t_path_[t_continue].path_point.id);
    cmd->setStartJoint(g_start_joint);
    cmd->planPath();

    // judge if the remaining path points after the pause point need to be moved forward or backward
    // t_tail_ always points to the recover path point
    int size = 0, offset = 0;
    size = t_real_end_ - t_tail_;
    offset = cmd->getPathLength() - t_tail_ + 1;
    moveFIFO(t_tail_, size, offset);
    t_real_start_ = t_real_start_ + offset;
    t_real_end_ = t_real_start_ + size;
    FST_INFO("move remaining paused path from %d with size %d to offset %d", t_tail_, size, offset);
    
    // generate MOVJ trajectory from t_path_[0] to t_path_[max_stamp_ - 1]
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
        t_path_[t_tail_].path_point = *it;
        ++t_tail_;
    }

    if (t_head_ != t_tail_)
    {
        for(size_t i = 0; i < t_tail_; ++i)
        {
            t_path_[i].time_from_start = 0;
            memset(t_path_[i].point.joint, 0, NUM_OF_JOINT * sizeof(double));
            memset(t_path_[i].point.omega, 0, NUM_OF_JOINT * sizeof(double));
            memset(t_path_[i].point.alpha, 0, NUM_OF_JOINT * sizeof(double));
        }
        // TODO
        //err = planJointTraj();
    }
    
    // insert a static point which confirms omega=0, alpha=0
    memcpy(t_path_[t_tail_].point.joint, t_path_[t_tail_ - 1].point.joint, NUM_OF_JOINT * sizeof(double));
    memset(t_path_[t_tail_].point.omega, 0, NUM_OF_JOINT * sizeof(double));
    memset(t_path_[t_tail_].point.alpha, 0, NUM_OF_JOINT * sizeof(double));
    t_path_[t_tail_].time_from_start = t_path_[t_tail_ - 1].time_from_start + g_cycle_time;
    ++t_tail_;

    // regenerate the trajectory of the remaining path of paused motion
    memcpy(&g_start_joint, t_path_[t_tail_ - 1].point.joint, sizeof(Joint));
    prev_traj_point_.point = t_path_[t_tail_ - 1].point;
    prev_traj_point_.time_from_start = t_path_[t_tail_ - 1].time_from_start;
    t_head_ = t_tail_;
    t_tail_ = t_tail_ + size;
    //auto_running_ = true;
    
    if (t_head_ != t_tail_)
    {
        if(t_path_[t_real_start_].path_point.type == MOTION_JOINT)
        {
            for(size_t i = t_head_; i < t_tail_; ++i)
            {
                t_path_[i].time_from_start = 0;
                memset(t_path_[i].point.joint, 0, NUM_OF_JOINT * sizeof(double));
                memset(t_path_[i].point.omega, 0, NUM_OF_JOINT * sizeof(double));
                memset(t_path_[i].point.alpha, 0, NUM_OF_JOINT * sizeof(double));
            }
            // TODO
            //err = planJointTraj();
        }
        else
        {
            // TODO
            //err = planTraj();
        }
    }

    //auto_running_ = true;
    memcpy(&g_start_joint, t_path_[t_tail_ - 1].point.joint, sizeof(Joint));

    FST_INFO("continueMove Trajectory--------------------------");
    for(size_t i = 0; i < t_tail_; ++i)
    {
        FST_INFO("point[%d]: joint = %f, omega = %f, alpha = %f, time = %f", 
            i, t_path_[i].point.joint[4], t_path_[i].point.omega[4], t_path_[i].point.alpha[4], t_path_[i].time_from_start);
        tos2  << t_path_[i].point.joint[0] << " "
            << t_path_[i].point.joint[1] << " "
            << t_path_[i].point.joint[2] << " "
            << t_path_[i].point.joint[3] << " "
            << t_path_[i].point.joint[4] << " "
            << t_path_[i].point.joint[5] << " "
            << t_path_[i].point.omega[0] << " "
            << t_path_[i].point.omega[1] << " "
            << t_path_[i].point.omega[2] << " "
            << t_path_[i].point.omega[3] << " "
            << t_path_[i].point.omega[4] << " "
            << t_path_[i].point.omega[5] << " "
            << t_path_[i].point.alpha[0] << " "
            << t_path_[i].point.alpha[1] << " "
            << t_path_[i].point.alpha[2] << " "
            << t_path_[i].point.alpha[3] << " "
            << t_path_[i].point.alpha[4] << " "
            << t_path_[i].point.alpha[5] << " "
            << t_path_[i].time_from_start << std::endl;       
    }
    tos2.close();
    */

    return SUCCESS;
}

ErrorCode ArmGroup::emcyStop(size_t pick_segment)
{
    /*
    if(pick_segment + 1 >= t_real_start_)   // emergency happened in command path
    {
        t_real_start_ = pick_segment + 1;
        t_real_end_ = t_tail_;
        t_tail_ = pick_segment + 1;
    }
    else
    {
        t_tail_ = t_real_start_;
    }
     */

    return SUCCESS;
}

ErrorCode ArmGroup::emcyStop(void)
{
    return emcyStop(pick_segment_);
}

void ArmGroup::moveFIFO(size_t start_index, int size, int offset)
{
    /*
    size_t end_index = start_index + size;
    if(offset < 0)
    {
        for(size_t i = start_index; i < end_index; ++i)
        {
            t_path_[i - offset] = t_path_[i];
        }
    }
    else if(offset > 0)
    {
        for(size_t i = end_index - 1; i >= start_index; --i)
        {
            t_path_[i + offset] = t_path_[i];
        }
    }
    else
    {}
     */
}

/*
ErrorCode ArmGroup::pickFromAuto(size_t num, vector<JointOutput> &points)
{
    MotionTime   tm;
    JointOutput  jout;
    JointState  *js;

    FST_WARN("pickFromAuto: pick points from auto");

    ControlPoint *path = pick_path_ptr_->path;
    size_t head = pick_path_ptr_->head;
    size_t tail = pick_path_ptr_->tail;

    for (size_t i = 0; i < num; i++)
    {
        while (pick_segment_ < tail && pick_time_ > path[pick_segment_].time_from_start)
        {
            pick_segment_++;

            if (pick_segment_ == tail && pick_path_ptr_->next->tail > 0)
            {
                pick_segment_  = 0;
                pick_path_ptr_ = pick_path_ptr_->next;
                path = pick_path_ptr_->path;
                head = pick_path_ptr_->head;
                tail = pick_path_ptr_->tail;
            }
        }
        //FST_INFO("pick time=%f, total time=%f", pick_time_, t_path_[t_tail_ - 1].time_from_start);
        //FST_INFO("pick segment=%d, segment time from start=%f", pick_segment_, t_path_[pick_segment_].time_from_start);
        if (pick_segment_ < tail && pick_time_ < path[pick_segment_].time_from_start)
        {
            //FST_WARN("pick t=%f seg=%d", pick_time_, pick_segment_);
            //FST_WARN("time_from_start=%f, duration=%f", t_path_[pick_segment_].time_from_start, t_path_[pick_segment_].duration);

            if (path[pick_segment_].smooth == false)
            {
                js = &path[pick_segment_].point;
                tm = path[pick_segment_].time_from_start - pick_time_;

                if (pick_time_ < g_cycle_time + MINIMUM_E9)
                {
                    jout.level = POINT_START;
                }
                else
                {
                    jout.level = POINT_MIDDLE;
                }
                jout.id = path[pick_segment_].path_point.id;
                jout.joint.j1 = js->joint[0] - js->omega[0] * tm + 0.5 * js->alpha[0] * tm * tm;
                jout.joint.j2 = js->joint[1] - js->omega[1] * tm + 0.5 * js->alpha[1] * tm * tm;
                jout.joint.j3 = js->joint[2] - js->omega[2] * tm + 0.5 * js->alpha[2] * tm * tm;
                jout.joint.j4 = js->joint[3] - js->omega[3] * tm + 0.5 * js->alpha[3] * tm * tm;
                jout.joint.j5 = js->joint[4] - js->omega[4] * tm + 0.5 * js->alpha[4] * tm * tm;
                jout.joint.j6 = js->joint[5] - js->omega[5] * tm + 0.5 * js->alpha[5] * tm * tm;
            }
            else
            {
                double (&coeff)[AXIS_IN_ALGORITHM][6] = path[pick_segment_].coeff;
                tm = pick_time_ - (path[pick_segment_].time_from_start - path[pick_segment_].duration);
                double t[6];
                t[0] = 1;
                t[1] = tm;
                t[2] = t[1] * tm;
                t[3] = t[2] * tm;
                t[4] = t[3] * tm;
                t[5] = t[4] * tm;
                jout.id = path[pick_segment_].path_point.id;
                jout.level = POINT_MIDDLE;
                jout.joint.j1 = coeff[0][5] * t[5] + coeff[0][4] * t[4] + coeff[0][3] * t[3] + 
                                coeff[0][2] * t[2] + coeff[0][1] * t[1] + coeff[0][0] * t[0];
                jout.joint.j2 = coeff[1][5] * t[5] + coeff[1][4] * t[4] + coeff[1][3] * t[3] + 
                                coeff[1][2] * t[2] + coeff[1][1] * t[1] + coeff[1][0] * t[0];
                jout.joint.j3 = coeff[2][5] * t[5] + coeff[2][4] * t[4] + coeff[2][3] * t[3] + 
                                coeff[2][2] * t[2] + coeff[2][1] * t[1] + coeff[2][0] * t[0];
                jout.joint.j4 = coeff[3][5] * t[5] + coeff[3][4] * t[4] + coeff[3][3] * t[3] + 
                                coeff[3][2] * t[2] + coeff[3][1] * t[1] + coeff[3][0] * t[0];
                jout.joint.j5 = coeff[4][5] * t[5] + coeff[4][4] * t[4] + coeff[4][3] * t[3] + 
                                coeff[4][2] * t[2] + coeff[4][1] * t[1] + coeff[4][0] * t[0];
                jout.joint.j6 = coeff[5][5] * t[5] + coeff[5][4] * t[4] + coeff[5][3] * t[3] + 
                                coeff[5][2] * t[2] + coeff[5][1] * t[1] + coeff[5][0] * t[0];
            }

            //FST_INFO("J5: j=%.6f, w=%.6f, a=%.6f, tm=%.4f, time_from_start=%.4f, duration=%.4f, res=%.6f",
            //         js->joint[4], js->omega[4], js->alpha[4],
            //         tm, t_path_[pick_segment_].time_from_start, t_path_[pick_segment_].duration,
            //         jout.joint.j5);
            FST_INFO("%d - %.3f - %f,%f,%f,%f,%f,%f", jout.level, pick_time_,
                     jout.joint.j1, jout.joint.j2, jout.joint.j3,
                     jout.joint.j4, jout.joint.j5, jout.joint.j6);
            points.push_back(jout);
            pick_time_ += g_cycle_time;
        }
        else
        {
            jout.id = path[tail - 1].path_point.id;
            jout.level = POINT_ENDING;
            jout.joint.j1 = path[tail - 1].point.joint[0];
            jout.joint.j2 = path[tail - 1].point.joint[1];
            jout.joint.j3 = path[tail - 1].point.joint[2];
            jout.joint.j4 = path[tail - 1].point.joint[3];
            jout.joint.j5 = path[tail - 1].point.joint[4];
            jout.joint.j6 = path[tail - 1].point.joint[5];

            FST_INFO("%d - %.3f - %f,%f,%f,%f,%f,%f", jout.level, pick_time_,
                     jout.joint.j1, jout.joint.j2, jout.joint.j3,
                     jout.joint.j4, jout.joint.j5, jout.joint.j6);
            points.push_back(jout);

            FST_INFO("Command ID = %d finished", path[tail - 1].path_point.source->getMotionID());
            auto_running_ = false;
            break;
        }
    }

    return SUCCESS;
}
*/


#define LOG_SEGMENT
ErrorCode ArmGroup::createTrajectory(void)
{
    ErrorCode err = SUCCESS;
    MotionTime exp_duration;

    double tolerance = 0.05;
    double vel_ratio = 1;
    double vel_max[6];
    TrajSegment seg;
    ControlPoint *cp;

    static int state = 0;

    //FST_LOG("createTrajectory: traj-duration=%.6f, traj-fifo-size=%d", traj_fifo_.duration(), traj_fifo_.size());
    
    if (motion_state_ == AUTO_RUNNING)
    {
        while (traj_fifo_.duration() < 0.1 && !traj_fifo_.full() || traj_fifo_.size() < 3)
        {
            if (pick_segment_ < pick_path_ptr_->tail)
            {
                if (pick_segment_ <= pick_path_ptr_->smooth_out_stamp)
                {
                    cp = &pick_path_ptr_->path[pick_segment_];
                    exp_duration = cp->command_duration / g_global_vel_ratio;
#ifdef LOG_SEGMENT
                    FST_LOG("---> forward-duration=%.6f, backward-duration=%.6f, expect-duration=%.6f",
                            cp->forward_duration, cp->backward_duration, exp_duration);
#endif

                    cp = &pick_path_ptr_->path[pick_segment_];
                    exp_duration = cp->command_duration / g_global_vel_ratio;

                    if (cp->forward_duration > 0 && cp->backward_duration > 0)
                    {
                        FST_ERROR("forward and backward crash!!! forward=%.6f, backward=%.6f",
                                  cp->forward_duration, cp->backward_duration);
                        return MOTION_INTERNAL_FAULT;
                    }

                    if (speed_state_ == SPEED_UP)
                    {
                        if (cp->forward_duration > 0)
                        {
                            double global_vel_ratio = g_global_vel_ratio;

                            do
                            {
                                seg.duration = cp->forward_duration;
                                seg.time_from_start = (cp - 1)->time_from_start + seg.duration;
                                cp->time_from_start = seg.time_from_start;

                                for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                                {
                                    seg.coeff[i][0] = (cp - 1)->forward_point.joint[i];
                                    seg.coeff[i][1] = (cp - 1)->forward_point.omega[i];
                                    seg.coeff[i][2] = cp->forward_point.alpha[i] / 2;
                                    seg.coeff[i][3] = 0;
                                    seg.coeff[i][4] = 0;
                                    seg.coeff[i][5] = 0;
                                    seg.inertia[i] = cp->inertia[i];
                                }

#ifdef LOG_SEGMENT
                                logTrajSegment("stamp", cp->path_point.stamp, seg);
#endif
                                traj_fifo_.push(seg);
                                pick_segment_ ++;
                                cp ++;
                            } while (pick_segment_ <= pick_path_ptr_->smooth_out_stamp && cp->forward_duration + MINIMUM_E6 > exp_duration);

                            speed_state_ = SPEED_KEEP;
                        }
                        else if (cp->backward_duration > 0)
                        {
                            createSpeedUpTraj(*(cp - 1), *cp, exp_duration);

                            if (cp->forward_duration - MINIMUM_E6 > cp->backward_duration)
                            {
                                // Vel-forward < Vel-backward
                                seg.duration = cp->forward_duration;
                                seg.time_from_start = (cp - 1)->time_from_start + seg.duration;
                                cp->time_from_start = seg.time_from_start;

                                for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                                {
                                    seg.coeff[i][0] = (cp - 1)->forward_point.joint[i];
                                    seg.coeff[i][1] = (cp - 1)->forward_point.omega[i];
                                    seg.coeff[i][2] = cp->forward_point.alpha[i] / 2;
                                    seg.coeff[i][3] = 0;
                                    seg.coeff[i][4] = 0;
                                    seg.coeff[i][5] = 0;
                                    seg.inertia[i] = cp->inertia[i];
                                }

#ifdef LOG_SEGMENT
                                logTrajSegment("stamp", cp->path_point.stamp, seg);
#endif
                                traj_fifo_.push(seg);
                                pick_segment_ ++;
                            }
                            else
                            {
                                speed_state_ = SPEED_DOWN;
#ifdef LOG_SEGMENT
                                FST_WARN("link forward-traj with backward-traj, rebuild curr segment");
#endif
                                MotionTime tm = cp->backward_duration;
                                smoothJoint2Joint((cp - 1)->forward_point, cp->backward_point, tm, seg);
                                seg.time_from_start = (cp - 1)->time_from_start + tm;
                                cp->time_from_start = seg.time_from_start;

                                for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                                {
                                    seg.inertia[i] = cp->inertia[i];
                                }

                                traj_fifo_.push(seg);
                                pick_segment_ ++;
                                cp->forward_point = cp->backward_point;
#ifdef LOG_SEGMENT
                                logTrajSegment("smooth", 0, seg);
#endif

                                while (pick_segment_ <= pick_path_ptr_->smooth_out_stamp)
                                {
                                    cp = &pick_path_ptr_->path[pick_segment_];
                                    cp->time_from_start = (cp - 1)->time_from_start + cp->backward_duration;
                                    seg.time_from_start = cp->time_from_start;
                                    seg.duration = cp->backward_duration;

                                    for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                                    {
                                        seg.coeff[i][0] = (cp - 1)->backward_point.joint[i];
                                        seg.coeff[i][1] = (cp - 1)->backward_point.omega[i];
                                        seg.coeff[i][2] = cp->backward_point.alpha[i] / 2;
                                        seg.coeff[i][3] = 0;
                                        seg.coeff[i][4] = 0;
                                        seg.coeff[i][5] = 0;
                                    }

#ifdef LOG_SEGMENT
                                    logTrajSegment("stamp", cp->path_point.stamp, seg);
#endif
                                    traj_fifo_.push(seg);
                                    pick_segment_ ++;
                                }
                            }
                        }
                        else
                        {
                            speed_state_ = SPEED_KEEP;
                            uniformTrajectory(*(cp - 1), *cp, exp_duration, seg);

                            seg.time_from_start = (cp - 1)->time_from_start + seg.duration;
                            cp->time_from_start = seg.time_from_start;

                            for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                            {
                                cp->forward_point.omega[i] = seg.coeff[i][1];
                                cp->forward_point.alpha[i] = 0;
                                seg.inertia[i] = 0;
                            }

#ifdef LOG_SEGMENT
                            logTrajSegment("uniform-stamp", cp->path_point.stamp, seg);
#endif
                            traj_fifo_.push(seg);
                            pick_segment_ ++;

                        }
                    }
                    else if (speed_state_ == SPEED_KEEP)
                    {
                        if (cp->forward_duration > 0)
                        {
                            ControlPoint *tmp = cp;

                            while (tmp->forward_duration > 0)
                            {
                                tmp->forward_duration = -1;
                                tmp ++;
                            }
                        }

                        uniformTrajectory(*(cp - 1), *cp, exp_duration, seg);

                        if ((cp->backward_duration < 0 || seg.duration - MINIMUM_E6 > cp->backward_duration)
                                && pick_segment_ < pick_path_ptr_->smooth_out_stamp)
                        {
                            seg.time_from_start = (cp - 1)->time_from_start + seg.duration;
                            cp->time_from_start = seg.time_from_start;

                            for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                            {
                                cp->forward_point.omega[i] = seg.coeff[i][1];
                                cp->forward_point.alpha[i] = 0;
                                seg.inertia[i] = 0;
                            }

#ifdef LOG_SEGMENT
                            logTrajSegment("uniform-stamp", cp->path_point.stamp, seg);
#endif
                            traj_fifo_.push(seg);
                            pick_segment_ ++;
                        }
                        else
                        {
                            speed_state_ = SPEED_DOWN;
#ifdef LOG_SEGMENT
                            FST_WARN("link forward-traj with backward-traj, rebuild curr segment");
#endif
                            MotionTime tm = cp->backward_duration;
                            smoothJoint2Joint((cp - 1)->forward_point, cp->backward_point, tm, seg);
                            seg.time_from_start = (cp - 1)->time_from_start + tm;
                            cp->time_from_start = seg.time_from_start;

                            for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                            {
                                if (cp->backward_duration > 0)
                                {
                                    seg.inertia[i] = cp->inertia[i];
                                }
                                else
                                {
                                    seg.inertia[i] = 0;
                                }
                            }

                            traj_fifo_.push(seg);
                            pick_segment_ ++;
                            cp->forward_point = cp->backward_point;
#ifdef LOG_SEGMENT
                            logTrajSegment("smooth", 0, seg);
#endif

                            while (pick_segment_ <= pick_path_ptr_->smooth_out_stamp)
                            {
                                cp = &pick_path_ptr_->path[pick_segment_];
                                cp->time_from_start = (cp - 1)->time_from_start + cp->backward_duration;
                                seg.time_from_start = cp->time_from_start;
                                seg.duration = cp->backward_duration;

                                for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                                {
                                    seg.coeff[i][0] = (cp - 1)->backward_point.joint[i];
                                    seg.coeff[i][1] = (cp - 1)->backward_point.omega[i];
                                    seg.coeff[i][2] = cp->backward_point.alpha[i] / 2;
                                    seg.coeff[i][3] = 0;
                                    seg.coeff[i][4] = 0;
                                    seg.coeff[i][5] = 0;

                                    seg.inertia[i] = cp->inertia[i];
                                }

#ifdef LOG_SEGMENT
                                logTrajSegment("stamp", cp->path_point.stamp, seg);
#endif
                                traj_fifo_.push(seg);
                                pick_segment_ ++;
                            }
                        }
                    }
                    else if (speed_state_ == SPEED_DOWN)
                    {
                        if (cp->backward_duration > 0)
                        {
                            while (pick_segment_ <= pick_path_ptr_->smooth_out_stamp)
                            {
                                cp = &pick_path_ptr_->path[pick_segment_];
                                cp->time_from_start = (cp - 1)->time_from_start + cp->backward_duration;
                                seg.time_from_start = cp->time_from_start;
                                seg.duration = cp->backward_duration;

                                for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                                {
                                    seg.coeff[i][0] = (cp - 1)->backward_point.joint[i];
                                    seg.coeff[i][1] = (cp - 1)->backward_point.omega[i];
                                    seg.coeff[i][2] = cp->backward_point.alpha[i] / 2;
                                    seg.coeff[i][3] = 0;
                                    seg.coeff[i][4] = 0;
                                    seg.coeff[i][5] = 0;

                                    seg.inertia[i] = cp->inertia[i];
                                }

#ifdef LOG_SEGMENT
                                logTrajSegment("stamp", cp->path_point.stamp, seg);
#endif
                                traj_fifo_.push(seg);
                                pick_segment_ ++;
                            }
                        }
                        else
                        {
                            FST_WARN("speed-state = SPEED_DOWN, it may be a trouble");
                        }
                    }
                    else
                    {
                        FST_ERROR("INVALID speed-state(%d)", speed_state_);
                        return MOTION_INTERNAL_FAULT;
                    }

                    if (pick_segment_ - 1 == pick_path_ptr_->smooth_out_stamp)
                    {
                        double time_tmp = pick_path_ptr_->path[pick_segment_ - 1].time_from_start - 0.075;
                        pick_path_ptr_->deadline = time_tmp < 0 ? MINIMUM_E6 : time_tmp;
                        FST_INFO("time-from-start of smooth out point: %.6f, deadline=%.6f",
                                 cp->time_from_start, pick_path_ptr_->deadline);
                    }
                }
                else
                {
                    if (pick_path_ptr_->next->valid)
                    {
#ifdef LOG_SEGMENT
                        FST_INFO("smooth to next traj");
#endif
                        ControlPoint *smooth_in  = &pick_path_ptr_->next->path[pick_path_ptr_->next->smooth_in_stamp];
                        ControlPoint *smooth_out = &pick_path_ptr_->path[pick_path_ptr_->smooth_out_stamp];
                        ControlPoint smooth_traj;
                        MotionTime exp_duration = smooth_in->command_duration / g_global_vel_ratio;

                        if (smooth_in->forward_duration < exp_duration)
                        {
                            // velocity of smooth_in > expect velocity
#ifdef LOG_SEGMENT
                            FST_INFO("rebuild omegas and alphas of smooth-in point");
#endif
                            double *jnt_s = smooth_in->forward_point.joint;
                            double *jnt_e = (smooth_in + 1)->forward_point.joint;
                            double *omega = smooth_in->forward_point.omega;
                            double *alpha = smooth_in->forward_point.alpha;

                            omega[0] = (jnt_e[0] - jnt_s[0]) / exp_duration;
                            omega[1] = (jnt_e[1] - jnt_s[1]) / exp_duration;
                            omega[2] = (jnt_e[2] - jnt_s[2]) / exp_duration;
                            omega[3] = (jnt_e[3] - jnt_s[3]) / exp_duration;
                            omega[4] = (jnt_e[4] - jnt_s[4]) / exp_duration;
                            omega[5] = (jnt_e[5] - jnt_s[5]) / exp_duration;

                            alpha[0] = 0;
                            alpha[1] = 0;
                            alpha[2] = 0;
                            alpha[3] = 0;
                            alpha[4] = 0;
                            alpha[5] = 0;
                        }

                        /*
                        double *omega = smooth_in->forward_point.omega;
                        double *alpha = smooth_out->forward_point.alpha;

                        for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                        {
                            if (omega[i] > g_omega_limit[i] * 0.9 && alpha[i] > 0 ||
                                omega[i] < -g_omega_limit[i] * 0.9 && alpha[i] < 0)
                            {
                                alpha[i] = 0;
                            }
                        }*/


                        TrajSegment smooth;
                        MotionTime tm = smooth_out->forward_duration * 
                                       (smooth_out->path_point.source->getPathLength() - smooth_out->path_point.stamp) +
                                        smooth_in->forward_duration * smooth_in->path_point.stamp;
                        FST_INFO("smooth time: %.6f, adjust to: %.6f", tm, tm * 1.2);
                        tm *= 1.2;

                        smoothJoint2Joint(smooth_out->forward_point, smooth_in->forward_point, tm, smooth);
                        smooth.time_from_start = smooth_out->time_from_start + tm;
                        smooth_in->time_from_start = smooth.time_from_start;

                        for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                        {
                            smooth.inertia[i] = smooth_out->inertia[i];
                        }

                        traj_fifo_.push(smooth);

#ifdef LOG_SEGMENT
                        FST_INFO("smooth-out: stamp=%d, time_from_start=%.4f duration=%.4f",
                                 smooth_out->path_point.stamp, smooth_out->time_from_start, smooth_out->forward_duration);
                        FST_INFO("    joint: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth_out->forward_point.joint[0], smooth_out->forward_point.joint[1], smooth_out->forward_point.joint[2],
                                 smooth_out->forward_point.joint[3], smooth_out->forward_point.joint[4], smooth_out->forward_point.joint[5]);
                        FST_INFO("    omega: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth_out->forward_point.omega[0], smooth_out->forward_point.omega[1], smooth_out->forward_point.omega[2],
                                 smooth_out->forward_point.omega[3], smooth_out->forward_point.omega[4], smooth_out->forward_point.omega[5]);
                        FST_INFO("    alpha: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth_out->forward_point.alpha[0], smooth_out->forward_point.alpha[1], smooth_out->forward_point.alpha[2],
                                 smooth_out->forward_point.alpha[3], smooth_out->forward_point.alpha[4], smooth_out->forward_point.alpha[5]);

                        FST_INFO("smooth-in: stamp=%d, time_from_start=%.4f duration=%.4f",
                                 smooth_in->path_point.stamp, smooth_in->time_from_start, smooth_in->forward_duration);
                        FST_INFO("    joint: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth_in->forward_point.joint[0], smooth_in->forward_point.joint[1], smooth_in->forward_point.joint[2],
                                 smooth_in->forward_point.joint[3], smooth_in->forward_point.joint[4], smooth_in->forward_point.joint[5]);
                        FST_INFO("    omega: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth_in->forward_point.omega[0], smooth_in->forward_point.omega[1], smooth_in->forward_point.omega[2],
                                 smooth_in->forward_point.omega[3], smooth_in->forward_point.omega[4], smooth_in->forward_point.omega[5]);
                        FST_INFO("    alpha: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth_in->forward_point.alpha[0], smooth_in->forward_point.alpha[1], smooth_in->forward_point.alpha[2],
                                 smooth_in->forward_point.alpha[3], smooth_in->forward_point.alpha[4], smooth_in->forward_point.alpha[5]);

                        FST_INFO("smooth - time_from_start:%.4f duration:%.4f",
                                 smooth.time_from_start, smooth.duration);
                        FST_INFO("    J1: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth.coeff[0][5], smooth.coeff[0][4], smooth.coeff[0][3],
                                 smooth.coeff[0][2], smooth.coeff[0][1], smooth.coeff[0][0]);
                        FST_INFO("    J2: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth.coeff[1][5], smooth.coeff[1][4], smooth.coeff[1][3],
                                 smooth.coeff[1][2], smooth.coeff[1][1], smooth.coeff[1][0]);
                        FST_INFO("    J3: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth.coeff[2][5], smooth.coeff[2][4], smooth.coeff[2][3],
                                 smooth.coeff[2][2], smooth.coeff[2][1], smooth.coeff[2][0]);
                        FST_INFO("    J4: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth.coeff[3][5], smooth.coeff[3][4], smooth.coeff[3][3],
                                 smooth.coeff[3][2], smooth.coeff[3][1], smooth.coeff[3][0]);
                        FST_INFO("    J5: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth.coeff[4][5], smooth.coeff[4][4], smooth.coeff[4][3],
                                 smooth.coeff[4][2], smooth.coeff[4][1], smooth.coeff[4][0]);
                        FST_INFO("    J6: %.6f %.6f %.6f %.6f %.6f %.6f",
                                 smooth.coeff[5][5], smooth.coeff[5][4], smooth.coeff[5][3],
                                 smooth.coeff[5][2], smooth.coeff[5][1], smooth.coeff[5][0]);
#endif

                        pick_path_ptr_->valid = false;
                        pick_path_ptr_ = pick_path_ptr_->next;
                        pick_segment_  = pick_path_ptr_->smooth_in_stamp + 1;
                        speed_state_ = SPEED_UP;
                    }
                    else
                    {
                        if (pick_time_ + MINIMUM_E6 > pick_path_ptr_->path[pick_path_ptr_->smooth_out_stamp].time_from_start)
                        {
                            FST_WARN("Next command is not ready, use fine instead of smooth");
                            FST_INFO("  smooth-out-stamp changed from %d to %d",
                                     pick_path_ptr_->smooth_out_stamp, pick_path_ptr_->tail - 1);
                            pick_path_ptr_->smooth_out_stamp = pick_path_ptr_->tail - 1;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
            else
            {
                pick_path_ptr_->valid = false;
                if (pick_path_ptr_->next->valid)
                {
                    speed_state_ = SPEED_UP;
                    pick_path_ptr_ = pick_path_ptr_->next;
                    pick_segment_ = 1;
                    pick_path_ptr_->path[0].time_from_start = traj_fifo_.back().time_from_start;
                    FST_INFO("pick_path_ptr_ switch to next, seg=%d, time_from_start=%.4f", 
                             pick_segment_, pick_path_ptr_->path[0].time_from_start);
                }
                else
                {
                    break;
                }
            }
        }
    }
    else if (motion_state_ == AUTO_RUNNING_TO_PAUSE)
    {
        // TODO
    }

    return SUCCESS;
}

ErrorCode ArmGroup::pickFromAuto(size_t num, vector<JointOutput> &points)
{
    MotionTime   tm;
    JointOutput  jout;
    JointState  *js;

    static Joint last_joint;

    //FST_WARN("pickFromAuto: pick points from auto");

    for (size_t i = 0; i < num; i++)
    {
        while (!traj_fifo_.empty() && pick_time_ > traj_fifo_.front().time_from_start)
        {
            traj_fifo_.dropFront();
        }

        createTrajectory();

        if (!traj_fifo_.empty())
        {
            if (pick_time_ < traj_fifo_.front().time_from_start)
            {
                jout.id = traj_fifo_.front().id;
                jout.level = POINT_MIDDLE;

                if (pick_time_ < g_cycle_time + MINIMUM_E9)
                {
                    jout.level = POINT_START;
                }

                const double (&coeff)[AXIS_IN_ALGORITHM][6] = traj_fifo_.front().coeff;
                tm = pick_time_ - (traj_fifo_.front().time_from_start - traj_fifo_.front().duration);

                double t[6];
                t[0] = 1;
                t[1] = tm;
                t[2] = t[1] * tm;
                t[3] = t[2] * tm;
                t[4] = t[3] * tm;
                t[5] = t[4] * tm;

                jout.joint.j1 = coeff[0][5] * t[5] + coeff[0][4] * t[4] + coeff[0][3] * t[3] + 
                                coeff[0][2] * t[2] + coeff[0][1] * t[1] + coeff[0][0] * t[0];
                jout.joint.j2 = coeff[1][5] * t[5] + coeff[1][4] * t[4] + coeff[1][3] * t[3] +
                                coeff[1][2] * t[2] + coeff[1][1] * t[1] + coeff[1][0] * t[0];
                jout.joint.j3 = coeff[2][5] * t[5] + coeff[2][4] * t[4] + coeff[2][3] * t[3] +
                                coeff[2][2] * t[2] + coeff[2][1] * t[1] + coeff[2][0] * t[0];
                jout.joint.j4 = coeff[3][5] * t[5] + coeff[3][4] * t[4] + coeff[3][3] * t[3] + 
                                coeff[3][2] * t[2] + coeff[3][1] * t[1] + coeff[3][0] * t[0];
                jout.joint.j5 = coeff[4][5] * t[5] + coeff[4][4] * t[4] + coeff[4][3] * t[3] +
                                coeff[4][2] * t[2] + coeff[4][1] * t[1] + coeff[4][0] * t[0];
                jout.joint.j6 = coeff[5][5] * t[5] + coeff[5][4] * t[4] + coeff[5][3] * t[3] +
                                coeff[5][2] * t[2] + coeff[5][1] * t[1] + coeff[5][0] * t[0];

                jout.omega.j1 = coeff[0][5] * t[4] * 5 + coeff[0][4] * t[3] * 4 + coeff[0][3] * t[2] * 3 +
                                coeff[0][2] * t[1] * 2 + coeff[0][1];
                jout.omega.j2 = coeff[1][5] * t[4] * 5 + coeff[1][4] * t[3] * 4 + coeff[1][3] * t[2] * 3 +
                                coeff[1][2] * t[1] * 2 + coeff[1][1];
                jout.omega.j3 = coeff[2][5] * t[4] * 5 + coeff[2][4] * t[3] * 4 + coeff[2][3] * t[2] * 3 +
                                coeff[2][2] * t[1] * 2 + coeff[2][1];
                jout.omega.j4 = coeff[3][5] * t[4] * 5 + coeff[3][4] * t[3] * 4 + coeff[3][3] * t[2] * 3 +
                                coeff[3][2] * t[1] * 2 + coeff[3][1];
                jout.omega.j5 = coeff[4][5] * t[4] * 5 + coeff[4][4] * t[3] * 4 + coeff[4][3] * t[2] * 3 +
                                coeff[4][2] * t[1] * 2 + coeff[4][1];
                jout.omega.j6 = coeff[5][5] * t[4] * 5 + coeff[5][4] * t[3] * 4 + coeff[5][3] * t[2] * 3 +
                                coeff[5][2] * t[1] * 2 + coeff[5][1];

                memset(&jout.alpha, 0, sizeof(JointAlpha));

                jout.inertia.j1 = traj_fifo_.front().inertia[0];
                jout.inertia.j2 = traj_fifo_.front().inertia[1];
                jout.inertia.j3 = traj_fifo_.front().inertia[2];
                jout.inertia.j4 = traj_fifo_.front().inertia[3];
                jout.inertia.j5 = traj_fifo_.front().inertia[4];
                jout.inertia.j6 = traj_fifo_.front().inertia[5];
                /*
                PoseEuler pe;
                forwardKinematics(jout.joint, pe);
                tos << pick_time_ << ","
                    << jout.joint.j1 << "," << jout.joint.j2 << "," << jout.joint.j3 << ","
                    << jout.joint.j4 << "," << jout.joint.j5 << "," << jout.joint.j6 << ","
                    << jout.omega.j1 << "," << jout.omega.j2 << "," << jout.omega.j3 << ","
                    << jout.omega.j4 << "," << jout.omega.j5 << "," << jout.omega.j6 << ","
                    << pe.position.x << "," << pe.position.y << "," << pe.position.z << ","
                    << pe.orientation.a << "," << pe.orientation.b << "," << pe.orientation.c << endl;
                */
                //FST_LOG("%d - %.3f - %f,%f,%f,%f,%f,%f", jout.level, pick_time_,
                //         jout.joint.j1, jout.joint.j2, jout.joint.j3,
                //         jout.joint.j4, jout.joint.j5, jout.joint.j6);

                if (pick_time_ < g_cycle_time + MINIMUM_E9)
                {
                    last_joint = jout.joint;
                }
                else
                {
                    for (size_t i = 0; i < AXIS_IN_ALGORITHM; i++)
                    {
                        if (fabs(*((double*)(&jout.joint.j1) + i) - *((double*)(&last_joint.j1) + i)) / g_cycle_time > g_omega_limit[i] * 1.3)
                        {
                            FST_ERROR("J%d omega beyond limit: prev=%.6f, next=%.6f, omega=%.4f", i + 1,
                            *((double*)(&last_joint.j1) + i), *((double*)(&jout.joint.j1) + i),
                            (*((double*)(&jout.joint.j1) + i) - *((double*)(&last_joint.j1) + i)) / g_cycle_time);

                            FST_ERROR("traj-seg: time_from_start=%.6f, duration=%.6f", traj_fifo_.front().time_from_start, traj_fifo_.front().duration);
                            FST_ERROR("          J%d - %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", i + 1,
                                     coeff[i][5], coeff[i][4], coeff[i][3], coeff[i][2], coeff[i][1], coeff[i][0]);

                            clearArmGroup();

                            //return JOINT_OMEGA_OVER_LIMIT;
                            return MOTION_INTERNAL_FAULT;
                        }
                    }
                    last_joint = jout.joint;
                }
                points.push_back(jout);
                pick_time_ += g_cycle_time;
            }
        }
        else
        {
            const double (&coeff)[AXIS_IN_ALGORITHM][6] = traj_fifo_.back().coeff;
            tm = traj_fifo_.back().duration;

            double t[6];
            t[0] = 1;
            t[1] = tm;
            t[2] = t[1] * tm;
            t[3] = t[2] * tm;
            t[4] = t[3] * tm;
            t[5] = t[4] * tm;
            
            jout.id = traj_fifo_.back().id;
            jout.level = POINT_ENDING;
            jout.joint.j1 = coeff[0][5] * t[5] + coeff[0][4] * t[4] + coeff[0][3] * t[3] + 
                            coeff[0][2] * t[2] + coeff[0][1] * t[1] + coeff[0][0] * t[0];
            jout.joint.j2 = coeff[1][5] * t[5] + coeff[1][4] * t[4] + coeff[1][3] * t[3] +
                            coeff[1][2] * t[2] + coeff[1][1] * t[1] + coeff[1][0] * t[0];
            jout.joint.j3 = coeff[2][5] * t[5] + coeff[2][4] * t[4] + coeff[2][3] * t[3] +
                            coeff[2][2] * t[2] + coeff[2][1] * t[1] + coeff[2][0] * t[0];
            jout.joint.j4 = coeff[3][5] * t[5] + coeff[3][4] * t[4] + coeff[3][3] * t[3] + 
                            coeff[3][2] * t[2] + coeff[3][1] * t[1] + coeff[3][0] * t[0];
            jout.joint.j5 = coeff[4][5] * t[5] + coeff[4][4] * t[4] + coeff[4][3] * t[3] +
                            coeff[4][2] * t[2] + coeff[4][1] * t[1] + coeff[4][0] * t[0];
            jout.joint.j6 = coeff[5][5] * t[5] + coeff[5][4] * t[4] + coeff[5][3] * t[3] +
                            coeff[5][2] * t[2] + coeff[5][1] * t[1] + coeff[5][0] * t[0];

            jout.omega.j1 = 0;
            jout.omega.j2 = 0;
            jout.omega.j3 = 0;
            jout.omega.j4 = 0;
            jout.omega.j5 = 0;
            jout.omega.j6 = 0;

            /*
            tos << pick_time_ << ","
                << jout.joint.j1 << "," << jout.joint.j2 << "," << jout.joint.j3 << ","
                << jout.joint.j4 << "," << jout.joint.j5 << "," << jout.joint.j6 << ","
                << jout.omega.j1 << "," << jout.omega.j2 << "," << jout.omega.j3 << ","
                << jout.omega.j4 << "," << jout.omega.j5 << "," << jout.omega.j6 << endl;
            */

            FST_INFO("%d - %.3f - %f,%f,%f,%f,%f,%f", jout.level, pick_time_,
                     jout.joint.j1, jout.joint.j2, jout.joint.j3,
                     jout.joint.j4, jout.joint.j5, jout.joint.j6);
            points.push_back(jout);

            FST_INFO("Command ID = %d finished", traj_fifo_.back().id);
            //auto_running_ = false;
            if (motion_state_ == AUTO_RUNNING)
            {
                motion_state_ = IDLE;
            }
            break;
        }
    }

    return SUCCESS;
}

ErrorCode ArmGroup::pickFromManual(size_t num, vector<JointOutput> &points)
{
    ErrorCode err = SUCCESS;
    FST_LOG("pickFromManual: pick points from manual");

    MANUAL_LOCK;
    if (manual_traj_.frame == JOINT)
    {
        err = pickManualJoint(num, points);
    }
    else
    {
        err = pickManualCartesian(num, points);
    }
    MANUAL_UNLOCK;

    for (vector<JointOutput>::iterator it = points.begin(); it != points.end(); ++it)
    {
        memset(&it->omega, 0, sizeof(JointOmega));
        memset(&it->alpha, 0, sizeof(JointAlpha));
        memset(&it->inertia, 0, sizeof(JointInertia));
    }

    /*
    PoseEuler pe;
    for (size_t i = 0; i < points.size(); i++)
    {
        forwardKinematics(points[i].joint, pe);
        tos << points[i].joint.j1 << "," << points[i].joint.j2 << ","
            << points[i].joint.j3 << "," << points[i].joint.j4 << ","
            << points[i].joint.j5 << "," << points[i].joint.j6 << "," << pe.position.x << ","
            << pe.position.y << "," << pe.position.z << "," << pe.orientation.a << ","
            << pe.orientation.b << "," << pe.orientation.c << endl;
    }
    */

    return err;
}

ErrorCode ArmGroup::pickManualJoint(size_t num, vector<JointOutput> &points)
{
    JointOutput jout;

    FST_LOG("pickManualJoint: pick point");

    if (manual_running_)
    {
        size_t index;
        double *angle;
        double *start;
        double *target;
        double tm, omega;

        for (index = 0; index < num; index++)
        {
            angle  = &jout.joint.j1;
            start  = &manual_traj_.joint_start.j1;
            target = &manual_traj_.joint_ending.j1;

            jout.id = -1;
            jout.level = POINT_MIDDLE;
            if (manual_pick_time_ < g_cycle_time + MINIMUM_E9)
            {
                jout.level = POINT_START;
            }
            
            for (size_t jnt = 0; jnt < 6; jnt++)
            {
                if (manual_pick_time_ < manual_traj_.coeff[jnt].start_time)
                {
                    *angle = *start;
                }
                else if (manual_pick_time_ < manual_traj_.coeff[jnt].stable_time)
                {
                    tm = manual_pick_time_ - manual_traj_.coeff[jnt].start_time;
                    *angle = *start + manual_traj_.coeff[jnt].start_alpha * tm * tm / 2;
                }
                else if (manual_pick_time_ < manual_traj_.coeff[jnt].brake_time)
                {
                    tm = manual_traj_.coeff[jnt].stable_time - manual_traj_.coeff[jnt].start_time;
                    omega = manual_traj_.coeff[jnt].start_alpha * tm;
                    *angle = *start + omega * tm / 2;
                    tm = manual_pick_time_ - manual_traj_.coeff[jnt].stable_time;
                    *angle = *angle + omega * tm;
                }
                else if (manual_pick_time_ < manual_traj_.coeff[jnt].stop_time)
                {
                    tm = manual_traj_.coeff[jnt].stable_time - manual_traj_.coeff[jnt].start_time;
                    omega = manual_traj_.coeff[jnt].start_alpha * tm;
                    *angle = *start + omega * tm / 2;
                    tm = manual_traj_.coeff[jnt].brake_time - manual_traj_.coeff[jnt].stable_time;
                    *angle = *angle + omega * tm;
                    tm = manual_pick_time_ - manual_traj_.coeff[jnt].brake_time;
                    *angle = *angle + omega * tm + manual_traj_.coeff[jnt].brake_alpha * tm * tm / 2;
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

            if (manual_pick_time_ >= manual_traj_.duration)
            {
                points.back().level = POINT_ENDING;
                FST_INFO("%d - %.3f - %f %f %f %f %f %f", points.back().level, manual_pick_time_,
                          points.back().joint.j1, points.back().joint.j2,  points.back().joint.j3,
                          points.back().joint.j4, points.back().joint.j5,  points.back().joint.j6);

                manual_traj_.direction[0] = STANDBY;
                manual_traj_.direction[1] = STANDBY;
                manual_traj_.direction[2] = STANDBY;
                manual_traj_.direction[3] = STANDBY;
                manual_traj_.direction[4] = STANDBY;
                manual_traj_.direction[5] = STANDBY;
                manual_traj_.duration = 0;
                memset(manual_traj_.coeff, 0, 6 * sizeof(ManualCoef));
                g_start_joint = manual_traj_.joint_ending;
                memset(&manual_traj_.joint_ending, 0, 6 * sizeof(double));
                memset(&manual_traj_.joint_start,  0, 6 * sizeof(double));
                manual_running_ = false;
                manual_pick_time_ = 0;
                break;
            }

            FST_LOG("%d - %.3f - %f %f %f %f %f %f", points.back().level, manual_pick_time_,
                      points.back().joint.j1, points.back().joint.j2, points.back().joint.j3,
                      points.back().joint.j4, points.back().joint.j5, points.back().joint.j6);

            manual_pick_time_ += g_cycle_time;
        }

        return SUCCESS;
    }
    else
    {
        FST_WARN("manual is not running");
        return SUCCESS;
    }
}


ErrorCode ArmGroup::pickManualCartesian(size_t num, vector<JointOutput> &points)
{
    ErrorCode   err = SUCCESS;
    Joint       joint;
    PoseEuler   pose;
    JointOutput jout;

    FST_LOG("pickManualCartesian: pick point");

    if (manual_running_)
    {
        size_t index;

        for (index = 0; index < num; index++)
        {
            jout.id = -1;
            jout.level = POINT_MIDDLE;
            
            if (manual_pick_time_ < g_cycle_time + MINIMUM_E9)
            {
                jout.level = POINT_START;
            }

            double vel[6], brake_time = 0;
            for (size_t i = 0; i < 6; i++)
            {
                if (manual_pick_time_ < manual_traj_.coeff[i].start_time)
                {
                    vel[i] = 0;
                }
                else if (manual_pick_time_ < manual_traj_.coeff[i].stable_time)
                {
                    vel[i] = manual_traj_.coeff[i].start_alpha * 
                            (manual_pick_time_ - manual_traj_.coeff[i].start_time);
                }
                else if (manual_pick_time_ < manual_traj_.coeff[i].brake_time)
                {
                    vel[i] = manual_traj_.coeff[i].start_alpha * 
                            (manual_traj_.coeff[i].stable_time - manual_traj_.coeff[i].start_time);
                }
                else if (manual_pick_time_ < manual_traj_.coeff[i].stop_time)
                {
                    vel[i] = manual_traj_.coeff[i].start_alpha * 
                            (manual_traj_.coeff[i].stable_time - manual_traj_.coeff[i].start_time) +
                             manual_traj_.coeff[i].brake_alpha *
                            (manual_pick_time_ - manual_traj_.coeff[i].brake_time);
                }
                else
                {
                    vel[i] = 0;
                }

                if (-vel[i] / manual_traj_.coeff[i].brake_alpha > brake_time)
                brake_time = -vel[i] / manual_traj_.coeff[i].brake_alpha;
            }

            interpolateManualCart(manual_pick_time_ + brake_time, pose);

            if (inverseKinematics(pose, g_ik_reference, joint) != SUCCESS ||
                !isJointInConstraint(joint, g_soft_constraint))
            {
                do {
                    brake_time -= 0.01;
                    interpolateManualCart(manual_pick_time_ + brake_time, pose);
                }
                while (inverseKinematics(pose, g_ik_reference, joint) != SUCCESS ||
                       !isJointInConstraint(joint, g_soft_constraint));
                
                manual_.repairCartesianContinuous(manual_pick_time_, brake_time, manual_traj_);
            }
            
            interpolateManualCart(manual_pick_time_, pose);
            err = chainIK(pose, g_ik_reference, jout.joint);

            if (err == SUCCESS)
            {
                points.push_back(jout);
                
                if (manual_pick_time_ >= manual_traj_.duration)
                {
                    points.back().level = POINT_ENDING;
                    FST_INFO("%d - %.3f - %f %f %f %f %f %f", points.back().level, manual_pick_time_,
                             points.back().joint.j1, points.back().joint.j2,  points.back().joint.j3,
                             points.back().joint.j4, points.back().joint.j5,  points.back().joint.j6);

                    manual_traj_.direction[0] = STANDBY;
                    manual_traj_.direction[1] = STANDBY;
                    manual_traj_.direction[2] = STANDBY;
                    manual_traj_.direction[3] = STANDBY;
                    manual_traj_.direction[4] = STANDBY;
                    manual_traj_.direction[5] = STANDBY;
                    manual_traj_.duration = 0;
                    memset(manual_traj_.coeff, 0, 6 * sizeof(ManualCoef));
                    g_start_joint = points.back().joint;
                    manual_running_ = false;
                    manual_pick_time_ = 0;
                    break;
                }

                FST_LOG("%d - %.3f - %f %f %f %f %f %f", points.back().level, manual_pick_time_,
                         points.back().joint.j1, points.back().joint.j2,  points.back().joint.j3,
                         points.back().joint.j4, points.back().joint.j5,  points.back().joint.j6);

                manual_pick_time_ += g_cycle_time;
            }
            else
            {
                FST_ERROR("pickManualCartesian: IK failed");
                return err;
            }
        }

        return SUCCESS;
    }
    else
    {
        FST_WARN("manual is not running");
        return SUCCESS;
    }
}

ErrorCode ArmGroup::interpolateManualCart(MotionTime time, PoseEuler &pose)
{
    double *value = &pose.position.x;
    double *start = &manual_traj_.cart_start.position.x;
    double *target = &manual_traj_.cart_ending.position.x;
    double tim, vel;

    for (size_t i = 0; i < 6; i++)
    {
        if (time < manual_traj_.coeff[i].start_time)
        {
            *value = *start;
        }
        else if (time < manual_traj_.coeff[i].stable_time)
        {
            tim = time - manual_traj_.coeff[i].start_time;
            *value = *start + manual_traj_.coeff[i].start_alpha * tim * tim / 2;
        }
        else if (time < manual_traj_.coeff[i].brake_time)
        {
            tim = manual_traj_.coeff[i].stable_time - manual_traj_.coeff[i].start_time;
            vel = manual_traj_.coeff[i].start_alpha * tim;
            *value = *start + vel * tim / 2;
            tim = time - manual_traj_.coeff[i].stable_time;
            *value = *value + vel * tim;
        }
        else if (time < manual_traj_.coeff[i].stop_time)
        {
            tim = manual_traj_.coeff[i].stable_time - manual_traj_.coeff[i].start_time;
            vel = manual_traj_.coeff[i].start_alpha * tim;
            *value = *start + vel * tim / 2;
            tim = manual_traj_.coeff[i].brake_time - manual_traj_.coeff[i].stable_time;
            *value = *value + vel * tim;
            tim = time - manual_traj_.coeff[i].brake_time;
            *value = *value + vel * tim + manual_traj_.coeff[i].brake_alpha * tim * tim / 2;
        }
        else
        {
            *value = *target;
        }

        ++ value;
        ++ start;
        ++ target;
    }

    //FST_INFO("# - Pose: %f %f %f, %f %f %f",
    //         pose.position.x, pose.position.y, pose.position.z,
    //         pose.orientation.a, pose.orientation.b, pose.orientation.c);
    return SUCCESS;
}



}
