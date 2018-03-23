/**********************************************************************
    Copyright:  Foresight-Robotics
    File:       fst_controller.cpp
    Author:     Feng Yun
    Data:       Aug.1  2016
    Modify:     Aug.30 2016
    Description:ArmGroup--Source code.
**********************************************************************/

#include <string.h>
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

    using_start_joint_ = false;
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
    FST_INFO("Initializing ArmGroup ...");

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
    return string("1.0.0");
}

//------------------------------------------------------------
// Function:    getCycleTime
// Summary: To get cycle time of interpolation algorithm.
// In:      None
// Out:     None
// Return:  cycle time
//------------------------------------------------------------
double ArmGroup::getCycleTime(void)
{
    return g_cycle_time;
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
    return g_global_speed_ratio;
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
        g_global_speed_ratio = ratio;
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
    return getFifoSize();
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
    return getFifoCapacity();
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
    MotionTime tm;
    JointOutput  jout;
    JointState  *js;
    points.clear();


    for (size_t i = 0; i < num; i++)
    {
        if (pick_time_ < t_path_[t_tail_ - 1].time_from_start)
        {
            while (pick_time_ > t_path_[pick_segment_].time_from_start)
            {
                pick_segment_++;
                if (pick_segment_ >= t_tail_)
                {
                    break;
                }
            }
            //FST_WARN("pick t=%f seg=%d", pick_time_, pick_segment_);
            //FST_WARN("time_from_start=%f, duration=%f", t_path_[pick_segment_].time_from_start, t_path_[pick_segment_].duration);

            if (pick_time_ <= t_path_[pick_segment_].time_from_start)
            {
                js = &t_path_[pick_segment_].point;
                tm = t_path_[pick_segment_].time_from_start - pick_time_;

                jout.id = t_path_[pick_segment_].path_point.id;
                jout.joint.j1 = js->joint[0] - js->omega[0] * tm + 0.5 * js->alpha[0] * tm * tm;
                jout.joint.j2 = js->joint[1] - js->omega[1] * tm + 0.5 * js->alpha[1] * tm * tm;
                jout.joint.j3 = js->joint[2] - js->omega[2] * tm + 0.5 * js->alpha[2] * tm * tm;
                jout.joint.j4 = js->joint[3] - js->omega[3] * tm + 0.5 * js->alpha[3] * tm * tm;
                jout.joint.j5 = js->joint[4] - js->omega[4] * tm + 0.5 * js->alpha[4] * tm * tm;
                jout.joint.j6 = js->joint[5] - js->omega[5] * tm + 0.5 * js->alpha[5] * tm * tm;

                points.push_back(jout);
                pick_time_ += g_cycle_time;
            }
            else
            {
                break;
            }
        }
        else
        {
            jout.id = t_path_[t_tail_ - 1].path_point.id;
            jout.joint.j1 = t_path_[t_tail_ - 1].point.joint[0];
            jout.joint.j2 = t_path_[t_tail_ - 1].point.joint[1];
            jout.joint.j3 = t_path_[t_tail_ - 1].point.joint[2];
            jout.joint.j4 = t_path_[t_tail_ - 1].point.joint[3];
            jout.joint.j5 = t_path_[t_tail_ - 1].point.joint[4];
            jout.joint.j6 = t_path_[t_tail_ - 1].point.joint[5];
            points.push_back(jout);
            break;
        }
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

    return SUCCESS;
}

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
        motion_start_joint_ = joint;
        using_start_joint_  = true;
        g_ik_reference = joint;

        memcpy(prev_traj_point_.point.joint, &joint, NUM_OF_JOINT * sizeof(double));
        memset(prev_traj_point_.point.omega, 0, NUM_OF_JOINT * sizeof(double));
        memset(prev_traj_point_.point.alpha, 0, NUM_OF_JOINT * sizeof(double));

        forwardKinematics(joint, prev_traj_point_.path_point.pose);

        FST_INFO("setStartState: joint - %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                 prev_traj_point_.point.joint[0],
                 prev_traj_point_.point.joint[1],
                 prev_traj_point_.point.joint[2],
                 prev_traj_point_.point.joint[3],
                 prev_traj_point_.point.joint[4],
                 prev_traj_point_.point.joint[5],
                 prev_traj_point_.point.joint[6],
                 prev_traj_point_.point.joint[7],
                 prev_traj_point_.point.joint[8]);
        
        FST_INFO("setStartState: omega - %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                 prev_traj_point_.point.omega[0],
                 prev_traj_point_.point.omega[1],
                 prev_traj_point_.point.omega[2],
                 prev_traj_point_.point.omega[3],
                 prev_traj_point_.point.omega[4],
                 prev_traj_point_.point.omega[5],
                 prev_traj_point_.point.omega[6],
                 prev_traj_point_.point.omega[7],
                 prev_traj_point_.point.omega[8]);

        FST_INFO("setStartState: alpha - %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
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
    return SUCCESS;
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
    return true;
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
    }
    else
    {
        FST_ERROR("autoMove: unsupported motion type (=%d)", target.type);
        return INVALID_PARAMETER;
    }

    if (err == SUCCESS)
    {
        FST_INFO("success");
    }

    return err;
}

//------------------------------------------------------------
// Function:    manualMove
// Summary: Plan a manual move trajectory (Joint/Line) with given
//          direction. If FIFO is empty at the moment, fill it.
// In:      button -> manual direction
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::manualMove(const std::vector<ManualDirection> &button)
{
    return SUCCESS;
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
    return SUCCESS;
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
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    setManualFrameMode
// Summary: To set manual frame mode.
// In:      frame   -> manual frame mode, JOINT/WORLD/USER/TOOL
// Out:     None
// Return:  None
//------------------------------------------------------------
void ArmGroup::setManualFrameMode(ManualFrameMode frame)
{}

//------------------------------------------------------------
// Function:    setManualMotionMode
// Summary: To set manual motion mode.
// In:      motion  -> manual motion mode, STEP/CONTINUOUS/POINT
// Out:     None
// Return:  None
//------------------------------------------------------------
void ArmGroup::setManualMotionMode(ManualMotionMode mode)
{}

//------------------------------------------------------------
// Function:    setManualJointStepLength
// Summary: To set step length in manual joint step working mode.
// In:      step    -> step length, unit: rad
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setManualJointStepLength(double step)
{
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    setManualCartesianStepLength
// Summary: To set step length in manual cartesian step working mode.
// In:      step    -> step length, unit: mm
// Out:     None
// Return:  error code
//------------------------------------------------------------
ErrorCode ArmGroup::setManualCartesianStepLength(double step)
{
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
    return 0.001;
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
    return 0.001;
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
    return SUCCESS;
}



















//------------------------------------------------------------
// Function:    isFirstConstraintCoveredBySecond
// Summary: To check wether first constraint is covered by the second.
// In:      first   -> the first joint constraint
//          second  -> the second joint constraint
// Out:     None
// Return:  true    -> first one is covered by the second one
//          false   -> first one is NOT covered by the second one
//------------------------------------------------------------
bool ArmGroup::isFirstConstraintCoveredBySecond(const JointConstraint &first, const JointConstraint &second)
{
    return true;
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

    if (tar.cnt < 0 || tar.cnt > 1)
    {
        FST_ERROR("  invalid smooth parameter (=%.4f), smooth parameter should in [0, 1]", tar.cnt);
        return INVALID_PARAMETER;
    }

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

    if (err == SUCCESS && using_start_joint_)
    {
        err = cmd->setStartJoint(motion_start_joint_);
    }
    if (err == SUCCESS)
    {
        err = cmd->planPathAndTrajectory(t_path_, t_head_, t_tail_);
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

    FST_INFO("Path plan and trajectory plan finished successfully");

/*  vector<PathPoint> points;

    err = cmd->pickCommonPoint(points);

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
*/    

/*
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

        FST_INFO("Computing inverse kinematics of %d points ...");
    }
    else
    {
        FST_INFO("All common points picked out successfully, computing inverse kinematics ...");
    }
    
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

    if (tar.cnt < 0 || tar.cnt > 1)
    {
        FST_ERROR("  invalid smooth parameter (=%.4f), smooth parameter should in [0, 1]", tar.cnt);
        return INVALID_PARAMETER;
    }

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
    if (err == SUCCESS && using_start_joint_)
    {
        err = cmd->setStartJoint(motion_start_joint_);
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
        t_path_[t_tail_].path_point = *it;
        err = convertPath2Trajectory(t_path_[t_tail_]);

        FST_INFO("%d - %.4f, %.4f, %.4f, %.4f, %.4f, %.4f",
                 t_path_[t_tail_].path_point.stamp,
                 t_path_[t_tail_].point.joint[0],
                 t_path_[t_tail_].point.joint[1],
                 t_path_[t_tail_].point.joint[2],
                 t_path_[t_tail_].point.joint[3],
                 t_path_[t_tail_].point.joint[4],
                 t_path_[t_tail_].point.joint[5]);

        if (err != SUCCESS) break;

        t_tail_++;
    }

    if (t_head_ != t_tail_)
    {
        err = planTraj();
    }

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
            FST_INFO("duration=%.6f, exp_duration=%.6f", pp->duration, pp->expect_duration);
            FST_INFO("j1-j6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p->joint[0], p->joint[1], p->joint[2], p->joint[3], p->joint[4], p->joint[5]);
            FST_INFO("w1-w6: %.f, %.6f, %.6f, %.6f, %.6f, %.6f", p->omega[0], p->omega[1], p->omega[2], p->omega[3], p->omega[4], p->omega[5]);
            FST_INFO("a1-a6: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", p->alpha[0], p->alpha[1], p->alpha[2], p->alpha[3], p->alpha[4], p->alpha[5]);

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
}

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

}
