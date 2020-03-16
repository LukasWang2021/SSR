#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <fstream>
#include <sys/syscall.h>
#include <motion_control_ros_basic.h>
#include <motion_control.h>
#include <tool_manager.h>
#include <coordinate_manager.h>
#include "../../coordinate_manager/include/coordinate_manager.h"
#include "../../tool_manager/include/tool_manager.h"



using namespace std;
using namespace basic_alg;
using namespace fst_base;
using namespace basic_alg;
using namespace fst_mc;
using namespace fst_hal;
using namespace fst_ctrl;



static void* runRealTimeTask(void *mc)
{
    ((MotionControl*)mc)->ringRealTimeTask();
    return NULL;
}

static void* runPriorityTask(void *mc)
{
    ((MotionControl*)mc)->ringPriorityTask();
    return NULL;
}

static void* runPlannerTask(void *mc)
{
    ((MotionControl*)mc)->ringPlannerTask();
    return NULL;
}

static void* runCommonTask(void *mc)
{
    ((MotionControl*)mc)->ringCommonTask();
    return NULL;
}

void MotionControl::ringCommonTask(void)
{
    Joint servo_joint;
    int ros_publish_cnt = 0;
    memset(&servo_joint, 0, sizeof(servo_joint));

    FST_WARN("Common task start.");
    FST_WARN("ThreadMotionControlCommon TID is %ld", syscall(SYS_gettid));

    while (common_thread_running_)
    {
        group_ptr_->doCommonLoop();
        
        if (param_ptr_->enable_ros_publish_)
        {
            ros_publish_cnt ++;

            if (ros_publish_cnt >= param_ptr_->cycle_per_publish_)
            {
                ros_publish_cnt = 0;
                servo_joint = group_ptr_->getLatestJoint();
                ros_basic_ptr_->pubJointState(servo_joint);
            }
        }

        usleep(param_ptr_->common_cycle_time_ * 1000);
    }

    FST_WARN("Common task quit.");
}

#define MAIN_PRIORITY 80
#define MAX_SAFE_STACK (1 * 1024 * 1024)    /* The maximum stack size which is
                                                guaranteed safe to access without
                                                faulting */


bool setPriority(int prio)
{
    struct sched_param param;
    param.sched_priority = prio; 
    if (sched_setscheduler(getpid(), SCHED_RR, &param) == -1) //set priority
    { 
        //FST_ERROR("sched_setscheduler() failed"); 
        return false;  
    } 
    return true;
}

void stack_prefault(void) {
        unsigned char dummy[MAX_SAFE_STACK];
        memset(dummy, 0, MAX_SAFE_STACK);
        return;
}


void MotionControl::ringRealTimeTask(void)
{
    float duration_1, duration_2;
    struct timeval start_time, middle_time, end_time;
    /*
    float *start_to_end = new float[1000000];
    size_t cycle = 0;
    size_t zone_0_5 = 0;
    size_t zone_5_10 = 0;
    size_t zone_10_15 = 0;
    size_t zone_15_20 = 0;
    size_t zone_over_20 = 0;
    */
    
    if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) 
    {
        FST_ERROR("mlockall failed");
        return; 
    }

    //Pre-fault our stack
    stack_prefault();

    FST_WARN("Realtime task start.");
    FST_WARN("ThreadMotionControlRealtime TID is %ld", syscall(SYS_gettid));

    while (realtime_thread_running_)
    {
        gettimeofday(&start_time, NULL);
        group_ptr_->doRealtimeLoop();
        gettimeofday(&middle_time, NULL);
        usleep(param_ptr_->realtime_cycle_time_ * 1000);
        gettimeofday(&end_time, NULL);
        duration_1 = (float)(middle_time.tv_sec - start_time.tv_sec) + (float)(middle_time.tv_usec - start_time.tv_usec) / 1000000;
        duration_2 = (float)(end_time.tv_sec - middle_time.tv_sec) + (float)(end_time.tv_usec - middle_time.tv_usec) / 1000000;

        if (duration_1 > 0.02)
        {
            FST_ERROR("RT task first stage duration over limit: %.6f", duration_1);
        }

        if (duration_2 > 0.02)
        {
            FST_ERROR("RT task second stage duration over limit: %.6f", duration_2);
        }
        //start_to_end[cycle] = (float)(end_time.tv_sec - start_time.tv_sec) + (float)(end_time.tv_usec - start_time.tv_usec) / 1000000;
        
        /*
        if (start_to_end[cycle] < 0.005)
        {
            zone_0_5++;
        }
        else if (start_to_end[cycle] < 0.01)
        {
            zone_5_10++;
        }
        else if (start_to_end[cycle] < 0.015)
        {
            zone_10_15++;
        }
        else if (start_to_end[cycle] < 0.02)
        {
            zone_15_20++;
        }
        else
        {
            zone_over_20++;
            FST_ERROR("ringRealTimeTask sleep over 20 ms: %.6f", start_to_end[cycle]);
        }

        cycle = (cycle + 1 == 1000000) ? 0 : cycle + 1;
        */
    }

    FST_WARN("Realtime task quit.");
    /*
    FST_WARN("ringRealTimeTask: 0-5: %d", zone_0_5);
    FST_WARN("ringRealTimeTask: 5-10: %d", zone_5_10);
    FST_WARN("ringRealTimeTask: 10-15: %d", zone_10_15);
    FST_WARN("ringRealTimeTask: 15-20: %d", zone_15_20);
    FST_WARN("ringRealTimeTask: over 20: %d", zone_over_20);

    ofstream  time_out("/root/realtime.csv");

    for (size_t i = 0; i < 1000000; i++)
    {
        time_out << start_to_end[i] << endl;
    }

    time_out.close();
    delete [] start_to_end;
    */
}

void MotionControl::ringPriorityTask(void)
{
    /*
    struct timeval start_time, end_time;
    float *start_to_end = new float[1000000];
    size_t cycle = 0;
    size_t zone_0_5 = 0;
    size_t zone_5_10 = 0;
    size_t zone_10_15 = 0;
    size_t zone_15_20 = 0;
    size_t zone_over_20 = 0;
    */

    FST_WARN("Priority task start.");
    FST_WARN("ThreadMotionControlPriority TID is %ld", syscall(SYS_gettid));

    while (priority_thread_running_)
    {
        group_ptr_->doPriorityLoop();
        //gettimeofday(&start_time, NULL);
        usleep(param_ptr_->priority_cycle_time_ * 1000);
        //gettimeofday(&end_time, NULL);
        //start_to_end[cycle] = (float)(end_time.tv_sec - start_time.tv_sec) + (float)(end_time.tv_usec - start_time.tv_usec) / 1000000;
        
        /*
        if (start_to_end[cycle] < 0.005)
        {
            zone_0_5++;
        }
        else if (start_to_end[cycle] < 0.01)
        {
            zone_5_10++;
        }
        else if (start_to_end[cycle] < 0.015)
        {
            zone_10_15++;
        }
        else if (start_to_end[cycle] < 0.02)
        {
            zone_15_20++;
        }
        else
        {
            zone_over_20++;
        }

        cycle = (cycle + 1 == 1000000) ? 0 : cycle + 1;
        */
    }

    FST_WARN("Priority task quit.");
    /*
    FST_WARN("ringPriorityTask: 0-5: %d", zone_0_5);
    FST_WARN("ringPriorityTask: 5-10: %d", zone_5_10);
    FST_WARN("ringPriorityTask: 10-15: %d", zone_10_15);
    FST_WARN("ringPriorityTask: 15-20: %d", zone_15_20);
    FST_WARN("ringPriorityTask: over 20: %d", zone_over_20);

    ofstream  time_out("/root/priority.csv");

    for (size_t i = 0; i < 1000000; i++)
    {
        time_out << start_to_end[i] << endl;
    }

    time_out.close();
    delete [] start_to_end;
    */
}

void MotionControl::ringPlannerTask(void)
{
    FST_WARN("Planner task start.");
    FST_WARN("ThreadMotionControlPriority TID is %ld", syscall(SYS_gettid));

    while (planner_thread_running_)
    {
        GroupState state = group_ptr_->getGroupState();
        pthread_mutex_lock(&instruction_mutex_);

        if (state != STANDBY && state != STANDBY_TO_AUTO && state != AUTO && state != AUTO_TO_STANDBY)
        {
            if (!instruction_fifo_.empty())
            {
                FST_WARN("Group is not in standby or auto state: 0x%x", state);
                FST_WARN("Instruction list size: %d, clear instruction list", instruction_fifo_.size());
                while (!instruction_fifo_.empty()) instruction_fifo_.pop();
            }
        }
        
        if (!instruction_fifo_.empty() && group_ptr_->nextMovePermitted())
        {
            ErrorCode err = SUCCESS;
            Instruction instruction = instruction_fifo_.front();
            instruction_fifo_.pop();
            instructions_handle_counter_ ++;

            if (instruction.type == MOTION)
            {
                if (instruction.user_op_mode == USER_OP_MODE_SLOWLY_MANUAL)
                {
                    if (instruction.target.type == MOTION_JOINT)
                    {
                        instruction.target.vel = instruction.target.vel > 0.322886 ? 0.322886 : instruction.target.vel;
                    }
                    else if (instruction.target.type == MOTION_LINE || instruction.target.type == MOTION_CIRCLE)
                    {
                        instruction.target.vel = instruction.target.vel > 250 ? 250 : instruction.target.vel;
                    }
                }

                err = autoMove(instruction.target);
            }
            else if (instruction.type == SET_UF)
            {
                err = setUserFrame(instruction.current_uf);
            }
            else if (instruction.type == SET_TF)
            {
                err = setToolFrame(instruction.current_tf);
            }
            else if (instruction.type == SET_OVC)
            {
                err = group_ptr_->setGlobalVelRatio(instruction.current_ovc);
            }
            else if (instruction.type == SET_OAC)
            {
                err = group_ptr_->setGlobalAccRatio(instruction.current_oac);
            }
            else
            {
                FST_ERROR("Invalid instruction type: %d", instruction.type);
                err = INVALID_PARAMETER;
            }

            if (err != SUCCESS)
            {
                if (((err >> 32) & 0xFFFF) > 2)
                {
                    motion_error_flag_ = true;
                }
                
                error_monitor_ptr_->add(err);
            }
        }

        pthread_mutex_unlock(&instruction_mutex_);
        usleep(param_ptr_->planner_cycle_time_ * 1000);
    }

    FST_INFO("Received instruction: %d, handled instruction: %d, instruction list size: %d", instructions_recv_counter_, instructions_handle_counter_, instruction_fifo_.size());
    FST_WARN("Planner task quit.");
}


MotionControl::MotionControl()
{
    device_manager_ptr_ = NULL;
    axis_group_manager_ptr_ = NULL;
    coordinate_manager_ptr_ = NULL;
    tool_manager_ptr_ = NULL;
    error_monitor_ptr_ = NULL;
    log_ptr_ = NULL;
    param_ptr_ = NULL;

    common_thread_running_ = false;
    planner_thread_running_ = false;
    priority_thread_running_ = false;
    realtime_thread_running_ = false;

    ros_basic_ptr_ = NULL;
    motion_error_flag_ = false;
    instructions_recv_counter_ = 0;
    instructions_handle_counter_ = 0;
}

MotionControl::~MotionControl()
{
    realtime_thread_running_ = false;
    priority_thread_running_ = false;
    planner_thread_running_ = false;
    common_thread_running_ = false;

    realtime_thread_.join();
    priority_thread_.join();
    planner_thread_.join();
    common_thread_.join();

    if (group_ptr_ != NULL)     {delete group_ptr_; group_ptr_ = NULL;};
    if (param_ptr_ != NULL)     {delete param_ptr_; param_ptr_ = NULL;};
    if (log_ptr_ != NULL)       {delete log_ptr_; log_ptr_ = NULL;};
    if (ros_basic_ptr_ != NULL)  {delete ros_basic_ptr_; ros_basic_ptr_ = NULL;};
}

ErrorCode MotionControl::init(fst_hal::DeviceManager* device_manager_ptr, AxisGroupManager* axis_group_manager_ptr,
                              fst_ctrl::CoordinateManager* coordinate_manager_ptr, fst_ctrl::ToolManager* tool_manager_ptr,
                              fst_base::ErrorMonitor *error_monitor_ptr)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new MotionControlParam();
    
    if (log_ptr_ == NULL || param_ptr_ == NULL)
    {
        return MC_INTERNAL_FAULT;
    }

    if (!log_ptr_->initLogger("MotionControl"))
    {
        FST_ERROR("Lost communication with log server, init MotionControl abort.");
        return MC_INTERNAL_FAULT;
    }

    if (pthread_mutex_init(&instruction_mutex_, NULL) != 0)
    {
        FST_ERROR("Fail to initialize motion mutex.");
        return MC_INTERNAL_FAULT;
    }

    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load MotionControl component parameters");
        return MC_INTERNAL_FAULT;
    }

    FST_INFO("Log-level of MotionControl setted to: %d", param_ptr_->log_level_);
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    if (param_ptr_->model_name_ == "RTM-P7A")
    {
        group_ptr_ = new ArmGroup(log_ptr_);
    }
    else if (param_ptr_->model_name_ == "SCARA")
    {
        //group_ptr_ = new ScaraGroup(log_ptr_);
        FST_ERROR("Invalid model name: %s", param_ptr_->model_name_.c_str());
        return MC_INTERNAL_FAULT;
    }
    else
    {
        FST_ERROR("Invalid model name: %s", param_ptr_->model_name_.c_str());
        return MC_INTERNAL_FAULT;
    }

    if (group_ptr_ == NULL)
    {
        FST_ERROR("Fail to create control group of %s", param_ptr_->model_name_.c_str());
        return MC_INTERNAL_FAULT;
    }

    //if (device_manager_ptr && axis_group_manager_ptr && coordinate_manager_ptr && tool_manager_ptr && error_monitor_ptr)
    if (coordinate_manager_ptr && tool_manager_ptr && error_monitor_ptr)
    {
        //device_manager_ptr_ = device_manager_ptr;
        //axis_group_manager_ptr_ = axis_group_manager_ptr;
        coordinate_manager_ptr_ = coordinate_manager_ptr;
        tool_manager_ptr_ = tool_manager_ptr;
        error_monitor_ptr_ = error_monitor_ptr;
    }
    else
    {
        FST_ERROR("device-manger: %x, group-manager: %x, coordinate-manager: %x, tool-manager: %x, error-monitor: %x",
                  device_manager_ptr, axis_group_manager_ptr, coordinate_manager_ptr, tool_manager_ptr, error_monitor_ptr);
        return INVALID_PARAMETER;
    }


    user_frame_id_ = 0;
    tool_frame_id_ = 0;

    if (param_ptr_->enable_ros_publish_)
    {
        ros_basic_ptr_ = new RosBasic;
        ros_basic_ptr_->initRosBasic();
    }

    ErrorCode  err = group_ptr_->initGroup(error_monitor_ptr, coordinate_manager_ptr_, tool_manager_ptr_);

    if (err == SUCCESS)
    {
        realtime_thread_running_ = true;

        if (realtime_thread_.run(runRealTimeTask, this, 80))
        {
            FST_INFO("Startup real-time task success.");
        }
        else
        {
            FST_ERROR("Fail to create real-time task.");
            realtime_thread_running_ = false;
            return MC_INTERNAL_FAULT;
        }

        usleep(50 * 1000);
        priority_thread_running_ = true;

        if (priority_thread_.run(runPriorityTask, this, 70))
        {
            FST_INFO("Startup priority task success.");
        }
        else
        {
            FST_ERROR("Fail to create priority task.");
            priority_thread_running_ = false;
            return MC_INTERNAL_FAULT;
        }

        usleep(50 * 1000);
        planner_thread_running_ = true;

        if (planner_thread_.run(runPlannerTask, this, 60))
        {
            FST_INFO("Startup planner task success.");
        }
        else
        {
            FST_ERROR("Fail to create planner task.");
            planner_thread_running_ = false;
            return MC_INTERNAL_FAULT;
        }

        usleep(50 * 1000);
        common_thread_running_ = true;

        if (common_thread_.run(runCommonTask, this, 40))
        {
            FST_INFO("Startup common task success.");
        }
        else
        {
            FST_ERROR("Fail to create common task.");
            common_thread_running_ = false;
            return MC_INTERNAL_FAULT;
        }

        FST_INFO("Initialize motion group success.");
    }
    else
    {
        FST_ERROR("Fail to init motion group");
        return err;
    }

    return SUCCESS;
}




ManualFrame MotionControl::getManualFrame(void)
{
    return group_ptr_->getManualFrame();
}

ErrorCode MotionControl::setManualFrame(ManualFrame frame)
{
    return group_ptr_->setManualFrame(frame);
}


void MotionControl::getAxisManualStep(double (&steps)[NUM_OF_JOINT])
{
    group_ptr_->getManualStepAxis(steps);
}

double MotionControl::getPositionManualStep(void)
{
    return group_ptr_->getManualStepPosition();
}

double MotionControl::getOrientationManualStep(void)
{
    return group_ptr_->getManualStepOrientation();
}

ErrorCode MotionControl::setAxisManualStep(const double (&steps)[NUM_OF_JOINT])
{
    return group_ptr_->setManualStepAxis(steps);
}

ErrorCode MotionControl::setPositionManualStep(double step)
{
    return group_ptr_->setManualStepPosition(step);
}

ErrorCode MotionControl::setOrientationManualStep(double step)
{
    return group_ptr_->setManualStepOrientation(step);
}

ErrorCode MotionControl::doStepManualMove(const GroupDirection &direction)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_FORBIDDEN)
    {
        FST_ERROR("Cannot manual move, calibrator-state = %d, all motion is forbidden.", MOTION_FORBIDDEN);
        return INVALID_SEQUENCE;
    }
    else if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_LIMITED)
    {
        if (group_ptr_->getManualFrame() != JOINT)
        {
            FST_ERROR("Cannot manual cartesian in limited state, calibrator-state = %d, manual cartesian is forbidden.", MOTION_LIMITED);
            return INVALID_SEQUENCE;
        }
    }

    return group_ptr_->manualMoveStep(&direction[0]);
}

ErrorCode MotionControl::doContinuousManualMove(const GroupDirection &direction)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_FORBIDDEN)
    {
        FST_ERROR("Cannot manual move, calibrator-state = %d, all motion is forbidden.", MOTION_FORBIDDEN);
        return INVALID_SEQUENCE;
    }
    else if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_LIMITED)
    {
        if (group_ptr_->getManualFrame() != JOINT)
        {
            FST_ERROR("Cannot manual cartesian in limited state, calibrator-state = %d, manual-frame = %d, manual cartesian is forbidden.", MOTION_LIMITED, group_ptr_->getManualFrame());
            return INVALID_SEQUENCE;
        }
    }

    return group_ptr_->manualMoveContinuous(&direction[0]);
}

ErrorCode MotionControl::doGotoPointManualMove(const Joint &joint)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        FST_ERROR("Cannot manual move to point in current state, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    IntactPoint point;
    point.joint = joint;
    point.tool_frame = group_ptr_->getToolFrame();
    point.user_frame = group_ptr_->getUserFrame();
    PoseEuler tcp_in_base, fcp_in_base;
    group_ptr_->getKinematicsPtr()->doFK(point.joint, fcp_in_base);
    group_ptr_->getTransformationPtr()->convertFcpToTcp(fcp_in_base, point.tool_frame, tcp_in_base);
    group_ptr_->getTransformationPtr()->convertPoseFromBaseToUser(tcp_in_base, point.user_frame, point.pose.pose);
    point.pose.posture = group_ptr_->getKinematicsPtr()->getPostureByJoint(point.joint);
    point.pose.turn = group_ptr_->getKinematicsPtr()->getTurnByJoint(point.joint);
    return group_ptr_->manualMoveToPoint(point);
}

ErrorCode MotionControl::doGotoPointManualMove(const PoseAndPosture &pose, int user_frame_id, int tool_frame_id)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        FST_ERROR("Cannot manual move to point in current state, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    if (user_frame_id != user_frame_id_ && user_frame_id != -1)
    {
        FST_ERROR("manualMove: user frame ID = %d mismatch with activated user frame = %d.", user_frame_id, user_frame_id_);
        return INVALID_PARAMETER;
    }

    if (tool_frame_id != tool_frame_id_ && tool_frame_id != -1)
    {
        FST_ERROR("manualMove: tool frame ID = %d mismatch with activated tool frame = %d.", tool_frame_id, tool_frame_id_);
        return INVALID_PARAMETER;
    }

    IntactPoint point;
    point.pose = pose;
    point.tool_frame = group_ptr_->getToolFrame();
    point.user_frame = group_ptr_->getUserFrame();
    PoseEuler tcp_in_base, fcp_in_base;
    group_ptr_->getTransformationPtr()->convertPoseFromUserToBase(point.pose.pose, point.user_frame, tcp_in_base);
    group_ptr_->getTransformationPtr()->convertTcpToFcp(tcp_in_base, point.tool_frame, fcp_in_base);

    if (!group_ptr_->getKinematicsPtr()->doIK(fcp_in_base, point.pose.posture, point.pose.turn, point.joint))
    {
        const PoseEuler &pe = point.pose.pose;
        const PoseEuler &tf = point.tool_frame;
        const PoseEuler &uf = point.user_frame;
        FST_ERROR("IK of manual target pose failed.");
        FST_ERROR("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pe.point_.x_, pe.point_.y_, pe.point_.z_, pe.euler_.a_, pe.euler_.b_, pe.euler_.c_);
        FST_ERROR("Posture: %d, %d, %d, %d", point.pose.posture.arm, point.pose.posture.elbow, point.pose.posture.wrist, point.pose.posture.flip);
        FST_ERROR("Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
        FST_ERROR("User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    return group_ptr_->manualMoveToPoint(point);
}

ErrorCode MotionControl::manualStop(void)
{
    return group_ptr_->manualStop();
}

ErrorCode MotionControl::setOfflineTrajectory(const std::string &offline_trajectory)
{
    string trajectory_file = "/root/robot_data/trajectory/";
    trajectory_file += offline_trajectory;
    return group_ptr_->setOfflineTrajectory(trajectory_file);
}

ErrorCode MotionControl::prepairOfflineTrajectory(void)
{
    if (group_ptr_->getGroupState() != STANDBY)
    {
        FST_ERROR("Fail to prepair offline trajectory, state = 0x%x", group_ptr_->getGroupState());
        return INVALID_SEQUENCE;
    }

    Joint start_joint = group_ptr_->getStartJointOfOfflineTrajectory();
    Joint current_joint = group_ptr_->getLatestJoint();

    if (start_joint.isEqual(current_joint, MINIMUM_E6))
    {
        return SUCCESS;
    }

    return doGotoPointManualMove(start_joint);
}

ErrorCode MotionControl::moveOfflineTrajectory(void)
{
    return group_ptr_->moveOfflineTrajectory();
}

void MotionControl::clearErrorFlag(void)
{
    motion_error_flag_ = false;
}

ErrorCode MotionControl::autoMove(const Instruction &instruction)
{
    GroupState state = group_ptr_->getGroupState();

    if (state != STANDBY && state != STANDBY_TO_AUTO && state != AUTO)
    {
        FST_ERROR("Cannot autoMove in current state: 0x%x", state);
        return INVALID_SEQUENCE;
    }

    pthread_mutex_lock(&instruction_mutex_);
    instruction_fifo_.push(instruction);
    instructions_recv_counter_ ++;
    FST_INFO("New instruction receieved, add to instruction list, list-size: %d, total-received: %d", instruction_fifo_.size(), instructions_recv_counter_);

    if (instruction_fifo_.size() > 1)
    {
        FST_WARN("Instruction fifo size larger than normal, it is a trouble.");
    }

    pthread_mutex_unlock(&instruction_mutex_);
    return SUCCESS;
}

ErrorCode MotionControl::autoMove(const MotionTarget &target)
{
    FST_INFO("MotionControl::autoMove motion-type: %d, smooth-type: %d", target.type, target.smooth_type);
    FST_INFO("vel: %.6f, acc: %.6f, cnt: %.6f, tool-frame: %d, user-frame: %d", target.vel, target.acc, target.cnt, target.tool_frame_id, target.user_frame_id);
    
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        FST_ERROR("Offset of the group is abnormal, auto move is forbidden, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    if (target.type != MOTION_JOINT && target.type != MOTION_LINE && target.type != MOTION_CIRCLE)
    {
        FST_ERROR("Invalid motion type = %d, autoMove aborted.", target.type);
        return INVALID_PARAMETER;
    }

    if (target.user_frame_id != user_frame_id_ && target.user_frame_id != -1)
    {
        FST_ERROR("autoMove: user frame ID = %d mismatch with activated user frame = %d.", target.user_frame_id, user_frame_id_);
        return MC_FRAME_MISMATCH;
    }

    if (target.tool_frame_id != tool_frame_id_ && target.tool_frame_id != -1)
    {
        FST_ERROR("autoMove: tool frame ID = %d mismatch with activated tool frame = %d.", target.tool_frame_id, tool_frame_id_);
        return MC_TOOL_MISMATCH;
    }

    ErrorCode err = SUCCESS;
    MotionInfo motion_info;
    motion_info.smooth_type = target.smooth_type;
    motion_info.type = target.type;
    motion_info.cnt = target.cnt;
    motion_info.vel = target.vel;
    motion_info.acc = target.acc;
    PoseEuler user_frame = group_ptr_->getUserFrame();
    PoseEuler tool_frame = group_ptr_->getToolFrame();

    if (target.tool_frame_offset.valid)
    {
        FST_INFO("Use tool offset, type: %d, id: %d", target.tool_frame_offset.coord_type, target.tool_frame_offset.offset_frame_id);

        if (target.tool_frame_offset.coord_type == COORDINATE_JOINT)
        {
            FST_ERROR("Cannot use an joint space offset onto tool frame");
            return INVALID_PARAMETER;
        }

        err = offsetToolFrame(target.tool_frame_offset.offset_frame_id, target.tool_frame_offset.offset_pose, tool_frame);

        if (err != SUCCESS)
        {
            FST_ERROR("Fail to offset too frame, code = 0x%llx", err);
            return err;
        }
    }

    TransMatrix trans_frame_offset;

    if (target.user_frame_offset.valid)
    {
        FST_INFO("Use frame offset, type: %d, id: %d", target.user_frame_offset.coord_type, target.user_frame_offset.offset_frame_id);

        if (target.user_frame_offset.coord_type == COORDINATE_JOINT)
        {
            char buffer[LOG_TEXT_SIZE];
            FST_INFO("Offset joint: %s", group_ptr_->printDBLine(&target.user_frame_offset.offset_joint.j1_, buffer, LOG_TEXT_SIZE));
        }
        else
        {
            err = getFrameOffsetMatrix(target.user_frame_offset.offset_frame_id, target.user_frame_offset.offset_pose, user_frame, trans_frame_offset);

            if (err != SUCCESS)
            {
                FST_ERROR("Fail to get offset in current frame, code = 0x%llx", err);
                return err;
            }
        }
    }

    if (target.type == MOTION_CIRCLE)
    {
        FST_INFO("Handle via point of this motion");
        err = handlePoint(target.via, user_frame, tool_frame, motion_info.via);

        if (err != SUCCESS)
        {
            FST_ERROR("Fail to handle via point, code = 0x%llx", err);
            return err;
        }

        if (target.user_frame_offset.valid)
        {
            err = target.user_frame_offset.coord_type == COORDINATE_JOINT ? offsetPoint(target.user_frame_offset.offset_joint, motion_info.via) : offsetPoint(trans_frame_offset, motion_info.via);

            if (err != SUCCESS)
            {
                FST_ERROR("Fail to offset target point, code = 0x%llx", err);
                return err;
            }
        }
    }

    FST_INFO("Handle target point of this motion");
    err = handlePoint(target.target, user_frame, tool_frame, motion_info.target);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to handle target point, code = 0x%llx", err);
        return err;
    }

    if (target.user_frame_offset.valid)
    {
        err = target.user_frame_offset.coord_type == COORDINATE_JOINT ? offsetPoint(target.user_frame_offset.offset_joint, motion_info.target) : offsetPoint(trans_frame_offset, motion_info.target);

        if (err != SUCCESS)
        {
            FST_ERROR("Fail to offset target point, code = 0x%llx", err);
            return err;
        }
    }

    char buffer[LOG_TEXT_SIZE];
    const PoseEuler &target_pose = motion_info.target.pose.pose;
    FST_INFO("Target-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
    FST_INFO("Target-turn: %d, %d, %d, %d, %d, %d", motion_info.target.pose.turn.j1, motion_info.target.pose.turn.j2, motion_info.target.pose.turn.j3, motion_info.target.pose.turn.j4, motion_info.target.pose.turn.j5, motion_info.target.pose.turn.j6);
    FST_INFO("Target-joint: %s", group_ptr_->printDBLine(&motion_info.target.joint.j1_, buffer, LOG_TEXT_SIZE));
    return group_ptr_->autoMove(motion_info);
}

ErrorCode MotionControl::offsetPoint(const Joint &offset_joint, IntactPoint &point)
{
    Kinematics *kinematics_ptr = group_ptr_->getKinematicsPtr();
    Transformation *transformation_ptr = group_ptr_->getTransformationPtr();
    point.joint += offset_joint;
    point.pose.turn = kinematics_ptr->getTurnByJoint(point.joint);
    point.pose.posture = kinematics_ptr->getPostureByJoint(point.joint);
    PoseEuler fcp_in_base, tcp_in_base;
    kinematics_ptr->doFK(point.joint, fcp_in_base);
    transformation_ptr->convertFcpToTcp(fcp_in_base, point.tool_frame, tcp_in_base);
    transformation_ptr->convertPoseFromBaseToUser(tcp_in_base, point.user_frame, point.pose.pose);
    return SUCCESS;
}

ErrorCode MotionControl::offsetPoint(const TransMatrix &trans_offset, IntactPoint &point)
{
    TransMatrix trans_point;
    Kinematics *kinematics_ptr = group_ptr_->getKinematicsPtr();
    Transformation *transformation_ptr = group_ptr_->getTransformationPtr();
    point.pose.pose.convertToTransMatrix(trans_point);
    trans_point.rightMultiply(trans_offset).convertToPoseEuler(point.pose.pose);
    PoseEuler fcp_in_base, tcp_in_base;
    transformation_ptr->convertPoseFromUserToBase(point.pose.pose, point.user_frame, tcp_in_base);
    transformation_ptr->convertTcpToFcp(tcp_in_base, point.tool_frame, fcp_in_base);
    
    if (!kinematics_ptr->doIK(fcp_in_base, point.pose.posture, point.pose.turn, point.joint))
    {
        const Posture &posture = point.pose.posture;
        const PoseEuler &pose = point.pose.pose;
        const PoseEuler &tf = point.tool_frame;
        const PoseEuler &uf = point.user_frame;
        FST_ERROR("IK of point failed.");
        FST_ERROR("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
        FST_ERROR("Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
        FST_ERROR("Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
        FST_ERROR("User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    return SUCCESS;
}

ErrorCode MotionControl::handlePoint(const TargetPoint &origin, const PoseEuler &user_frame, const PoseEuler &tool_frame, IntactPoint &point)
{
    Kinematics *kinematics_ptr = group_ptr_->getKinematicsPtr();
    Transformation *transformation_ptr = group_ptr_->getTransformationPtr();
    point.user_frame = user_frame;
    point.tool_frame = tool_frame;

    if (origin.type == COORDINATE_JOINT)
    {
        PoseEuler fcp_in_base, tcp_in_base;
        point.joint = origin.joint;
        point.pose.turn = kinematics_ptr->getTurnByJoint(point.joint);
        point.pose.posture = kinematics_ptr->getPostureByJoint(point.joint);
        kinematics_ptr->doFK(point.joint, fcp_in_base);
        transformation_ptr->convertFcpToTcp(fcp_in_base, tool_frame, tcp_in_base);
        transformation_ptr->convertPoseFromBaseToUser(tcp_in_base, user_frame, point.pose.pose);
    }
    else
    {
        PoseEuler fcp_in_base, tcp_in_base;
        point.pose = origin.pose;
        transformation_ptr->convertPoseFromUserToBase(point.pose.pose, user_frame, tcp_in_base);
        transformation_ptr->convertTcpToFcp(tcp_in_base, tool_frame, fcp_in_base);
        
        if (!kinematics_ptr->doIK(fcp_in_base, point.pose.posture, point.pose.turn, point.joint))
        {
            const Posture &posture = point.pose.posture;
            const PoseEuler &pose = point.pose.pose;
            const PoseEuler &tf = point.tool_frame;
            const PoseEuler &uf = point.user_frame;
            FST_ERROR("IK of point failed.");
            FST_ERROR("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            FST_ERROR("Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            FST_ERROR("Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            FST_ERROR("User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            return MC_COMPUTE_IK_FAIL;
        }
    }

    return SUCCESS;
}

ErrorCode MotionControl::getFrameOffsetMatrix(int frame_id, const PoseEuler &offset, const PoseEuler &current_frame, TransMatrix &matrix)
{
    FST_INFO("Reference-frame-id: %d, offset-pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", frame_id, offset.point_.x_, offset.point_.y_, offset.point_.z_, offset.euler_.a_, offset.euler_.b_, offset.euler_.c_);
    FST_INFO("Current-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", current_frame.point_.x_, current_frame.point_.y_, current_frame.point_.z_, current_frame.euler_.a_, current_frame.euler_.b_, current_frame.euler_.c_);
    
    if (frame_id < 0)
    {
        // 基于当前user-frame进行偏移
        offset.convertToTransMatrix(matrix);
    }
    else if (frame_id == 0)
    {
        // 基于base-frame进行偏移
        TransMatrix trans_offset, trans_uf, trans_temp;
        current_frame.convertToTransMatrix(trans_uf);
        trans_temp.rotation_matrix_ = trans_uf.rotation_matrix_;
        memset(&trans_temp.trans_vector_, 0, sizeof(trans_temp.trans_vector_));
        trans_temp.inverse();
        offset.convertToTransMatrix(trans_offset);
        trans_temp.rightMultiply(trans_offset, matrix);
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err = coordinate_manager_ptr_->getCoordInfoById(frame_id, uf_info);

        if (err != SUCCESS)
        {
            FST_ERROR("Fail to get reference user frame from given id");
            return err;
        }

        if (!uf_info.is_valid)
        {
            FST_ERROR("Reference user frame indicated by given id is invalid");
            return INVALID_PARAMETER;
        }

        FST_INFO("Reference-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f",   uf_info.data.point_.x_, uf_info.data.point_.y_, uf_info.data.point_.z_,
                                                                           uf_info.data.euler_.a_, uf_info.data.euler_.b_, uf_info.data.euler_.c_);
        
        TransMatrix trans_offset, trans_uf, trans_temp, trans_reference_uf;
        current_frame.convertToTransMatrix(trans_uf);
        uf_info.data.convertToTransMatrix(trans_reference_uf);
        trans_temp.rotation_matrix_ = trans_uf.rotation_matrix_;
        trans_temp.trans_vector_ = trans_reference_uf.trans_vector_;
        trans_temp.inverse();
        offset.convertToTransMatrix(trans_offset);
        trans_temp.rightMultiply(trans_reference_uf).rightMultiply(trans_offset, matrix);
        PoseEuler offset_in_user;
        matrix.convertToPoseEuler(offset_in_user);
        FST_INFO("Offset in current frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", offset_in_user.point_.x_, offset_in_user.point_.y_, offset_in_user.point_.z_,
                                                                                 offset_in_user.euler_.a_, offset_in_user.euler_.b_, offset_in_user.euler_.c_);
    }

    return SUCCESS;
}

ErrorCode MotionControl::offsetToolFrame(int tool_id, const PoseEuler &offset, PoseEuler &tool_frame)
{
    FST_INFO("Reference-tool-id: %d, offset-pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_id, offset.point_.x_, offset.point_.y_, offset.point_.z_, offset.euler_.a_, offset.euler_.b_, offset.euler_.c_);
    FST_INFO("Origin-tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame.point_.x_, tool_frame.point_.y_, tool_frame.point_.z_, tool_frame.euler_.a_, tool_frame.euler_.b_, tool_frame.euler_.c_);
    
    if (tool_id < 0)
    {
        // 基于当前tool-frame进行偏移
        TransMatrix trans_offset, trans_tf;
        tool_frame.convertToTransMatrix(trans_tf);
        offset.convertToTransMatrix(trans_offset);
        trans_tf.rightMultiply(trans_offset).convertToPoseEuler(tool_frame);
    }
    else if (tool_id == 0)
    {
        // 基于flange-frame进行偏移
        TransMatrix trans_offset, trans_tf, trans_temp;
        tool_frame.convertToTransMatrix(trans_tf);
        trans_temp.rotation_matrix_ = trans_tf.rotation_matrix_;
        memset(&trans_temp.trans_vector_, 0, sizeof(trans_temp.trans_vector_));
        trans_temp.inverse();
        offset.convertToTransMatrix(trans_offset);
        trans_tf.rightMultiply(trans_temp).rightMultiply(trans_offset).convertToPoseEuler(tool_frame);
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err = tool_manager_ptr_->getToolInfoById(tool_id, tf_info);

        if (err != SUCCESS)
        {
            FST_ERROR("Fail to get reference tool frame from given id");
            return err;
        }

        if (!tf_info.is_valid)
        {
            FST_ERROR("Reference tool frame indicated by given id is invalid");
            return INVALID_PARAMETER;
        }

        TransMatrix trans_offset, trans_tf, trans_temp, trans_reference_tf;
        tool_frame.convertToTransMatrix(trans_tf);
        tf_info.data.convertToTransMatrix(trans_reference_tf);
        trans_temp.rotation_matrix_ = trans_tf.rotation_matrix_;
        trans_temp.trans_vector_ = trans_reference_tf.trans_vector_;
        trans_temp.inverse();
        offset.convertToTransMatrix(trans_offset);
        trans_tf.rightMultiply(trans_temp).rightMultiply(trans_reference_tf).rightMultiply(trans_offset).convertToPoseEuler(tool_frame);
        FST_INFO("Reference tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf_info.data.point_.x_, tf_info.data.point_.y_, tf_info.data.point_.z_, tf_info.data.euler_.a_, tf_info.data.euler_.b_, tf_info.data.euler_.c_);
    }

    FST_INFO("New-tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame.point_.x_, tool_frame.point_.y_, tool_frame.point_.z_, tool_frame.euler_.a_, tool_frame.euler_.b_, tool_frame.euler_.c_);
    return SUCCESS;
}

ErrorCode MotionControl::abortMove(void)
{
    return group_ptr_->abortMove();
}

ErrorCode MotionControl::pauseMove(void)
{
    return group_ptr_->pauseMove();
}

ErrorCode MotionControl::restartMove(void)
{
    return group_ptr_->restartMove();
}

bool MotionControl::nextMovePermitted(void)
{
    pthread_mutex_lock(&instruction_mutex_);
    
    if (!instruction_fifo_.empty())
    {
        pthread_mutex_unlock(&instruction_mutex_);
        return false;
    }

    if (motion_error_flag_)
    {
        pthread_mutex_unlock(&instruction_mutex_);
        return false;
    }

    if (!group_ptr_->nextMovePermitted())
    {
        pthread_mutex_unlock(&instruction_mutex_);
        return false;
    }

    pthread_mutex_unlock(&instruction_mutex_);
    return true;
}

ErrorCode MotionControl::setOffset(size_t index, double offset)
{
    return group_ptr_->getCalibratorPtr()->setOffset(index, offset);
}

ErrorCode MotionControl::setOffset(const double (&offset)[NUM_OF_JOINT])
{
    return group_ptr_->getCalibratorPtr()->setOffset(offset);
}

void MotionControl::getOffset(double (&offset)[NUM_OF_JOINT])
{
    group_ptr_->getCalibratorPtr()->getOffset(offset);
}

void MotionControl::getOffsetMask(OffsetMask (&mask)[NUM_OF_JOINT])
{
    group_ptr_->getCalibratorPtr()->getOffsetMask(mask);
}

CalibrateState MotionControl::getCalibrateState(void)
{
    return group_ptr_->getCalibratorPtr()->getCalibrateState();
}

/*
ErrorCode MotionControl::saveJoint(void)
{
    //return group_ptr_->getCalibratorPtr()->saveJoint();
    return SUCCESS;
}
*/

ErrorCode MotionControl::saveOffset(void)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }
    
    ErrorCode err = group_ptr_->getCalibratorPtr()->saveOffset();

    if (err == SUCCESS)
    {
        OffsetMask mask[NUM_OF_JOINT];
        group_ptr_->getCalibratorPtr()->getOffsetMask(mask);

        for (size_t i = 0; i < group_ptr_->getNumberOfJoint(); i++)
        {
            if (mask[i] == OFFSET_UNMASK)
            {
                group_ptr_->getSoftConstraintPtr()->resetMask(i);
            }
        }

        return SUCCESS;
    }
    else
    {
        return err;
    }
}

ErrorCode MotionControl::checkOffset(CalibrateState &cali_stat, OffsetState (&offset_stat)[NUM_OF_JOINT])
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->checkOffset(cali_stat, offset_stat);
}

ErrorCode MotionControl::maskOffsetLostError(void)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    ErrorCode err = group_ptr_->getCalibratorPtr()->maskOffsetLostError();

    if (err == SUCCESS)
    {
        OffsetMask mask[NUM_OF_JOINT];
        group_ptr_->getCalibratorPtr()->getOffsetMask(mask);

        for (size_t i = 0; i < group_ptr_->getNumberOfJoint(); i++)
        {
            if (mask[i] == OFFSET_MASKED)
            {
                group_ptr_->getSoftConstraintPtr()->setMask(i);
            }
        }

        return SUCCESS;
    }
    else
    {
        return err;
    }
}

void MotionControl::getOffsetState(OffsetState (&offset_stat)[NUM_OF_JOINT])
{
    return group_ptr_->getCalibratorPtr()->getOffsetState(offset_stat);
}

ErrorCode MotionControl::setOffsetState(size_t index, OffsetState stat)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->setOffsetState(index, stat);
}

ErrorCode MotionControl::calibrateOffset(void)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->calibrateOffset();
}

ErrorCode MotionControl::calibrateOffset(size_t index)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->calibrateOffset(index);
}

ErrorCode MotionControl::calibrateOffset(const size_t *pindex, size_t length)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->calibrateOffset(pindex, length);
}

bool MotionControl::isReferenceAvailable(void)
{
    return group_ptr_->getCalibratorPtr()->isReferenceAvailable();
}

ErrorCode MotionControl::deleteReference(void)
{
    return group_ptr_->getCalibratorPtr()->deleteReference();
}

ErrorCode MotionControl::saveReference(void)
{
    return group_ptr_->getCalibratorPtr()->saveReference();
}

ErrorCode MotionControl::fastCalibrate(void)
{
    return group_ptr_->getCalibratorPtr()->fastCalibrate();
}

ErrorCode MotionControl::fastCalibrate(size_t index)
{
    return group_ptr_->getCalibratorPtr()->fastCalibrate(index);
}

ErrorCode MotionControl::fastCalibrate(const size_t *pindex, size_t length)
{
    return group_ptr_->getCalibratorPtr()->fastCalibrate(pindex, length);
}

ErrorCode MotionControl::getSoftConstraint(JointConstraint &soft_constraint)
{
    return group_ptr_->getSoftConstraint(soft_constraint);
}

ErrorCode MotionControl::getFirmConstraint(JointConstraint &firm_constraint)
{
    return group_ptr_->getFirmConstraint(firm_constraint);
}

ErrorCode MotionControl::getHardConstraint(JointConstraint &hard_constraint)
{
    return group_ptr_->getHardConstraint(hard_constraint);
}

ErrorCode MotionControl::setSoftConstraint(const JointConstraint &soft_constraint)
{
    return group_ptr_->setSoftConstraint(soft_constraint);
}

ErrorCode MotionControl::setFirmConstraint(const JointConstraint &firm_constraint)
{
    return group_ptr_->setFirmConstraint(firm_constraint);
}

ErrorCode MotionControl::setHardConstraint(const JointConstraint &hard_constraint)
{
    return group_ptr_->setHardConstraint(hard_constraint);
}

ErrorCode MotionControl::stopGroup(void)
{
    return group_ptr_->stopGroup();
}

ErrorCode MotionControl::resetGroup(void)
{
    motion_error_flag_ = false;
    return group_ptr_->resetGroup();
}

ErrorCode MotionControl::clearGroup(void)
{
    pthread_mutex_lock(&instruction_mutex_);
    while (!instruction_fifo_.empty()) instruction_fifo_.pop();
    pthread_mutex_unlock(&instruction_mutex_);
    return group_ptr_->clearGroup();
}

ErrorCode MotionControl::convertCartToJoint(const PoseAndPosture &pose, int user_frame_id, int tool_frame_id, Joint &joint)
{
    if (user_frame_id == user_frame_id_ && tool_frame_id == tool_frame_id_)
    {
        return group_ptr_->convertCartToJoint(pose, joint);
    }

    PoseEuler tf, uf;

    if (user_frame_id == 0)
    {
        memset(&uf, 0, sizeof(uf));
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

        if (err_user == SUCCESS && uf_info.is_valid)
        {
            uf = uf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get user frame from given ID = %d", user_frame_id);
            return err_user == SUCCESS ? INVALID_PARAMETER : err_user;
        }
    }

    if (tool_frame_id == 0)
    {
        memset(&tf, 0, sizeof(tf));
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

        if (err_tool == SUCCESS && tf_info.is_valid)
        {
            tf = tf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get tool frame from given ID = %d", tool_frame_id);
            return err_tool == SUCCESS ? INVALID_PARAMETER : err_tool;
        }
    }

    return group_ptr_->convertCartToJoint(pose, uf, tf, joint);
}

ErrorCode MotionControl::convertCartToJoint(const PoseEuler &pose, int user_frame_id, int tool_frame_id, Joint &joint)
{
    if (user_frame_id == user_frame_id_ && tool_frame_id == tool_frame_id_)
    {
        return group_ptr_->convertCartToJoint(pose, joint);
    }

    PoseEuler tf, uf;

    if (user_frame_id == 0)
    {
        memset(&uf, 0, sizeof(uf));
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

        if (err_user == SUCCESS && uf_info.is_valid)
        {
            uf = uf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get user frame from given ID.");
            return err_user;
        }
    }

    if (tool_frame_id == 0)
    {
        memset(&tf, 0, sizeof(tf));
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

        if (err_tool == SUCCESS && tf_info.is_valid)
        {
            tf = tf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get tool frame from given id");
            return err_tool;
        }
    }

    return group_ptr_->convertCartToJoint(pose, uf, tf, joint);
}

ErrorCode MotionControl::convertJointToCart(const Joint &joint, int user_frame_id, int tool_frame_id, PoseEuler &pose)
{
    if (user_frame_id == user_frame_id_ && tool_frame_id == tool_frame_id_)
    {
        return group_ptr_->convertJointToCart(joint, pose);  // transform from base to user
    }

    PoseEuler tf, uf;

    if (user_frame_id == 0)
    {
        memset(&uf, 0, sizeof(uf));
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

        if (err_user == SUCCESS && uf_info.is_valid)
        {
            uf = uf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get user frame from given ID = %d", user_frame_id);
            return err_user == SUCCESS ? INVALID_PARAMETER : err_user;
        }
    }

    if (tool_frame_id == 0)
    {
        memset(&tf, 0, sizeof(tf));
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

        if (err_tool == SUCCESS && tf_info.is_valid)
        {
            tf = tf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get tool frame from given id = %d", tool_frame_id);
            return err_tool == SUCCESS ? INVALID_PARAMETER : err_tool;
        }
    }

    return group_ptr_->convertJointToCart(joint, uf, tf, pose);
}

Posture MotionControl::getPostureFromJoint(const Joint &joint)
{
    return group_ptr_->getKinematicsPtr()->getPostureByJoint(joint);
}

Turn MotionControl::getTurnFromJoint(const Joint &joint)
{
    return group_ptr_->getKinematicsPtr()->getTurnByJoint(joint);
}

string MotionControl::getModelName(void)
{
    return param_ptr_->model_name_;
}

size_t MotionControl::getNumberOfAxis(void)
{
    return group_ptr_->getNumberOfJoint();
}

void MotionControl::getTypeOfAxis(AxisType *types)
{
    group_ptr_->getTypeOfAxis(types);
}

GroupState MotionControl::getGroupState(void)
{
    return group_ptr_->getGroupState();
}

ServoState MotionControl::getServoState(void)
{
    return group_ptr_->getServoState();
}

ErrorCode MotionControl::getServoVersion(std::string &version)
{
    return group_ptr_->getServoVersion(version);
}

PoseEuler MotionControl::getCurrentPose(void)
{
    PoseEuler pose, tcp_in_base, fcp_in_base;
    group_ptr_->getKinematicsPtr()->doFK(group_ptr_->getLatestJoint(), fcp_in_base);
    group_ptr_->getTransformationPtr()->convertFcpToTcp(fcp_in_base, group_ptr_->getToolFrame(), tcp_in_base);
    group_ptr_->getTransformationPtr()->convertPoseFromBaseToUser(tcp_in_base, group_ptr_->getUserFrame(), pose);
    return pose;
}

void MotionControl::getCurrentPose(PoseEuler &pose)
{
    PoseEuler tcp_in_base, fcp_in_base;
    group_ptr_->getKinematicsPtr()->doFK(group_ptr_->getLatestJoint(), fcp_in_base);
    group_ptr_->getTransformationPtr()->convertFcpToTcp(fcp_in_base, group_ptr_->getToolFrame(), tcp_in_base);
    group_ptr_->getTransformationPtr()->convertPoseFromBaseToUser(tcp_in_base, group_ptr_->getUserFrame(), pose);
}

Joint MotionControl::getServoJoint(void)
{
    return group_ptr_->getLatestJoint();
}

ErrorCode MotionControl::setGlobalVelRatio(double ratio)
{
    return group_ptr_->setGlobalVelRatio(ratio);
}

ErrorCode MotionControl::setGlobalAccRatio(double ratio)
{
    return group_ptr_->setGlobalAccRatio(ratio);
}

double MotionControl::getGlobalVelRatio(void)
{
    return group_ptr_->getGlobalVelRatio();
}

double MotionControl::getGlobalAccRatio(void)
{
    return group_ptr_->getGlobalAccRatio();
}

void MotionControl::getToolFrame(int &id)
{
    id = tool_frame_id_;
}

ErrorCode MotionControl::setToolFrame(int id)
{
    FST_INFO("Set tool frame: id = %d, current is %d", id, tool_frame_id_);

    if (id == 0)
    {
        PoseEuler tf = {0, 0, 0, 0, 0, 0};
        group_ptr_->setToolFrame(tf);
        tool_frame_id_ = id;
        FST_INFO("Using tool-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame_id_, tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
        return SUCCESS;
    }
    else
    {
        ToolInfo  tf_info;
        ErrorCode err = tool_manager_ptr_->getToolInfoById(id, tf_info);

        if (err == SUCCESS && tf_info.is_valid)
        {
            group_ptr_->setToolFrame(tf_info.data);
            tool_frame_id_ = id;
            FST_INFO("Using tool-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame_id_, tf_info.data.point_.x_, tf_info.data.point_.y_, tf_info.data.point_.z_, tf_info.data.euler_.a_, tf_info.data.euler_.b_, tf_info.data.euler_.c_);
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail to get tool frame from given id");
            return err;
        }
    }
}

void MotionControl::getUserFrame(int &id)
{
    id = user_frame_id_;
}

ErrorCode MotionControl::setUserFrame(int id)
{
    FST_INFO("Set user frame id = %d, current is %d", id, user_frame_id_);

    if (id == 0)
    {
        PoseEuler uf = {0, 0, 0, 0, 0, 0};
        group_ptr_->setUserFrame(uf);
        user_frame_id_ = id;
        FST_INFO("Using user-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", user_frame_id_, uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
        return SUCCESS;
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err = coordinate_manager_ptr_->getCoordInfoById(id, uf_info);

        if (err == SUCCESS && uf_info.is_valid)
        {
            group_ptr_->setUserFrame(uf_info.data);
            user_frame_id_ = id;
            FST_INFO("Using user-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", user_frame_id_, uf_info.data.point_.x_, uf_info.data.point_.y_, uf_info.data.point_.z_, uf_info.data.euler_.a_, uf_info.data.euler_.b_, uf_info.data.euler_.c_);
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail to get user frame from given id");
            return err;
        }
    }
}

// payload
ErrorCode MotionControl::setPayload(int id)
{
    return group_ptr_->setPayload(id);
}

void MotionControl::getPayload(int &id)
{
    group_ptr_->getPayload(id);
}

ErrorCode MotionControl::addPayload(const PayloadInfo& info)
{
    return group_ptr_->addPayload(info);
}

ErrorCode MotionControl::deletePayload(int id)
{
    return group_ptr_->deletePayload(id);
}

ErrorCode MotionControl::updatePayload(const basic_alg::PayloadInfo& info)
{
    return group_ptr_->updatePayload(info);
}

ErrorCode MotionControl::movePayload(int expect_id, int original_id)
{
    return group_ptr_->movePayload(expect_id, original_id);
}

ErrorCode MotionControl::getPayloadInfoById(int id, PayloadInfo& info)
{
    return group_ptr_->getPayloadInfoById(id, info);
}

std::vector<basic_alg::PayloadSummaryInfo> MotionControl::getAllValidPayloadSummaryInfo(void)
{
    return group_ptr_->getAllValidPayloadSummaryInfo();
}

void MotionControl::getAllValidPayloadSummaryInfo(vector<PayloadSummaryInfo>& info_list)
{
    group_ptr_->getAllValidPayloadSummaryInfo(info_list);
}

