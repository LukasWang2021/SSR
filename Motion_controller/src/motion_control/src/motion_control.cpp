#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <fstream>
#include <sys/syscall.h>
#include <sys/time.h>
#include <motion_control.h>
#include <tool_manager.h>
#include <coordinate_manager.h>
#include "error_queue.h"

using namespace std;
using namespace basic_alg;
using namespace group_space;
using namespace fst_ctrl;
using namespace base_space;
using namespace log_space;
using namespace group_space;

static void* runRealTimeTask(void *mc)
{
    log_space::LogProducer log_manager;
    uint32_t g_isr = 0;
    log_manager.init("MC_RT", &g_isr);
    ((MotionControl*)mc)->ringRealTimeTask();
    return NULL;
}

static void* runPriorityTask(void *mc)
{
    log_space::LogProducer log_manager;
    uint32_t g_isr = 0;
    log_manager.init("MC_Priority", &g_isr);
    ((MotionControl*)mc)->ringPriorityTask();
    return NULL;
}

static void* runPlannerTask(void *mc)
{
    log_space::LogProducer log_manager;
    uint32_t g_isr = 0;
    log_manager.init("MC_Planner", &g_isr);
    ((MotionControl*)mc)->ringPlannerTask();
    return NULL;
}

static void* runCommonTask(void *mc)
{
    log_space::LogProducer log_manager;
    uint32_t g_isr = 0;
    log_manager.init("MC_Common", &g_isr);
    ((MotionControl*)mc)->ringCommonTask();
    return NULL;
}

void MotionControl::ringCommonTask(void)
{
    LogProducer::warn("mc","ThreadMotionControlCommon TID is %ld", syscall(SYS_gettid));

    while (common_thread_running_)
    {
        group_ptr_->doCommonLoop();

        usleep(param_ptr_->common_cycle_time_ * 1000);
    }

    printf("Common task quit.\n");
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
        //LogProducer::error("mc","sched_setscheduler() failed"); 
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

    if (mlockall(MCL_CURRENT|MCL_FUTURE) == -1) 
    {
        LogProducer::error("mc","mlockall failed");
        return; 
    }

    //Pre-fault our stack
    stack_prefault();

    LogProducer::warn("mc","ThreadMotionControlRealtime TID is %ld", syscall(SYS_gettid));

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
            LogProducer::error("mc","RT task first stage duration over limit: %.6f", duration_1);
        }

        if (duration_2 > 0.02)
        {
            //LogProducer::error("mc","RT task second stage duration over limit: %.6f", duration_2);
        }
  
    }

    printf("Realtime task quit.\n");
}

void MotionControl::ringPriorityTask(void)
{
    LogProducer::warn("mc","ThreadMotionControlPriority TID is %ld", syscall(SYS_gettid));

    while (priority_thread_running_)
    {
        group_ptr_->doPriorityLoop();

        usleep(param_ptr_->priority_cycle_time_ * 1000);
    }

    printf("Priority task quit.\n");
}

void MotionControl::ringPlannerTask(void)
{
    LogProducer::warn("mc","ThreadMotionControlPriority TID is %ld", syscall(SYS_gettid));

    while (planner_thread_running_)
    {
        pthread_mutex_lock(&instruction_mutex_);
        
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
            else if (instruction.type == STE_PAYLOAD)
            {
                err = setPayload(instruction.payload_id);
            }
            else
            {
                LogProducer::error("mc","Invalid instruction type: %d", instruction.type);
                err = INVALID_PARAMETER;
            }

            if (err != SUCCESS)
            {
                LogProducer::error("mc","Execute instruction error, code: 0x%llx", err);
                uint32_t err_level = ((err >> 32) & 0xFFFF);

                if (err_level > 2)
                {
                    motion_error_flag_ = true;
                }

                if (err_level >= 3 && err_level <= 7)
                {
                    LogProducer::error("mc","Call interpreter pause, line: %d", instruction.line_num);
                    (*instruction.interp_pause)(true);
                    (*instruction.set_line_num)(instruction.line_num);
                }
                
                ErrorQueue::instance().push(err);
            }
        }

        pthread_mutex_unlock(&instruction_mutex_);
        usleep(param_ptr_->planner_cycle_time_ * 1000);
    }

    //printf("Received instruction: %d, handled instruction: %d, instruction list size: %d\n", instructions_recv_counter_, instructions_handle_counter_, instruction_fifo_.size());
    printf("Planner task quit.\n");
}


MotionControl::MotionControl(int32_t id):
    Group(id)
{
    coordinate_manager_ptr_ = NULL;
    tool_manager_ptr_ = NULL;
    param_ptr_ = NULL;
    group_ptr_ = NULL;

    common_thread_running_ = false;
    planner_thread_running_ = false;
    priority_thread_running_ = false;
    realtime_thread_running_ = false;

    motion_error_flag_ = false;
    instructions_recv_counter_ = 0;
    instructions_handle_counter_ = 0;

    work_mode_ = group_space::USER_OP_MODE_MANUAL;
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
}

ErrorCode MotionControl::initApplication(fst_ctrl::CoordinateManager* coordinate_manager_ptr, fst_ctrl::ToolManager* tool_manager_ptr)
{
    param_ptr_ = new MotionControlParam();
    
    if (param_ptr_ == NULL)
    {
        return MC_INTERNAL_FAULT;
    }

    if (pthread_mutex_init(&instruction_mutex_, NULL) != 0)
    {
        LogProducer::error("mc","Fail to initialize motion mutex.");
        return MC_INTERNAL_FAULT;
    }

    if(!param_ptr_->loadParam())
    {
        LogProducer::error("mc","Failed to load MotionControl component parameters");
        return MC_INTERNAL_FAULT;
    }

    group_ptr_ = new ArmGroup();

    if (group_ptr_ == NULL)
    {
        LogProducer::error("mc","Fail to create control group of %s", param_ptr_->model_name_.c_str());
        return MC_INTERNAL_FAULT;
    }

    if (coordinate_manager_ptr && tool_manager_ptr)
    {
        coordinate_manager_ptr_ = coordinate_manager_ptr;
        tool_manager_ptr_ = tool_manager_ptr;
    }
    else
    {
        LogProducer::error("mc","coordinate-manager: %x, tool-manager: %x",
                   coordinate_manager_ptr, tool_manager_ptr);
        return INVALID_PARAMETER;
    }


    user_frame_id_ = 0;
    tool_frame_id_ = 0;

    ErrorCode  err = group_ptr_->initGroup(coordinate_manager_ptr_, tool_manager_ptr_, &axis_group_, &sm_, cpu_comm_ptr_);

    if (err == SUCCESS)
    {
        realtime_thread_running_ = true;

        if (realtime_thread_.run(runRealTimeTask, this, 80))
        {
            LogProducer::info("mc","Startup real-time task success.");
        }
        else
        {
            LogProducer::error("mc","Fail to create real-time task.");
            realtime_thread_running_ = false;
            return MC_INTERNAL_FAULT;
        }

        usleep(50 * 1000);
        priority_thread_running_ = true;

        if (priority_thread_.run(runPriorityTask, this, 70))
        {
            LogProducer::info("mc","Startup priority task success.");
        }
        else
        {
            LogProducer::error("mc","Fail to create priority task.");
            priority_thread_running_ = false;
            return MC_INTERNAL_FAULT;
        }

        usleep(50 * 1000);
        planner_thread_running_ = true;

        if (planner_thread_.run(runPlannerTask, this, 60))
        {
            LogProducer::info("mc","Startup planner task success.");
        }
        else
        {
            LogProducer::error("mc","Fail to create planner task.");
            planner_thread_running_ = false;
            return MC_INTERNAL_FAULT;
        }

        usleep(50 * 1000);
        common_thread_running_ = true;

        if (common_thread_.run(runCommonTask, this, 40))
        {
            LogProducer::info("mc","Startup common task success.");
        }
        else
        {
            LogProducer::error("mc","Fail to create common task.");
            common_thread_running_ = false;
            return MC_INTERNAL_FAULT;
        }

        LogProducer::info("mc","Initialize motion group success.");
    }
    else
    {
        LogProducer::error("mc","Fail to init motion group");
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
        LogProducer::error("mc","Cannot manual move, calibrator-state = %d, all motion is forbidden.", MOTION_FORBIDDEN);
        return INVALID_SEQUENCE;
    }
    else if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_LIMITED)
    {
        if (group_ptr_->getManualFrame() != JOINT)
        {
            LogProducer::error("mc","Cannot manual cartesian in limited state, calibrator-state = %d, manual cartesian is forbidden.", MOTION_LIMITED);
            return INVALID_SEQUENCE;
        }
    }

    return group_ptr_->manualMoveStep(&direction[0]);
}

ErrorCode MotionControl::doContinuousManualMove(const GroupDirection &direction)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_FORBIDDEN)
    {
        LogProducer::error("mc","Cannot manual move, calibrator-state = %d, all motion is forbidden.", MOTION_FORBIDDEN);
        return INVALID_SEQUENCE;
    }
    else if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_LIMITED)
    {
        if (group_ptr_->getManualFrame() != JOINT)
        {
            LogProducer::error("mc","Cannot manual cartesian in limited state, calibrator-state = %d, manual-frame = %d, manual cartesian is forbidden.", MOTION_LIMITED, group_ptr_->getManualFrame());
            return INVALID_SEQUENCE;
        }
    }

    return group_ptr_->manualMoveContinuous(&direction[0]);
}

ErrorCode MotionControl::doGotoPointManualMove(const Joint &joint)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        LogProducer::error("mc","Cannot manual move to point in current state, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
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
        LogProducer::error("mc","Cannot manual move to point in current state, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    if (user_frame_id != user_frame_id_ && user_frame_id != -1)
    {
        LogProducer::error("mc","manualMove: user frame ID = %d mismatch with activated user frame = %d.", user_frame_id, user_frame_id_);
        return INVALID_PARAMETER;
    }

    if (tool_frame_id != tool_frame_id_ && tool_frame_id != -1)
    {
        LogProducer::error("mc","manualMove: tool frame ID = %d mismatch with activated tool frame = %d.", tool_frame_id, tool_frame_id_);
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
        LogProducer::error("mc","IK of manual target pose failed.");
        LogProducer::error("mc","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pe.point_.x_, pe.point_.y_, pe.point_.z_, pe.euler_.a_, pe.euler_.b_, pe.euler_.c_);
        LogProducer::error("mc","Posture: %d, %d, %d, %d", point.pose.posture.arm, point.pose.posture.elbow, point.pose.posture.wrist, point.pose.posture.flip);
        LogProducer::error("mc","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
        LogProducer::error("mc","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    return group_ptr_->manualMoveToPoint(point);
}

ErrorCode MotionControl::setOfflineTrajectory(const std::string &offline_trajectory)
{
    string trajectory_file = "/root/robot_data/trajectory/";
    trajectory_file += offline_trajectory;
    return group_ptr_->setOfflineTrajectory(trajectory_file);
}

ErrorCode MotionControl::prepairOfflineTrajectory(void)
{
    if (group_ptr_->getMotionControlState() != STANDBY)
    {
        LogProducer::error("mc","Fail to prepair offline trajectory, state = 0x%x", group_ptr_->getMotionControlState());
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

ErrorCode MotionControl::autoMove(const struct Instruction &instruction)
{
    /*
    MotionControlState state = group_ptr_->getMotionControlState();
    ServoState servo_state = group_ptr_->getServoState();

    if (state != STANDBY && state != STANDBY_TO_AUTO && state != AUTO)
    {
        LogProducer::error("mc","Cannot autoMove in current state: 0x%x", state);
        return INVALID_SEQUENCE;
    }

    if (servo_state != SERVO_IDLE && servo_state != SERVO_RUNNING)
    {
        LogProducer::error("mc","Cannot autoMove in current servo-state: 0x%x", servo_state);
        return INVALID_SEQUENCE;
    }
    */

    // if (state != STANDBY && state != STANDBY_TO_AUTO && state != AUTO)
    // {
    //     LogProducer::error("mc","Cannot autoMove in current state: 0x%x", state);
    //     return INVALID_SEQUENCE;
    // }

    // if (servo_state != SERVO_IDLE && servo_state != SERVO_RUNNING)
    // {
    //     LogProducer::error("mc","Cannot autoMove in current servo-state: 0x%x", servo_state);
    //     return INVALID_SEQUENCE;
    // }

    pthread_mutex_lock(&instruction_mutex_);
    instruction_fifo_.push(instruction);
    instructions_recv_counter_ ++;
    LogProducer::info("mc","New instruction receieved, add to instruction list, list-size: %d, total-received: %d", instruction_fifo_.size(), instructions_recv_counter_);

    if (instruction_fifo_.size() > 1)
    {
        LogProducer::warn("mc","Instruction fifo size larger than normal, it is a trouble.");
    }

    pthread_mutex_unlock(&instruction_mutex_);
    return SUCCESS;
}

ErrorCode MotionControl::autoMove(const MotionTarget &target)
{
    LogProducer::info("mc","MotionControl::autoMove motion-type: %d, smooth-type: %d", target.type, target.smooth_type);
    LogProducer::info("mc","vel: %.6f, acc: %.6f, cnt: %.6f, tool-frame: %d, user-frame: %d", target.vel, target.acc, target.cnt, target.tool_frame_id, target.user_frame_id);
    
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        LogProducer::error("mc","Offset of the group is abnormal, auto move is forbidden, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    if (target.type != MOTION_JOINT && target.type != MOTION_LINE && target.type != MOTION_CIRCLE)
    {
        LogProducer::error("mc","Invalid motion type = %d, autoMove aborted.", target.type);
        return INVALID_PARAMETER;
    }

    if (target.user_frame_id != user_frame_id_ && target.user_frame_id != -1)
    {
        LogProducer::error("mc","autoMove: user frame ID = %d mismatch with activated user frame = %d.", target.user_frame_id, user_frame_id_);
        return MC_FRAME_MISMATCH;
    }

    if (target.tool_frame_id != tool_frame_id_ && target.tool_frame_id != -1)
    {
        LogProducer::error("mc","autoMove: tool frame ID = %d mismatch with activated tool frame = %d.", target.tool_frame_id, tool_frame_id_);
        return MC_TOOL_MISMATCH;
    }

    ErrorCode err = SUCCESS;
    MotionInfo motion_info;
    motion_info.smooth_type = target.smooth_type;
    motion_info.type = target.type;
    motion_info.cnt = target.cnt;
    motion_info.vel = target.vel;
    motion_info.acc = target.acc;
    motion_info.is_swift = target.is_swift;
    PoseEuler user_frame = group_ptr_->getUserFrame();
    PoseEuler tool_frame = group_ptr_->getToolFrame();

    if (target.tool_frame_offset.valid)
    {
        LogProducer::info("mc","Use tool offset, type: %d, id: %d", target.tool_frame_offset.coord_type, target.tool_frame_offset.offset_frame_id);

        if (target.tool_frame_offset.coord_type == COORDINATE_JOINT)
        {
            LogProducer::error("mc","Cannot use an joint space offset onto tool frame");
            return INVALID_PARAMETER;
        }

        err = offsetToolFrame(target.tool_frame_offset.offset_frame_id, target.tool_frame_offset.offset_pose, tool_frame);

        if (err != SUCCESS)
        {
            LogProducer::error("mc","Fail to offset too frame, code = 0x%llx", err);
            return err;
        }
    }

    PoseEuler offset_frame;

    if (target.user_frame_offset.valid)
    {
        LogProducer::info("mc","Use frame offset, type: %d, id: %d", target.user_frame_offset.coord_type, target.user_frame_offset.offset_frame_id);
        
        if (target.user_frame_offset.coord_type == COORDINATE_JOINT)
        {
            char buffer[LOG_TEXT_SIZE];
            LogProducer::info("mc","Offset joint: %s", group_ptr_->printDBLine(&target.user_frame_offset.offset_joint.j1_, buffer, LOG_TEXT_SIZE));
        }
        else
        {
            const PoseEuler &offset = target.user_frame_offset.offset_pose;
            LogProducer::info("mc","Offset pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", offset.point_.x_, offset.point_.y_, offset.point_.z_, offset.euler_.a_, offset.euler_.b_, offset.euler_.c_);

            if (target.user_frame_offset.offset_frame_id < 0)
            {
                // 基于当前user-frame进行偏移
                offset_frame = user_frame;
            }
            else if (target.user_frame_offset.offset_frame_id == 0)
            {
                // 基于base-frame进行偏移
                memset(&offset_frame, 0, sizeof(offset_frame));
            }
            else
            {
                CoordInfo uf_info;
                ErrorCode err = coordinate_manager_ptr_->getCoordInfoById(target.user_frame_offset.offset_frame_id, uf_info);

                if (err != SUCCESS)
                {
                    LogProducer::error("mc","Fail to get reference user frame from given id");
                    return err;
                }

                if (!uf_info.is_valid)
                {
                    LogProducer::error("mc","Reference user frame indicated by given id is invalid");
                    return INVALID_PARAMETER;
                }

                offset_frame = uf_info.data;
            }

            LogProducer::info("mc","Offset frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", offset_frame.point_.x_, offset_frame.point_.y_, offset_frame.point_.z_, offset_frame.euler_.a_, offset_frame.euler_.b_, offset_frame.euler_.c_);
        }
    }

    if (target.type == MOTION_CIRCLE)
    {
        LogProducer::info("mc","Handle via point of this motion");
        err = handlePoint(target.via, user_frame, tool_frame, motion_info.via);

        if (err != SUCCESS)
        {
            LogProducer::error("mc","Fail to handle via point, code = 0x%llx", err);
            return err;
        }

        if (target.user_frame_offset.valid)
        {
            err = target.user_frame_offset.coord_type == COORDINATE_JOINT ? offsetPoint(target.user_frame_offset.offset_joint, motion_info.via) : offsetPoint(offset_frame, target.user_frame_offset.offset_pose, motion_info.via);

            if (err != SUCCESS)
            {
                LogProducer::error("mc","Fail to offset target point, code = 0x%llx", err);
                return err;
            }
        }
    }

    LogProducer::info("mc","Handle target point of this motion");
    err = handlePoint(target.target, user_frame, tool_frame, motion_info.target);

    if (err != SUCCESS)
    {
        LogProducer::error("mc","Fail to handle target point, code = 0x%llx", err);
        return err;
    }

    if (target.user_frame_offset.valid)
    {
        err = target.user_frame_offset.coord_type == COORDINATE_JOINT ? offsetPoint(target.user_frame_offset.offset_joint, motion_info.target) : offsetPoint(offset_frame, target.user_frame_offset.offset_pose, motion_info.target);

        if (err != SUCCESS)
        {
            LogProducer::error("mc","Fail to offset target point, code = 0x%llx", err);
            return err;
        }
    }

    char buffer[LOG_TEXT_SIZE];
    const PoseEuler &target_pose = motion_info.target.pose.pose;
    LogProducer::info("mc","Target-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
    LogProducer::info("mc","Target-turn: %d, %d, %d, %d, %d, %d", motion_info.target.pose.turn.j1, motion_info.target.pose.turn.j2, motion_info.target.pose.turn.j3, motion_info.target.pose.turn.j4, motion_info.target.pose.turn.j5, motion_info.target.pose.turn.j6);
    LogProducer::info("mc","Target-joint: %s", group_ptr_->printDBLine(&motion_info.target.joint.j1_, buffer, LOG_TEXT_SIZE));
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

ErrorCode MotionControl::offsetPoint(const PoseEuler &offset_frame, const PoseEuler &offset, IntactPoint &point)
{
    //LogProducer::info("mc","Offset-frame: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", offset_frame.point_.x_, offset_frame.point_.y_, offset_frame.point_.z_, offset_frame.euler_.a_, offset_frame.euler_.b_, offset_frame.euler_.c_);
    //LogProducer::info("mc","Offset: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", offset.point_.x_, offset.point_.y_, offset.point_.z_, offset.euler_.a_, offset.euler_.b_, offset.euler_.c_);
    //LogProducer::info("mc","Frame: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", point.user_frame.point_.x_, point.user_frame.point_.y_, point.user_frame.point_.z_, point.user_frame.euler_.a_, point.user_frame.euler_.b_, point.user_frame.euler_.c_);
    //LogProducer::info("mc","Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", point.pose.pose.point_.x_, point.pose.pose.point_.y_, point.pose.pose.point_.z_, point.pose.pose.euler_.a_, point.pose.pose.euler_.b_, point.pose.pose.euler_.c_);

    TransMatrix trans_uf, trans_tf, trans_point, trans_frame, trans_offset, trans_result;
    point.pose.pose.convertToTransMatrix(trans_point);
    point.user_frame.convertToTransMatrix(trans_uf);
    point.tool_frame.convertToTransMatrix(trans_tf);
    offset_frame.convertToTransMatrix(trans_frame);
    offset.convertToTransMatrix(trans_offset);

    Point point_offset_in_uf;
    RotationMatrix rotate = trans_uf.rotation_matrix_;
    rotate.inverse();
    rotate.rightMultiply(trans_frame.rotation_matrix_);
    rotate.multiplyByTransVector(trans_offset.trans_vector_, point_offset_in_uf);
    trans_result.trans_vector_ = trans_point.trans_vector_ + point_offset_in_uf;
    
    rotate = trans_uf.rotation_matrix_;
    rotate.inverse();
    rotate.leftMultiply(trans_point.rotation_matrix_).rightMultiply(trans_frame.rotation_matrix_).rightMultiply(trans_offset.rotation_matrix_);
    trans_result.rotation_matrix_ = rotate;
    trans_result.convertToPoseEuler(point.pose.pose);
    //LogProducer::info("mc","Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", point.pose.pose.point_.x_, point.pose.pose.point_.y_, point.pose.pose.point_.z_, point.pose.pose.euler_.a_, point.pose.pose.euler_.b_, point.pose.pose.euler_.c_);
    trans_tf.inverse();
    trans_result.leftMultiply(trans_uf).rightMultiply(trans_tf);
    Kinematics *kinematics_ptr = group_ptr_->getKinematicsPtr();
    
    if (!kinematics_ptr->doIK(trans_result, point.pose.posture, point.pose.turn, point.joint))
    {
        const Posture &posture = point.pose.posture;
        const PoseEuler &pose = point.pose.pose;
        const PoseEuler &tf = point.tool_frame;
        const PoseEuler &uf = point.user_frame;
        LogProducer::error("mc","IK of point failed.");
        LogProducer::error("mc","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
        LogProducer::error("mc","Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
        LogProducer::error("mc","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
        LogProducer::error("mc","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
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
            LogProducer::error("mc","IK of point failed.");
            LogProducer::error("mc","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            LogProducer::error("mc","Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            LogProducer::error("mc","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            LogProducer::error("mc","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            return MC_COMPUTE_IK_FAIL;
        }
    }

    return SUCCESS;
}

ErrorCode MotionControl::offsetToolFrame(int tool_id, const PoseEuler &offset, PoseEuler &tool_frame)
{
    LogProducer::info("mc","Reference-tool-id: %d, offset-pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_id, offset.point_.x_, offset.point_.y_, offset.point_.z_, offset.euler_.a_, offset.euler_.b_, offset.euler_.c_);
    LogProducer::info("mc","Origin-tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame.point_.x_, tool_frame.point_.y_, tool_frame.point_.z_, tool_frame.euler_.a_, tool_frame.euler_.b_, tool_frame.euler_.c_);
    
    TransMatrix trans_tf, trans_tf_ref;
    tool_frame.convertToTransMatrix(trans_tf);

    if (tool_id < 0)
    {
        // 基于当前tool-frame进行偏移
        trans_tf_ref = trans_tf;
    }
    else if (tool_id == 0)
    {
        // 基于flange-frame进行偏移
        trans_tf_ref.trans_vector_.x_ = 0;
        trans_tf_ref.trans_vector_.y_ = 0;
        trans_tf_ref.trans_vector_.z_ = 0;
        trans_tf_ref.rotation_matrix_.eye();
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err = tool_manager_ptr_->getToolInfoById(tool_id, tf_info);

        if (err != SUCCESS)
        {
            LogProducer::error("mc","Fail to get reference tool frame from given id");
            return err;
        }

        if (!tf_info.is_valid)
        {
            LogProducer::error("mc","Reference tool frame indicated by given id is invalid");
            return INVALID_PARAMETER;
        }

        tf_info.data.convertToTransMatrix(trans_tf_ref);
        LogProducer::info("mc","Reference tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf_info.data.point_.x_, tf_info.data.point_.y_, tf_info.data.point_.z_, tf_info.data.euler_.a_, tf_info.data.euler_.b_, tf_info.data.euler_.c_);
    }

    TransMatrix trans_offset, trans_new_tf;
    offset.convertToTransMatrix(trans_offset);
    trans_tf_ref.rotation_matrix_.multiplyByTransVector(trans_offset.trans_vector_, trans_new_tf.trans_vector_);
    trans_new_tf.trans_vector_ += trans_tf.trans_vector_;
    trans_new_tf.rotation_matrix_ = trans_tf_ref.rotation_matrix_;
    trans_new_tf.rotation_matrix_.inverse();
    trans_new_tf.rotation_matrix_.leftMultiply(trans_offset.rotation_matrix_).leftMultiply(trans_tf_ref.rotation_matrix_).rightMultiply(trans_tf.rotation_matrix_);
    trans_new_tf.convertToPoseEuler(tool_frame);
    LogProducer::info("mc","New-tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame.point_.x_, tool_frame.point_.y_, tool_frame.point_.z_, tool_frame.euler_.a_, tool_frame.euler_.b_, tool_frame.euler_.c_);
    return SUCCESS;
}

ErrorCode MotionControl::pauseMove(void)
{
    return group_ptr_->pauseMove();
}

ErrorCode MotionControl::restartMove(void)
{
    return group_ptr_->restartMove();
}

bool MotionControl::isMoving(void)
{
    return group_ptr_->isMoving();
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
    ErrorCode err = group_ptr_->getCalibratorPtr()->setOffset(index, offset);

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

ErrorCode MotionControl::setOffset(const double (&offset)[NUM_OF_JOINT])
{
    ErrorCode err = group_ptr_->getCalibratorPtr()->setOffset(offset);

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

ErrorCode MotionControl::checkOffset(CalibrateState &cali_stat, OffsetState (&offset_stat)[NUM_OF_JOINT])
{
    if (group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->checkOffset(cali_stat, offset_stat);
}

ErrorCode MotionControl::maskOffsetLostError(void)
{
    if (group_ptr_->getServoState() != SERVO_DISABLE)
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
    if (group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->setOffsetState(index, stat);
}

ErrorCode MotionControl::calibrateOffset(double (&offset)[NUM_OF_JOINT])
{
    if (group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->calibrateOffset(offset);
}

ErrorCode MotionControl::calibrateOffset(size_t index, double (&offset)[NUM_OF_JOINT])
{
    if (group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->calibrateOffset(index, offset);
}

ErrorCode MotionControl::calibrateOffset(const size_t *pindex, size_t length, double (&offset)[NUM_OF_JOINT])
{
    if (group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->calibrateOffset(pindex, length, offset);
}


ErrorCode MotionControl::resetEncoderMultiTurnValue(void)
{
    if (group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }
    
    return group_ptr_->getCalibratorPtr()->resetEncoderMultiTurnValue();
}

void MotionControl::getUsingCoord(basic_alg::PoseEuler &usingcoord)
{
    usingcoord = group_ptr_->getUserFrame();
}

void MotionControl::getUsingTool(basic_alg::PoseEuler &usingtool)
{
    usingtool = group_ptr_->getToolFrame();
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
    return SUCCESS;
}

ErrorCode MotionControl::clearGroup(void)
{
    pthread_mutex_lock(&instruction_mutex_);
    while (!instruction_fifo_.empty()) instruction_fifo_.pop();
    pthread_mutex_unlock(&instruction_mutex_);
    return group_ptr_->clearGroup();
}

ErrorCode MotionControl::clearTeachGroup(void)
{
    return group_ptr_->clearTeachGroup();
}

void MotionControl::shiftCoordOfPose(const PoseEuler &old_coord, const PoseEuler &old_pose, const PoseEuler &new_coord, PoseEuler &new_pose)
{
    TransMatrix trans_pose, trans_old_coord, trans_new_coord;
    old_pose.convertToTransMatrix(trans_pose);
    old_coord.convertToTransMatrix(trans_old_coord);
    new_coord.convertToTransMatrix(trans_new_coord);
    trans_new_coord.inverse();
    trans_pose.leftMultiply(trans_old_coord).leftMultiply(trans_new_coord).convertToPoseEuler(new_pose);
}

void MotionControl::shiftToolOfPose(const PoseEuler &old_tool, const PoseEuler &old_pose, const PoseEuler &new_tool, PoseEuler &new_pose)
{
    TransMatrix trans_pose, trans_old_tool, trans_new_tool;
    old_pose.convertToTransMatrix(trans_pose);
    new_tool.convertToTransMatrix(trans_new_tool);
    old_tool.convertToTransMatrix(trans_old_tool);
    trans_old_tool.inverse();
    trans_pose.rightMultiply(trans_old_tool).rightMultiply(trans_new_tool).convertToPoseEuler(new_pose);
}

ErrorCode MotionControl::isLinearPathReachable(uint32_t group_id, 
                                    int32_t start_coord_id, int32_t start_tool_id, const PoseAndPosture &start, 
                                    int32_t target_coord_id, int32_t target_tool_id, const PoseAndPosture &target)
{
    if (start_coord_id < 0) start_coord_id = user_frame_id_;
    if (target_coord_id < 0) target_coord_id = user_frame_id_;
    if (start_tool_id < 0) start_tool_id = tool_frame_id_;
    if (target_tool_id < 0) target_tool_id = tool_frame_id_;
    if (start_coord_id != target_coord_id || start_tool_id != target_tool_id)
    {
        LogProducer::error("mc","Start coord=%d, tool=%d, target coord=%d, tool=%d mismatch, linear path unreachable", start_coord_id, start_tool_id, target_coord_id, target_tool_id);
        return INVALID_PARAMETER;
    }

    BaseGroup *group_ptr = group_ptr_;
    PoseEuler coord, tool;

    if (start_coord_id < 0)
    {
        coord = group_ptr->getUserFrame();
    }
    else if (start_coord_id == 0)
    {
        memset(&coord, 0, sizeof(coord));
    }
    else
    {
        CoordInfo info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(start_coord_id, info);

        if (err_user == SUCCESS && info.is_valid)
        {
            coord = info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get coord from given ID = %d", start_coord_id);
            return err_user == SUCCESS ? INVALID_PARAMETER : err_user;
        }
    }

    if (start_tool_id < 0)
    {
        tool = group_ptr->getToolFrame();
    }
    else if (start_tool_id == 0)
    {
        memset(&tool, 0, sizeof(tool));
    }
    else
    {
        ToolInfo info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(start_tool_id, info);

        if (err_tool == SUCCESS && info.is_valid)
        {
            tool = info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get tool from given ID = %d", start_tool_id);
            return err_tool == SUCCESS ? INVALID_PARAMETER : err_tool;
        }
    }

    IntactPoint start_point, target_point;
    start_point.user_frame = coord;
    start_point.tool_frame = tool;
    start_point.pose = start;
    target_point.user_frame = coord;
    target_point.tool_frame = tool;
    target_point.pose = target;

    TransMatrix trans_coord, trans_tool, trans_pose;
    coord.convertToTransMatrix(trans_coord);
    tool.convertToTransMatrix(trans_tool);
    trans_tool.inverse();
    start.pose.convertToTransMatrix(trans_pose);
    trans_pose.rightMultiply(trans_tool).leftMultiply(trans_coord);

    if (!group_ptr->getKinematicsPtr()->doIK(trans_pose, start.posture, start.turn, start_point.joint))
    {
        const Point &point = start.pose.point_;
        const Euler &euler = start.pose.euler_;
        const Posture posture = start.posture;
        const Turn &turn = start.turn;
        LogProducer::error("mc","Fail to compute IK using given pose: %.6f %.6f %.6f %.6f %.6f %.6f", point.x_, point.y_, point.z_, euler.a_, euler.b_, euler.c_);
        LogProducer::error("mc","Posture: ARM=%d, ELBOW=%d, WRIST=%d, FLIP=%d", posture.arm, posture.elbow, posture.wrist, posture.flip);
        LogProducer::error("mc","Start-turn: %d, %d, %d, %d, %d, %d", turn.j1, turn.j2, turn.j3, turn.j4, turn.j5, turn.j6);
        LogProducer::error("mc","Coord: %.6f %.6f %.6f %.6f %.6f %.6f", coord.point_.x_, coord.point_.y_, coord.point_.z_, coord.euler_.a_, coord.euler_.b_, coord.euler_.c_);
        LogProducer::error("mc","Tool: %.6f %.6f %.6f %.6f %.6f %.6f", tool.point_.x_, tool.point_.y_, tool.point_.z_, tool.euler_.a_, tool.euler_.b_, tool.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    target.pose.convertToTransMatrix(trans_pose);
    trans_pose.rightMultiply(trans_tool).leftMultiply(trans_coord);

    if (!group_ptr->getKinematicsPtr()->doIK(trans_pose, target.posture, target.turn, target_point.joint))
    {
        const Point &point = target.pose.point_;
        const Euler &euler = target.pose.euler_;
        const Posture posture = target.posture;
        const Turn &turn = target.turn;
        LogProducer::error("mc","Fail to compute IK using given pose: %.6f %.6f %.6f %.6f %.6f %.6f", point.x_, point.y_, point.z_, euler.a_, euler.b_, euler.c_);
        LogProducer::error("mc","Posture: ARM=%d, ELBOW=%d, WRIST=%d, FLIP=%d", posture.arm, posture.elbow, posture.wrist, posture.flip);
        LogProducer::error("mc","Target-turn: %d, %d, %d, %d, %d, %d", turn.j1, turn.j2, turn.j3, turn.j4, turn.j5, turn.j6);
        LogProducer::error("mc","Coord: %.6f %.6f %.6f %.6f %.6f %.6f", coord.point_.x_, coord.point_.y_, coord.point_.z_, coord.euler_.a_, coord.euler_.b_, coord.euler_.c_);
        LogProducer::error("mc","Tool: %.6f %.6f %.6f %.6f %.6f %.6f", tool.point_.x_, tool.point_.y_, tool.point_.z_, tool.euler_.a_, tool.euler_.b_, tool.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    return group_ptr->isLinearPathReachable(start_point, target_point);
}

ErrorCode MotionControl::isPoseReachable(uint32_t group_id, const Joint &joint)
{
    return group_ptr_->getSoftConstraintPtr()->isJointInConstraint(joint) ? SUCCESS : JOINT_OUT_OF_CONSTRAINT;
}

ErrorCode MotionControl::isPoseReachable(uint32_t group_id, int32_t coord_id, int32_t tool_id, const PoseAndPosture &pose)
{
    PoseEuler coord, tool;

    if (coord_id < 0)
    {
        coord = group_ptr_->getUserFrame();
    }
    else if (coord_id == 0)
    {
        memset(&coord, 0, sizeof(coord));
    }
    else
    {
        CoordInfo info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(coord_id, info);

        if (err_user == SUCCESS && info.is_valid)
        {
            coord = info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get coord from given ID = %d", coord_id);
            return err_user == SUCCESS ? INVALID_PARAMETER : err_user;
        }
    }

    if (tool_id < 0)
    {
        tool = group_ptr_->getToolFrame();
    }
    else if (tool_id == 0)
    {
        memset(&tool, 0, sizeof(tool));
    }
    else
    {
        ToolInfo info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_id, info);

        if (err_tool == SUCCESS && info.is_valid)
        {
            tool = info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get tool from given ID = %d", tool_id);
            return err_tool == SUCCESS ? INVALID_PARAMETER : err_tool;
        }
    }

    TransMatrix trans_pose, trans_coord, trans_tool;
    pose.pose.convertToTransMatrix(trans_pose);
    coord.convertToTransMatrix(trans_coord);
    tool.convertToTransMatrix(trans_tool);
    trans_tool.inverse();
    trans_pose.rightMultiply(trans_tool).leftMultiply(trans_coord);
    Joint joint;

    if (!group_ptr_->getKinematicsPtr()->doIK(trans_pose, pose.posture, pose.turn, joint))
    {
        const Point &point = pose.pose.point_;
        const Euler &euler = pose.pose.euler_;
        const Posture posture = pose.posture;
        const Turn &turn = pose.turn;
        LogProducer::error("mc","Fail to compute IK using given pose: %.6f %.6f %.6f %.6f %.6f %.6f", point.x_, point.y_, point.z_, euler.a_, euler.b_, euler.c_);
        LogProducer::error("mc","Posture: ARM=%d, ELBOW=%d, WRIST=%d, FLIP=%d", posture.arm, posture.elbow, posture.wrist, posture.flip);
        LogProducer::error("mc","Target-turn: %d, %d, %d, %d, %d, %d", turn.j1, turn.j2, turn.j3, turn.j4, turn.j5, turn.j6);
        LogProducer::error("mc","Coord: %.6f %.6f %.6f %.6f %.6f %.6f", coord.point_.x_, coord.point_.y_, coord.point_.z_, coord.euler_.a_, coord.euler_.b_, coord.euler_.c_);
        LogProducer::error("mc","Tool: %.6f %.6f %.6f %.6f %.6f %.6f", tool.point_.x_, tool.point_.y_, tool.point_.z_, tool.euler_.a_, tool.euler_.b_, tool.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    return isPoseReachable(group_id, joint);
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
            LogProducer::error("mc","Fail to get user frame from given ID = %d", user_frame_id);
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
            LogProducer::error("mc","Fail to get tool frame from given ID = %d", tool_frame_id);
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
            LogProducer::error("mc","Fail to get user frame from given ID.");
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
            LogProducer::error("mc","Fail to get tool frame from given id");
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
            LogProducer::error("mc","Fail to get user frame from given ID = %d", user_frame_id);
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
            LogProducer::error("mc","Fail to get tool frame from given id = %d", tool_frame_id);
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

int MotionControl::getGroupID(void)
{
    return group_ptr_->getID();
}

MotionControlState MotionControl::getMotionControlState(void)
{
    return group_ptr_->getMotionControlState();
}

ServoState MotionControl::getServoState(void)
{
    return group_ptr_->getServoState();
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
    LogProducer::info("mc","Set tool frame: id = %d, current is %d", id, tool_frame_id_);

    if (id == 0)
    {
        PoseEuler tf = {0, 0, 0, 0, 0, 0};
        group_ptr_->setToolFrame(tf);
        tool_frame_id_ = id;
        LogProducer::info("mc","Using tool-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame_id_, tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
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
            LogProducer::info("mc","Using tool-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame_id_, tf_info.data.point_.x_, tf_info.data.point_.y_, tf_info.data.point_.z_, tf_info.data.euler_.a_, tf_info.data.euler_.b_, tf_info.data.euler_.c_);
            return SUCCESS;
        }
        else
        {
            LogProducer::error("mc","Fail to get tool frame from given id");
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
    LogProducer::info("mc","Set user frame id = %d, current is %d", id, user_frame_id_);

    if (id == 0)
    {
        PoseEuler uf = {0, 0, 0, 0, 0, 0};
        group_ptr_->setUserFrame(uf);
        user_frame_id_ = id;
        LogProducer::info("mc","Using user-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", user_frame_id_, uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
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
            LogProducer::info("mc","Using user-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", user_frame_id_, uf_info.data.point_.x_, uf_info.data.point_.y_, uf_info.data.point_.z_, uf_info.data.euler_.a_, uf_info.data.euler_.b_, uf_info.data.euler_.c_);
            return SUCCESS;
        }
        else
        {
            LogProducer::error("mc","Fail to get user frame from given id");
            return err;
        }
    }
}

// payload
ErrorCode MotionControl::setPayload(int id)
{
    MotionControlState mc_state = group_ptr_->getMotionControlState();
    ServoState servo_state = group_ptr_->getServoState();

    if (mc_state != STANDBY)
    {
        LogProducer::error("mc","Cannot set payload while group-state: %d", mc_state);
        return INVALID_SEQUENCE;
    }

    if (servo_state != SERVO_IDLE && servo_state != SERVO_DISABLE)
    {
        LogProducer::error("mc","Cannot set payload while servo-state: %d", servo_state);
        return INVALID_SEQUENCE;
    }

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

//ssr
void MotionControl::setWorkMode(UserOpMode mode)
{
    work_mode_ = mode;
}

UserOpMode MotionControl::getWorkMode(void)
{
    return work_mode_;
}

//pure function no realization
ErrorCode MotionControl::mcGroupHalt(double dec, double jerk)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcGroupInterrupt(double dec, double jerk)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcGroupContinue(void)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcGroupSetPosition(const std::vector<double> &position, bool relative, CoordType_e coord_type)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcMoveDirectAbsolute(const std::vector<double> &position, CoordType_e coord_type,
        double vel_pct, double acc_pct, double jerk)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcMoveLinearAbsolute(const std::vector<double> &position, CoordType_e coord_type, 
        double velocity, double acc, double dec, double jerk)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcGroupReadActualPosition(CoordType_e coord_type, std::vector<double> &position)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcGroupReadActualVelocity(CoordType_e coord_type, std::vector<double> &velocity)
{
    return GROUP_INVALID_PARAM;
}
bool MotionControl::initApplication(void)
{
    return false;
}
bool MotionControl::reloadSystemModel(void)
{
    return true;
}
bool MotionControl::pushBackFB(void* fb_ptr)
{
    return false;
}
base_space::FBQueueStatus_e MotionControl::getFBQStatus()
{
    return FBQ_STATUS_FULL;
}
void MotionControl::processFBQ()
{
    
}
void MotionControl::processTBQ()
{
    
}
void MotionControl::clearBQ()
{
    
}