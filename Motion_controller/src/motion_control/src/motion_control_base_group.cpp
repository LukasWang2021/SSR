/*************************************************************************
	> File Name: motion_control_base_group.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 11时25分56秒
 ************************************************************************/

#include <unistd.h>
#include <string.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <time.h>

#include <motion_control_base_group.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <common_file_path.h>
#include <basic_alg.h>

using namespace std;
using namespace fst_base;
using namespace basic_alg;
using namespace fst_parameter;

static void dumpShareMemory(void)
{
    ofstream  shm_out("/root/share_memory.dump");
    int fd = open("/dev/fst_shmem", O_RDWR);
    void *ptr = mmap(NULL, 524288, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x38100000);
    uint32_t *pdata = (uint32_t*)ptr;
    char buffer[1024];

    for (uint32_t i = 0; i < 524288; i += 16 * 4)
    {
        sprintf(buffer, "0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x", i,
            pdata[0], pdata[1], pdata[2], pdata[3], pdata[4], pdata[5], pdata[6], pdata[7], 
            pdata[8], pdata[9], pdata[10], pdata[11], pdata[12], pdata[13], pdata[14], pdata[15]);
        pdata += 16;
        shm_out << buffer << endl;
    }

    shm_out.flush();
    shm_out.close();
}

// #define OUTPUT_TRAJECTORY_POINTS
// #define DISPLAY_TRAJECTORY_POINTS

namespace fst_mc
{

#ifdef OUTPUT_TRAJECTORY_POINTS
#define OUTPUT_TRAJECTORY_POINTS_SIZE   (200 * 1000)

struct JointOut
{
    double time;
    TrajectoryPoint point;
};

size_t    g_output_trajectory_points_index = 0;
JointOut  g_output_trajectory_points[OUTPUT_TRAJECTORY_POINTS_SIZE];
#endif

// ------- for test only -------- //
// Pose    g_path_point[10000];
// size_t  g_path_index = 0;
// ofstream  g_path("path_points.csv");
// ------- for test only -------- //

BaseGroup::BaseGroup(fst_log::Logger* plog)
{
    group_state_ = UNKNOW;
    log_ptr_ = plog;
    auto_time_ = 0;
    manual_time_ = 0;

    dynamics_ptr_ = NULL;
    kinematics_ptr_ = NULL;

    vel_ratio_ = 0;
    acc_ratio_ = 0;

    fine_cycle_ = 0;
    fine_threshold_ = 10;

    pause_status_.pause_valid = false;
    pause_status_.pause_index = 0;

    stop_request_ = false;
    reset_request_ = false;
    abort_request_ = false;
    clear_request_ = false;
    error_request_ = false;
    offline_request_ = false;
    auto_to_pause_request_ = false;
    pause_to_auto_request_ = false;
    standby_to_auto_request_ = false;
    standby_to_manual_request_ = false;
    auto_to_standby_request_ = false;
    offline_to_standby_request_ = false;
    manual_to_standby_request_ = false;
    pausing_to_pause_request_ = false;
    pause_return_to_standby_request_ = false;
    start_of_motion_ = false;
    memset(&user_frame_, 0, sizeof(user_frame_));
    memset(&tool_frame_, 0, sizeof(tool_frame_));
    memset(&world_frame_, 0, sizeof(world_frame_));
}

BaseGroup::~BaseGroup()
{
#ifdef OUTPUT_TRAJECTORY_POINTS
    PoseQuaternion pose;
    ofstream  points_out("trajectory_points.csv");
    printf("正在将缓存中的轨迹点录入文件：trajectory_points.csv ... 请稍后\n");
    points_out << "level,time,pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],vel[0],vel[1],vel[2],vel[3],vel[4],vel[5],acc[0],acc[1],acc[2],acc[3],acc[4],acc[5],tor[0],tor[1],tor[2],tor[3],tor[4],tor[5],x,y,z,qx,qy,qz,qw" << endl;

    for (size_t i = 0; i < g_output_trajectory_points_index; i++)
    {
        auto &point = g_output_trajectory_points[i].point.state;
        kinematics_ptr_->doFK(point.angle, pose);
        points_out  << g_output_trajectory_points[i].point.level << "," << g_output_trajectory_points[i].time << ","
                    << point.angle[0] << "," << point.angle[1] << "," << point.angle[2] << "," << point.angle[3] << "," << point.angle[4] << "," << point.angle[5] << ","
                    << point.omega[0] << "," << point.omega[1] << "," << point.omega[2] << "," << point.omega[3] << "," << point.omega[4] << "," << point.omega[5] << ","
                    << point.alpha[0] << "," << point.alpha[1] << "," << point.alpha[2] << "," << point.alpha[3] << "," << point.alpha[4] << "," << point.alpha[5] << ","
                    << point.torque[0] << "," << point.torque[1] << "," << point.torque[2] << "," << point.torque[3] << "," << point.torque[4] << "," << point.torque[5] << ","
                    << pose.point_.x_ << "," << pose.point_.y_ << "," << pose.point_.z_ << "," << 
                    pose.quaternion_.x_ << "," << pose.quaternion_.y_ << "," << pose.quaternion_.z_ << "," << pose.quaternion_.w_ << endl;
    }

    points_out.close();
    printf("录入完成！\n");
#endif

    if (dynamics_ptr_ != NULL) {delete dynamics_ptr_; dynamics_ptr_ = NULL;}
    if (kinematics_ptr_ != NULL) {delete kinematics_ptr_; kinematics_ptr_ = NULL;}
}

void BaseGroup::reportError(const ErrorCode &error)
{
    error_monitor_ptr_->add(error);
}

void BaseGroup::setUserFrame(const PoseEuler &uf)
{
    user_frame_ = uf;
    manual_teach_.setUserFrame(uf);
}

void BaseGroup::setToolFrame(const PoseEuler &tf)
{
    tool_frame_ = tf;
    manual_teach_.setToolFrame(tf);
}

void BaseGroup::setWorldFrame(const PoseEuler &wf)
{
    world_frame_ = wf;
    manual_teach_.setWorldFrame(wf);
}

const PoseEuler& BaseGroup::getUserFrame(void)
{
    return user_frame_;
}

const PoseEuler& BaseGroup::getToolFrame(void)
{
    return tool_frame_;
}

const PoseEuler& BaseGroup::getWorldFrame(void)
{
    return world_frame_;
}

// payload
void BaseGroup::getPayload(int &id)
{
    dynamics_ptr_->getPayload(id);
}

ErrorCode BaseGroup::setPayload(int id)
{
    return dynamics_ptr_->setPayload(id);
}

ErrorCode BaseGroup::addPayload(const PayloadInfo& info)
{
    return dynamics_ptr_->addPayload(info);
}

ErrorCode BaseGroup::deletePayload(int id)
{
    return dynamics_ptr_->deletePayload(id);
}

ErrorCode BaseGroup::updatePayload(const PayloadInfo& info)
{
    return dynamics_ptr_->updatePayload(info);
}

ErrorCode BaseGroup::movePayload(int expect_id, int original_id)
{
    return dynamics_ptr_->movePayload(expect_id, original_id);
}

ErrorCode BaseGroup::getPayloadInfoById(int id, PayloadInfo& info)
{
    return dynamics_ptr_->getPayloadInfoById(id, info);
}

vector<PayloadSummaryInfo> BaseGroup::getAllValidPayloadSummaryInfo(void)
{
    return dynamics_ptr_->getAllValidPayloadSummaryInfo();
}

void BaseGroup::getAllValidPayloadSummaryInfo(vector<PayloadSummaryInfo>& info_list)
{
    dynamics_ptr_->getAllValidPayloadSummaryInfo(info_list);
}

ErrorCode BaseGroup::convertCartToJoint(const PoseAndPosture &pose, const PoseEuler &uf, const PoseEuler &tf, Joint &joint)
{
    PoseEuler tcp_in_base, fcp_in_base;
    transformation_.convertPoseFromUserToBase(pose.pose, uf, tcp_in_base);
    transformation_.convertTcpToFcp(tcp_in_base, tf, fcp_in_base);
    return kinematics_ptr_->doIK(fcp_in_base, pose.posture, pose.turn, joint) ? SUCCESS : MC_COMPUTE_IK_FAIL;
}

ErrorCode BaseGroup::convertCartToJoint(const PoseEuler &pose, const PoseEuler &uf, const PoseEuler &tf, Joint &joint)
{
    PoseEuler tcp_in_base, fcp_in_base;
    transformation_.convertPoseFromUserToBase(pose, uf, tcp_in_base);
    transformation_.convertTcpToFcp(tcp_in_base, tf, fcp_in_base);
    return kinematics_ptr_->doIK(fcp_in_base, getLatestJoint(), joint) ? SUCCESS : MC_COMPUTE_IK_FAIL;
}

ErrorCode BaseGroup::convertJointToCart(const Joint &joint, const PoseEuler &uf, const PoseEuler &tf, PoseEuler &pose)
{
    PoseEuler tcp_in_base, fcp_in_base;
    kinematics_ptr_->doFK(joint, fcp_in_base);
    transformation_.convertFcpToTcp(fcp_in_base, tf, tcp_in_base);
    transformation_.convertPoseFromBaseToUser(tcp_in_base, uf, pose);
    return SUCCESS;
}

ErrorCode BaseGroup::convertCartToJoint(const PoseAndPosture &pose, Joint &joint)
{
    PoseEuler tcp_in_base, fcp_in_base;
    transformation_.convertPoseFromUserToBase(pose.pose, user_frame_, tcp_in_base);
    transformation_.convertTcpToFcp(tcp_in_base, tool_frame_, fcp_in_base);
    return kinematics_ptr_->doIK(fcp_in_base, pose.posture, pose.turn, joint) ? SUCCESS : MC_COMPUTE_IK_FAIL;
}

ErrorCode BaseGroup::convertCartToJoint(const PoseEuler &pose, Joint &joint)
{
    PoseEuler tcp_in_base, fcp_in_base;
    transformation_.convertPoseFromUserToBase(pose, user_frame_, tcp_in_base);
    transformation_.convertTcpToFcp(tcp_in_base, tool_frame_, fcp_in_base);
    return kinematics_ptr_->doIK(fcp_in_base, getLatestJoint(), joint) ? SUCCESS : MC_COMPUTE_IK_FAIL;
}

ErrorCode BaseGroup::convertJointToCart(const Joint &joint, PoseEuler &pose)
{
    PoseEuler tcp_in_base, fcp_in_base;
    kinematics_ptr_->doFK(joint, fcp_in_base);
    transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
    transformation_.convertPoseFromBaseToUser(tcp_in_base, user_frame_, pose);
    return SUCCESS;
}

ManualFrame BaseGroup::getManualFrame(void)
{
    return manual_teach_.getManualFrame();
}

ErrorCode BaseGroup::setManualFrame(ManualFrame frame)
{
    FST_INFO("Set manual frame = %d, current frame is %d", frame, manual_teach_.getManualFrame());

    if (group_state_ != STANDBY && group_state_ != DISABLE)
    {
        FST_ERROR("Cannot set frame in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    pthread_mutex_lock(&manual_traj_mutex_);

    if (frame != manual_teach_.getManualFrame())
    {
        manual_teach_.setManualFrame(frame);
    }

    pthread_mutex_unlock(&manual_traj_mutex_);
    FST_INFO("Done.");
    return SUCCESS;
}

void BaseGroup::getManualStepAxis(double *steps)
{
    char buffer[LOG_TEXT_SIZE];
    manual_teach_.getManualStepAxis(steps);
    FST_INFO("Get manual step axis = %s", printDBLine(steps, buffer, LOG_TEXT_SIZE));
}

double BaseGroup::getManualStepPosition(void)
{
    double step = manual_teach_.getManualStepPosition();
    FST_INFO("Get manual step position = %.4f", step);
    return step;
}

double BaseGroup::getManualStepOrientation(void)
{
    double step = manual_teach_.getManualStepOrientation();
    FST_INFO("Get manual step orientation = %.6f", step);
    return step;
}

ErrorCode BaseGroup::setManualStepAxis(const double *steps)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set manual steps for axis = %s", printDBLine(steps, buffer, LOG_TEXT_SIZE));
    return manual_teach_.setManualStepAxis(steps);
}

ErrorCode BaseGroup::setManualStepPosition(double step)
{
    FST_INFO("Set manual step position = %.4f", step);
    return manual_teach_.setManualStepPosition(step);
}

ErrorCode BaseGroup::setManualStepOrientation(double step)
{
    FST_INFO("Set manual step orientation = %.6f", step);
    return manual_teach_.setManualStepOrientation(step);
}

ErrorCode BaseGroup::manualMoveToPoint(const IntactPoint &point)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual to target point");

    if (group_state_ != STANDBY || servo_state_ != SERVO_IDLE)
    {
        FST_ERROR("Cannot manual to target in current group-state = %d, servo-state = %d", group_state_, servo_state_);
        return MC_FAIL_MANUAL_TO_POINT;
    }

    Joint start_joint = start_joint_;
    FST_INFO("Joint: %s", printDBLine(&point.joint[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", point.pose.pose.point_.x_, point.pose.pose.point_.y_, point.pose.pose.point_.z_, point.pose.pose.euler_.a_, point.pose.pose.euler_.b_, point.pose.pose.euler_.c_);
    FST_INFO("Posture: %d, %d, %d, %d", point.pose.posture.arm, point.pose.posture.elbow, point.pose.posture.wrist, point.pose.posture.flip);
    FST_INFO("UserFrame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", point.user_frame.point_.x_, point.user_frame.point_.y_, point.user_frame.point_.z_, point.user_frame.euler_.a_, point.user_frame.euler_.b_, point.user_frame.euler_.c_);
    FST_INFO("ToolFrame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", point.tool_frame.point_.x_, point.tool_frame.point_.y_, point.tool_frame.point_.z_, point.tool_frame.euler_.a_, point.tool_frame.euler_.b_, point.tool_frame.euler_.c_);
    FST_INFO("Start-joint = %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(start_joint))
    {
        FST_ERROR("Start-joint is out of soft constraint, manual-mode-apoint is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (!soft_constraint_.isJointInConstraint(point.joint))
    {
        FST_ERROR("Target-joint out of constraint: %s", printDBLine(&point.joint[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    manual_time_ = 0;
    pthread_mutex_lock(&manual_traj_mutex_);
    ErrorCode err = manual_teach_.manualToJoint(start_joint_, point.joint);
    pthread_mutex_unlock(&manual_traj_mutex_);

    if (err == SUCCESS)
    {
        FST_INFO("Manual move to target joint, total-duration = %.4f, Success.", manual_teach_.getDuration());
        standby_to_manual_request_ = true;
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        return err;
    }
}

ErrorCode BaseGroup::manualMoveStep(const ManualDirection *direction)
{
    FST_INFO("Manual step by direction.");

    if (group_state_ != STANDBY || servo_state_ != SERVO_IDLE)
    {
        FST_ERROR("Cannot manual step in current group-state = %d, servo-state = %d", group_state_, servo_state_);
        return MC_FAIL_MANUAL_STEP;
    }

    Joint start_joint = start_joint_;

    if (!soft_constraint_.isJointInConstraint(start_joint))
    {
        if (manual_teach_.getManualFrame() != JOINT)
        {
            char buffer[LOG_TEXT_SIZE];
            FST_ERROR("Start-joint = %s", printDBLine(&start_joint.j1_, buffer, LOG_TEXT_SIZE));
            FST_ERROR("Start-joint is out of soft constraint, cannot manual in cartesian space.");
            return MC_FAIL_MANUAL_STEP;
        }

        for (size_t i = 0; i < getNumberOfJoint(); i++)
        {
            if (start_joint[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
            {
                FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                            i + 1, start_joint[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                return MC_FAIL_MANUAL_STEP;
            }
            else if (start_joint[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
            {
                FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                            i + 1, start_joint[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                return MC_FAIL_MANUAL_STEP;
            }
        }
    }

    pthread_mutex_lock(&manual_traj_mutex_);
    manual_time_ = 0;
    ErrorCode err = manual_teach_.manualStep(direction, start_joint);
    pthread_mutex_unlock(&manual_traj_mutex_);

    if (err == SUCCESS)
    {
        FST_INFO("Manual move step, total-duration = %.4f, Success.", manual_teach_.getDuration());
        standby_to_manual_request_ = true;
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        return err;
    }
}

ErrorCode BaseGroup::manualMoveContinuous(const ManualDirection *direction)
{
    size_t joint_num = getNumberOfJoint();

    if ((group_state_ != STANDBY && group_state_ != MANUAL) || (group_state_ == STANDBY && servo_state_ != SERVO_IDLE))
    {
        FST_ERROR("Cannot manual continuous in current grp-state = %d, servo-state = %d", group_state_, servo_state_);
        return MC_FAIL_MANUAL_CONTINUOUS;
    }

    Joint start_joint = start_joint_;

    if (!soft_constraint_.isJointInConstraint(start_joint))
    {
        if (manual_teach_.getManualFrame() != JOINT)
        {
            char buffer[LOG_TEXT_SIZE];
            FST_ERROR("Start-joint = %s", printDBLine(&start_joint.j1_, buffer, LOG_TEXT_SIZE));
            FST_ERROR("Start-joint is out of soft constraint, cannot manual in cartesian space.");
            return MC_FAIL_MANUAL_CONTINUOUS;
        }

        for (size_t i = 0; i < joint_num; i++)
        {
            if (start_joint[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
            {
                FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                            i + 1, start_joint[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                return MC_FAIL_MANUAL_CONTINUOUS;
            }
            else if (start_joint[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
            {
                FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                            i + 1, start_joint[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                return MC_FAIL_MANUAL_CONTINUOUS;
            }
        }
    }

    if (group_state_ == STANDBY)
    {
        FST_INFO("Manual continuous by direction.");
        pthread_mutex_lock(&manual_traj_mutex_);
        manual_time_ = 0;
        ErrorCode err = manual_teach_.manualContinuous(direction, start_joint);
        pthread_mutex_unlock(&manual_traj_mutex_);

        if (err == SUCCESS)
        {
            FST_INFO("Manual move continuous, total-duration = %.4f, Success.", manual_teach_.getDuration());
            standby_to_manual_request_ = true;
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
            return err;
        }
    }
    else if (group_state_ == MANUAL)
    {
        pthread_mutex_lock(&manual_traj_mutex_);
        ErrorCode err = manual_teach_.manualContinuous(direction, manual_time_);
        pthread_mutex_unlock(&manual_traj_mutex_);
        return err;
    }
    else
    {
        FST_ERROR("Cannot manual continuous in current grp-state = %d, servo-state = %d", group_state_, servo_state_);
        return MC_FAIL_MANUAL_CONTINUOUS;
    }
}

ErrorCode BaseGroup::manualStop(void)
{
    FST_INFO("Manual stop, grp-state: %d, manual-mode: %d, manual-frame: %d", group_state_, manual_teach_.getManualMode(), manual_teach_.getManualFrame());
    pthread_mutex_lock(&manual_traj_mutex_);

    if (group_state_ == MANUAL && manual_time_ < manual_teach_.getDuration())
    {
        manual_teach_.manualStop(manual_time_);
        FST_INFO("Success, the group will stop in %.4fs", manual_teach_.getDuration() - manual_time_);
    }
    else
    {
        FST_INFO("The group is not in manual state, group-state: %d, manual-time: %.6f, manual-duration: %.6f", group_state_, manual_time_, manual_teach_.getDuration());
    }

    pthread_mutex_unlock(&manual_traj_mutex_);
    return SUCCESS;
}


ErrorCode BaseGroup::pauseMove(void)
{
    /*
    FST_INFO("Pause move request received.");

    if (group_state_ != AUTO)
    {
        FST_WARN("Group state is %d, pause request refused.", group_state_);
        return SUCCESS;
    }

    pthread_mutex_lock(&planner_list_mutex_);

    if (traj_list_ptr_ == NULL)
    {
        pthread_mutex_unlock(&planner_list_mutex_);
        FST_WARN("Trajectory cache list is empty.");
        return SUCCESS;
    }

    FST_INFO("Path list:");
    PathCacheList *path_ptr = path_list_ptr_;

    while (path_ptr)
    {
        PathCache &cache = path_ptr->path_cache;
        FST_INFO("  %p: id = %d, next-ptr = %p, length = %d, smooth-in = %d, smooth-out = %d", path_ptr, path_ptr->id, path_ptr->next_ptr, cache.cache_length, cache.smooth_in_index, cache.smooth_out_index);
        path_ptr = path_ptr->next_ptr;
    }

    FST_INFO("Trajectory list:");
    TrajectoryCacheList *traj_ptr = traj_list_ptr_;

    while (traj_ptr)
    {
        TrajectoryCache &cache = traj_ptr->trajectory_cache;
        FST_INFO("  %p: pick = %d, next-ptr = %p, length = %d, smooth-out = %d, path-ptr = %p", traj_ptr, traj_ptr->pick_index, traj_ptr->next_ptr, cache.cache_length, cache.smooth_out_index, cache.path_cache_ptr);
        traj_ptr = traj_ptr->next_ptr;
    }

    char buffer[LOG_TEXT_SIZE];
    JointState pause_state;
    int pause_index = traj_list_ptr_->pick_index;
    TrajectoryCache &traj_cache = traj_list_ptr_->trajectory_cache;

    if (traj_cache.smooth_out_index == -1 && pause_index >= (int)traj_cache.cache_length)
    {
        pthread_mutex_unlock(&planner_list_mutex_);
        FST_INFO("Near ending point, do not need plan pause trajectory.");
        return SUCCESS;
    }
    else if (traj_cache.smooth_out_index != -1 && pause_index >= (int)traj_cache.smooth_out_index)
    {
        pthread_mutex_unlock(&planner_list_mutex_);
        FST_ERROR("Near smooth point, cannot pause right now.");
        return TRAJ_PLANNING_PAUSE_FAILED;
    }
    
    sampleBlockEnding(traj_cache.cache[pause_index], pause_state);
    TrajectoryCacheList *pause_traj_ptr = traj_cache_pool_.getCachePtr();

    if (pause_traj_ptr == NULL)
    {
        pthread_mutex_unlock(&planner_list_mutex_);
        FST_ERROR("No traj-cache available, set more caches in config file.");
        return MC_NO_ENOUGH_CACHE;
    }

    int paused_on_index = traj_cache.cache[pause_index].index_in_path_cache;
    TrajectoryCache &pause_traj_cache = pause_traj_ptr->trajectory_cache;
    ErrorCode err = planPauseTrajectory(*traj_cache.path_cache_ptr, pause_state, acc_ratio_, pause_traj_cache, paused_on_index);
    
    FST_INFO("angle: %s", printDBLine(&pause_state.angle[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("omega: %s", printDBLine(&pause_state.omega[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("alpha: %s", printDBLine(&pause_state.alpha[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Path cache length: %d, pause index: %d, paused index: %d", traj_cache.path_cache_ptr->cache_length, traj_cache.cache[pause_index].index_in_path_cache, paused_on_index);
    FST_INFO("Pause traj cache length: %d", pause_traj_cache.cache_length);

    for (size_t i = 0; i < pause_traj_cache.cache_length; i++)
    {
        FST_INFO("  duration of block-%d = %.4f", i, pause_traj_cache.cache[i].duration);
    }

    if (err != SUCCESS)
    {
        pthread_mutex_unlock(&planner_list_mutex_);
        traj_cache_pool_.freeCachePtr(pause_traj_ptr);
        FST_ERROR("Fail to plan pause trajectory, code = 0x%llx", err);
        return err;
    }

    // 放弃pause_index之后的轨迹段
    traj_cache.smooth_out_index = pause_index;
    // 放弃后面的轨迹缓存
    TrajectoryCacheList *this_ptr = traj_list_ptr_->next_ptr;
    TrajectoryCacheList *next_ptr;

    while (this_ptr)
    {
        next_ptr = this_ptr->next_ptr;
        traj_cache_pool_.freeCachePtr(this_ptr);
        this_ptr = next_ptr;
    }

    pause_traj_cache.smooth_out_index = pause_traj_cache.cache_length - 1;
    pause_traj_cache.path_cache_ptr = NULL;
    pause_traj_ptr->pick_index = 0;
    pause_traj_ptr->pick_from_block = 0;
    pause_traj_ptr->time_from_start = traj_cache.cache[pause_index].time_from_start + traj_cache.cache[pause_index].duration;
    pause_traj_ptr->next_ptr = NULL;
    updateTimeFromStart(*pause_traj_ptr);
    traj_list_ptr_->trajectory_cache.path_cache_ptr = NULL;
    traj_list_ptr_->next_ptr = pause_traj_ptr;
    pause_status_.pause_valid = true;
    pause_status_.pause_index = paused_on_index;

    this_ptr = traj_list_ptr_;
    while (this_ptr)
    {
        FST_INFO("Cache: %p, pick-index = %d, time-from-start = %f, length = %d", this_ptr, this_ptr->pick_index, this_ptr->time_from_start, this_ptr->trajectory_cache.cache_length);
        this_ptr = this_ptr->next_ptr;
    }

    sampleBlockEnding(pause_traj_cache.cache[pause_traj_cache.cache_length - 1], pause_state);
    start_joint_ = pause_state.angle;
    auto_to_pause_request_ = true;
    pthread_mutex_unlock(&planner_list_mutex_);
    FST_INFO("Pause trajectory planning success, going to pause at: %s", printDBLine(&pause_state.angle[0], buffer, LOG_TEXT_SIZE));
    */
    return SUCCESS;
}

ErrorCode BaseGroup::restartMove(void)
{
    FST_INFO("Restart move request received.");
    /*
    if (group_state_ != PAUSE)
    {
        FST_WARN("Group state is %d, restart request refused.", group_state_);
        return SUCCESS;
    }

    pthread_mutex_lock(&planner_list_mutex_);
    ErrorCode err = SUCCESS;
    Joint start = start_joint_;
    Joint pause = path_list_ptr_->path_cache.cache[pause_status_.pause_index].joint;
    PoseEuler fcp_start_pose, fcp_pause_pose, tcp_start_pose, tcp_pause_pose;
    kinematics_ptr_->doFK(start, fcp_start_pose);
    kinematics_ptr_->doFK(pause, fcp_pause_pose);
    transformation_.convertFcpToTcp(fcp_start_pose, tool_frame_, tcp_start_pose);
    transformation_.convertFcpToTcp(fcp_pause_pose, tool_frame_, tcp_pause_pose);

    if (getDistance(tcp_start_pose.point_, tcp_pause_pose.point_) > 0.1 || getOrientationAngle(tcp_start_pose.euler_, tcp_pause_pose.euler_) > 0.1)
    {
        // 位置或姿态角偏出暂停点，需要先回到暂停点
        char buffer[LOG_TEXT_SIZE];
        FST_INFO("Start-joint: %s", printDBLine(&start[0], buffer, LOG_TEXT_SIZE));
        FST_INFO("Pause-joint: %s", printDBLine(&pause[0], buffer, LOG_TEXT_SIZE));
        FST_INFO("Start-pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", tcp_start_pose.point_.x_, tcp_start_pose.point_.y_, tcp_start_pose.point_.z_, tcp_start_pose.euler_.a_, tcp_start_pose.euler_.b_, tcp_start_pose.euler_.c_);
        FST_INFO("Start-pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", tcp_pause_pose.point_.x_, tcp_pause_pose.point_.y_, tcp_pause_pose.point_.z_, tcp_pause_pose.euler_.a_, tcp_pause_pose.euler_.b_, tcp_pause_pose.euler_.c_);
        FST_INFO("Start-pose different with pause-pose, move back to pause-pose.");
        
        PathCacheList *path_cache_ptr = path_cache_pool_.getCachePtr();
        TrajectoryCacheList *traj_cache_ptr = traj_cache_pool_.getCachePtr();

        if (path_cache_ptr == NULL || traj_cache_ptr == NULL)
        {
            pthread_mutex_unlock(&planner_list_mutex_);
            FST_ERROR("No path-cache (=%p) or traj-cache (=%p) available, set more caches in config file.", path_cache_ptr, traj_cache_ptr);
            path_cache_pool_.freeCachePtr(path_cache_ptr);
            traj_cache_pool_.freeCachePtr(traj_cache_ptr);
            return MC_NO_ENOUGH_CACHE;
        }
        
        MotionInfo target;
        target.type = MOTION_JOINT;
        target.cnt = 0;
        target.vel = 0.0625;
        target.target.joint = pause;
        target.target.user_frame = user_frame_;
        target.target.tool_frame = tool_frame_;
        target.target.pose.posture = kinematics_ptr_->getPostureByJoint(pause);
        PoseEuler fcp_in_base, tcp_in_base;
        kinematics_ptr_->doFK(pause, fcp_in_base);
        transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
        transformation_.convertPoseFromBaseToUser(tcp_in_base, user_frame_, target.target.pose.pose);
        
        err = autoStableJoint(start, target, path_cache_ptr->path_cache, traj_cache_ptr->trajectory_cache);
        path_cache_pool_.freeCachePtr(path_cache_ptr);

        if (err != SUCCESS)
        {
            pthread_mutex_unlock(&planner_list_mutex_);
            traj_cache_pool_.freeCachePtr(traj_cache_ptr);
            FST_ERROR("Fail to plan trajectory back to pause-pose, code = 0x%llx", err);
            return err;
        }
        
        traj_cache_ptr->pick_index = 0;
        traj_cache_ptr->pick_from_block = 0;
        traj_cache_ptr->time_from_start = 0;
        traj_cache_ptr->next_ptr = NULL;
        traj_cache_ptr->trajectory_cache.path_cache_ptr = NULL;
        traj_list_ptr_ = traj_cache_ptr;
        JointState joint_state;
        sampleBlockEnding(traj_cache_ptr->trajectory_cache.cache[traj_cache_ptr->trajectory_cache.cache_length - 1], joint_state);
        start_joint_ = joint_state.angle;
    }

    pause_to_auto_request_ = true;
    pthread_mutex_unlock(&planner_list_mutex_);
    */
    return SUCCESS;
}


ErrorCode BaseGroup::autoMove(const MotionInfo &info)
{
    char buffer[LOG_TEXT_SIZE];
    const PoseEuler &pose = info.target.pose.pose;
    const Posture &posture = info.target.pose.posture;

    FST_INFO("Auto move request received, type = %d", info.type);
    FST_INFO("vel = %.6f, acc = %.6f, cnt = %.6f", info.vel, info.acc, info.cnt);
    FST_INFO("start-joint: %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("target-joint: %s", printDBLine(&info.target.joint.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("target-pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
    FST_INFO("target-posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
    ErrorCode err = checkMotionTarget(info);

    if (err != SUCCESS)
    {
        // moveJ或者movel时如果目标点和起始点重合，直接跳过这条指令的规划执行，什么也不做
        // 如果是其他错误则报错，提前结束运动规划
        if (err == TARGET_COINCIDENCE && info.type != MOTION_CIRCLE)
        {
            FST_WARN("Target coincidence with start, nothing to do.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Parameter check failed, code = 0x%llx", err);
            return err;
        }
    }

    if (plan_traj_ptr_->valid == true)
    {
        FST_ERROR("No trajectory cache available, cannot plan new trajectory before old trajectory finish.");
        return MC_INTERNAL_FAULT;
    }

    FST_INFO("Parameter check passed, planning ...");
    err = plan_traj_ptr_->trajectory.planTrajectory(start_joint_, info, vel_ratio_, acc_ratio_);
    MotionInfo motion_info_this = plan_traj_ptr_->trajectory.getMotionInfo();
    MotionInfo motion_info_pre = pick_traj_ptr_->trajectory.getMotionInfo();

    if (err != SUCCESS)
    {
        FST_ERROR("Planning failed with code = 0x%llx, autoMove aborted.", err);
        return err;
    }

    start_joint_ = info.target.joint;
    FST_INFO("Planning success, duration of trajectory: %.6f", plan_traj_ptr_->trajectory.getDuration());
    double delta_time = 0;
    double distance_start2end = 0.0, quatern_angle_start2end = 0.0;

    if (motion_info_this.type == MOTION_LINE)
    {
        uint32_t point_num_start = 1;
        JointState state_start;
        PoseQuaternion pose_start, pose_target;
        double postion_vel_start;

        err = plan_traj_ptr_->trajectory.sampleCartesianTrajectory(MINIMUM_E9, start_joint_, point_num_start, 
            state_start, pose_start, postion_vel_start);
        info.target.pose.pose.convertToPoseQuaternion(pose_target);

        distance_start2end = pose_start.point_.distanceToPoint(info.target.pose.pose.point_);
        quatern_angle_start2end = fabs(pose_target.quaternion_.getIncludedAngle(pose_start.quaternion_));
    }

    if (pick_traj_ptr_->valid && pick_traj_ptr_->end_with_smooth)
    {
        if (motion_info_this.type != MOTION_LINE
            || MIN_LINE_DISTANCE_TO_SMOOTH < distance_start2end
            || quatern_angle_start2end < MAX_LINE_QUATERNION_ANGLE_NO_SMOOTH) // 上一条指令带平滑: 计算平滑切入点,规划平滑段轨迹
        {
            uint32_t point_num = 1;
            double smooth_in_time = plan_traj_ptr_->trajectory.getSmoothInTime(pick_traj_ptr_->smooth_distance);

            Joint fly_by_joint = pick_traj_ptr_->trajectory.getMotionInfo().target.joint;
            PoseEuler fly_by_pose_e = pick_traj_ptr_->trajectory.getMotionInfo().target.pose.pose;
            PoseQuaternion fly_by_pose;
            fly_by_pose_e.convertToPoseQuaternion(fly_by_pose);

            plan_traj_ptr_->smooth.setUserFrame(info.target.user_frame);
            plan_traj_ptr_->smooth.setToolFrame(info.target.tool_frame);
            plan_traj_ptr_->smooth.setTrajectoryPlanner(pick_traj_ptr_->trajectory, plan_traj_ptr_->trajectory);

            plan_traj_ptr_->smooth.updateTrajectoryInfo(pick_traj_ptr_->smooth_time, smooth_in_time, cycle_time_, 
               pick_traj_ptr_->orientation_smooth_out_time, plan_traj_ptr_->orientation_smooth_in_time);
            plan_traj_ptr_->smooth_in_time = smooth_in_time;

            FST_INFO("pre-orientation-smooth-out-time: %lf, pre-point-smooth-out-time: %lf, point-smooth-in-time: %lf, orientation-smooth-in-time: %lf",
                 pick_traj_ptr_->orientation_smooth_out_time, pick_traj_ptr_->smooth_time, smooth_in_time, plan_traj_ptr_->orientation_smooth_in_time);

            point_num = 1;
            JointState smooth_in_state, smooth_out_state;
            plan_traj_ptr_->trajectory.sampleTrajectory(smooth_in_time, fly_by_joint, point_num, &smooth_in_state);
            pick_traj_ptr_->trajectory.sampleTrajectory(pick_traj_ptr_->smooth_time, fly_by_joint, point_num, &smooth_out_state);

            FST_INFO("smooth-out angle: %s", printDBLine(&smooth_out_state.angle.j1_, buffer, LOG_TEXT_SIZE));
            FST_INFO("smooth-out omega: %s", printDBLine(&smooth_out_state.omega.j1_, buffer, LOG_TEXT_SIZE));
            FST_INFO("smooth-in angle: %s", printDBLine(&smooth_in_state.angle.j1_, buffer, LOG_TEXT_SIZE));
            FST_INFO("smooth-in omega: %s", printDBLine(&smooth_in_state.omega.j1_, buffer, LOG_TEXT_SIZE));
            FST_INFO("fly-by: %s", printDBLine(&fly_by_joint.j1_, buffer, LOG_TEXT_SIZE));
            
            if (motion_info_pre.type == MOTION_CIRCLE && motion_info_this.type == MOTION_CIRCLE)
            {
                PoseQuaternion smooth_in_pose, smooth_out_pose;
                double smooth_in_vel, smooth_out_vel;
	            plan_traj_ptr_->trajectory.sampleCircleCartesianTrajectory(smooth_in_time, smooth_in_pose, smooth_in_vel);
                pick_traj_ptr_->trajectory.sampleCircleCartesianTrajectory(pick_traj_ptr_->smooth_time, smooth_out_pose, smooth_out_vel);

                err = plan_traj_ptr_->smooth.planCircleTrajectory(smooth_out_pose, fly_by_pose, smooth_in_pose, smooth_out_vel, smooth_in_vel, 
                    smooth_out_state, smooth_in_state);
            }
            else 
            {
                err = plan_traj_ptr_->smooth.planTrajectory(smooth_out_state, fly_by_joint, smooth_in_state);
            }

            if (err == SUCCESS)
            {
                plan_traj_ptr_->start_from_smooth = true;
                delta_time = plan_traj_ptr_->smooth.getDuration() - floor(plan_traj_ptr_->smooth.getDuration() / cycle_time_) * cycle_time_;
                FST_INFO("Start from smooth, smooth-in-time: %.6f, orientation-smooth-in-time: %.6f, smooth-duration: %.6f,  delta-time: %.6f", 
                    smooth_in_time, plan_traj_ptr_->orientation_smooth_in_time, plan_traj_ptr_->smooth.getDuration(), delta_time);
            }
            else
            {
                plan_traj_ptr_->start_from_smooth = false;
                FST_INFO("Smooth trajectory plan failed, code = 0x%llx, start from stable.", err);
            }
        }
        else 
        {
            pthread_mutex_lock(&planner_list_mutex_);
            pick_traj_ptr_->end_with_smooth = false;
            pick_traj_ptr_->smooth_time = -1;
            pick_traj_ptr_->orientation_smooth_out_time = -1;
            pick_traj_ptr_->smooth_distance = -1;
            pthread_mutex_unlock(&planner_list_mutex_);
            plan_traj_ptr_->start_from_smooth = false;
            FST_INFO("Start from stable.");
        }
    }
    else 
    {
        plan_traj_ptr_->start_from_smooth = false;
        FST_INFO("Start from stable.");
    }

    if (info.cnt > MINIMUM_E3)
    {
        if (info.type == MOTION_LINE
            && distance_start2end < MIN_LINE_DISTANCE_TO_SMOOTH
            && MAX_LINE_QUATERNION_ANGLE_NO_SMOOTH < quatern_angle_start2end)
        {
            // CNT = 0
            plan_traj_ptr_->smooth_distance = -1;
            plan_traj_ptr_->smooth_time = -1;
            plan_traj_ptr_->orientation_smooth_out_time = -1;
            plan_traj_ptr_->end_with_smooth = false;
            fine_enable_ = false;
            FST_INFO("End with pre-fetch. CNT = 0");
        }
        else 
        {
            // 本条指令带平滑: 计算切出时间和切出点
             plan_traj_ptr_->trajectory.getSmoothOutTimeAndDistance(info.cnt, cycle_time_ - delta_time, plan_traj_ptr_->smooth_time, plan_traj_ptr_->smooth_distance);

             if (info.type == MOTION_LINE)
             {
                 double orientation_smooth_out_time = ceil(plan_traj_ptr_->trajectory.getDuration() / cycle_time_ / 2) * cycle_time_;
                 plan_traj_ptr_->orientation_smooth_out_time = orientation_smooth_out_time > plan_traj_ptr_->smooth_time ? plan_traj_ptr_->smooth_time : orientation_smooth_out_time;
             }
             else 
             {
                plan_traj_ptr_->orientation_smooth_out_time = plan_traj_ptr_->smooth_time;
             }

             FST_INFO("orientation-smooth-out-time: %.6f, point-smooth-time: %.6f", plan_traj_ptr_->orientation_smooth_out_time, plan_traj_ptr_->smooth_time);
    
             if (plan_traj_ptr_->smooth_time > MINIMUM_E3 && plan_traj_ptr_->orientation_smooth_out_time > MINIMUM_E3 && plan_traj_ptr_->smooth_distance > 0.1)
             {
                plan_traj_ptr_->end_with_smooth = true;
                fine_enable_ = false;
                FST_INFO("End with smooth, smooth-time: %.6f, orientation-smooth-time: %.6f, smooth-distance: %.6f", plan_traj_ptr_->smooth_time, plan_traj_ptr_->orientation_smooth_out_time, plan_traj_ptr_->smooth_distance);
             }
             else
             {
                plan_traj_ptr_->smooth_distance = -1;
                plan_traj_ptr_->smooth_time = -1;
                plan_traj_ptr_->orientation_smooth_out_time = -1;
                plan_traj_ptr_->end_with_smooth = false;
                fine_enable_ = false;
                FST_INFO("End with pre-fetch.");
             }
        }
    }
    else if (fabs(info.cnt) < MINIMUM_E3)
    {
        // CNT = 0
        plan_traj_ptr_->smooth_distance = -1;
        plan_traj_ptr_->smooth_time = -1;
        plan_traj_ptr_->orientation_smooth_out_time = -1;
        plan_traj_ptr_->end_with_smooth = false;
        fine_enable_ = false;
        FST_INFO("End with pre-fetch.");
    }
    else
    {
        plan_traj_ptr_->smooth_distance = -1;
        plan_traj_ptr_->smooth_time = -1;
        plan_traj_ptr_->orientation_smooth_out_time = -1;
        plan_traj_ptr_->end_with_smooth = false;
        fine_enable_ = true;
        FST_INFO("End with fine.");
    }
    
    pthread_mutex_lock(&planner_list_mutex_);

    if (plan_traj_ptr_->start_from_smooth)
    {
        if (!pick_traj_ptr_->valid || !pick_traj_ptr_->end_with_smooth)
        {
            // 如果下发线程已经被迫放弃圆滑
            FST_WARN("Smooth fail, start from stable, pick.valid: %d, pick.smooth: %d", pick_traj_ptr_->valid, pick_traj_ptr_->end_with_smooth);
            plan_traj_ptr_->start_from_smooth = false;
        }
    }
    
    plan_traj_ptr_->valid = true;
    plan_traj_ptr_ = plan_traj_ptr_->next;
    pthread_mutex_unlock(&planner_list_mutex_);

    if (group_state_ == STANDBY)
    {
        standby_to_auto_request_ = true;
    }

    FST_INFO("Trajectory plan finished.");
    return err;
}


ErrorCode BaseGroup::checkStartState(const Joint &start_joint)
{
    //if (group_state_ == STANDBY && servo_state_ == SERVO_IDLE && traj_list_ptr_ == NULL && path_list_ptr_ == NULL)
    // FIXME
    if (group_state_ == STANDBY && servo_state_ == SERVO_IDLE)
    {
        Joint control_joint;
        Joint current_joint = getLatestJoint();

        if (bare_core_.getControlPosition(&control_joint[0], getNumberOfJoint()))
        {
            if (!isSameJoint(current_joint, control_joint, joint_tracking_accuracy_))
            {
                char buffer[LOG_TEXT_SIZE];
                FST_ERROR("Control-position different with current-position, it might be a trouble.");
                FST_ERROR("control-position: %s", printDBLine(&control_joint[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("current-position: %s", printDBLine(&current_joint[0], buffer, LOG_TEXT_SIZE));
                return MC_JOINT_TRACKING_ERROR;
            }

            if (!isSameJoint(start_joint, control_joint, joint_tracking_accuracy_))
            {
                char buffer[LOG_TEXT_SIZE];
                FST_ERROR("Control-position different with start-position, it might be a trouble.");
                FST_ERROR("control-position: %s", printDBLine(&control_joint[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("start-position:   %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));
                return MC_JOINT_TRACKING_ERROR;
            }
        }
        else
        {
            FST_ERROR("Cannot get control position from bare core.");
            return MC_COMMUNICATION_WITH_BARECORE_FAIL;
        }
    }

    if (!soft_constraint_.isJointInConstraint(start_joint))
    {
        char buffer[LOG_TEXT_SIZE];
        FST_ERROR("Start joint out of soft constraint.");
        FST_ERROR("  joint: %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));
        FST_ERROR("  upper: %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
        FST_ERROR("  lower: %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    return SUCCESS;
}

ErrorCode BaseGroup::checkMotionTarget(const MotionInfo &info)
{
    if (info.type != MOTION_JOINT && info.type != MOTION_LINE && info.type != MOTION_CIRCLE)
    {
        FST_ERROR("Invalid motion type: %d", info.type);
        return INVALID_PARAMETER;
    }
    
    if (info.smooth_type == SMOOTH_DISTANCE)
    {
        if (info.cnt < -MINIMUM_E9)
        {
            FST_ERROR("Invalid CNT by smooth-distance: %.12f", info.cnt);
            return INVALID_PARAMETER;
        }
    }
    else if (info.smooth_type == SMOOTH_VELOCITY)
    {
        if (info.cnt < -MINIMUM_E9 || info.cnt > 1 + MINIMUM_E9)
        {
            FST_ERROR("Invalid CNT by smooth-velocity: %.12f", info.cnt);
            return INVALID_PARAMETER;
        }
    }
    else
    {
        if (fabs(info.cnt + 1) > MINIMUM_E9)
        {
            FST_ERROR("Invalid CNT by smooth-none: %.12f", info.cnt);
            return INVALID_PARAMETER;
        }
    }

    // CNT ∈ [0, 1] U CNT = -1
    //if (fabs(info.cnt + 1) > MINIMUM_E9 && (info.cnt < -MINIMUM_E9 || info.cnt > 1 + MINIMUM_E9))
    //{
    //    FST_ERROR("Invalid CNT: %.12f", info.cnt);
    //    return INVALID_PARAMETER;
    //}

    if (  ((info.type == MOTION_JOINT) && (info.vel < MINIMUM_E6 || info.vel > 1 + MINIMUM_E6)) ||
          ((info.type == MOTION_LINE || info.type == MOTION_CIRCLE) && (info.vel < cartesian_vel_min_ || info.vel > cartesian_vel_max_))  )
    {
        FST_ERROR("Invalid vel: %.6f", info.vel);
        return INVALID_PARAMETER;
    }

    if (info.acc < MINIMUM_E6 || info.acc > 1 + MINIMUM_E6)
    {
        FST_ERROR("Invalid acc: %.6f", info.acc);
        return INVALID_PARAMETER;
    }

    if (info.type == MOTION_JOINT)
    {
        if (!soft_constraint_.isJointInConstraint(info.target.joint))
        {
            char buffer[LOG_TEXT_SIZE];
            const Posture &posture = info.target.pose.posture;
            const PoseEuler &pose = info.target.pose.pose;
            const PoseEuler &uf = info.target.user_frame;
            const PoseEuler &tf = info.target.tool_frame;

            FST_ERROR("Target joint out of soft constraint.");
            FST_ERROR("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            FST_ERROR("Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            FST_ERROR("Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            FST_ERROR("User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            FST_ERROR("Joint = %s", printDBLine(&info.target.joint[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
            return JOINT_OUT_OF_CONSTRAINT;
        }

        bool is_same_joint = true;
        uint32_t joint_num = getNumberOfJoint();

        for (uint32_t j = 0; j < joint_num; j++)
        {
            if (fabs(info.target.joint[j] - start_joint_[j]) > MINIMUM_E6)
            {
                is_same_joint = false;
                break;
            }
        }

        if (is_same_joint)
        {
            char buffer[LOG_TEXT_SIZE];
            FST_WARN("Target joint coincidence with start joint.");
            FST_WARN("Start-joint = %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
            FST_WARN("Target-joint = %s", printDBLine(&info.target.joint.j1_, buffer, LOG_TEXT_SIZE));
            return TARGET_COINCIDENCE;
        }
    }

    if (info.type == MOTION_LINE)
    {
        PoseEuler start_pose;
        PoseEuler fcp_in_base, tcp_in_base;
        kinematics_ptr_->doFK(start_joint_, fcp_in_base);
        transformation_.convertFcpToTcp(fcp_in_base, info.target.tool_frame, tcp_in_base);
        transformation_.convertPoseFromBaseToUser(tcp_in_base, info.target.user_frame, start_pose);
        Posture start_posture = kinematics_ptr_->getPostureByJoint(start_joint_);
        const Posture &target_posture = info.target.pose.posture;
        FST_INFO("Start-pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
        FST_INFO("Start-posture: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", start_posture.arm, start_posture.elbow, start_posture.wrist, start_posture.flip);

        if (!soft_constraint_.isJointInConstraint(info.target.joint))
        {
            char buffer[LOG_TEXT_SIZE];
            const Posture &posture = info.target.pose.posture;
            const PoseEuler &pose = info.target.pose.pose;
            const PoseEuler &uf = info.target.user_frame;
            const PoseEuler &tf = info.target.tool_frame;

            FST_ERROR("Target joint out of soft constraint.");
            FST_ERROR("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            FST_ERROR("Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            FST_ERROR("Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            FST_ERROR("User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            FST_ERROR("Joint = %s", printDBLine(&info.target.joint[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
            return JOINT_OUT_OF_CONSTRAINT;
        }

        if (!isPostureMatch(start_posture, target_posture))
        {
            FST_ERROR("Posture of target mismatch with start.");
            FST_ERROR("Posture of start: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", start_posture.arm, start_posture.elbow, start_posture.wrist, start_posture.flip);
            FST_ERROR("Posture of target: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", target_posture.arm, target_posture.elbow, target_posture.wrist, target_posture.flip);
            return MC_POSTURE_MISMATCH;
        }

        if (getDistance(start_pose.point_, info.target.pose.pose.point_) < MINIMUM_E3 && getOrientationAngle(start_pose.euler_, info.target.pose.pose.euler_) < MINIMUM_E3)
        {
            char buffer[LOG_TEXT_SIZE];
            const Joint &target_joint = info.target.joint;
            const PoseEuler &target_pose = info.target.pose.pose;
            FST_WARN("Target pose coincidence with start.");
            FST_WARN("Start-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
            FST_WARN("Target-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
            FST_WARN("Start-joint = %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
            FST_WARN("Target-joint = %s", printDBLine(&target_joint.j1_, buffer, LOG_TEXT_SIZE));
            return TARGET_COINCIDENCE;
        }
    }

    if (info.type == MOTION_CIRCLE)
    {
        PoseEuler start_pose;
        PoseEuler fcp_in_base, tcp_in_base;
        kinematics_ptr_->doFK(start_joint_, fcp_in_base);
        transformation_.convertFcpToTcp(fcp_in_base, info.target.tool_frame, tcp_in_base);
        transformation_.convertPoseFromBaseToUser(tcp_in_base, info.target.user_frame, start_pose);
        Posture start_posture = kinematics_ptr_->getPostureByJoint(start_joint_);
        const Posture &target_posture = info.target.pose.posture;
        const Posture &via_posture = info.via.pose.posture;
        FST_INFO("Start-pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
        FST_INFO("Start-posture: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", start_posture.arm, start_posture.elbow, start_posture.wrist, start_posture.flip);

        if (!soft_constraint_.isJointInConstraint(info.target.joint))
        {
            char buffer[LOG_TEXT_SIZE];
            const Posture &posture = info.target.pose.posture;
            const PoseEuler &pose = info.target.pose.pose;
            const PoseEuler &uf = info.target.user_frame;
            const PoseEuler &tf = info.target.tool_frame;

            FST_ERROR("Target joint out of soft constraint.");
            FST_ERROR("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            FST_ERROR("Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            FST_ERROR("Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            FST_ERROR("User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            FST_ERROR("Joint = %s", printDBLine(&info.target.joint[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
            return JOINT_OUT_OF_CONSTRAINT;
        }

        if (!soft_constraint_.isJointInConstraint(info.via.joint))
        {
            char buffer[LOG_TEXT_SIZE];
            const Posture &posture = info.via.pose.posture;
            const PoseEuler &pose = info.via.pose.pose;
            const PoseEuler &uf = info.via.user_frame;
            const PoseEuler &tf = info.via.tool_frame;
            FST_ERROR("Via joint out of soft constraint.");
            FST_ERROR("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            FST_ERROR("Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            FST_ERROR("Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            FST_ERROR("User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            FST_ERROR("Joint = %s", printDBLine(&info.via.joint[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
            return JOINT_OUT_OF_CONSTRAINT;
        }

        if (!isPostureMatch(start_posture, target_posture))
        {
            FST_ERROR("Posture of target mismatch with start.");
            FST_ERROR("Posture of start: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", start_posture.arm, start_posture.elbow, start_posture.wrist, start_posture.flip);
            FST_ERROR("Posture of target: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", target_posture.arm, target_posture.elbow, target_posture.wrist, target_posture.flip);
            return MC_POSTURE_MISMATCH;
        }

        if (!isPostureMatch(start_posture, via_posture))
        {
            FST_ERROR("Posture of via mismatch with start.");
            FST_ERROR("Posture of start: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", start_posture.arm, start_posture.elbow, start_posture.wrist, start_posture.flip);
            FST_ERROR("Posture of via: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", via_posture.arm, via_posture.elbow, via_posture.wrist, via_posture.flip);
            return MC_POSTURE_MISMATCH;
        }

        if (getDistance(start_pose.point_, info.target.pose.pose.point_) < MINIMUM_E3 && getOrientationAngle(start_pose.euler_, info.target.pose.pose.euler_) < MINIMUM_E3)
        {
            char buffer[LOG_TEXT_SIZE];
            const Joint &target_joint = info.target.joint;
            const PoseEuler &target_pose = info.target.pose.pose;
            FST_WARN("Target pose coincidence with start.");
            FST_WARN("Start-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
            FST_WARN("Target-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
            FST_WARN("Start-joint = %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
            FST_WARN("Target-joint = %s", printDBLine(&target_joint.j1_, buffer, LOG_TEXT_SIZE));
            return TARGET_COINCIDENCE;
        }

        if (getDistance(start_pose.point_, info.via.pose.pose.point_) < MINIMUM_E3 && getOrientationAngle(start_pose.euler_, info.via.pose.pose.euler_) < MINIMUM_E3)
        {
            char buffer[LOG_TEXT_SIZE];
            const Joint &via_joint = info.via.joint;
            const PoseEuler &via_pose = info.via.pose.pose;
            FST_WARN("Via pose coincidence with start.");
            FST_WARN("  start-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
            FST_WARN("  via-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", via_pose.point_.x_, via_pose.point_.y_, via_pose.point_.z_, via_pose.euler_.a_, via_pose.euler_.b_, via_pose.euler_.c_);
            FST_WARN("  start-joint = %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
            FST_WARN("  via-joint = %s", printDBLine(&via_joint.j1_, buffer, LOG_TEXT_SIZE));
            return TARGET_COINCIDENCE;
        }

        if (getDistance(info.target.pose.pose.point_, info.via.pose.pose.point_) < MINIMUM_E3 && getOrientationAngle(info.target.pose.pose.euler_, info.via.pose.pose.euler_) < MINIMUM_E3)
        {
            char buffer[LOG_TEXT_SIZE];
            const Joint &via_joint = info.via.joint;
            const Joint &target_joint = info.target.joint;
            const PoseEuler &via_pose = info.via.pose.pose;
            const PoseEuler &target_pose = info.target.pose.pose;
            FST_WARN("Via pose coincidence with target.");
            FST_WARN("  via-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", via_pose.point_.x_, via_pose.point_.y_, via_pose.point_.z_, via_pose.euler_.a_, via_pose.euler_.b_, via_pose.euler_.c_);
            FST_WARN("  target-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
            FST_WARN("  via-joint = %s", printDBLine(&via_joint.j1_, buffer, LOG_TEXT_SIZE));
            FST_WARN("  target-joint = %s", printDBLine(&target_joint.j1_, buffer, LOG_TEXT_SIZE));
            return TARGET_COINCIDENCE;
        }
    }

    return SUCCESS;
}

bool BaseGroup::isPostureMatch(const Posture &posture_1, const Posture &posture_2)
{
    return (posture_1.arm == posture_2.arm) && (posture_1.elbow == posture_2.elbow) && (posture_1.wrist == posture_2.wrist) && (posture_1.flip == posture_2.flip);
}

bool BaseGroup::nextMovePermitted(void)
{
    // FST_WARN("is-next-Move-Permitted ?");
    uint32_t branch = 0;
    GroupState state = group_state_;
    pthread_mutex_lock(&planner_list_mutex_);
    
    if (state == STANDBY && standby_to_auto_request_)
    {
        // 第一条运动指令规划成功,还未开始取点
        branch = 1;
    }

    else if ((state == STANDBY_TO_AUTO || state == AUTO) && fine_enable_)
    {
        // fine语句需等待state切回STANDBY后才允许执行下一条
        branch = 2;
    }

    else if ((state == STANDBY_TO_AUTO || state == AUTO) && plan_traj_ptr_->valid)
    {
        branch = 3;
    }

    else if (
        (state == STANDBY_TO_AUTO || state == AUTO) 
        && pick_traj_ptr_->valid
        && (pick_traj_ptr_->start_from_smooth || (pick_traj_ptr_->end_with_smooth && (auto_time_ < pick_traj_ptr_->orientation_smooth_out_time + MINIMUM_E6)) || !pick_traj_ptr_->end_with_smooth))
    {
        // 运动轨迹上的点还未取完
        branch = 4;
    }
    
    else if (state == AUTO_TO_STANDBY)
    {
        // 正在等待Fine到位
        branch = 5;
    }

    /*
    if (state == PAUSE || state == PAUSING || state == PAUSE_RETURN || state == AUTO_TO_PAUSE || state == PAUSE_TO_PAUSE_RETURN || state == PAUSE_RETURN_TO_STANDBY)
    {
        pthread_mutex_unlock(&planner_list_mutex_);
        return false;
    }
    */

    
    //FST_WARN("Next motion permitted: grp-state = %d, branch = %u", state, branch);

    if (branch == 0)
    {
        FST_WARN("Next motion permitted: state=%d, pick->valid=%d, plan->valid=%d, pick->start_from_smooth=%d, pick->end_with_smooth=%d, auto_time=%.6f, pick->smooth_time=%.6f", 
            state, pick_traj_ptr_->valid, plan_traj_ptr_->valid, pick_traj_ptr_->start_from_smooth, pick_traj_ptr_->end_with_smooth, auto_time_, pick_traj_ptr_->smooth_time);
    }
    else
    {
        //FST_LOG("Next motion not permitted, branch: %d", branch);
    }

    pthread_mutex_unlock(&planner_list_mutex_);
    return branch == 0;
}

Joint BaseGroup::getLatestJoint(void)
{
    pthread_mutex_lock(&servo_mutex_);
    Joint joint(servo_joint_);
    pthread_mutex_unlock(&servo_mutex_);
    return joint;
}

ServoState BaseGroup::getServoState(void)
{
    pthread_mutex_lock(&servo_mutex_);
    ServoState state(servo_state_);
    pthread_mutex_unlock(&servo_mutex_);
    return state;
}

ErrorCode BaseGroup::getServoVersion(std::string &version)
{
    char buffer[256];

    if (bare_core_.readVersion(buffer, 256))
    {
        version = buffer;
        return SUCCESS;
    }
    else
    {
        version.clear();
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }
}

GroupState BaseGroup::getGroupState(void)
{
    return group_state_;
}

ErrorCode BaseGroup::setGlobalVelRatio(double ratio)
{
    FST_INFO("Set global velocity ratio: %.4f", ratio);

    if (ratio < MINIMUM_E3 || ratio > 1)
    {
        FST_ERROR("Given ratio out of range (0, 1)");
        return INVALID_PARAMETER;
    }
    else
    {
        vel_ratio_ = ratio;
        manual_teach_.setGlobalVelRatio(vel_ratio_);
        return SUCCESS;
    }
}

ErrorCode BaseGroup::setGlobalAccRatio(double ratio)
{
    FST_INFO("Set global acceleration ratio: %.4f", ratio);

    if (ratio < MINIMUM_E3 || ratio > 1)
    {
        FST_ERROR("Given ratio out of range (0, 1)");
        return INVALID_PARAMETER;
    }
    else
    {
        acc_ratio_ = ratio;
        manual_teach_.setGlobalAccRatio(acc_ratio_);
        return SUCCESS;
    }
}

double BaseGroup::getGlobalVelRatio(void)
{
    return vel_ratio_;
}

double BaseGroup::getGlobalAccRatio(void)
{
    return acc_ratio_;
}

ErrorCode BaseGroup::pickPointsFromManualTrajectory(TrajectoryPoint *points, uint32_t &length)
{
    uint32_t picked = 0;
    ErrorCode err = SUCCESS;
    pthread_mutex_lock(&manual_traj_mutex_);

    for (uint32_t i = 0; i < length; i++)
    {
        err = pickManualPoint(points[i]);

        if (err != SUCCESS)
        {
            break;
        }

        picked ++;

        if (points[i].level == POINT_ENDING)
        {
            break;
        }
    }

    pthread_mutex_unlock(&manual_traj_mutex_);
    length = picked;

    if (err == SUCCESS)
    {
        if (points[length - 1].level == POINT_ENDING)
        {
            char buffer[LOG_TEXT_SIZE];
            manual_to_standby_request_ = true;
            FST_INFO("Get ending-point: %.4f - %s", manual_time_, printDBLine(&points[length - 1].state.angle[0], buffer, LOG_TEXT_SIZE));
            start_joint_ = points[length - 1].state.angle;
        }

        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to pick points from manual trajectory, code = 0x%llx.", err);
        return err;
    }
}

ErrorCode BaseGroup::pickManualPoint(TrajectoryPoint &point)
{
    static Joint reference = start_joint_;

    if (start_of_motion_)
    {
        start_of_motion_ = false;
        reference = start_joint_;
        point.level = POINT_START;
    }
    else
    {
        point.level = POINT_MIDDLE;
    }

    ErrorCode err = manual_teach_.sampleTrajectory(manual_time_, reference, point.state);
    //memset(static_cast<void*>(&point.state.torque), 0, sizeof(point.state.torque));

    if (err != SUCCESS)
    {
        return err;
    }

    reference = point.state.angle;
    manual_time_ += cycle_time_;

    if (manual_time_ > manual_teach_.getDuration())
    {
        point.level = POINT_ENDING;
    }

    //char buffer[LOG_TEXT_SIZE];
    //FST_INFO(">> manual joint: %s", printDBLine(&point.state.angle[0], buffer, LOG_TEXT_SIZE));
    //FST_INFO(">> manual omega: %s", printDBLine(&point.state.omega[0], buffer, LOG_TEXT_SIZE));
#ifdef OUTPUT_TRAJECTORY_POINTS
    if (g_output_trajectory_points_index < OUTPUT_TRAJECTORY_POINTS_SIZE)
    {
        g_output_trajectory_points[g_output_trajectory_points_index].time = manual_time_;
        g_output_trajectory_points[g_output_trajectory_points_index].point = point;
        g_output_trajectory_points_index ++;
    }
#endif

    return SUCCESS;
}

bool BaseGroup::updateStartJoint(void)
{
    char buffer[LOG_TEXT_SIZE];
    Joint control_joint;
    Joint current_joint = getLatestJoint();

    if (bare_core_.getControlPosition(&control_joint[0], getNumberOfJoint()))
    {
        FST_INFO("Control-position: %s", printDBLine(&control_joint[0], buffer, LOG_TEXT_SIZE));
        FST_INFO("Current-position: %s", printDBLine(&current_joint[0], buffer, LOG_TEXT_SIZE));

        //if (isSameJoint(current_joint, control_joint, MINIMUM_E3))
        if (isSameJoint(current_joint, control_joint, joint_tracking_accuracy_))
        {
            start_joint_ = control_joint;
            memset(&start_joint_[getNumberOfJoint()], 0, (NUM_OF_JOINT - getNumberOfJoint()) * sizeof(double));
            return true;
        }
        else
        {
            FST_ERROR("Control-position different with current-position.");
            return false;
        }
    }
    else
    {
        FST_ERROR("Cannot get control position from bare core.");
        return false;
    }
}

void BaseGroup::fillTrajectoryFifo(void)
{
    static Joint reference = getLatestJoint();
    uint32_t fill_point_num = 0;
    ErrorCode err = SUCCESS;

    if (group_state_ == AUTO || group_state_ == STANDBY_TO_AUTO)
    {
        TrajectoryPoint point;
        uint32_t num_of_point = 1;
        pthread_mutex_lock(&planner_list_mutex_);

        if (traj_fifo_.empty())
        {
            reference = getLatestJoint();
        }

        while (!traj_fifo_.full() && fill_point_num < 50)
        {
            if (!pick_traj_ptr_->valid)
            {
                // 没有可取的轨迹点
                break;
            }
            else if (pick_traj_ptr_->start_from_smooth)
            {
                // 取点位置位于轨迹段前的平滑段上
                err = pick_traj_ptr_->smooth.sampleTrajectory(auto_time_, reference, num_of_point, &point.state);
#ifdef DISPLAY_TRAJECTORY_POINTS
                FST_INFO("s-%.6f: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", auto_time_, point.state.angle[0], point.state.angle[1], point.state.angle[2], point.state.angle[3], point.state.angle[4], point.state.angle[5]);
#endif
                if (num_of_point > 0)
                {
                    if (!soft_constraint_.isJointInConstraint(point.state.angle))
                    {
                        char buffer[LOG_TEXT_SIZE];
                        FST_ERROR("Trajectory point out of soft constraint:");
                        FST_ERROR("Time: %.6f, joint: %s", auto_time_, printDBLine(&point.state.angle[0], buffer, LOG_TEXT_SIZE));
                        reportError(JOINT_OUT_OF_CONSTRAINT);
                        break;
                    }

                    point.level = POINT_MIDDLE;
                    traj_fifo_.push(point);
                    reference = point.state.angle;
                    fill_point_num ++;

                    if (auto_time_ + cycle_time_ >= pick_traj_ptr_->smooth.getDuration())
                    {
                        // 平滑段已取完,不足1个cycle_time的剩余时间累加计入接下来的轨迹段
                        FST_WARN("Trajectory start with smooth, smooth picked out, switch to trajectory.");
                        pick_traj_ptr_->start_from_smooth = false;
                        auto_time_ = pick_traj_ptr_->orientation_smooth_in_time + (auto_time_ + cycle_time_ - pick_traj_ptr_->smooth.getDuration());
                    }
                    else
                    {
                        auto_time_ += cycle_time_ * num_of_point;
                    }

                    // 直接进入下一个循环周期
                    continue;
                }

                if (err != SUCCESS)
                {
                    FST_ERROR("Fail to sample point on trajectory, code = 0x%llx", err);
                    reportError(err);
                    break;
                }
            }
            else
            {
                // 取点位置位于轨迹段上
                if (!pick_traj_ptr_->end_with_smooth && auto_time_ > pick_traj_ptr_->trajectory.getDuration() + cycle_time_)
                {
                    // 本条轨迹不带平滑并且已经取完,不带平滑的轨迹末尾需要多取1个点,保证轨迹上的最后一个点能取到
                    pick_traj_ptr_->valid = false;
                    pick_traj_ptr_ = pick_traj_ptr_->next;
                    auto_time_ = cycle_time_;
                    FST_WARN("Trajectory end without smooth, switch to next trajectory.");
                    continue;
                }
                else if (pick_traj_ptr_->end_with_smooth && auto_time_ > pick_traj_ptr_->orientation_smooth_out_time + MINIMUM_E6)
                {
                    // 有平滑,且平滑切出点之前的轨迹已经取完,且平滑切出点也已经取出
                    if (pick_traj_ptr_->next->valid)
                    {
                        // 下一条语句已经就绪,切换到下一条语句;
                        // 直接开始下个循环
                        auto_time_ = auto_time_ - pick_traj_ptr_->orientation_smooth_out_time;
                        FST_WARN("Trajectory end with smooth, next trajectory available, switch to next trajectory. auto_time_ = %lf", auto_time_);
                        pick_traj_ptr_->valid = false;
                        pick_traj_ptr_ = pick_traj_ptr_->next;
                        continue;
                    }
                    else
                    {
                        // 下一条语句尚未就绪
                        if (traj_fifo_.size() < traj_fifo_lower_limit_)
                        {
                            // 轨迹FIFO中轨迹不足, 放弃平滑, 改为CNT0或FINE
                            FST_WARN("Trajectory end with smooth, next trajectory not available when fifo-size = %d, switch to none smooth trajectory.", traj_fifo_.size());
			                fine_enable_ = true;
                            pick_traj_ptr_->end_with_smooth = false;
                            pick_traj_ptr_->smooth_time = -1;
                            pick_traj_ptr_->orientation_smooth_out_time = -1;
                            pick_traj_ptr_->smooth_distance = -1;
                            continue;
                        }
                        else
                        {
                            // 轨迹FIFO中点数尚足,等待下条指令规划完毕

                            break;
                        }
                    }
                }

                // 取点位置位于轨迹上,能够正常取点
                err = pick_traj_ptr_->trajectory.sampleTrajectory(auto_time_, reference, num_of_point, &point.state);
#ifdef DISPLAY_TRAJECTORY_POINTS
                FST_INFO("t-%.6f: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", auto_time_, point.state.angle[0], point.state.angle[1], point.state.angle[2], point.state.angle[3], point.state.angle[4], point.state.angle[5]);
#endif
                if (num_of_point > 0)
                {
                    if (!soft_constraint_.isJointInConstraint(point.state.angle))
                    {
                        char buffer[LOG_TEXT_SIZE];
                        FST_ERROR("Trajectory point out of soft constraint:");
                        FST_ERROR("Time: %.6f, joint: %s", auto_time_, printDBLine(&point.state.angle[0], buffer, LOG_TEXT_SIZE));
                        reportError(JOINT_OUT_OF_CONSTRAINT);
                        break;
                    }

                    if (start_of_motion_)
                    {
                        start_of_motion_ = false;
                        point.level = POINT_START;
                    }
                    else
                    {
                        point.level = POINT_MIDDLE;
                    }

                    traj_fifo_.push(point);
                    reference = point.state.angle;
                    auto_time_ += cycle_time_ * num_of_point;
                    fill_point_num ++;
                }

                if (err != SUCCESS)
                {
                    FST_ERROR("Fail to sample point on trajectory, code = 0x%llx", err);
                    reportError(err);
                    break;
                }
            }
        }

        pthread_mutex_unlock(&planner_list_mutex_);
    }
}

void BaseGroup::doCommonLoop(void)
{
    updateJointRecorder();
    doStateMachine();
}

void BaseGroup::doRealtimeLoop(void)
{
    sendTrajectoryFlow();
}

void BaseGroup::doPriorityLoop(void)
{
    updateServoStateAndJoint();
    //checkEncoderState();
    fillTrajectoryFifo();
}

void BaseGroup::updateJointRecorder(void)
{
    static size_t loop_cnt = 0;

    if (++loop_cnt > joint_record_update_cycle_)
    {
        if (calibrator_.saveJoint() == SUCCESS)
        {
            loop_cnt = 0;
        }
    }

    if (loop_cnt > joint_record_update_timeout_)
    {
        loop_cnt = 0;
        FST_ERROR("Record timeout, cannot save joint into NvRam.");
        reportError(MC_RECORD_JOINT_TIMEOUT);
    }
}

void BaseGroup::updateServoStateAndJoint(void)
{
    static ServoState barecore_state = SERVO_INIT;
    static Joint barecore_joint = {0};
    static uint32_t encoder_state[NUM_OF_JOINT] = {0};
    static uint32_t fail_cnt = 0;

    if (bare_core_.getLatestJoint(barecore_joint, encoder_state, barecore_state))
    {
        if (servo_state_ != barecore_state)
        {
            FST_INFO("Servo-state switch %d to %d", servo_state_, barecore_state);

            if ((servo_state_ == SERVO_RUNNING) && (barecore_state != SERVO_IDLE))
            {
                FST_ERROR("Group-state: %d, servo-state: %d, point-cache: %d, auto_to_standby_request: %d, auto_to_pause_request: %d", 
                group_state_, servo_state_, bare_core_.isPointCacheEmpty(), auto_to_standby_request_, auto_to_pause_request_);
                dumpShareMemory();
            }
        }

        pthread_mutex_lock(&servo_mutex_);
        servo_state_ = barecore_state;
        servo_joint_ = barecore_joint;
        memcpy(encoder_state_, encoder_state, sizeof(encoder_state_));
        pthread_mutex_unlock(&servo_mutex_);
        fail_cnt = 0;
    }
    else
    {
        if (++fail_cnt > servo_update_timeout_)
        {
            fail_cnt = 0;
            FST_ERROR("Fail to update joint and state from bare core.");
            reportError(MC_COMMUNICATION_WITH_BARECORE_FAIL);
            error_request_ = true;
        }
    }
}

/*
void BaseGroup::checkEncoderState(void)
{
    static uint32_t loop_cnt = 0;
    static uint32_t encoder_state_last[NUM_OF_JOINT] = {0};
    static uint32_t joint_num = getNumberOfJoint();
    static uint32_t encoder_error[NUM_OF_JOINT] = {0};

    for (int j = 0; j < joint_num; j++)
    {
        if (encoder_state_last[j] == encoder_state_[j])
        {
            continue;
        }

        if ((encoder_state_[j] & COMMUNICATION_LOST) || (encoder_state_[j] & COMMUNICATION_ERROR))
        {
            encoder_error[j] = 1;
        }

        encoder_state_last[j] = encoder_state_[j];
    }
}
*/

ErrorCode BaseGroup::sendAutoTrajectoryFlow(void)
{
    if (bare_core_.isPointCacheEmpty())
    {
        size_t length = 10;
        TrajectoryPoint points[10];
        ErrorCode err = pickPointsFromTrajectoryFifo(points, length);

        if (err != SUCCESS)
        {
            FST_ERROR("sendAutoTrajectoryFlow: cannot pick point from trajectory fifo.");
            return err;
        }

        bool res = bare_core_.fillPointCache(points, length, POINT_POS_VEL);

        if (points[length - 1].level == POINT_ENDING)
        {
            // 取到了ENDING-POINT，意味着轨迹FIFO已经取完,必须要切换状态机
            char buffer[LOG_TEXT_SIZE];
            FST_INFO("Get ending-point: %s", printDBLine(&points[length - 1].state.angle[0], buffer, LOG_TEXT_SIZE));
            FST_INFO("Length of this package: %d, fill result: %d", length, res);

            if (group_state_ == AUTO)
            {
                PoseQuaternion fcp_in_base;
                kinematics_ptr_->doFK(points[length - 1].state.angle, fcp_in_base);
                transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, fine_pose_);
                auto_to_standby_request_ = true;
            }
            else if (group_state_ == PAUSING)
            {
                pausing_to_pause_request_ = true;
            }
            else if (group_state_ == PAUSE_RETURN)
            {
                pause_return_to_standby_request_ = true;
            }
        }
    }

    return bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
}

ErrorCode BaseGroup::pickPointsFromTrajectoryFifo(TrajectoryPoint *points, size_t &length)
{
    ErrorCode err = SUCCESS;
    size_t picked_num = 0;
    size_t i;

    for (i = 0; i < length; i++)
    {
        if (!traj_fifo_.fetch(points[i]))
        {
            break;
        }
        
        picked_num++;
    }

    if ((i > 0 && i < length) || (i == length && traj_fifo_.empty()))
    {
        points[i - 1].level = POINT_ENDING;
    }

#ifdef OUTPUT_TRAJECTORY_POINTS
    for (i = 0; i < picked_num; i++)
    {
        if (g_output_trajectory_points_index < OUTPUT_TRAJECTORY_POINTS_SIZE)
        {
            g_output_trajectory_points[g_output_trajectory_points_index].time = 0;
            g_output_trajectory_points[g_output_trajectory_points_index].point = points[i];
            g_output_trajectory_points_index++;
        }
    }
#endif

    length = picked_num;
    return err;
}

ErrorCode BaseGroup::sendManualTrajectoryFlow(void)
{
    ErrorCode err;

    if (bare_core_.isPointCacheEmpty())
    {
        size_t length = 10;
        TrajectoryPoint points[10];
        err = pickPointsFromManualTrajectory(points, length);

        if (err != SUCCESS)
        {
            FST_ERROR("sendPoint: cannot pick point from manual motion.");
            return err;
        }

        bare_core_.fillPointCache(points, length, POINT_POS);
    }

    return bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
}

void BaseGroup::sendTrajectoryFlow(void)
{
    static size_t error_cnt = 0;
    ErrorCode err = SUCCESS;

    if (group_state_ == AUTO && !auto_to_standby_request_ && !auto_to_pause_request_)
    {
        err = sendAutoTrajectoryFlow();
    }
    else if (group_state_ == OFFLINE && !offline_to_standby_request_)
    {
        err = sendOfflineTrajectoryFlow();
    }
    else if (group_state_ == OFFLINE_TO_STANDBY)
    {
        if (!bare_core_.isPointCacheEmpty())
        {
            err = bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
        }
    }
    else if (group_state_ == PAUSING && !pausing_to_pause_request_)
    {
        err = sendAutoTrajectoryFlow();
    }
    else if (group_state_ == PAUSE_RETURN && !pause_return_to_standby_request_)
    {
        err = sendAutoTrajectoryFlow();
    }
    else if (group_state_ == MANUAL && !manual_to_standby_request_)
    {
        err = sendManualTrajectoryFlow();
    }
    else if ((group_state_ == AUTO && (auto_to_standby_request_ || auto_to_pause_request_)) || group_state_ == AUTO_TO_STANDBY || group_state_ == AUTO_TO_PAUSE || group_state_ == MANUAL_TO_STANDBY)
    {
        if (!bare_core_.isPointCacheEmpty())
        {
            err = bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
        }
    }

    if (err == SUCCESS)
    {    
        error_cnt = 0;
    }
    else
    {
        if (err == MC_SEND_TRAJECTORY_FAIL)
        {
            error_cnt ++;

            if (error_cnt > trajectory_flow_timeout_)
            {
                error_cnt = 0;
                error_request_ = true;
                reportError(MC_SEND_TRAJECTORY_FAIL);
                FST_ERROR("sendTrajectoryFlow: bare core time-out.");
            }
        }
        else
        {
            FST_ERROR("sendTrajectoryFlow aborted, code = 0x%llx", err);
            error_request_ = true;
            reportError(err);
            error_cnt = 0;
        }
    }
}

bool BaseGroup::isSameJoint(const Joint &joint1, const Joint &joint2, double thres)
{
    size_t  joint_num = getNumberOfJoint();

    for (size_t i = 0; i < joint_num; i++)
    {
        if (fabs(joint1[i] - joint2[i]) > thres)
        {
            return false;
        }
    }

    return true;
}

bool BaseGroup::isSameJoint(const Joint &joint1, const Joint &joint2, const Joint &thres)
{
    size_t  joint_num = getNumberOfJoint();

    for (size_t i = 0; i < joint_num; i++)
    {
        if (fabs(joint1[i] - joint2[i]) > thres[i])
        {
            return false;
        }
    }

    return true;
}

void BaseGroup::getTypeOfAxis(AxisType *types)
{
    size_t  joint_num = getNumberOfJoint();

    for (size_t i = 0; i < joint_num; i++)
    {
        types[i] = type_of_axis_[i];
    }
}

Transformation* BaseGroup::getTransformationPtr(void)
{
    return &transformation_;
}

Kinematics* BaseGroup::getKinematicsPtr(void)
{
    return kinematics_ptr_;
}

Calibrator* BaseGroup::getCalibratorPtr(void)
{
    return &calibrator_;
}

Constraint* BaseGroup::getSoftConstraintPtr(void)
{
    return &soft_constraint_;
}

ErrorCode BaseGroup::getSoftConstraint(JointConstraint &soft_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    soft_constraint_.getConstraint(soft_constraint);
    FST_INFO("Get soft constraint:");
    FST_INFO("  lower = %s", printDBLine(&soft_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&soft_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::getFirmConstraint(JointConstraint &firm_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    firm_constraint_.getConstraint(firm_constraint);
    FST_INFO("Get firm constraint.");
    FST_INFO("  lower = %s", printDBLine(&firm_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&firm_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::getHardConstraint(JointConstraint &hard_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    hard_constraint_.getConstraint(hard_constraint);
    FST_INFO("Get hard constraint.");
    FST_INFO("  lower = %s", printDBLine(&hard_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&hard_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::setSoftConstraint(const JointConstraint &soft_constraint)
{
    Joint lower, upper;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set soft constraint.");

    soft_constraint_.getConstraint(lower, upper);
    FST_INFO("Origin soft constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    FST_INFO("Given soft constraint:");
    FST_INFO("  lower = %s", printDBLine(&soft_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&soft_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    firm_constraint_.getConstraint(lower, upper);
    FST_INFO("Firm constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    if (firm_constraint_.isCoverConstaint(soft_constraint))
    {
        ParamGroup param;
        vector<double> v_upper(&soft_constraint.upper[0], &soft_constraint.upper[0] + NUM_OF_JOINT);
        vector<double> v_lower(&soft_constraint.lower[0], &soft_constraint.lower[0] + NUM_OF_JOINT);

        for (size_t i = getNumberOfJoint(); i < NUM_OF_JOINT; i++)
        {
            v_upper[i] = 0;
            v_lower[i] = 0;
        }

        if (param.loadParamFile(AXIS_GROUP_DIR"soft_constraint.yaml") &&
            param.setParam("soft_constraint/upper", v_upper) &&
            param.setParam("soft_constraint/lower", v_lower) &&
            param.dumpParamFile())
        {
            soft_constraint_.setConstraint(soft_constraint);
            FST_INFO("Soft constraint updated successfully.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail dumping soft constraint to config file");
            return param.getLastError();
        }
    }
    else
    {
        FST_ERROR("Given soft constraint out of firm constraint.");
        return INVALID_PARAMETER;
    }
}


ErrorCode BaseGroup::setFirmConstraint(const JointConstraint &firm_constraint)
{
    Joint lower, upper;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set firm constraint.");

    firm_constraint_.getConstraint(lower, upper);
    FST_INFO("Origin firm constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    FST_INFO("Given firm constraint:");
    FST_INFO("  lower = %s", printDBLine(&firm_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&firm_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    hard_constraint_.getConstraint(lower, upper);
    FST_INFO("Hard constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    if (hard_constraint_.isCoverConstaint(firm_constraint_))
    {
        ParamGroup param;
        vector<double> v_upper(&firm_constraint.upper[0], &firm_constraint.upper[0] + NUM_OF_JOINT);
        vector<double> v_lower(&firm_constraint.lower[0], &firm_constraint.lower[0] + NUM_OF_JOINT);

        for (size_t i = getNumberOfJoint(); i < NUM_OF_JOINT; i++)
        {
            v_upper[i] = 0;
            v_lower[i] = 0;
        }

        if (param.loadParamFile(AXIS_GROUP_DIR"firm_constraint.yaml") &&
            param.setParam("firm_constraint/upper", v_upper) &&
            param.setParam("firm_constraint/lower", v_lower) &&
            param.dumpParamFile())
        {
            firm_constraint_.setConstraint(firm_constraint);
            FST_INFO("Firm constraint updated successfully.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail dumping firm constraint to config file");
            return param.getLastError();
        }
    }
    else
    {
        FST_ERROR("Given firm constraint out of hard constraint.");
        return INVALID_PARAMETER;
    }
}


ErrorCode BaseGroup::setHardConstraint(const JointConstraint &hard_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set hard constraint.");
    FST_INFO("Origin hard constraint:");
    FST_INFO("  lower = %s", printDBLine(&hard_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&hard_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));

    FST_INFO("Given hard constraint:");
    FST_INFO("  lower = %s", printDBLine(&hard_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&hard_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    ParamGroup param;
    vector<double> v_upper(&hard_constraint.upper[0], &hard_constraint.upper[0] + NUM_OF_JOINT);
    vector<double> v_lower(&hard_constraint.lower[0], &hard_constraint.lower[0] + NUM_OF_JOINT);

    for (size_t i = getNumberOfJoint(); i < NUM_OF_JOINT; i++)
    {
        v_upper[i] = 0;
        v_lower[i] = 0;
    }

    if (param.loadParamFile(AXIS_GROUP_DIR"hard_constraint.yaml") &&
        param.setParam("hard_constraint/upper", v_upper) &&
        param.setParam("hard_constraint/lower", v_lower) &&
        param.dumpParamFile())
    {
        hard_constraint_.setConstraint(hard_constraint);
        FST_INFO("Hard constraint updated successfully.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail dumping hard constraint to config file");
        return param.getLastError();
    }
}

ErrorCode BaseGroup::downloadServoParam(const char *file_name)
{
    ParamGroup param;
    FST_INFO("Download servo parameters to bare core ...");

    if (!param.loadParamFile(file_name))
    {
        FST_ERROR("Fail to load servo config file: %s, code = 0x%llx", file_name, param.getLastError());
        return param.getLastError();
    }

    int param_length;
    int startaddr_stored;
    int stored_length;
    vector<int> stored_conf;

    if (!param.getParam("servo/stored_start", startaddr_stored) ||
        !param.getParam("servo/param_length", param_length) ||
        !param.getParam("servo/stored_length", stored_length) ||
        !param.getParam("servo/stored_param", stored_conf))
    {
        FST_ERROR("Fail to load max velocity of each axis, code = 0x%llx", param.getLastError());
        return param.getLastError();
    }

    if (startaddr_stored + stored_length > param_length)
    {
        FST_ERROR("Servo param format invalid: start_addr + length > param_length");
        return INVALID_PARAMETER;
    }

    if ((int)stored_conf.size() * 4 != stored_length)
    {
        FST_ERROR("Servo param format invalid: stored_length mismatch with array_size");
        return INVALID_PARAMETER;
    }

    char *datastr = new char[param_length];

    if (datastr == NULL)
    {
        FST_ERROR("Fail to buffer servo parameter");
        return MC_INTERNAL_FAULT;
    }

    memcpy(datastr + startaddr_stored, static_cast<void*>(&stored_conf[0]), stored_length);
    
    for (int addr = startaddr_stored; addr < startaddr_stored + stored_length;)
    {
        int seg_len = startaddr_stored + stored_length - addr;
        seg_len = seg_len > 512 ? 512 : seg_len;
        // std::cout << "addr=" << addr << ", startaddr=" << startaddr_stored << ", seg_len=" << seg_len << std::endl;

        if (!bare_core_.downloadServoParam(addr, &datastr[addr], seg_len))
        {
            FST_ERROR("Fail to download servo parameter to barecore");
            return MC_COMMUNICATION_WITH_BARECORE_FAIL;
        }

        addr += seg_len;
    }

    delete [] datastr;
    return SUCCESS;
}



} // namespace fst_mc
