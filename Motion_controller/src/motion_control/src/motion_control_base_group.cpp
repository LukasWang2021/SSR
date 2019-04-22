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
#include <time.h>

#include <motion_control_base_group.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <common_file_path.h>
#include <basic_alg.h>
#include <segment_alg.h>

using namespace std;
using namespace fst_base;
using namespace basic_alg;
using namespace fst_parameter;
using namespace fst_algorithm;

//#define OUTPUT_JOINT_POINT
//#define OUTPUT_PATH_CACHE
//#define OUTPUT_TRAJ_CACHE


namespace fst_mc
{

#ifdef OUTPUT_JOINT_POINT
#define OUTPUT_JOINT_POINT_SIZE   (200 * 1000)

struct JointOut
{
    double time;
    TrajectoryPoint point;
};

size_t    g_joint_output_index = 0;
JointOut  g_joint_output_array[OUTPUT_JOINT_POINT_SIZE];
ofstream  g_joint_out("jout.csv");
#endif

#ifdef OUTPUT_PATH_CACHE
#define OUTPUT_PATH_CACHE_SIZE   (32)
ofstream  g_path_out("pout.csv");
size_t    g_path_output_index = 0;
PathCacheList   g_path_output_array[OUTPUT_PATH_CACHE_SIZE];
#endif

#ifdef OUTPUT_TRAJ_CACHE
#define OUTPUT_TRAJ_CACHE_SIZE   (32)
ofstream  g_traj_out("tout.csv");
size_t    g_traj_output_index = 0;
TrajectoryCacheList   g_traj_output_array[OUTPUT_TRAJ_CACHE_SIZE];
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

    path_list_ptr_ = NULL;
    traj_list_ptr_ = NULL;

    dynamics_ptr_ = NULL;
    kinematics_ptr_ = NULL;

    vel_ratio_ = 0;
    acc_ratio_ = 0;

    pause_status_.pause_valid = false;
    pause_status_.pause_index = 0;

    stop_request_ = false;
    reset_request_ = false;
    abort_request_ = false;
    clear_request_ = false;
    error_request_ = false;
    auto_to_pause_request_ = false;
    pause_to_auto_request_ = false;
    auto_to_standby_request_ = false;
    manual_to_standby_request_ = false;
    pausing_to_pause_request_ = false;
    pause_return_to_standby_request_ = false;
    memset(&user_frame_, 0, sizeof(user_frame_));
    memset(&tool_frame_, 0, sizeof(tool_frame_));
}

BaseGroup::~BaseGroup()
{

#ifdef OUTPUT_JOINT_POINT
    PoseEuler pose;
    printf("正在将缓存中的轨迹点录入文件：jout.csv ... 请稍后\n");
    g_joint_out << "level,time,angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],omega[0],omega[1],omega[2],omega[3],omega[4],omega[5],alpha[0],alpha[1],alpha[2],alpha[3],alpha[4],alpha[5],x,y,z,a,b,c" << endl;

    for (size_t i = 0; i < g_joint_output_index; i++)
    {
        auto &point = g_joint_output_array[i].point;
        kinematics_ptr_->doFK(point.angle, pose);
        g_joint_out << point.level << "," << g_joint_output_array[i].time << ","
                    << point.angle[0] << "," << point.angle[1] << "," << point.angle[2] << "," << point.angle[3] << "," << point.angle[4] << "," << point.angle[5] << ","
                    << point.omega[0] << "," << point.omega[1] << "," << point.omega[2] << "," << point.omega[3] << "," << point.omega[4] << "," << point.omega[5] << ","
                    << point.alpha[0] << "," << point.alpha[1] << "," << point.alpha[2] << "," << point.alpha[3] << "," << point.alpha[4] << "," << point.alpha[5] << ","
                    << pose.point_.x_ << "," << pose.point_.y_ << "," << pose.point_.z_ << "," << pose.euler_.a_ << "," << pose.euler_.b_ << "," << pose.euler_.c_ << endl;
    }

    g_joint_out.close();
    printf("录入完成！\n");
#endif

#ifdef OUTPUT_PATH_CACHE
    printf("正在将缓存中的规划路径录入文件：pout.csv ... 请稍后\n");

    for (size_t i = 0; i < g_path_output_index; i++)
    {
        PathCache &path_cache = g_path_output_array[i].path_cache;
        g_path_out << "path-" << i << ",id=" << g_path_output_array[i].id << ",smooth-in=" << path_cache.smooth_in_index << ",smooth-out=" << path_cache.smooth_out_index << ",cache-length=" << path_cache.cache_length << endl;
        
        if (path_cache.target.type == MOTION_JOINT)
        {
            g_path_out << "move-joint,cnt=" << path_cache.target.cnt << ",vel=" << path_cache.target.vel 
                       << ",target[0]=" << path_cache.target.joint_target[0] << ",target[1]=" << path_cache.target.joint_target[1] << ",target[2]=" << path_cache.target.joint_target[2]
                       << ",target[3]=" << path_cache.target.joint_target[3] << ",target[4]=" << path_cache.target.joint_target[4] << ",target[5]=" << path_cache.target.joint_target[5] << endl;
            
            for (size_t j = 0; j < path_cache.cache_length; j++)
            {
                PathBlock &block = path_cache.cache[j];
                g_path_out << "block-" << j << ",point-type=" << block.point_type << ",motion-type=" << block.motion_type 
                           << ",joint[0]=" << block.joint[0] << ",joint[1]=" << block.joint[1] << ",joint[2]=" << block.joint[2] 
                           << ",joint[3]=" << block.joint[3] << ",joint[4]=" << block.joint[4] << ",joint[5]=" << block.joint[5] << endl;
            }
        }
        else
        {
            if (path_cache.target.type == MOTION_LINE)
            {
                g_path_out << "move-line,cnt=" << path_cache.target.cnt << ",vel=" << path_cache.target.vel 
                           << ",target-x=" << path_cache.target.pose_target.point_.x_ << ",target-y=" << path_cache.target.pose_target.point_.y_ << ",target-z=" << path_cache.target.pose_target.point_.z_
                           << ",target-a=" << path_cache.target.pose_target.euler_.a_ << ",target-b=" << path_cache.target.pose_target.euler_.b_ << ",target-c=" << path_cache.target.pose_target.euler_.c_ << endl;
            }
            else if (path_cache.target.type == MOTION_CIRCLE)
            {
                g_path_out << "move-circle,cnt=" << path_cache.target.cnt << ",vel=" << path_cache.target.vel 
                           << ",via-x=" << path_cache.target.circle_target.pose1.point_.x_ << ",via-y=" << path_cache.target.circle_target.pose1.point_.y_ << ",via-z=" << path_cache.target.circle_target.pose1.point_.z_
                           << ",via-a=" << path_cache.target.circle_target.pose1.euler_.a_ << ",via-b=" << path_cache.target.circle_target.pose1.euler_.b_ << ",via-c=" << path_cache.target.circle_target.pose1.euler_.c_
                           << ",target-x=" << path_cache.target.circle_target.pose2.point_.x_ << ",target-y=" << path_cache.target.circle_target.pose2.point_.y_ << ",target-z=" << path_cache.target.circle_target.pose2.point_.z_
                           << ",target-a=" << path_cache.target.circle_target.pose2.euler_.a_ << ",target-b=" << path_cache.target.circle_target.pose2.euler_.b_ << ",target-c=" << path_cache.target.circle_target.pose2.euler_.c_ << endl;
            }
            else
            {
                g_path_out << "unknow motion type" << endl;
                continue;
            }
            
            for (size_t j = 0; j < path_cache.cache_length; j++)
            {
                PathBlock &block = path_cache.cache[j];
                g_path_out << "block-" << j << ",point-type=" << block.point_type << ",motion-type=" << block.motion_type 
                           << ",x=" << block.pose.point_.x_ << ",y=" << block.pose.point_.y_ << ",z=" << block.pose.point_.z_ 
                           << ",ow=" << block.pose.quaternion_.w_ << ",ox=" << block.pose.quaternion_.x_ << ",oy=" << block.pose.quaternion_.y_ << ",oz=" << block.pose.quaternion_.z_ << endl;
            }
        }
    }

    g_path_out.close();
    printf("录入完成！\n");
#endif

#ifdef OUTPUT_TRAJ_CACHE
    printf("正在将缓存中的规划轨迹录入文件：tout.csv ... 请稍后\n");

    for (size_t i = 0; i < g_traj_output_index; i++)
    {
        TrajectoryCache &traj_cache = g_traj_output_array[i].trajectory_cache;
        double total_duration = traj_cache.cache_length > 0 ? traj_cache.cache[traj_cache.cache_length].time_from_start - g_traj_output_array[i].time_from_start : 0;
        g_traj_out << "traj-" << i << ",time-from-start=" << g_traj_output_array[i].time_from_start << ",total-duration=" << total_duration << ",smooth-out=" << traj_cache.smooth_out_index << ",cache-length=" << traj_cache.cache_length << endl;
        
        for (size_t j = 0; j < traj_cache.cache_length; j++)
        {
            TrajectoryBlock &block = traj_cache.cache[j];
            g_traj_out << "block-" << j << ",index-in-path=" << block.index_in_path_cache << ",time-from-start=" << block.time_from_start << ",duration=" << block.duration << endl;

            for (size_t k = 0; k < NUM_OF_JOINT; k++)
            {
                g_traj_out << "axis-" << k << ",c0=" << block.axis[k].data[0] << ",c1=" << block.axis[k].data[1] << ",c2=" << block.axis[k].data[2] << ",c3=" << block.axis[k].data[3] << ",c4=" << block.axis[k].data[4] << ",c5=" << block.axis[k].data[5] << endl;
            }
        }
    }

    g_traj_out.close();
    printf("录入完成！\n");
#endif

    // ------- for test only -------- //
    // for (size_t i = 0; i < g_path_index; i++)
    // {
    //     Pose &p = g_path_point[i];
    //     g_path << p[0] << "," << p[1] << "," << p[2] << "," << p[3] << "," << p[4] << "," << p[5] << "," << p[6] << endl;
    // }

    // g_path.close();
    // ------- for test only -------- //
}

void BaseGroup::reportError(const ErrorCode &error)
{
    error_monitor_ptr_->add(error);
}

ErrorCode BaseGroup::resetGroup(void)
{
    reset_request_ = true;
    return SUCCESS;
}

ErrorCode BaseGroup::stopGroup(void)
{
    stop_request_ = true;
    return true;
}

ErrorCode BaseGroup::abortMove(void)
{
    abort_request_ = true;
    return SUCCESS;
}

ErrorCode BaseGroup::clearGroup(void)
{
    clear_request_ = true;
    return SUCCESS;
}

void BaseGroup::setUserFrame(const PoseEuler &uf)
{
    user_frame_ = uf;
}

void BaseGroup::setToolFrame(const PoseEuler &tf)
{
    tool_frame_ = tf;
}

void BaseGroup::setWorldFrame(const PoseEuler &wf)
{
    world_frame_ = wf;
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
    //FST_INFO("Get manual frame = %d", manual_traj_.frame);
    return manual_traj_.frame;
}

ErrorCode BaseGroup::setManualFrame(ManualFrame frame)
{
    FST_INFO("Set manual frame = %d, current frame is %d", frame, manual_traj_.frame);

    if (group_state_ != STANDBY && group_state_ != DISABLE)
    {
        FST_ERROR("Cannot set frame in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    pthread_mutex_lock(&manual_traj_mutex_);

    if (frame != manual_traj_.frame)
    {
        manual_traj_.frame = frame;
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
    FST_INFO("Manual to target point:");

    if (group_state_ != STANDBY || servo_state_ != SERVO_IDLE)
    {
        FST_ERROR("Cannot manual to target in current group-state = %d, servo-state = %d", group_state_, servo_state_);
        return MC_FAIL_MANUAL_TO_POINT;
    }

    manual_traj_.joint_start = start_joint_;
    FST_INFO("Joint: %s", printDBLine(&point.joint[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", point.pose.pose.point_.x_, point.pose.pose.point_.y_, point.pose.pose.point_.z_, point.pose.pose.euler_.a_, point.pose.pose.euler_.b_, point.pose.pose.euler_.c_);
    FST_INFO("Posture: %d, %d, %d, %d", point.pose.posture.arm, point.pose.posture.elbow, point.pose.posture.wrist, point.pose.posture.flip);
    FST_INFO("UserFrame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", point.user_frame.point_.x_, point.user_frame.point_.y_, point.user_frame.point_.z_, point.user_frame.euler_.a_, point.user_frame.euler_.b_, point.user_frame.euler_.c_);
    FST_INFO("ToolFrame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", point.tool_frame.point_.x_, point.tool_frame.point_.y_, point.tool_frame.point_.z_, point.tool_frame.euler_.a_, point.tool_frame.euler_.b_, point.tool_frame.euler_.c_);
    FST_INFO("Start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
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
    manual_traj_.mode = APOINT;
    pthread_mutex_lock(&manual_traj_mutex_);
    ErrorCode err = manual_teach_.manualByTarget(point.joint, manual_time_, manual_traj_);
    pthread_mutex_unlock(&manual_traj_mutex_);

    if (err == SUCCESS)
    {
        FST_INFO("Manual move to target joint, total-duration = %.4f, Success.", manual_traj_.duration);
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        ManualFrame frame = manual_traj_.frame;
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        manual_traj_.frame = frame;
        return err;
    }
}

/*
ErrorCode BaseGroup::manualMoveToPoint(const PoseEuler &pose)
{
    ErrorCode err;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual to target pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f",
             pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);

    if (group_state_ != STANDBY || servo_state_ != SERVO_IDLE)
    {
        FST_ERROR("Cannot manual to target in current group-state = %d, servo-state = %d", group_state_, servo_state_);
        return SUCCESS;
    }

    if (manual_traj_.frame != BASE && manual_traj_.frame != USER && manual_traj_.frame != WORLD)
    {
        FST_ERROR("Cannot manual to target in current frame = %d", manual_traj_.frame);
        return INVALID_SEQUENCE;
    }

    manual_traj_.joint_start = start_joint_;
    FST_ERROR("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        FST_ERROR("start-joint is out of soft constraint, manual-mode-apoint is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    Joint res_joint;
    Joint ref_joint = getLatestJoint();
    PoseEuler pose_of_fcp, fcp_in_base;

    transformation_.convertTcpToFcp(pose, tool_frame_, pose_of_fcp);

    switch (manual_traj_.frame)
    {
        case BASE:
            err = kinematics_ptr_->doIK(pose_of_fcp, ref_joint, res_joint) ? SUCCESS : MC_COMPUTE_IK_FAIL;
            break;
        case USER:
            transformation_.convertPoseFromUserToBase(pose_of_fcp, user_frame_, fcp_in_base);
            err = kinematics_ptr_->doIK(fcp_in_base, ref_joint, res_joint) ? SUCCESS : MC_COMPUTE_IK_FAIL;
            break;
        case WORLD:
            transformation_.convertPoseFromUserToBase(pose_of_fcp, world_frame_, fcp_in_base);
            err = kinematics_ptr_->doIK(fcp_in_base, ref_joint, res_joint) ? SUCCESS : MC_COMPUTE_IK_FAIL;    // TODO: transform from world frame to base frame
            break;
        default:
            FST_ERROR("Invalid manual frame = %d in manual to pose mode", manual_traj_.frame);
            return MC_INTERNAL_FAULT;
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to get inverse kinematics of given target, code = 0x%llx", err);
        return err;
    }

    if (!soft_constraint_.isJointInConstraint(res_joint))
    {
        FST_ERROR("target-joint is out of soft constraint: %s", printDBLine(&res_joint[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    manual_time_ = 0;
    manual_traj_.mode = APOINT;
    //TODO
    //manual_traj_.frame = JOINT;
    err = manual_teach_.manualByTarget(res_joint, manual_time_, manual_traj_);

    if (err == SUCCESS)
    {
        FST_INFO("Manual move to target pose, total-duration = %.4f, Success.", manual_traj_.duration);
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        ManualFrame = frame = manual_traj_.frame;
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        manual_traj_.frame = frame;
        return err;
    }
}
*/

ErrorCode BaseGroup::manualMoveStep(const ManualDirection *direction)
{
    FST_INFO("Manual step frame=%d by direction.", manual_traj_.frame);

    if (group_state_ != STANDBY || servo_state_ != SERVO_IDLE)
    {
        FST_ERROR("Cannot manual step in current group-state = %d, servo-state = %d", group_state_, servo_state_);
        return MC_FAIL_MANUAL_STEP;
    }

    char buffer[LOG_TEXT_SIZE];
    manual_traj_.joint_start = start_joint_;
    FST_INFO("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        if (manual_traj_.frame == JOINT)
        {
            for (size_t i = 0; i < getNumberOfJoint(); i++)
            {
                if (manual_traj_.joint_start[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
                {
                    FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                              i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                    return MC_FAIL_MANUAL_STEP;
                }
                else if (manual_traj_.joint_start[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
                {
                    FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                              i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                    return MC_FAIL_MANUAL_STEP;
                }
            }
        }
        else
        {
            FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
            return MC_FAIL_MANUAL_STEP;
        }
    }

    PoseEuler fcp_in_base, tcp_in_base, tcp_in_user;

    switch (manual_traj_.frame)
    {
        case JOINT:
            break;
        case BASE:
            kinematics_ptr_->doFK(manual_traj_.joint_start, fcp_in_base);
            transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
            manual_traj_.cart_start = tcp_in_base;
        case USER:
            kinematics_ptr_->doFK(manual_traj_.joint_start, fcp_in_base);
            transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
            transformation_.convertPoseFromBaseToUser(tcp_in_base, user_frame_, tcp_in_user);
            manual_traj_.cart_start = tcp_in_user;
            break;
        case WORLD:
            kinematics_ptr_->doFK(manual_traj_.joint_start, fcp_in_base);
            transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
            transformation_.convertPoseFromBaseToUser(tcp_in_base, world_frame_, tcp_in_user);
            manual_traj_.cart_start = tcp_in_user;
            break;
        case TOOL:
            kinematics_ptr_->doFK(manual_traj_.joint_start, fcp_in_base);
            manual_traj_.tool_coordinate = fcp_in_base;
            memset(&manual_traj_.cart_start, 0, sizeof(manual_traj_.cart_start));
            break;
        default:
            FST_ERROR("Unsupported manual frame: %d", manual_traj_.frame);
            return MC_INTERNAL_FAULT;
    }

    manual_time_ = 0;
    manual_traj_.mode = STEP;
    pthread_mutex_lock(&manual_traj_mutex_);
    ErrorCode err = manual_teach_.manualStepByDirect(direction, manual_time_, manual_traj_);
    pthread_mutex_unlock(&manual_traj_mutex_);

    if (err == SUCCESS)
    {
        FST_INFO("Manual move step, total-duration = %.4f, Success.", manual_traj_.duration);
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        ManualFrame frame = manual_traj_.frame;
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        manual_traj_.frame = frame;
        return err;
    }
}

ErrorCode BaseGroup::manualMoveContinuous(const ManualDirection *direction)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual continuous frame=%d by direction.", manual_traj_.frame);

    if ((group_state_ != STANDBY && group_state_ != MANUAL) || (group_state_ == STANDBY && servo_state_ != SERVO_IDLE))
    {
        FST_ERROR("Cannot manual continuous in current grp-state = %d, servo-state = %d", group_state_, servo_state_);
        return MC_FAIL_MANUAL_CONTINUOUS;
    }

    if (group_state_ == STANDBY)
    {
        manual_traj_.joint_start = start_joint_;
        FST_INFO("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

        if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
        {
            if (manual_traj_.frame == JOINT)
            {
                for (size_t i = 0; i < getNumberOfJoint(); i++)
                {
                    if (manual_traj_.joint_start[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
                    {
                        FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).", i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                        return MC_FAIL_MANUAL_CONTINUOUS;
                    }
                    else if (manual_traj_.joint_start[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
                    {
                        FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).", i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                        return MC_FAIL_MANUAL_CONTINUOUS;
                    }
                }
            }
            else
            {
                FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
                return MC_FAIL_MANUAL_CONTINUOUS;
            }
        }

        PoseEuler fcp_in_base, tcp_in_base, tcp_in_user;

        switch (manual_traj_.frame)
        {
            case JOINT:
                break;
            case BASE:
                kinematics_ptr_->doFK(manual_traj_.joint_start, fcp_in_base);
                transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
                manual_traj_.cart_start = tcp_in_base;
                manual_traj_.cart_ending = tcp_in_base;
                //FST_INFO("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));
                //FST_INFO("FCP-in-base: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", fcp_in_base.point_.x_, fcp_in_base.point_.y_, fcp_in_base.point_.z_, fcp_in_base.euler_.a_, fcp_in_base.euler_.b_, fcp_in_base.euler_.c_);
                //FST_INFO("TCP-in-base: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tcp_in_base.point_.x_, tcp_in_base.point_.y_, tcp_in_base.point_.z_, tcp_in_base.euler_.a_, tcp_in_base.euler_.b_, tcp_in_base.euler_.c_);
                //FST_INFO("Tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame_.point_.x_, tool_frame_.point_.y_, tool_frame_.point_.z_, tool_frame_.euler_.a_, tool_frame_.euler_.b_, tool_frame_.euler_.c_);
                //transformation_.convertTcpToFcp(tcp_in_base, tool_frame_, fcp_in_base);
                //FST_INFO("FCP-in-base: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", fcp_in_base.point_.x_, fcp_in_base.point_.y_, fcp_in_base.point_.z_, fcp_in_base.euler_.a_, fcp_in_base.euler_.b_, fcp_in_base.euler_.c_);
                break;
            case USER:
                kinematics_ptr_->doFK(manual_traj_.joint_start, fcp_in_base);
                transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
                transformation_.convertPoseFromBaseToUser(tcp_in_base, user_frame_, tcp_in_user);
                manual_traj_.cart_start = tcp_in_user;
                manual_traj_.cart_ending = tcp_in_user;
                break;
            case WORLD:
                kinematics_ptr_->doFK(manual_traj_.joint_start, fcp_in_base);
                transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
                transformation_.convertPoseFromBaseToUser(tcp_in_base, world_frame_, tcp_in_user);
                manual_traj_.cart_start = tcp_in_user;
                manual_traj_.cart_ending = tcp_in_user;
                break;
            case TOOL:
                kinematics_ptr_->doFK(manual_traj_.joint_start, fcp_in_base);
                manual_traj_.tool_coordinate = fcp_in_base;
                memset(&manual_traj_.cart_start, 0, sizeof(manual_traj_.cart_start));
                memset(&manual_traj_.cart_ending, 0, sizeof(manual_traj_.cart_start));
                break;
            default:
                FST_ERROR("Unsupported manual frame: %d", manual_traj_.frame);
                return MC_INTERNAL_FAULT;
        }

        manual_time_ = 0;
        manual_traj_.mode = CONTINUOUS;
        pthread_mutex_lock(&manual_traj_mutex_);
        ErrorCode err = manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);
        pthread_mutex_unlock(&manual_traj_mutex_);

        if (err == SUCCESS)
        {
            FST_INFO("Manual move continous, total-duration = %.4f, Success.", manual_traj_.duration);
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
            ManualFrame frame = manual_traj_.frame;
            memset(&manual_traj_, 0, sizeof(ManualTrajectory));
            manual_traj_.frame = frame;
            return err;
        }
    }
    else if (group_state_ == MANUAL)
    {
        for (size_t i = 0; i < getNumberOfJoint(); i++)
        {
            if (manual_traj_.direction[i] != direction[i])
            {
                pthread_mutex_lock(&manual_traj_mutex_);
                ErrorCode err = manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);
                pthread_mutex_unlock(&manual_traj_mutex_);
                return err;
            }
            else
            {
                continue;
            }
        }

        FST_INFO("Given directions same as current motion, do not need replan.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Cannot manual continuous in current grp-state = %d, servo-state = %d", group_state_, servo_state_);
        return MC_FAIL_MANUAL_CONTINUOUS;
    }
}

ErrorCode BaseGroup::manualStop(void)
{
    FST_INFO("Manual stop, grp-state: %d, manual-mode: %d, manual-frame: %d", group_state_, manual_traj_.mode, manual_traj_.frame);

    if (group_state_ == MANUAL)
    {
        if (manual_traj_.mode == CONTINUOUS)
        {
            ManualDirection direction[NUM_OF_JOINT] = {STANDING};
            pthread_mutex_lock(&manual_traj_mutex_);
            ErrorCode err = manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);
            pthread_mutex_unlock(&manual_traj_mutex_);

            if (err == SUCCESS)
            {
                FST_INFO("Success, the group will stop in %.4fs", manual_traj_.duration - manual_time_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to stop current manual motion, error-code = 0x%llx", err);
                return err;
            }
        }
        else if (manual_traj_.mode == APOINT)
        {
            pthread_mutex_lock(&manual_traj_mutex_);
            ErrorCode err = manual_teach_.manualStop(manual_time_, manual_traj_);
            pthread_mutex_unlock(&manual_traj_mutex_);

            if (err == SUCCESS)
            {
                FST_INFO("Success, the group will stop in %.4fs", manual_traj_.duration - manual_time_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to stop current manual motion, error-code = 0x%llx", err);
                return err;
            }
        }
        else
        {
            FST_INFO("Cannot stop manual motion in step mode, the group will stop in %.4fs", manual_traj_.duration - manual_time_);
            return SUCCESS;
        }
    }
    else
    {
        FST_INFO("The group is not in manual state, current-state = %d", group_state_);
        return SUCCESS;
    }
}

void BaseGroup::linkCacheList(PathCacheList *path_ptr, TrajectoryCacheList *traj_ptr)
{
    path_ptr->next_ptr = NULL;
    traj_ptr->next_ptr = NULL;

    if (path_list_ptr_ == NULL)
    {
        path_list_ptr_ = path_ptr;
    }
    else
    {
        getLastPathCacheListPtr()->next_ptr = path_ptr;
    }
    
    if (traj_list_ptr_ == NULL)
    {
        traj_list_ptr_ = traj_ptr;
    }
    else
    {
        getLastTrajectoryCacheListPtr()->next_ptr = traj_ptr;
    }
}

ErrorCode BaseGroup::pauseMove(void)
{
    FST_INFO("Pause move request received.");

    if (group_state_ != AUTO)
    {
        FST_WARN("Group state is %d, pause request refused.", group_state_);
        return SUCCESS;
    }

    pthread_mutex_lock(&cache_list_mutex_);

    if (traj_list_ptr_ == NULL)
    {
        pthread_mutex_unlock(&cache_list_mutex_);
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
        pthread_mutex_unlock(&cache_list_mutex_);
        FST_INFO("Near ending point, do not need plan pause trajectory.");
        return SUCCESS;
    }
    else if (traj_cache.smooth_out_index != -1 && pause_index >= (int)traj_cache.smooth_out_index)
    {
        pthread_mutex_unlock(&cache_list_mutex_);
        FST_ERROR("Near smooth point, cannot pause right now.");
        return TRAJ_PLANNING_PAUSE_FAILED;
    }
    
    sampleBlockEnding(traj_cache.cache[pause_index], pause_state);
    TrajectoryCacheList *pause_traj_ptr = traj_cache_pool_.getCachePtr();

    if (pause_traj_ptr == NULL)
    {
        pthread_mutex_unlock(&cache_list_mutex_);
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
        pthread_mutex_unlock(&cache_list_mutex_);
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
    pthread_mutex_unlock(&cache_list_mutex_);
    FST_INFO("Pause trajectory planning success, going to pause at: %s", printDBLine(&pause_state.angle[0], buffer, LOG_TEXT_SIZE));

#ifdef OUTPUT_TRAJ_CACHE
    g_traj_output_array[g_traj_output_index] = *traj_ptr;
    g_traj_output_index = (g_traj_output_index + 1) % OUTPUT_TRAJ_CACHE_SIZE;
#endif

    return SUCCESS;
}

ErrorCode BaseGroup::restartMove(void)
{
    FST_INFO("Restart move request received.");

    if (group_state_ != PAUSE)
    {
        FST_WARN("Group state is %d, restart request refused.", group_state_);
        return SUCCESS;
    }

    pthread_mutex_lock(&cache_list_mutex_);
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
            pthread_mutex_unlock(&cache_list_mutex_);
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
            pthread_mutex_unlock(&cache_list_mutex_);
            traj_cache_pool_.freeCachePtr(traj_cache_ptr);
            FST_ERROR("Fail to plan trajectory back to pause-pose, code = 0x%llx", err);
            return MC_INTERNAL_FAULT;
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
    pthread_mutex_unlock(&cache_list_mutex_);
    return SUCCESS;
}

ErrorCode BaseGroup::replanPathCache(void)
{
    pthread_mutex_lock(&cache_list_mutex_);
    PathCacheList *path_ptr = path_list_ptr_;
    path_list_ptr_ = NULL;
    pthread_mutex_unlock(&cache_list_mutex_);

    PathCacheList *p = path_ptr;
    while (p) {FST_INFO("Path cahce: %p", p); p = p->next_ptr;}

    FST_INFO("Replan path cache: %p", path_ptr);
    PathCacheList *path_cache_ptr = path_cache_pool_.getCachePtr();
    TrajectoryCacheList *traj_cache_ptr = traj_cache_pool_.getCachePtr();

    if (path_cache_ptr == NULL || traj_cache_ptr == NULL)
    {
        path_list_ptr_ = path_ptr;
        pthread_mutex_unlock(&cache_list_mutex_);
        FST_ERROR("No path-cache (=%p) or traj-cache (=%p) available, set more caches in config file.", path_cache_ptr, traj_cache_ptr);
        path_cache_pool_.freeCachePtr(path_cache_ptr);
        traj_cache_pool_.freeCachePtr(traj_cache_ptr);
        return MC_NO_ENOUGH_CACHE;
    }
    
    char buffer[LOG_TEXT_SIZE];
    path_cache_ptr->id = path_ptr->id;
    path_cache_ptr->next_ptr = NULL;
    path_cache_ptr->path_cache.target = path_ptr->path_cache.target;
    path_cache_ptr->path_cache.smooth_in_index = -1;
    path_cache_ptr->path_cache.cache_length = path_ptr->path_cache.cache_length - pause_status_.pause_index;
    path_cache_ptr->path_cache.smooth_out_index = path_ptr->path_cache.smooth_out_index == -1 ? -1 : path_ptr->path_cache.smooth_out_index - pause_status_.pause_index;
    memcpy(path_cache_ptr->path_cache.cache, path_ptr->path_cache.cache + pause_status_.pause_index, path_cache_ptr->path_cache.cache_length * sizeof(PathBlock));

    FST_INFO("pause-index=%d", pause_status_.pause_index);
    FST_INFO("start-joint: %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
    //FST_INFO("target: %s", printDBLine(&path_cache_ptr->path_cache.target.joint_target.j1_, buffer, LOG_TEXT_SIZE));
    //FST_INFO("old-cache-length = %d, new-cache-length = %d", path_ptr->path_cache.cache_length, path_cache_ptr->path_cache.cache_length);
    //FST_INFO("old-last-joint: %s", printDBLine(&path_ptr->path_cache.cache[path_ptr->path_cache.cache_length - 1].joint.j1_, buffer, LOG_TEXT_SIZE));
    //FST_INFO("new-last-joint: %s", printDBLine(&path_cache_ptr->path_cache.cache[path_cache_ptr->path_cache.cache_length - 1].joint.j1_, buffer, LOG_TEXT_SIZE));

    JointState start_state;
    start_state.angle = start_joint_;
    memset(&start_state.omega, 0, sizeof(start_state.omega));
    memset(&start_state.alpha, 0, sizeof(start_state.alpha));
    
    FST_INFO("start-joint: %s", printDBLine(&start_state.angle.j1_, buffer, LOG_TEXT_SIZE));
    ErrorCode err = planTrajectory(path_cache_ptr->path_cache, start_state, vel_ratio_, acc_ratio_, traj_cache_ptr->trajectory_cache);

    if (err != SUCCESS)
    {
        pthread_mutex_unlock(&cache_list_mutex_);
        path_list_ptr_ = path_ptr;
        path_cache_pool_.freeCachePtr(path_cache_ptr);
        traj_cache_pool_.freeCachePtr(traj_cache_ptr);
        FST_ERROR("Trajectory plan failed, code = 0x%llx", err);
        return err;
    }
    
    traj_cache_ptr->pick_index = 0;
    traj_cache_ptr->pick_from_block = 0;
    traj_cache_ptr->time_from_start = 0;
    traj_cache_ptr->next_ptr = NULL;
    traj_cache_ptr->trajectory_cache.path_cache_ptr = &path_cache_ptr->path_cache;
    start_joint_ = path_cache_ptr->path_cache.cache[path_cache_ptr->path_cache.cache_length - 1].joint;

    FST_INFO("Traj-cache-length: %d", traj_cache_ptr->trajectory_cache.cache_length);
    FST_INFO("New-start-joint: %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("auto-time: %.4f", auto_time_);
   
    updateTimeFromStart(*traj_cache_ptr);
    linkCacheList(path_cache_ptr, traj_cache_ptr);
    pthread_mutex_unlock(&cache_list_mutex_);
    FST_INFO("Planning success.");
    
    PathCacheList *ptr = path_ptr;
    path_ptr = path_ptr->next_ptr;
    path_cache_pool_.freeCachePtr(ptr);

    while (path_ptr)
    {
        FST_INFO("Replan path cache: %p", path_ptr);
        int id = path_ptr->id;
        MotionInfo target = path_ptr->path_cache.target;
        ptr = path_ptr;
        path_ptr = path_ptr->next_ptr;
        path_cache_pool_.freeCachePtr(ptr);
        ErrorCode err = autoMove(id, target);

        if (err != SUCCESS)
        {
            FST_ERROR("Fail to replan path cache: %p, code = 0x%llx", path_ptr, err);

            while (path_ptr)
            {
                ptr = path_ptr;
                path_ptr = path_ptr->next_ptr;
                path_cache_pool_.freeCachePtr(ptr);
            }

            return err;
        }
    }
    
    FST_INFO("All path cache replanned successfully.");
    return SUCCESS;
}

ErrorCode BaseGroup::autoMove(int id, const MotionInfo &info)
{
    FST_INFO("Auto move request received, ID = %d, type = %d", id, info.type);
    Joint start = start_joint_;

    ErrorCode err = checkStartState(start);

    if (err != SUCCESS)
    {
        FST_ERROR("Start state check failed, code = 0x%llx", err);
        return err;
    }

    err = checkMotionTarget(info);

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

    auto *path_ptr = path_cache_pool_.getCachePtr();
    auto *traj_ptr = traj_cache_pool_.getCachePtr();
    
    if (path_ptr == NULL || traj_ptr == NULL)
    {
        FST_ERROR("No path-cache (=%p) or traj-cache (=%p) available, set more caches in config file.", path_ptr, traj_ptr);
        path_cache_pool_.freeCachePtr(path_ptr);
        traj_cache_pool_.freeCachePtr(traj_ptr);
        return MC_NO_ENOUGH_CACHE;
    }

    if (info.type == MOTION_JOINT)
    {
        err = autoJoint(start, info, *path_ptr, *traj_ptr);
    }
    else if (info.type == MOTION_LINE)
    {
        err = autoLine(start, info, *path_ptr, *traj_ptr);
    }
    else if (info.type == MOTION_CIRCLE)
    {
        err = autoCircle(start, info, *path_ptr, *traj_ptr);
    }
    else
    {
        FST_ERROR("Invalid motion type = %d, autoMove aborted.", info.type);
        err = INVALID_PARAMETER;
    }

    if (err == SUCCESS)
    {
        path_ptr->id = id;
        traj_ptr->pick_index = 0;
        traj_ptr->pick_from_block = 0;
        traj_ptr->trajectory_cache.path_cache_ptr = &path_ptr->path_cache;

        // 上一条指令不带平滑时traj_ptr->time_from_start应该是0，否则它应该等于上一条指令的平滑切出时间
        MotionTime start_time = 0;
        pthread_mutex_lock(&cache_list_mutex_);
        auto *last_traj_ptr = getLastTrajectoryCacheListPtr();

        if (last_traj_ptr != NULL && last_traj_ptr->trajectory_cache.smooth_out_index != -1)
        {
            auto &smooth_out_block = last_traj_ptr->trajectory_cache.cache[last_traj_ptr->trajectory_cache.smooth_out_index];
            start_time = smooth_out_block.time_from_start + smooth_out_block.duration;
        }

        if (fabs(start_time - traj_ptr->time_from_start) < MINIMUM_E6)
        {
            // 更新start-joint
            start_joint_ = path_ptr->path_cache.cache[path_ptr->path_cache.cache_length - 1].joint;
            updateTimeFromStart(*traj_ptr);
            linkCacheList(path_ptr, traj_ptr);
            pthread_mutex_unlock(&cache_list_mutex_);

#ifdef OUTPUT_PATH_CACHE
            g_path_output_array[g_path_output_index] = *path_ptr;
            g_path_output_index = (g_path_output_index + 1) % OUTPUT_PATH_CACHE_SIZE;
#endif
#ifdef OUTPUT_TRAJ_CACHE
            g_traj_output_array[g_traj_output_index] = *traj_ptr;
            g_traj_output_index = (g_traj_output_index + 1) % OUTPUT_TRAJ_CACHE_SIZE;
#endif
            // ------- for test only -------- //
            // for (size_t i = 0; i < path_ptr->path_cache.cache_length; i++)
            // {
            //     g_path_point[g_path_index] = path_ptr->path_cache.cache[i].pose;
            //     g_path_index = (g_path_index + 1) % 10000;
            // }
            // ------- for test only -------- //

            FST_INFO("Planning success.");
            return SUCCESS;
        }
        else
        {
            pthread_mutex_unlock(&cache_list_mutex_);
            FST_ERROR("Start-time of this motion not equal with smooth-out-time of last motion.");
            path_cache_pool_.freeCachePtr(path_ptr);
            traj_cache_pool_.freeCachePtr(traj_ptr);
            return MC_INTERNAL_FAULT;
        }
    }
    else
    {
        FST_ERROR("Planning failed with code = 0x%llx, autoMove aborted.", err);
        path_cache_pool_.freeCachePtr(path_ptr);
        traj_cache_pool_.freeCachePtr(traj_ptr);
        return err;
    }
}


ErrorCode BaseGroup::checkStartState(const Joint &start_joint)
{
    if (group_state_ == STANDBY && servo_state_ == SERVO_IDLE && traj_list_ptr_ == NULL && path_list_ptr_ == NULL)
    {
        Joint control_joint, current_joint;
        getLatestJoint(current_joint);

        if (bare_core_.getControlPosition(&control_joint[0], getNumberOfJoint()))
        {
            if (!isSameJoint(current_joint, control_joint, joint_tracking_accuracy_))
            {
                char buffer[LOG_TEXT_SIZE];
                FST_ERROR("Control-position different with current-position, it might be a trouble.");
                FST_ERROR("control-position: %s", printDBLine(&control_joint[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("current-position: %s", printDBLine(&current_joint[0], buffer, LOG_TEXT_SIZE));
                return MC_INTERNAL_FAULT;
            }

            if (!isSameJoint(start_joint, control_joint, joint_tracking_accuracy_))
            {
                char buffer[LOG_TEXT_SIZE];
                FST_ERROR("Control-position different with start-position, it might be a trouble.");
                FST_ERROR("control-position: %s", printDBLine(&control_joint[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("start-position:   %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));
                return MC_INTERNAL_FAULT;
            }
        }
        else
        {
            FST_ERROR("Cannot get control position from bare core.");
            return BARE_CORE_TIMEOUT;
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
    
    // CNT ∈ [0, 1] U CNT = -1
    if (fabs(info.cnt + 1) > MINIMUM_E9 && (info.cnt < -MINIMUM_E9 || info.cnt > 1 + MINIMUM_E9))
    {
        FST_ERROR("Invalid CNT: %.12f", info.cnt);
        return INVALID_PARAMETER;
    }

    if (  ((info.type == MOTION_JOINT) && (info.vel < MINIMUM_E6 || info.vel > 1 + MINIMUM_E6)) ||
          ((info.type == MOTION_LINE || info.type == MOTION_CIRCLE) && (info.vel < cartesian_vel_min_ || info.vel > cartesian_vel_max_))  )
    {
        FST_ERROR("Invalid vel: %.6f", info.vel);
        return INVALID_PARAMETER;
    }

    Joint target_joint;
    PoseEuler start_pose;
    PoseEuler fcp_in_base, tcp_in_base;
    kinematics_ptr_->doFK(start_joint_, fcp_in_base);
    transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
    transformation_.convertPoseFromBaseToUser(tcp_in_base, user_frame_, start_pose);

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

    if (info.type == MOTION_CIRCLE)
    {
        if (!soft_constraint_.isJointInConstraint(info.via.joint))
        {
            char buffer[LOG_TEXT_SIZE];
            const Posture &posture = info.via.pose.posture;
            const PoseEuler &pose = info.via.pose.pose;
            const PoseEuler &uf = info.via.user_frame;
            const PoseEuler &tf = info.via.tool_frame;
            FST_ERROR("Vis joint out of soft constraint.");
            FST_ERROR("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            FST_ERROR("Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            FST_ERROR("Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            FST_ERROR("User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            FST_ERROR("Joint = %s", printDBLine(&info.via.joint[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
            return JOINT_OUT_OF_CONSTRAINT;
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

bool BaseGroup::isLastMotionSmooth(void)
{
    // 上一条指令未走完且上一条指令的CNT介于0 ~ 1之间
    TrajectoryCache *traj_ptr = getLastTrajectoryCachePtr();
    return traj_ptr != NULL && traj_ptr->path_cache_ptr->target.cnt > -MINIMUM_E3;
}

PathCache* BaseGroup::getLastPathCachePtr(void)
{
    PathCacheList *p = getLastPathCacheListPtr();
    return p != NULL ? &p->path_cache : NULL;
}

PathCacheList* BaseGroup::getLastPathCacheListPtr(void)
{
    if (path_list_ptr_ != NULL)
    {
        PathCacheList *p = path_list_ptr_;

        while (p->next_ptr != NULL)
        {
            p = p->next_ptr;
        }

        return p;
    }
    else
    {
        return NULL;
    }
}

TrajectoryCache* BaseGroup::getLastTrajectoryCachePtr(void)
{
    TrajectoryCacheList *p = getLastTrajectoryCacheListPtr();
    return p != NULL ? &p->trajectory_cache : NULL;
}

TrajectoryCacheList* BaseGroup::getLastTrajectoryCacheListPtr(void)
{
    if (traj_list_ptr_ != NULL)
    {
        TrajectoryCacheList *p = traj_list_ptr_;

        while (p->next_ptr != NULL)
        {
            p = p->next_ptr;
        }

        return p;
    }
    else
    {
        return NULL;
    }
}

bool BaseGroup::checkPath(const PathCache &path)
{
    if (path.cache_length > PATH_CACHE_SIZE)
    {
        FST_ERROR("Cache-length = %d", path.cache_length);
        return false;
    }

    if (path.target.cnt < 0 && path.smooth_out_index != -1)
    {
        FST_ERROR("CNT = %.6f, smooth-out = %d", path.target.cnt, path.smooth_out_index);
        return false;
    }

    if (path.smooth_out_index < -1 || path.smooth_out_index >= (int)path.cache_length)
    {
        FST_ERROR("Cache-length = %d, smooth-out = %d", path.cache_length, path.smooth_out_index);
        return false;
    }

    if (path.smooth_in_index < -1 || path.smooth_in_index >= (int)path.cache_length)
    {
        FST_ERROR("Cache-length = %d, smooth-in = %d", path.cache_length, path.smooth_in_index);
        return false;
    }

    return true;
}

bool BaseGroup::checkTrajectory(const TrajectoryCache &trajectory)
{
    if (trajectory.cache_length > TRAJECTORY_CACHE_SIZE)
    {
        FST_ERROR("Cache-length = %d", trajectory.cache_length);
        return false;
    }

    if (trajectory.smooth_out_index < -1 || trajectory.smooth_out_index >= (int)trajectory.cache_length)
    {
        FST_ERROR("Cache-length = %d, smooth-out = %d", trajectory.cache_length, trajectory.smooth_out_index);
        return false;
    }

    for (size_t i = 0; i < trajectory.cache_length; i++)
    {
        if (trajectory.cache[i].duration <= 0 || trajectory.cache[i].duration > 50)
        {
            FST_ERROR("trajectory-block %d: duration = %.6f", i, trajectory.cache[i].duration);
            return false;
        }
    }

    return true;
}

ErrorCode BaseGroup::autoJoint(const Joint &start, const MotionInfo &info, PathCacheList &path, TrajectoryCacheList &trajectory)
{
    MotionTime start_time = 0;
    pthread_mutex_lock(&cache_list_mutex_);

    if (isLastMotionSmooth())
    {
        // 获取上一条轨迹的切出时间，作为本条指令的起始时间
        auto *last_traj_ptr = getLastTrajectoryCacheListPtr();

        if (last_traj_ptr->trajectory_cache.smooth_out_index != -1)
        {
            auto &smooth_out_block = last_traj_ptr->trajectory_cache.cache[last_traj_ptr->trajectory_cache.smooth_out_index];
            start_time = smooth_out_block.time_from_start + smooth_out_block.duration;
        }

        // 获取上一条轨迹切出点的位置、速度、加速度
        JointState start_state;
        TrajectoryCache &traj_cache = last_traj_ptr->trajectory_cache;
        sampleBlockEnding(traj_cache.cache[traj_cache.smooth_out_index], start_state);
        auto last_cnt = traj_cache.path_cache_ptr->target.cnt;
        pthread_mutex_unlock(&cache_list_mutex_);
        trajectory.time_from_start = start_time;
        FST_INFO("Last motion with smooth, cnt = %.4f, time-of-smooth", last_cnt, trajectory.time_from_start);
        return autoSmoothJoint(start_state, traj_cache.path_cache_ptr->target, info, path.path_cache, trajectory.trajectory_cache);
    }
    else
    {
        pthread_mutex_unlock(&cache_list_mutex_);
        trajectory.time_from_start = start_time;
        FST_INFO("Last motion without smooth, start from stable");
        return autoStableJoint(start, info, path.path_cache, trajectory.trajectory_cache);
    }
}

ErrorCode BaseGroup::autoStableJoint(const Joint &start, const MotionInfo &info, PathCache &path, TrajectoryCache &trajectory)
{
    char buffer[LOG_TEXT_SIZE];
    clock_t start_clock, end_clock;
    double  path_plan_time, traj_plan_time;
    const PoseEuler &pose = info.target.pose.pose;
    const Posture &posture = info.target.pose.posture;
    FST_INFO("autoStableJoint: vel = %.4f, cnt = %.4f", info.vel, info.cnt);
    FST_INFO("start-joint: %s", printDBLine(&start[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("target-joint: %s", printDBLine(&info.target.joint.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("target-pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f,", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
    FST_INFO("target-posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);

    start_clock = clock();
    ErrorCode err = planPathJoint(start, info, path);
    end_clock = clock();
    path_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;
    path.target = info;

    if (err != SUCCESS)
    {
        FST_ERROR("Path plan failed, code = 0x%llx", err);
        return err;
    }

    if (!checkPath(path))
    {
        FST_ERROR("Data in path cache is invalid.");
        return MC_INTERNAL_FAULT;
    }
    
    FST_INFO("Path plan success, total-point = %d, smooth-in = %d, smooth-out = %d", path.cache_length, path.smooth_in_index, path.smooth_out_index);

    JointState start_state;
    start_state.angle = start;
    memset(&start_state.omega, 0, sizeof(start_state.omega));
    memset(&start_state.alpha, 0, sizeof(start_state.alpha));
    start_clock = clock();
    err = planTrajectory(path, start_state, vel_ratio_, acc_ratio_, trajectory);
    end_clock = clock();
    traj_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err != SUCCESS)
    {
        FST_ERROR("Trajectory plan failed, code = 0x%llx", err);
        return err;
    }

    if (!checkTrajectory(trajectory))
    {
        FST_ERROR("Data in trajectory cache is invalid.");
        return MC_INTERNAL_FAULT;
    }

    FST_INFO("Trajectory plan success, total-segment = %d, smooth-out = %d,", trajectory.cache_length, trajectory.smooth_out_index);
    FST_INFO("autoStableJoint success, path-plan: %.4f ms, traj-plan: %.4f ms", path_plan_time, traj_plan_time);
    return SUCCESS;
}

ErrorCode BaseGroup::autoSmoothJoint(const JointState &start_state, const MotionInfo &via, const MotionInfo &target, PathCache &path, TrajectoryCache &trajectory)
{
    char buffer[LOG_TEXT_SIZE];
    const PoseEuler &pose = target.target.pose.pose;
    const Posture &posture = target.target.pose.posture;
    FST_INFO("autoSmoothJoint: vel = %.4f, cnt = %.4f", target.vel, target.cnt);
    FST_INFO("Start-angle: %s", printDBLine(&start_state.angle[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-omega: %s", printDBLine(&start_state.omega[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-alpha: %s", printDBLine(&start_state.alpha[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Via-joint: %s", printDBLine(&via.target.joint[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Target-joint = %s", printDBLine(&target.target.joint[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("target-pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f,", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
    FST_INFO("target-posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);

    ErrorCode err = planPathSmoothJoint(start_state.angle, via, target, path);
    path.target = target;

    if (err != SUCCESS)
    {
        FST_ERROR("Path plan failed, code = 0x%llx", err);
        return err;
    }

    if (!checkPath(path))
    {
        FST_ERROR("Data in path cache is invalid.");
        return MC_INTERNAL_FAULT;
    }

    FST_INFO("Path plan success, total-point = %d, smooth-in = %d, smooth-out = %d",path.cache_length, path.smooth_in_index, path.smooth_out_index);
    err = planTrajectorySmooth(path, start_state, via, vel_ratio_, acc_ratio_, trajectory);

    if (err != SUCCESS)
    {
        FST_ERROR("Trajectory plan failed, code = 0x%llx", err);
        return err;
    }

    if (!checkTrajectory(trajectory))
    {
        FST_ERROR("Data in trajectory cache is invalid.");
        return MC_INTERNAL_FAULT;
    }

    FST_INFO("Trajectory plan success, total-segment = %d, smooth-out = %d,", trajectory.cache_length, trajectory.smooth_out_index);
    FST_INFO("autoSmoothJoint success.");
    return SUCCESS;
}

void BaseGroup::sampleBlockStart(const TrajectoryBlock &block, JointState &state)
{
    const double *data;
    size_t joint_num = getNumberOfJoint();

    for (size_t i = 0; i < joint_num; i++)
    {
        data = block.axis[i].data;
        state.angle[i] = data[0];
        state.omega[i] = data[1];
        state.alpha[i] = data[2] * 2;
    }
}

void BaseGroup::sampleBlockEnding(const TrajectoryBlock &block, JointState &state)
{
    const double *data;
    size_t joint_num = getNumberOfJoint();
    MotionTime tm[6];
    tm[0] = 1;
    tm[1] = block.duration;
    tm[2] = block.duration * tm[1];
    tm[3] = block.duration * tm[2];
    tm[4] = block.duration * tm[3];
    tm[5] = block.duration * tm[4];
    
    for (size_t i = 0; i < joint_num; i++)
    {
        data = block.axis[i].data;
        state.angle[i] = data[0] + data[1] * tm[1] + data[2] * tm[2] + data[3] * tm[3] + data[4] * tm[4] + data[5] * tm[5];
        state.omega[i] = data[1] + data[2] * tm[1] * 2 + data[3] * tm[2] * 3 + data[4] * tm[3] * 4 + data[5] * tm[4] * 5;
        state.alpha[i] = data[2] * 2 + data[3] * tm[1] * 6 + data[4] * tm[2] * 12 + data[5] * tm[3] * 20;
    }
}

void BaseGroup::sampleBlock(const TrajectoryBlock &block, MotionTime time_from_block, JointState &state)
{
    const double *data;
    size_t joint_num = getNumberOfJoint();
    MotionTime tm[6];
    tm[0] = 1;
    tm[1] = time_from_block;
    tm[2] = time_from_block * tm[1];
    tm[3] = time_from_block * tm[2];
    tm[4] = time_from_block * tm[3];
    tm[5] = time_from_block * tm[4];
    
    for (size_t i = 0; i < joint_num; i++)
    {
        data = block.axis[i].data;
        state.angle[i] = data[0] + data[1] * tm[1] + data[2] * tm[2] + data[3] * tm[3] + data[4] * tm[4] + data[5] * tm[5];
        state.omega[i] = data[1] + data[2] * tm[1] * 2 + data[3] * tm[2] * 3 + data[4] * tm[3] * 4 + data[5] * tm[4] * 5;
        state.alpha[i] = data[2] * 2 + data[3] * tm[1] * 6 + data[4] * tm[2] * 12 + data[5] * tm[3] * 20;
    }
}

ErrorCode BaseGroup::computeInverseKinematicsOnPathCache(const Joint &start, PathCache &path)
{
    ErrorCode err = SUCCESS;
    Joint reference = start;
    PoseQuaternion tcp_in_base, fcp_in_base;
    Posture posture = path.target.target.pose.posture;

    for (size_t i = 0; i < path.cache_length; i++)
    {
        transformation_.convertPoseFromUserToBase(path.cache[i].pose, user_frame_, tcp_in_base);
        transformation_.convertTcpToFcp(tcp_in_base, tool_frame_, fcp_in_base);
        err = kinematics_ptr_->doIK(fcp_in_base, posture, path.cache[i].joint) ? SUCCESS : MC_COMPUTE_IK_FAIL;

        if (err != SUCCESS)
        {
            char buffer[LOG_TEXT_SIZE];
            PoseQuaternion &pose = path.cache[i].pose;
            PoseEuler tcp_user = Pose2PoseEuler(pose);
            PoseEuler fcp_base = Pose2PoseEuler(fcp_in_base);
            FST_ERROR("IK failed, code = 0x%llx", err);
            FST_ERROR("  pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.quaternion_.w_, pose.quaternion_.x_, pose.quaternion_.y_, pose.quaternion_.z_);
            FST_ERROR("  user-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", user_frame_.point_.x_, user_frame_.point_.y_, user_frame_.point_.z_, user_frame_.euler_.a_, user_frame_.euler_.b_, user_frame_.euler_.c_);
            FST_ERROR("  tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame_.point_.x_, tool_frame_.point_.y_, tool_frame_.point_.z_, tool_frame_.euler_.a_, tool_frame_.euler_.b_, tool_frame_.euler_.c_);
            FST_ERROR("  tcp-in-user: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tcp_user.point_.x_, tcp_user.point_.y_, tcp_user.point_.z_, tcp_user.euler_.a_, tcp_user.euler_.b_, tcp_user.euler_.c_);
            FST_ERROR("  fcp-in-base: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", fcp_base.point_.x_, fcp_base.point_.y_, fcp_base.point_.z_, fcp_base.euler_.a_, fcp_base.euler_.b_, fcp_base.euler_.c_);
            //FST_ERROR("  reference: %s", printDBLine(&reference[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("  posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            break;
        }

        if (!soft_constraint_.isJointInConstraint(path.cache[i].joint))
        {
            char buffer[LOG_TEXT_SIZE];
            PoseQuaternion &pose = path.cache[i].pose;
            PoseEuler fcp_base = Pose2PoseEuler(fcp_in_base);
            PoseEuler tcp_user = Pose2PoseEuler(pose);
            FST_ERROR("IK result out of soft constraint:");
            FST_ERROR("  pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.quaternion_.w_, pose.quaternion_.x_, pose.quaternion_.y_, pose.quaternion_.z_);
            FST_ERROR("  user-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", user_frame_.point_.x_, user_frame_.point_.y_, user_frame_.point_.z_, user_frame_.euler_.a_, user_frame_.euler_.b_, user_frame_.euler_.c_);
            FST_ERROR("  tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame_.point_.x_, tool_frame_.point_.y_, tool_frame_.point_.z_, tool_frame_.euler_.a_, tool_frame_.euler_.b_, tool_frame_.euler_.c_);
            FST_ERROR("  tcp-in-user: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tcp_user.point_.x_, tcp_user.point_.y_, tcp_user.point_.z_, tcp_user.euler_.a_, tcp_user.euler_.b_, tcp_user.euler_.c_);
            FST_ERROR("  fcp-in-base: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", fcp_base.point_.x_, fcp_base.point_.y_, fcp_base.point_.z_, fcp_base.euler_.a_, fcp_base.euler_.b_, fcp_base.euler_.c_);
            //FST_ERROR("  reference: %s", printDBLine(&reference[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("  posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            FST_ERROR("  joint: %s", printDBLine(&path.cache[i].joint[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("  upper: %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("  lower: %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
            err = JOINT_OUT_OF_CONSTRAINT;
            break;
        }
    }

    return err;
}

ErrorCode BaseGroup::autoLine(const Joint &start, const MotionInfo &info, PathCacheList &path, TrajectoryCacheList &trajectory)
{
    MotionTime start_time = 0;
    pthread_mutex_lock(&cache_list_mutex_);

    if (isLastMotionSmooth())
    {
        // 获取上一条轨迹的切出时间，作为本条指令的起始时间
        auto *last_traj_ptr = getLastTrajectoryCacheListPtr();

        if (last_traj_ptr->trajectory_cache.smooth_out_index != -1)
        {
            auto &smooth_out_block = last_traj_ptr->trajectory_cache.cache[last_traj_ptr->trajectory_cache.smooth_out_index];
            start_time = smooth_out_block.time_from_start + smooth_out_block.duration;
        }

        // 获取上一条轨迹切出点的位置、速度、加速度
        JointState start_state;
        TrajectoryCache &traj_cache = last_traj_ptr->trajectory_cache;
        sampleBlockEnding(traj_cache.cache[traj_cache.smooth_out_index], start_state);
        auto last_cnt = traj_cache.path_cache_ptr->target.cnt;
        pthread_mutex_unlock(&cache_list_mutex_);
        trajectory.time_from_start = start_time;
        FST_INFO("Last motion with smooth, cnt = %.4f, time-of-smooth", last_cnt, start_time);
        return autoSmoothLine(start_state, traj_cache.path_cache_ptr->target, info, path.path_cache, trajectory.trajectory_cache);
    }
    else
    {
        pthread_mutex_unlock(&cache_list_mutex_);
        trajectory.time_from_start = start_time;
        FST_INFO("Last motion without smooth, start from stable");
        return autoStableLine(start, info, path.path_cache, trajectory.trajectory_cache);
    }
}

ErrorCode BaseGroup::autoStableLine(const Joint &start, const MotionInfo &info, PathCache &path, TrajectoryCache &trajectory)
{
    char buffer[LOG_TEXT_SIZE];
    clock_t start_clock, end_clock;
    double  path_plan_time, path_ik_time, traj_plan_time;
    PoseEuler fcp_in_base, tcp_in_base, start_pose;
    kinematics_ptr_->doFK(start, fcp_in_base);
    transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
    transformation_.convertPoseFromBaseToUser(tcp_in_base, user_frame_, start_pose);

    const Posture &posture = info.target.pose.posture;
    const PoseEuler &target_pose = info.target.pose.pose;
    FST_INFO("autoStableLine: vel = %.4f, cnt = %.4f", info.vel, info.cnt);
    FST_INFO("Start-joint: %s", printDBLine(&start[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
    FST_INFO("Target-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", target_pose.point_.x_, target_pose.point_.y_,target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
    FST_INFO("Target-posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
    FST_INFO("Target-joint: %s", printDBLine(&info.target.joint[0], buffer, LOG_TEXT_SIZE));

    start_clock = clock();
    ErrorCode err = planPathLine(start_pose, info, path);
    end_clock = clock();
    path_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;
    path.target = info;

    if (err != SUCCESS)
    {
        FST_ERROR("Path plan failed, code = 0x%llx", err);
        return err;
    }

    if (!checkPath(path))
    {
        FST_ERROR("Data in path cache is invalid.");
        return MC_INTERNAL_FAULT;
    }
    
    FST_INFO("Path plan success, total-point = %d, smooth-in = %d, smooth-out = %d, inverse kinematics ...", path.cache_length, path.smooth_in_index, path.smooth_out_index);

    start_clock = clock();
    err = computeInverseKinematicsOnPathCache(start, path);
    end_clock = clock();
    path_ik_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err != SUCCESS)
    {
        FST_ERROR("Inverse kinematics failed, code = 0x%llx", err);
        return err;
    }
    
    FST_INFO("Inverse kinematics success, plan trajectory ...");

    JointState start_state;
    start_state.angle = start;
    memset(&start_state.omega, 0, sizeof(start_state.omega));
    memset(&start_state.alpha, 0, sizeof(start_state.alpha));
    start_clock = clock();
    err = planTrajectory(path, start_state, vel_ratio_, acc_ratio_, trajectory);
    end_clock = clock();
    traj_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err != SUCCESS)
    {
        FST_ERROR("Trajectory plan failed, code = 0x%llx", err);
        return err;
    }

    if (!checkTrajectory(trajectory))
    {
        FST_ERROR("Data in trajectory cache is invalid.");
        return MC_INTERNAL_FAULT;
    }
    
    FST_INFO("Trajectory plan success, total-segment = %d, smooth-out = %d,", trajectory.cache_length, trajectory.smooth_out_index);
    FST_INFO("autoStableLine success, path-plan: %.4f ms, path-ik: %.4f ms, traj-plan: %.4f ms", path_plan_time, path_ik_time, traj_plan_time);
    return SUCCESS;
}

ErrorCode BaseGroup::autoSmoothLine(const JointState &start_state, const MotionInfo &via, const MotionInfo &info, PathCache &path, TrajectoryCache &trajectory)
{
    char buffer[LOG_TEXT_SIZE];
    clock_t start_clock, end_clock;
    double  path_plan_time, path_ik_time, traj_plan_time;
    PoseEuler fcp_in_base, tcp_in_base, start_pose;
    kinematics_ptr_->doFK(start_state.angle, fcp_in_base);
    transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
    transformation_.convertPoseFromBaseToUser(tcp_in_base, user_frame_, start_pose);
    const PoseEuler &via_pose = via.target.pose.pose;
    const PoseEuler &target_pose = info.target.pose.pose;
    FST_INFO("autoSmoothLine: vel = %.4f, cnt = %.4f", info.vel, info.cnt);
    FST_INFO("Start-joint: %s", printDBLine(&start_state.angle[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-omega: %s", printDBLine(&start_state.omega[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-alpha: %s", printDBLine(&start_state.alpha[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
    FST_INFO("Via-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", via_pose.point_.x_, via_pose.point_.y_, via_pose.point_.z_, via_pose.euler_.a_, via_pose.euler_.b_, via_pose.euler_.c_);
    FST_INFO("Target-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
    FST_INFO("Via-joint: %s", printDBLine(&via.target.joint[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Target-joint: %s", printDBLine(&info.target.joint[0], buffer, LOG_TEXT_SIZE));

    start_clock = clock();
    ErrorCode err = planPathSmoothLine(start_pose, via, info, path);
    end_clock = clock();
    path_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;
    path.target = info;

    if (err != SUCCESS)
    {
        FST_ERROR("Path plan failed, code = 0x%llx", err);
        return err;
    }

    if (!checkPath(path))
    {
        FST_ERROR("Data in path cache is invalid.");
        return MC_INTERNAL_FAULT;
    }

    FST_INFO("Path plan success, total-point = %d, smooth-in = %d, smooth-out = %d, inverse kinematics ...", path.cache_length, path.smooth_in_index, path.smooth_out_index);

    start_clock = clock();
    err = computeInverseKinematicsOnPathCache(start_state.angle, path);
    end_clock = clock();
    path_ik_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err != SUCCESS)
    {
        FST_ERROR("Inverse kinematics failed, code = 0x%llx", err);
        return err;
    }
    
    FST_INFO("Inverse kinematics success, plan trajectory ...");

    start_clock = clock();
    err = planTrajectorySmooth(path, start_state, via, vel_ratio_, acc_ratio_, trajectory);
    end_clock = clock();
    traj_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err != SUCCESS)
    {
        FST_ERROR("Trajectory plan failed, code = 0x%llx", err);
        return err;
    }

    if (!checkTrajectory(trajectory))
    {
        FST_ERROR("Data in trajectory cache is invalid.");
        return MC_INTERNAL_FAULT;
    }
    
    FST_INFO("Trajectory plan success, total-segment = %d, smooth-out = %d,", trajectory.cache_length, trajectory.smooth_out_index);
    FST_INFO("autoSmoothLine success, path-plan: %.4f ms, path-ik: %.4f ms, traj-plan: %.4f ms", path_plan_time, path_ik_time, traj_plan_time);
    return SUCCESS;
}

ErrorCode BaseGroup::autoCircle(const Joint &start, const MotionInfo &info, PathCacheList &path, TrajectoryCacheList &trajectory)
{
    MotionTime start_time = 0;
    pthread_mutex_lock(&cache_list_mutex_);

    if (isLastMotionSmooth())
    {
        // 获取上一条轨迹的切出时间，作为本条指令的起始时间
        auto *last_traj_ptr = getLastTrajectoryCacheListPtr();

        if (last_traj_ptr->trajectory_cache.smooth_out_index != -1)
        {
            auto &smooth_out_block = last_traj_ptr->trajectory_cache.cache[last_traj_ptr->trajectory_cache.smooth_out_index];
            start_time = smooth_out_block.time_from_start + smooth_out_block.duration;
        }

        // 获取上一条轨迹切出点的位置、速度、加速度
        JointState start_state;
        TrajectoryCache &traj_cache = last_traj_ptr->trajectory_cache;
        sampleBlockEnding(traj_cache.cache[traj_cache.smooth_out_index], start_state);
        auto last_cnt = traj_cache.path_cache_ptr->target.cnt;
        pthread_mutex_unlock(&cache_list_mutex_);
        trajectory.time_from_start = start_time;
        FST_INFO("Last motion with smooth, cnt = %.4f, time-of-smooth", last_cnt, start_time);
        return autoSmoothCircle(start_state, traj_cache.path_cache_ptr->target, info, path.path_cache, trajectory.trajectory_cache);
    }
    else
    {
        pthread_mutex_unlock(&cache_list_mutex_);
        trajectory.time_from_start = start_time;
        FST_INFO("Last motion without smooth, start from stable");
        return autoStableCircle(start, info, path.path_cache, trajectory.trajectory_cache);
    }
}


ErrorCode BaseGroup::autoStableCircle(const Joint &start, const MotionInfo &info, PathCache &path, TrajectoryCache &trajectory)
{
    char buffer[LOG_TEXT_SIZE];
    clock_t start_clock, end_clock;
    double  path_plan_time, path_ik_time, traj_plan_time;
    PoseEuler fcp_in_base, tcp_in_base, start_pose;
    kinematics_ptr_->doFK(start, fcp_in_base);
    transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
    transformation_.convertPoseFromBaseToUser(tcp_in_base, user_frame_, start_pose);

    const PoseEuler &via_pose = info.via.pose.pose;
    const PoseEuler &target_pose = info.target.pose.pose;
    FST_INFO("autoStableCircle: vel = %.4f, cnt = %.4f", info.vel, info.cnt);
    FST_INFO("Start-joint: %s", printDBLine(&start[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
    FST_INFO("Via-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", via_pose.point_.x_, via_pose.point_.y_, via_pose.point_.z_, via_pose.euler_.a_, via_pose.euler_.b_, via_pose.euler_.c_);
    FST_INFO("Target-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
    FST_INFO("Via-joint: %s", printDBLine(&info.via.joint[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Target-joint: %s", printDBLine(&info.target.joint[0], buffer, LOG_TEXT_SIZE));

    start_clock = clock();
    ErrorCode err = planPathCircle(start_pose, info, path);
    end_clock = clock();
    path_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;
    path.target = info;

    if (err != SUCCESS)
    {
        FST_ERROR("Path plan failed, code = 0x%llx", err);
        return err;
    }

    if (!checkPath(path))
    {
        FST_ERROR("Data in path cache is invalid.");
        return MC_INTERNAL_FAULT;
    }

    FST_INFO("Path plan success, total-point = %d, smooth-in = %d, smooth-out = %d, inverse kinematics ...", path.cache_length, path.smooth_in_index, path.smooth_out_index);

    start_clock = clock();
    err = computeInverseKinematicsOnPathCache(start, path);
    end_clock = clock();
    path_ik_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err != SUCCESS)
    {
        FST_ERROR("Inverse kinematics failed, code = 0x%llx", err);
        return err;
    }

    if (!checkPath(path))
    {
        FST_ERROR("Data in path cache is invalid.");
        return MC_INTERNAL_FAULT;
    }

    FST_INFO("Inverse kinematics success, plan trajectory ...");

    JointState start_state;
    start_state.angle = start;
    memset(&start_state.omega, 0, sizeof(start_state.omega));
    memset(&start_state.alpha, 0, sizeof(start_state.alpha));
    start_clock = clock();
    err = planTrajectory(path, start_state, vel_ratio_, acc_ratio_, trajectory);
    end_clock = clock();
    traj_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err != SUCCESS)
    {
        FST_ERROR("Trajectory plan failed, code = 0x%llx", err);
        return err;
    }

    if (!checkTrajectory(trajectory))
    {
        FST_ERROR("Data in trajectory cache is invalid.");
        return MC_INTERNAL_FAULT;
    }
        
    FST_INFO("Trajectory plan success, total-segment = %d, smooth-out = %d,", trajectory.cache_length, trajectory.smooth_out_index);
    FST_INFO("autoStableCircle success, path-plan: %.4f ms, path-ik: %.4f ms, traj-plan: %.4f ms", path_plan_time, path_ik_time, traj_plan_time);
    return SUCCESS;
}


ErrorCode BaseGroup::autoSmoothCircle(const JointState &start_state, const MotionInfo &via, const MotionInfo &target, PathCache &path, TrajectoryCache &trajectory)
{
    char buffer[LOG_TEXT_SIZE];
    clock_t start_clock, end_clock;
    double  path_plan_time, path_ik_time, traj_plan_time;
    PoseEuler fcp_in_base, tcp_in_base, start_pose;
    kinematics_ptr_->doFK(start_state.angle, fcp_in_base);
    transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
    transformation_.convertPoseFromBaseToUser(tcp_in_base, user_frame_, start_pose);
    
    const PoseEuler &last_pose = via.target.pose.pose;
    const PoseEuler &via_pose = target.via.pose.pose;
    const PoseEuler &target_pose = target.target.pose.pose;
    FST_INFO("autoSmoothCircle: vel = %.4f, cnt = %.4f", target.vel, target.cnt);
    FST_INFO("Start-joint: %s", printDBLine(&start_state.angle[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-omega: %s", printDBLine(&start_state.omega[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-alpha: %s", printDBLine(&start_state.alpha[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
    FST_INFO("Via-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", via_pose.point_.x_, via_pose.point_.y_, via_pose.point_.z_, via_pose.euler_.a_, via_pose.euler_.b_, via_pose.euler_.c_);
    FST_INFO("Target-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
    FST_INFO("Last-target-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", last_pose.point_.x_, last_pose.point_.y_, last_pose.point_.z_, last_pose.euler_.a_, last_pose.euler_.b_, last_pose.euler_.c_);
    FST_INFO("Via-joint: %s", printDBLine(&target.via.joint[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Target-joint: %s", printDBLine(&target.target.joint[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Last-target-joint: %s", printDBLine(&via.target.joint[0], buffer, LOG_TEXT_SIZE));

    start_clock = clock();
    ErrorCode err = planPathSmoothCircle(start_pose, via, target, path);
    end_clock = clock();
    path_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;
    path.target = target;

    if (err != SUCCESS)
    {
        FST_ERROR("Path plan failed, code = 0x%llx", err);
        return err;
    }

    if (!checkPath(path))
    {
        FST_ERROR("Data in path cache is invalid.");
        return MC_INTERNAL_FAULT;
    }
        
    FST_INFO("Path plan success, total-point = %d, smooth-in = %d, smooth-out = %d, inverse kinematics ...", path.cache_length, path.smooth_in_index, path.smooth_out_index);

    start_clock = clock();
    err = computeInverseKinematicsOnPathCache(start_state.angle, path);
    end_clock = clock();
    path_ik_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err != SUCCESS)
    {
        FST_ERROR("Inverse kinematics failed, code = 0x%llx", err);
        return err;
    }
    
    FST_INFO("Inverse kinematics success, plan trajectory ...");
    
    start_clock = clock();
    err = planTrajectorySmooth(path, start_state, via, vel_ratio_, acc_ratio_, trajectory);
    end_clock = clock();
    traj_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err != SUCCESS)
    {
        FST_ERROR("Trajectory plan failed, code = 0x%llx", err);
        return err;
    }
    
    if (!checkTrajectory(trajectory))
    {
        FST_ERROR("Data in trajectory cache is invalid.");
        return MC_INTERNAL_FAULT;
    }

    FST_INFO("Trajectory plan success, total-segment = %d, smooth-out = %d,", trajectory.cache_length, trajectory.smooth_out_index);
    FST_INFO("autoSmoothCircle success, path-plan: %.4f ms, path-ik: %.4f ms, traj-plan: %.4f ms", path_plan_time, path_ik_time, traj_plan_time);
    return SUCCESS;
}

bool BaseGroup::nextMovePermitted(void)
{
    // FST_WARN("is-next-Move-Permitted ?");
    pthread_mutex_lock(&cache_list_mutex_);

    if (fine_waiter_.isEnable() && !fine_waiter_.isStable())
    {
        pthread_mutex_unlock(&cache_list_mutex_);
        // FST_WARN("fine-waiter is enable and fine-waiter is not stable");
        return false;
    }

    if (group_state_ == PAUSE || group_state_ == PAUSING || group_state_ == PAUSE_RETURN || group_state_ == AUTO_TO_PAUSE || group_state_ == PAUSE_TO_PAUSE_RETURN || group_state_ == PAUSE_RETURN_TO_STANDBY)
    {
        pthread_mutex_unlock(&cache_list_mutex_);
        return false;
    }

    auto *last_traj_ptr = getLastTrajectoryCacheListPtr();

    if (group_state_ == STANDBY && last_traj_ptr != NULL)
    {
        pthread_mutex_unlock(&cache_list_mutex_);
        // FST_WARN("Not-permitted, grp-state = %d, traj-list-ptr = %p", group_state_, last_traj_ptr);
        return false;
    }
    else if (group_state_ == AUTO || group_state_ == STANDBY_TO_AUTO)
    {
        if (last_traj_ptr != NULL)
        {
            if (last_traj_ptr->trajectory_cache.smooth_out_index == -1)
            {
                pthread_mutex_unlock(&cache_list_mutex_);
                // FST_WARN("Not-permitted, fine, pick-index = %d, cache-length = %d", last_traj_ptr->pick_index, last_traj_ptr->trajectory_cache.cache_length);
                return false;
            }
            else
            {
                if ((int)last_traj_ptr->pick_index <= last_traj_ptr->trajectory_cache.smooth_out_index)
                {
                    pthread_mutex_unlock(&cache_list_mutex_);
                    // FST_WARN("Not-permitted, smooth, pick-index = %d, smooth-out-index = %d", last_traj_ptr->pick_index, last_traj_ptr->trajectory_cache.smooth_out_index);
                    return false;
                }
                else
                {
                    pthread_mutex_unlock(&cache_list_mutex_);
                    FST_WARN("Next motion permitted, smooth, grp-state = %d, pick-index = %d, smooth-out-index = %d", group_state_, last_traj_ptr->pick_index, last_traj_ptr->trajectory_cache.smooth_out_index);
                    return true;
                }
            }
        }
    }

    pthread_mutex_unlock(&cache_list_mutex_);
    FST_WARN("Next motion permitted, grp-state = %d, traj-list-ptr = %p, fine.enable = %d, fine.stable = %d", group_state_, last_traj_ptr, fine_waiter_.isEnable(), fine_waiter_.isStable());
    return true;
}


ErrorCode BaseGroup::moveOffLineTrajectory(int id, const string &file_name)
{
    FST_INFO("Auto move by trajectory, motion ID = %d, name = %s", id, file_name.c_str());
    Joint control_joint, current_joint;

    if (group_state_ == STANDBY && servo_state_ == SERVO_IDLE)
    {
        getLatestJoint(current_joint);

        if (bare_core_.getControlPosition(&control_joint[0], getNumberOfJoint()))
        {
            if (!isSameJoint(current_joint, control_joint, MINIMUM_E3))
            {
                char buffer[LOG_TEXT_SIZE];
                FST_ERROR("Control-position different with current-position, it might be a trouble.");
                FST_ERROR("Control-position: %s", printDBLine(&control_joint[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("Current-position: %s", printDBLine(&current_joint[0], buffer, LOG_TEXT_SIZE));
                return MC_INTERNAL_FAULT;
            }
        }
        else
        {
            FST_ERROR("Cannot get control position from bare core.");
            return BARE_CORE_TIMEOUT;
        }
    }
    else
    {
        FST_ERROR("Cannot start auto motion in current group state: %d, servo-state: %d", group_state_, servo_state_);
        return INVALID_SEQUENCE;
    }

    string line;
    string name = file_name + ".csv";
    ifstream in(name);

    if (in)
    {
        while (getline(in, line))
        {
            FST_INFO("%s", line.c_str());
        }

        FST_INFO("Done!");
    }
    else
    {
        FST_ERROR("File: %s not found.", name.c_str());
        return INVALID_PARAMETER;
    }

    return SUCCESS;
}


void BaseGroup::getLatestJoint(Joint &joint)
{
    pthread_mutex_lock(&servo_mutex_);
    joint = servo_joint_;
    pthread_mutex_unlock(&servo_mutex_);
}

Joint BaseGroup::getLatestJoint(void)
{
    pthread_mutex_lock(&servo_mutex_);
    Joint joint(servo_joint_);
    pthread_mutex_unlock(&servo_mutex_);
    return joint;
}

void BaseGroup::getServoState(ServoState &state)
{
    pthread_mutex_lock(&servo_mutex_);
    state = servo_state_;
    pthread_mutex_unlock(&servo_mutex_);
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
        return BARE_CORE_TIMEOUT;
    }
}

void BaseGroup::getGroupState(GroupState &state)
{
    state = group_state_;
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

ErrorCode BaseGroup::pickPointsFromManualTrajectory(TrajectoryPoint *points, size_t &length)
{
    pthread_mutex_lock(&manual_traj_mutex_);
    ErrorCode err = (manual_traj_.frame == JOINT || manual_traj_.mode == APOINT) ? pickPointsFromManualJoint(points, length) : pickPointsFromManualCartesian(points, length);
    pthread_mutex_unlock(&manual_traj_mutex_);

    if (err == SUCCESS)
    {
        if (points[length - 1].level == POINT_ENDING)
        {
            char buffer[LOG_TEXT_SIZE];
            manual_to_standby_request_ = true;
            FST_INFO("Get ending-point: %.4f - %s", manual_time_, printDBLine(&points[length - 1].angle[0], buffer, LOG_TEXT_SIZE));
            resetManualTrajectory();
            start_joint_ = points[length - 1].angle;
        }

        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to pick points from manual trajectory, code = 0x%llx.", err);
        return err;
    }
}

ErrorCode BaseGroup::pickPointsFromManualJoint(TrajectoryPoint *points, size_t &length)
{
    size_t picked_num = 0;
    size_t joint_num = getNumberOfJoint();
    double *angle_ptr, *start_ptr, *target_ptr;
    double tm, omega;
    
    //FST_INFO("Pick from manual joint, manual-time = %.4f", manual_time_);

    for (size_t i = 0 ; i < length; i++)
    {
        points[i].level = manual_time_ > MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
        memset(&points[i].omega, 0, sizeof(Joint));
        memset(&points[i].alpha, 0, sizeof(Joint));
        memset(&points[i].ma_cv_g, 0, sizeof(Joint));

        angle_ptr  = (double*)&points[i].angle;
        start_ptr  = (double*)&manual_traj_.joint_start;
        target_ptr = (double*)&manual_traj_.joint_ending;

        manual_time_ += cycle_time_;

        for (size_t jnt = 0; jnt < joint_num; jnt++)
        {
            if (manual_time_ < manual_traj_.coeff[jnt].start_time)
            {
                *angle_ptr = *start_ptr;
            }
            else if (manual_time_ < manual_traj_.coeff[jnt].stable_time)
            {
                tm = manual_time_ - manual_traj_.coeff[jnt].start_time;
                *angle_ptr = *start_ptr + manual_traj_.coeff[jnt].start_alpha * tm * tm / 2;
            }
            else if (manual_time_ < manual_traj_.coeff[jnt].brake_time)
            {
                tm = manual_traj_.coeff[jnt].stable_time - manual_traj_.coeff[jnt].start_time;
                omega = manual_traj_.coeff[jnt].start_alpha * tm;
                *angle_ptr = *start_ptr + omega * tm / 2;
                tm = manual_time_ - manual_traj_.coeff[jnt].stable_time;
                *angle_ptr = *angle_ptr + omega * tm;
            }
            else if (manual_time_ < manual_traj_.coeff[jnt].stop_time)
            {
                tm = manual_traj_.coeff[jnt].stable_time - manual_traj_.coeff[jnt].start_time;
                omega = manual_traj_.coeff[jnt].start_alpha * tm;
                *angle_ptr = *start_ptr + omega * tm / 2;
                tm = manual_traj_.coeff[jnt].brake_time - manual_traj_.coeff[jnt].stable_time;
                *angle_ptr = *angle_ptr + omega * tm;
                tm = manual_time_ - manual_traj_.coeff[jnt].brake_time;
                *angle_ptr = *angle_ptr + omega * tm + manual_traj_.coeff[jnt].brake_alpha * tm * tm / 2;
            }
            else
            {
                *angle_ptr = *target_ptr;
                *start_ptr = *target_ptr;
            }

            ++ angle_ptr;
            ++ start_ptr;
            ++ target_ptr;
        }

        //char buffer[LOG_TEXT_SIZE];
        //FST_INFO("  >> joint: %s", printDBLine(&points[i].angle[0], buffer, LOG_TEXT_SIZE));
        picked_num ++;

        if (manual_time_ >= manual_traj_.duration)
        {
            points[i].level = POINT_ENDING;
            break;
        }
    }

    length = picked_num;
    return SUCCESS;
}

ErrorCode BaseGroup::pickPointsFromManualCartesian(TrajectoryPoint *points, size_t &length)
{
    ErrorCode err = SUCCESS;
    PoseEuler pose, tcp_in_base, fcp_in_base;
    //Joint ref_joint;
    double tim, vel;
    double *axis_ptr, *start_ptr, *target_ptr;
    size_t picked_num = 0;
    
    //FST_INFO("Pick from manual cartesian, manual-time = %.4f", manual_time_);
    Posture posture = kinematics_ptr_->getPostureByJoint(getLatestJoint());
    //FST_INFO("posture: %d,%d,%d,%d", posture.arm, posture.elbow, posture.wrist, posture.flip);
    //ref_joint = getLatestJoint();

    for (size_t i = 0; i < length; i++)
    {
        points[i].level = manual_time_ > MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
        memset(&points[i].omega, 0, sizeof(Joint));
        memset(&points[i].alpha, 0, sizeof(Joint));
        memset(&points[i].ma_cv_g, 0, sizeof(Joint));

        axis_ptr   = (double *) &pose.point_.x_;
        start_ptr  = (double *) &manual_traj_.cart_start.point_.x_;
        target_ptr = (double *) &manual_traj_.cart_ending.point_.x_;

        manual_time_ += cycle_time_;

        for (size_t i = 0; i < 6; i++)
        {
            if (manual_time_ < manual_traj_.coeff[i].start_time)
            {
                *axis_ptr = *start_ptr;
            }
            else if (manual_time_ < manual_traj_.coeff[i].stable_time)
            {
                tim = manual_time_ - manual_traj_.coeff[i].start_time;
                *axis_ptr = *start_ptr + manual_traj_.coeff[i].start_alpha * tim * tim / 2;
            }
            else if (manual_time_ < manual_traj_.coeff[i].brake_time)
            {
                tim = manual_traj_.coeff[i].stable_time - manual_traj_.coeff[i].start_time;
                vel = manual_traj_.coeff[i].start_alpha * tim;
                *axis_ptr = *start_ptr + vel * tim / 2;
                tim = manual_time_ - manual_traj_.coeff[i].stable_time;
                *axis_ptr = *axis_ptr + vel * tim;
            }
            else if (manual_time_ < manual_traj_.coeff[i].stop_time)
            {
                tim = manual_traj_.coeff[i].stable_time - manual_traj_.coeff[i].start_time;
                vel = manual_traj_.coeff[i].start_alpha * tim;
                *axis_ptr = *start_ptr + vel * tim / 2;
                tim = manual_traj_.coeff[i].brake_time - manual_traj_.coeff[i].stable_time;
                *axis_ptr = *axis_ptr + vel * tim;
                tim = manual_time_ - manual_traj_.coeff[i].brake_time;
                *axis_ptr = *axis_ptr + vel * tim + manual_traj_.coeff[i].brake_alpha * tim * tim / 2;
            }
            else
            {
                *axis_ptr = *target_ptr;
                *start_ptr = *target_ptr;
            }

            ++ axis_ptr;
            ++ start_ptr;
            ++ target_ptr;
        }

        switch (manual_traj_.frame)
        {
            case BASE:
                transformation_.convertTcpToFcp(pose, tool_frame_, fcp_in_base);
                err = kinematics_ptr_->doIK(fcp_in_base, posture, points[i].angle) ? SUCCESS : MC_COMPUTE_IK_FAIL;
                break;
            case USER:
                transformation_.convertPoseFromUserToBase(pose, user_frame_, tcp_in_base);
                transformation_.convertTcpToFcp(tcp_in_base, tool_frame_, fcp_in_base);
                err = kinematics_ptr_->doIK(fcp_in_base, posture, points[i].angle) ? SUCCESS : MC_COMPUTE_IK_FAIL;
                break;
            case WORLD:
                transformation_.convertPoseFromUserToBase(pose, world_frame_, tcp_in_base);
                transformation_.convertTcpToFcp(tcp_in_base, tool_frame_, fcp_in_base);
                err = kinematics_ptr_->doIK(fcp_in_base, posture, points[i].angle) ? SUCCESS : MC_COMPUTE_IK_FAIL;
                break;
            case TOOL:
                transformation_.convertPoseFromToolToBase(pose, manual_traj_.tool_coordinate, tool_frame_, fcp_in_base);
                err = kinematics_ptr_->doIK(fcp_in_base, posture, points[i].angle) ? SUCCESS : MC_COMPUTE_IK_FAIL;
                break;
            case JOINT:
            default:
                err = MC_INTERNAL_FAULT;
                break;
        }

        if (err != SUCCESS)
        {
            char buffer[LOG_TEXT_SIZE];
            FST_ERROR("IK failed, code = 0x%llx", err);
            FST_ERROR("  mode: %d, frame: %d, duration: %.4f, manual-time: %.4f", manual_traj_.mode, manual_traj_.frame, manual_traj_.duration, manual_time_);
            FST_ERROR("  joint-start: %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("  cart-start: %s", printDBLine(&manual_traj_.cart_start.point_.x_, buffer, LOG_TEXT_SIZE));
            FST_ERROR("  cart-end: %s", printDBLine(&manual_traj_.cart_ending.point_.x_, buffer, LOG_TEXT_SIZE));

            for (size_t i = 0; i < 6; i++)
            {
                ManualCoef &coe = manual_traj_.coeff[i];
                FST_ERROR("    axis-%d: start=%.4f, stable=%.4f, brake=%.4f, stop=%.4f, start_alpha=%.6f, brake_alpha=%.6f", i, coe.start_time, coe.stable_time, coe.brake_time, coe.stop_time, coe.start_alpha, coe.brake_alpha);
            }

            FST_ERROR("  pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            FST_ERROR("  user-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", user_frame_.point_.x_, user_frame_.point_.y_, user_frame_.point_.z_, user_frame_.euler_.a_, user_frame_.euler_.b_, user_frame_.euler_.c_);
            FST_ERROR("  tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame_.point_.x_, tool_frame_.point_.y_, tool_frame_.point_.z_, tool_frame_.euler_.a_, tool_frame_.euler_.b_, tool_frame_.euler_.c_);
            FST_ERROR("  fcp-in-base: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", fcp_in_base.point_.x_, fcp_in_base.point_.y_, fcp_in_base.point_.z_, fcp_in_base.euler_.a_, fcp_in_base.euler_.b_, fcp_in_base.euler_.c_);
            //FST_ERROR("  reference: %s", printDBLine(&ref_joint[0], buffer, LOG_TEXT_SIZE));
            break;
        }

        if (!soft_constraint_.isJointInConstraint(points[i].angle))
        {
            char buffer[LOG_TEXT_SIZE];
            FST_ERROR("IK result out of soft constraint:");
            FST_ERROR("  mode: %d, frame: %d, duration: %.4f, manual-time: %.4f", manual_traj_.mode, manual_traj_.frame, manual_traj_.duration, manual_time_);
            FST_ERROR("  joint-start: %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("  cart-start: %s", printDBLine(&manual_traj_.cart_start.point_.x_, buffer, LOG_TEXT_SIZE));
            FST_ERROR("  cart-end: %s", printDBLine(&manual_traj_.cart_ending.point_.x_, buffer, LOG_TEXT_SIZE));

            for (size_t i = 0; i < 6; i++)
            {
                ManualCoef &coe = manual_traj_.coeff[i];
                FST_ERROR("    axis-%d: start=%.4f, stable=%.4f, brake=%.4f, stop=%.4f, start_alpha=%.6f, brake_alpha=%.6f", i, coe.start_time, coe.stable_time, coe.brake_time, coe.stop_time, coe.start_alpha, coe.brake_alpha);
            }

            FST_ERROR("  pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            FST_ERROR("  user-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", user_frame_.point_.x_, user_frame_.point_.y_, user_frame_.point_.z_, user_frame_.euler_.a_, user_frame_.euler_.b_, user_frame_.euler_.c_);
            FST_ERROR("  tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame_.point_.x_, tool_frame_.point_.y_, tool_frame_.point_.z_, tool_frame_.euler_.a_, tool_frame_.euler_.b_, tool_frame_.euler_.c_);
            FST_ERROR("  fcp-in-base: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", fcp_in_base.point_.x_, fcp_in_base.point_.y_, fcp_in_base.point_.z_, fcp_in_base.euler_.a_, fcp_in_base.euler_.b_, fcp_in_base.euler_.c_);
            //FST_ERROR("  reference: %s", printDBLine(&ref_joint[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("  joint: %s", printDBLine(&points[i].angle[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("  upper: %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("  lower: %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
            err = JOINT_OUT_OF_CONSTRAINT;
            break;
        }

        //char buffer[LOG_TEXT_SIZE];
        //FST_INFO("  >> pose : %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
        //FST_INFO("  >> tf : %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", tool_frame_.point_.x_, tool_frame_.point_.y_, tool_frame_.point_.z_, tool_frame_.euler_.a_, tool_frame_.euler_.b_, tool_frame_.euler_.c_);
        //FST_INFO("  >> fcp_in_base : %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", fcp_in_base.point_.x_, fcp_in_base.point_.y_, fcp_in_base.point_.z_, fcp_in_base.euler_.a_, fcp_in_base.euler_.b_, fcp_in_base.euler_.c_);
        //FST_INFO("  >> joint: %s", printDBLine(&points[i].angle[0], buffer, LOG_TEXT_SIZE));
        picked_num ++;
        //ref_joint = points[i].angle;

        if (manual_time_ >= manual_traj_.duration)
        {
            points[i].level = POINT_ENDING;
            break;
        }
    }

    //FST_INFO("Pick from manual cartesian, %d picked", picked_num);
    length = picked_num;
    return err;
}

void BaseGroup::resetManualTrajectory(void)
{
    pthread_mutex_lock(&manual_traj_mutex_);
    manual_time_ = 0;
    manual_traj_.duration = -99.99;
    memset(manual_traj_.coeff, 0, NUM_OF_JOINT * sizeof(ManualCoef));
    memset(manual_traj_.direction, 0, sizeof(manual_traj_.direction));
    pthread_mutex_unlock(&manual_traj_mutex_);
}

void BaseGroup::doStateMachine(void)
{
    static size_t disable_to_standby_cnt = 0;
    static size_t standby_to_disable_cnt = 0;
    static size_t standby_to_auto_cnt = 0;
    static size_t auto_to_standby_cnt = 0;
    static size_t manual_to_standby_cnt = 0;
    static size_t auto_to_pause_cnt = 0;
    static size_t pause_to_pause_return_cnt = 0;
    static size_t pause_return_to_standby_cnt = 0;

    auto servo_state = getServoState();

    if (abort_request_)
    {
        FST_INFO("Abort group request received, group-state = %d.", group_state_);
        abort_request_ = false;
        clear_request_ = true;
    }
    

    /*
    if (abort_request_)
    {
        FST_INFO("Abort group request received, group-state = %d.", group_state_);

        if (group_state_ == AUTO)
        {
            group_state_ = STANDBY;
            pthread_mutex_lock(&cache_list_mutex_);
            auto path_ptr = path_list_ptr_;
            auto traj_ptr = traj_list_ptr_;

            while (path_ptr != NULL)
            {
                path_list_ptr_ = path_list_ptr_->next_ptr;
                path_cache_pool_.freeCachePtr(path_ptr);
                path_ptr = path_list_ptr_;
            }

            while (traj_ptr != NULL)
            {
                traj_list_ptr_ = traj_list_ptr_->next_ptr;
                traj_cache_pool_.freeCachePtr(traj_ptr);
                traj_ptr = traj_list_ptr_;
            }

            pthread_mutex_unlock(&cache_list_mutex_);
            auto_time_ = 0;
            traj_fifo_.clear();
            bare_core_.clearPointCache();
            fine_waiter_.disableWaiter();
            abort_request_ = false;
            FST_INFO("Auto move Aborted");
        }
        else if (group_state_ == MANUAL)
        {
            group_state_ = STANDBY;
            resetManualTrajectory();
            bare_core_.clearPointCache();
            abort_request_ = false;
            FST_INFO("Manual move Aborted");
        }
        else
        {}
    }
    */

    if (error_request_)
    {
        stop_request_ = true;
        clear_request_ = true;
        error_request_ = false;
    }

    if (clear_request_)
    {
        FST_INFO("Clear group request received, group-state = %d", group_state_);

        if (group_state_ != UNKNOW && group_state_ != DISABLE && group_state_ != DISABLE_TO_STANDBY && group_state_ != STANDBY_TO_DISABLE)
        {
            group_state_ = STANDBY;
        }
        
        pthread_mutex_lock(&cache_list_mutex_);
        auto path_ptr = path_list_ptr_;
        auto traj_ptr = traj_list_ptr_;

        while (path_ptr != NULL)
        {
            path_list_ptr_ = path_list_ptr_->next_ptr;
            path_cache_pool_.freeCachePtr(path_ptr);
            path_ptr = path_list_ptr_;
        }

        while (traj_ptr != NULL)
        {
            traj_list_ptr_ = traj_list_ptr_->next_ptr;
            traj_cache_pool_.freeCachePtr(traj_ptr);
            traj_ptr = traj_list_ptr_;
        }

        pause_status_.pause_valid = false;
        pause_status_.pause_index = 0;
        pthread_mutex_unlock(&cache_list_mutex_);
        auto_time_ = 0;
        traj_fifo_.clear();
        fine_waiter_.disableWaiter();

        resetManualTrajectory();
        bare_core_.clearPointCache();
        clear_request_ = false;
        FST_INFO("Group cleared.");
    }

    if (reset_request_ && group_state_ != DISABLE)
    {
        reset_request_ = false;
    }

    if (stop_request_ && group_state_ == DISABLE)
    {
        stop_request_ = false;
    }

    if (auto_to_pause_request_ && group_state_ != AUTO)
    {
        auto_to_pause_request_ = false;
    }

    if (pause_to_auto_request_ && group_state_ != PAUSE)
    {
        pause_to_auto_request_ = false;
    }

    switch (group_state_)
    {
        case DISABLE:
        {
            if (reset_request_)
            {
                FST_INFO("Reset request received, reset barecore.");
                reset_request_ = false;
                bare_core_.resetBareCore();
                disable_to_standby_cnt = 0;
                group_state_ = DISABLE_TO_STANDBY;
            }

            break;
        }

        case STANDBY:
        {
            if (stop_request_)
            {
                FST_INFO("Stop request received, stop barecore.");
                stop_request_ = false;
                bare_core_.stopBareCore();
                standby_to_disable_cnt = 0;
                group_state_ = STANDBY_TO_DISABLE;
            }

            if (traj_list_ptr_ != NULL)
            {
                standby_to_auto_cnt = 0;
                group_state_ = STANDBY_TO_AUTO;
            }
            else if (manual_traj_.duration > MINIMUM_E6)
            {
                group_state_ = STANDBY_TO_MANUAL;
            }

            break;
        }

        case AUTO:
        {
            if (auto_to_standby_request_)
            {
                auto_to_standby_request_ = false;
                auto_to_standby_cnt = 0;
                group_state_ = AUTO_TO_STANDBY;
            }

            if (auto_to_pause_request_)
            {
                auto_to_pause_request_ = false;
                //auto_to_pause_cnt = 0;
                group_state_ = PAUSING;
            }

            if (stop_request_)
            {
                stop_request_ = false;
                error_request_ = true;
            }

            break;
        }

        case PAUSING:
        {
            if (pausing_to_pause_request_)
            {
                pausing_to_pause_request_ = false;
                auto_to_pause_cnt = 0;
                group_state_ = AUTO_TO_PAUSE;
            }
        }

        case MANUAL:
        {
            if (manual_to_standby_request_)
            {
                manual_to_standby_request_ = false;
                manual_to_standby_cnt = 0;
                group_state_ = MANUAL_TO_STANDBY;
            }

            if (stop_request_)
            {
                stop_request_ = false;
                error_request_ = true;
            }

            break;
        }

        case PAUSE:
        {
            if (pause_to_auto_request_)
            {
                pause_to_auto_request_ = false;

                if (traj_list_ptr_ != NULL)
                {
                    pause_to_pause_return_cnt = 0;
                    group_state_ = PAUSE_TO_PAUSE_RETURN;
                }
                else
                {
                    group_state_ = PAUSE_RETURN_TO_STANDBY;
                    FST_WARN("Group-state switch to pause-return.");
                }
            }
            break;
        }

        case PAUSE_RETURN:
        {
            if (pause_return_to_standby_request_)
            {
                pause_return_to_standby_request_ = false;
                pause_return_to_standby_cnt = 0;
                group_state_ = PAUSE_RETURN_TO_STANDBY;
            }

            break;
        }

        case PAUSE_RETURN_TO_STANDBY:
        {
            if (servo_state == SERVO_IDLE)
            {
                // 检查是否停稳，停稳后切换到standby
                if (fine_waiter_.isEnable())
                {
                    if (fine_waiter_.isStable())
                    {
                        fine_waiter_.disableWaiter();
                        pause_status_.pause_valid = false;
                        group_state_ = STANDBY;
                        FST_WARN("Group-state switch to standby.");
                    }
                }
                else
                {
                    group_state_ = STANDBY;
                    pause_status_.pause_valid = false;
                    FST_WARN("Group-state switch to standby.");
                }
            }

            pause_return_to_standby_cnt ++;

            if (pause_return_to_standby_cnt > auto_to_standby_timeout_)
            {
                FST_ERROR("Auto to standby timeout, error-request asserted.");
                reportError(BARE_CORE_TIMEOUT);
                error_request_ = true;
            }

            if (group_state_ == STANDBY)
            {
                // replan path cache
                FST_INFO("Pause return finished, replan path cache.");
                ErrorCode err = replanPathCache();

                if (err != SUCCESS)
                {
                    reportError(err);
                    error_request_ = true;
                    FST_WARN("Fail to replan path cache, code = 0x%llx.", err);
                }

                FST_INFO("Replan path cache success.");
            }

            break;
        }

        case PAUSE_TO_PAUSE_RETURN:
        {
            pause_to_pause_return_cnt ++;

            if (traj_fifo_.full() || (pause_to_pause_return_cnt > 100 && !traj_fifo_.empty()))
            {
                auto_time_ = 0;
                group_state_ = PAUSE_RETURN;
                FST_WARN("Group-state switch to pause-return, fifo-size = %d, counter = %d", traj_fifo_.size(), standby_to_auto_cnt);
            }
            
            if (pause_to_pause_return_cnt > standby_to_auto_timeout_ && traj_fifo_.empty())
            {
                group_state_ = PAUSE;
                FST_WARN("Group-state switch to pause.");
                FST_WARN("Pause to pause-return time-out but trajectory-fifo still empty.");
            }

            break;
        }

        case DISABLE_TO_STANDBY:
        {
            if (servo_state == SERVO_IDLE)
            {
                if (updateStartJoint())
                {
                    FST_WARN("Group-state switch to standby.");
                    group_state_ = STANDBY;
                }
                else
                {
                    FST_ERROR("Fail to update start joint, group-state switch to disable.");
                    group_state_ = DISABLE;
                }
            }

            disable_to_standby_cnt ++;

            if (disable_to_standby_cnt > disable_to_standby_timeout_)
            {
                FST_WARN("Reset to standby timeout, group-state switch to disable.");
                group_state_ = DISABLE;
            }

            break;
        }

        case STANDBY_TO_DISABLE:
        {
            if (servo_state == SERVO_DISABLE)
            {
                FST_WARN("Group-state switch to disable.");
                group_state_ = DISABLE;
            }
            else
            {
                standby_to_disable_cnt ++;

                if (standby_to_disable_cnt > standby_to_disable_timeout_)
                {
                    FST_WARN("Stop to disable timeout, group-state switch to STANDBY.");
                    group_state_ = STANDBY;
                }
            }

            break;
        }

        case STANDBY_TO_AUTO:
        {
            standby_to_auto_cnt ++;

            if (traj_fifo_.full() || (standby_to_auto_cnt > 100 && !traj_fifo_.empty()))
            {
                auto_time_ = 0;
                group_state_ = AUTO;
                FST_WARN("Group-state switch to auto, fifo-size = %d, counter = %d", traj_fifo_.size(), standby_to_auto_cnt);
            }
            
            if (standby_to_auto_cnt > standby_to_auto_timeout_ && traj_fifo_.empty())
            {
                group_state_ = STANDBY;
                FST_WARN("Group-state switch to standby.");
                FST_WARN("Standby to auto time-out but trajectory-fifo still empty.");
            }
            
            break;
        }
        
        case AUTO_TO_STANDBY:
        {
            if (servo_state == SERVO_IDLE)
            {
                // 检查是否停稳，停稳后切换到standby
                if (fine_waiter_.isEnable())
                {
                    if (fine_waiter_.isStable())
                    {
                        fine_waiter_.disableWaiter();
                        group_state_ = STANDBY;
                        FST_WARN("Group-state switch to standby.");
                    }
                }
                else
                {
                    group_state_ = STANDBY;
                    FST_WARN("Group-state switch to standby.");
                }
            }

            auto_to_standby_cnt ++;

            if (auto_to_standby_cnt > auto_to_standby_timeout_)
            {
                FST_ERROR("Auto to standby timeout, error-request asserted.");
                reportError(BARE_CORE_TIMEOUT);
                error_request_ = true;
            }

            break;
        }

        case STANDBY_TO_MANUAL:
        {
            manual_time_ = 0;
            group_state_ = MANUAL;
            FST_WARN("Group-state switch to manual.");
            break;
        }
            
        case MANUAL_TO_STANDBY:
        {
            if (servo_state == SERVO_IDLE)
            {
                group_state_ = STANDBY;
                FST_WARN("Group-state switch to standby.");
            }

            manual_to_standby_cnt ++;

            if (manual_to_standby_cnt > manual_to_standby_timeout_)
            {
                FST_ERROR("Manual to standby timeout, error-request asserted.");
                reportError(BARE_CORE_TIMEOUT);
                error_request_ = true;
            }

            break;
        }

        case AUTO_TO_PAUSE:
        {
            if (servo_state == SERVO_IDLE)
            {
                // 检查是否停稳，停稳后切换到pause
                if (fine_waiter_.isEnable())
                {
                    if (fine_waiter_.isStable())
                    {
                        fine_waiter_.disableWaiter();
                        group_state_ = PAUSE;
                        FST_WARN("Group-state switch to pause.");
                    }
                }
                else
                {
                    group_state_ = PAUSE;
                    FST_WARN("Group-state switch to pause.");
                }
            }

            auto_to_pause_cnt ++;

            if (auto_to_pause_cnt > auto_to_pause_timeout_)
            {
                FST_ERROR("Auto to pause timeout, error-request asserted.");
                reportError(BARE_CORE_TIMEOUT);
                error_request_ = true;
            }

            break;
        }

        case UNKNOW:
        {
            if (servo_state == SERVO_DISABLE)
            {
                FST_WARN("Group-state switch to disable.");
                group_state_ = DISABLE;
            }
            else
            {
                //FST_WARN("Group-state is unknow but servo-state is %d, send stop request to servo.", servo_state);
                //bare_core_.stopBareCore();
                FST_ERROR("Group-state is UNKNOW but servo-state is %d", servo_state);
                reportError(MC_INTERNAL_FAULT);
            }

            break;
        }

        default:
        {
            FST_ERROR("Group-state is invalid: 0x%x", group_state_);
            FST_ERROR("Group-state switch to unknow.");
            group_state_ = UNKNOW;
            break;
        }
    }

    if (servo_state == SERVO_IDLE && group_state_ == DISABLE)
    {
        FST_ERROR("Group-state is disable but servo-state is idle, switch group-state to UNKNOW");
        group_state_ = UNKNOW;
    }

    if (servo_state == SERVO_DISABLE && group_state_ == STANDBY)
    {
        FST_ERROR("Group-state is STANDBY but servo-state is diabale, switch group-state to UNKNOW");
        group_state_ = UNKNOW;
        clear_request_ = true;
    }

    if (servo_state == SERVO_DISABLE && group_state_ == AUTO)
    {
        FST_ERROR("Group-state is AUTO but servo-state is diabale, switch group-state to UNKNOW");
        group_state_ = UNKNOW;
        clear_request_ = true;
    }
}

bool BaseGroup::updateStartJoint(void)
{
    char buffer[LOG_TEXT_SIZE];
    Joint current_joint, control_joint;
    getLatestJoint(current_joint);

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
    if (group_state_ == AUTO || group_state_ == PAUSING || group_state_ == STANDBY_TO_AUTO || group_state_ == PAUSE_TO_PAUSE_RETURN || group_state_ == PAUSE_RETURN)
    {
        pthread_mutex_lock(&cache_list_mutex_);

        if (traj_list_ptr_ != NULL)
        {
            auto &traj_cache = traj_list_ptr_->trajectory_cache;
            TrajectorySegment segment;

            while ((traj_cache.smooth_out_index == -1 && traj_list_ptr_->pick_index < traj_cache.cache_length) ||
                   (traj_cache.smooth_out_index != -1 && (int)traj_list_ptr_->pick_index <= traj_cache.smooth_out_index))
            {
                TrajectoryBlock &block = traj_cache.cache[traj_list_ptr_->pick_index];
                segment.time_from_block = traj_list_ptr_->pick_from_block;
                segment.time_from_start = block.time_from_start + segment.time_from_block;
                segment.duration = traj_list_ptr_->pick_from_block + duration_per_segment_ < block.duration ? duration_per_segment_ : block.duration - traj_list_ptr_->pick_from_block;
                memcpy(segment.axis, block.axis, NUM_OF_JOINT * sizeof(AxisCoeff));

                // for test
                // FST_INFO("time-of-block=%f, time-from-block=%f", block.time_from_start, segment.time_from_block);
                // FST_INFO("pick-from-block=%f, duration-per-seg=%f, duration-of-block=%f", traj_list_ptr_->pick_from_block, duration_per_segment_, block.duration);

                if (traj_fifo_.pushTrajectorySegment(segment) == SUCCESS)
                {
                    //FST_INFO("push: start-time=%.4f, duration = %.4f, block-time = %.4f, block-duration = %.4f", segment.time_from_start, segment.duration, segment.time_from_block, block.duration);
                    if (traj_list_ptr_->pick_from_block + duration_per_segment_ < block.duration)
                    {
                        // 这个block还有部分未进入轨迹FIFO中
                        traj_list_ptr_->pick_from_block += duration_per_segment_;
                    }
                    else
                    {
                        // 这个blcok全部进入轨迹FIFO中，更新block指引
                        traj_list_ptr_->pick_index ++;
                        traj_list_ptr_->pick_from_block = 0;
                    }
                }
                else
                {
                    // 轨迹FIFO已填满，结束循环
                    break;
                }
            }

            // 如果是带Fine的指令，当整条轨迹已经全部进入轨迹FIFO后，设置Fine到位检测，从路径缓存链表和轨迹缓存链表中移除本条路径和轨迹
            if (traj_cache.smooth_out_index == -1 && traj_list_ptr_->pick_index >= traj_cache.cache_length)
            {
                setFineWaiter();
                freeFirstCacheList();
            }

            // 如果是带平滑的指令，当切出点之前的轨迹已经全部进入轨迹FIFO后
            if (traj_cache.smooth_out_index != -1 && (int)traj_list_ptr_->pick_index > traj_cache.smooth_out_index)
            {
                // 如果下一条指令已经就绪，切换到下一条指令轨迹
                if (traj_list_ptr_->next_ptr != NULL)
                {
                    freeFirstCacheList();
                }
                else // 如果下一条指令尚未就绪
                {   
                    // 如果轨迹FIFO中有效轨迹长度不足
                    if (traj_fifo_.size() < 5)
                    {
                         // CNT不等于0时，放弃平滑，处理为CNT等于0
                        if (traj_cache.smooth_out_index < (int)traj_cache.cache_length - 1)
                        {
                            traj_cache.smooth_out_index = traj_cache.cache_length - 1;
                        }
                        else // CNT等于0时，直接移除缓存链表中的cache
                        {
                            freeFirstCacheList();
                        }
                    }
                    else // 轨迹FIFO中有效轨迹长度足够，等待下一跳指令就绪
                    {}
                }
            }
        } // if (traj_list_ptr_ != NULL)
        else
        {}

        pthread_mutex_unlock(&cache_list_mutex_);
    } // if (group_state_ == AUTO || group_state_ == STANDBY_TO_AUTO)
    else
    {}
}

void BaseGroup::freeFirstCacheList(void)
{
    auto *path_ptr = path_list_ptr_;
    auto *traj_ptr = traj_list_ptr_;

    if (traj_ptr->trajectory_cache.path_cache_ptr)
    {
        path_list_ptr_ = path_list_ptr_->next_ptr;
        path_cache_pool_.freeCachePtr(path_ptr);
        FST_INFO("Free path cache: %p", path_ptr);
    }

    traj_list_ptr_ = traj_list_ptr_->next_ptr;
    traj_cache_pool_.freeCachePtr(traj_ptr);
    FST_INFO("Free trajectory cache: %p", traj_ptr);
}

void BaseGroup::setFineWaiter(void)
{
    PoseQuaternion target;
    auto &block = path_list_ptr_->path_cache.cache[path_list_ptr_->path_cache.cache_length - 1];

    if (block.motion_type == MOTION_JOINT)
    {
        PoseEuler fcp_in_base, tcp_in_base, tcp_in_user;
        kinematics_ptr_->doFK(block.joint, fcp_in_base);
        transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
        transformation_.convertPoseFromBaseToUser(tcp_in_base, user_frame_, tcp_in_user);
        target = PoseEuler2Pose(tcp_in_user);
    }
    else
    {
        target = block.pose;
    }

    FST_INFO("setFineWaiter: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f, %.4f", target.point_.x_, target.point_.y_, target.point_.z_, target.quaternion_.w_, target.quaternion_.x_, target.quaternion_.y_, target.quaternion_.z_);
    fine_waiter_.enableWaiter(target);
}

void BaseGroup::loopFineWaiter(void)
{
    if (fine_waiter_.isEnable() && (group_state_ == AUTO_TO_STANDBY || group_state_ == AUTO_TO_PAUSE) && servo_state_ == SERVO_IDLE)
    {  
        PoseEuler fcp_in_base, tcp_in_base, tcp_in_user;
        kinematics_ptr_->doFK(getLatestJoint(), fcp_in_base);
        transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
        transformation_.convertPoseFromBaseToUser(tcp_in_base, user_frame_, tcp_in_user);
        PoseQuaternion barecore_pose = PoseEuler2Pose(tcp_in_user);
        //FST_INFO("loopFineWaiter: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f, %.4f", barecore_pose[0], barecore_pose[1], barecore_pose[2], barecore_pose[3], barecore_pose[4], barecore_pose[5], barecore_pose[6]);
        fine_waiter_.checkWaiter(barecore_pose);
    }
}

void BaseGroup::updateTimeFromStart(TrajectoryCacheList &cache)
{
    MotionTime time_from_start = cache.time_from_start;

    for (size_t i = 0; i < cache.trajectory_cache.cache_length; i++)
    {
        cache.trajectory_cache.cache[i].time_from_start = time_from_start;
        time_from_start += cache.trajectory_cache.cache[i].duration;
    }

    cache.trajectory_cache.cache[cache.trajectory_cache.cache_length].time_from_start = time_from_start;
}

void BaseGroup::doCommonLoop(void)
{
    updateJointRecorder();
    doStateMachine();
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
    static ServoState   barecore_state = SERVO_INIT;
    static Joint        barecore_joint;
    static size_t       fail_cnt = 0;

    if (bare_core_.getLatestJoint(barecore_joint, barecore_state))
    {
        if (servo_state_ != barecore_state)
        {
            FST_INFO("Servo-state switch %d to %d", servo_state_, barecore_state);
        }

        pthread_mutex_lock(&servo_mutex_);
        servo_state_ = barecore_state;
        servo_joint_ = barecore_joint;
        pthread_mutex_unlock(&servo_mutex_);
    }
    else
    {
        if (++fail_cnt > servo_update_timeout_)
        {
            fail_cnt = 0;
            FST_ERROR("Fail to update joint and state from bare core.");
            reportError(BARE_CORE_TIMEOUT);
            error_request_ = true;
        }
    }
}

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

        bare_core_.fillPointCache(points, length, POINT_POS_VEL);

        if (points[length - 1].level == POINT_ENDING)
        {
            // 取到了ENDING-POINT，意味着轨迹结束，需要切状态机
            if (group_state_ == AUTO)
            {
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

    return bare_core_.sendPoint() ? SUCCESS : BARE_CORE_TIMEOUT;
}

ErrorCode BaseGroup::pickPointsFromTrajectoryFifo(TrajectoryPoint *points, size_t &length)
{
    ErrorCode err = SUCCESS;
    size_t picked_num = 0;

    for (size_t i = 0; i < length; i++)
    {
        auto_time_ += cycle_time_;
        err = traj_fifo_.pickTrajectoryPoint(auto_time_, points[i]);

        if (err == SUCCESS)
        {
            picked_num++;

            if (auto_time_ < cycle_time_ + MINIMUM_E6 && points[i].level == POINT_MIDDLE)
            {
                points[i].level = POINT_START;
            }

#ifdef OUTPUT_JOINT_POINT
            g_joint_output_array[g_joint_output_index].time = auto_time_;
            g_joint_output_array[g_joint_output_index].point = points[i];
            g_joint_output_index = (g_joint_output_index + 1) % OUTPUT_JOINT_POINT_SIZE;
#endif

            if (points[i].level == POINT_ENDING)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

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

    return bare_core_.sendPoint() ? SUCCESS : BARE_CORE_TIMEOUT;
}

void BaseGroup::sendTrajectoryFlow(void)
{
    static size_t error_cnt = 0;
    ErrorCode err = SUCCESS;

    if (group_state_ == AUTO && !auto_to_standby_request_ && !auto_to_pause_request_)
    {
        err = sendAutoTrajectoryFlow();
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
    else if (group_state_ == AUTO_TO_STANDBY || group_state_ == AUTO_TO_PAUSE || group_state_ == MANUAL_TO_STANDBY)
    {
        if (!bare_core_.isPointCacheEmpty())
        {
            err = bare_core_.sendPoint() ? SUCCESS : BARE_CORE_TIMEOUT;
        }
    }

    if (err == SUCCESS)
    {
        error_cnt = 0;
    }
    else
    {
        if (err == BARE_CORE_TIMEOUT)
        {
            error_cnt ++;

            if (error_cnt > trajectory_flow_timeout_)
            {
                error_cnt = 0;
                error_request_ = true;
                reportError(BARE_CORE_TIMEOUT);
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

void BaseGroup::doPriorityLoop(void)
{
    updateServoStateAndJoint();
    sendTrajectoryFlow();
    loopFineWaiter();
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




FineWaiter::FineWaiter(void)
{
    enable_ = false;
    threshold_ = MINIMUM_E6;
    stable_cnt_ = 0;
    stable_cycle_ = 5;
}

FineWaiter::~FineWaiter(void)
{}

void FineWaiter::initFineWaiter(size_t stable_cycle, double threshold)
{
    threshold_ = threshold;
    stable_cycle_ = stable_cycle;
}

void FineWaiter::enableWaiter(const PoseQuaternion &target)
{
    waiting_pose_ = target;
    stable_cnt_ = 0;
    enable_ = true;
}

void FineWaiter::disableWaiter(void)
{
    enable_ = false;
}

void FineWaiter::checkWaiter(const PoseQuaternion &pose)
{
    if (getDistance(pose.point_, waiting_pose_.point_) < threshold_)
    {
        stable_cnt_ = stable_cnt_ < stable_cycle_ ? stable_cnt_ + 1 : stable_cycle_;
    }
    else
    {
        stable_cnt_ = 0;
    }
}

bool FineWaiter::isEnable(void)
{
    return enable_;
}

bool FineWaiter::isStable(void)
{
    return stable_cnt_ == stable_cycle_;
}














} // namespace fst_mc
