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
    manual_frame_ = JOINT;

    path_list_ptr_ = NULL;
    traj_list_ptr_ = NULL;

    dynamics_ptr_ = NULL;
    kinematics_ptr_ = NULL;

    vel_ratio_ = 0;
    acc_ratio_ = 0;

    stop_request_ = false;
    reset_request_ = false;
    abort_request_ = false;
    clear_request_ = false;
    error_request_ = false;
    auto_to_standby_request_ = false;
    manual_to_standby_request_ = false;
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
        kinematics_ptr_->forwardKinematicsInUser(point.angle, pose);
        g_joint_out << point.level << "," << g_joint_output_array[i].time << ","
                    << point.angle[0] << "," << point.angle[1] << "," << point.angle[2] << "," << point.angle[3] << "," << point.angle[4] << "," << point.angle[5] << ","
                    << point.omega[0] << "," << point.omega[1] << "," << point.omega[2] << "," << point.omega[3] << "," << point.omega[4] << "," << point.omega[5] << ","
                    << point.alpha[0] << "," << point.alpha[1] << "," << point.alpha[2] << "," << point.alpha[3] << "," << point.alpha[4] << "," << point.alpha[5] << ","
                    << pose.position.x << "," << pose.position.y << "," << pose.position.z << "," << pose.orientation.a << "," << pose.orientation.b << "," << pose.orientation.c << endl;
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
                           << ",target-x=" << path_cache.target.pose_target[0] << ",target-y=" << path_cache.target.pose_target[1] << ",target-z=" << path_cache.target.pose_target[2]
                           << ",target-a=" << path_cache.target.pose_target[3] << ",target-b=" << path_cache.target.pose_target[4] << ",target-c=" << path_cache.target.pose_target[5] << endl;
            }
            else if (path_cache.target.type == MOTION_CIRCLE)
            {
                g_path_out << "move-circle,cnt=" << path_cache.target.cnt << ",vel=" << path_cache.target.vel 
                           << ",via-x=" << path_cache.target.circle_target.pose1[0] << ",via-y=" << path_cache.target.circle_target.pose1[1] << ",via-z=" << path_cache.target.circle_target.pose1[2]
                           << ",via-a=" << path_cache.target.circle_target.pose1[3] << ",via-b=" << path_cache.target.circle_target.pose1[4] << ",via-c=" << path_cache.target.circle_target.pose1[5]
                           << ",target-x=" << path_cache.target.circle_target.pose2[0] << ",target-y=" << path_cache.target.circle_target.pose2[1] << ",target-z=" << path_cache.target.circle_target.pose2[2]
                           << ",target-a=" << path_cache.target.circle_target.pose2[3] << ",target-b=" << path_cache.target.circle_target.pose2[4] << ",target-c=" << path_cache.target.circle_target.pose2[5] << endl;
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
                           << ",x=" << block.pose[0] << ",y=" << block.pose[1] << ",z=" << block.pose[2] 
                           << ",ow=" << block.pose[3] << ",ox=" << block.pose[4] << ",oy=" << block.pose[5] << ",oz=" << block.pose[6] << endl;
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

ManualFrame BaseGroup::getManualFrame(void)
{
    //FST_INFO("Get manual frame = %d", manual_frame_);
    return manual_frame_;
}

ErrorCode BaseGroup::setManualFrame(ManualFrame frame)
{
    FST_INFO("Set manual frame = %d, current frame is %d", frame, manual_frame_);

    if (frame != manual_frame_)
    {
        if (group_state_ == STANDBY)
        {
            manual_frame_ = frame;
            FST_INFO("Done.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Cannot set frame in current state = %d", group_state_);
            return INVALID_SEQUENCE;
        }
    }
    else
    {
        return SUCCESS;
    }
}

double BaseGroup::getManualStepAxis(void)
{
    double step = manual_teach_.getManualStepAxis();
    FST_INFO("Get manual step axis = %.6f", step);
    return step;
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

ErrorCode BaseGroup::setManualStepAxis(double step)
{
    FST_INFO("Set manual step axis = %.6f", step);
    return manual_teach_.setManualStepAxis(step);
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

ErrorCode BaseGroup::manualMoveToPoint(const Joint &joint)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual to target joint: %s", printDBLine(&joint[0], buffer, LOG_TEXT_SIZE));

    if (group_state_ != STANDBY || servo_state_ != SERVO_IDLE)
    {
        FST_ERROR("Cannot manual to target in current group-state = %d, servo-state = %d", group_state_, servo_state_);
        return SUCCESS;
    }

    if (manual_frame_ != JOINT)
    {
        FST_ERROR("Cannot manual to target in current frame = %d", manual_frame_);
        return INVALID_SEQUENCE;
    }

    manual_traj_.joint_start = start_joint_;
    FST_INFO("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        FST_ERROR("start-joint is out of soft constraint, manual-mode-apoint is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (!soft_constraint_.isJointInConstraint(joint))
    {
        FST_ERROR("target-joint out of constraint: %s", printDBLine(&joint[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    manual_time_ = 0;
    manual_traj_.mode = APOINT;
    manual_traj_.frame = JOINT;
    ErrorCode err = manual_teach_.manualByTarget(joint, manual_time_, manual_traj_);

    if (err == SUCCESS)
    {
        FST_INFO("Manual move to target joint, total-duration = %.4f, Success.", manual_traj_.duration);
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        return err;
    }
}

ErrorCode BaseGroup::manualMoveToPoint(const PoseEuler &pose)
{
    ErrorCode err;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual to target pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f",
             pose.position.x, pose.position.y, pose.position.z, pose.orientation.a, pose.orientation.b, pose.orientation.c);

    if (group_state_ != STANDBY || servo_state_ != SERVO_IDLE)
    {
        FST_ERROR("Cannot manual to target in current group-state = %d, servo-state = %d", group_state_, servo_state_);
        return SUCCESS;
    }

    if (manual_frame_ != BASE && manual_frame_ != USER && manual_frame_ != WORLD)
    {
        FST_ERROR("Cannot manual to target in current frame = %d", manual_frame_);
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

    switch (manual_frame_)
    {
        case BASE:
            err = kinematics_ptr_->inverseKinematicsInBase(pose, ref_joint, res_joint);
            break;
        case USER:
            err = kinematics_ptr_->inverseKinematicsInUser(pose, ref_joint, res_joint);
            break;
        case WORLD:
            err = kinematics_ptr_->inverseKinematicsInWorld(pose, ref_joint, res_joint);
            break;
        default:
            FST_ERROR("Invalid manual frame = %d in manual to pose mode", manual_frame_);
            return MOTION_INTERNAL_FAULT;
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
    manual_traj_.frame = manual_frame_;
    err = manual_teach_.manualByTarget(res_joint, manual_time_, manual_traj_);

    if (err == SUCCESS)
    {
        FST_INFO("Manual move to target pose, total-duration = %.4f, Success.", manual_traj_.duration);
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        return err;
    }
}

ErrorCode BaseGroup::manualMoveStep(const ManualDirection *direction)
{
    PoseEuler pose;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual step frame=%d by direction.", manual_frame_);

    if (group_state_ != STANDBY || servo_state_ != SERVO_IDLE)
    {
        FST_ERROR("Cannot manual step in current group-state = %d, servo-state = %d", group_state_, servo_state_);
        return SUCCESS;
    }

    manual_traj_.joint_start = start_joint_;
    FST_INFO("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        if (manual_frame_ == JOINT)
        {
            for (size_t i = 0; i < getNumberOfJoint(); i++)
            {
                if (manual_traj_.joint_start[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
                {
                    FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                              i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                    return INVALID_PARAMETER;
                }
                else if (manual_traj_.joint_start[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
                {
                    FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                              i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                    return INVALID_PARAMETER;
                }
            }
        }
        else
        {
            FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
            return INVALID_SEQUENCE;
        }
    }

    switch (manual_frame_)
    {
        case JOINT:
            break;
        case BASE:
            kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, manual_traj_.cart_start);
        case USER:
            kinematics_ptr_->forwardKinematicsInUser(manual_traj_.joint_start, manual_traj_.cart_start);
            break;
        case WORLD:
            kinematics_ptr_->forwardKinematicsInWorld(manual_traj_.joint_start, manual_traj_.cart_start);
            break;
        case TOOL:
            kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, pose);
            manual_traj_.tool_coordinate = pose;
            memset(&manual_traj_.cart_start, 0, sizeof(manual_traj_.cart_start));
            break;
        default:
            FST_ERROR("Unsupported manual frame: %d", manual_frame_);
            return MOTION_INTERNAL_FAULT;
    }

    manual_time_ = 0;
    manual_traj_.mode = STEP;
    manual_traj_.frame = manual_frame_;
    ErrorCode err = manual_teach_.manualStepByDirect(direction, manual_time_, manual_traj_);

    if (err == SUCCESS)
    {
        FST_INFO("Manual move step, total-duration = %.4f, Success.", manual_traj_.duration);
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        return err;
    }
}

ErrorCode BaseGroup::manualMoveContinuous(const ManualDirection *direction)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual continuous frame=%d by direction.", manual_frame_);

    if (group_state_ != STANDBY && group_state_ != MANUAL)
    {
        FST_ERROR("Cannot manual continuous in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    if (group_state_ == STANDBY)
    {
        if (servo_state_ != SERVO_IDLE)
        {
            FST_ERROR("Cannot manual continuous in current grp-state = %d,  servo-state = %d", group_state_, servo_state_);
            return SUCCESS;
        }

        PoseEuler pose;
        manual_traj_.joint_start = start_joint_;
        FST_INFO("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

        if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
        {
            if (manual_frame_ == JOINT)
            {
                for (size_t i = 0; i < getNumberOfJoint(); i++)
                {
                    if (manual_traj_.joint_start[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
                    {
                        FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                                  i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                        return INVALID_PARAMETER;
                    }
                    else if (manual_traj_.joint_start[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
                    {
                        FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                                  i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                        return INVALID_PARAMETER;
                    }
                }
            }
            else
            {
                FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
                return INVALID_SEQUENCE;
            }
        }

        switch (manual_frame_)
        {
            case JOINT:
                break;
            case BASE:
                kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, manual_traj_.cart_start);
            case USER:
                kinematics_ptr_->forwardKinematicsInUser(manual_traj_.joint_start, manual_traj_.cart_start);
                break;
            case WORLD:
                kinematics_ptr_->forwardKinematicsInWorld(manual_traj_.joint_start, manual_traj_.cart_start);
                break;
            case TOOL:
                kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, pose);
                manual_traj_.tool_coordinate = pose;
                memset(&manual_traj_.cart_start, 0, sizeof(manual_traj_.cart_start));
                break;
            default:
                FST_ERROR("Unsupported manual frame: %d", manual_frame_);
                return MOTION_INTERNAL_FAULT;
        }

        manual_time_ = 0;
        manual_traj_.mode = CONTINUOUS;
        manual_traj_.frame = manual_frame_;
        ErrorCode err = manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);

        if (err == SUCCESS)
        {
            FST_INFO("Manual move continous, total-duration = %.4f, Success.", manual_traj_.duration);
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
            memset(&manual_traj_, 0, sizeof(ManualTrajectory));
            return err;
        }
    }
    else if (group_state_ == MANUAL)
    {
        for (size_t i = 0; i < getNumberOfJoint(); i++)
        {
            if (manual_traj_.direction[i] != direction[i])
            {
                return manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);
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
        FST_ERROR("Cannot manual now, current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }
}

ErrorCode BaseGroup::manualStop(void)
{
    FST_INFO("Manual stop request received.");

    if (group_state_ == MANUAL)
    {
        FST_INFO("Manual mode = %d, frame = %d", manual_traj_.mode, manual_traj_.frame);

        if (manual_traj_.mode == CONTINUOUS)
        {
            ManualDirection direction[NUM_OF_JOINT] = {STANDING};
            ErrorCode err = manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);

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
            ErrorCode err = manual_teach_.manualStop(manual_time_, manual_traj_);

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
            return INVALID_SEQUENCE;
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


ErrorCode BaseGroup::autoMove(int id, const MotionTarget &target)
{
    FST_INFO("Auto move request received, ID = %d, type = %d", id, target.type);
    Joint start = start_joint_;

    ErrorCode err = checkStartState(start);

    if (err != SUCCESS)
    {
        FST_ERROR("Start state check failed, code = 0x%llx", err);
        return err;
    }

    err = checkMotionTarget(target);

    if (err != SUCCESS)
    {
        // 如果目标点和起始点重合，直接跳过这条指令的规划执行，什么也不做
        // 如果是其他错误则报错，提前结束运动规划
        if (err == TARGET_COINCIDENCE)
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
        return MOTION_INTERNAL_FAULT;
    }

    if (target.type == MOTION_JOINT)
    {
        err = autoJoint(start, target, *path_ptr, *traj_ptr);
    }
    else if (target.type == MOTION_LINE)
    {
        err = autoLine(start, target, *path_ptr, *traj_ptr);
    }
    else if (target.type == MOTION_CIRCLE)
    {
        err = autoCircle(start, target, *path_ptr, *traj_ptr);
    }
    else
    {
        FST_ERROR("Invalid motion type = %d, autoMove aborted.", target.type);
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
            return MOTION_INTERNAL_FAULT;
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
    if (group_state_ == STANDBY && servo_state_ == SERVO_IDLE)
    {
        Joint control_joint, current_joint;
        getLatestJoint(current_joint);

        if (bare_core_.getControlPosition(&control_joint[0], getNumberOfJoint()))
        {
            if (!isSameJoint(current_joint, control_joint, MINIMUM_E3 * 10))
            {
                char buffer[LOG_TEXT_SIZE];
                FST_ERROR("Control-position different with current-position, it might be a trouble.");
                FST_ERROR("control-position: %s", printDBLine(&control_joint[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("current-position: %s", printDBLine(&current_joint[0], buffer, LOG_TEXT_SIZE));
                return MOTION_INTERNAL_FAULT;
            }

            if (!isSameJoint(start_joint, control_joint, MINIMUM_E3))
            {
                char buffer[LOG_TEXT_SIZE];
                FST_ERROR("Control-position different with start-position, it might be a trouble.");
                FST_ERROR("control-position: %s", printDBLine(&control_joint[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("start-position:   %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));
                return MOTION_INTERNAL_FAULT;
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

ErrorCode BaseGroup::checkMotionTarget(const MotionTarget &target)
{
    // CNT ∈ [0, 1] U CNT = -1
    if (fabs(target.cnt + 1) > MINIMUM_E9 && (target.cnt < -MINIMUM_E9 || target.cnt > 1 + MINIMUM_E9))
    {
        FST_ERROR("Invalid CNT: %.12f", target.cnt);
        return INVALID_PARAMETER;
    }

    if (target.type == MOTION_JOINT)
    {
        if (target.vel < MINIMUM_E6 || target.vel > 1 + MINIMUM_E6)
        {
            FST_ERROR("Invalid vel: %.6f", target.vel);
            return INVALID_PARAMETER;
        }

        if (!soft_constraint_.isJointInConstraint(target.joint_target))
        {
            char buffer[LOG_TEXT_SIZE];
            FST_ERROR("Target joint out of soft constraint.");
            FST_ERROR("  joint = %s", printDBLine(&target.joint_target[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("  upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            FST_ERROR("  lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
            return JOINT_OUT_OF_CONSTRAINT;
        }

        if (isSameJoint(start_joint_, target.joint_target, MINIMUM_E3))
        {
            char buffer[LOG_TEXT_SIZE];
            FST_WARN("Target joint coincidence with start joint.");
            FST_WARN("  start-joint = %s", printDBLine(&start_joint_[0], buffer, LOG_TEXT_SIZE));
            FST_WARN("  target-joint = %s", printDBLine(&target.joint_target[0], buffer, LOG_TEXT_SIZE));
            return TARGET_COINCIDENCE;
        }

        return SUCCESS;
    }
    else if (target.type == MOTION_LINE || target.type == MOTION_CIRCLE)
    {
        if (target.vel < cartesian_vel_min_ || target.vel > cartesian_vel_max_)
        {
            FST_ERROR("Invalid vel: %.6f", target.vel);
            return INVALID_PARAMETER;
        }

        Pose start_pose;
        kinematics_ptr_->forwardKinematicsInUser(start_joint_, start_pose);

        if (target.type == MOTION_LINE)
        {
            Pose target_pose = PoseEuler2Pose(target.pose_target);

            if (getDistance(start_pose.position, target_pose.position) < MINIMUM_E3 && 
                getOrientationAngle(start_pose.orientation, target_pose.orientation) < MINIMUM_E3)
            {
                FST_ERROR("Target pose coincidence with start.");
                FST_ERROR("  start  = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f",
                             start_pose.position.x, start_pose.position.y, start_pose.position.z,
                             start_pose.orientation.w, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z);
                FST_ERROR("  target = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f",
                             target_pose.position.x, target_pose.position.y, target_pose.position.z,
                             target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
                return TARGET_COINCIDENCE;
            }
        }
        else
        {
            Pose pose1 = PoseEuler2Pose(target.circle_target.pose1);
            Pose pose2 = PoseEuler2Pose(target.circle_target.pose2);

            if (getDistance(start_pose.position, pose1.position) < MINIMUM_E3 && 
                getOrientationAngle(start_pose.orientation, pose1.orientation) < MINIMUM_E3)
            {
                FST_ERROR("Middle pose of the circle coincidence with start.");
                FST_ERROR("  start  = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f",
                             start_pose.position.x, start_pose.position.y, start_pose.position.z,
                             start_pose.orientation.w, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z);
                FST_ERROR("  middle = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f",
                             pose1.position.x, pose1.position.y, pose1.position.z,
                             pose1.orientation.w, pose1.orientation.x, pose1.orientation.y, pose1.orientation.z);
                return TARGET_COINCIDENCE;
            }

            if (getDistance(start_pose.position, pose2.position) < MINIMUM_E3 &&
                getOrientationAngle(start_pose.orientation, pose2.orientation) < MINIMUM_E3)
            {
                FST_ERROR("Target pose of the circle coincidence with start.");
                FST_ERROR("  start  = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f",
                             start_pose.position.x, start_pose.position.y, start_pose.position.z,
                             start_pose.orientation.w, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z);
                FST_ERROR("  target = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f",
                             pose2.position.x, pose2.position.y, pose2.position.z,
                             pose2.orientation.w, pose2.orientation.x, pose2.orientation.y, pose2.orientation.z);
                return TARGET_COINCIDENCE;
            }
            
            if (getDistance(pose1.position, pose2.position) < MINIMUM_E3 &&
                getOrientationAngle(pose1.orientation, pose2.orientation) < MINIMUM_E3)
            {
                FST_ERROR("Middle pose coincidence with target pose of the circle.");
                FST_ERROR("  middle = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f",
                             pose1.position.x, pose1.position.y, pose1.position.z,
                             pose1.orientation.w, pose1.orientation.x, pose1.orientation.y, pose1.orientation.z);
                FST_ERROR("  target = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f",
                             pose2.position.x, pose2.position.y, pose2.position.z,
                             pose2.orientation.w, pose2.orientation.x, pose2.orientation.y, pose2.orientation.z);
                // TODO
                return INVALID_PARAMETER;
            }
        }

        return SUCCESS;
    }
    else
    {
        FST_ERROR("Invalid motion type: %d", target.type);
        return INVALID_PARAMETER;
    }
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

ErrorCode BaseGroup::autoJoint(const Joint &start, const MotionTarget &target, PathCacheList &path, TrajectoryCacheList &trajectory)
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
        return autoSmoothJoint(start_state, traj_cache.path_cache_ptr->target, target, path.path_cache, trajectory.trajectory_cache);
    }
    else
    {
        pthread_mutex_unlock(&cache_list_mutex_);
        trajectory.time_from_start = start_time;
        FST_INFO("Last motion without smooth, start from stable");
        return autoStableJoint(start, target, path.path_cache, trajectory.trajectory_cache);
    }
}

ErrorCode BaseGroup::autoStableJoint(const Joint &start, const MotionTarget &target, PathCache &path, TrajectoryCache &trajectory)
{
    char buffer[LOG_TEXT_SIZE];
    clock_t start_clock, end_clock;
    double  path_plan_time, traj_plan_time;

    FST_INFO("autoStableJoint: vel = %.4f, cnt = %.4f", target.vel, target.cnt);
    FST_INFO("  start  = %s", printDBLine(&start[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  target = %s", printDBLine(&target.joint_target[0], buffer, LOG_TEXT_SIZE));

    start_clock = clock();
    ErrorCode err = planPathJoint(start, target, path);
    end_clock = clock();
    path_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err == SUCCESS)
    {
        path.target = target;
        FST_INFO("Path plan success, total-point = %d, smooth-in = %d, smooth-out = %d",
                    path.cache_length, path.smooth_in_index, path.smooth_out_index);
    }
    else
    {
        FST_ERROR("Path plan failed, code = 0x%llx", err);
        return err;
    }

    JointState start_state;
    start_state.angle = start;
    memset(&start_state.omega, 0, sizeof(start_state.omega));
    memset(&start_state.alpha, 0, sizeof(start_state.alpha));
    start_clock = clock();
    err = planTrajectory(path, start_state, vel_ratio_, acc_ratio_, trajectory);
    end_clock = clock();
    traj_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err == SUCCESS)
    {
        FST_INFO("Trajectory plan success, total-segment = %d, smooth-out = %d,",
                    trajectory.cache_length, trajectory.smooth_out_index);
    }
    else
    {
        FST_ERROR("Trajectory plan failed, code = 0x%llx", err);
        return err;
    }

    FST_INFO("autoStableJoint success, path-plan: %.4f ms, traj-plan: %.4f ms", path_plan_time, traj_plan_time);
    return SUCCESS;
}

ErrorCode BaseGroup::autoSmoothJoint(const JointState &start_state,
                                     const MotionTarget &via,
                                     const MotionTarget &target,
                                     PathCache &path,
                                     TrajectoryCache &trajectory)
{
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("autoSmoothJoint: vel = %.4f, cnt = %.4f", target.vel, target.cnt);
    FST_INFO("  start angle = %s", printDBLine(&start_state.angle[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("        omega = %s", printDBLine(&start_state.omega[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("        alpha = %s", printDBLine(&start_state.alpha[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  via    = %s", printDBLine(&via.joint_target[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  target = %s", printDBLine(&target.joint_target[0], buffer, LOG_TEXT_SIZE));

    ErrorCode err = SUCCESS;
    //ErrorCode err = planPathSmoothJoint(start.angle, via.joint_target, target, path);

    if (err == SUCCESS)
    {
        path.target = target;
        FST_INFO("Path plan success, total-point = %d, smooth-in = %d, smooth-out = %d",
                    path.cache_length, path.smooth_in_index, path.smooth_out_index);
    }
    else
    {
        FST_ERROR("Path plan failed, code = 0x%llx", err);
        return err;
    }

    //err = planTrajectorySmooth(path, start_state, via, vel_ratio_, acc_ratio_, trajectory);

    if (err == SUCCESS)
    {
        FST_INFO("Trajectory plan success, total-segment = %d, smooth-out = %d,",
                    trajectory.cache_length, trajectory.smooth_out_index);
    }
    else
    {
        FST_ERROR("Trajectory plan failed, code = 0x%llx", err);
        return err;
    }

    FST_INFO("autoSmoothJoint success.");
    return SUCCESS;
}

ErrorCode BaseGroup::autoLine(const Joint &start, const MotionTarget &target, PathCacheList &path, TrajectoryCacheList &trajectory)
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
        return autoSmoothLine(start_state, traj_cache.path_cache_ptr->target, target, path.path_cache, trajectory.trajectory_cache);
    }
    else
    {
        pthread_mutex_unlock(&cache_list_mutex_);
        trajectory.time_from_start = start_time;
        FST_INFO("Last motion without smooth, start from stable");
        return autoStableLine(start, target, path.path_cache, trajectory.trajectory_cache);
    }
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

    for (size_t i = 0; i < path.cache_length; i++)
    {
        err = kinematics_ptr_->inverseKinematicsInUser(path.cache[i].pose, reference, path.cache[i].joint);

        if (err == SUCCESS)
        {
            if (soft_constraint_.isJointInConstraint(path.cache[i].joint))
            {
                reference = path.cache[i].joint;
                continue;
            }
            else
            {
                char buffer[LOG_TEXT_SIZE];
                Pose &pose = path.cache[i].pose;
                FST_ERROR("IK result out of soft constraint:");
                FST_ERROR("  pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]);
                FST_ERROR("  joint: %s", printDBLine(&path.cache[i].joint[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  upper: %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  lower: %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
                err = JOINT_OUT_OF_CONSTRAINT;
                break;
            }
        }
        else
        {
            Pose &pose = path.cache[i].pose;
            FST_ERROR("IK failed, code = 0x%llx", err);
            FST_ERROR("  pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]);
            break;
        }
    }

    return err;
}


ErrorCode BaseGroup::autoStableLine(const Joint &start, const MotionTarget &target, PathCache &path, TrajectoryCache &trajectory)
{
    char buffer[LOG_TEXT_SIZE];
    clock_t start_clock, end_clock;
    double  path_plan_time, path_ik_time, traj_plan_time;
    PoseEuler start_pose;
    kinematics_ptr_->forwardKinematicsInUser(start, start_pose);

    FST_INFO("autoStableLine: vel = %.4f, cnt = %.4f", target.vel, target.cnt);
    FST_INFO("  start joint = %s", printDBLine(&start[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  start pose  = %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", 
             start_pose.position.x, start_pose.position.y, start_pose.position.z, 
             start_pose.orientation.a, start_pose.orientation.b, start_pose.orientation.c);
    FST_INFO("  target pose = %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", 
             target.pose_target.position.x, target.pose_target.position.y, target.pose_target.position.z, 
             target.pose_target.orientation.a, target.pose_target.orientation.b, target.pose_target.orientation.c);

    start_clock = clock();
    ErrorCode err = planPathLine(start_pose, target, path);
    end_clock = clock();
    path_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err == SUCCESS)
    {
        FST_INFO("Path plan success, total-point = %d, smooth-in = %d, smooth-out = %d, inverse kinematics ...",
                      path.cache_length, path.smooth_in_index, path.smooth_out_index);

        start_clock = clock();
        err = computeInverseKinematicsOnPathCache(start, path);
        end_clock = clock();
        path_ik_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

        if (err == SUCCESS)
        {
            path.target = target;
            FST_INFO("Inverse kinematics success, plan trajectory ...");
        }
        else
        {
            FST_ERROR("Inverse kinematics failed, code = 0x%llx", err);
            return err;
        }
    }
    else
    {
        FST_ERROR("Path plan failed, code = 0x%llx", err);
        return err;
    }

    JointState start_state;
    start_state.angle = start;
    memset(&start_state.omega, 0, sizeof(start_state.omega));
    memset(&start_state.alpha, 0, sizeof(start_state.alpha));
    start_clock = clock();
    err = planTrajectory(path, start_state, vel_ratio_, acc_ratio_, trajectory);
    end_clock = clock();
    traj_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err == SUCCESS)
    {
        FST_INFO("Trajectory plan success, total-segment = %d, smooth-out = %d,",
                    trajectory.cache_length, trajectory.smooth_out_index);
    }
    else
    {
        FST_ERROR("Trajectory plan failed, code = 0x%llx", err);
        return err;
    }

    FST_INFO("autoStableLine success, path-plan: %.4f ms, path-ik: %.4f ms, traj-plan: %.4f ms", path_plan_time, path_ik_time, traj_plan_time);
    return SUCCESS;
}

ErrorCode BaseGroup::autoSmoothLine(const JointState &start_state,
                                    const MotionTarget &via,
                                    const MotionTarget &target,
                                    PathCache &path,
                                    TrajectoryCache &trajectory)
{
    char buffer[LOG_TEXT_SIZE];
    clock_t start_clock, end_clock;
    double  path_plan_time, path_ik_time, traj_plan_time;
    PoseEuler start_pose;
    kinematics_ptr_->forwardKinematicsInUser(start_state.angle, start_pose);

    FST_INFO("autoSmoothLine: vel = %.4f, cnt = %.4f", target.vel, target.cnt);
    FST_INFO("  start joint = %s", printDBLine(&start_state.angle[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  start omega = %s", printDBLine(&start_state.omega[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  start alpha = %s", printDBLine(&start_state.alpha[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  start pose  = %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", 
             start_pose.position.x, start_pose.position.y, start_pose.position.z, 
             start_pose.orientation.a, start_pose.orientation.b, start_pose.orientation.c);
    FST_INFO("  target pose = %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", 
             target.pose_target.position.x, target.pose_target.position.y, target.pose_target.position.z, 
             target.pose_target.orientation.a, target.pose_target.orientation.b, target.pose_target.orientation.c);

    start_clock = clock();
    ErrorCode err = planPathSmoothLine(start_pose, via, target, path);
    end_clock = clock();
    path_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err == SUCCESS)
    {
        FST_INFO("Path plan success, total-point = %d, smooth-in = %d, smooth-out = %d, inverse kinematics ...",
                      path.cache_length, path.smooth_in_index, path.smooth_out_index);

        start_clock = clock();
        err = computeInverseKinematicsOnPathCache(start_state.angle, path);
        end_clock = clock();
        path_ik_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

        if (err == SUCCESS)
        {
            path.target = target;
            FST_INFO("Inverse kinematics success, plan trajectory ...");
        }
        else
        {
            FST_ERROR("Inverse kinematics failed, code = 0x%llx", err);
            return err;
        }
    }
    else
    {
        FST_ERROR("Path plan failed, code = 0x%llx", err);
        return err;
    }

    start_clock = clock();
    err = planTrajectorySmooth(path, start_state, via, vel_ratio_, acc_ratio_, trajectory);
    end_clock = clock();
    traj_plan_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;

    if (err == SUCCESS)
    {
        FST_INFO("Trajectory plan success, total-segment = %d, smooth-out = %d,",
                    trajectory.cache_length, trajectory.smooth_out_index);
    }
    else
    {
        FST_ERROR("Trajectory plan failed, code = 0x%llx", err);
        return err;
    }

    FST_INFO("autoSmoothLine success, path-plan: %.4f ms, path-ik: %.4f ms, traj-plan: %.4f ms", path_plan_time, path_ik_time, traj_plan_time);
    return SUCCESS;
}


// TODO
ErrorCode BaseGroup::autoCircle(const Joint &start, const MotionTarget &target, PathCacheList &path, TrajectoryCacheList &trajectory)
{
    return SUCCESS;
}


ErrorCode BaseGroup::autoStableCircle(const Joint &start, const MotionTarget &target, PathCache &path, TrajectoryCache &trajectory)
{
    return SUCCESS;
}


ErrorCode BaseGroup::autoSmoothCircle(const JointState &start_state,
                                      const MotionTarget &via,
                                      const MotionTarget &target,
                                      PathCache &path,
                                      TrajectoryCache &trajectory)
{
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
                return MOTION_INTERNAL_FAULT;
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

ErrorCode BaseGroup::setToolFrame(const PoseEuler &tf)
{
    return kinematics_ptr_->setToolFrame(tf);
}

ErrorCode BaseGroup::setUserFrame(const PoseEuler &uf)
{
    return kinematics_ptr_->setUserFrame(uf);
}

ErrorCode BaseGroup::setWorldFrame(const PoseEuler &wf)
{
    return kinematics_ptr_->setUserFrame(wf);
}

ErrorCode BaseGroup::pickPointsFromManualTrajectory(TrajectoryPoint *points, size_t &length)
{
    ErrorCode err = (manual_traj_.frame == JOINT || manual_traj_.mode == APOINT) ? pickPointsFromManualJoint(points, length) : pickPointsFromManualCartesian(points, length);

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
    
    FST_INFO("Pick from manual joint, manual-time = %.4f", manual_time_);

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
    PoseEuler pose;
    double tim, vel;
    double *axis_ptr, *start_ptr, *target_ptr;
    size_t picked_num = 0;

    FST_INFO("Pick from manual cartesian, manual-time = %.4f", manual_time_);

    for (size_t i = 0; i < length; i++)
    {
        points[i].level = manual_time_ > MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
        memset(&points[i].omega, 0, sizeof(Joint));
        memset(&points[i].alpha, 0, sizeof(Joint));
        memset(&points[i].ma_cv_g, 0, sizeof(Joint));

        axis_ptr   = (double *) &pose[0];
        start_ptr  = (double *) &manual_traj_.cart_start[0];
        target_ptr = (double *) &manual_traj_.cart_ending[0];

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

        Joint ref_joint = getLatestJoint();

        switch (manual_traj_.frame)
        {
            case BASE:
                err = kinematics_ptr_->inverseKinematicsInBase(pose, ref_joint, points[i].angle);
                break;
            case USER:
                err = kinematics_ptr_->inverseKinematicsInUser(pose, ref_joint, points[i].angle);
                break;
            case WORLD:
                err = kinematics_ptr_->inverseKinematicsInWorld(pose, ref_joint, points[i].angle);
                break;
            case TOOL:
                err = kinematics_ptr_->inverseKinematicsInTool(manual_traj_.tool_coordinate, pose, ref_joint, points[i].angle);
                break;
            case JOINT:
            default:
                err = MOTION_INTERNAL_FAULT;
                break;
        }

        if (err == SUCCESS && soft_constraint_.isJointInConstraint(points[i].angle))
        {
            picked_num ++;

            if (manual_time_ >= manual_traj_.duration)
            {
                points[i].level = POINT_ENDING;
                break;
            }
        }
        else
        {
            if (err != SUCCESS)
            {
                FST_ERROR("pickFromManualCartesian: IK failed.");
                break;
            }
            else
            {
                FST_ERROR("pickFromManualCartesian: IK result out of soft constraint.");
                err = JOINT_OUT_OF_CONSTRAINT;
                break;
            }
        }
    }

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

    auto servo_state = getServoState();

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

            if (stop_request_)
            {
                stop_request_ = false;
                error_request_ = true;
            }

            break;
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
                FST_WARN("Auto to standby timeout, error-request asserted.");
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
                FST_WARN("Manual to standby timeout, error-request asserted.");
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
                FST_WARN("Group-state is unknow but servo-state is %d, send stop request to servo.", servo_state);
                bare_core_.stopBareCore();
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
    }

    if (servo_state == SERVO_DISABLE && group_state_ == AUTO)
    {
        FST_ERROR("Group-state is AUTO but servo-state is diabale, switch group-state to UNKNOW");
        group_state_ = UNKNOW;
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

        if (isSameJoint(current_joint, control_joint, MINIMUM_E3))
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
    if (group_state_ == AUTO || group_state_ == STANDBY_TO_AUTO)
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
                        if (traj_cache.smooth_out_index < (int)traj_cache.cache_length)
                        {
                            traj_cache.smooth_out_index = traj_cache.cache_length;
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
    path_list_ptr_ = path_list_ptr_->next_ptr;
    traj_list_ptr_ = traj_list_ptr_->next_ptr;
    path_cache_pool_.freeCachePtr(path_ptr);
    traj_cache_pool_.freeCachePtr(traj_ptr);
}

void BaseGroup::setFineWaiter(void)
{
    Pose target;
    auto &block = path_list_ptr_->path_cache.cache[path_list_ptr_->path_cache.cache_length - 1];

    if (block.motion_type == MOTION_JOINT)
    {
        kinematics_ptr_->forwardKinematicsInUser(block.joint, target);
    }
    else
    {
        target = block.pose;
    }

    FST_INFO("setFineWaiter: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f, %.4f", target[0], target[1], target[2], target[3], target[4], target[5], target[6]);
    fine_waiter_.enableWaiter(target);
}

void BaseGroup::loopFineWaiter(void)
{
    if (fine_waiter_.isEnable() && group_state_ == AUTO_TO_STANDBY && servo_state_ == SERVO_IDLE)
    {  
        Pose   barecore_pose;
        kinematics_ptr_->forwardKinematicsInUser(getLatestJoint(), barecore_pose);
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
    doStateMachine();
    fillTrajectoryFifo();
}

void BaseGroup::updateServoStateAndJoint(void)
{
    static ServoState   barecore_state = SERVO_INIT;
    static Joint        barecore_joint = {0};
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
            auto_to_standby_request_ = true;
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

    if (group_state_ == AUTO && !auto_to_standby_request_)
    {
        err = sendAutoTrajectoryFlow();
    }
    else if (group_state_ == MANUAL && !manual_to_standby_request_)
    {
        err = sendManualTrajectoryFlow();
    }
    else if (group_state_ == AUTO_TO_STANDBY || group_state_ == MANUAL_TO_STANDBY)
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

BaseKinematics* BaseGroup::getKinematicsPtr(void)
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

void FineWaiter::enableWaiter(const Pose &target)
{
    waiting_pose_ = target;
    stable_cnt_ = 0;
    enable_ = true;
}

void FineWaiter::disableWaiter(void)
{
    enable_ = false;
}

void FineWaiter::checkWaiter(const Pose &pose)
{
    if (getDistance(pose.position, waiting_pose_.position) < threshold_)
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
