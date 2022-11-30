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
#include <sys/file.h> 
#include <sys/mman.h>

#include <motion_control_base_group.h>
#include <common_file_path.h>
#include <basic_alg.h>
#include "yaml_help.h"
#include "log_manager_producer.h"

using namespace std;
using namespace group_space;
using namespace basic_alg;
using namespace base_space;
using namespace log_space;




namespace group_space
{

BaseGroup::BaseGroup()
{
    servo_state_ = SERVO_INIT;
    mc_state_ = STANDBY;
    auto_time_ = 0;
    manual_time_ = 0;

    dynamics_ptr_ = NULL;
    kinematics_ptr_ = NULL;

    id_ = -1;
    vel_ratio_ = 0;
    acc_ratio_ = 0;

    fine_cycle_ = 0;
    fine_threshold_ = 10;

    clear_request_ = false;
    stop_barecore_ = false;
    clear_teach_request_ = false;
    standby_to_offline_request_ = false;
    auto_to_pause_request_ = false;
    pause_to_auto_request_ = false;
    manual_to_pause_request_ = false;
    pause_to_manual_request_ = false;
    standby_to_auto_request_ = false;
    standby_to_manual_request_ = false;
    standby_to_online_request_ = false;
    online_to_standby_request_ = false;
    online_to_pause_request_ = false;
    auto_to_standby_request_ = false;
    offline_to_standby_request_ = false;
    manual_to_standby_request_ = false;
    pausing_to_pause_request_ = false;
    pause_return_to_pause_request_ = false;
    filling_points_into_traj_fifo_ = false;
    manual_trajectory_check_fail_ = false;
    start_of_motion_ = false;
    traj_log_enable_ = false;
    memset(&user_frame_, 0, sizeof(user_frame_));
    memset(&tool_frame_, 0, sizeof(tool_frame_));
    memset(&world_frame_, 0, sizeof(world_frame_));
    is_continuous_manual_move_timeout_ = false;
    is_continuous_manual_time_count_valid_ = false;
    memset(&last_continuous_manual_move_rpc_time_, 0, sizeof(struct timeval));
}

BaseGroup::~BaseGroup()
{
    if (dynamics_ptr_ != NULL) {delete dynamics_ptr_; dynamics_ptr_ = NULL;}
    if (kinematics_ptr_ != NULL) {delete kinematics_ptr_; kinematics_ptr_ = NULL;}
}


ErrorCode BaseGroup::manualMoveToPoint(const IntactPoint &point)
{
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_base","Manual to target point");
    MotionControlState mc_state = mc_state_;
    ServoState servo_state = getServoState();
    //检查可执行状态
    if ((mc_state != STANDBY && mc_state != PAUSE) || servo_state != SERVO_IDLE)
    {
        LogProducer::error("mc_base","Cannot manual to target in current MC-state = %s, servo-state = %s", 
            getMontionControlStatusString(mc_state).c_str(), getMCServoStatusString(servo_state).c_str());
        return MC_FAIL_MANUAL_TO_POINT;
    }
    //如果起始点位和目标点位不在限位附近
    start_joint_ = getLatestJoint();
    Joint start_joint = start_joint_;
    LogProducer::info("mc_base","Joint: %s", printDBLine(&point.joint[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", point.pose.pose.point_.x_, point.pose.pose.point_.y_, point.pose.pose.point_.z_, point.pose.pose.euler_.a_, point.pose.pose.euler_.b_, point.pose.pose.euler_.c_);
    LogProducer::info("mc_base","Posture: %d, %d, %d, %d", point.pose.posture.arm, point.pose.posture.elbow, point.pose.posture.wrist, point.pose.posture.flip);
    LogProducer::info("mc_base","UserFrame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", point.user_frame.point_.x_, point.user_frame.point_.y_, point.user_frame.point_.z_, point.user_frame.euler_.a_, point.user_frame.euler_.b_, point.user_frame.euler_.c_);
    LogProducer::info("mc_base","ToolFrame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", point.tool_frame.point_.x_, point.tool_frame.point_.y_, point.tool_frame.point_.z_, point.tool_frame.euler_.a_, point.tool_frame.euler_.b_, point.tool_frame.euler_.c_);
    LogProducer::info("mc_base","Start-joint = %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));
    if (!soft_constraint_.isJointInConstraint(start_joint, MINIMUM_E3))
    {
        LogProducer::error("mc_base","Start-joint is out of soft constraint, manual-mode-apoint is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (!soft_constraint_.isJointInConstraint(point.joint))
    {
        LogProducer::error("mc_base","Target-joint out of constraint: %s", printDBLine(&point.joint[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    manual_time_ = 0;
    pthread_mutex_lock(&manual_traj_mutex_);
    ErrorCode err = manual_teach_.manualToJoint(start_joint, point.joint);
    double duration = manual_teach_.getDuration();
    pthread_mutex_unlock(&manual_traj_mutex_);
    //切换MC状态机，到manual状态后由实时线程下发轨迹.
    if (err == SUCCESS)
    {
        LogProducer::info("mc_base","Manual move to target joint, total-duration = %.4f, Success.", duration);
        if (duration > MINIMUM_E6 && mc_state == STANDBY) standby_to_manual_request_ = true;
        else if (duration > MINIMUM_E6 && mc_state == PAUSE) pause_to_manual_request_ = true;
        return SUCCESS;
    }
    else
    {
        LogProducer::error("mc_base","Fail to create manual trajectory, error-code = 0x%llx", err);
        return err;
    }
}

ErrorCode BaseGroup::manualMoveStep(const ManualDirection *direction)
{
    LogProducer::info("mc_base","Manual step by direction.");
    MotionControlState mc_state = mc_state_;
    ServoState servo_state = getServoState();
    //检查可执行状态
    if ((mc_state != STANDBY && mc_state != PAUSE) || servo_state != SERVO_IDLE)
    {
        LogProducer::error("mc_base","Cannot manual step in current MC-state = %s, servo-state = %s", 
            getMontionControlStatusString(mc_state).c_str(), getMCServoStatusString(servo_state).c_str());
        return MC_FAIL_MANUAL_STEP;
    }

    start_joint_ = getLatestJoint();
    Joint start_joint = start_joint_;
    //如果在限位附近，只能关节坐标系，关节向内运动
    if (!soft_constraint_.isJointInConstraint(start_joint, MINIMUM_E3))
    {
        if (manual_teach_.getManualFrame() != JOINT)
        {
            char buffer[LOG_TEXT_SIZE];
            LogProducer::error("mc_base","Start-joint = %s", printDBLine(&start_joint.j1_, buffer, LOG_TEXT_SIZE));
            LogProducer::error("mc_base","Start-joint is out of soft constraint, cannot manual in cartesian space.");
            return MC_FAIL_MANUAL_STEP;
        }

        for (size_t i = 0; i < getNumberOfJoint(); i++)
        {
            if (start_joint[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
            {
                LogProducer::error("mc_base","J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                            i + 1, start_joint[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                return MC_FAIL_MANUAL_STEP;
            }
            else if (start_joint[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
            {
                LogProducer::error("mc_base","J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                            i + 1, start_joint[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                return MC_FAIL_MANUAL_STEP;
            }
        }
    }
    //规划路径
    pthread_mutex_lock(&manual_traj_mutex_);
    manual_time_ = 0;
    ErrorCode err = manual_teach_.manualStep(direction, start_joint);
    double duration = manual_teach_.getDuration();
    pthread_mutex_unlock(&manual_traj_mutex_);
    //切换MC状态机，到manual状态后由实时线程下发轨迹.
    if (err == SUCCESS)
    {
        LogProducer::info("mc_base","Manual move step, total-duration = %.4f, Success.", duration);
        if (duration > MINIMUM_E6 && mc_state == STANDBY) standby_to_manual_request_ = true;
        else if (duration > MINIMUM_E6 && mc_state == PAUSE) pause_to_manual_request_ = true;
        return SUCCESS;
    }
    else
    {
        LogProducer::error("mc_base","Fail to create manual trajectory, error-code = 0x%llx", err);
        return err;
    }
}

ErrorCode BaseGroup::manualMoveContinuous(const ManualDirection *direction)
{
    if (!updateContinuousManualMoveRpcTime())
    {
        LogProducer::error("mc_base","Cannot manual continuous for last movement timeout");
        return MC_FAIL_MANUAL_CONTINUOUS;
    }

    MotionControlState mc_state = mc_state_;
    ServoState servo_state = getServoState();
    size_t joint_num = getNumberOfJoint();
    //检查可执行状态
    if ((mc_state != STANDBY && mc_state != MANUAL && mc_state != PAUSE && mc_state != PAUSE_MANUAL) || ((mc_state == STANDBY || mc_state == PAUSE) && servo_state != SERVO_IDLE))
    {
        LogProducer::error("mc_base","Cannot manual continuous in current mc-state = %s, servo-state = %s", getMontionControlStatusString(mc_state).c_str(), getMCServoStatusString(servo_state).c_str());
        return MC_FAIL_MANUAL_CONTINUOUS;
    }
    if (mc_state != STANDBY && mc_state != PAUSE && manual_trajectory_check_fail_)
    {
        LogProducer::error("mc_base","Manual trajectory check failed, manual continuous refused");
        return MC_FAIL_MANUAL_CONTINUOUS;
    }

    start_joint_ = getLatestJoint();
    Joint start_joint = start_joint_;
    //如果在限位附近，只能关节坐标系，关节向内运动
    if (!soft_constraint_.isJointInConstraint(start_joint, MINIMUM_E3))
    {
        if (manual_teach_.getManualFrame() != JOINT)
        {
            char buffer[LOG_TEXT_SIZE];
            LogProducer::error("mc_base","Start-joint = %s", printDBLine(&start_joint.j1_, buffer, LOG_TEXT_SIZE));
            LogProducer::error("mc_base","Start-joint is out of soft constraint, cannot manual in cartesian space.");
            return MC_FAIL_MANUAL_CONTINUOUS;
        }

        for (size_t i = 0; i < joint_num; i++)
        {
            if (start_joint[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
            {
                LogProducer::error("mc_base","J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                            i + 1, start_joint[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                return MC_FAIL_MANUAL_CONTINUOUS;
            }
            else if (start_joint[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
            {
                LogProducer::error("mc_base","J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                            i + 1, start_joint[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                return MC_FAIL_MANUAL_CONTINUOUS;
            }
        }
    }

    if (mc_state_ == STANDBY || mc_state == PAUSE)
    {
        //规划路径
        LogProducer::info("mc_base","Manual continuous by direction.");
        pthread_mutex_lock(&manual_traj_mutex_);
        ErrorCode err = manual_teach_.manualContinuous(direction, start_joint);
        double duration = manual_teach_.getDuration();
        pthread_mutex_unlock(&manual_traj_mutex_);

        if (err != SUCCESS)
        {
            LogProducer::error("mc_base","Fail to create manual trajectory, error-code = 0x%llx", err);
            return err;
        }
        //切换MC状态机，到manual状态后由实时线程下发轨迹.
        LogProducer::info("mc_base","Manual move continuous, total-duration = %.4f, Success.", duration);
        if (duration > MINIMUM_E6 && mc_state == STANDBY) standby_to_manual_request_ = true;
        else if (duration > MINIMUM_E6 && mc_state == PAUSE) pause_to_manual_request_ = true;
        return SUCCESS;
        
    }
    else if (mc_state_ == MANUAL || mc_state == PAUSE_MANUAL)
    {
        //if (manual_time_ < 0.15)
        //{
        //    // 示教起始的150ms用于填充下层的轨迹FIFO，这段时间之内做轨迹合成可能由于下层FIFO不足导致轨迹断掉
        //    return SUCCESS;
        //}
        bool direction_changed = false;
        bool all_standing = true;
        ManualDirection old_direction[NUM_OF_JOINT];
        pthread_mutex_lock(&manual_traj_mutex_);
        manual_teach_.getDirection(old_direction);
        ErrorCode err = manual_teach_.manualContinuous(direction, manual_time_);
        
        for (uint32_t j = 0; j < getNumberOfJoint(); j++)
        {
            if (old_direction[j] != direction[j])
            {
                direction_changed = true;
            }

            if (direction[j] != STANDING)
            {
                all_standing = false;
            }
        }

        if (manual_teach_.getManualFrame() != JOINT && manual_teach_.getManualMode() != APOINT && direction_changed && !all_standing)
        {
            // 示教轨迹有更新，需要重新检查轨迹
            double start_time = manual_time_;
            double end_time = manual_time_ + 0.5 > manual_teach_.getDuration() ? manual_teach_.getDuration() : manual_time_ + 0.5;
            double step_time = cycle_time_ * 10;
            err = checkManualTrajectory(start_time, end_time, step_time, manaul_reference_);

            if (err != SUCCESS)
            {
                LogProducer::error("mc_base","Fail to check manual trajectory, error-code = 0x%llx", err);
                manualStop();
                manual_trajectory_check_fail_ = true;
                pthread_mutex_unlock(&manual_traj_mutex_);
                return err;
            }
        }
        
        pthread_mutex_unlock(&manual_traj_mutex_);
        return err;
    }
    else
    {
        LogProducer::error("mc_base","Cannot manual continuous in current grp-state = %s, servo-state = %s", getMontionControlStatusString(mc_state_).c_str(), getMCServoStatusString(servo_state_).c_str());
        return MC_FAIL_MANUAL_CONTINUOUS;
    }
}

bool BaseGroup::updateContinuousManualMoveRpcTime()
{
    if(is_continuous_manual_move_timeout_)
    {
        return false;
    }
    is_continuous_manual_time_count_valid_ = true;
    
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    pthread_mutex_lock(&manual_rpc_mutex_);
    last_continuous_manual_move_rpc_time_.tv_sec = current_time.tv_sec;
    last_continuous_manual_move_rpc_time_.tv_usec = current_time.tv_usec;
    pthread_mutex_unlock(&manual_rpc_mutex_);
    return true;
}

void BaseGroup::handleContinueousManualRpcTimeOut()
{
    if(is_continuous_manual_move_timeout_)
    {
        return;
    }

    if(!is_continuous_manual_time_count_valid_)
    {
        return;
    }
    
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    pthread_mutex_lock(&manual_rpc_mutex_);
    long long delta_tv_sec = current_time.tv_sec - last_continuous_manual_move_rpc_time_.tv_sec;
    long long delta_tv_usec = current_time.tv_usec - last_continuous_manual_move_rpc_time_.tv_usec;
    pthread_mutex_unlock(&manual_rpc_mutex_);
    long long time_elapse = delta_tv_sec * 1000000 + delta_tv_usec;
    if(time_elapse > 1000000)//1 second
    {
        LogProducer::warn("mc_base","doContinuousManualMove receive time out, do manual to standstill.");
        is_continuous_manual_move_timeout_ = true;
        GroupDirection direction;
        direction.axis1 = STANDING;
        direction.axis2 = STANDING;
        direction.axis3 = STANDING;
        direction.axis4 = STANDING;
        direction.axis5 = STANDING;
        direction.axis6 = STANDING;
        direction.axis7 = STANDING;
        direction.axis8 = STANDING;
        direction.axis9 = STANDING;        
        ErrorCode error_code = manualMoveContinuous(&direction[0]);
        if(error_code != SUCCESS)
        {
            ErrorQueue::instance().push(error_code);
        }
    }
}

void BaseGroup::manualStopWithLock(void)
{
    LogProducer::info("mc_base","Manual stop with lock");
    pthread_mutex_lock(&manual_traj_mutex_);
    manualStop();
    pthread_mutex_unlock(&manual_traj_mutex_);
    LogProducer::info("mc_base","Lock stop success");
}

void BaseGroup::manualStop(void)
{
    MotionControlState mc_state = mc_state_;
    LogProducer::info("mc_base","Stop manual teach, grp-state: %s, manual-mode: %d, manual-frame: %d", getMontionControlStatusString(mc_state).c_str(), manual_teach_.getManualMode(), manual_teach_.getManualFrame());

    if ((mc_state == MANUAL || mc_state == STANDBY_TO_MANUAL || mc_state == PAUSE_MANUAL || mc_state == PAUSE_TO_PAUSE_MANUAL) && manual_time_ < manual_teach_.getDuration())
    {
        manual_teach_.manualStop(manual_time_);
        LogProducer::info("mc_base","Success, the group will stop in %.4fs", manual_teach_.getDuration() - manual_time_);
    }
    else
    {
        LogProducer::info("mc_base","The group is not in manual state, MC-state: %s, manual-time: %.6f, manual-duration: %.6f", 
            getMontionControlStatusString(mc_state).c_str(), manual_time_, manual_teach_.getDuration());
    }
}

bool BaseGroup::isMoving(void)
{
    ServoState servo_state = getServoState();
    MotionControlState mc_state = mc_state_;
    return servo_state == SERVO_RUNNING && (mc_state == AUTO || mc_state == AUTO_TO_PAUSING || mc_state == PAUSING || 
            mc_state == PAUSE_RETURN || mc_state == RESUME ||
            mc_state == MANUAL || mc_state == PAUSE_MANUAL || mc_state == OFFLINE || mc_state == ONLINE);
}

ErrorCode BaseGroup::pauseMove(void)
{
    MotionControlState mc_state = mc_state_;
    LogProducer::info("mc_base","Pause move request received.");

    if (mc_state == AUTO && !auto_to_standby_request_ && !auto_to_pause_request_)
    {
        auto_to_pause_request_ = true;
        return SUCCESS;
    }
    else if (mc_state == MANUAL || mc_state == STANDBY_TO_MANUAL || mc_state == PAUSE_MANUAL || mc_state == PAUSE_TO_PAUSE_MANUAL)
    {
        manualStopWithLock();
        return SUCCESS;
    }
    else if (mc_state == PAUSE_RETURN || mc_state == PAUSE_TO_PAUSE_RETURN)
    {
        stop_barecore_ = true;
        return SUCCESS;
    }
    else if(mc_state == OFFLINE && !offline_to_pause_request_)
    {
        ErrorCode err = planOfflinePause();
        if (err != SUCCESS) return err;

        offline_to_pause_request_ = true;
        return SUCCESS;
    }
    else
    {}

    LogProducer::warn("mc_base","MC-state is %s, pause request refused.", getMontionControlStatusString(mc_state).c_str());
    return INVALID_SEQUENCE;
}


ErrorCode BaseGroup::planPauseTrajectory(void)
{
    TrajectoryPoint traj_point;
    vector<double> time_stamps(traj_fifo_.size());
    vector<double>::iterator time_stamps_iterator = time_stamps.begin();
    vector<JointState> trajectory(traj_fifo_.size());
    vector<JointState>::iterator trajectory_iterator = trajectory.begin();
    LogProducer::info("mc_base","Plan pause trajectory, %d points in trajectory FIFO", traj_fifo_.size());

    while (!traj_fifo_.empty())
    {
        traj_fifo_.fetch(traj_point);
        *trajectory_iterator = traj_point.state;
        ++trajectory_iterator;
        *time_stamps_iterator = traj_point.time_stamp;
        ++time_stamps_iterator;
    }

    ErrorCode err = SUCCESS;
    uint32_t pause_at = 0;
    double ratio = 0;

    /*
    uint32_t joint_num = getNumberOfJoint();
    double omega_max[] = {5.8118, 4.6600, 5.8118, 7.8540, 7.0686, 10.5592, 0.0, 0.0, 0.0};
    for (uint32_t i = 0; i < joint_num; i++)
    {
        if (fabs(trajectory[0].omega[i] / omega_max[i]) > ratio)
        {
            ratio = fabs(trajectory[0].omega[i] / omega_max[i]);
        }
    }
    */

    ratio = 0.5;
    err = pause_resume_planner_.planPauseTrajectory(ratio, trajectory, pause_trajectory_, pause_at);

    while (err != SUCCESS && ratio < 3 + MINIMUM_E6)
    {
        ratio += 0.5;
        LogProducer::info("mc_base","Alpha ratio: %.2f", ratio);
        err = pause_resume_planner_.planPauseTrajectory(ratio, trajectory, pause_trajectory_, pause_at);
    }

    if (err != SUCCESS)
    {
        traj_point.level = POINT_MIDDLE;
        time_stamps_iterator = time_stamps.begin();

        for (vector<JointState>::iterator it = trajectory.begin(); it != trajectory.end(); ++it)
        {
            traj_point.time_stamp = *time_stamps_iterator;
            time_stamps_iterator++;
            traj_point.state = *it;
            traj_fifo_.push(traj_point);
        }

        LogProducer::error("mc_base","Fail to plan pause trajectory, code: 0x%llx", err);
        return err;
    }

    size_t num = 0;
    size_t pause_trajectory_size = pause_trajectory_.size();
    LogProducer::info("mc_base","Pause trajectory ready with %d points, pause at %d", pause_trajectory_size, pause_at);
    traj_point.level = POINT_MIDDLE;
    pause_trajectory_time_stamp_ = time_stamps[0];

    for (uint32_t i = 0; i < pause_trajectory_size && !traj_fifo_.full(); ++i)
    {
        traj_point.time_stamp = pause_trajectory_time_stamp_;
        pause_trajectory_time_stamp_ += cycle_time_;
        traj_point.state = pause_trajectory_[i];
        dynamics_ptr_->getTorqueInverseDynamics(traj_point.state.angle, *(JointVelocity*)(&traj_point.state.omega), *(JointAcceleration*)(&traj_point.state.alpha), *(JointTorque*)(&traj_point.state.torque));
        traj_fifo_.push(traj_point);
        num++;
    }

    // 暂停后剩余的轨迹点保留，等待暂停恢复时使用
    uint32_t trajectory_size = trajectory.size();
    remain_trajectory_.clear();

    for (uint32_t i = pause_at; i < trajectory_size; i++)
    {
        remain_trajectory_.push_back(trajectory[i]);
    }

    char buffer[LOG_TEXT_SIZE];
    start_joint_before_pause_ = start_joint_;
    start_joint_ = pause_trajectory_.back().angle;
    pause_joint_ = trajectory[pause_at].angle;
    pause_trajectory_.erase(pause_trajectory_.begin(), pause_trajectory_.begin() + num);
    LogProducer::info("mc_base","Fill trajectory fifo finished, %d points fill into trajectory fifo, %d points left", num, pause_trajectory_.size());
    LogProducer::info("mc_base","Group will pause at: %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}

ErrorCode BaseGroup::planPauseReturnTrajectory(void)
{
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_base","Start joint different with pause joint, plan pause return trajectory.");
    LogProducer::info("mc_base","Start: %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","Pause: %s", printDBLine(&pause_joint_.j1_, buffer, LOG_TEXT_SIZE));
    resume_trajectory_.clear();

    if (!soft_constraint_.isJointInConstraint(start_joint_, MINIMUM_E3))
    {
        LogProducer::error("mc_base","Start joint out of soft constraint.");
        LogProducer::error("mc_base","Joint = %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
        LogProducer::error("mc_base","Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
        LogProducer::error("mc_base","Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (!soft_constraint_.isJointInConstraint(pause_joint_))
    {
        LogProducer::error("mc_base","Pause joint out of soft constraint.");
        LogProducer::error("mc_base","Joint = %s", printDBLine(&pause_joint_.j1_, buffer, LOG_TEXT_SIZE));
        LogProducer::error("mc_base","Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
        LogProducer::error("mc_base","Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    joint_planner_.planTrajectory(start_joint_, pause_joint_, 0.125, 0.5, 0.125, 0.125);
    LogProducer::info("mc_base","duration: %.6f", joint_planner_.getDuration());
    double duration = joint_planner_.getDuration() + cycle_time_;
    JointState sample_state;

    for (double sample_time = cycle_time_; sample_time < duration; sample_time += cycle_time_)
    {
        joint_planner_.sampleTrajectory(sample_time, sample_state);
        resume_trajectory_.push_back(sample_state);
    }

    LogProducer::info("mc_base","Return to pause position: %d points", resume_trajectory_.size());
    resume_trajectory_time_stamp_ = cycle_time_;
    start_joint_ = pause_joint_;

    TrajectoryPoint point;
    point.level = POINT_START;
    uint32_t insert_index = 0;
    uint32_t trajectory_size = resume_trajectory_.size();

    while (!traj_fifo_.full() && insert_index < trajectory_size)
    {
        point.time_stamp = resume_trajectory_time_stamp_;
        resume_trajectory_time_stamp_ += cycle_time_;
        point.state = resume_trajectory_[insert_index];
        dynamics_ptr_->getTorqueInverseDynamics(point.state.angle, *(JointVelocity*)(&point.state.omega), *(JointAcceleration*)(&point.state.alpha), *(JointTorque*)(&point.state.torque));
        traj_fifo_.push(point);
        insert_index++;
        point.level = POINT_MIDDLE;
    }

    resume_trajectory_.erase(resume_trajectory_.begin(), resume_trajectory_.begin() + insert_index);
    LogProducer::info("mc_base","Total %d points, %d points filled into trajectory fifo, %d points left", trajectory_size, insert_index, resume_trajectory_.size());
    return SUCCESS;
}


ErrorCode BaseGroup::planResumeTrajectory(void)
{
    TrajectoryPoint point;
    origin_trajectory_.clear();
    uint32_t origin_trajectory_size = traj_fifo_.size();
    origin_trajectory_.resize(origin_trajectory_size);
    LogProducer::info("mc_base","Plan resume trajectory.");

    for (uint32_t i = 0; i < origin_trajectory_size; i++)
    {
        traj_fifo_.fetch(point);
        origin_trajectory_[i] = point.state;
    }

    //char buffer[LOG_TEXT_SIZE];
    //
    //for (uint32_t i = 0; i < origin_trajectory_.size(); ++i)
    //{
    //    LogProducer::info("mc_base","origin-%d: %s", i, printDBLine(&origin_trajectory_[i].angle.j1_, buffer, LOG_TEXT_SIZE));
    //}

    uint32_t resume_at = 0;
    double alpha_ratio = 0.5;
    resume_trajectory_.clear();
    LogProducer::info("mc_base","Plan resume trajectory, input %d points", origin_trajectory_size);
    LogProducer::info("mc_base","Alpha ratio: %.2f", alpha_ratio);
    ErrorCode err = pause_resume_planner_.planResumeTrajectory(origin_trajectory_, resume_trajectory_, resume_at, alpha_ratio);

    while (err != SUCCESS && alpha_ratio < 3 + MINIMUM_E6)
    {
        alpha_ratio += 0.25;
        LogProducer::info("mc_base","Alpha ratio: %.2f", alpha_ratio);
        err = pause_resume_planner_.planResumeTrajectory(origin_trajectory_, resume_trajectory_, resume_at, alpha_ratio);
    }

    if (err != SUCCESS)
    {
        LogProducer::error("mc_base","Fail to plan resume trajectory, code: 0x%llx", err);
        return err;
    }

    LogProducer::info("mc_base","Resume trajectory ready with %d points, resume at %d", resume_trajectory_.size(), resume_at);
    /*
    ofstream out_before("/root/before_resume.csv");
    ofstream out_after("/root/after_resume.csv");

    for (auto it = origin_trajectory_.begin(); it != origin_trajectory_.end(); ++it)
    {
        out_before << it->angle[0] << "," << it->angle[1] << "," << it->angle[2] << "," << it->angle[3] << "," << it->angle[4] << "," << it->angle[5] << "," << it->omega[0] << "," << it->omega[1] << "," << it->omega[2] << "," << it->omega[3] << "," << it->omega[4] << "," << it->omega[5] << endl;
    }

    for (auto it = resume_trajectory_.begin(); it != resume_trajectory_.end(); ++it)
    {
        out_after << it->angle[0] << "," << it->angle[1] << "," << it->angle[2] << "," << it->angle[3] << "," << it->angle[4] << "," << it->angle[5] << "," << it->omega[0] << "," << it->omega[1] << "," << it->omega[2] << "," << it->omega[3] << "," << it->omega[4] << "," << it->omega[5] << endl;
    }

    out_before.close();
    out_after.close();
    */
    //for (uint32_t i = 0; i < resume_trajectory_.size(); ++i)
    //{
    //    LogProducer::info("mc_base","resume-%d: %s", i, printDBLine(&resume_trajectory_[i].angle.j1_, buffer, LOG_TEXT_SIZE));
    //}

    for (uint32_t i = resume_at + 1; i < origin_trajectory_size; ++i)
    {
        //LogProducer::info("mc_base","remain-%d: %s", i, printDBLine(&origin_trajectory_[i].angle.j1_, buffer, LOG_TEXT_SIZE));
        resume_trajectory_.push_back(origin_trajectory_[i]);
    }

    LogProducer::info("mc_base","Last %d points in origin trajectory push back to resume trajectory, resume trajectory size: %d", origin_trajectory_size - 1 - resume_at, resume_trajectory_.size());
    resume_trajectory_time_stamp_ = cycle_time_;
    point.level = POINT_START;
    uint32_t insert_index = 0;
    uint32_t trajectory_size = resume_trajectory_.size();

    while (!traj_fifo_.full() && insert_index < trajectory_size)
    {
        point.time_stamp = resume_trajectory_time_stamp_;
        resume_trajectory_time_stamp_ += cycle_time_;
        point.state = resume_trajectory_[insert_index];
        dynamics_ptr_->getTorqueInverseDynamics(point.state.angle, *(JointVelocity*)(&point.state.omega), *(JointAcceleration*)(&point.state.alpha), *(JointTorque*)(&point.state.torque));
        traj_fifo_.push(point);
        insert_index++;
        point.level = POINT_MIDDLE;
    }

    resume_trajectory_.erase(resume_trajectory_.begin(), resume_trajectory_.begin() + insert_index);
    LogProducer::info("mc_base","Total %d points, %d points filled into trajectory fifo, %d points left", trajectory_size, insert_index, resume_trajectory_.size());
    return SUCCESS;
}


ErrorCode BaseGroup::restartMove(void)
{
    LogProducer::info("mc_base","Restart move request received.");
    MotionControlState mc_state = mc_state_;
    ServoState servo_state = getServoState();
    ErrorCode err = SUCCESS;
    
    if (mc_state == PAUSE && servo_state == SERVO_IDLE)
    {
        pause_to_auto_request_ = true;
        return SUCCESS;
    }
    else if(mc_state == PAUSED_OFFLINE && servo_state == SERVO_IDLE)
    {
        err = planOfflineResume();
        if(err != SUCCESS) return err;
        usleep(10000);
        err = setOfflineTrajectory(offline_trajectory_file_name_);
        if(err != SUCCESS) return err;

        pause_to_offline_request_ = true;

        return SUCCESS;
    }
    else if (mc_state == STANDBY && servo_state == SERVO_IDLE)
    {
        return SUCCESS;
    }
    else
    {
        return INVALID_SEQUENCE;
    }
}


ErrorCode BaseGroup::isLinearPathReachable(const IntactPoint &start, const IntactPoint &target)
{
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_base","Check linear path reachable request received");
    LogProducer::info("mc_base","start joint: %s", printDBLine(&start.joint.j1_, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","target joint: %s", printDBLine(&target.joint.j1_, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","start pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", start.pose.pose.point_.x_, start.pose.pose.point_.y_, start.pose.pose.point_.z_, start.pose.pose.euler_.a_, start.pose.pose.euler_.b_, start.pose.pose.euler_.c_);
    LogProducer::info("mc_base","target pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", target.pose.pose.point_.x_, target.pose.pose.point_.y_, target.pose.pose.point_.z_, target.pose.pose.euler_.a_, target.pose.pose.euler_.b_, target.pose.pose.euler_.c_);
    MotionInfo info;
    info.type = MOTION_LINE;
    info.smooth_type = SMOOTH_NONE;
    info.is_swift = false;
    info.cnt = -1;
    info.vel = 2000;
    info.target = target;

    if (!soft_constraint_.isJointInConstraint(start.joint))
    {
        const Posture &posture = start.pose.posture;
        const PoseEuler &pose = start.pose.pose;
        const PoseEuler &uf = start.user_frame;
        const PoseEuler &tf = start.tool_frame;

        LogProducer::error("mc_base","Start joint out of soft constraint.");
        LogProducer::error("mc_base","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
        LogProducer::error("mc_base","Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
        LogProducer::error("mc_base","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
        LogProducer::error("mc_base","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
        LogProducer::error("mc_base","Joint = %s", printDBLine(&start.joint[0], buffer, LOG_TEXT_SIZE));
        LogProducer::error("mc_base","Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
        LogProducer::error("mc_base","Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (!soft_constraint_.isJointInConstraint(target.joint))
    {
        const Posture &posture = target.pose.posture;
        const PoseEuler &pose = target.pose.pose;
        const PoseEuler &uf = target.user_frame;
        const PoseEuler &tf = target.tool_frame;

        LogProducer::error("mc_base","Target joint out of soft constraint.");
        LogProducer::error("mc_base","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
        LogProducer::error("mc_base","Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
        LogProducer::error("mc_base","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
        LogProducer::error("mc_base","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
        LogProducer::error("mc_base","Joint = %s", printDBLine(&target.joint[0], buffer, LOG_TEXT_SIZE));
        LogProducer::error("mc_base","Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
        LogProducer::error("mc_base","Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (!isPostureMatch(start.pose.posture, target.pose.posture))
    {
        LogProducer::error("mc_base","Posture of target mismatch with start.");
        LogProducer::error("mc_base","Posture of start: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", start.pose.posture.arm, start.pose.posture.elbow, start.pose.posture.wrist, start.pose.posture.flip);
        LogProducer::error("mc_base","Posture of target: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", target.pose.posture.arm, target.pose.posture.elbow, target.pose.posture.wrist, target.pose.posture.flip);
        return MC_POSTURE_MISMATCH;
    }

    if (getDistance(start.pose.pose.point_, target.pose.pose.point_) < MINIMUM_E3 && getOrientationAngle(start.pose.pose.euler_, target.pose.pose.euler_) < MINIMUM_E3)
    {
        // Target pose coincidence with start
        return SUCCESS;
    }

    LogProducer::info("mc_base","Parameter check passed, planning ...");
    ErrorCode err = planner_for_check_.planTrajectory(start.joint, info, 1, 1);

    if (err != SUCCESS)
    {
        LogProducer::error("mc_base","Planning failed with code = 0x%llx, linear path unreachable.", err);
        return err;
    }

    LogProducer::info("mc_base","Planning success, linear path reachable");
    return SUCCESS;
}


ErrorCode BaseGroup::autoMove(const MotionInfo &info)
{
    char buffer[LOG_TEXT_SIZE];
    const PoseEuler &pose = info.target.pose.pose;
    const Posture &posture = info.target.pose.posture;

    // update start_joint_ before moving
    start_joint_ = getLatestJoint();

    LogProducer::info("mc_base","Auto move request received, type = %d", info.type);
    LogProducer::info("mc_base","vel = %.6f, acc = %.6f, cnt = %.6f, swift = %d", info.vel, info.acc, info.cnt, info.is_swift);
    LogProducer::info("mc_base","start-joint: %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","target-joint: %s", printDBLine(&info.target.joint.j1_, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","target-pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
    LogProducer::info("mc_base","target-posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);

    if (info.type == MOTION_CIRCLE)
    {
        const PoseEuler &via_pose = info.via.pose.pose;
        const Posture &via_posture = info.via.pose.posture;
        LogProducer::info("mc_base","via-pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", via_pose.point_.x_, via_pose.point_.y_, via_pose.point_.z_, via_pose.euler_.a_, via_pose.euler_.b_, via_pose.euler_.c_);
        LogProducer::info("mc_base","via-posture: %d, %d, %d, %d", via_posture.arm, via_posture.elbow, via_posture.wrist, via_posture.flip);
    }
    
    ErrorCode err = checkMotionTarget(info);

    if (err != SUCCESS)
    {
        // moveJ或者movel时如果目标点和起始点重合，直接跳过这条指令的规划执行，什么也不做
        // 如果是其他错误则报错，提前结束运动规划
        if (err == TARGET_COINCIDENCE)
        {
            LogProducer::warn("mc_base","Target coincidence with start, nothing to do.");
            return SUCCESS;
        }
        else
        {
            LogProducer::error("mc_base","Parameter check failed, code = 0x%llx", err);
            return err;
        }
    }

    if (plan_traj_ptr_->valid == true)
    {
        LogProducer::error("mc_base","No trajectory cache available, cannot plan new trajectory before old trajectory finish.");
        return MC_INTERNAL_FAULT;
    }

    if (info.is_swift)
    {
        plan_traj_ptr_->trajectory.useSwiftParam();
    }
    else
    {
        plan_traj_ptr_->trajectory.useStableParam();
    }

    LogProducer::info("mc_base","Parameter check passed, planning trajectory ...");
    err = plan_traj_ptr_->trajectory.planTrajectory(start_joint_, info, vel_ratio_, acc_ratio_);
    MotionInfo motion_info_this = plan_traj_ptr_->trajectory.getMotionInfo();
    MotionInfo motion_info_pre = pick_traj_ptr_->trajectory.getMotionInfo();

    if (err != SUCCESS)
    {
        LogProducer::error("mc_base","Planning failed with code = 0x%llx, autoMove aborted.", err);
        return err;
    }

    LogProducer::info("mc_base","Planning trajectory success, duration of trajectory: %.6f", plan_traj_ptr_->trajectory.getDuration());
    start_joint_ = info.target.joint;

    if (pick_traj_ptr_->valid && pick_traj_ptr_->end_with_smooth)
    {
        // 上一条指令带平滑: 计算平滑切入点,规划平滑段轨迹
        LogProducer::info("mc_base","Planning smooth ...");
        plan_traj_ptr_->smooth.setCoord(info.target.user_frame);
        plan_traj_ptr_->smooth.setTool(info.target.tool_frame);
        plan_traj_ptr_->smooth.planTrajectory(pick_traj_ptr_->trajectory, plan_traj_ptr_->trajectory, pick_traj_ptr_->trajectory.getMotionInfo().cnt);
        double smooth_in_time = plan_traj_ptr_->smooth.getDuration();
        double smooth_out_time = pick_traj_ptr_->trajectory.getDuration() - plan_traj_ptr_->smooth.getDuration();

        if (smooth_in_time > MINIMUM_E3)
        {
            pthread_mutex_lock(&planner_list_mutex_);
            pick_traj_ptr_->smooth_time = smooth_out_time;
            plan_traj_ptr_->smooth_in_time = smooth_in_time;
            plan_traj_ptr_->start_from_smooth = true;
            pthread_mutex_unlock(&planner_list_mutex_);
            LogProducer::info("mc_base","Start from smooth, smooth-duration: %.6f", plan_traj_ptr_->smooth.getDuration());
        }
        else
        {
            pthread_mutex_lock(&planner_list_mutex_);
            pick_traj_ptr_->smooth_time = pick_traj_ptr_->trajectory.getDuration();
            plan_traj_ptr_->start_from_smooth = false;
            pthread_mutex_unlock(&planner_list_mutex_);
            LogProducer::info("mc_base","Start from stable.", err);
        }
    }
    else 
    {
        plan_traj_ptr_->start_from_smooth = false;
        LogProducer::info("mc_base","Start from stable.");
    }

    if (info.cnt > MINIMUM_E3)
    {
        // 本条指令带平滑: 计算切出时间和切出点
        plan_traj_ptr_->smooth_time = plan_traj_ptr_->trajectory.getSmoothOutTime(info.cnt);

        if (plan_traj_ptr_->smooth_time > MINIMUM_E3)
        {
            plan_traj_ptr_->end_with_smooth = true;
            fine_enable_ = false;
            LogProducer::info("mc_base","End with smooth, smooth-time: %.6f, smooth-distance: %.6f", plan_traj_ptr_->smooth_time, info.cnt);
        }
        else
        {
            plan_traj_ptr_->smooth_time = -1;
            plan_traj_ptr_->end_with_smooth = false;
            fine_enable_ = false;
            LogProducer::info("mc_base","End with pre-fetch.");
        }
    }
    else if (fabs(info.cnt) < MINIMUM_E3)
    {
        // CNT = 0
        plan_traj_ptr_->smooth_time = -1;
        plan_traj_ptr_->end_with_smooth = false;
        fine_enable_ = false;
        LogProducer::info("mc_base","End with pre-fetch.");
    }
    else
    {
        plan_traj_ptr_->smooth_time = -1;
        plan_traj_ptr_->end_with_smooth = false;
        fine_enable_ = true;
        LogProducer::info("mc_base","End with fine.");
    }
    
    pthread_mutex_lock(&planner_list_mutex_);

    if (plan_traj_ptr_->start_from_smooth)
    {
        if (!pick_traj_ptr_->valid || !pick_traj_ptr_->end_with_smooth)
        {
            // 如果下发线程已经被迫放弃圆滑
            LogProducer::warn("mc_base","Smooth fail, start from stable, pick.valid: %d, pick.smooth: %d", pick_traj_ptr_->valid, pick_traj_ptr_->end_with_smooth);
            plan_traj_ptr_->start_from_smooth = false;
        }
    }
    
    plan_traj_ptr_->valid = true;
    plan_traj_ptr_ = plan_traj_ptr_->next;
    pthread_mutex_unlock(&planner_list_mutex_);

    if (mc_state_ == STANDBY)
    {
        standby_to_auto_request_ = true;
    }

    LogProducer::info("mc_base","Trajectory plan finished.");
    return err;
}

/*
ErrorCode BaseGroup::checkStartState(const Joint &start_joint)
{
    //if (mc_state_ == STANDBY && servo_state_ == SERVO_IDLE && traj_list_ptr_ == NULL && path_list_ptr_ == NULL)
    // FIXME
    if (mc_state_ == STANDBY && servo_state_ == SERVO_IDLE)
    {
        Joint control_joint;
        Joint current_joint = getLatestJoint();

        if (bare_core_.getControlPosition(&control_joint[0], getNumberOfJoint()))
        {
            if (!isSameJoint(current_joint, control_joint, joint_tracking_accuracy_))
            {
                char buffer[LOG_TEXT_SIZE];
                LogProducer::error("mc_base","Control-position different with current-position, it might be a trouble.");
                LogProducer::error("mc_base","control-position: %s", printDBLine(&control_joint[0], buffer, LOG_TEXT_SIZE));
                LogProducer::error("mc_base","current-position: %s", printDBLine(&current_joint[0], buffer, LOG_TEXT_SIZE));
                return MC_JOINT_TRACKING_ERROR;
            }

            if (!isSameJoint(start_joint, control_joint, joint_tracking_accuracy_))
            {
                char buffer[LOG_TEXT_SIZE];
                LogProducer::error("mc_base","Control-position different with start-position, it might be a trouble.");
                LogProducer::error("mc_base","control-position: %s", printDBLine(&control_joint[0], buffer, LOG_TEXT_SIZE));
                LogProducer::error("mc_base","start-position:   %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));
                return MC_JOINT_TRACKING_ERROR;
            }
        }
        else
        {
            LogProducer::error("mc_base","Cannot get control position from bare core.");
            return MC_COMMUNICATION_WITH_BARECORE_FAIL;
        }
    }

    if (!soft_constraint_.isJointInConstraint(start_joint))
    {
        char buffer[LOG_TEXT_SIZE];
        LogProducer::error("mc_base","Start joint out of soft constraint.");
        LogProducer::error("mc_base","  joint: %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));
        LogProducer::error("mc_base","  upper: %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
        LogProducer::error("mc_base","  lower: %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    return SUCCESS;
}
*/

ErrorCode BaseGroup::checkMotionTarget(const MotionInfo &info)
{
    if (info.type != MOTION_JOINT && info.type != MOTION_LINE && info.type != MOTION_CIRCLE)
    {
        LogProducer::error("mc_base","Invalid motion type: %d", info.type);
        return INVALID_PARAMETER;
    }
    
    if (info.smooth_type == SMOOTH_DISTANCE)
    {
        if (info.cnt < -MINIMUM_E9)
        {
            LogProducer::error("mc_base","Invalid CNT by smooth-distance: %.12f", info.cnt);
            return INVALID_PARAMETER;
        }
    }
    else if (info.smooth_type == SMOOTH_VELOCITY)
    {
        if (info.cnt < -MINIMUM_E9 || info.cnt > 1 + MINIMUM_E9)
        {
            LogProducer::error("mc_base","Invalid CNT by smooth-velocity: %.12f", info.cnt);
            return INVALID_PARAMETER;
        }
    }
    else
    {
        if (fabs(info.cnt + 1) > MINIMUM_E9)
        {
            LogProducer::error("mc_base","Invalid CNT by smooth-none: %.12f", info.cnt);
            return INVALID_PARAMETER;
        }
    }

    // CNT ∈ [0, 1] U CNT = -1
    //if (fabs(info.cnt + 1) > MINIMUM_E9 && (info.cnt < -MINIMUM_E9 || info.cnt > 1 + MINIMUM_E9))
    //{
    //    LogProducer::error("mc_base","Invalid CNT: %.12f", info.cnt);
    //    return INVALID_PARAMETER;
    //}

    if (  ((info.type == MOTION_JOINT) && (info.vel < MINIMUM_E6 || info.vel > 1 + MINIMUM_E6)) ||
          ((info.type == MOTION_LINE || info.type == MOTION_CIRCLE) && (info.vel < cartesian_vel_min_ || info.vel > cartesian_vel_max_))  )
    {
        LogProducer::error("mc_base","Invalid vel: %.6f", info.vel);
        return INVALID_PARAMETER;
    }

    // acc should be 0 ~ 1, cannot be 0 but can be 1
    if (info.acc < MINIMUM_E6 || info.acc > 1 + MINIMUM_E6)
    {
        LogProducer::error("mc_base","Invalid acc: %.6f", info.acc);
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

            LogProducer::error("mc_base","Target joint out of soft constraint.");
            LogProducer::error("mc_base","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            LogProducer::error("mc_base","Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            LogProducer::error("mc_base","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            LogProducer::error("mc_base","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            LogProducer::error("mc_base","Joint = %s", printDBLine(&info.target.joint[0], buffer, LOG_TEXT_SIZE));
            LogProducer::error("mc_base","Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            LogProducer::error("mc_base","Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
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
            LogProducer::warn("mc_base","Target joint coincidence with start joint.");
            LogProducer::warn("mc_base","Start-joint = %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
            LogProducer::warn("mc_base","Target-joint = %s", printDBLine(&info.target.joint.j1_, buffer, LOG_TEXT_SIZE));
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
        LogProducer::info("mc_base","Start-pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
        LogProducer::info("mc_base","Start-posture: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", start_posture.arm, start_posture.elbow, start_posture.wrist, start_posture.flip);

        if (!soft_constraint_.isJointInConstraint(info.target.joint))
        {
            char buffer[LOG_TEXT_SIZE];
            const Posture &posture = info.target.pose.posture;
            const PoseEuler &pose = info.target.pose.pose;
            const PoseEuler &uf = info.target.user_frame;
            const PoseEuler &tf = info.target.tool_frame;

            LogProducer::error("mc_base","Target joint out of soft constraint.");
            LogProducer::error("mc_base","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            LogProducer::error("mc_base","Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            LogProducer::error("mc_base","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            LogProducer::error("mc_base","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            LogProducer::error("mc_base","Joint = %s", printDBLine(&info.target.joint[0], buffer, LOG_TEXT_SIZE));
            LogProducer::error("mc_base","Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            LogProducer::error("mc_base","Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
            return JOINT_OUT_OF_CONSTRAINT;
        }

        if (!isPostureMatch(start_posture, target_posture))
        {
            LogProducer::error("mc_base","Posture of target mismatch with start.");
            LogProducer::error("mc_base","Posture of start: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", start_posture.arm, start_posture.elbow, start_posture.wrist, start_posture.flip);
            LogProducer::error("mc_base","Posture of target: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", target_posture.arm, target_posture.elbow, target_posture.wrist, target_posture.flip);
            return MC_POSTURE_MISMATCH;
        }

        if (getDistance(start_pose.point_, info.target.pose.pose.point_) < MINIMUM_E3 && getOrientationAngle(start_pose.euler_, info.target.pose.pose.euler_) < MINIMUM_E3)
        {
            char buffer[LOG_TEXT_SIZE];
            const Joint &target_joint = info.target.joint;
            const PoseEuler &target_pose = info.target.pose.pose;
            LogProducer::warn("mc_base","Target pose coincidence with start.");
            LogProducer::warn("mc_base","Start-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
            LogProducer::warn("mc_base","Target-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
            LogProducer::warn("mc_base","Start-joint = %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
            LogProducer::warn("mc_base","Target-joint = %s", printDBLine(&target_joint.j1_, buffer, LOG_TEXT_SIZE));
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
        LogProducer::info("mc_base","Start-pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
        LogProducer::info("mc_base","Start-posture: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", start_posture.arm, start_posture.elbow, start_posture.wrist, start_posture.flip);

        if (!soft_constraint_.isJointInConstraint(info.target.joint))
        {
            char buffer[LOG_TEXT_SIZE];
            const Posture &posture = info.target.pose.posture;
            const PoseEuler &pose = info.target.pose.pose;
            const PoseEuler &uf = info.target.user_frame;
            const PoseEuler &tf = info.target.tool_frame;

            LogProducer::error("mc_base","Target joint out of soft constraint.");
            LogProducer::error("mc_base","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            LogProducer::error("mc_base","Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            LogProducer::error("mc_base","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            LogProducer::error("mc_base","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            LogProducer::error("mc_base","Joint = %s", printDBLine(&info.target.joint[0], buffer, LOG_TEXT_SIZE));
            LogProducer::error("mc_base","Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            LogProducer::error("mc_base","Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
            return JOINT_OUT_OF_CONSTRAINT;
        }

        if (!soft_constraint_.isJointInConstraint(info.via.joint))
        {
            char buffer[LOG_TEXT_SIZE];
            const Posture &posture = info.via.pose.posture;
            const PoseEuler &pose = info.via.pose.pose;
            const PoseEuler &uf = info.via.user_frame;
            const PoseEuler &tf = info.via.tool_frame;
            LogProducer::error("mc_base","Via joint out of soft constraint.");
            LogProducer::error("mc_base","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            LogProducer::error("mc_base","Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            LogProducer::error("mc_base","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            LogProducer::error("mc_base","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            LogProducer::error("mc_base","Joint = %s", printDBLine(&info.via.joint[0], buffer, LOG_TEXT_SIZE));
            LogProducer::error("mc_base","Upper = %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
            LogProducer::error("mc_base","Lower = %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
            return JOINT_OUT_OF_CONSTRAINT;
        }

        if (!isPostureMatch(start_posture, target_posture))
        {
            LogProducer::error("mc_base","Posture of target mismatch with start.");
            LogProducer::error("mc_base","Posture of start: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", start_posture.arm, start_posture.elbow, start_posture.wrist, start_posture.flip);
            LogProducer::error("mc_base","Posture of target: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", target_posture.arm, target_posture.elbow, target_posture.wrist, target_posture.flip);
            return MC_POSTURE_MISMATCH;
        }

        if (!isPostureMatch(start_posture, via_posture))
        {
            LogProducer::error("mc_base","Posture of via mismatch with start.");
            LogProducer::error("mc_base","Posture of start: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", start_posture.arm, start_posture.elbow, start_posture.wrist, start_posture.flip);
            LogProducer::error("mc_base","Posture of via: ARM = %d, ELBOW = %d, WRIST = %d, FLIP = %d", via_posture.arm, via_posture.elbow, via_posture.wrist, via_posture.flip);
            return MC_POSTURE_MISMATCH;
        }

        if (getDistance(start_pose.point_, info.target.pose.pose.point_) < MINIMUM_E0)
        {
            char buffer[LOG_TEXT_SIZE];
            const Joint &target_joint = info.target.joint;
            const PoseEuler &target_pose = info.target.pose.pose;
            LogProducer::warn("mc_base","Target pose coincidence with start.");
            LogProducer::warn("mc_base","Start-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
            LogProducer::warn("mc_base","Target-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
            LogProducer::warn("mc_base","Start-joint = %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
            LogProducer::warn("mc_base","Target-joint = %s", printDBLine(&target_joint.j1_, buffer, LOG_TEXT_SIZE));
            return MC_ARC_PLANNING_FAIL;
        }

        if (getDistance(start_pose.point_, info.via.pose.pose.point_) < MINIMUM_E0)
        {
            char buffer[LOG_TEXT_SIZE];
            const Joint &via_joint = info.via.joint;
            const PoseEuler &via_pose = info.via.pose.pose;
            LogProducer::warn("mc_base","Via pose coincidence with start.");
            LogProducer::warn("mc_base","  start-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
            LogProducer::warn("mc_base","  via-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", via_pose.point_.x_, via_pose.point_.y_, via_pose.point_.z_, via_pose.euler_.a_, via_pose.euler_.b_, via_pose.euler_.c_);
            LogProducer::warn("mc_base","  start-joint = %s", printDBLine(&start_joint_.j1_, buffer, LOG_TEXT_SIZE));
            LogProducer::warn("mc_base","  via-joint = %s", printDBLine(&via_joint.j1_, buffer, LOG_TEXT_SIZE));
            return MC_ARC_PLANNING_FAIL;
        }

        if (getDistance(info.target.pose.pose.point_, info.via.pose.pose.point_) < MINIMUM_E0)
        {
            char buffer[LOG_TEXT_SIZE];
            const Joint &via_joint = info.via.joint;
            const Joint &target_joint = info.target.joint;
            const PoseEuler &via_pose = info.via.pose.pose;
            const PoseEuler &target_pose = info.target.pose.pose;
            LogProducer::warn("mc_base","Via pose coincidence with target.");
            LogProducer::warn("mc_base","  via-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", via_pose.point_.x_, via_pose.point_.y_, via_pose.point_.z_, via_pose.euler_.a_, via_pose.euler_.b_, via_pose.euler_.c_);
            LogProducer::warn("mc_base","  target-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
            LogProducer::warn("mc_base","  via-joint = %s", printDBLine(&via_joint.j1_, buffer, LOG_TEXT_SIZE));
            LogProducer::warn("mc_base","  target-joint = %s", printDBLine(&target_joint.j1_, buffer, LOG_TEXT_SIZE));
            return MC_ARC_PLANNING_FAIL;
        }

        Point point21 = info.via.pose.pose.point_ - start_pose.point_;
        Point point31 = info.target.pose.pose.point_ - start_pose.point_;

        if (point31.isParallel(point21, MINIMUM_E0))
        {
            const PoseEuler &via_pose = info.via.pose.pose;
            const PoseEuler &target_pose = info.target.pose.pose;
            LogProducer::warn("mc_base","Three points are collinear, arc planning fail.");
            LogProducer::warn("mc_base","  start-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", start_pose.point_.x_, start_pose.point_.y_, start_pose.point_.z_, start_pose.euler_.a_, start_pose.euler_.b_, start_pose.euler_.c_);
            LogProducer::warn("mc_base","  via-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", via_pose.point_.x_, via_pose.point_.y_, via_pose.point_.z_, via_pose.euler_.a_, via_pose.euler_.b_, via_pose.euler_.c_);
            LogProducer::warn("mc_base","  target-pose = %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
            return MC_ARC_PLANNING_FAIL;
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
    // LogProducer::warn("mc_base","is-next-Move-Permitted ?");
    uint32_t branch = 0;
    MotionControlState state = mc_state_;
    ServoState servo_state = getServoState();
    pthread_mutex_lock(&planner_list_mutex_);
    
    if (state == STANDBY && (standby_to_auto_request_ || pick_traj_ptr_->valid))
    {
        // 第一条运动指令规划成功,还未开始取点
        branch = 1;
    }

    else if ((state == STANDBY_TO_AUTO || state == AUTO) && fine_enable_)
    {
        // fine语句需等待state切回STANDBY后才允许执行下一条
        branch = 2;
    }

    else if ((state == STANDBY_TO_AUTO || state == AUTO || state == PREPARE_RESUME) && plan_traj_ptr_->valid)
    {
        branch = 3;
    }

    else if (
        (state == STANDBY_TO_AUTO || state == AUTO || state == PREPARE_RESUME) 
        && pick_traj_ptr_->valid
        && (pick_traj_ptr_->start_from_smooth || (pick_traj_ptr_->end_with_smooth && (auto_time_ < pick_traj_ptr_->smooth_time + MINIMUM_E6)) || !pick_traj_ptr_->end_with_smooth))
    {
        branch = 4;
    }
    
    else if (state == AUTO_TO_STANDBY)
    {
        // 正在等待Fine到位
        branch = 5;
    }

    else if (state == PAUSE || state == PAUSING || state == AUTO_TO_PAUSING || state == PAUSING_TO_PAUSE)
    {
        branch = 6;
    }

    else if (state == RESUME || state == PAUSE_TO_RESUME)
    {
        branch = 7;
    }

    else if (state == PAUSE_TO_PAUSE_RETURN || state == PAUSE_RETURN || state == PAUSE_RETURN_TO_PAUSE)
    {
        branch = 8;
    }

    else if (state == PAUSE_TO_PAUSE_MANUAL || state == PAUSE_MANUAL || state == PAUSE_MANUAL_TO_PAUSE)
    {
        branch = 9;
    }
    else if (state == PREPARE_RESUME && fine_enable_)
    {
        // 暂停恢复fine语句时不允许向下预读下一条，使用当前语句剩余点规划resume轨迹
        branch = 10;
    }

    if (servo_state != SERVO_IDLE && servo_state != SERVO_RUNNING)
    {
        branch = 100;
    }

    /*
    if (state == PAUSE || state == PAUSING || state == PAUSE_RETURN || state == AUTO_TO_PAUSE || state == PAUSE_TO_PAUSE_RETURN || state == PAUSE_RETURN_TO_STANDBY)
    {
        pthread_mutex_unlock(&planner_list_mutex_);
        return false;
    }
    */

    
    //LogProducer::warn("mc_base","Next motion permitted: grp-state = %d, branch = %u", state, branch);

    if (branch == 0)
    {
        LogProducer::warn("mc_base","Next motion permitted: state=0x%x, servo-state=%s, pick->valid=%d, plan->valid=%d, pick->start_from_smooth=%d, pick->end_with_smooth=%d, auto_time=%.6f, pick->smooth_time=%.6f", 
            state, getMCServoStatusString(servo_state).c_str(), pick_traj_ptr_->valid, plan_traj_ptr_->valid, pick_traj_ptr_->start_from_smooth, pick_traj_ptr_->end_with_smooth, auto_time_, pick_traj_ptr_->smooth_time);
    }
    else
    {
        //LogProducer::info("mc_base","Next motion not permitted, branch: %d", branch);
    }

    pthread_mutex_unlock(&planner_list_mutex_);
    return branch == 0;
}


ErrorCode BaseGroup::checkManualTrajectory(double start_time, double end_time, double step_time, Joint reference)
{
    LogProducer::info("mc_base","Check manual trajectory, start at: %.6f, end at: %.6f, step: %.6f", start_time, end_time, step_time);
    ErrorCode err = SUCCESS;
    JointState state;

    for (double t = start_time; t < end_time; t += step_time)
    {
        err = manual_teach_.sampleTrajectory(t, reference, state);

        if (err != SUCCESS)
        {
            LogProducer::error("mc_base","Trajectory check failed at t: %.6f, code: 0x%llx", t, err);
            return err;
        }
        else if (!soft_constraint_.isJointInConstraint(state.angle, MINIMUM_E3))
        {
            char buffer[LOG_TEXT_SIZE];
            LogProducer::error("mc_base","Trajectory near soft constraint at t: %.6f, joint: %s", t, printDBLine(&state.angle.j1_, buffer, LOG_TEXT_SIZE));
            return JOINT_OUT_OF_CONSTRAINT;
        }
        else if (kinematics_ptr_->nearSingularPosition(state.angle))
        {
            char buffer[LOG_TEXT_SIZE];
            LogProducer::error("mc_base","Trajectory near singular position at t: %.6f, joint: %s", t, printDBLine(&state.angle.j1_, buffer, LOG_TEXT_SIZE));
            return MC_MANUAL_TO_SINGULAR_POSITION;
        }
        else
        {}

        reference = state.angle;
    }

    LogProducer::info("mc_base","Check manual trajectory passed");
    return SUCCESS;
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


} // namespace fst_mc
