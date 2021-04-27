/*************************************************************************
	> File Name: motion_control_state_machine.cpp
	> Author: 
	> Mail: 
	> Created Time: 2019年11月29日 星期五 15时51分28秒
 ************************************************************************/
#include <basic_alg.h>
#include <motion_control_base_group.h>
#include "log_manager_producer.h"

using namespace std;
using namespace basic_alg;
using namespace log_space;

namespace group_space
{

void BaseGroup::doStateMachine(void)
{
    static uint32_t standby_to_auto_cnt = 0;
    static uint32_t auto_to_standby_cnt = 0;
    static uint32_t manual_to_standby_cnt = 0;
    static uint32_t pausing_to_pause_cnt = 0;
    static uint32_t pause_return_to_pause_cnt = 0;
    static uint32_t offline_to_standby_cnt = 0;
    static uint32_t prepare_resume_cnt = 0;
    static uint32_t pause_manual_to_pause_cnt = 0;
    static uint32_t fine_counter = 0;

    GroupState group_state = group_state_;
    ServoState servo_state = getServoState();

    if (clear_request_ && (group_state == STANDBY || group_state == PAUSE))
    {
        LogProducer::info("mc_sm","Clear group, group-state = %d", group_state);
        group_state_ = STANDBY;
        
        pthread_mutex_lock(&planner_list_mutex_);
        auto_time_ = 0;
        trajectory_a_.valid = false;
        trajectory_b_.valid = false;
        plan_traj_ptr_ = &trajectory_a_;
        pick_traj_ptr_ = &trajectory_a_;
        traj_fifo_.clear();
        bare_core_.clearPointCache();
        fine_enable_ = false;
        standby_to_auto_request_ = false;
        auto_to_standby_request_ = false;
        auto_to_pause_request_ = false;
        pause_to_auto_request_ = false;
        pthread_mutex_unlock(&planner_list_mutex_);

        pthread_mutex_lock(&offline_mutex_);
        standby_to_offline_request_ = false;
        offline_to_standby_request_ = false;
        offline_trajectory_cache_head_ = 0;
        offline_trajectory_cache_tail_ = 0;
        offline_trajectory_first_point_ = false;
        offline_trajectory_last_point_ = false;
        pthread_mutex_unlock(&offline_mutex_);

        manual_trajectory_check_fail_ = false;
        standby_to_manual_request_ = false;
        manual_to_standby_request_ = false;
        pause_to_manual_request_ = false;
        manual_to_pause_request_ = false;

        clear_request_ = false;
        stop_barecore_ = false;
        LogProducer::info("mc_sm","Group cleared.");
    }

    if (clear_teach_request_)
    {
        LogProducer::info("mc_sm","Clear teach group, group-state = 0x%x", group_state);
        pthread_mutex_lock(&manual_traj_mutex_);
        manual_time_ = 0;
        clear_teach_request_ = false;
        manual_trajectory_check_fail_ = false;

        if (group_state == STANDBY || group_state == MANUAL || group_state == MANUAL_TO_STANDBY || group_state == STANDBY_TO_MANUAL)
        {
            group_state_ = STANDBY;
            standby_to_manual_request_ = false;
            manual_to_standby_request_ = false;
            bare_core_.clearPointCache();
        }
        else if (group_state == PAUSE || group_state == PAUSE_MANUAL || group_state == PAUSE_TO_PAUSE_MANUAL || group_state == PAUSE_MANUAL_TO_PAUSE)
        {
            group_state_ = PAUSE;
            pause_to_manual_request_ = false;
            manual_to_pause_request_ = false;
            bare_core_.clearPointCache();
        }
        pthread_mutex_unlock(&manual_traj_mutex_);
        LogProducer::info("mc_sm","Teach group cleared, group-state = 0x%x", group_state);
    }

    if (standby_to_offline_request_ && group_state != STANDBY)
    {
        standby_to_offline_request_ = false;
    }

    if (offline_to_standby_request_ && group_state != OFFLINE)
    {
        offline_to_standby_request_ = false;
    }

    if (standby_to_auto_request_ && group_state != STANDBY)
    {
        standby_to_auto_request_ = false;
    }

    if (auto_to_standby_request_ && group_state != AUTO)
    {
        auto_to_standby_request_ = false;
    }

    if (standby_to_manual_request_ && group_state != STANDBY)
    {
        standby_to_manual_request_ = false;
    }

    if (manual_to_standby_request_ && group_state != MANUAL)
    {
        manual_to_standby_request_ = false;
    }

    if (pause_to_manual_request_ && group_state != PAUSE)
    {
        pause_to_manual_request_ = false;
    }

    if (manual_to_pause_request_ && group_state != PAUSE_MANUAL)
    {
        manual_to_pause_request_ = false;
    }

    if (auto_to_pause_request_ && group_state != AUTO)
    {
        auto_to_pause_request_ = false;
    }

    if (pause_to_auto_request_ && group_state != PAUSE && group_state != PAUSE_TO_PAUSE_RETURN && group_state != PAUSE_RETURN && group_state != PAUSE_RETURN_TO_PAUSE)
    {
        pause_to_auto_request_ = false;
    }


    if (stop_barecore_ && (servo_state == SERVO_DISABLE))
    {
        LogProducer::info("mc_sm","Barecore stop, group-state = 0x%x, servo-state = 0x%x", group_state, servo_state);
        stop_barecore_ = false;

        if (pause_to_auto_request_) pause_to_auto_request_ = false;

        if (group_state == MANUAL || group_state == MANUAL_TO_STANDBY || group_state == STANDBY_TO_MANUAL ||
            group_state == OFFLINE || group_state == OFFLINE_TO_STANDBY || group_state == STANDBY_TO_OFFLINE)
        {
            group_state_ = STANDBY;
            clear_request_ = true;
            LogProducer::warn("mc_sm","Group state switch to STANDBY.");
        }
        else if (group_state == PAUSE_MANUAL || group_state == PAUSE_TO_PAUSE_MANUAL || group_state == PAUSE_MANUAL_TO_PAUSE ||
                 group_state == PAUSING_TO_PAUSE || group_state == PAUSE_RETURN_TO_PAUSE || group_state == PREPARE_RESUME)
        {
            group_state_ = PAUSE;
            LogProducer::warn("mc_sm","Group state switch to PAUSE.");
        }
        else if (group_state == PAUSING)
        {
            pause_trajectory_.clear();
            traj_fifo_.clear();
            bare_core_.clearPointCache();
            group_state_ = PAUSE;
            LogProducer::warn("mc_sm","Group state switch to PAUSE.");
        }
        else if (group_state == PAUSE_RETURN)
        {
            resume_trajectory_.clear();
            traj_fifo_.clear();
            bare_core_.clearPointCache();
            group_state_ = PAUSE;
            LogProducer::warn("mc_sm","Group state switch to PAUSE.");
        }
        else if (group_state == RESUME)
        {
            group_state_ = PAUSE;
            LogProducer::warn("mc_sm","Group state switch to PAUSE.");
            remain_trajectory_.assign(origin_trajectory_.begin(), origin_trajectory_.end());
            start_joint_before_pause_ = start_joint_;
            pause_joint_ = remain_trajectory_.front().angle;
            resume_trajectory_.clear();
            traj_fifo_.clear();
            bare_core_.clearPointCache();
        }
        else if (group_state == AUTO_TO_PAUSING || group_state == PAUSE_TO_RESUME || group_state == PAUSE_TO_PAUSE_RETURN)
        {}
        else if (group_state == AUTO_TO_STANDBY)
        {
            group_state_ = STANDBY;
            LogProducer::warn("mc_sm","Group state switch to STANDBY.");
        }
        else if (group_state == AUTO && auto_to_standby_request_ == true)
        {
            group_state_ = STANDBY;
            bare_core_.clearPointCache();
            LogProducer::warn("mc_sm","Group state switch to STANDBY.");
        }
        else if (group_state == AUTO || group_state == STANDBY_TO_AUTO)
        {
            group_state_ = PAUSE;
            LogProducer::warn("mc_sm","Group state switch to PAUSE.");
            start_joint_before_pause_ = start_joint_;
            remain_trajectory_.clear();
            TrajectoryPoint point;

            // 保存FIFO中的点
            while (!traj_fifo_.empty())
            {
                traj_fifo_.fetch(point);
                remain_trajectory_.push_back(point.state);
                //char buffer[LOG_TEXT_SIZE];
                //LogProducer::info("mc_sm","joint in FIFO: %s", printDBLine(&point.state.angle.j1_, buffer, LOG_TEXT_SIZE));
            }

            LogProducer::info("mc_sm","Remain traj size: %d", remain_trajectory_.size());
            bare_core_.clearPointCache();

            if (!remain_trajectory_.empty())
            {
                pause_joint_ = remain_trajectory_.front().angle;
            }
            else
            {
                LogProducer::error("mc_sm","No trajectory point in FIFO.");
                reportError(MC_INTERNAL_FAULT);
            }
        }
    }

    switch (group_state)
    {
        case STANDBY:
        {
            pthread_mutex_lock(&planner_list_mutex_);

            if (pick_traj_ptr_->valid && standby_to_auto_request_ == false)
            {
                LogProducer::info("mc_sm","Group state: standby, pick_traj_ptr_->valid is true but standby_to_auto_request_ is false, set request");
                standby_to_auto_request_ = true;
            }
    
            pthread_mutex_unlock(&planner_list_mutex_);

            if (standby_to_auto_request_)
            {
                standby_to_auto_cnt = 0;
                auto_time_ = cycle_time_;
                start_of_motion_ = true;
                group_state_ = STANDBY_TO_AUTO;
                standby_to_auto_request_ = false;
                LogProducer::warn("mc_sm","Group state switch to STANDBY_TO_AUTO");
            }
            else if (standby_to_manual_request_)
            {
                group_state_ = STANDBY_TO_MANUAL;
                standby_to_manual_request_ = false;
                LogProducer::warn("mc_sm","Group state switch to STANDBY_TO_MANUAL");
            }
            else if (standby_to_offline_request_)
            {
                group_state_ = STANDBY_TO_OFFLINE;
                LogProducer::warn("mc_sm","Group state switch to STANDBY_TO_OFFLINE");
            }

            break;
        }

        case AUTO:
        {
            if (auto_to_standby_request_)
            {
                fine_counter = 0;
                auto_to_standby_cnt = 0;
                group_state_ = AUTO_TO_STANDBY;
                auto_to_standby_request_ = false;
                LogProducer::warn("mc_sm","Group state switch to AUTO_TO_STANDBY");
            }

            if (auto_to_pause_request_ && traj_fifo_.size() > 250)
            {
                group_state_ = AUTO_TO_PAUSING;
                auto_to_pause_request_ = false;
                LogProducer::warn("mc_sm","Group state switch to AUTO_TO_PAUSING");
            }

            break;
        }

        case PAUSING:
        {
            if (pausing_to_pause_request_)
            {
                pausing_to_pause_cnt = 0;
                group_state_ = PAUSING_TO_PAUSE;
                pausing_to_pause_request_ = false;
                LogProducer::warn("mc_sm","Group state switch to PAUSING_TO_PAUSE");
            }

            break;
        }

        case MANUAL:
        {
            if (manual_to_standby_request_)
            {
                manual_to_standby_cnt = 0;
                group_state_ = MANUAL_TO_STANDBY;
                manual_to_standby_request_ = false;
                LogProducer::warn("mc_sm","Group state state switch to MANUAL_TO_STANDBY");
            }

            break;
        }

		case OFFLINE:
        {
            fillOfflineCache();

            if (offline_to_standby_request_)
            {
                offline_to_standby_cnt = 0;
                group_state_ = OFFLINE_TO_STANDBY;
                offline_to_standby_request_ = false;
                LogProducer::warn("mc_sm","Group state switch to OFFLINE_TO_STANDBY");
            }

            break;
        }

        case PAUSE:
        {
            if (pause_to_auto_request_)
            {
                if (isSameJoint(pause_joint_, start_joint_))
                {
                    TrajectoryPoint point;
                    memset(&point, 0, sizeof(point));
                    point.level = POINT_MIDDLE;
                    point.time_stamp = 0;

                    LogProducer::info("mc_sm","Resume: remain traj size is %d", remain_trajectory_.size());

                    if (remain_trajectory_.size() > 0)
                    {
                        for (vector<JointState>::iterator it = remain_trajectory_.begin(); it != remain_trajectory_.end(); ++it)
                        {
                            point.state = *it;
                            traj_fifo_.push(point);
                        }
                    }
                    else
                    {
                        point.state.angle = pause_joint_;
                        traj_fifo_.push(point);
                    }

                    LogProducer::info("mc_sm","Resume: traj fifo size is %d", traj_fifo_.size());
                    
                    start_joint_ = start_joint_before_pause_;
                    prepare_resume_cnt = 0;
                    group_state_ = PREPARE_RESUME;
                    pause_to_auto_request_ = false;
                    LogProducer::warn("mc_sm","Group state switch to PREPARE_RESUME");
                }
                else
                {
                    group_state_ = PAUSE_TO_PAUSE_RETURN;
                    LogProducer::warn("mc_sm","Group state switch to PAUSE_TO_PAUSE_RETURN");
                }
            }

            if (pause_to_manual_request_)
            {
                manual_time_ = cycle_time_;
                start_of_motion_ = true;
                group_state_ = PAUSE_TO_PAUSE_MANUAL;
                pause_to_manual_request_ = false;
                LogProducer::warn("mc_sm","Group state switch to PAUSE_TO_PAUSE_MANUAL");
            }

            break;
        }

        case PAUSE_MANUAL:
        {
            if (manual_to_pause_request_)
            {
                pause_manual_to_pause_cnt = 0;
                group_state_ = PAUSE_MANUAL_TO_PAUSE;
                manual_to_pause_request_ = false;
                LogProducer::warn("mc_sm","Group state switch to PAUSE_MANUAL_TO_PAUSE");
            }

            break;
        }

        case RESUME:
        {
            if (resume_trajectory_.size() == 0)
            {
                group_state_ = AUTO;
                LogProducer::warn("mc_sm","Group state switch to AUTO.");
            }

            break;
        }

        case PREPARE_RESUME:
        {
            if ((traj_fifo_.size() > 250) || (prepare_resume_cnt > 250 && !traj_fifo_.empty()))
            {
                group_state_ = PAUSE_TO_RESUME;
                LogProducer::info("mc_sm","PREPARE_RESUME: traj fifo size is %d", traj_fifo_.size());
                LogProducer::warn("mc_sm","Group state switch to PAUSE_TO_RESUME.");
            }
            else
            {
                prepare_resume_cnt ++;

                if (prepare_resume_cnt > 300)
                {
                    group_state_ = PAUSE;
                    LogProducer::warn("mc_sm","Group state switch to PAUSE.");
                    LogProducer::warn("mc_sm","Pause to prepare resume time-out but trajectory-fifo still empty.");
                }
            }

            break;
        }

        case PAUSE_TO_RESUME:
        {
            ErrorCode err = planResumeTrajectory();

            if (err == SUCCESS)
            {
                group_state_ = RESUME;
                LogProducer::warn("mc_sm","Group state switch to RESUME.");
            }
            else
            {
                reportError(err);
                group_state_ = PAUSE;
                LogProducer::warn("mc_sm","Group state switch to PAUSE.");
            }

            break;
        }

        case PAUSE_RETURN:
        {
            if (pause_return_to_pause_request_)
            {
                pause_return_to_pause_cnt = 0;
                group_state_ = PAUSE_RETURN_TO_PAUSE;
                pause_return_to_pause_request_ = false;
                LogProducer::warn("mc_sm","Group state switch to PAUSE_RETURN_TO_PAUSE.");
            }

            break;
        }

        case PAUSE_RETURN_TO_PAUSE:
        {
            if (servo_state == SERVO_IDLE)
            {
                group_state_ = PAUSE;
                LogProducer::warn("mc_sm","Group state switch to PAUSE.");
            }
            else
            {
                pause_return_to_pause_cnt ++;

                if (pause_return_to_pause_cnt > auto_to_standby_timeout_)
                {
                    LogProducer::error("mc_sm","Pause return to pause timeout.");
                    reportError(MC_SWITCH_STATE_TIMEOUT);
                    group_state_ = PAUSE;
                }
            }

            break;
        }

        case PAUSE_TO_PAUSE_RETURN:
        {
            ErrorCode err = planPauseReturnTrajectory();

            if (err == SUCCESS)
            {
                group_state_ = PAUSE_RETURN;
                LogProducer::warn("mc_sm","Group state switch to PAUSE_RETURN.");
            }
            else
            {
                group_state_ = PAUSE;
                LogProducer::warn("mc_sm","Group state switch to PAUSE.");
            }
            
            break;
        }

        case PAUSE_TO_PAUSE_MANUAL:
        {
            doPauseToManual();
            break;
        }

        case PAUSE_MANUAL_TO_PAUSE:
        {
            doPauseManualToPause(servo_state,pause_manual_to_pause_cnt);
            break;
        }

        case STANDBY_TO_AUTO:
        {
            doStandbyToAuto(servo_state, standby_to_auto_cnt);
            break;
        }
        
        case AUTO_TO_STANDBY:
        {
            doAutoToStandby(servo_state, auto_to_standby_cnt, fine_counter);
            break;
        }

        case STANDBY_TO_MANUAL:
        {
            doStandbyToManual();
            break;
        }
            
        case MANUAL_TO_STANDBY:
        {
            doManualToStandby(servo_state, manual_to_standby_cnt);
            break;
        }

        case STANDBY_TO_OFFLINE:
        {
			doStandbyToOffline();
            break;
        }

        case OFFLINE_TO_STANDBY:
        {
            doOfflineToStandby(servo_state, offline_to_standby_cnt);
            break;
        }

        case PAUSING_TO_PAUSE:
        {
            doPausingToPause(servo_state, pausing_to_pause_cnt);
            break;
        }

        case AUTO_TO_PAUSING:
        {
            /*
            ErrorCode err = planPauseTrajectory();

            if (err != SUCCESS)
            {
                group_state_ = AUTO;
                LogProducer::info("mc_sm","Group-state switch to auto.");
                reportError(err);
            }
            else
            {
                group_state_ = PAUSING;
                LogProducer::info("mc_sm","Group-state switch to pausing.");
            }
            */

            break;
        }

        default:
        {
            LogProducer::error("mc_sm","Group-state is invalid: 0x%x", group_state);
            reportError(MC_INTERNAL_FAULT);
            break;
        }
    }
}


void BaseGroup::doPausingToPause(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		group_state_ = PAUSE;
        LogProducer::warn("mc_sm","Group state switch to pause.");
        return;
	}

	fail_counter ++;

	if (fail_counter > auto_to_pause_timeout_)
	{
        group_state_ = PAUSE;
		reportError(MC_SWITCH_STATE_TIMEOUT);
        LogProducer::error("mc_sm","Pausing to pause timeout.");
	}
}

void BaseGroup::doStandbyToOffline(void)
{
	pthread_mutex_lock(&offline_mutex_);
	offline_trajectory_cache_head_ = 0;
	offline_trajectory_cache_tail_ = 0;
	offline_trajectory_first_point_ = true;
	offline_trajectory_last_point_ = false;
	pthread_mutex_unlock(&offline_mutex_);
	while (fillOfflineCache());
    group_state_ = OFFLINE;
	LogProducer::warn("mc_sm","Group state switch to OFFLINE");
}

void BaseGroup::doOfflineToStandby(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		group_state_ = STANDBY;
		LogProducer::warn("mc_sm","Group state switch to STANDBY.");
	}

	fail_counter ++;

	if (fail_counter > offline_to_standby_timeout_)
	{
        group_state_ = STANDBY;
		reportError(MC_SWITCH_STATE_TIMEOUT);
        LogProducer::error("mc_sm","Offline to standby timeout.");
	}
}

void BaseGroup::doStandbyToAuto(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (traj_fifo_.full() || (fail_counter > 100 && !traj_fifo_.empty()))
	{
		group_state_ = AUTO;
		LogProducer::warn("mc_sm","Group state switch to AUTO, fifo-size = %d, counter = %d", traj_fifo_.size(), fail_counter);
	}

	fail_counter ++;
	
	if (fail_counter > standby_to_auto_timeout_ && traj_fifo_.empty())
	{
		group_state_ = STANDBY;
		LogProducer::warn("mc_sm","Group state switch to standby.");
		LogProducer::warn("mc_sm","Standby to auto time-out but trajectory-fifo still empty.");
	}
}

void BaseGroup::doAutoToStandby(const ServoState &servo_state, uint32_t &fail_counter, uint32_t &fine_counter)
{
    if (servo_state == SERVO_IDLE)
    {
        // 检查是否停稳，停稳后切换到standby
        PoseQuaternion fcp_in_base, tcp_in_base;
        kinematics_ptr_->doFK(getLatestJoint(), fcp_in_base);
        transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);

        if (getDistance(tcp_in_base.point_, fine_pose_.point_) < fine_threshold_)
        {
            fine_counter ++;

            if (fine_counter >= fine_cycle_)
            {
                group_state_ = STANDBY;
                LogProducer::warn("mc_sm","Group state switch to STANDBY.");
                return;
            }
        }
        else
        {
            fine_counter = 0;
        }
    }

    fail_counter ++;

    if (fail_counter > auto_to_standby_timeout_)
    {
        group_state_ = STANDBY;
        reportError(MC_SWITCH_STATE_TIMEOUT);
        LogProducer::error("mc_sm","Auto to standby timeout.");
        
        if (servo_state == SERVO_IDLE)
        {
            PoseQuaternion fcp_in_base, tcp_in_base;
            kinematics_ptr_->doFK(getLatestJoint(), fcp_in_base);
            transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
            const PoseQuaternion &pose = tcp_in_base;
            const PoseQuaternion &wait = fine_pose_;
            
            LogProducer::info("mc_sm","Current fcp in base: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f, %.4f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.quaternion_.w_, pose.quaternion_.x_, pose.quaternion_.y_, pose.quaternion_.z_);
            LogProducer::info("mc_sm","Waiting fcp in base: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f, %.4f", wait.point_.x_, wait.point_.y_, wait.point_.z_, wait.quaternion_.w_, wait.quaternion_.x_, wait.quaternion_.y_, wait.quaternion_.z_);
        }
        else
        {
            LogProducer::error("mc_sm","Servo-state: %d", servo_state);
        }
    }
}

void BaseGroup::doStandbyToManual(void)
{
    // 关节空间步进／连续运动不会超限，不会有奇异点问题，不需要检查轨迹
    if (manual_teach_.getManualFrame() == JOINT || manual_teach_.getManualMode() == APOINT)
    {
        manual_time_ = cycle_time_;
        start_of_motion_ = true;
        manual_trajectory_check_fail_ = false;
        manual_fifo_.clear();
        fillManualFIFO();
        group_state_ = MANUAL;
	    LogProducer::warn("mc_sm","Group state switch to MANUAL.");
        return;
    }
    
    // 笛卡尔空间运动需要检查轨迹是否超限／是否接近奇异点
    if (manual_teach_.getManualMode() == STEP)
    {
        double duration = manual_teach_.getDuration();
        double step_time = duration / 50;
        ErrorCode err = checkManualTrajectory(step_time, duration, step_time, start_joint_);

        if (err != SUCCESS)
        {
            reportError(err);
            group_state_ = STANDBY;
            LogProducer::warn("mc_sm","Group state switch to STANDBY.");
            return;
        }
    }
    else if (manual_teach_.getManualMode() == CONTINUOUS)
    {
        double duration = manual_teach_.getDuration() < 0.5 ? manual_teach_.getDuration() : 0.5;
        double step_time = duration / 50;
        ErrorCode err = checkManualTrajectory(step_time, duration, step_time, start_joint_);
        
        if (err != SUCCESS)
        {
            reportError(err);
            group_state_ = STANDBY;
            LogProducer::warn("mc_sm","Group state switch to STANDBY.");
            return;
        }
    }
    else
    {}
    
    manual_time_ = cycle_time_;
    start_of_motion_ = true;
    manual_trajectory_check_fail_ = false;
    manual_fifo_.clear();
    fillManualFIFO();
    group_state_ = MANUAL;
    LogProducer::warn("mc_sm","Group state switch to MANUAL.");
}

void BaseGroup::doPauseToManual(void)
{
    // 关节空间步进／连续运动不会超限，不会有奇异点问题，不需要检查轨迹
    if (manual_teach_.getManualFrame() == JOINT || manual_teach_.getManualMode() == APOINT)
    {
        manual_time_ = cycle_time_;
        start_of_motion_ = true;
        manual_trajectory_check_fail_ = false;
        manual_fifo_.clear();
        fillManualFIFO();
        group_state_ = PAUSE_MANUAL;
	    LogProducer::warn("mc_sm","Group state switch to PAUSE_MANUAL.");
        return;
    }
    
    // 笛卡尔空间运动需要检查轨迹是否超限／是否接近奇异点
    if (manual_teach_.getManualMode() == STEP)
    {
        double duration = manual_teach_.getDuration();
        double step_time = duration / 50;
        ErrorCode err = checkManualTrajectory(step_time, duration, step_time, start_joint_);

        if (err != SUCCESS)
        {
            reportError(err);
            group_state_ = PAUSE;
            LogProducer::warn("mc_sm","Group state switch to PAUSE.");
            return;
        }
    }
    else if (manual_teach_.getManualMode() == CONTINUOUS)
    {
        double duration = manual_teach_.getDuration() < 0.5 ? manual_teach_.getDuration() : 0.5;
        double step_time = duration / 50;
        ErrorCode err = checkManualTrajectory(step_time, duration, step_time, start_joint_);
        
        if (err != SUCCESS)
        {
            reportError(err);
            group_state_ = PAUSE;
            LogProducer::warn("mc_sm","Group state switch to PAUSE.");
            return;
        }
    }
    else
    {}
    
    manual_time_ = cycle_time_;
    start_of_motion_ = true;
    manual_trajectory_check_fail_ = false;
    manual_fifo_.clear();
    fillManualFIFO();
    group_state_ = PAUSE_MANUAL;
    LogProducer::warn("mc_sm","Group state switch to PAUSE_MANUAL.");
}

void BaseGroup::doManualToStandby(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		group_state_ = STANDBY;
		LogProducer::warn("mc_sm","Group state switch to STANDBY.");
	}

	fail_counter ++;

	if (fail_counter > manual_to_standby_timeout_)
	{
        group_state_ = STANDBY;
        reportError(MC_SWITCH_STATE_TIMEOUT);
		LogProducer::error("mc_sm","Manual to standby timeou.");
	}
}

void BaseGroup::doPauseManualToPause(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		group_state_ = PAUSE;
		LogProducer::warn("mc_sm","Group state switch to PAUSE.");
	}

	fail_counter ++;

	if (fail_counter > manual_to_standby_timeout_)
	{
        group_state_ = PAUSE;
        reportError(MC_SWITCH_STATE_TIMEOUT);
		LogProducer::error("mc_sm","Manual to pause timeout.");
	}
}


ErrorCode BaseGroup::stopGroup(void)
{
    LogProducer::info("mc_sm","Stop request received, group-state = 0x%x", group_state_);
    stop_barecore_ = true;
    return SUCCESS;
}

ErrorCode BaseGroup::clearGroup(void)
{
    LogProducer::info("mc_sm","Clear request received, group-state = 0x%x", group_state_);
    clear_request_ = true;
    return SUCCESS;
}

ErrorCode BaseGroup::clearTeachGroup(void)
{
    LogProducer::info("mc_sm","Clear teach request received, group-state = 0x%x", group_state_);
    clear_teach_request_ = true;
    return SUCCESS;
}

}
