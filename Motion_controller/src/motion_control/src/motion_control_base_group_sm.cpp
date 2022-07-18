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
    static uint32_t pausing_offline_to_pause_cnt = 0;
    static uint32_t prepare_resume_cnt = 0;
    static uint32_t pause_manual_to_pause_cnt = 0;
    static uint32_t fine_counter = 0;

    MotionControlState mc_state = mc_state_;
    ServoState servo_state = getServoState();

    if (clear_request_ && (mc_state == STANDBY || mc_state == PAUSE || mc_state == PAUSED_OFFLINE))
    {
        LogProducer::info("mc_sm","Clear group, MC-state = %s", getMontionControlStatusString(mc_state).c_str());
        mc_state_ = STANDBY;
        
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

        pthread_mutex_lock(&online_traj_mutex_);
        online_trajectory_last_point_ = false;
        pthread_mutex_unlock(&online_traj_mutex_);
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
        pthread_mutex_lock(&manual_traj_mutex_);
        manual_time_ = 0;
        clear_teach_request_ = false;
        manual_trajectory_check_fail_ = false;

        if (mc_state == STANDBY || mc_state == MANUAL || mc_state == MANUAL_TO_STANDBY || mc_state == STANDBY_TO_MANUAL)
        {
            mc_state_ = STANDBY;
            standby_to_manual_request_ = false;
            manual_to_standby_request_ = false;
        }
        else if (mc_state == PAUSE || mc_state == PAUSE_MANUAL || mc_state == PAUSE_TO_PAUSE_MANUAL || mc_state == PAUSE_MANUAL_TO_PAUSE)
        {
            mc_state_ = PAUSE;
            pause_to_manual_request_ = false;
            manual_to_pause_request_ = false;
        }
        bare_core_.clearPointCache();

        pthread_mutex_unlock(&manual_traj_mutex_);
        LogProducer::info("mc_sm","Teach group cleared, MC-state = %s", getMontionControlStatusString(mc_state).c_str());
    }

    if (standby_to_offline_request_ && mc_state != STANDBY)
    {
        standby_to_offline_request_ = false;
    }

    if (offline_to_standby_request_ && mc_state != OFFLINE)
    {
        offline_to_standby_request_ = false;
    }

    if (standby_to_auto_request_ && mc_state != STANDBY)
    {
        standby_to_auto_request_ = false;
    }

    if (auto_to_standby_request_ && mc_state != AUTO)
    {
        auto_to_standby_request_ = false;
    }

    if (standby_to_manual_request_ && mc_state != STANDBY)
    {
        standby_to_manual_request_ = false;
    }
    if (standby_to_online_request_ && mc_state != STANDBY)
    {
        standby_to_online_request_ = false;
    }
    if (manual_to_standby_request_ && mc_state != MANUAL)
    {
        manual_to_standby_request_ = false;
    }

    if (pause_to_manual_request_ && mc_state != PAUSE)
    {
        pause_to_manual_request_ = false;
    }

    if (manual_to_pause_request_ && mc_state != PAUSE_MANUAL)
    {
        manual_to_pause_request_ = false;
    }

    if (auto_to_pause_request_ && mc_state != AUTO)
    {
        auto_to_pause_request_ = false;
    }

    if (pause_to_auto_request_ && mc_state != PAUSE && mc_state != PAUSE_TO_PAUSE_RETURN && mc_state != PAUSE_RETURN && mc_state != PAUSE_RETURN_TO_PAUSE)
    {
        pause_to_auto_request_ = false;
    }


    if (stop_barecore_ && (servo_state == SERVO_DISABLE))
    {
        LogProducer::info("mc_sm","Barecore stop, MC-state = %s, servo-state = %s", getMontionControlStatusString(mc_state).c_str(), getMCServoStatusString(servo_state).c_str());
        stop_barecore_ = false;

        if (pause_to_auto_request_) pause_to_auto_request_ = false;

        if (mc_state == MANUAL || mc_state == MANUAL_TO_STANDBY || mc_state == STANDBY_TO_MANUAL ||
            mc_state == OFFLINE || mc_state == OFFLINE_TO_STANDBY || mc_state == STANDBY_TO_OFFLINE || mc_state == ONLINE)
        {
            mc_state_ = STANDBY;
            clear_request_ = true;
            LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY.");
        }
        else if (mc_state == PAUSE_MANUAL || mc_state == PAUSE_TO_PAUSE_MANUAL || mc_state == PAUSE_MANUAL_TO_PAUSE ||
                 mc_state == PAUSING_TO_PAUSE || mc_state == PAUSE_RETURN_TO_PAUSE || mc_state == PREPARE_RESUME)
        {
            mc_state_ = PAUSE;
            LogProducer::warn("mc_sm","1 MC-state switch to MC_PAUSE.");
        }
        else if (mc_state == PAUSING)
        {
            pause_trajectory_.clear();
            traj_fifo_.clear();
            bare_core_.clearPointCache();
            mc_state_ = PAUSE;
            LogProducer::warn("mc_sm","2 MC-state switch to MC_PAUSE.");
        }
        else if (mc_state == PAUSE_RETURN)
        {
            resume_trajectory_.clear();
            traj_fifo_.clear();
            bare_core_.clearPointCache();
            mc_state_ = PAUSE;
            LogProducer::warn("mc_sm","3 MC-state switch to MC_PAUSE.");
        }
        else if (mc_state == RESUME)
        {
            mc_state_ = PAUSE;
            LogProducer::warn("mc_sm","4 MC-state switch to MC_PAUSE.");
            remain_trajectory_.assign(origin_trajectory_.begin(), origin_trajectory_.end());
            start_joint_before_pause_ = start_joint_;
            pause_joint_ = remain_trajectory_.front().angle;
            resume_trajectory_.clear();
            traj_fifo_.clear();
            bare_core_.clearPointCache();
        }
        else if (mc_state == AUTO_TO_PAUSING || mc_state == PAUSE_TO_RESUME || mc_state == PAUSE_TO_PAUSE_RETURN)
        {
            
        }
        else if (mc_state == AUTO_TO_STANDBY)
        {
            mc_state_ = STANDBY;
            LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY.");
        }
        else if (mc_state == AUTO && auto_to_standby_request_ == true)
        {
            mc_state_ = STANDBY;
            bare_core_.clearPointCache();
            LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY.");
        }
        else if (mc_state == AUTO || mc_state == STANDBY_TO_AUTO)
        {
            mc_state_ = PAUSE;
            LogProducer::warn("mc_sm","5 MC-state switch to MC_PAUSE.");
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

    switch (mc_state)
    {
        case STANDBY:
        {
            pthread_mutex_lock(&planner_list_mutex_);

            if (pick_traj_ptr_->valid && standby_to_auto_request_ == false)
            {
                LogProducer::info("mc_sm","MC-state: MC_STANDBY, pick_traj_ptr_->valid is true but standby_to_auto_request_ is false, set request");
                standby_to_auto_request_ = true;
            }
    
            pthread_mutex_unlock(&planner_list_mutex_);

            if (standby_to_auto_request_)
            {
                standby_to_auto_cnt = 0;
                auto_time_ = cycle_time_;
                start_of_motion_ = true;
                mc_state_ = STANDBY_TO_AUTO;
                standby_to_auto_request_ = false;
                LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY_TO_AUTO");
            }
            else if (standby_to_manual_request_)
            {
                mc_state_ = STANDBY_TO_MANUAL;
                standby_to_manual_request_ = false;
                LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY_TO_MANUAL");
            }
            else if (standby_to_offline_request_)
            {
                mc_state_ = STANDBY_TO_OFFLINE;
                LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY_TO_OFFLINE");
            }
            else if(standby_to_online_request_)
            {
                mc_state_ = ONLINE;
                LogProducer::warn("mc_sm","MC-state switch to ONLINE from STANDBY");
                online_time_ = 0;
	            online_trajectory_last_point_ = false;
            }
            break;
        }

        case AUTO:
        {
            if (auto_to_standby_request_)
            {
                fine_counter = 0;
                auto_to_standby_cnt = 0;
                mc_state_ = AUTO_TO_STANDBY;
                auto_to_standby_request_ = false;
                LogProducer::warn("mc_sm","MC-state switch to MC_AUTO_TO_STANDBY");
            }

            if (auto_to_pause_request_ && traj_fifo_.size() > 250)
            {
                mc_state_ = AUTO_TO_PAUSING;
                auto_to_pause_request_ = false;
                LogProducer::warn("mc_sm","MC-state switch to MC_AUTO_TO_PAUSING");
            }

            break;
        }

        case PAUSING:
        {
            if (pausing_to_pause_request_)
            {
                pausing_to_pause_cnt = 0;
                mc_state_ = PAUSING_TO_PAUSE;
                pausing_to_pause_request_ = false;
                LogProducer::warn("mc_sm","MC-state switch to MC_PAUSING_TO_PAUSE");
            }

            break;
        }

        case MANUAL:
        {
            if (manual_to_standby_request_)
            {
                manual_to_standby_cnt = 0;
                mc_state_ = MANUAL_TO_STANDBY;
                manual_to_standby_request_ = false;
                LogProducer::warn("mc_sm","MC-state state switch to MC_MANUAL_TO_STANDBY");
            }
            handleContinueousManualRpcTimeOut();
            break;
        }

		case OFFLINE:
        {
            fillOfflineCache();

            if(offline_to_pause_request_)
            {
                mc_state_ = OFFLINE_TO_PAUSING;
                offline_to_pause_request_ = false;
                LogProducer::warn("mc_sm","MC-state switch to MC_OFFLINE_TO_PAUSING");
            }

            if (offline_to_standby_request_)
            {
                offline_to_standby_cnt = 0;
                mc_state_ = OFFLINE_TO_STANDBY;
                offline_to_standby_request_ = false;
                LogProducer::warn("mc_sm","MC-state switch to MC_OFFLINE_TO_STANDBY");
            }

            break;
        }

        case OFFLINE_TO_PAUSING:
        {
            pause_joint_ = pause_trajectory_.back().angle;
            mc_state_ = PAUSING_OFFLINE;
            LogProducer::info("mc_sm","MC-state switch to PAUSING_OFFLINE.");
            break;
        }
        case PAUSING_OFFLINE:
        {
            if (pausing_offline_to_pause_request_)
            {
                // pausing_to_pause_cnt = 0;
                offline_to_standby_cnt = 0;
                mc_state_ = PAUSING_OFFLINE_TO_PAUSE;
                pausing_offline_to_pause_request_ = false;
                LogProducer::warn("mc_sm","MC-state switch to offline MC_PAUSING_TO_PAUSE");
            }
        }
        case PAUSED_OFFLINE:
        {
            if(pause_to_offline_request_) // resume
            {
                // check is the same 
                if (!isSameJoint(pause_joint_, start_joint_))
                {
                    mc_state_ = STANDBY;
                    pause_to_offline_request_ = false;
                    LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY");
                    break;
                }
                pause_joint_ = pause_trajectory_.back().angle;
                doStandbyToOffline();
                LogProducer::info("mc_sm","MC-state switch to OFFLINE.");
                pause_to_offline_request_ = false;
            }
            break;
        }

        case ONLINE:
        {
            if (online_to_standby_request_)
            {
                mc_state_ = STANDBY;
                online_to_standby_request_ = false;
                online_fifo_.clear();
                
                LogProducer::warn("mc_sm","MC-state switch to MC__STANDBY");
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
                    mc_state_ = PREPARE_RESUME;
                    pause_to_auto_request_ = false;
                    LogProducer::warn("mc_sm","MC-state switch to MC_PREPARE_RESUME");
                }
                else
                {
                    mc_state_ = PAUSE_TO_PAUSE_RETURN;
                    LogProducer::warn("mc_sm","MC-state switch to MC_PAUSE_TO_PAUSE_RETURN");
                }
            }

            if (pause_to_manual_request_)
            {
                manual_time_ = cycle_time_;
                start_of_motion_ = true;
                mc_state_ = PAUSE_TO_PAUSE_MANUAL;
                pause_to_manual_request_ = false;
                LogProducer::warn("mc_sm","MC-state switch to MC_PAUSE_TO_PAUSE_MANUAL");
            }
            break;
        }

        case PAUSE_MANUAL:
        {
            if (manual_to_pause_request_)
            {
                pause_manual_to_pause_cnt = 0;
                mc_state_ = PAUSE_MANUAL_TO_PAUSE;
                manual_to_pause_request_ = false;
                LogProducer::warn("mc_sm","MC-state switch to MC_PAUSE_MANUAL_TO_PAUSE");
            }

            break;
        }
        case RESUME:
        {
            if (resume_trajectory_.size() == 0)
            {
                mc_state_ = AUTO;
                LogProducer::warn("mc_sm","MC-state switch to MC_AUTO.");
            }

            break;
        }

        case PREPARE_RESUME:
        {
            if ((traj_fifo_.size() > 250) || (prepare_resume_cnt > 250 && !traj_fifo_.empty()))
            {
                mc_state_ = PAUSE_TO_RESUME;
                LogProducer::info("mc_sm","PREPARE_RESUME: traj fifo size is %d", traj_fifo_.size());
                LogProducer::warn("mc_sm","MC-state switch to MC_PAUSE_TO_RESUME.");
            }
            else
            {
                prepare_resume_cnt ++;

                if (prepare_resume_cnt > 300)
                {
                    mc_state_ = PAUSE;
                    LogProducer::warn("mc_sm","6 MC-state switch to MC_PAUSE.");
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
                mc_state_ = RESUME;
                LogProducer::warn("mc_sm","MC-state switch to MC_RESUME.");
            }
            else
            {
                reportError(err);
                mc_state_ = PAUSE;
                LogProducer::warn("mc_sm","7 MC-state switch to MC_PAUSE.");
            }

            break;
        }

        case PAUSE_RETURN:
        {
            if (pause_return_to_pause_request_)
            {
                pause_return_to_pause_cnt = 0;
                mc_state_ = PAUSE_RETURN_TO_PAUSE;
                pause_return_to_pause_request_ = false;
                LogProducer::warn("mc_sm","MC-state switch to MC_PAUSE_RETURN_TO_PAUSE.");
            }

            break;
        }

        case PAUSE_RETURN_TO_PAUSE:
        {
            if (servo_state == SERVO_IDLE)
            {
                mc_state_ = PAUSE;
                LogProducer::warn("mc_sm","MC-state switch to PAUSE.");
            }
            else
            {
                pause_return_to_pause_cnt ++;

                if (pause_return_to_pause_cnt > auto_to_standby_timeout_)
                {
                    LogProducer::error("mc_sm","Pause return to pause timeout.");
                    reportError(MC_SWITCH_STATE_TIMEOUT);
                    mc_state_ = PAUSE;
                }
            }

            break;
        }

        case PAUSE_TO_PAUSE_RETURN:
        {
            ErrorCode err = planPauseReturnTrajectory();

            if (err == SUCCESS)
            {
                mc_state_ = PAUSE_RETURN;
                LogProducer::warn("mc_sm","MC-state switch to MC_PAUSE_RETURN.");
            }
            else
            {
                mc_state_ = PAUSE;
                LogProducer::warn("mc_sm","8 MC-state switch to MC_PAUSE.");
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

        case PAUSING_OFFLINE_TO_PAUSE:
        {
            doPausingOfflineToPause(servo_state, pausing_offline_to_pause_cnt);
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
                mc_state_ = AUTO;
                LogProducer::info("mc_sm","MC-state switch to MC_AUTO.");
                reportError(err);
            }
            else
            {
                mc_state_ = PAUSING;
                LogProducer::info("mc_sm","MC-state switch to MC_PAUSING.");
            }
            */

            break;
        }

        default:
        {
            LogProducer::error("mc_sm","MC-state is invalid: 0x%x", getMontionControlStatusString(mc_state).c_str());
            reportError(MC_INTERNAL_FAULT);
            break;
        }
    }
}


void BaseGroup::doPausingToPause(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		mc_state_ = PAUSE;
        LogProducer::warn("mc_sm","9 MC-state switch to MC_PAUSE.");
        return;
	}

	fail_counter ++;

	if (fail_counter > auto_to_pause_timeout_)
	{
        mc_state_ = PAUSE;
		reportError(MC_SWITCH_STATE_TIMEOUT);
        LogProducer::error("mc_sm","Pausing to pause timeout.");
	}
}

void BaseGroup::doPausingOfflineToPause(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		mc_state_ = PAUSED_OFFLINE;
        LogProducer::warn("mc_sm","MC-state switch to offline MC_PAUSE.");
        return;
	}

	fail_counter ++;

	if (fail_counter > auto_to_pause_timeout_)
	{
        mc_state_ = PAUSED_OFFLINE;
		reportError(MC_SWITCH_STATE_TIMEOUT);
        LogProducer::error("mc_sm","offline pausing to pause timeout.");
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
    mc_state_ = OFFLINE;
	LogProducer::warn("mc_sm","MC-state switch to MC_OFFLINE");
}

void BaseGroup::doOfflineToStandby(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		mc_state_ = STANDBY;
		LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY.");
	}

	fail_counter ++;

	if (fail_counter > offline_to_standby_timeout_)
	{
        mc_state_ = STANDBY;
		reportError(MC_SWITCH_STATE_TIMEOUT);
        LogProducer::error("mc_sm","Offline to standby timeout.");
	}
}

void BaseGroup::doStandbyToAuto(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (traj_fifo_.full() || (fail_counter > 100 && !traj_fifo_.empty()))
	{
		mc_state_ = AUTO;
		LogProducer::warn("mc_sm","MC-state switch to MC_AUTO, fifo-size = %d, counter = %d", traj_fifo_.size(), fail_counter);
	}

	fail_counter ++;
	
	if (fail_counter > standby_to_auto_timeout_ && traj_fifo_.empty())
	{
		mc_state_ = STANDBY;
		LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY.");
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
                mc_state_ = STANDBY;
                LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY.");
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
        mc_state_ = STANDBY;
        reportError(MC_SWITCH_STATE_TIMEOUT);
        LogProducer::error("mc_sm","MC_AUTO to MC_STANDBY timeout.");
        
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
            LogProducer::error("mc_sm","Servo-state: %s", getMCServoStatusString(servo_state).c_str());
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
        mc_state_ = MANUAL;
	    LogProducer::warn("mc_sm","MC-state switch to MC_MANUAL.");
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
            mc_state_ = STANDBY;
            LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY.");
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
            mc_state_ = STANDBY;
            LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY.");
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
    mc_state_ = MANUAL;
    LogProducer::warn("mc_sm","MC-state switch to MC_MANUAL.");
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
        mc_state_ = PAUSE_MANUAL;
	    LogProducer::warn("mc_sm","MC-state switch to MC_PAUSE_MANUAL.");
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
            mc_state_ = PAUSE;
            LogProducer::warn("mc_sm","10 MC-state switch to MC_PAUSE.");
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
            mc_state_ = PAUSE;
            LogProducer::warn("mc_sm","11 MC-state switch to MC_PAUSE.");
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
    mc_state_ = PAUSE_MANUAL;
    LogProducer::warn("mc_sm","MC-state switch to MC_PAUSE_MANUAL.");
}

void BaseGroup::doManualToStandby(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		mc_state_ = STANDBY;
        clearTeachGroup();//todo check
		LogProducer::warn("mc_sm","MC-state switch to MC_STANDBY.");
	}

	fail_counter ++;

	if (fail_counter > manual_to_standby_timeout_)
	{
        mc_state_ = STANDBY;
        reportError(MC_SWITCH_STATE_TIMEOUT);
		LogProducer::error("mc_sm","Manual to standby timeout. servo_state=%d", servo_state);
	}
}

void BaseGroup::doPauseManualToPause(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		mc_state_ = PAUSE;
		LogProducer::warn("mc_sm","12 MC-state switch to MC_PAUSE.");
	}

	fail_counter ++;

	if (fail_counter > manual_to_standby_timeout_)
	{
        mc_state_ = PAUSE;
        reportError(MC_SWITCH_STATE_TIMEOUT);
		LogProducer::error("mc_sm","Manual to pause timeout.");
	}
}


ErrorCode BaseGroup::stopGroup(void)
{
    LogProducer::info("mc_sm","Stop request received, MC-state = %s", getMontionControlStatusString(mc_state_).c_str());
    stop_barecore_ = true;
    return SUCCESS;
}

ErrorCode BaseGroup::clearGroup(void)
{
    LogProducer::info("mc_sm","Clear request received, MC-state = %s", getMontionControlStatusString(mc_state_).c_str());
    clear_request_ = true;
    return SUCCESS;
}

ErrorCode BaseGroup::clearTeachGroup(void)
{
    LogProducer::info("mc_sm","Clear teach request received, MC-state = %s", getMontionControlStatusString(mc_state_).c_str());
    clear_teach_request_ = true;
    is_continuous_manual_move_timeout_ = false;
    is_continuous_manual_time_count_valid_ = false;
    return SUCCESS;
}

MotionControlState BaseGroup::getMotionControlState(void)
{
    return mc_state_;
}

}
