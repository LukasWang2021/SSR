/*************************************************************************
	> File Name: motion_control_state_machine.cpp
	> Author: 
	> Mail: 
	> Created Time: 2019年11月29日 星期五 15时51分28秒
 ************************************************************************/
#include <basic_alg.h>
#include <motion_control_base_group.h>

using namespace basic_alg;

namespace fst_mc
{

void BaseGroup::doStateMachine(void)
{
    static uint32_t disable_to_standby_cnt = 0;
    static uint32_t standby_to_disable_cnt = 0;
    static uint32_t standby_to_auto_cnt = 0;
    static uint32_t auto_to_standby_cnt = 0;
    static uint32_t manual_to_standby_cnt = 0;
    static uint32_t auto_to_pause_cnt = 0;
    static uint32_t pause_to_pause_return_cnt = 0;
    static uint32_t pause_return_to_standby_cnt = 0;
    static uint32_t offline_to_standby_cnt = 0;

    static uint32_t fine_counter = 0;

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
            pthread_mutex_lock(&planner_list_mutex_);
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

            pthread_mutex_unlock(&planner_list_mutex_);
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
        
        pthread_mutex_lock(&planner_list_mutex_);
        pause_status_.pause_valid = false;
        pause_status_.pause_index = 0;
        auto_time_ = 0;
        trajectory_a_.valid = false;
        trajectory_b_.valid = false;
        plan_traj_ptr_ = &trajectory_a_;
        pick_traj_ptr_ = &trajectory_a_;
        traj_fifo_.clear();
        bare_core_.clearPointCache();
        fine_enable_ = false;
        standby_to_auto_request_ = false;
        pthread_mutex_unlock(&planner_list_mutex_);
        pthread_mutex_lock(&offline_mutex_);
        offline_request_ = false;
        offline_trajectory_cache_head_ = 0;
        offline_trajectory_cache_tail_ = 0;
        offline_trajectory_first_point_ = false;
        offline_trajectory_last_point_ = false;
        pthread_mutex_unlock(&offline_mutex_);
        standby_to_manual_request_ = false;
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

    if (offline_request_ && group_state_ != STANDBY)
    {
        offline_request_ = false;
    }

    if (auto_to_pause_request_ && group_state_ != AUTO)
    {
        auto_to_pause_request_ = false;
    }

    if (pause_to_auto_request_ && group_state_ != PAUSE)
    {
        pause_to_auto_request_ = false;
    }

    if (offline_to_standby_request_ && group_state_ != OFFLINE)
    {
        offline_to_standby_request_ = false;
    }

    if (standby_to_auto_request_ && group_state_ != STANDBY)
    {
        standby_to_auto_request_ = false;
    }

    if (standby_to_manual_request_ && group_state_ != STANDBY)
    {
        standby_to_manual_request_ = false;
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

            pthread_mutex_lock(&planner_list_mutex_);

            if (pick_traj_ptr_->valid && standby_to_auto_request_ == false)
            {
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
            }
            else if (standby_to_manual_request_)
            {
                manual_time_ = cycle_time_;
                start_of_motion_ = true;
                group_state_ = STANDBY_TO_MANUAL;
                standby_to_manual_request_ = false;
            }
            else if (offline_request_)
            {
                group_state_ = STANDBY_TO_OFFLINE;
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

		case OFFLINE:
        {
            fillOfflineCache();

            if (offline_to_standby_request_)
            {
                FST_WARN("group switch OFFLINE_TO_STANDBY");
                offline_to_standby_request_ = false;
                group_state_ = OFFLINE_TO_STANDBY;
                offline_to_standby_cnt = 0;
            }

            break;
        }

        case PAUSE:
        {
            if (pause_to_auto_request_)
            {
                pause_to_auto_request_ = false;

                /*
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
                */
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
                // FIXME
                group_state_ = STANDBY;
                // 检查是否停稳，停稳后切换到standby
                /*
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
                */
            }

            pause_return_to_standby_cnt ++;

            if (pause_return_to_standby_cnt > auto_to_standby_timeout_)
            {
                FST_ERROR("Auto to standby timeout, error-request asserted.");
                reportError(MC_SWITCH_STATE_TIMEOUT);
                error_request_ = true;
            }

            if (group_state_ == STANDBY)
            {
                // replan path cache
                FST_INFO("Pause return finished, replan path cache.");
                ErrorCode err = SUCCESS;
                //ErrorCode err = replanPathCache();

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

            // FIXME
            /*
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
            */

            break;
        }

        case DISABLE_TO_STANDBY:
        {
			doDisableToStandby(servo_state, disable_to_standby_cnt);
            break;
        }

        case STANDBY_TO_DISABLE:
        {
			doStandbyToDisable(servo_state, standby_to_disable_cnt);
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

        case AUTO_TO_PAUSE:
        {
			doAutoToPause(servo_state, auto_to_pause_cnt);
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
                static size_t unknow_cnt = 0;
                unknow_cnt++;

                if (unknow_cnt > 1000)
                {
                    unknow_cnt = 0;
                    FST_WARN("Group-state is unknow but servo-state is %d, send stop request to barecore.", servo_state);
                    bare_core_.stopBareCore();
                    //FST_ERROR("Group-state is UNKNOW but servo-state is %d", servo_state);
                    reportError(MC_INTERNAL_FAULT);
                }
                
                //FST_WARN("Group-state switch to disable.");
                //group_state_ = DISABLE;
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

void BaseGroup::doAutoToPause(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		// FIXME
		group_state_ = PAUSE;
		// 检查是否停稳，停稳后切换到pause
		/*
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
		*/
	}

	fail_counter ++;

	if (fail_counter > auto_to_pause_timeout_)
	{
		FST_ERROR("Auto to pause timeout, error-request asserted.");
		reportError(MC_SWITCH_STATE_TIMEOUT);
		error_request_ = true;
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
	FST_WARN("Group-state switch to offline");
	group_state_ = OFFLINE;
}

void BaseGroup::doOfflineToStandby(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		group_state_ = STANDBY;
		FST_WARN("Group-state switch to standby.");
	}

	fail_counter ++;

	if (fail_counter > offline_to_standby_timeout_)
	{
		FST_ERROR("Offline to standby timeout, error-request asserted.");
		reportError(MC_SWITCH_STATE_TIMEOUT);
		error_request_ = true;
	}
}

void BaseGroup::doStandbyToAuto(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (traj_fifo_.full() || (fail_counter > 100 && !traj_fifo_.empty()))
	{
		group_state_ = AUTO;
		FST_WARN("Group-state switch to auto, fifo-size = %d, counter = %d", traj_fifo_.size(), fail_counter);
	}

	fail_counter ++;
	
	if (fail_counter > standby_to_auto_timeout_ && traj_fifo_.empty())
	{
		group_state_ = STANDBY;
		FST_WARN("Group-state switch to standby.");
		FST_WARN("Standby to auto time-out but trajectory-fifo still empty.");
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
                FST_WARN("Group-state switch to standby.");
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
        FST_ERROR("Auto to standby timeout, error-request asserted.");
        reportError(MC_SWITCH_STATE_TIMEOUT);
        error_request_ = true;
        
        if (servo_state == SERVO_IDLE)
        {
            PoseQuaternion fcp_in_base, tcp_in_base;
            kinematics_ptr_->doFK(getLatestJoint(), fcp_in_base);
            transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, tcp_in_base);
            const PoseQuaternion &pose = tcp_in_base;
            const PoseQuaternion &wait = fine_pose_;
            
            FST_INFO("Current fcp in base: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f, %.4f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.quaternion_.w_, pose.quaternion_.x_, pose.quaternion_.y_, pose.quaternion_.z_);
            FST_INFO("Waiting fcp in base: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f, %.4f", wait.point_.x_, wait.point_.y_, wait.point_.z_, wait.quaternion_.w_, wait.quaternion_.x_, wait.quaternion_.y_, wait.quaternion_.z_);
        }
        else
        {
            FST_ERROR("Servo-state: %d", servo_state);
        }
    }
}

void BaseGroup::doStandbyToManual(void)
{
	group_state_ = MANUAL;
	FST_WARN("Group-state switch to manual.");
}

void BaseGroup::doManualToStandby(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_IDLE)
	{
		group_state_ = STANDBY;
		FST_WARN("Group-state switch to standby.");
	}

	fail_counter ++;

	if (fail_counter > manual_to_standby_timeout_)
	{
		FST_ERROR("Manual to standby timeout, error-request asserted.");
		reportError(MC_SWITCH_STATE_TIMEOUT);
		error_request_ = true;
	}
}

void BaseGroup::doStandbyToDisable(const ServoState &servo_state, uint32_t &fail_counter)
{
	if (servo_state == SERVO_DISABLE)
	{
		FST_WARN("Group-state switch to disable.");
		group_state_ = DISABLE;
	}
	else
	{
		fail_counter ++;

		if (fail_counter > standby_to_disable_timeout_)
		{
			FST_WARN("Stop to disable timeout, group-state switch to STANDBY.");
			group_state_ = STANDBY;
		}
	}
}

void BaseGroup::doDisableToStandby(const ServoState &servo_state, uint32_t &fail_counter)
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

	fail_counter ++;

	if (fail_counter > disable_to_standby_timeout_)
	{
		FST_WARN("Reset to standby timeout, group-state switch to disable.");
		group_state_ = DISABLE;
	}
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


}
