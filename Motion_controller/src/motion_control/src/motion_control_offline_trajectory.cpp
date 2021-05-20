/*************************************************************************
	> File Name: motion_control_offline_trajectory.cpp
	> Author: 
	> Mail: 
	> Created Time: 2019年11月29日 星期五 15时07分13秒
 ************************************************************************/
#include <fstream>
#include <sstream>
#include <math.h>
#include <common_error_code.h>
#include <motion_control_base_group.h>
#include "log_manager_producer.h"

using namespace std;
using namespace basic_alg;
using namespace log_space;

namespace group_space
{

ErrorCode BaseGroup::setOfflineTrajectory(const std::string &offline_trajectory)
{
    if (mc_state_ == OFFLINE || mc_state_ == OFFLINE_TO_STANDBY || mc_state_ == STANDBY_TO_OFFLINE)
    {
        LogProducer::error("mc_offline_traj","Fail to set offline trajectory, state = 0x%x", mc_state_);
        return INVALID_SEQUENCE;
    }

    if (offline_trajectory_file_.is_open())
    {
        offline_trajectory_file_.close();
    }

    offline_trajectory_file_.open(offline_trajectory);

    if (!offline_trajectory_file_.is_open())
    {
        LogProducer::error("mc_offline_traj","Fail to open offline trajectory file");
        return INVALID_PARAMETER;
    }

    uint32_t joint_num;
    double cycle_time;
    char line_str[512];
    offline_trajectory_file_.getline(line_str, sizeof(line_str));
    stringstream ss;
    ss << line_str;

    ss >> joint_num;
    ss >> cycle_time;
    ss >> offline_trajectory_size_;

    if (joint_num != getNumberOfJoint())
    {
        LogProducer::error("mc_offline_traj","Invalid num of joint = %d, while %d expected", joint_num, getNumberOfJoint());
        return INVALID_PARAMETER;
    }

    if (fabs(cycle_time - cycle_time_) > MINIMUM_E9)
    {
        LogProducer::error("mc_offline_traj","Invalid cycle time = %.9f, while %.9f expected", cycle_time, cycle_time_);
        return INVALID_PARAMETER;
    }

    if (offline_trajectory_size_ == 0 || offline_trajectory_size_ > 60 * 1000)
    {
        LogProducer::error("mc_offline_traj","Invalid trajectory size = %d", offline_trajectory_size_);
        return INVALID_PARAMETER;
    }

    offline_trajectory_file_.getline(line_str, sizeof(line_str));
    ss.clear();
    ss << line_str;

    for (uint32_t i = 0; i < joint_num; i++)
    {
        ss >> offline_start_joint_[i];
    }

    char buffer[LOG_TEXT_SIZE];
    offline_trajectory_time_stamp_ = cycle_time_;
    LogProducer::info("mc_offline_traj","Set offline trajectory success, trajectory size: %d", offline_trajectory_size_);
    LogProducer::info("mc_offline_traj","Start of the trajectory: %s", printDBLine(&offline_start_joint_[0], buffer, LOG_TEXT_SIZE));

    return SUCCESS;
}

Joint BaseGroup::getStartJointOfOfflineTrajectory(void)
{
    return offline_start_joint_;
}

ErrorCode BaseGroup::moveOfflineTrajectory(void)
{
    standby_to_offline_request_ = true;
    return SUCCESS;
}

uint32_t BaseGroup::getOfflineCacheSize(void)
{
    if (offline_trajectory_cache_tail_ >= offline_trajectory_cache_head_)
    {
        return offline_trajectory_cache_tail_ - offline_trajectory_cache_head_;
    }
    
    return offline_trajectory_cache_tail_ + OFFLINE_TRAJECTORY_CACHE_SIZE - offline_trajectory_cache_head_;
}

bool BaseGroup::fillOfflineCache(void)
{
    const static uint32_t joint_number = getNumberOfJoint();
    const static uint32_t local_cache_size = 50;
    static uint32_t picked_number = 0;
    static char line_str[512];
    static TrajectoryPoint local_cache[local_cache_size];

    pthread_mutex_lock(&offline_mutex_);

    if (getOfflineCacheSize() + local_cache_size >= OFFLINE_TRAJECTORY_CACHE_SIZE)
    {
        pthread_mutex_unlock(&offline_mutex_);
        return false;
    }

    pthread_mutex_unlock(&offline_mutex_);

    for(picked_number = 0; picked_number < local_cache_size; picked_number++)
    {
        if (offline_trajectory_file_.getline(line_str, sizeof(line_str)))
        {
            stringstream ss;
            ss << line_str;

            for (uint32_t j = 0; j < joint_number; j++)
            {
                ss >> local_cache[picked_number].state.angle[j];
            }
            
            for (uint32_t j = 0; j < joint_number; j++)
            {
                ss >> local_cache[picked_number].state.omega[j];
            }

            for (uint32_t j = 0; j < joint_number; j++)
            {
                ss >> local_cache[picked_number].state.alpha[j];
            }

            for (uint32_t j = 0; j < joint_number; j++)
            {
                ss >> local_cache[picked_number].state.torque[j];
            }
        }
        else
        {
            offline_trajectory_last_point_ = true;
            break;
        }

        local_cache[picked_number].time_stamp = offline_trajectory_time_stamp_;
        local_cache[picked_number].level = POINT_MIDDLE;
        offline_trajectory_time_stamp_ += cycle_time_;
    }

    if (picked_number == 0)
    {
        return false;
    }

    if (offline_trajectory_first_point_)
    {
        local_cache[0].level = POINT_START;
        offline_trajectory_first_point_ = false;
    }

    if (offline_trajectory_last_point_)
    {
        local_cache[picked_number - 1].level = POINT_ENDING;
        start_joint_ = local_cache[picked_number - 1].state.angle;
        offline_trajectory_last_point_ = false;
    }
    
    pthread_mutex_lock(&offline_mutex_);

    for (uint32_t i = 0; i < picked_number; i++)
    {
        offline_trajectory_cache_[offline_trajectory_cache_tail_] = local_cache[i];
        offline_trajectory_cache_tail_ = offline_trajectory_cache_tail_ + 1 < OFFLINE_TRAJECTORY_CACHE_SIZE ? offline_trajectory_cache_tail_ + 1 : 0;
    }

    pthread_mutex_unlock(&offline_mutex_);
    return true;
}

ErrorCode BaseGroup::pickPointsFromOfflineCache(TrajectoryPoint *points, size_t &length)
{
    ErrorCode err = SUCCESS;
    uint32_t picked_num = 0;
    pthread_mutex_lock(&offline_mutex_);

    for (uint32_t i = 0; i < length; i++)
    {
        if (getOfflineCacheSize() != 0)
        {
            points[i] = offline_trajectory_cache_[offline_trajectory_cache_head_];
            offline_trajectory_cache_head_ = offline_trajectory_cache_head_ + 1 < OFFLINE_TRAJECTORY_CACHE_SIZE ? offline_trajectory_cache_head_ + 1 : 0;
            picked_num++;
        }
        else
        {
            break;
        }
    }

    if (traj_log_enable_)
    {
        for (uint32_t i = 0; i < picked_num; i++)
        {
            traj_log_data_ptr_[traj_log_ctrl_ptr_->write_index] = points[i];
            traj_log_ctrl_ptr_->write_index = (traj_log_ctrl_ptr_->write_index + 1 < traj_log_ctrl_ptr_->max_of_points) ? (traj_log_ctrl_ptr_->write_index + 1) : 0;
            traj_log_ctrl_ptr_->num_of_points = (traj_log_ctrl_ptr_->num_of_points < traj_log_ctrl_ptr_->max_of_points) ? (traj_log_ctrl_ptr_->num_of_points + 1) : traj_log_ctrl_ptr_->max_of_points;
        }
    }

    pthread_mutex_unlock(&offline_mutex_);
    length = picked_num;
    return err;
}

ErrorCode BaseGroup::sendOfflineTrajectoryFlow(void)
{
    if (bare_core_.isPointCacheEmpty())
    {
        size_t length = 10;
        TrajectoryPoint points[10];
        ErrorCode err = pickPointsFromOfflineCache(points, length);
        //LogProducer::info("mc_offline_traj","sendOfflineTrajectoryFlow: %d, head=%d, tail=%d", length, offline_trajectory_cache_head_, offline_trajectory_cache_tail_);

        if (err != SUCCESS)
        {
            LogProducer::error("mc_offline_traj","sendOfflineTrajectoryFlow: cannot pick point from trajectory fifo.");
            return err;
        }

        bare_core_.fillPointCache(points, length, POINT_POS_VEL);

        if (points[length - 1].level == POINT_ENDING)
        {
            // 取到了ENDING-POINT，意味着轨迹FIFO已经取完,必须要切换状态机
            if (mc_state_ == OFFLINE)
            {
                offline_to_standby_request_ = true;
            }
        }
    }

    return bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
}


}

