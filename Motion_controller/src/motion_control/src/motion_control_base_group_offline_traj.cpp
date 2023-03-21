/*************************************************************************
	> File Name: motion_control_offline_trajectory.cpp
	> Author: 
	> Mail: 
	> Created Time: 2019年11月29日 星期五 15时07分13秒
 ************************************************************************/
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <vector>
#include <common_error_code.h>
#include "motion_control.h"
//#include  "controller_rpc.h"
#include "motion_control_base_group.h"
#include "log_manager_producer.h"

using namespace std;
using namespace basic_alg;
using namespace log_space;

namespace group_space
{

inline void file_to_string(vector<string> &record, const string& line, char delimiter);
inline double string_to_float(string str);
inline void file_to_string(vector<string> &record, const string& line, char delimiter)
{
    int linepos=0;
    char c;
    int linemax = line.length();
    string curstring;
    record.clear();
    while (linepos<linemax)
    {
        c = line[linepos];
        if (isdigit(c) || c=='.' || c == '-')
        {
            curstring+=c;
        }
        else if(c==delimiter && curstring.size())
        {
            record.push_back(curstring);
            curstring="";
        }
        ++linepos;
    }
    if(curstring.size())
        record.push_back(curstring);
    return;
}
inline double string_to_float(string str){
    int i=0,len=str.length();
    double sum = 0;
    bool pn_flag=1;
    if(str[0] == '-')
    {pn_flag = 0;}
    else
    {sum = sum*10 + str[0] - '0';}
    ++i;
    while(i<len)
    {
        if(str[i]=='.') break;
        sum = sum*10 + str[i] - '0';
        ++i;
    }
    ++i;
    double t=1,d=1;
    while (i<len)
    {
        d*=0.1;
        t=str[i]-'0';
        sum+=t*d;
        ++i;
    }
    if(pn_flag)
        return sum;
    else
        return -1.0*sum;
}

/**************************************************
* 函数功能: 读取位置姿态轨迹文件数据
* 参数:offline_euler_trajectory_filePath---文件名称
* 返回值:错误码
*******************************************************/
ErrorCode BaseGroup::readEulerTrajectoryFile(const std::string &offline_euler_trajectory_filePath, vector<vector<double> >&euler_trajArr)
{
    if (mc_state_ == OFFLINE || mc_state_ == OFFLINE_TO_STANDBY || mc_state_ == STANDBY_TO_OFFLINE)
    {
        LogProducer::error("mc_offline_traj","Fail to convert euler trajectory to joint trajectory, state = 0x%x", mc_state_);
        return INVALID_SEQUENCE;
    }
    if (offline_euler_trajectory_file_.is_open())
    {
        offline_euler_trajectory_file_.close();
    }
    cout << "[debug info]离线文件路径及名称: "<< offline_euler_trajectory_filePath << endl;
    offline_euler_trajectory_file_.open(offline_euler_trajectory_filePath.c_str());
    if (!offline_euler_trajectory_file_.is_open())
    {
        LogProducer::error("mc_offline_traj","Fail to open offline trajectory file");
        return INVALID_PARAMETER;
    }
    
    vector<double> data_line;
    vector<string> row;
    string line;
    int line_cnt=0;//数据行计数
    getline(offline_euler_trajectory_file_, line);//跳过第一行表头
    while (getline(offline_euler_trajectory_file_, line) && offline_euler_trajectory_file_.good())//逐行读取
    {
        file_to_string(row, line,',');//把line里的单元格数字字符提取出来,','为单元格分隔符
        line_cnt++;
		printf("[%3d] ",line_cnt);
        for(int i=0,leng=row.size();i<leng;i++)
        {
            data_line.push_back(string_to_float(row[i]));
            printf("%lf ",data_line[i]);
        }
        printf("\n");
        euler_trajArr.push_back(data_line);
        data_line.clear(); 
        /*
        if(line_cnt >= 10)
        {
            break;
        } 
        */     
    }
    offline_euler_trajectory_file_.close();
    return SUCCESS;
}

ErrorCode BaseGroup::setOfflineTrajectory(const std::string &offline_trajectory)
{
    if (mc_state_ == OFFLINE || mc_state_ == OFFLINE_TO_STANDBY || mc_state_ == STANDBY_TO_OFFLINE)
    {
        LogProducer::error("mc_offline_traj","Fail to set offline trajectory, state = 0x%x", mc_state_);
        return INVALID_SEQUENCE;
    }

    offline_traj_point_read_cnt_ = 0;
    offline_trajectory_size_ = (uint32_t)origin_trajectory_.size();

    for (uint32_t i = 0; i < 6; i++)
    {
        offline_start_joint_[i] = origin_trajectory_[0].angle[i];
    }
    offline_trajectory_time_stamp_ = cycle_time_;
    char buffer[LOG_TEXT_SIZE];
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
    offline_traj_point_read_cnt_ = 0;//log 取点行数
    return SUCCESS;
}

ErrorCode BaseGroup::setOfflineViaPoints(const vector<PoseEuler> &via_points, bool is_new)
{
    offline_planner_.setViaPoints(via_points, is_new);
    return 0;
}

ErrorCode BaseGroup::planOfflineTrajectory(string traj_name, double traj_vel)
{
    LogProducer::info("BaseGroup","start plan");

    if(!offline_planner_.viaPoints2Traj(traj_vel))
    {
        LogProducer::error("BaseGroup","transfer via points to trajectory failed");
        return MC_VP2TRAJ_PLAN_FAILED;
    }
    LogProducer::info("BaseGroup","finish plan");

    vector<PoseEuler> xyz_traj = offline_planner_.getResampledTraj();
    LogProducer::info("BaseGroup","trajectory size %ld", xyz_traj.size());
    origin_trajectory_.clear();
    origin_trajectory_.resize(xyz_traj.size());
    Posture posture = {1, 1, 1, 0};
    JointState joint_state;
    joint_state.alpha.zero();
    joint_state.angle.zero();
    joint_state.jerk.zero();
    joint_state.omega.zero();
    joint_state.torque.zero();
    JointState last_state;

    auto org_traj_iter = origin_trajectory_.begin();
    for(auto iter = xyz_traj.begin(); iter != xyz_traj.end(); ++iter)
    {
        if(!kinematics_ptr_->doIK(*iter, posture, joint_state.angle))
        {
            LogProducer::error("BaseGroup", "offline trajectory point IK failed");
            LogProducer::error("BaseGroup", "failed point(%lf, %lf, %lf, %lf, %lf, %lf) count %ld", 
            iter->point_.x_, iter->point_.y_, iter->point_.z_, 
            iter->euler_.a_, iter->euler_.b_, iter->euler_.c_, 
            iter - xyz_traj.begin());
            return MC_VP2TRAJ_PLAN_FAILED;
        }
        if(iter == xyz_traj.begin())
        {

        }
        else
        {
            // calc the angle speed
            joint_state.omega = joint_state.angle - last_state.angle;

        }
        *org_traj_iter++ = joint_state;
        last_state = joint_state;
    }

    return SUCCESS;
}

ErrorCode BaseGroup::planOfflinePause(void)
{
    pthread_mutex_lock(&offline_mutex_);
    uint32_t left_points = getOfflineCacheSize(); // local cache left points
    pthread_mutex_unlock(&offline_mutex_);
    // offline plan
    LogProducer::warn("BaseGroup", "start offline pause plan on index %u, cache left points %u", offline_traj_point_read_cnt_, left_points);
    if(!offline_planner_.trajPausePlan(offline_traj_point_read_cnt_, 0, 0, 0, 0))
    {
        LogProducer::error("BaseGroup", "offline pause plan failed");
        return MC_PAUSE_FAILED;
    }
    vector<PoseEuler> pause_traj = offline_planner_.getPauseTraj();

    Posture posture = {1, 1, 1, 0};
    JointState joint_state;
    JointState last_state = origin_trajectory_[offline_traj_point_read_cnt_];
    joint_state.alpha.zero();
    joint_state.angle.zero();
    joint_state.jerk.zero();
    joint_state.omega.zero();
    joint_state.torque.zero();
    pause_trajectory_.clear();
    pause_trajectory_time_stamp_ = offline_traj_point_read_cnt_ * cycle_time_;
    LogProducer::info("BaseGroup", "offline pause plan success last state( %lf,%lf,%lf,%lf,%lf,%lf )", 
    last_state.angle.j1_, last_state.angle.j2_, last_state.angle.j3_, 
    last_state.angle.j4_, last_state.angle.j5_, last_state.angle.j6_);

    for(auto iter = pause_traj.begin(); iter < pause_traj.end(); ++iter)
    {
        if(!kinematics_ptr_->doIK(*iter, posture, joint_state.angle))
        {
            LogProducer::error("BaseGroup", "offline trajectory pause point IK failed index %d", iter - pause_traj.begin());
            LogProducer::error("BaseGroup", "failed point(%lf, %lf, %lf, %lf, %lf, %lf)", 
            iter->point_.x_, iter->point_.y_, iter->point_.z_, iter->euler_.a_, iter->euler_.b_, iter->euler_.c_);
            // need error stop
            return MC_PAUSE_FAILED;
        }
        joint_state.omega = last_state.angle - joint_state.angle;
        pause_trajectory_.push_back(joint_state);
        last_state = joint_state;
    }
    pause_trajectory_.shrink_to_fit();
    LogProducer::info("BaseGroup", "offline pause traj filled");

    return SUCCESS;
}

ErrorCode BaseGroup::planOfflineResume(void)
{
    LogProducer::warn("BaseGroup", "start offline resume plan");
    if(!offline_planner_.trajResumePlan(0.0, 0.0, 0.0, 0.0))
    {
        LogProducer::error("BaseGroup", "offline resume plan failed");
        return MC_RESUME_FAILED;
    }
    vector<PoseEuler> resume_traj = offline_planner_.getResampledTraj();
    LogProducer::info("BaseGroup", "offline resume plan success");
    offline_trajectory_file_name_ += ".resume";
    std::ofstream traj_out_file(offline_trajectory_file_name_, ios::out);
    // write the traj file head
    traj_out_file << getNumberOfJoint() << " " << 0.001 << " " << resume_traj.size() << "\n";
    traj_out_file << fixed << setprecision(10);

    Posture posture = {1, 1, 1, 0};
    JointState joint_state;
    JointState last_state;
    joint_state.alpha.zero();
    joint_state.angle.zero();
    joint_state.jerk.zero();
    joint_state.omega.zero();
    joint_state.torque.zero();
    // resume_trajectory_.clear();
    origin_trajectory_.clear();
    for(auto iter = resume_traj.begin(); iter != resume_traj.end(); ++iter)
    {
        if(!kinematics_ptr_->doIK(*iter, posture, joint_state.angle))
        {
            LogProducer::error("BaseGroup", "offline resume trajectory point IK failed");
            LogProducer::error("BaseGroup", "failed point(%lf, %lf, %lf, %lf, %lf, %lf) count %ld", 
            iter->point_.x_, iter->point_.y_, iter->point_.z_, 
            iter->euler_.a_, iter->euler_.b_, iter->euler_.c_, 
            iter - resume_traj.begin());
            return MC_VP2TRAJ_PLAN_FAILED;
        }
        if(iter == resume_traj.begin())
        {
#if ONLINE_FILE
            // write the first joint angle to file
            traj_out_file
            << joint_state.angle.j1_ << " " << joint_state.angle.j2_ << " " 
            << joint_state.angle.j3_ << " " << joint_state.angle.j4_ << " " 
            << joint_state.angle.j5_ << " " << joint_state.angle.j6_ << "\n";
            // write the first joint state to file
            traj_out_file
            << joint_state.angle.j1_ << " " << joint_state.angle.j2_ << " " 
            << joint_state.angle.j3_ << " " << joint_state.angle.j4_ << " " 
            << joint_state.angle.j5_ << " " << joint_state.angle.j6_ << " "
            // first point is zero
            << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " "
            << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " "
            << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << "\n";
#endif
        }
        else
        {
            // calc the angle speed
            joint_state.omega = joint_state.angle - last_state.angle;
#if ONLINE_FILE
            // write the first joint state to file
            traj_out_file
            << joint_state.angle.j1_ << " " << joint_state.angle.j2_ << " " 
            << joint_state.angle.j3_ << " " << joint_state.angle.j4_ << " " 
            << joint_state.angle.j5_ << " " << joint_state.angle.j6_ << " "

            << joint_state.omega.j1_ * 1000 << " " << joint_state.omega.j2_ * 1000 << " " 
            << joint_state.omega.j3_ * 1000 << " " << joint_state.omega.j4_ * 1000 << " " 
            << joint_state.omega.j5_ * 1000 << " " << joint_state.omega.j6_ * 1000 << " "

            << joint_state.alpha.j1_ << " " << joint_state.alpha.j2_ << " " 
            << joint_state.alpha.j3_ << " " << joint_state.alpha.j4_ << " " 
            << joint_state.alpha.j5_ << " " << joint_state.alpha.j6_ << " "

            << joint_state.torque.j1_ << " " << joint_state.torque.j2_ << " " 
            << joint_state.torque.j3_ << " " << joint_state.torque.j4_ << " " 
            << joint_state.torque.j5_ << " " << joint_state.torque.j6_ << "\n";
#endif
        }
        // resume_trajectory_.push_back(joint_state);
        origin_trajectory_.push_back(joint_state);
        last_state = joint_state;
    }
#if ONLINE_FILE
    resume_trajectory_.shrink_to_fit();
#else
    offline_traj_point_read_cnt_ = 0;
    origin_trajectory_.shrink_to_fit();
    offline_trajectory_size_ = (uint32_t)origin_trajectory_.size();
#endif
    LogProducer::info("BaseGroup", "offline resume traj file filled");

    traj_out_file.close();

    return SUCCESS;
}

uint32_t BaseGroup::getOfflineCacheSize(void)
{
    if (offline_trajectory_cache_tail_ >= offline_trajectory_cache_head_)
    {
        return offline_trajectory_cache_tail_ - offline_trajectory_cache_head_;
    }
    else
    {
        return offline_trajectory_cache_tail_ + OFFLINE_TRAJECTORY_CACHE_SIZE - offline_trajectory_cache_head_;
    }
}

bool BaseGroup::fillOfflineCache(void)
{
    const static uint32_t local_cache_size = 50;
    static TrajectoryPoint local_cache[local_cache_size];

    pthread_mutex_lock(&offline_mutex_);

    if (getOfflineCacheSize() + local_cache_size >= OFFLINE_TRAJECTORY_CACHE_SIZE)
    {
        pthread_mutex_unlock(&offline_mutex_);
        return false;
    }

    pthread_mutex_unlock(&offline_mutex_);

    uint32_t pick_cnt = 0;
    for(pick_cnt = 0; pick_cnt < local_cache_size; ++pick_cnt)
    {
        if(offline_traj_point_read_cnt_ < offline_trajectory_size_)
        {
            local_cache[pick_cnt].state = origin_trajectory_[offline_traj_point_read_cnt_++];
        }
        else
        {
            offline_trajectory_last_point_ = true;
            LogProducer::info("fillOfflineCache","set last point status to true");
            break;
        }
        
        local_cache[pick_cnt].time_stamp = offline_trajectory_time_stamp_;
        local_cache[pick_cnt].level = POINT_MIDDLE;
        offline_trajectory_time_stamp_ += cycle_time_;
    }

    if (pick_cnt == 0) return false;

    if (offline_trajectory_first_point_)
    {
        local_cache[0].level = POINT_START;
        offline_trajectory_first_point_ = false;
    }

    if (offline_trajectory_last_point_)
    {
        LogProducer::info("fillOfflineCache","point ending");
        local_cache[pick_cnt - 1].level = POINT_ENDING;
        start_joint_ = local_cache[pick_cnt - 1].state.angle;
        offline_trajectory_last_point_ = false;
        LogProducer::info("fillOfflineCache","set last point status to false");
    }
    
    pthread_mutex_lock(&offline_mutex_);

    for (uint32_t i = 0; i < pick_cnt; i++)
    {
        offline_trajectory_cache_[offline_trajectory_cache_tail_] = local_cache[i];
        offline_trajectory_cache_tail_ = offline_trajectory_cache_tail_ + 1 < OFFLINE_TRAJECTORY_CACHE_SIZE ? offline_trajectory_cache_tail_ + 1 : 0;
    }

    pthread_mutex_unlock(&offline_mutex_);
    return true;
}

bool BaseGroup::fillOfflinePauseCache(void)
{
    const static uint32_t local_cache_size = 50;
    static uint32_t picked_number = 0;
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
        if (picked_number < pause_trajectory_.size())
        {
            local_cache[picked_number].state = pause_trajectory_[picked_number];
        }
        else
        {
            offline_trajectory_last_point_ = true;
            LogProducer::info("fillOfflinePauseCache", "set last point status to true");
            break;
        }
        local_cache[picked_number].time_stamp = pause_trajectory_time_stamp_;
        local_cache[picked_number].level = POINT_MIDDLE;
        pause_trajectory_time_stamp_ += cycle_time_;
    }

    if (picked_number == 0)
    {
        return false;
    }
    if (picked_number > 0)
    {
        pause_trajectory_.erase(pause_trajectory_.begin(), pause_trajectory_.begin() + picked_number);
    }

    if(offline_trajectory_size_ == offline_traj_point_read_cnt_)
    {
        offline_trajectory_last_point_ = true;//取到最后一点,标记结束点
        LogProducer::info("fillOfflinePauseCache", "set last_point status to true");
    }
        
    if (offline_trajectory_last_point_)
    {
        LogProducer::info("fillOfflinePauseCache","pause point ending");
        local_cache[picked_number - 1].level = POINT_ENDING;
        start_joint_ = local_cache[picked_number - 1].state.angle;
        offline_trajectory_last_point_ = false;
        LogProducer::info("fillOfflinePauseCache", "set last_point status to false");
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
    if(length < 1)
    {
        err = BASE_GROUP_PICK_POINTS_FROM_OFFLINECACHE_NULL;
    }
    return err;
}

ErrorCode BaseGroup::sendOfflineTrajectoryFlow(void)
{
    if (bare_core_.isPointCacheEmpty())
    {
        size_t length = JC_POINT_NUM;
        TrajectoryPoint points[JC_POINT_NUM];
        ErrorCode err = pickPointsFromOfflineCache(points, length);//length < 1时返回0x1111
        if (err != SUCCESS)
        {
            LogProducer::error("mc_offline_traj","sendOfflineTrajectoryFlow: cannot pick point from trajectory fifo.");
            return MC_SEND_TRAJECTORY_FAIL;
        }
        err = bare_core_.fillPointCache(points, length, POINT_POS_VEL);
        
        if (points[length - 1].level == POINT_ENDING)
        {
            // 取到了ENDING-POINT，意味着轨迹FIFO已经取完,必须要切换状态机
            if (mc_state_ == OFFLINE)
            {
                offline_to_standby_request_ = true;
            }
            else if(mc_state_ == PAUSING_OFFLINE && !offline_ready_to_pause_request_)
            {
                pausing_offline_to_pause_request_ = true;
            }
        }
    }

    return bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
}


}

