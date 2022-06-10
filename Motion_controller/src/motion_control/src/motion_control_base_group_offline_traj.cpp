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
    offline_traj_point_readCnt = 0;//log 取点行数
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

    Posture posture = {1, 1, 1, 0};
    JointState joint_state;
    joint_state.alpha.zero();
    joint_state.angle.zero();
    joint_state.jerk.zero();
    joint_state.omega.zero();
    joint_state.torque.zero();
    JointState last_state;
    double traj_time = 0.0;
    std::ofstream traj_out_file(traj_name, ios::out);
    // write the traj file head
    traj_out_file << getNumberOfJoint() << " " << 0.001 << " " << xyz_traj.size() << "\n";
    traj_out_file << fixed << setprecision(10);

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
        }
        else
        {
            // calc the angle speed
            joint_state.omega = joint_state.angle - last_state.angle;
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
        }
        origin_trajectory_.push_back(joint_state);
        last_state = joint_state;
    }
    traj_out_file.close();

    return SUCCESS;
}

ErrorCode BaseGroup::planOfflinePause(void)
{
    pthread_mutex_lock(&offline_mutex_);
    uint32_t left_points = getOfflineCacheSize(); // local cache left points
    pthread_mutex_unlock(&offline_mutex_);
    // offline plan
    offline_planner_.trajPausePlan(offline_traj_point_readCnt, 0, 0, 0, 0);
    vector<PoseEuler> pause_traj = offline_planner_.getPauseTraj();

    Posture posture = {1, 1, 1, 0};
    JointState joint_state;
    JointState last_state = origin_trajectory_[offline_traj_point_readCnt];
    joint_state.alpha.zero();
    joint_state.angle.zero();
    joint_state.jerk.zero();
    joint_state.omega.zero();
    joint_state.torque.zero();
    pause_trajectory_.clear();

    for(auto iter = pause_traj.begin(); iter < pause_traj.end(); ++iter)
    {
        if(!kinematics_ptr_->doIK(*iter, posture, joint_state.angle))
        {
            LogProducer::error("BaseGroup", "offline trajectory pause point IK failed");
            LogProducer::error("BaseGroup", "failed point(%lf, %lf, %lf, %lf, %lf, %lf)", 
            iter->point_.x_, iter->point_.y_, iter->point_.z_, iter->euler_.a_, iter->euler_.b_, iter->euler_.c_);
            // need error stop
            return -1;
        }
        joint_state.omega = last_state.angle - joint_state.angle;
        pause_trajectory_.push_back(joint_state);
        last_state = joint_state;
    }

    return 0;
}

ErrorCode BaseGroup::planOfflineResume(void)
{
    return 0;
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
            offline_traj_point_readCnt++;
            //LogProducer::info("mc_offline_traj","read point [%d]-(%f,%f,%f,%f,%f,%f),", offline_traj_point_readCnt,local_cache[picked_number].state.angle[0],local_cache[picked_number].state.angle[1],local_cache[picked_number].state.angle[2],local_cache[picked_number].state.angle[3],local_cache[picked_number].state.angle[4],local_cache[picked_number].state.angle[5]);
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
    if(offline_trajectory_size_ == offline_traj_point_readCnt) offline_trajectory_last_point_ = true;//取到最后一点,标记结束点
    //LogProducer::info("fillOfflineCache","picked: %d, last = %d", picked_number, offline_trajectory_last_point_);
    if (offline_trajectory_first_point_)
    {
        local_cache[0].level = POINT_START;
        offline_trajectory_first_point_ = false;
    }

    if (offline_trajectory_last_point_)
    {
        LogProducer::info("fillOfflineCache","point ending");
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

bool BaseGroup::fillOfflinePauseCache(void)
{
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
        size_t length = 10;
        TrajectoryPoint points[10];
        ErrorCode err = pickPointsFromOfflineCache(points, length);//length < 1时返回0x1111
        if (err != SUCCESS)
        {
            LogProducer::error("mc_offline_traj","sendOfflineTrajectoryFlow: cannot pick point from trajectory fifo.");
            return err;
        }
        err = bare_core_.fillPointCache(points, length, POINT_POS_VEL);
        
        if(err == true)//debug infomation
        {
            for (size_t i = 0; i < length; i++)
            {
                LogProducer::info("OfflinetrjS|barecore_fillPointCache"," %d) level=%d time_stamp=%.4f (%.6f,%.6f,%.6f,%.6f,%.6f,%.6f)",
                i+1,points[i].level, points[i].time_stamp, 
                points[i].state.angle.j1_, points[i].state.angle.j2_, points[i].state.angle.j3_,
                points[i].state.angle.j4_,points[i].state.angle.j5_,points[i].state.angle.j6_);
            }
        }
        //LogProducer::info("mc_offline_traj","sendOfflineTrajectoryFlow: %d, head=%d, tail=%d", length, offline_trajectory_cache_head_, offline_trajectory_cache_tail_);
        
        if (points[length - 1].level == POINT_ENDING)
        {
            // 取到了ENDING-POINT，意味着轨迹FIFO已经取完,必须要切换状态机
            if (mc_state_ == OFFLINE)
            {
                offline_to_standby_request_ = true;
            }
            else if(mc_state_ == PAUSE_OFFLINE)
            {
                offline_to_pause_request_ = true;
            }
        }
    }
    else
    {
        LogProducer::warn("sendOfflineTrajectoryFlow","bare_core_ is not PointCacheEmpty");
    }
    //LogProducer::info("mc_offline_traj","sendOfflineTrajectoryFlow: bare.sendPoint");
    return bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
}


}

