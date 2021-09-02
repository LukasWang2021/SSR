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

static void dumpShareMemory(void)
{
    ofstream  shm_out("/root/share_memory.dump");
    int fd = open("/dev/mem", O_RDWR);
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


namespace group_space
{

void BaseGroup::doCommonLoop(void)
{
    updateJointRecorder();
    doStateMachine();
}

void BaseGroup::doPriorityLoop(void)
{
    updateServoStateAndJoint();
    //checkEncoderState();
    fillTrajectoryFifo();
}

void BaseGroup::doRealtimeLoop(void)
{
    sendTrajectoryFlow();
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
        LogProducer::error("mc_base","Record timeout, cannot save joint into NvRam.");
        reportError(MC_RECORD_JOINT_TIMEOUT);
    }
}

void BaseGroup::updateServoStateAndJoint(void)
{
    static ServoState last_servo_state = SERVO_INIT;
    static ServoState barecore_state = SERVO_INIT;
    static Joint barecore_joint = {0};
    static uint32_t encoder_state[NUM_OF_JOINT] = {0};

    if (bare_core_.getLatestJoint(barecore_joint, encoder_state, barecore_state))
    {
        pthread_mutex_lock(&servo_mutex_);
        servo_state_ = barecore_state;
        servo_joint_ = barecore_joint;
        memcpy(encoder_state_, encoder_state, sizeof(encoder_state_));
        pthread_mutex_unlock(&servo_mutex_);

        if (last_servo_state != servo_state_)
        {
            // LogProducer::info("mc_base","Servo-state switch %s to %s", getMCServoStatusString(last_servo_state).c_str(),
            //     getMCServoStatusString(servo_state_).c_str());

            if ((last_servo_state == SERVO_RUNNING) && (servo_state_ != SERVO_IDLE))
            {
                LogProducer::error("mc_base","MC-state: %s, point-cache-empty: %d, auto_to_standby_request: %d, auto_to_pause_request: %d", 
                getMontionControlStatusString(mc_state_).c_str(), bare_core_.isPointCacheEmpty(), auto_to_standby_request_, auto_to_pause_request_);
                LogProducer::info("mc_base","Dump share memory ...");
                dumpShareMemory();
                LogProducer::info("mc_base","Done.");
            }

            if ((last_servo_state != SERVO_IDLE && last_servo_state != SERVO_RUNNING) && (servo_state_ == SERVO_IDLE))
            {
                updateStartJoint();
            }

            last_servo_state = servo_state_;
        }
    }
}

bool BaseGroup::updateStartJoint(void)
{
    start_joint_ = getLatestJoint();

    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_base","Update Start-position: %s", printDBLine(&start_joint_[0], buffer, LOG_TEXT_SIZE));
    return true;
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

void BaseGroup::fillTrajectoryFifo(void)
{
    static Joint reference = getLatestJoint();
    static uint32_t waiting_smooth_cnt = 0;
    uint32_t fill_point_num = 0;
    ErrorCode err = SUCCESS;
    filling_points_into_traj_fifo_ = true;

    if ((mc_state_ == AUTO && !auto_to_standby_request_) || mc_state_ == STANDBY_TO_AUTO || mc_state_ == PREPARE_RESUME)
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
                err = pick_traj_ptr_->smooth.sampleTrajectory(auto_time_, reference, point.state);

                if (err != SUCCESS)
                {
                    LogProducer::error("mc_base","Fail to sample point on trajectory, code = 0x%llx", err);
                    reportError(err);
                    break;
                }

                if (!soft_constraint_.isJointInConstraint(point.state.angle, MINIMUM_E3))
                {
                    char buffer[LOG_TEXT_SIZE];
                    LogProducer::error("mc_base","Trajectory point out of soft constraint:");
                    LogProducer::error("mc_base","Time: %.6f, joint: %s", auto_time_, printDBLine(&point.state.angle[0], buffer, LOG_TEXT_SIZE));
                    reportError(JOINT_OUT_OF_CONSTRAINT);
                    break;
                }

                point.time_stamp = auto_time_;
                point.level = POINT_MIDDLE;
                traj_fifo_.push(point);
                reference = point.state.angle;
                fill_point_num ++;

                if (auto_time_ + cycle_time_ >= pick_traj_ptr_->smooth.getDuration())
                {
                    // 平滑段已取完,不足1个cycle_time的剩余时间累加计入接下来的轨迹段
                    LogProducer::warn("mc_base","Trajectory start with smooth, smooth picked out, switch to trajectory.");
                    pick_traj_ptr_->start_from_smooth = false;
                    auto_time_ = pick_traj_ptr_->smooth_in_time + (auto_time_ + cycle_time_ - pick_traj_ptr_->smooth.getDuration());
                }
                else
                {
                    auto_time_ += cycle_time_ * num_of_point;
                }

                // 直接进入下一个循环周期
                continue;
                
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
                    waiting_smooth_cnt = 0;
                    LogProducer::warn("mc_base","Trajectory end without smooth, switch to next trajectory.");
                    continue;
                }
                else if (pick_traj_ptr_->end_with_smooth && auto_time_ > pick_traj_ptr_->smooth_time + MINIMUM_E6)
                {
                    // 有平滑,且平滑切出点之前的轨迹已经取完,且平滑切出点也已经取出
                    if (pick_traj_ptr_->next->valid)
                    {
                        // 下一条语句已经就绪,切换到下一条语句;
                        // 直接开始下个循环
                        auto_time_ = auto_time_ - pick_traj_ptr_->smooth_time;
                        LogProducer::warn("mc_base","Trajectory end with smooth, next trajectory available, switch to next trajectory. auto_time_ = %lf", auto_time_);
                        pick_traj_ptr_->valid = false;
                        pick_traj_ptr_ = pick_traj_ptr_->next;
                        waiting_smooth_cnt = 0;
                        continue;
                    }
                    else
                    {
                        waiting_smooth_cnt ++;

                        // 下一条语句尚未就绪
                        if (traj_fifo_.size() < traj_fifo_lower_limit_)
                        {
                            // 轨迹FIFO中轨迹不足, 放弃平滑, 改为CNT0或FINE
                            LogProducer::warn("mc_base","Trajectory end with smooth, next trajectory not available when fifo-size = %d, switch to none smooth trajectory.", traj_fifo_.size());
			                fine_enable_ = true;
                            pick_traj_ptr_->end_with_smooth = false;
                            pick_traj_ptr_->smooth_time = -1;
                            pick_traj_ptr_->smooth_distance = -1;
                            continue;
                        }
                        else if (mc_state_ == PREPARE_RESUME && waiting_smooth_cnt > 150)
                        {
                            // 在PREPARE_RESUME阶段FIFO只进不出，如果卡在圆滑切出点之前且点数大于限制值时上述条件恒不能达成，可能造成暂停恢复规划时点数不足的问题，
                            // 此处需要计时，当超时后下一条语句仍未介入则放弃平滑，保证恢复规划时点数足够
                            // 超时时间300ms, 放弃平滑, 改为CNT0或FINE
                            LogProducer::warn("mc_base","Trajectory end with smooth, next trajectory not available when fifo-size = %d in prepare resume, switch to none smooth trajectory.", traj_fifo_.size());
                            fine_enable_ = true;
                            pick_traj_ptr_->end_with_smooth = false;
                            pick_traj_ptr_->smooth_time = -1;
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
                else
                {
                    // 本条轨迹尚未取完
                }

                // 取点位置位于轨迹上,能够正常取点
                err = pick_traj_ptr_->trajectory.sampleTrajectory(auto_time_, reference, num_of_point, &point.state);

                if (err != SUCCESS)
                {
                    LogProducer::error("mc_base","Fail to sample point on trajectory, code = 0x%llx", err);
                    reportError(err);
                    break;
                }

                if (num_of_point > 0)
                {
                    if (!soft_constraint_.isJointInConstraint(point.state.angle, MINIMUM_E3))
                    {
                        char buffer[LOG_TEXT_SIZE];
                        LogProducer::error("mc_base","Trajectory point out of soft constraint:");
                        LogProducer::error("mc_base","Time: %.6f, joint: %s", auto_time_, printDBLine(&point.state.angle[0], buffer, LOG_TEXT_SIZE));
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

                    point.time_stamp = auto_time_;
                    traj_fifo_.push(point);
                    reference = point.state.angle;
                    auto_time_ += cycle_time_ * num_of_point;
                    fill_point_num ++;
                }
            }
        }

        pthread_mutex_unlock(&planner_list_mutex_);
    }
    else if (mc_state_ == AUTO_TO_PAUSING)
    {
        ErrorCode err = planPauseTrajectory();

        if (err != SUCCESS)
        {
            mc_state_ = AUTO;
            LogProducer::info("mc_base","MC-state switch to MC_AUTO.");
            reportError(err);
        }
        else
        {
            mc_state_ = PAUSING;
            LogProducer::info("mc_base","MC-state switch to MC_PAUSING.");
        }
    }
    else if (mc_state_ == PAUSING)
    {
        uint32_t num = 0;
        uint32_t pause_trajectory_size = pause_trajectory_.size();
        TrajectoryPoint traj_point;
        traj_point.level = POINT_MIDDLE;

        while (!traj_fifo_.full() && num < pause_trajectory_size)
        {
            traj_point.time_stamp = pause_trajectory_time_stamp_;
            pause_trajectory_time_stamp_ += cycle_time_;
            traj_point.state = pause_trajectory_[num];
            dynamics_ptr_->getTorqueInverseDynamics(traj_point.state.angle, *(JointVelocity*)(&traj_point.state.omega), *(JointAcceleration*)(&traj_point.state.alpha), *(JointTorque*)(&traj_point.state.torque));
            traj_fifo_.push(traj_point);
            num++;
        }

        if (num > 0)
        {
            pause_trajectory_.erase(pause_trajectory_.begin(), pause_trajectory_.begin() + num);
        }
    }
    else if (mc_state_ == PAUSE_RETURN)
    {
        uint32_t num = 0;
        uint32_t return_trajectory_size = resume_trajectory_.size();
        TrajectoryPoint traj_point;
        traj_point.level = POINT_MIDDLE;

        while (!traj_fifo_.full() && num < return_trajectory_size)
        {
            traj_point.time_stamp = resume_trajectory_time_stamp_;
            resume_trajectory_time_stamp_ += cycle_time_;
            traj_point.state = resume_trajectory_[num];
            dynamics_ptr_->getTorqueInverseDynamics(traj_point.state.angle, *(JointVelocity*)(&traj_point.state.omega), *(JointAcceleration*)(&traj_point.state.alpha), *(JointTorque*)(&traj_point.state.torque));
            traj_fifo_.push(traj_point);
            num++;
        }

        if (num > 0)
        {
            resume_trajectory_.erase(resume_trajectory_.begin(), resume_trajectory_.begin() + num);
        }
    }
    else if (mc_state_ == RESUME)
    {
        uint32_t num = 0;
        uint32_t resume_trajectory_size = resume_trajectory_.size();
        TrajectoryPoint traj_point;
        traj_point.level = POINT_MIDDLE;

        while (!traj_fifo_.full() && num < resume_trajectory_size)
        {
            traj_point.time_stamp = resume_trajectory_time_stamp_;
            resume_trajectory_time_stamp_ += cycle_time_;
            traj_point.state = resume_trajectory_[num];
            dynamics_ptr_->getTorqueInverseDynamics(traj_point.state.angle, *(JointVelocity*)(&traj_point.state.omega), *(JointAcceleration*)(&traj_point.state.alpha), *(JointTorque*)(&traj_point.state.torque));
            traj_fifo_.push(traj_point);
            num++;
        }

        if (num > 0)
        {
            resume_trajectory_.erase(resume_trajectory_.begin(), resume_trajectory_.begin() + num);
        }
    }
    else if (mc_state_ == MANUAL || mc_state_ == PAUSE_MANUAL)
    {
        err = fillManualFIFO();

        if (err != SUCCESS)
        {
            reportError(err);
        }
    }

    filling_points_into_traj_fifo_ = false;
}

ErrorCode BaseGroup::fillManualFIFO(void)
{
    ErrorCode err = SUCCESS;
    TrajectoryPoint point;
    pthread_mutex_lock(&manual_traj_mutex_);

    while (!manual_fifo_.full() && manual_time_ <= manual_teach_.getDuration())
    {
        err = pickManualPoint(point);

        if (err != SUCCESS)
        {
            pthread_mutex_unlock(&manual_traj_mutex_);
            return err;
        }

        manual_fifo_.push(point);
    }

    pthread_mutex_unlock(&manual_traj_mutex_);
    return SUCCESS;
}

ErrorCode BaseGroup::pickManualPoint(TrajectoryPoint &point)
{
    //做标记，如果是轨迹起始
    if (start_of_motion_)
    {
        start_of_motion_ = false;
        manaul_reference_ = start_joint_;
        point.level = POINT_START;
    }
    else
    {
        point.level = POINT_MIDDLE;
    }

    ErrorCode err = manual_teach_.sampleTrajectory(manual_time_, manaul_reference_, point.state);
    //memset(static_cast<void*>(&point.state.torque), 0, sizeof(point.state.torque));

    if (err != SUCCESS)
    {
        return err;
    }

    if (!manual_trajectory_check_fail_ && manual_teach_.getManualMode() == CONTINUOUS && manual_teach_.getManualFrame() != JOINT)
    {
        JointState forestate;
        double foresight = (manual_time_ + 0.5) < manual_teach_.getDuration() ? (manual_time_ + 0.5) : manual_teach_.getDuration();
        err = manual_teach_.sampleTrajectory(foresight, point.state.angle, forestate);

        if (err != SUCCESS)
        {
            LogProducer::error("mc_base","Manual trajectory check fail, stop manual");
            manual_trajectory_check_fail_ = true;
        }
        else if (!soft_constraint_.isJointInConstraint(forestate.angle, MINIMUM_E3))
        {
            char buffer[LOG_TEXT_SIZE];
            LogProducer::error("mc_base","Manual trajectory near soft constraint, stop manual");
            LogProducer::error("mc_base","Time: %.6f, joint: %s", manual_time_, printDBLine(&forestate.angle.j1_, buffer, LOG_TEXT_SIZE));
            err = JOINT_OUT_OF_CONSTRAINT;
            manual_trajectory_check_fail_ = true;
        }
        else if(kinematics_ptr_->nearSingularPosition(forestate.angle))
        {
            char buffer[LOG_TEXT_SIZE];
            LogProducer::error("mc_base","Manual trajectory near singular position, stop manual");
            LogProducer::error("mc_base","Time: %.6f, joint: %s", manual_time_, printDBLine(&forestate.angle.j1_, buffer, LOG_TEXT_SIZE));
            err = MC_MANUAL_TO_SINGULAR_POSITION;
            manual_trajectory_check_fail_ = true;
        }
        else
        {}

        if (err != SUCCESS)
        {
            manualStop();
            reportError(err);
        }
    }
    //更新下一个参考点，更新时间戳
    manaul_reference_ = point.state.angle;
    point.time_stamp = manual_time_;
    manual_time_ += cycle_time_;

    if (manual_time_ > manual_teach_.getDuration())
    {
        point.level = POINT_ENDING;
    }

    //char buffer[LOG_TEXT_SIZE];
    //LogProducer::info("mc_base",">> manual joint: %s", printDBLine(&point.state.angle[0], buffer, LOG_TEXT_SIZE));
    //LogProducer::info("mc_base",">> manual omega: %s", printDBLine(&point.state.omega[0], buffer, LOG_TEXT_SIZE));

    if (traj_log_enable_)
    {
        traj_log_data_ptr_[traj_log_ctrl_ptr_->write_index] = point;
        traj_log_ctrl_ptr_->write_index = (traj_log_ctrl_ptr_->write_index + 1 < traj_log_ctrl_ptr_->max_of_points) ? (traj_log_ctrl_ptr_->write_index + 1) : 0;
        traj_log_ctrl_ptr_->num_of_points = (traj_log_ctrl_ptr_->num_of_points < traj_log_ctrl_ptr_->max_of_points) ? (traj_log_ctrl_ptr_->num_of_points + 1) : traj_log_ctrl_ptr_->max_of_points;
    }

    return SUCCESS;
}


void BaseGroup::sendTrajectoryFlow(void)
{
    static size_t error_cnt = 0;
    ErrorCode err = SUCCESS;
    ServoState servo_state = getServoState();
    MotionControlState mc_state = mc_state_;

    if (servo_state != SERVO_IDLE && servo_state != SERVO_RUNNING)
    {
        return;
    }

    if (mc_state == AUTO && !auto_to_standby_request_)
    {
        err = sendAutoTrajectoryFlow();
    }
    else if ((mc_state == AUTO && auto_to_standby_request_) || mc_state == AUTO_TO_STANDBY)
    {
        if (!bare_core_.isPointCacheEmpty())
        {
            err = bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
        }
    }

    else if (mc_state == OFFLINE && !offline_to_standby_request_)
    {
        err = sendOfflineTrajectoryFlow();
    }
    else if ((mc_state == OFFLINE && offline_to_standby_request_) || mc_state == OFFLINE_TO_STANDBY)
    {
        if (!bare_core_.isPointCacheEmpty())
        {
            err = bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
        }
    }

    else if (mc_state == PAUSING && !pausing_to_pause_request_)
    {
        err = sendAutoTrajectoryFlow();
    }
    else if ((mc_state == PAUSING && pausing_to_pause_request_) || mc_state == AUTO_TO_PAUSING || mc_state == PAUSING_TO_PAUSE)
    {
        if (!bare_core_.isPointCacheEmpty())
        {
            err = bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
        }
    }

    else if (mc_state == PAUSE_RETURN && !pause_return_to_pause_request_)
    {
        err = sendAutoTrajectoryFlow();
    }
    else if ((mc_state == PAUSE_RETURN && pause_return_to_pause_request_) || mc_state == PAUSE_RETURN_TO_PAUSE)
    {
        if (!bare_core_.isPointCacheEmpty())
        {
            err = bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
        }
    }

    else if (mc_state == RESUME)
    {
        err = sendAutoTrajectoryFlow();
    }

    else if (mc_state == MANUAL && !manual_to_standby_request_)
    {
        err = sendManualTrajectoryFlow();
    }
    else if ((mc_state == MANUAL && manual_to_standby_request_) || mc_state == MANUAL_TO_STANDBY)
    {
        if (!bare_core_.isPointCacheEmpty())
        {
            err = bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
        }
    }

    else if (mc_state == PAUSE_MANUAL && !manual_to_pause_request_)
    {
        err = sendManualTrajectoryFlow();
    }
    else if ((mc_state == PAUSE_MANUAL && manual_to_pause_request_) || mc_state == PAUSE_MANUAL_TO_PAUSE)
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
                reportError(MC_SEND_TRAJECTORY_FAIL);
                LogProducer::error("mc_base","sendTrajectoryFlow: bare core time-out, servo state: %s.", getMCServoStatusString(servo_state).c_str());
            }
        }
        else
        {
            LogProducer::error("mc_base","sendTrajectoryFlow aborted, code = 0x%llx", err);
            reportError(err);
            error_cnt = 0;
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
            LogProducer::error("mc_base","sendAutoTrajectoryFlow: cannot pick point from trajectory fifo.");
            return err;
        }

        bool res = bare_core_.fillPointCache(points, length, POINT_POS_VEL);

        if (points[length - 1].level == POINT_ENDING)
        {
            // 取到了ENDING-POINT，意味着轨迹FIFO已经取完,必须要切换状态机
            char buffer[LOG_TEXT_SIZE];
            LogProducer::info("mc_base","Get ending-point: %s", printDBLine(&points[length - 1].state.angle[0], buffer, LOG_TEXT_SIZE));
            LogProducer::info("mc_base","Length of this package: %d, fill result: %d", length, res);

            if (mc_state_ == AUTO)
            {
                PoseQuaternion fcp_in_base;
                kinematics_ptr_->doFK(points[length - 1].state.angle, fcp_in_base);
                transformation_.convertFcpToTcp(fcp_in_base, tool_frame_, fine_pose_);
                auto_to_standby_request_ = true;
            }
            else if (mc_state_ == PAUSING)
            {
                pausing_to_pause_request_ = true;
            }
            else if (mc_state_ == PAUSE_RETURN)
            {
                pause_return_to_pause_request_ = true;
            }
        }
    }

    return bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
}

ErrorCode BaseGroup::pickPointsFromTrajectoryFifo(TrajectoryPoint *points, size_t &length)
{
    ErrorCode err = SUCCESS;
    uint32_t picked_num = 0;
    uint32_t wait_cycle = 3;
    uint32_t i;

    while ((traj_fifo_.size() < length + 1) && filling_points_into_traj_fifo_ && (wait_cycle > 0))
    {
        // 如果由于各种问题导致的，FIFO中点数不足，且正在填充FIFO，这种情况下等待10ms
        LogProducer::warn("mc_base","Not enough %d points in trajectory FIFO, but fifo fillter is filling points, wait 10 ms", traj_fifo_.size());
        usleep(10 * 1000);
        wait_cycle --;
        LogProducer::warn("mc_base","Now we have %d points in trajectory FIFO", traj_fifo_.size());
    }

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

    if (traj_log_enable_)
    {
        for (i = 0; i < picked_num; i++)
        {
            traj_log_data_ptr_[traj_log_ctrl_ptr_->write_index] = points[i];
            traj_log_ctrl_ptr_->write_index = (traj_log_ctrl_ptr_->write_index + 1 < traj_log_ctrl_ptr_->max_of_points) ? (traj_log_ctrl_ptr_->write_index + 1) : 0;
            traj_log_ctrl_ptr_->num_of_points = (traj_log_ctrl_ptr_->num_of_points < traj_log_ctrl_ptr_->max_of_points) ? (traj_log_ctrl_ptr_->num_of_points + 1) : traj_log_ctrl_ptr_->max_of_points;
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
            LogProducer::error("mc_base","sendPoint: cannot pick point from manual motion.");
            return err;
        }

        bare_core_.fillPointCache(points, length, POINT_POS);
    }

    return bare_core_.sendPoint() ? SUCCESS : MC_SEND_TRAJECTORY_FAIL;
}

ErrorCode BaseGroup::pickPointsFromManualTrajectory(TrajectoryPoint *points, size_t &length)
{
    size_t picked = 0;
    ErrorCode err = SUCCESS;
    MotionControlState mc_state = mc_state_;

    for (size_t i = 0; i < length; i++)
    {
        if (!manual_fifo_.empty())
        {
            manual_fifo_.fetch(points[i]);
            //LogProducer::info("mc_base","%d-%.4f-%.6f,%.6f,%.6f", points[i].level, points[i].time_stamp, points[i].state.angle.j1_, points[i].state.angle.j2_, points[i].state.angle.j3_);
            picked ++;
        }
        else
        {
            break;
        }
    }

    length = picked;

    if (err == SUCCESS)
    {
        if (points[length - 1].level == POINT_ENDING)
        {
            char buffer[LOG_TEXT_SIZE];
            if (mc_state == MANUAL) manual_to_standby_request_ = true;
            else if (mc_state == PAUSE_MANUAL) manual_to_pause_request_ = true;
            LogProducer::info("mc_base","Get ending-point: %.4f - %s", manual_time_, printDBLine(&points[length - 1].state.angle[0], buffer, LOG_TEXT_SIZE));
            start_joint_ = points[length - 1].state.angle;
        }

        return SUCCESS;
    }
    else
    {
        LogProducer::error("mc_base","Fail to pick points from manual trajectory, code = 0x%llx.", err);
        return err;
    }
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


} // namespace fst_mc
