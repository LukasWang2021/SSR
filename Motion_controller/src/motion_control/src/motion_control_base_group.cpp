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
}

BaseGroup::~BaseGroup()
{
    if (dynamics_ptr_ != NULL) {delete dynamics_ptr_; dynamics_ptr_ = NULL;}
    if (kinematics_ptr_ != NULL) {delete kinematics_ptr_; kinematics_ptr_ = NULL;}
}

bool BaseGroup::initTrajectoryLogSpace(void)
{
    int fd = shm_open("rtm_trajectory", O_CREAT|O_RDWR, 00777);
    
    if (-1 == fd)
    {
        LogProducer::error("mc_base","Fail to create rtm_trajectory");
        return false;
    }

    int lock = flock(fd, LOCK_EX | LOCK_NB);
    
    if (lock == -1)
    {
        LogProducer::error("mc_base","Fail to take over rtm_trajectory");
        return false;
    }

    ftruncate(fd, TRAJECTORY_LOG_CONTROL_SIZE + TRAJECTORY_LOG_DATA_SIZE);
    void *ptr = mmap(NULL, TRAJECTORY_LOG_CONTROL_SIZE + TRAJECTORY_LOG_DATA_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    
    if (ptr == MAP_FAILED)
    {
        close(fd);
        LogProducer::error("mc_base","Fail to map rtm_trajectory");
        return false;
    }

    memset(ptr, 0, TRAJECTORY_LOG_CONTROL_SIZE + TRAJECTORY_LOG_DATA_SIZE);
    traj_log_ctrl_ptr_ = static_cast<TrajectoryLogControl*>(ptr);
    getModelName(traj_log_ctrl_ptr_->model_name, sizeof(traj_log_ctrl_ptr_->model_name));
    traj_log_ctrl_ptr_->max_of_points = TRAJECTORY_LOG_DATA_SIZE / sizeof(TrajectoryPoint);
    traj_log_ctrl_ptr_->num_of_points = 0;
    traj_log_ctrl_ptr_->write_index = 0;
    traj_log_data_ptr_ = reinterpret_cast<TrajectoryPoint*>((char*)ptr + TRAJECTORY_LOG_CONTROL_SIZE);
    return true;
}

void BaseGroup::reportError(const ErrorCode &error)
{
    ErrorQueue::instance().push(error);
}

void BaseGroup::setUserFrame(const PoseEuler &uf)
{
    user_frame_ = uf;
    manual_teach_.setUserFrame(uf);
}

void BaseGroup::setToolFrame(const PoseEuler &tf)
{
    tool_frame_ = tf;
    manual_teach_.setToolFrame(tf);
}

void BaseGroup::setWorldFrame(const PoseEuler &wf)
{
    world_frame_ = wf;
    manual_teach_.setWorldFrame(wf);
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

// payload
void BaseGroup::getPayload(int &id)
{
    dynamics_ptr_->getPayload(id);
}

ErrorCode BaseGroup::setPayload(int id)
{
    return dynamics_ptr_->setPayload(id);
}

ErrorCode BaseGroup::addPayload(const PayloadInfo& info)
{
    return dynamics_ptr_->addPayload(info);
}

ErrorCode BaseGroup::deletePayload(int id)
{
    return dynamics_ptr_->deletePayload(id);
}

ErrorCode BaseGroup::updatePayload(const PayloadInfo& info)
{
    return dynamics_ptr_->updatePayload(info);
}

ErrorCode BaseGroup::movePayload(int expect_id, int original_id)
{
    return dynamics_ptr_->movePayload(expect_id, original_id);
}

ErrorCode BaseGroup::getPayloadInfoById(int id, PayloadInfo& info)
{
    return dynamics_ptr_->getPayloadInfoById(id, info);
}

vector<PayloadSummaryInfo> BaseGroup::getAllValidPayloadSummaryInfo(void)
{
    return dynamics_ptr_->getAllValidPayloadSummaryInfo();
}

void BaseGroup::getAllValidPayloadSummaryInfo(vector<PayloadSummaryInfo>& info_list)
{
    dynamics_ptr_->getAllValidPayloadSummaryInfo(info_list);
}

ErrorCode BaseGroup::convertCartToJoint(const PoseAndPosture &pose, const PoseEuler &uf, const PoseEuler &tf, Joint &joint)
{
    PoseEuler tcp_in_base, fcp_in_base;
    transformation_.convertPoseFromUserToBase(pose.pose, uf, tcp_in_base);
    transformation_.convertTcpToFcp(tcp_in_base, tf, fcp_in_base);
    return kinematics_ptr_->doIK(fcp_in_base, pose.posture, pose.turn, joint) ? SUCCESS : MC_COMPUTE_IK_FAIL;
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

ErrorCode BaseGroup::convertCartToJoint(const PoseAndPosture &pose, Joint &joint)
{
    PoseEuler tcp_in_base, fcp_in_base;
    transformation_.convertPoseFromUserToBase(pose.pose, user_frame_, tcp_in_base);
    transformation_.convertTcpToFcp(tcp_in_base, tool_frame_, fcp_in_base);
    return kinematics_ptr_->doIK(fcp_in_base, pose.posture, pose.turn, joint) ? SUCCESS : MC_COMPUTE_IK_FAIL;
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
    return manual_teach_.getManualFrame();
}

ErrorCode BaseGroup::setManualFrame(ManualFrame frame)
{
    LogProducer::info("mc_base","Set manual frame = %d, current frame is %d", frame, manual_teach_.getManualFrame());
    MotionControlState mc_state = mc_state_;

    if (mc_state != STANDBY && mc_state != PAUSE)
    {
        LogProducer::error("mc_base","Cannot set frame in current state = %s", getMontionControlStatusString(mc_state).c_str());
        return INVALID_SEQUENCE;
    }

    pthread_mutex_lock(&manual_traj_mutex_);

    if (frame != manual_teach_.getManualFrame())
    {
        manual_teach_.setManualFrame(frame);
    }

    pthread_mutex_unlock(&manual_traj_mutex_);
    LogProducer::info("mc_base","Done.");
    return SUCCESS;
}

void BaseGroup::getManualStepAxis(double *steps)
{
    char buffer[LOG_TEXT_SIZE];
    manual_teach_.getManualStepAxis(steps);
    LogProducer::info("mc_base","Get manual step axis = %s", printDBLine(steps, buffer, LOG_TEXT_SIZE));
}

double BaseGroup::getManualStepPosition(void)
{
    double step = manual_teach_.getManualStepPosition();
    LogProducer::info("mc_base","Get manual step position = %.4f", step);
    return step;
}

double BaseGroup::getManualStepOrientation(void)
{
    double step = manual_teach_.getManualStepOrientation();
    LogProducer::info("mc_base","Get manual step orientation = %.6f", step);
    return step;
}

ErrorCode BaseGroup::setManualStepAxis(const double *steps)
{
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_base","Set manual steps for axis = %s", printDBLine(steps, buffer, LOG_TEXT_SIZE));
    return manual_teach_.setManualStepAxis(steps);
}

ErrorCode BaseGroup::setManualStepPosition(double step)
{
    LogProducer::info("mc_base","Set manual step position = %.4f", step);
    return manual_teach_.setManualStepPosition(step);
}

ErrorCode BaseGroup::setManualStepOrientation(double step)
{
    LogProducer::info("mc_base","Set manual step orientation = %.6f", step);
    return manual_teach_.setManualStepOrientation(step);
}

ErrorCode BaseGroup::manualMoveToPoint(const IntactPoint &point)
{
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_base","Manual to target point");
    MotionControlState mc_state = mc_state_;
    ServoState servo_state = getServoState();

    if ((mc_state != STANDBY && mc_state != PAUSE) || servo_state != SERVO_IDLE)
    {
        LogProducer::error("mc_base","Cannot manual to target in current MC-state = %s, servo-state = %s", 
            getMontionControlStatusString(mc_state).c_str(), getMCServoStatusString(servo_state).c_str());
        return MC_FAIL_MANUAL_TO_POINT;
    }

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

    if ((mc_state != STANDBY && mc_state != PAUSE) || servo_state != SERVO_IDLE)
    {
        LogProducer::error("mc_base","Cannot manual step in current MC-state = %s, servo-state = %s", 
            getMontionControlStatusString(mc_state).c_str(), getMCServoStatusString(servo_state).c_str());
        return MC_FAIL_MANUAL_STEP;
    }

    start_joint_ = getLatestJoint();
    Joint start_joint = start_joint_;

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

    pthread_mutex_lock(&manual_traj_mutex_);
    manual_time_ = 0;
    ErrorCode err = manual_teach_.manualStep(direction, start_joint);
    double duration = manual_teach_.getDuration();
    pthread_mutex_unlock(&manual_traj_mutex_);

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
    MotionControlState mc_state = mc_state_;
    ServoState servo_state = getServoState();
    size_t joint_num = getNumberOfJoint();

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

        /*
        double duration = manual_teach_.getDuration() < 0.5 ? manual_teach_.getDuration() : 0.5;
        double step_time = cycle_time_;
        err = checkManualTrajectory(step_time, duration, step_time, start_joint);

        if (err != SUCCESS)
        {
            LogProducer::error("mc_base","Fail to check manual trajectory, error-code = 0x%llx", err);
            return err;
        }
        */

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
            mc_state == MANUAL || mc_state == PAUSE_MANUAL || mc_state == OFFLINE);
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
    
    if (mc_state == PAUSE && servo_state == SERVO_IDLE)
    {
        pause_to_auto_request_ = true;
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


MotionControlState BaseGroup::getMotionControlState(void)
{
    return mc_state_;
}

ErrorCode BaseGroup::setGlobalVelRatio(double ratio)
{
    LogProducer::info("mc_base","Set global velocity ratio: %.4f", ratio);

    if (ratio < MINIMUM_E3 || ratio > 1)
    {
        LogProducer::error("mc_base","Given ratio out of range (0, 1)");
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
    LogProducer::info("mc_base","Set global acceleration ratio: %.4f", ratio);

    if (ratio < MINIMUM_E3 || ratio > 1)
    {
        LogProducer::error("mc_base","Given ratio out of range (0, 1)");
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

ErrorCode BaseGroup::pickManualPoint(TrajectoryPoint &point)
{
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

bool BaseGroup::updateStartJoint(void)
{
    start_joint_ = getLatestJoint();

    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_base","Update Start-position: %s", printDBLine(&start_joint_[0], buffer, LOG_TEXT_SIZE));
    return true;
}

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

void BaseGroup::doCommonLoop(void)
{
    updateJointRecorder();
    doStateMachine();
}

void BaseGroup::doRealtimeLoop(void)
{
    sendTrajectoryFlow();
}

void BaseGroup::doPriorityLoop(void)
{
    updateServoStateAndJoint();
    //checkEncoderState();
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
    static uint32_t fail_cnt = 0;

    if (bare_core_.getLatestJoint(barecore_joint, encoder_state, barecore_state))
    {
        pthread_mutex_lock(&servo_mutex_);
        servo_state_ = barecore_state;
        servo_joint_ = barecore_joint;
        memcpy(encoder_state_, encoder_state, sizeof(encoder_state_));
        pthread_mutex_unlock(&servo_mutex_);
        fail_cnt = 0;

        if (last_servo_state != servo_state_)
        {
            LogProducer::info("mc_base","Servo-state switch %s to %s", getMCServoStatusString(last_servo_state).c_str(),
                getMCServoStatusString(servo_state_).c_str());

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
    else
    {
        if (++fail_cnt > servo_update_timeout_)
        {
            fail_cnt = 0;
            LogProducer::error("mc_base","Fail to update joint and state from bare core.");
            reportError(MC_FAIL_GET_FEEDBACK_JOINT);
        }
    }
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
                LogProducer::error("mc_base","sendTrajectoryFlow: bare core time-out, servo state: 0x%x.", servo_state);
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

int BaseGroup::getID(void)
{
    return id_;
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
    LogProducer::info("mc_base","Get soft constraint:");
    LogProducer::info("mc_base","  lower = %s", printDBLine(&soft_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","  upper = %s", printDBLine(&soft_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::getFirmConstraint(JointConstraint &firm_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    firm_constraint_.getConstraint(firm_constraint);
    LogProducer::info("mc_base","Get firm constraint.");
    LogProducer::info("mc_base","  lower = %s", printDBLine(&firm_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","  upper = %s", printDBLine(&firm_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::getHardConstraint(JointConstraint &hard_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    hard_constraint_.getConstraint(hard_constraint);
    LogProducer::info("mc_base","Get hard constraint.");
    LogProducer::info("mc_base","  lower = %s", printDBLine(&hard_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","  upper = %s", printDBLine(&hard_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::setSoftConstraint(const JointConstraint &soft_constraint)
{
    Joint lower, upper;
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_base","Set soft constraint.");

    soft_constraint_.getConstraint(lower, upper);
    LogProducer::info("mc_base","Origin soft constraint:");
    LogProducer::info("mc_base","  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    LogProducer::info("mc_base","Given soft constraint:");
    LogProducer::info("mc_base","  lower = %s", printDBLine(&soft_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","  upper = %s", printDBLine(&soft_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    firm_constraint_.getConstraint(lower, upper);
    LogProducer::info("mc_base","Firm constraint:");
    LogProducer::info("mc_base","  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    if (firm_constraint_.isCoverConstaint(soft_constraint))
    {
        base_space::YamlHelp param;
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
            param.dumpParamFile(AXIS_GROUP_DIR"soft_constraint.yaml"))
        {
            soft_constraint_.setConstraint(soft_constraint);
            LogProducer::info("mc_base","Soft constraint updated successfully.");
            return SUCCESS;
        }
        else
        {
            LogProducer::error("mc_base","Fail dumping soft constraint to config file");
            return MC_SET_PARAM_FAILED;
        }
    }
    else
    {
        LogProducer::error("mc_base","Given soft constraint out of firm constraint.");
        return INVALID_PARAMETER;
    }
}


ErrorCode BaseGroup::setFirmConstraint(const JointConstraint &firm_constraint)
{
    Joint lower, upper;
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_base","Set firm constraint.");

    firm_constraint_.getConstraint(lower, upper);
    LogProducer::info("mc_base","Origin firm constraint:");
    LogProducer::info("mc_base","  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    LogProducer::info("mc_base","Given firm constraint:");
    LogProducer::info("mc_base","  lower = %s", printDBLine(&firm_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","  upper = %s", printDBLine(&firm_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    hard_constraint_.getConstraint(lower, upper);
    LogProducer::info("mc_base","Hard constraint:");
    LogProducer::info("mc_base","  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    if (hard_constraint_.isCoverConstaint(firm_constraint_))
    {
        base_space::YamlHelp param;
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
            param.dumpParamFile(AXIS_GROUP_DIR"firm_constraint.yaml"))
        {
            firm_constraint_.setConstraint(firm_constraint);
            LogProducer::info("mc_base","Firm constraint updated successfully.");
            return SUCCESS;
        }
        else
        {
            LogProducer::error("mc_base","Fail dumping firm constraint to config file");
            return MC_SET_PARAM_FAILED;
        }
    }
    else
    {
        LogProducer::error("mc_base","Given firm constraint out of hard constraint.");
        return INVALID_PARAMETER;
    }
}


ErrorCode BaseGroup::setHardConstraint(const JointConstraint &hard_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_base","Set hard constraint.");
    LogProducer::info("mc_base","Origin hard constraint:");
    LogProducer::info("mc_base","  lower = %s", printDBLine(&hard_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","  upper = %s", printDBLine(&hard_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));

    LogProducer::info("mc_base","Given hard constraint:");
    LogProducer::info("mc_base","  lower = %s", printDBLine(&hard_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_base","  upper = %s", printDBLine(&hard_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    base_space::YamlHelp param;
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
        param.dumpParamFile(AXIS_GROUP_DIR"hard_constraint.yaml"))
    {
        hard_constraint_.setConstraint(hard_constraint);
        LogProducer::info("mc_base","Hard constraint updated successfully.");
        return SUCCESS;
    }
    else
    {
        LogProducer::error("mc_base","Fail dumping hard constraint to config file");
        return MC_SET_PARAM_FAILED;
    }
}

std::string BaseGroup::getMCServoStatusString(ServoState servo_status)
{
    switch(servo_status)
    {
        case SERVO_INIT:              return std::string("UNKNOWN");
        case SERVO_IDLE:              return std::string("MC_SERVO_IDLE");
        case SERVO_RUNNING:           return std::string("MC_SERVO_RUNNING");
        case SERVO_DISABLE:           return std::string("MC_SERVO_DISABLE");
        case SERVO_WAIT_READY:        return std::string("MC_SERVO_WAIT_READY");
        case SERVO_WAIT_DOWN:         return std::string("MC_SERVO_WAIT_DOWN");
        default:                      return std::string("Unknown");
    }
}

std::string BaseGroup::getMontionControlStatusString(MotionControlState mc_status)
{
    switch(mc_status)
    {
        case STANDBY:               return std::string("MC_STANDBY");
        case MANUAL:                return std::string("MC_MANUAL");
        case AUTO:                  return std::string("MC_AUTO");
        case PAUSE:                 return std::string("MC_PAUSE");
        case PAUSE_RETURN:          return std::string("MC_PAUSE_RETURN");
        case PAUSE_MANUAL:          return std::string("MC_PAUSE_MANUAL");
        case PAUSING:               return std::string("MC_PAUSING");
        case OFFLINE:               return std::string("MC_OFFLINE");
        case RESUME:                return std::string("MC_RESUME");
        case PREPARE_RESUME:        return std::string("MC_PREPARE_RESUME");

        case MANUAL_TO_STANDBY:     return std::string("MC_MANUAL_TO_STANDBY");
        case STANDBY_TO_MANUAL:     return std::string("MC_STANDBY_TO_MANUAL");
        case AUTO_TO_STANDBY:       return std::string("MC_AUTO_TO_STANDBY");
        case STANDBY_TO_AUTO:       return std::string("MC_STANDBY_TO_AUTO");
        case STANDBY_TO_OFFLINE:    return std::string("MC_STANDBY_TO_OFFLINE");
        case OFFLINE_TO_STANDBY:    return std::string("MC_OFFLINE_TO_STANDBY");
        case AUTO_TO_PAUSING:       return std::string("MC_AUTO_TO_PAUSING");
        case PAUSING_TO_PAUSE:      return std::string("MC_PAUSING_TO_PAUSE");
        case PAUSE_TO_RESUME:       return std::string("MC_PAUSE_TO_RESUME");
        case PAUSE_RETURN_TO_PAUSE: return std::string("MC_PAUSE_RETURN_TO_PAUSE");
        case PAUSE_TO_PAUSE_RETURN: return std::string("MC_PAUSE_TO_PAUSE_RETURN");
        case PAUSE_TO_PAUSE_MANUAL: return std::string("MC_PAUSE_TO_PAUSE_MANUAL");
        case PAUSE_MANUAL_TO_PAUSE: return std::string("MC_PAUSE_MANUAL_TO_PAUSE");
        default:                    return std::string("MC_Unknown");
    }
}


} // namespace fst_mc
