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
    /*
    LogProducer::info("convertCartToJoint","PoseAndPosture-->Joint \npose=<%lf,%lf,%lf,%lf,%lf,%lf>\nuser_frame_=<%lf,%lf,%lf,%lf,%lf,%lf>\ntool_frame_=<%lf,%lf,%lf,%lf,%lf,%lf>\ntcp_in_base=<%lf,%lf,%lf,%lf,%lf,%lf>\nfcp_in_base=<%lf,%lf,%lf,%lf,%lf,%lf>",
    pose.pose.point_.x_, pose.pose.point_.y_, pose.pose.point_.z_, pose.pose.euler_.a_,  pose.pose.euler_.b_,  pose.pose.euler_.c_,
    user_frame_.point_.x_, user_frame_.point_.y_,user_frame_.point_.z_,user_frame_.euler_.a_,user_frame_.euler_.b_,user_frame_.euler_.c_,
    tool_frame_.point_.x_, tool_frame_.point_.y_,tool_frame_.point_.z_,tool_frame_.euler_.a_,tool_frame_.euler_.b_,tool_frame_.euler_.c_,
    tcp_in_base.point_.x_, tcp_in_base.point_.y_,tcp_in_base.point_.z_,tcp_in_base.euler_.a_,tcp_in_base.euler_.b_,tcp_in_base.euler_.c_,
    fcp_in_base.point_.x_, fcp_in_base.point_.y_,fcp_in_base.point_.z_,fcp_in_base.euler_.a_,fcp_in_base.euler_.b_,fcp_in_base.euler_.c_
    );*/
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


double BaseGroup::decouplingAxis6ByRad(double fifth_pos, double sixth_pos)
{
    return bare_core_.decouplingAxis6ByRad(fifth_pos, sixth_pos);
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
        case ONLINE:                return std::string("MC_ONLINE");
        case PAUSE_ONLINE:          return std::string("MC_PAUSE_ONLINE");
        case PAUSING_OFFLINE:       return std::string("MC_PAUSING_OFFLINE");
        case PAUSING_OFFLINE_TO_PAUSE: return std::string("MC_PAUSING_OFFLINE_TO_PAUSE");
        case PAUSED_OFFLINE:        return std::string("MC_PAUSED_OFFLINE");

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
