#include <string.h>
#include <motion_control_ros_basic.h>
#include <motion_control.h>
#include <tool_manager.h>
#include <coordinate_manager.h>
#include "../../coordinate_manager/include/coordinate_manager.h"
#include "../../tool_manager/include/tool_manager.h"



using namespace basic_alg;
using namespace fst_base;
using namespace basic_alg;
using namespace fst_mc;
using namespace fst_hal;
using namespace fst_ctrl;



static void runRealTimeTask(void *mc)
{
    ((MotionControl*)mc)->ringPriorityTask();
}

static void runNonRealTimeTask(void *mc)
{
    ((MotionControl*)mc)->ringCommonTask();
}

void MotionControl::ringCommonTask(void)
{
    Joint servo_joint;
    int ros_publish_cnt = 0;
    memset(&servo_joint, 0, sizeof(servo_joint));

    FST_WARN("Non-Realtime task start.");

    while (non_rt_thread_running_)
    {
        group_ptr_->doCommonLoop();
        
        if (param_ptr_->enable_ros_publish_)
        {
            ros_publish_cnt ++;

            if (ros_publish_cnt >= param_ptr_->cycle_per_publish_)
            {
                ros_publish_cnt = 0;
                group_ptr_->getLatestJoint(servo_joint);
                ros_basic_ptr_->pubJointState(servo_joint);
            }
        }

        usleep(param_ptr_->non_rt_cycle_time_ * 1000);
    }

    FST_WARN("Non-Realtime task quit.");
}

void MotionControl::ringPriorityTask(void)
{
    FST_WARN("Realtime task start.");

    while (rt_thread_running_)
    {
        group_ptr_->doPriorityLoop();
        usleep(param_ptr_->rt_cycle_time_ * 1000);
    }

    FST_WARN("Realtime task quit.");
}


MotionControl::MotionControl()
{
    device_manager_ptr_ = NULL;
    axis_group_manager_ptr_ = NULL;
    coordinate_manager_ptr_ = NULL;
    tool_manager_ptr_ = NULL;
    error_monitor_ptr_ = NULL;
    log_ptr_ = NULL;
    param_ptr_ = NULL;

    rt_thread_running_ = false;
    non_rt_thread_running_ = false;

    ros_basic_ptr_ = NULL;
}

MotionControl::~MotionControl()
{
    if (non_rt_thread_running_)
    {
        non_rt_thread_running_ = false;
        non_rt_thread_.join();
    }

    if (rt_thread_running_)
    {
        rt_thread_running_ = false;
        rt_thread_.join();
    }

    if (group_ptr_ != NULL)     {delete group_ptr_; group_ptr_ = NULL;};
    if (param_ptr_ != NULL)     {delete param_ptr_; param_ptr_ = NULL;};
    if (log_ptr_ != NULL)       {delete log_ptr_; log_ptr_ = NULL;};
    if (ros_basic_ptr_ != NULL)  {delete ros_basic_ptr_; ros_basic_ptr_ = NULL;};
}

ErrorCode MotionControl::init(fst_hal::DeviceManager* device_manager_ptr, AxisGroupManager* axis_group_manager_ptr,
                              fst_ctrl::CoordinateManager* coordinate_manager_ptr, fst_ctrl::ToolManager* tool_manager_ptr,
                              fst_base::ErrorMonitor *error_monitor_ptr)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new MotionControlParam();
    
    if (log_ptr_ == NULL || param_ptr_ == NULL)
    {
        return MC_INTERNAL_FAULT;
    }

    if (!log_ptr_->initLogger("MotionControl"))
    {
        FST_ERROR("Lost communication with log server, init MotionControl abort.");
        return MC_INTERNAL_FAULT;
    }

    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load MotionControl component parameters");
        return MC_INTERNAL_FAULT;
    }

    FST_INFO("Log-level of MotionControl setted to: %d", param_ptr_->log_level_);
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);

    if (param_ptr_->model_name_ == "PUMA")
    {
        group_ptr_ = new ArmGroup(log_ptr_);
    }
    else if (param_ptr_->model_name_ == "SCARA")
    {
        group_ptr_ = new ScaraGroup(log_ptr_);
    }
    else
    {
        FST_ERROR("Invalid model name: %s", param_ptr_->model_name_.c_str());
        return MC_INTERNAL_FAULT;
    }

    if (group_ptr_ == NULL)
    {
        FST_ERROR("Fail to create control group of %s", param_ptr_->model_name_.c_str());
        return MC_INTERNAL_FAULT;
    }

    //if (device_manager_ptr && axis_group_manager_ptr && coordinate_manager_ptr && tool_manager_ptr && error_monitor_ptr)
    if (coordinate_manager_ptr && tool_manager_ptr && error_monitor_ptr)
    {
        //device_manager_ptr_ = device_manager_ptr;
        //axis_group_manager_ptr_ = axis_group_manager_ptr;
        coordinate_manager_ptr_ = coordinate_manager_ptr;
        tool_manager_ptr_ = tool_manager_ptr;
        error_monitor_ptr_ = error_monitor_ptr;
    }
    else
    {
        FST_ERROR("device-manger: %x, group-manager: %x, coordinate-manager: %x, tool-manager: %x, error-monitor: %x",
                  device_manager_ptr, axis_group_manager_ptr, coordinate_manager_ptr, tool_manager_ptr, error_monitor_ptr);
        return INVALID_PARAMETER;
    }


    user_frame_id_ = 0;
    tool_frame_id_ = 0;

    if (param_ptr_->enable_ros_publish_)
    {
        ros_basic_ptr_ = new RosBasic;
        ros_basic_ptr_->initRosBasic();
    }

    ErrorCode  err = group_ptr_->initGroup(error_monitor_ptr, coordinate_manager_ptr_, tool_manager_ptr_);

    if (err == SUCCESS)
    {
        rt_thread_running_ = true;

        if (rt_thread_.run(&runRealTimeTask, this, 80))
        {
            FST_INFO("Startup real-time task success.");
        }
        else
        {
            FST_ERROR("Fail to create real-time task.");
            rt_thread_running_ = false;
            return MC_INTERNAL_FAULT;
        }

        usleep(50 * 1000);
        non_rt_thread_running_ = true;

        if (non_rt_thread_.run(&runNonRealTimeTask, this, 40))
        {
            FST_INFO("Startup non-real-time task success.");
        }
        else
        {
            FST_ERROR("Fail to create non-real-time task.");
            non_rt_thread_running_ = false;
            return MC_INTERNAL_FAULT;
        }

        FST_INFO("Initialize motion group success.");
    }
    else
    {
        FST_ERROR("Fail to init motion group");
        return err;
    }

    return SUCCESS;
}




ManualFrame MotionControl::getManualFrame(void)
{
    return group_ptr_->getManualFrame();
}

ErrorCode MotionControl::setManualFrame(ManualFrame frame)
{
    return group_ptr_->setManualFrame(frame);
}


void MotionControl::getAxisManualStep(double (&steps)[NUM_OF_JOINT])
{
    group_ptr_->getManualStepAxis(steps);
}

double MotionControl::getPositionManualStep(void)
{
    return group_ptr_->getManualStepPosition();
}

double MotionControl::getOrientationManualStep(void)
{
    return group_ptr_->getManualStepOrientation();
}

ErrorCode MotionControl::setAxisManualStep(const double (&steps)[NUM_OF_JOINT])
{
    return group_ptr_->setManualStepAxis(steps);
}

ErrorCode MotionControl::setPositionManualStep(double step)
{
    return group_ptr_->setManualStepPosition(step);
}

ErrorCode MotionControl::setOrientationManualStep(double step)
{
    return group_ptr_->setManualStepOrientation(step);
}

ErrorCode MotionControl::doStepManualMove(const GroupDirection &direction)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_FORBIDDEN)
    {
        FST_ERROR("Cannot manual move, calibrator-state = %d, all motion is forbidden.", MOTION_FORBIDDEN);
        return INVALID_SEQUENCE;
    }
    else if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_LIMITED)
    {
        if (group_ptr_->getManualFrame() != JOINT)
        {
            FST_ERROR("Cannot manual cartesian in limited state, calibrator-state = %d, manual cartesian is forbidden.", MOTION_LIMITED);
            return INVALID_SEQUENCE;
        }
    }

    return group_ptr_->manualMoveStep(&direction[0]);
}

ErrorCode MotionControl::doContinuousManualMove(const GroupDirection &direction)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_FORBIDDEN)
    {
        FST_ERROR("Cannot manual move, calibrator-state = %d, all motion is forbidden.", MOTION_FORBIDDEN);
        return INVALID_SEQUENCE;
    }
    else if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_LIMITED)
    {
        if (group_ptr_->getManualFrame() != JOINT)
        {
            FST_ERROR("Cannot manual cartesian in limited state, calibrator-state = %d, manual-frame = %d, manual cartesian is forbidden.", MOTION_LIMITED, group_ptr_->getManualFrame());
            return INVALID_SEQUENCE;
        }
    }

    return group_ptr_->manualMoveContinuous(&direction[0]);
}

ErrorCode MotionControl::doGotoPointManualMove(const Joint &joint)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        FST_ERROR("Cannot manual move to point in current state, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    IntactPoint point;
    point.joint = joint;
    point.tool_frame = group_ptr_->getToolFrame();
    point.user_frame = group_ptr_->getUserFrame();
    PoseEuler tcp_in_base, fcp_in_base;
    group_ptr_->getKinematicsPtr()->doFK(point.joint, fcp_in_base);
    group_ptr_->getTransformationPtr()->convertFcpToTcp(fcp_in_base, point.tool_frame, tcp_in_base);
    group_ptr_->getTransformationPtr()->convertPoseFromBaseToUser(tcp_in_base, point.user_frame, point.pose.pose);
    point.pose.posture = group_ptr_->getKinematicsPtr()->getPostureByJoint(point.joint);
    return group_ptr_->manualMoveToPoint(point);
}

ErrorCode MotionControl::doGotoPointManualMove(const PoseAndPosture &pose, int user_frame_id, int tool_frame_id)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        FST_ERROR("Cannot manual move to point in current state, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    if (user_frame_id != user_frame_id_ && user_frame_id != -1)
    {
        FST_ERROR("manualMove: user frame ID = %d mismatch with activated user frame = %d.", user_frame_id, user_frame_id_);
        return INVALID_PARAMETER;
    }

    if (tool_frame_id != tool_frame_id_ && tool_frame_id != -1)
    {
        FST_ERROR("manualMove: tool frame ID = %d mismatch with activated tool frame = %d.", tool_frame_id, tool_frame_id_);
        return INVALID_PARAMETER;
    }

    IntactPoint point;
    point.pose = pose;
    point.tool_frame = group_ptr_->getToolFrame();
    point.user_frame = group_ptr_->getUserFrame();
    PoseEuler tcp_in_base, fcp_in_base;
    group_ptr_->getTransformationPtr()->convertPoseFromUserToBase(point.pose.pose, point.user_frame, tcp_in_base);
    group_ptr_->getTransformationPtr()->convertTcpToFcp(tcp_in_base, point.tool_frame, fcp_in_base);

    if (!group_ptr_->getKinematicsPtr()->doIK(fcp_in_base, point.pose.posture, point.joint))
    {
        const PoseEuler &pe = point.pose.pose;
        const PoseEuler &tf = point.tool_frame;
        const PoseEuler &uf = point.user_frame;
        FST_ERROR("IK of manual target pose failed.");
        FST_ERROR("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pe.point_.x_, pe.point_.y_, pe.point_.z_, pe.euler_.a_, pe.euler_.b_, pe.euler_.c_);
        FST_ERROR("Posture: %d, %d, %d, %d", point.pose.posture.arm, point.pose.posture.elbow, point.pose.posture.wrist, point.pose.posture.flip);
        FST_ERROR("Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
        FST_ERROR("User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    return group_ptr_->manualMoveToPoint(point);
}

ErrorCode MotionControl::doGotoPointManualMove(const PoseEuler &pose)
{
    // To delete
    return SUCCESS;
}

ErrorCode MotionControl::manualStop(void)
{
    return group_ptr_->manualStop();
}

ErrorCode MotionControl::autoMove(int id, const MotionTarget &target)
{
    FST_INFO("MotionControl::autoMove called.");

    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        FST_ERROR("Offset of the group is abnormal, auto move is forbidden, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    if (target.type != MOTION_JOINT && target.type != MOTION_LINE && target.type != MOTION_CIRCLE)
    {
        FST_ERROR("Invalid motion type = %d, autoMove aborted.", target.type);
        return INVALID_PARAMETER;
    }

    if (target.user_frame_id != user_frame_id_ && target.user_frame_id != -1)
    {
        FST_ERROR("autoMove: user frame ID = %d mismatch with activated user frame = %d.", target.user_frame_id, user_frame_id_);
        return INVALID_PARAMETER;
    }

    if (target.tool_frame_id != tool_frame_id_ && target.tool_frame_id != -1)
    {
        FST_ERROR("autoMove: tool frame ID = %d mismatch with activated tool frame = %d.", target.tool_frame_id, tool_frame_id_);
        return INVALID_PARAMETER;
    }

    MotionInfo motion_info;
    motion_info.type = target.type;
    motion_info.cnt = target.cnt;
    motion_info.vel = target.vel;
    motion_info.target.user_frame = group_ptr_->getUserFrame();
    motion_info.target.tool_frame = group_ptr_->getToolFrame();

    if (target.target.type == COORDINATE_JOINT)
    {
        PoseEuler fcp_in_base, tcp_in_base;
        motion_info.target.joint = target.target.joint;
        motion_info.target.pose.posture = group_ptr_->getKinematicsPtr()->getPostureByJoint(target.target.joint);
        group_ptr_->getKinematicsPtr()->doFK(target.target.joint, fcp_in_base);
        group_ptr_->getTransformationPtr()->convertFcpToTcp(fcp_in_base, motion_info.target.tool_frame, tcp_in_base);
        group_ptr_->getTransformationPtr()->convertPoseFromBaseToUser(tcp_in_base, motion_info.target.user_frame, motion_info.target.pose.pose);
    }
    else
    {
        PoseEuler fcp_in_base, tcp_in_base;
        motion_info.target.pose = target.target.pose;
        group_ptr_->getTransformationPtr()->convertPoseFromUserToBase(target.target.pose.pose, motion_info.target.user_frame, tcp_in_base);
        group_ptr_->getTransformationPtr()->convertTcpToFcp(tcp_in_base, motion_info.target.tool_frame, fcp_in_base);
        
        if (!group_ptr_->getKinematicsPtr()->doIK(fcp_in_base, target.target.pose.posture, motion_info.target.joint))
        {
            const PoseEuler &pose = target.target.pose.pose;
            const PoseEuler &tf = motion_info.target.tool_frame;
            const PoseEuler &uf = motion_info.target.user_frame;
            FST_ERROR("IK of target pose failed.");
            FST_ERROR("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            FST_ERROR("Posture: %d, %d, %d, %d", target.target.pose.posture.arm, target.target.pose.posture.elbow, target.target.pose.posture.wrist, target.target.pose.posture.flip);
            FST_ERROR("Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            FST_ERROR("User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            return MC_COMPUTE_IK_FAIL;
        }
    }

    if (target.type == MOTION_CIRCLE)
    {
        motion_info.via.user_frame = group_ptr_->getUserFrame();
        motion_info.via.tool_frame = group_ptr_->getToolFrame();

        if (target.via.type == COORDINATE_JOINT)
        {
            PoseEuler fcp_in_base, tcp_in_base;
            motion_info.via.joint = target.via.joint;
            motion_info.via.pose.posture = group_ptr_->getKinematicsPtr()->getPostureByJoint(target.via.joint);
            group_ptr_->getKinematicsPtr()->doFK(target.via.joint, fcp_in_base);
            group_ptr_->getTransformationPtr()->convertFcpToTcp(fcp_in_base, motion_info.via.tool_frame, tcp_in_base);
            group_ptr_->getTransformationPtr()->convertPoseFromBaseToUser(tcp_in_base, motion_info.via.user_frame, motion_info.via.pose.pose);
        }
        else
        {
            PoseEuler fcp_in_base, tcp_in_base;
            motion_info.via.pose = target.via.pose;
            group_ptr_->getTransformationPtr()->convertPoseFromUserToBase(target.via.pose.pose, motion_info.via.user_frame, tcp_in_base);
            group_ptr_->getTransformationPtr()->convertTcpToFcp(tcp_in_base, motion_info.via.tool_frame, fcp_in_base);
            
            if (!group_ptr_->getKinematicsPtr()->doIK(fcp_in_base, target.via.pose.posture, motion_info.via.joint))
            {
                const PoseEuler &pose = target.via.pose.pose;
                const PoseEuler &tf = motion_info.via.tool_frame;
                const PoseEuler &uf = motion_info.via.user_frame;
                FST_ERROR("IK of via pose failed.");
                FST_ERROR("Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
                FST_ERROR("Posture: %d, %d, %d, %d", target.via.pose.posture.arm, target.via.pose.posture.elbow, target.via.pose.posture.wrist, target.via.pose.posture.flip);
                FST_ERROR("Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
                FST_ERROR("User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
                return MC_COMPUTE_IK_FAIL;
            }
        }
    }

    return group_ptr_->autoMove(id, motion_info);
}

ErrorCode MotionControl::abortMove(void)
{
    return group_ptr_->abortMove();
}

ErrorCode MotionControl::pauseMove(void)
{
    return group_ptr_->pauseMove();
}

ErrorCode MotionControl::restartMove(void)
{
    return group_ptr_->restartMove();
}

bool MotionControl::nextMovePermitted(void)
{
    return group_ptr_->nextMovePermitted();
}

void MotionControl::setOffset(size_t index, double offset)
{
    group_ptr_->getCalibratorPtr()->setOffset(index, offset);
}

void MotionControl::setOffset(const double (&offset)[NUM_OF_JOINT])
{
    group_ptr_->getCalibratorPtr()->setOffset(offset);
}

void MotionControl::getOffset(double (&offset)[NUM_OF_JOINT])
{
    group_ptr_->getCalibratorPtr()->getOffset(offset);
}

void MotionControl::getOffsetMask(OffsetMask (&mask)[NUM_OF_JOINT])
{
    group_ptr_->getCalibratorPtr()->getOffsetMask(mask);
}

CalibrateState MotionControl::getCalibrateState(void)
{
    return group_ptr_->getCalibratorPtr()->getCalibrateState();
}

/*
ErrorCode MotionControl::saveJoint(void)
{
    //return group_ptr_->getCalibratorPtr()->saveJoint();
    return SUCCESS;
}
*/

ErrorCode MotionControl::saveOffset(void)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }
    
    ErrorCode err = group_ptr_->getCalibratorPtr()->saveOffset();

    if (err == SUCCESS)
    {
        size_t length = 0;
        size_t index[NUM_OF_JOINT];
        OffsetMask mask[NUM_OF_JOINT];
        group_ptr_->getCalibratorPtr()->getOffsetMask(mask);

        for (size_t i = 0; i < group_ptr_->getNumberOfJoint(); i++)
        {
            if (mask[i] == OFFSET_UNMASK)
            {
                index[length++] = i;
            }
        }

        group_ptr_->getSoftConstraintPtr()->resetMask(index, length);
        return SUCCESS;
    }
    else
    {
        return err;
    }
}

ErrorCode MotionControl::checkOffset(CalibrateState &cali_stat, OffsetState (&offset_stat)[NUM_OF_JOINT])
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->checkOffset(cali_stat, offset_stat);
}

ErrorCode MotionControl::maskOffsetLostError(void)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    ErrorCode err = group_ptr_->getCalibratorPtr()->maskOffsetLostError();

    if (err == SUCCESS)
    {
        size_t length = 0;
        size_t index[NUM_OF_JOINT];
        OffsetMask mask[NUM_OF_JOINT];
        group_ptr_->getCalibratorPtr()->getOffsetMask(mask);

        for (size_t i = 0; i < group_ptr_->getNumberOfJoint(); i++)
        {
            if (mask[i] == OFFSET_MASKED)
            {
                index[length++] = i;
            }
        }

        group_ptr_->getSoftConstraintPtr()->setMask(index, length);
        return SUCCESS;
    }
    else
    {
        return err;
    }
}

void MotionControl::getOffsetState(OffsetState (&offset_stat)[NUM_OF_JOINT])
{
    return group_ptr_->getCalibratorPtr()->getOffsetState(offset_stat);
}

ErrorCode MotionControl::setOffsetState(size_t index, OffsetState stat)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->setOffsetState(index, stat);
}

ErrorCode MotionControl::calibrateOffset(void)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->calibrateOffset();
}

ErrorCode MotionControl::calibrateOffset(size_t index)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->calibrateOffset(index);
}

ErrorCode MotionControl::calibrateOffset(const size_t *pindex, size_t length)
{
    if (group_ptr_->getGroupState() != DISABLE || group_ptr_->getServoState() != SERVO_DISABLE)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->calibrateOffset(pindex, length);
}

bool MotionControl::isReferenceAvailable(void)
{
    return group_ptr_->getCalibratorPtr()->isReferenceAvailable();
}

ErrorCode MotionControl::deleteReference(void)
{
    return group_ptr_->getCalibratorPtr()->deleteReference();
}

ErrorCode MotionControl::saveReference(void)
{
    return group_ptr_->getCalibratorPtr()->saveReference();
}

ErrorCode MotionControl::fastCalibrate(void)
{
    return group_ptr_->getCalibratorPtr()->fastCalibrate();
}

ErrorCode MotionControl::fastCalibrate(size_t index)
{
    return group_ptr_->getCalibratorPtr()->fastCalibrate(index);
}

ErrorCode MotionControl::fastCalibrate(const size_t *pindex, size_t length)
{
    return group_ptr_->getCalibratorPtr()->fastCalibrate(pindex, length);
}

ErrorCode MotionControl::getSoftConstraint(JointConstraint &soft_constraint)
{
    return group_ptr_->getSoftConstraint(soft_constraint);
}

ErrorCode MotionControl::getFirmConstraint(JointConstraint &firm_constraint)
{
    return group_ptr_->getFirmConstraint(firm_constraint);
}

ErrorCode MotionControl::getHardConstraint(JointConstraint &hard_constraint)
{
    return group_ptr_->getHardConstraint(hard_constraint);
}

ErrorCode MotionControl::setSoftConstraint(const JointConstraint &soft_constraint)
{
    return group_ptr_->setSoftConstraint(soft_constraint);
}

ErrorCode MotionControl::setFirmConstraint(const JointConstraint &firm_constraint)
{
    return group_ptr_->setFirmConstraint(firm_constraint);
}

ErrorCode MotionControl::setHardConstraint(const JointConstraint &hard_constraint)
{
    return group_ptr_->setHardConstraint(hard_constraint);
}

ErrorCode MotionControl::stopGroup(void)
{
    return group_ptr_->stopGroup();
}

ErrorCode MotionControl::resetGroup(void)
{
    return group_ptr_->resetGroup();
}

ErrorCode MotionControl::clearGroup(void)
{
    return group_ptr_->clearGroup();
}

ErrorCode MotionControl::convertCartToJoint(const PoseAndPosture &pose, int user_frame_id, int tool_frame_id, Joint &joint)
{
    if (user_frame_id == user_frame_id_ && tool_frame_id == tool_frame_id_)
    {
        return group_ptr_->convertCartToJoint(pose, joint);
    }

    PoseEuler tf, uf;

    if (user_frame_id == 0)
    {
        memset(&uf, 0, sizeof(uf));
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

        if (err_user == SUCCESS && uf_info.is_valid)
        {
            uf = uf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get user frame from given ID.");
            return err_user;
        }
    }

    if (tool_frame_id == 0)
    {
        memset(&tf, 0, sizeof(tf));
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

        if (err_tool == SUCCESS && tf_info.is_valid)
        {
            tf = tf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get tool frame from given id");
            return err_tool;
        }
    }

    return group_ptr_->convertCartToJoint(pose, uf, tf, joint);
}

ErrorCode MotionControl::convertCartToJoint(const PoseEuler &pose, int user_frame_id, int tool_frame_id, Joint &joint)
{
    if (user_frame_id == user_frame_id_ && tool_frame_id == tool_frame_id_)
    {
        return group_ptr_->convertCartToJoint(pose, joint);
    }

    PoseEuler tf, uf;

    if (user_frame_id == 0)
    {
        memset(&uf, 0, sizeof(uf));
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

        if (err_user == SUCCESS && uf_info.is_valid)
        {
            uf = uf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get user frame from given ID.");
            return err_user;
        }
    }

    if (tool_frame_id == 0)
    {
        memset(&tf, 0, sizeof(tf));
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

        if (err_tool == SUCCESS && tf_info.is_valid)
        {
            tf = tf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get tool frame from given id");
            return err_tool;
        }
    }

    return group_ptr_->convertCartToJoint(pose, uf, tf, joint);
}

ErrorCode MotionControl::convertJointToCart(const Joint &joint, int user_frame_id, int tool_frame_id, PoseEuler &pose)
{
    if (user_frame_id == user_frame_id_ && tool_frame_id == tool_frame_id_)
    {
        return group_ptr_->convertJointToCart(joint, pose);  // transform from base to user
    }

    PoseEuler tf, uf;

    if (user_frame_id == 0)
    {
        memset(&uf, 0, sizeof(uf));
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

        if (err_user == SUCCESS && uf_info.is_valid)
        {
            uf = uf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get user frame from given ID.");
            return err_user;
        }
    }

    if (tool_frame_id == 0)
    {
        memset(&tf, 0, sizeof(tf));
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

        if (err_tool == SUCCESS && tf_info.is_valid)
        {
            tf = tf_info.data;
        }
        else
        {
            FST_ERROR("Fail to get tool frame from given id");
            return err_tool;
        }
    }

    return group_ptr_->convertJointToCart(joint, uf, tf, pose);
}

Posture MotionControl::getPostureFromJoint(const Joint &joint)
{
    return group_ptr_->getKinematicsPtr()->getPostureByJoint(joint);
}

string MotionControl::getModelName(void)
{
    return param_ptr_->model_name_;
}

size_t MotionControl::getNumberOfAxis(void)
{
    return group_ptr_->getNumberOfJoint();
}

void MotionControl::getTypeOfAxis(AxisType *types)
{
    group_ptr_->getTypeOfAxis(types);
}

GroupState MotionControl::getGroupState(void)
{
    return group_ptr_->getGroupState();
}

ServoState MotionControl::getServoState(void)
{
    return group_ptr_->getServoState();
}

ErrorCode MotionControl::getServoVersion(std::string &version)
{
    return group_ptr_->getServoVersion(version);
}

PoseEuler MotionControl::getCurrentPose(void)
{
    PoseEuler pose, tcp_in_base, fcp_in_base;
    group_ptr_->getKinematicsPtr()->doFK(group_ptr_->getLatestJoint(), fcp_in_base);
    group_ptr_->getTransformationPtr()->convertFcpToTcp(fcp_in_base, group_ptr_->getToolFrame(), tcp_in_base);
    group_ptr_->getTransformationPtr()->convertPoseFromBaseToUser(tcp_in_base, group_ptr_->getUserFrame(), pose);
    return pose;
}

void MotionControl::getCurrentPose(PoseEuler &pose)
{
    PoseEuler tcp_in_base, fcp_in_base;
    group_ptr_->getKinematicsPtr()->doFK(group_ptr_->getLatestJoint(), fcp_in_base);
    group_ptr_->getTransformationPtr()->convertFcpToTcp(fcp_in_base, group_ptr_->getToolFrame(), tcp_in_base);
    group_ptr_->getTransformationPtr()->convertPoseFromBaseToUser(tcp_in_base, group_ptr_->getUserFrame(), pose);
}

Joint MotionControl::getServoJoint(void)
{
    return group_ptr_->getLatestJoint();
}

void MotionControl::getServoJoint(Joint &joint)
{
    group_ptr_->getLatestJoint(joint);
}

size_t MotionControl::getFIFOLength(void)
{
    return group_ptr_->getFIFOLength();
}

ErrorCode MotionControl::setGlobalVelRatio(double ratio)
{
    return group_ptr_->setGlobalVelRatio(ratio);
}

ErrorCode MotionControl::setGlobalAccRatio(double ratio)
{
    return group_ptr_->setGlobalAccRatio(ratio);
}

double MotionControl::getGlobalVelRatio(void)
{
    return group_ptr_->getGlobalVelRatio();
}

double MotionControl::getGlobalAccRatio(void)
{
    return group_ptr_->getGlobalAccRatio();
}

void MotionControl::getToolFrame(int &id)
{
    id = tool_frame_id_;
}

ErrorCode MotionControl::setToolFrame(int id)
{
    FST_INFO("Set tool frame: id = %d, current is %d", id, tool_frame_id_);

    if (id != tool_frame_id_)
    {
        if (id == 0)
        {
            PoseEuler tf = {0, 0, 0, 0, 0, 0};
            group_ptr_->setToolFrame(tf);
            tool_frame_id_ = id;
            FST_INFO("Success, current tool frame id is %d", tool_frame_id_);
            FST_INFO("Using tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            return SUCCESS;

        }
        else
        {
            ToolInfo  tf_info;
            ErrorCode err = tool_manager_ptr_->getToolInfoById(id, tf_info);

            if (err == SUCCESS && tf_info.is_valid)
            {
                group_ptr_->setToolFrame(tf_info.data);
                tool_frame_id_ = id;
                FST_INFO("Success, current tool frame id is %d", tool_frame_id_);
                FST_INFO("Using tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf_info.data.point_.x_, tf_info.data.point_.y_, tf_info.data.point_.z_, tf_info.data.euler_.a_, tf_info.data.euler_.b_, tf_info.data.euler_.c_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to get tool frame from given id");
                return err;
            }
        }
    }
    else
    {
        FST_INFO("Success!");
        return SUCCESS;
    }
}

void MotionControl::getUserFrame(int &id)
{
    id = user_frame_id_;
}

ErrorCode MotionControl::setUserFrame(int id)
{
    FST_INFO("Set user frame id = %d, current is %d", id, user_frame_id_);

    if (id != user_frame_id_)
    {
        if (id == 0)
        {
            PoseEuler uf = {0, 0, 0, 0, 0, 0};
            group_ptr_->setUserFrame(uf);
            user_frame_id_ = id;
            FST_INFO("Success, current user frame id is %d", user_frame_id_);
            FST_INFO("Using user-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            return SUCCESS;
        }
        else
        {
            CoordInfo uf_info;
            ErrorCode err = coordinate_manager_ptr_->getCoordInfoById(id, uf_info);

            if (err == SUCCESS && uf_info.is_valid)
            {
                group_ptr_->setUserFrame(uf_info.data);
                user_frame_id_ = id;
                FST_INFO("Success, current user frame ID switch to %d", user_frame_id_);
                FST_INFO("Using user-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf_info.data.point_.x_, uf_info.data.point_.y_, uf_info.data.point_.z_, uf_info.data.euler_.a_, uf_info.data.euler_.b_, uf_info.data.euler_.c_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to get user frame from given id");
                return err;
            }
        }
    }
    else
    {
        FST_INFO("Success!");
        return SUCCESS;
    }
}



