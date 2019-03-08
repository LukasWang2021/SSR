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
        return MOTION_INTERNAL_FAULT;
    }

    if (!log_ptr_->initLogger("MotionControl"))
    {
        FST_ERROR("Lost communication with log server, init MotionControl abort.");
        return MOTION_INTERNAL_FAULT;
    }

    if(!param_ptr_->loadParam())
    {
        FST_ERROR("Failed to load MotionControl component parameters");
        return MOTION_INTERNAL_FAULT;
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
        return MOTION_INTERNAL_FAULT;
    }

    if (group_ptr_ == NULL)
    {
        FST_ERROR("Fail to create control group of %s", param_ptr_->model_name_.c_str());
        return MOTION_INTERNAL_FAULT;
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

    ErrorCode  err = group_ptr_->initGroup(error_monitor_ptr);

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
            return MOTION_INTERNAL_FAULT;
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
            return MOTION_INTERNAL_FAULT;
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

    return group_ptr_->manualMoveToPoint(joint);
}

ErrorCode MotionControl::doGotoPointManualMove(const PoseEuler &pose)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        FST_ERROR("Cannot manual move to point in current state, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    return group_ptr_->manualMoveToPoint(pose);
}

ErrorCode MotionControl::manualStop(void)
{
    return group_ptr_->manualStop();
}

ErrorCode MotionControl::autoMove(int id, const MotionTarget &target)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        FST_ERROR("Offset of the group is abnormal, auto move is forbidden, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    /* For test only
    if (target.type == MOTION_LINE || target.type == MOTION_CIRCLE)
    {
        if (user_frame_id_ != target.user_frame_id)
        {
            FST_ERROR("autoMove: user frame ID = %d mismatch with activated user frame = %d.", target.user_frame_id, user_frame_id_);
            return INVALID_PARAMETER;
        }

        if (tool_frame_id_ != target.tool_frame_id)
        {
            FST_ERROR("autoMove: tool frame ID = %d mismatch with activated tool frame = %d.", target.tool_frame_id, tool_frame_id_);
            return INVALID_PARAMETER;
        }
    }
    */

    return group_ptr_->autoMove(id, target);
}

ErrorCode MotionControl::abortMove(void)
{
    return group_ptr_->abortMove();
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

ErrorCode MotionControl::saveJoint(void)
{
    return group_ptr_->getCalibratorPtr()->saveJoint();
}

ErrorCode MotionControl::saveOffset(void)
{
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
    return group_ptr_->getCalibratorPtr()->checkOffset(cali_stat, offset_stat);
}

ErrorCode MotionControl::maskOffsetLostError(void)
{
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

ErrorCode MotionControl::setOffsetState(size_t index, OffsetState stat)
{
    return group_ptr_->getCalibratorPtr()->setOffsetState(index, stat);
}

ErrorCode MotionControl::calibrateOffset(void)
{
    return group_ptr_->getCalibratorPtr()->calibrateOffset();
}

ErrorCode MotionControl::calibrateOffset(size_t index)
{
    return group_ptr_->getCalibratorPtr()->calibrateOffset(index);
}

ErrorCode MotionControl::calibrateOffset(const size_t *pindex, size_t length)
{
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

ErrorCode MotionControl::convertCartToJoint(const PoseEuler &pose, int user_frame_id, int tool_frame_id, Joint &joint)
{
    if (user_frame_id == user_frame_id_ && tool_frame_id == tool_frame_id_)
    {
        return group_ptr_->getKinematicsPtr()->doIK(pose, group_ptr_->getLatestJoint(), joint);     // transform from user to base
    }
    else
    {
        //Matrix tf, uf;

        if (user_frame_id == 0)
        {
            //uf.eye();
        }
        else
        {
            CoordInfo uf_info;
            ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

            if (err_user == SUCCESS && uf_info.is_valid)
            {
                //uf = uf_info.data;
            }
            else
            {
                FST_ERROR("Fail to get user frame from given ID.");
                return err_user;
            }
        }

        if (tool_frame_id == 0)
        {
            //tf.eye();
        }
        else
        {
            ToolInfo tf_info;
            ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

            if (err_tool == SUCCESS && tf_info.is_valid)
            {
                //tf = tf_info.data;
            }
            else
            {
                FST_ERROR("Fail to get tool frame from given id");
                return err_tool;
            }
        }

       // return group_ptr_->getKinematicsPtr()->inverseKinematics(pose, uf, tf, group_ptr_->getLatestJoint(), joint);
       return group_ptr_->getKinematicsPtr()->doIK(pose, group_ptr_->getLatestJoint(), joint);  // transform uf tf to base
    }
}

ErrorCode MotionControl::convertJointToCart(const Joint &joint, int user_frame_id, int tool_frame_id, PoseEuler &pose)
{
    if (user_frame_id == user_frame_id_ && tool_frame_id == tool_frame_id_)
    {
        group_ptr_->getKinematicsPtr()->doFK(joint, pose);  // transform from base to user
        return SUCCESS;
    }
    else
    {
        //Matrix tf, uf;

        if (user_frame_id == 0)
        {
            //uf.eye();
        }
        else
        {
            CoordInfo uf_info;
            ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

            if (err_user == SUCCESS && uf_info.is_valid)
            {
                //uf = uf_info.data;
            }
            else
            {
                FST_ERROR("Fail to get user frame from given ID.");
                return err_user;
            }
        }

        if (tool_frame_id == 0)
        {
            //tf.eye();
        }
        else
        {
            ToolInfo tf_info;
            ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

            if (err_tool == SUCCESS && tf_info.is_valid)
            {
                //tf = tf_info.data;
            }
            else
            {
                FST_ERROR("Fail to get tool frame from given id");
                return err_tool;
            }
        }

        //group_ptr_->getKinematicsPtr()->forwardKinematics(joint, uf, tf, pose);
        group_ptr_->getKinematicsPtr()->doFK(joint, pose);      // transform base to uf tf
        return SUCCESS;
    }
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
    PoseEuler pose;
    group_ptr_->getKinematicsPtr()->doFK(group_ptr_->getLatestJoint(), pose);   // transform from base to user
    return pose;
}

void MotionControl::getCurrentPose(PoseEuler &pose)
{
    group_ptr_->getKinematicsPtr()->doFK(group_ptr_->getLatestJoint(), pose);   // transform from base to user
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
            //Matrix tf;
            ErrorCode err = SUCCESS;
            // TODO
            // ErrorCode err = group_ptr_->getKinematicsPtr()->setToolFrame(tf.eye());

            if (err == SUCCESS)
            {
                tool_frame_id_ = id;
                FST_INFO("Success, current tool frame id is %d", tool_frame_id_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to set tool frame.");
                return err;
            }
        }
        else
        {
            ToolInfo  tf_info;
            ErrorCode err = tool_manager_ptr_->getToolInfoById(id, tf_info);

            if (err == SUCCESS && tf_info.is_valid)
            {
                // TODO
                // err = group_ptr_->getKinematicsPtr()->setToolFrame(tf_info.data);

                if (err == SUCCESS)
                {
                    tool_frame_id_ = id;
                    FST_INFO("Success, current tool frame id is %d", tool_frame_id_);
                    return SUCCESS;
                }
                else
                {
                    FST_ERROR("Fail to set tool frame.");
                    return err;
                }
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
            //Matrix uf;
            // TODO
            // ErrorCode err = group_ptr_->getKinematicsPtr()->setUserFrame(uf.eye());
            ErrorCode err = SUCCESS;

            if (err == SUCCESS)
            {
                user_frame_id_ = id;
                FST_INFO("Success, current user frame id is %d", user_frame_id_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to set user frame.");
                return err;
            }
        }
        else
        {
            CoordInfo uf_info;
            ErrorCode err = coordinate_manager_ptr_->getCoordInfoById(id, uf_info);

            if (err == SUCCESS && uf_info.is_valid)
            {
                // TODO
                // err = group_ptr_->getKinematicsPtr()->setUserFrame(uf_info.data);

                if (err == SUCCESS)
                {
                    user_frame_id_ = id;
                    FST_INFO("Success, current user frame ID switch to %d", user_frame_id_);
                    return SUCCESS;
                }
                else
                {
                    FST_ERROR("Fail to set user frame.");
                    return err;
                }
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



