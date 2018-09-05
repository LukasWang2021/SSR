#include <string.h>
#include <motion_control.h>
#include <motion_control_arm_group.h>
#include "../../tool_manager/include/tool_manager.h"


using namespace fst_base;
using namespace basic_alg;
using namespace fst_mc;
using namespace fst_hal;
using namespace fst_ctrl;


static void rtTask(void *group)
{
    ((BaseGroup*)group)->realtimeTask();
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
}

MotionControl::~MotionControl()
{
    stopRealtimeTask();

    if (group_ptr_ != NULL)   {delete group_ptr_; group_ptr_ = NULL;};
    if (param_ptr_ != NULL)   {delete param_ptr_; param_ptr_ = NULL;};
    if (log_ptr_ != NULL)   {delete log_ptr_; log_ptr_ = NULL;};
}

ErrorCode MotionControl::init(fst_hal::DeviceManager* device_manager_ptr, AxisGroupManager* axis_group_manager_ptr,
                              fst_ctrl::CoordinateManager* coordinate_manager_ptr, fst_ctrl::ToolManager* tool_manager_ptr,
                              fst_base::ErrorMonitor *error_monitor_ptr)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new MotionControlParam();
    group_ptr_ = new ArmGroup(log_ptr_);
    //group_ptr_ = new ScalaGroup(log_ptr_);

    if (log_ptr_ == NULL || param_ptr_ == NULL || group_ptr_ == NULL)
    {
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

    ErrorCode  err = group_ptr_->initGroup(error_monitor_ptr);

    if (err == SUCCESS)
    {
        if (startRealtimeTask())
        {
            FST_INFO("Initialize motion group success.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail to create rt task.");
            return MOTION_INTERNAL_FAULT;
        }
    }
    else
    {
        FST_ERROR("Fail to init motion group");
        return err;
    }
}

bool MotionControl::startRealtimeTask(void)
{
    group_ptr_->activeRealtimeTask();
    return rt_thread_.run(&rtTask, group_ptr_, 80);
}

bool MotionControl::stopRealtimeTask(void)
{
    group_ptr_->inactiveRealtimeTask();
    rt_thread_.join();
    return true;
}



/*
MotionFrame MotionControl::getManualFrame(void)
{
    return group_ptr_->getManualFrame();
}

ErrorCode MotionControl::setManualFrame(MotionFrame frame)
{
    return group_ptr_->setManualFrame(frame);
}
*/

double MotionControl::getRotateManualStep(void)
{
    return group_ptr_->getManualStepAxis();
}

double MotionControl::getPrismaticManualStep(void)
{
    // TODO
    return 0;
}

double MotionControl::getPositionManualStep(void)
{
    return group_ptr_->getManualStepPosition();
}

double MotionControl::getOrientationManualStep(void)
{
    return group_ptr_->getManualStepOrientation();
}

ErrorCode MotionControl::setRotateManualStep(double step)
{
    return group_ptr_->setManualStepAxis(step);
}

ErrorCode MotionControl::setPrismaticManualStep(double step)
{
    // TODO
    return SUCCESS;
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
    return group_ptr_->manualMoveStep(&direction[0]);
}

ErrorCode MotionControl::doContinuousManualMove(const GroupDirection &direction)
{
    return group_ptr_->manualMoveContinuous(&direction[0]);
}

ErrorCode MotionControl::doGotoPointManualMove(const Joint &joint)
{
    return group_ptr_->manualMoveToPoint(joint);
}

ErrorCode MotionControl::doGotoPointManualMove(const PoseEuler &pose)
{
    return group_ptr_->manualMoveToPoint(pose);
}

ErrorCode MotionControl::manualStop(void)
{
    return group_ptr_->manualStop();
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

ErrorCode MotionControl::convertCartToJoint(const PoseEuler &pose, MotionFrame frame, int user_frame_id, int tool_frame_id, Joint &joint)
{
    ToolInfo  tf_info;
    ErrorCode err = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

    if (err == SUCCESS)
    {
        Matrix tf = tf_info.data;

        if (frame == BASE)
        {
            Matrix uf;
            uf.eye();
            return group_ptr_->getKinematicsPtr()->inverseKinematics(pose, uf, tf, group_ptr_->getLatestJoint(), joint);
        }
        else if (frame == USER)
        {
            CoordInfo uf_info;
            err = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

            if (err == SUCCESS)
            {
                Matrix uf = uf_info.data;
                return group_ptr_->getKinematicsPtr()->inverseKinematics(pose, uf, tf, group_ptr_->getLatestJoint(), joint);
            }
            else
            {
                FST_ERROR("Fail to get user frame from given id");
                return err;
            }
        }
        else if (frame == WORLD)
        {
            return group_ptr_->getKinematicsPtr()->inverseKinematics(pose, group_ptr_->getKinematicsPtr()->getWorldFrame(), tf, group_ptr_->getLatestJoint(), joint);
        }
        else
        {
            FST_ERROR("convertCartToJoint: invalid given frame: %d", frame);
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail to get tool frame from given id");
        return err;
    }
}

ErrorCode MotionControl::convertJointToCart(const Joint &joint, MotionFrame frame, int user_frame_id, int tool_frame_id, PoseEuler &pose)
{
    ToolInfo  tf_info;
    ErrorCode err = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

    if (err == SUCCESS)
    {
        Matrix tf = tf_info.data;

        if (frame == BASE)
        {
            Matrix uf;
            uf.eye();
            group_ptr_->getKinematicsPtr()->forwardKinematics(joint, uf, tf, pose);
            return SUCCESS;
        }
        else if (frame == USER)
        {
            CoordInfo uf_info;
            err = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

            if (err == SUCCESS)
            {
                Matrix uf = uf_info.data;
                group_ptr_->getKinematicsPtr()->forwardKinematics(joint, uf, tf, pose);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to get user frame from given id");
                return err;
            }
        }
        else if (frame == WORLD)
        {
            group_ptr_->getKinematicsPtr()->forwardKinematics(joint, group_ptr_->getKinematicsPtr()->getWorldFrame(), tf, pose);
            return SUCCESS;
        }
        else
        {
            FST_ERROR("convertCartToJoint: invalid given frame: %d", frame);
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail to get tool frame from given id");
        return err;
    }
}

GroupState MotionControl::getGroupState(void)
{
    return group_ptr_->getGroupState();
}

ServoState MotionControl::getServoState(void)
{
    return group_ptr_->getServoState();
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

void MotionControl::getToolFrameID(int &id)
{
    id = tool_frame_id_;
}

ErrorCode MotionControl::setToolFrameID(int id)
{
    FST_INFO("Set tool frame: id = %d, current is %d", id, tool_frame_id_);

    if (id != tool_frame_id_)
    {
        ErrorCode err;
        ToolInfo  tf_info;
        err = tool_manager_ptr_->getToolInfoById(id, tf_info);

        if (err == SUCCESS)
        {
            err = group_ptr_->setToolFrame(tf_info.data);

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
    else
    {
        FST_INFO("Success!");
        return SUCCESS;
    }
}

void MotionControl::getMotionFrameID(MotionFrame &frame, int &id)
{
    frame = group_ptr_->getMotionFrame();

    if (frame == USER)
    {
        id = user_frame_id_;
    }
    else if (frame == TOOL)
    {
        id = tool_frame_id_;
    }
    else
    {
        id = 0;
    }
}

ErrorCode MotionControl::setMotionFrameID(MotionFrame frame, int id)
{
    FST_INFO("Set motion frame: %d, id = %d", frame, id);
    ErrorCode err;

    if (frame != group_ptr_->getMotionFrame())
    {
        FST_INFO("Current frame: %d", group_ptr_->getMotionFrame());
        err = group_ptr_->setMotionFrame(frame);

        if (err == SUCCESS)
        {
            FST_INFO("Current frame switch to %d success.", group_ptr_->getMotionFrame());
        }
        else
        {
            FST_ERROR("Fail to set motion frame, code = 0x%llx", err);
            return err;
        }
    }

    if (group_ptr_->getMotionFrame() == USER)
    {
        FST_INFO("Current user frame ID = %d", user_frame_id_);

        if (id != user_frame_id_)
        {
            CoordInfo uf_info;
            err = coordinate_manager_ptr_->getCoordInfoById(id, uf_info);

            if (err == SUCCESS)
            {
                err = group_ptr_->setUserFrame(uf_info.data);

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
                FST_ERROR("Fail to get tool frame from given id");
                return err;
            }
        }
        else
        {
            FST_INFO("Success!");
            return SUCCESS;
        }
    }
    else if (frame == TOOL)
    {
        FST_INFO("Current tool frame ID = %d", tool_frame_id_);

        if (id != tool_frame_id_)
        {
            ToolInfo  tf_info;
            err = tool_manager_ptr_->getToolInfoById(id, tf_info);

            if (err == SUCCESS)
            {
                err = group_ptr_->setToolFrame(tf_info.data);

                if (err == SUCCESS)
                {
                    tool_frame_id_ = id;
                    FST_INFO("Success, current tool frame ID switch to %d", tool_frame_id_);
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
        else
        {
            FST_INFO("Success!");
            return SUCCESS;
        }
    }
    else
    {
        FST_INFO("Ignore frame ID, success!");
        return SUCCESS;
    }
}



