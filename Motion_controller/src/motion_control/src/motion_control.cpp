#include <string.h>
#include <motion_control.h>
#include <motion_control_arm_group.h>


using namespace fst_base;
using namespace fst_mc;
using namespace fst_hal;
using namespace fst_ctrl;


static void rtTask(void *group)
{
    ((BaseGroup*)group)->realtimeTask();
}


MotionControl::MotionControl():
        device_manager_ptr_(NULL), axis_group_manager_ptr_(NULL),
        coordinate_manager_ptr_(NULL), tool_manager_ptr_(NULL),
        error_monitor_ptr_(NULL),
        log_ptr_(NULL), param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    assert(log_ptr_ != NULL);
    param_ptr_ = new MotionControlParam();
    assert(param_ptr_ != NULL);
    group_ptr_ = new ArmGroup(log_ptr_);
    assert(group_ptr_ != NULL);
    //group_ptr_ = new ScalaGroup(log_ptr_);

}

MotionControl::~MotionControl()
{
    stopRealtimeTask();

    if (log_ptr_ != NULL)   {delete log_ptr_; log_ptr_ = NULL;};
    if (param_ptr_ != NULL)   {delete param_ptr_; param_ptr_ = NULL;};
    if (group_ptr_ != NULL)   {delete group_ptr_; group_ptr_ = NULL;};
}

ErrorCode MotionControl::init(fst_hal::DeviceManager* device_manager_ptr, AxisGroupManager* axis_group_manager_ptr,
                              fst_ctrl::CoordinateManager* coordinate_manager_ptr, fst_ctrl::ToolManager* tool_manager_ptr,
                              fst_base::ErrorMonitor *error_monitor_ptr)
{
    device_manager_ptr_ = device_manager_ptr;
    axis_group_manager_ptr_ = axis_group_manager_ptr;
    coordinate_manager_ptr_ = coordinate_manager_ptr;
    tool_manager_ptr_ = tool_manager_ptr;
    error_monitor_ptr_ = error_monitor_ptr;
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

ErrorCode MotionControl::setManualFrame(ManualFrame frame)
{
    return group_ptr_->setManualFrame(frame);
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
    return SUCCESS;
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



