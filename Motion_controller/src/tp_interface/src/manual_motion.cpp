#include "manual_motion.h"
#include "error_monitor.h"
#include "error_code.h"

//!!!!qj!!!!!
using std::vector;

ManualMotion::ManualMotion(Robot *robot, ArmGroup *arm_group):robot_(robot),arm_group_(arm_group)
{
   // manu_state_ = IDLE_R;
    ready_ = false;
    vel_ratio_ = arm_group_->getManualMaxSpeedRatio();
}

ManualMotion::~ManualMotion()
{

}

bool ManualMotion::isReady()
{
    if (ready_)
    {
        ready_ = false;
        return true;
    }
    return false;
}

/*ProgramState ManualMotion::getManuState()*/
//{
    //return manu_state_;
/*}*/

double ManualMotion::getVelRatio()
{
    return vel_ratio_;
}

motion_spec_ManualFrame& ManualMotion::getManuFrame()
{
    return manu_frame_;
}

motion_spec_ManualType& ManualMotion::getManuType()
{
    return manu_type_;
}

void ManualMotion::setManuCommand(motion_spec_ManualCommand command)
{
    step_counter_ = 0;
   // arm_group_->startManualTeach();     //ready to run manual pose


    if (command.has_velocity)
    {
        if ((command.velocity < 0) || (command.velocity > 100))
        {
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            return;
        }

        vel_ratio_ = command.velocity;
        FST_INFO("vel_ratio_:%f", vel_ratio_);
        arm_group_->setManualSpeedRatio(vel_ratio_/100);
    }
    //FST_INFO("has_type:%d, has_frame:%d", command.has_type, command.has_frame);
    if (command.has_type)
    {
        if ((command.type < motion_spec_ManualType_STEP)
        || (command.type > motion_spec_ManualType_APPOINT))
        {
            FST_ERROR("invalid type:%d", command.type);
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            return;
        }
        manu_type_ = command.type;
        //qianjin
        //arm_group_->setManualMotionMode((ManualMotionMode)manu_type_);
        arm_group_->setManualMode((ManualMode)manu_type_);
    }
    if (command.has_frame)
    {
        if ((command.frame < motion_spec_ManualFrame_JOINT)
        || (command.frame > motion_spec_ManualFrame_TOOL))
        {
            FST_ERROR("invalid frame:%d", command.frame);
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            return;
        }
        manu_frame_ = command.frame;
        //qianjin
        //arm_group_->setManualFrameMode((ManualFrameMode)manu_frame_);
        arm_group_->setManualFrame((ManualFrame)manu_frame_);
    }

    if (command.has_target)
    {
        motion_spec_TeachTarget *target = &command.target;
        if (target->directions_count > 0)
        {
            if ((manu_type_ != motion_spec_ManualType_STEP)
            && (manu_type_ != motion_spec_ManualType_CONTINUE))
            {
                FST_ERROR("manu_type_ is not the same as command:%d", manu_type_);
                rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
                return;
            }
            vector<ManualDirection> button;
            for (int i = 0; i < target->directions_count; i++)
            {
                button.push_back((ManualDirection)target->directions[i]);
            }
            FST_INFO("type:%d, frame:%d", manu_type_, manu_frame_);
            U64 result = arm_group_->manualMove(button);
            if (TPI_SUCCESS != result)
            {
                rcs::Error::instance()->add(result);
                return;
            }

        }
        else if (target->coordinates_count > 0)
        {
            U64 result = TPI_SUCCESS;
            if (manu_type_ != motion_spec_ManualType_APPOINT)
            {
                FST_ERROR("manu_type_ is not the same as command:%d", manu_type_);
                rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
                return;
            }
            if (manu_frame_ == motion_spec_ManualFrame_JOINT)
            {
                Joint target_jnts = *(Joint*)target->coordinates;
                result = arm_group_->manualMove(target_jnts);
            }
            else if (manu_frame_ == motion_spec_ManualFrame_USER)
            {
                //robot_->getJointFromPose(*(PoseEuler*)target->coordinates, target_jnts);
                PoseEuler target_pose = *(PoseEuler*)target->coordinates;
                result = arm_group_->manualMove(target_pose);
            }
            else
            {
                rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
                return;
            }
            if (TPI_SUCCESS != result)
            {
                rcs::Error::instance()->add(result);
                return;
            }
        }

        if (ready_ == false)
            ready_ = true;
    }
}

void ManualMotion::setTeachTarget(motion_spec_TeachTarget target)
{
    if (target.directions_count > 0)
    {
        if (manu_type_ != motion_spec_ManualType_CONTINUE)
        {
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            return;
        }
        
        step_counter_ = 0;

        bool tmp = 0;
        for (int i = 0; i < target.directions_count; i++)
        {
            tmp |= target.directions[i];
        }
        if (tmp == 0) //pause
        {
            pause();
            FST_INFO("================");
        }
        vector<ManualDirection> button;
        for (int i = 0; i < target.directions_count; i++)
        {
            button.push_back((ManualDirection)target.directions[i]);
        }
        arm_group_->manualMove(button);
    }
}

void ManualMotion::pause()
{
    step_counter_ = 0;
}

bool ManualMotion::stepCounter()
{
    static int timesout = MANUAL_CMD_TIMEOUT/STATE_MACHINE_INTERVAL;
    if (manu_frame_ == motion_spec_ManualType_CONTINUE)
    {
        step_counter_++;
        if (step_counter_ >= timesout)
        {
            step_counter_ = 0;            
            return true;
        }
    }
    return false;
}
