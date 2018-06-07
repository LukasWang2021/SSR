#include "manual_motion.h"
#include "error_monitor.h"
#include "error_code.h"

/* qianjin */
using std::vector;

ManualMotion::ManualMotion(Robot *robot, ArmGroup *arm_group):robot_(robot),arm_group_(arm_group)
{
    /* manu_state_ = IDLE_R; */
    ready_ = false;

    /* fengyun */
    vel_ratio_ = arm_group_->getGlobalVelRatio();
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
    if (command.has_velocity)
    {
        if ((0 <= command.velocity) && (command.velocity <= 100))
        {
            vel_ratio_ = command.velocity;
            FST_INFO("vel_ratio_:%f", vel_ratio_);
    
            /* fengyun
            /* use global-velocity-ratio to
            /*    control manual speed according to GongShaoqiu
            /* global-vel-ratio should :
            /*    lower than 6.25% in limited manual mode
            /*    lower than 100% in unlimited manual mode
            */
            arm_group_->setGlobalVelRatio(vel_ratio_ / 100);
        }
        else
        {
            FST_INFO("Manual Motion : Param error from TP");
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            return;
        }
    }

    if (command.has_type)
    {
        if ((motion_spec_ManualType_STEP <= command.type) &&
            (command.type <= motion_spec_ManualType_APPOINT))
        {
            manu_type_ = command.type;
            arm_group_->setManualMode((ManualMode)manu_type_);
        }
        else
        {
            FST_ERROR("invalid type:%d", command.type);
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            return;
        }
    }

    if (command.has_frame)
    {
        if ((motion_spec_ManualFrame_JOINT <= command.frame) &&
            (command.frame <= motion_spec_ManualFrame_TOOL))
        {
            manu_frame_ = command.frame;
            arm_group_->setManualFrame((ManualFrame)manu_frame_);
        }
        else
        {
            FST_ERROR("invalid frame:%d", command.frame);
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            return;
        }
    }

    if((command.has_stepJoint))
    {
        if (manu_type_ == motion_spec_ManualType_STEP)
        {
            if(0 <= command.stepJoint)
                arm_group_->setManualJointStep(command.stepJoint);
            else
            {
                FST_ERROR("Manual Motion : Joint step is too small to set");
                rcs::Error::instance()->add(FALT_SET_STEP);
            }
        }
        else
        {
            FST_ERROR("Manual Motion : Param error from TP");
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
        }
    }

    if((command.has_stepPosition))
    {
        if (manu_type_ == motion_spec_ManualType_STEP)
        {
            if(0 <= command.stepPosition)
                arm_group_->setManualCartesianPositionStep(command.stepPosition);
            else
            {
                FST_ERROR("Manual Motion : Position step is too small to set");
                rcs::Error::instance()->add(FALT_SET_STEP);
            }
        }
        else
        {
            FST_ERROR("Manual Motion : Param error from TP");
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
        }
    }

    if((command.has_stepOrientation))
    {
        if (manu_type_ == motion_spec_ManualType_STEP)
        {
            if(0 <= command.stepOrientation)
                arm_group_->setManualCartesianPositionStep(command.stepOrientation);
            else
            {
                FST_ERROR("Manual Motion : Orientation step is too small to set");
                rcs::Error::instance()->add(FALT_SET_STEP);
            }
        }
        else
        {
            FST_ERROR("Manual Motion : Param error from TP");
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
        }
    }

    if (!command.has_target) return;

    motion_spec_TeachTarget *target = &command.target;

    if ((0 < target->directions_count) &&
        (0 < target->coordinates_count))
    {
        FST_ERROR("Manual Motion : Both directions and coordinates",
            " can not be set at the same time");
        rcs::Error::instance()->add(FALT_SET_TARGET);
        return;
    }

    if (0 < target->directions_count)
    {
        if ((manu_type_ != motion_spec_ManualType_STEP) &&
            (manu_type_ != motion_spec_ManualType_CONTINUE))
        {
            FST_ERROR("Manual Motion : SetManuCommand manu_type_",
                " is not the same as command : %d", manu_type_);
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            return;
        }
    }

    if(0 < target->coordinates_count)
    {
        if (manu_type_ != motion_spec_ManualType_APPOINT)
        {
            FST_ERROR("Manual Motion : APPOINT manu_type_ ",
                " is not the same as command : %d",
                manu_type_);
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            return;
        }
    }

    U64 result = TPI_SUCCESS;

    if (0 < target->directions_count)
    {
        vector<ManualDirection> button;
        ManualDirection dir;
        for (int i = 0; i < target->directions_count; i++)
        {
            dir = target->directions[i] == 0 ? STANDBY : 
                (target->directions[i] == 1 ? INCREASE : DECREASE);
            button.push_back(dir);
        }

        FST_INFO("type:%d, frame:%d", manu_type_, manu_frame_);
        result = arm_group_->manualMove(button);
    }

    if (0 < target->coordinates_count)
    {
        if (manu_frame_ == motion_spec_ManualFrame_JOINT)
        {
            Joint target_jnts ;
            target_jnts.j1 = target->coordinates[0];
            target_jnts.j2 = target->coordinates[1];
            target_jnts.j3 = target->coordinates[2];
            target_jnts.j4 = target->coordinates[3];
            target_jnts.j5 = target->coordinates[4];
            target_jnts.j6 = target->coordinates[5];
            result = arm_group_->manualMove(target_jnts);
        }
        else if ((manu_frame_ == motion_spec_ManualFrame_USER)
            || (manu_frame_ == motion_spec_ManualFrame_TOOL)
            || (manu_frame_ == motion_spec_ManualFrame_WORLD))
        {
            PoseEuler target_pose ;
            target_pose.position.x = target->coordinates[0];
            target_pose.position.y = target->coordinates[1];
            target_pose.position.z = target->coordinates[2];
            target_pose.orientation.a = target->coordinates[3];
            target_pose.orientation.b = target->coordinates[4];
            target_pose.orientation.c = target->coordinates[5];
            result = arm_group_->manualMove(target_pose);
        }
        else
        {
            FST_ERROR("Manual Motion : Param error from TP");
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            return;
        }
    }

    if (TPI_SUCCESS != result)
    {
        rcs::Error::instance()->add(result);
        return;
    }

    if (ready_ == false) ready_ = true;

    FST_INFO("");
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
        ManualDirection dir;
        for (int i = 0; i < target.directions_count; i++)
        {
            dir = target.directions[i] == 0 ? STANDBY : (target.directions[i] == 1 ? INCREASE : DECREASE);
            button.push_back(dir);
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


