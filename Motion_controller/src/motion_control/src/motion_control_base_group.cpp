/*************************************************************************
	> File Name: motion_control_base_group.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 11时25分56秒
 ************************************************************************/

#include <unistd.h>
#include <string.h>
#include <vector>
#include <fstream>
#include <time.h>

#include <motion_control_base_group.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <trajectory_alg.h>
#include <common_file_path.h>
#include <basic_alg.h>

using namespace std;
using namespace fst_base;
using namespace basic_alg;
using namespace fst_parameter;

//#define OUTPUT_JOUT
//#define OUTPUT_COEFF
//#define OUTPUT_POUT

namespace fst_mc
{
#ifdef OUTPUT_JOUT
#define JOUT_SIZE   (200 * 1000)

struct JointOut
{
    double time;
    Joint  ma_cv_g;
    JointPoint point;
};

size_t    g_jindex = 0;
JointOut  g_jout[JOUT_SIZE];
ofstream  jout("jout.csv");

#endif

#ifdef OUTPUT_COEFF
#define COEFF_SIZE   (100000)


enum CoeffOutputType
{
    FORE_CYCLE = 0,
    BACK_CYCLE = 1,
    SMOOTH_CYCLE = 2,
};

struct CoeffOut
{
    JointPoint start;
    JointPoint ending;
    Joint alpha_upper;
    Joint alpha_lower;
    double exp_duration;
    double duration;
    double start_time;
    CoeffOutputType type;
    TrajSegment segment[NUM_OF_JOINT];
};

size_t   g_cindex = 0;
CoeffOut g_cout[COEFF_SIZE];
ofstream coeff_out("cout.csv");

#endif

#ifdef OUTPUT_POUT
std::ofstream pout("pout.csv");
#endif

BaseGroup::BaseGroup(fst_log::Logger* plog)
{
    group_state_ = STANDBY;
    log_ptr_ = plog;
    auto_time_ = 0;
    manual_time_ = 0;
    manual_frame_ = JOINT;

    auto_pick_ptr_ = NULL;
    auto_cache_ptr_ = NULL;
    auto_pick_segment_ = 0;

    vel_ratio_ = 0;
    acc_ratio_ = 0;

    waiting_fine_ = false;
}

BaseGroup::~BaseGroup()
{
    if (auto_cache_ptr_ != NULL)
    {
        delete [] auto_cache_ptr_;
        auto_cache_ptr_ = NULL;
    }

#ifdef OUTPUT_JOUT
    jout << "time-of-start,angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],omega[0],omega[1],omega[2],omega[3],omega[4],omega[5],alpha[0],alpha[1],alpha[2],alpha[3],alpha[4],alpha[5]" << endl;

    for (size_t i = 0; i < g_jindex; i++)
    {
        jout << g_jout[i].time << ","
             << g_jout[i].point.angle[0] << "," << g_jout[i].point.angle[1] << "," << g_jout[i].point.angle[2] << "," << g_jout[i].point.angle[3] << "," << g_jout[i].point.angle[4] << "," << g_jout[i].point.angle[5] << ","
             << g_jout[i].point.omega[0] << "," << g_jout[i].point.omega[1] << "," << g_jout[i].point.omega[2] << "," << g_jout[i].point.omega[3] << "," << g_jout[i].point.omega[4] << "," << g_jout[i].point.omega[5] << ","
             << g_jout[i].point.alpha[0] << "," << g_jout[i].point.alpha[1] << "," << g_jout[i].point.alpha[2] << "," << g_jout[i].point.alpha[3] << "," << g_jout[i].point.alpha[4] << "," << g_jout[i].point.alpha[5] << endl;
    }

    jout.close();
#endif

#ifdef OUTPUT_COEFF
    coeff_out << "type,"
              << "start.angle[0],start.angle[1],start.angle[2],start.angle[3],start.angle[4],start.angle[5],start.omega[0],start.omega[1],start.omega[2],start.omega[3],start.omega[4],start.omega[5],start.alpha[0],start.alpha[1],start.alpha[2],start.alpha[3],start.alpha[4],start.alpha[5],"
              << "ending.angle[0],ending.angle[1],ending.angle[2],ending.angle[3],ending.angle[4],ending.angle[5],ending.omega[0],ending.omega[1],ending.omega[2],ending.omega[3],ending.omega[4],ending.omega[5],ending.alpha[0],ending.alpha[1],ending.alpha[2],ending.alpha[3],ending.alpha[4],ending.alpha[5],"
              << "alpha_upper[0],alpha_upper[1],alpha_upper[2],alpha_upper[3],alpha_upper[4],alpha_upper[5],alpha_lower[0],alpha_lower[1],alpha_lower[2],alpha_lower[3],alpha_lower[4],alpha_lower[5],"
              << "exp-duration,duration,start-time,"
              << "j0.duration[0],j0.duration[1],j0.duration[2],j0.duration[3],"
              << "j0.coeff[0][3],j0.coeff[0][2],j0.coeff[0][1],j0.coeff[0][0],j0.coeff[1][3],j0.coeff[1][2],j0.coeff[1][1],j0.coeff[1][0],j0.coeff[2][3],j0.coeff[2][2],j0.coeff[2][1],j0.coeff[2][0],j0.coeff[3][3],j0.coeff[3][2],j0.coeff[3][1],j0.coeff[3][0],"
              << "j1.duration[0],j1.duration[1],j1.duration[2],j1.duration[3],"
              << "j1.coeff[0][3],j1.coeff[0][2],j1.coeff[0][1],j1.coeff[0][0],j1.coeff[1][3],j1.coeff[1][2],j1.coeff[1][1],j1.coeff[1][0],j1.coeff[2][3],j1.coeff[2][2],j1.coeff[2][1],j1.coeff[2][0],j1.coeff[3][3],j1.coeff[3][2],j1.coeff[3][1],j1.coeff[3][0],"
              << "j2.duration[0],j2.duration[1],j2.duration[2],j2.duration[3],"
              << "j2.coeff[0][3],j2.coeff[0][2],j2.coeff[0][1],j2.coeff[0][0],j2.coeff[1][3],j2.coeff[1][2],j2.coeff[1][1],j2.coeff[1][0],j2.coeff[2][3],j2.coeff[2][2],j2.coeff[2][1],j2.coeff[2][0],j2.coeff[3][3],j2.coeff[3][2],j2.coeff[3][1],j2.coeff[3][0],"
              << "j3.duration[0],j3.duration[1],j3.duration[2],j3.duration[3],"
              << "j3.coeff[0][3],j3.coeff[0][2],j3.coeff[0][1],j3.coeff[0][0],j3.coeff[1][3],j3.coeff[1][2],j3.coeff[1][1],j3.coeff[1][0],j3.coeff[2][3],j3.coeff[2][2],j3.coeff[2][1],j3.coeff[2][0],j3.coeff[3][3],j3.coeff[3][2],j3.coeff[3][1],j3.coeff[3][0],"
              << "j4.duration[0],j4.duration[1],j4.duration[2],j4.duration[3],"
              << "j4.coeff[0][3],j4.coeff[0][2],j4.coeff[0][1],j4.coeff[0][0],j4.coeff[1][3],j4.coeff[1][2],j4.coeff[1][1],j4.coeff[1][0],j4.coeff[2][3],j4.coeff[2][2],j4.coeff[2][1],j4.coeff[2][0],j4.coeff[3][3],j4.coeff[3][2],j4.coeff[3][1],j4.coeff[3][0],"
              << "j5.duration[0],j5.duration[1],j5.duration[2],j5.duration[3],"
              << "j5.coeff[0][3],j5.coeff[0][2],j5.coeff[0][1],j5.coeff[0][0],j5.coeff[1][3],j5.coeff[1][2],j5.coeff[1][1],j5.coeff[1][0],j5.coeff[2][3],j5.coeff[2][2],j5.coeff[2][1],j5.coeff[2][0],j5.coeff[3][3],j5.coeff[3][2],j5.coeff[3][1],j5.coeff[3][0],"
              << endl;
    for (size_t i = 0; i < g_cindex; i++)
    {
        coeff_out << g_cout[i].type << ","
                  << g_cout[i].start.angle[0] << "," << g_cout[i].start.angle[1] << "," << g_cout[i].start.angle[2] << "," << g_cout[i].start.angle[3] << "," << g_cout[i].start.angle[4] << "," << g_cout[i].start.angle[5] << ","
                  << g_cout[i].start.omega[0] << "," << g_cout[i].start.omega[1] << "," << g_cout[i].start.omega[2] << "," << g_cout[i].start.omega[3] << "," << g_cout[i].start.omega[4] << "," << g_cout[i].start.omega[5] << ","
                  << g_cout[i].start.alpha[0] << "," << g_cout[i].start.alpha[1] << "," << g_cout[i].start.alpha[2] << "," << g_cout[i].start.alpha[3] << "," << g_cout[i].start.alpha[4] << "," << g_cout[i].start.alpha[5] << ","
                  << g_cout[i].ending.angle[0] << "," << g_cout[i].ending.angle[1] << "," << g_cout[i].ending.angle[2] << "," << g_cout[i].ending.angle[3] << "," << g_cout[i].ending.angle[4] << "," << g_cout[i].ending.angle[5] << ","
                  << g_cout[i].ending.omega[0] << "," << g_cout[i].ending.omega[1] << "," << g_cout[i].ending.omega[2] << "," << g_cout[i].ending.omega[3] << "," << g_cout[i].ending.omega[4] << "," << g_cout[i].ending.omega[5] << ","
                  << g_cout[i].ending.alpha[0] << "," << g_cout[i].ending.alpha[1] << "," << g_cout[i].ending.alpha[2] << "," << g_cout[i].ending.alpha[3] << "," << g_cout[i].ending.alpha[4] << "," << g_cout[i].ending.alpha[5] << ","
                  << g_cout[i].alpha_upper[0] << "," << g_cout[i].alpha_upper[1] << "," << g_cout[i].alpha_upper[2] << "," << g_cout[i].alpha_upper[3] << "," << g_cout[i].alpha_upper[4] << "," << g_cout[i].alpha_upper[5] << ","
                  << g_cout[i].alpha_lower[0] << "," << g_cout[i].alpha_lower[1] << "," << g_cout[i].alpha_lower[2] << "," << g_cout[i].alpha_lower[3] << "," << g_cout[i].alpha_lower[4] << "," << g_cout[i].alpha_lower[5] << ","
                  << g_cout[i].exp_duration << "," << g_cout[i].duration << "," << g_cout[i].start_time;

        for (size_t j = 0; j < 6; j++)
        {
            coeff_out << g_cout[i].segment[j].duration[0] << "," << g_cout[i].segment[j].duration[1] << "," << g_cout[i].segment[j].duration[2] << "," << g_cout[i].segment[j].duration[3] << ","
                      << g_cout[i].segment[j].coeff[0][3] << "," << g_cout[i].segment[j].coeff[0][2] << "," << g_cout[i].segment[j].coeff[0][1] << "," << g_cout[i].segment[j].coeff[0][0] << ","
                      << g_cout[i].segment[j].coeff[1][3] << "," << g_cout[i].segment[j].coeff[1][2] << "," << g_cout[i].segment[j].coeff[1][1] << "," << g_cout[i].segment[j].coeff[1][0] << ","
                      << g_cout[i].segment[j].coeff[2][3] << "," << g_cout[i].segment[j].coeff[2][2] << "," << g_cout[i].segment[j].coeff[2][1] << "," << g_cout[i].segment[j].coeff[2][0] << ","
                      << g_cout[i].segment[j].coeff[3][3] << "," << g_cout[i].segment[j].coeff[3][2] << "," << g_cout[i].segment[j].coeff[3][1] << "," << g_cout[i].segment[j].coeff[3][0];
        }

        coeff_out << endl;
    }
#endif

#ifdef OUTPUT_POUT
    pout.close();
#endif
}

void BaseGroup::reportError(const ErrorCode &error)
{
    error_monitor_ptr_->add(error);
}

ErrorCode BaseGroup::resetGroup(void)
{
    return bare_core_.resetBareCore() == true ? SUCCESS : BARE_CORE_TIMEOUT;
}

ErrorCode BaseGroup::stopGroup(void)
{
    if (group_state_ == MANUAL || (group_state_ & 0xF) == MANUAL || ((group_state_ >> 4) & 0xF) == MANUAL)
    {
        group_state_ = STANDBY;
        manual_time_ = 0;
        memset(&manual_traj_, 0, sizeof(manual_traj_));

        if (!bare_core_.isPointCacheEmpty())
        {
            bare_core_.clearPointCache();
        }
    }

    return bare_core_.stopBareCore() == true ? SUCCESS : BARE_CORE_TIMEOUT;
}

ErrorCode BaseGroup::clearGroup(void)
{
    FST_INFO("Clear group, current group state = %d", group_state_);

    group_state_ = STANDBY;
    manual_time_ = 0;
    memset(&manual_traj_, 0, sizeof(manual_traj_));

    if (!bare_core_.isPointCacheEmpty())
    {
        bare_core_.clearPointCache();
    }

    return SUCCESS;
}



ManualFrame BaseGroup::getManualFrame(void)
{
    //FST_INFO("Get manual frame = %d", manual_frame_);
    return manual_frame_;
}

ErrorCode BaseGroup::setManualFrame(ManualFrame frame)
{
    FST_INFO("Set manual frame = %d, current frame is %d", frame, manual_frame_);

    if (frame != manual_frame_)
    {
        if (group_state_ == STANDBY)
        {
            manual_frame_ = frame;
            FST_INFO("Done.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Cannot set frame in current state = %d", group_state_);
            return INVALID_SEQUENCE;
        }
    }
    else
    {
        return SUCCESS;
    }
}

double BaseGroup::getManualStepAxis(void)
{
    double step = manual_teach_.getManualStepAxis();
    FST_INFO("Get manual step axis = %.6f", step);
    return step;
}

double BaseGroup::getManualStepPosition(void)
{
    double step = manual_teach_.getManualStepPosition();
    FST_INFO("Get manual step position = %.4f", step);
    return step;
}

double BaseGroup::getManualStepOrientation(void)
{
    double step = manual_teach_.getManualStepOrientation();
    FST_INFO("Get manual step orientation = %.6f", step);
    return step;
}

ErrorCode BaseGroup::setManualStepAxis(double step)
{
    FST_INFO("Set manual step axis = %.6f", step);
    return manual_teach_.setManualStepAxis(step);
}

ErrorCode BaseGroup::setManualStepPosition(double step)
{
    FST_INFO("Set manual step position = %.4f", step);
    return manual_teach_.setManualStepPosition(step);
}

ErrorCode BaseGroup::setManualStepOrientation(double step)
{
    FST_INFO("Set manual step orientation = %.6f", step);
    return manual_teach_.setManualStepOrientation(step);
}

ErrorCode BaseGroup::manualMoveToPoint(const Joint &joint)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual to target joint: %s", printDBLine(&joint[0], buffer, LOG_TEXT_SIZE));

    if (group_state_ != STANDBY || servo_state_ != SERVO_IDLE)
    {
        FST_ERROR("Cannot manual to target in current group-state = %d, servo-state = %d", group_state_, servo_state_);
        return INVALID_SEQUENCE;
    }

    if (manual_frame_ != JOINT)
    {
        FST_ERROR("Cannot manual to target in current frame = %d", manual_frame_);
        return INVALID_SEQUENCE;
    }

    getLatestJoint(manual_traj_.joint_start);
    FST_ERROR("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        FST_ERROR("start-joint is out of soft constraint, manual-mode-apoint is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (!soft_constraint_.isJointInConstraint(joint))
    {
        FST_ERROR("target-joint out of constraint: %s", printDBLine(&joint[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    manual_time_ = 0;
    manual_traj_.mode = APOINT;
    manual_traj_.frame = JOINT;
    ErrorCode err = manual_teach_.manualByTarget(joint, manual_time_, manual_traj_);

    if (err == SUCCESS)
    {
        group_state_ = MANUAL;
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        return err;
    }
}

ErrorCode BaseGroup::manualMoveToPoint(const PoseEuler &pose)
{
    ErrorCode err;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual to target pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f",
             pose.position.x, pose.position.y, pose.position.z, pose.orientation.a, pose.orientation.b, pose.orientation.c);

    if (group_state_ != STANDBY || servo_state_ != SERVO_IDLE)
    {
        FST_ERROR("Cannot manual to target in current group-state = %d, servo-state = %d", group_state_, servo_state_);
        return INVALID_SEQUENCE;
    }

    if (manual_frame_ != BASE && manual_frame_ != USER && manual_frame_ != WORLD)
    {
        FST_ERROR("Cannot manual to target in current frame = %d", manual_frame_);
        return INVALID_SEQUENCE;
    }

    getLatestJoint(manual_traj_.joint_start);
    FST_ERROR("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        FST_ERROR("start-joint is out of soft constraint, manual-mode-apoint is disabled.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    Joint res_joint;
    Joint ref_joint = getLatestJoint();

    switch (manual_frame_)
    {
        case BASE:
            err = kinematics_ptr_->inverseKinematicsInBase(pose, ref_joint, res_joint);
            break;
        case USER:
            err = kinematics_ptr_->inverseKinematicsInUser(pose, ref_joint, res_joint);
            break;
        case WORLD:
            err = kinematics_ptr_->inverseKinematicsInWorld(pose, ref_joint, res_joint);
            break;
        default:
            FST_ERROR("Invalid manual frame = %d in manual to pose mode", manual_frame_);
            return MOTION_INTERNAL_FAULT;
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to get inverse kinematics of given target, code = 0x%llx", err);
        return err;
    }

    if (!soft_constraint_.isJointInConstraint(res_joint))
    {
        FST_ERROR("target-joint is out of soft constraint: %s", printDBLine(&res_joint[0], buffer, LOG_TEXT_SIZE));
        return JOINT_OUT_OF_CONSTRAINT;
    }

    manual_time_ = 0;
    manual_traj_.mode = APOINT;
    manual_traj_.frame = manual_frame_;
    err = manual_teach_.manualByTarget(res_joint, manual_time_, manual_traj_);

    if (err == SUCCESS)
    {
        group_state_ = MANUAL;
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        return err;
    }
}

ErrorCode BaseGroup::manualMoveStep(const ManualDirection *direction)
{
    PoseEuler pose;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual step frame=%d by direction.", manual_frame_);

    if (group_state_ != STANDBY || servo_state_ != SERVO_IDLE)
    {
        FST_ERROR("Cannot manual step in current group-state = %d, servo-state = %d", group_state_, servo_state_);
        return INVALID_SEQUENCE;
    }

    getLatestJoint(manual_traj_.joint_start);
    FST_INFO("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
    {
        if (manual_frame_ == JOINT)
        {
            for (size_t i = 0; i < getNumberOfJoint(); i++)
            {
                if (manual_traj_.joint_start[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
                {
                    FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                              i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                    return INVALID_PARAMETER;
                }
                else if (manual_traj_.joint_start[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
                {
                    FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                              i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                    return INVALID_PARAMETER;
                }
            }
        }
        else
        {
            FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
            return INVALID_SEQUENCE;
        }
    }

    switch (manual_frame_)
    {
        case JOINT:
            break;
        case BASE:
            kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, manual_traj_.cart_start);
        case USER:
            kinematics_ptr_->forwardKinematicsInUser(manual_traj_.joint_start, manual_traj_.cart_start);
            break;
        case WORLD:
            kinematics_ptr_->forwardKinematicsInWorld(manual_traj_.joint_start, manual_traj_.cart_start);
            break;
        case TOOL:
            kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, pose);
            manual_traj_.tool_coordinate = pose;
            memset(&manual_traj_.cart_start, 0, sizeof(manual_traj_.cart_start));
            break;
        default:
            FST_ERROR("Unsupported manual frame: %d", manual_frame_);
            return MOTION_INTERNAL_FAULT;
    }

    manual_time_ = 0;
    manual_traj_.mode = STEP;
    manual_traj_.frame = manual_frame_;
    ErrorCode err = manual_teach_.manualStepByDirect(direction, manual_time_, manual_traj_);

    if (err == SUCCESS)
    {
        group_state_ = MANUAL;
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
        memset(&manual_traj_, 0, sizeof(ManualTrajectory));
        return err;
    }
}

ErrorCode BaseGroup::manualMoveContinuous(const ManualDirection *direction)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual continuous frame=%d by direction.", manual_frame_);

    if (group_state_ != STANDBY && group_state_ != MANUAL)
    {
        FST_ERROR("Cannot manual continuous in current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }

    if (group_state_ == STANDBY)
    {
        if (servo_state_ != SERVO_IDLE)
        {
            FST_ERROR("Cannot manual continuous in current servo-state = %d", servo_state_);
        }

        PoseEuler pose;
        getLatestJoint(manual_traj_.joint_start);
        FST_INFO("start-joint = %s", printDBLine(&manual_traj_.joint_start[0], buffer, LOG_TEXT_SIZE));

        if (!soft_constraint_.isJointInConstraint(manual_traj_.joint_start))
        {
            if (manual_frame_ == JOINT)
            {
                for (size_t i = 0; i < getNumberOfJoint(); i++)
                {
                    if (manual_traj_.joint_start[i] > soft_constraint_.upper()[i] + MINIMUM_E9 && direction[i] == INCREASE)
                    {
                        FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (increase).",
                                  i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                        return INVALID_PARAMETER;
                    }
                    else if (manual_traj_.joint_start[i] < soft_constraint_.lower()[i] - MINIMUM_E9 && direction[i] == DECREASE)
                    {
                        FST_ERROR("J%d = %.4f out of range [%.4f, %.4f], cannot move as given direction (decrease).",
                                  i + 1, manual_traj_.joint_start[i], soft_constraint_.lower()[i], soft_constraint_.upper()[i]);
                        return INVALID_PARAMETER;
                    }
                }
            }
            else
            {
                FST_ERROR("start-joint is out of soft constraint, manual-frame-cartesian is disabled.");
                return INVALID_SEQUENCE;
            }
        }

        switch (manual_frame_)
        {
            case JOINT:
                break;
            case BASE:
                kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, manual_traj_.cart_start);
            case USER:
                kinematics_ptr_->forwardKinematicsInUser(manual_traj_.joint_start, manual_traj_.cart_start);
                break;
            case WORLD:
                kinematics_ptr_->forwardKinematicsInWorld(manual_traj_.joint_start, manual_traj_.cart_start);
                break;
            case TOOL:
                kinematics_ptr_->forwardKinematicsInBase(manual_traj_.joint_start, pose);
                manual_traj_.tool_coordinate = pose;
                memset(&manual_traj_.cart_start, 0, sizeof(manual_traj_.cart_start));
                break;
            default:
                FST_ERROR("Unsupported manual frame: %d", manual_frame_);
                return MOTION_INTERNAL_FAULT;
        }

        manual_time_ = 0;
        manual_traj_.mode = CONTINUOUS;
        manual_traj_.frame = manual_frame_;
        ErrorCode err = manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);

        if (err == SUCCESS)
        {
            group_state_ = MANUAL;
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail to create manual trajectory, error-code = 0x%llx", err);
            memset(&manual_traj_, 0, sizeof(ManualTrajectory));
            return err;
        }
    }
    else if (group_state_ == MANUAL)
    {
        for (size_t i = 0; i < getNumberOfJoint(); i++)
        {
            if (manual_traj_.direction[i] != direction[i])
            {
                return manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);
            }
            else
            {
                continue;
            }
        }

        FST_INFO("Given directions same as current motion, do not need replan.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Cannot manual now, current state = %d", group_state_);
        return INVALID_SEQUENCE;
    }
}

ErrorCode BaseGroup::manualStop(void)
{
    FST_INFO("Manual stop request received.");

    if (group_state_ == MANUAL)
    {
        FST_INFO("Manual mode = %d, frame = %d", manual_traj_.mode, manual_traj_.frame);

        if (manual_traj_.mode == CONTINUOUS)
        {
            ManualDirection direction[NUM_OF_JOINT] = {STANDING};
            ErrorCode err = manual_teach_.manualContinuousByDirect(direction, manual_time_, manual_traj_);

            if (err == SUCCESS)
            {
                FST_INFO("Success, the group will stop in %.4fs", manual_traj_.duration - manual_time_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to stop current manual motion, error-code = 0x%llx", err);
                return err;
            }
        }
        else if (manual_traj_.mode == APOINT)
        {
            ErrorCode err = manual_teach_.manualStop(manual_time_, manual_traj_);

            if (err == SUCCESS)
            {
                FST_INFO("Success, the group will stop in %.4fs", manual_traj_.duration - manual_time_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to stop current manual motion, error-code = 0x%llx", err);
                return err;
            }
        }
        else
        {
            FST_INFO("Cannot stop manual motion in step mode, the group will stop in %.4fs", manual_traj_.duration - manual_time_);
            return INVALID_SEQUENCE;
        }
    }
    else
    {
        FST_INFO("The group is not in manual state, current-state = %d", group_state_);
        return SUCCESS;
    }
}


ErrorCode BaseGroup::autoMove(int id, const MotionTarget &target)
{
    FST_INFO("Auto move request received, motion ID = %d, type = %d", id, target.type);
    ErrorCode err;

    switch (target.type)
    {
        case MOTION_JOINT:
            err = autoJoint(target.joint_target, target.vel, target.cnt, id);
            break;
        case MOTION_LINE:
            err = autoLine(target.pose_target, target.vel, target.cnt, id);
            break;
        case MOTION_CIRCLE:
            //err = autoCircle(target.circle_target.pose1, target.circle_target.pose2, target.vel, target.cnt, id);
            //break;
        default:
            FST_ERROR("Invalid motion type, auto move aborted.");
            err = INVALID_PARAMETER;
            break;
    }

    if (err == SUCCESS)
    {
        //if (group_state_ == STANDBY)
        pthread_mutex_lock(&auto_mutex_);
        FST_INFO("grp-state=%d, servo-state=%d, auto-ptr-valid = %d", group_state_, servo_state_, auto_pick_ptr_->valid);

        if (auto_pick_ptr_->valid == false)
        {
            size_t cnt = 0;

            while (!auto_pick_ptr_->valid && cnt < AUTO_CACHE_SIZE)
            {
                auto_pick_ptr_ = auto_pick_ptr_->next;
            }

            if (auto_pick_ptr_->valid)
            {
                auto_pick_segment_ = 1;
                FST_INFO("Start motion, pick-cache = %p, pick-seg = %d", auto_pick_ptr_, auto_pick_segment_);
            }
            else
            {
                FST_INFO("Trajectory cache not ready.");
                pthread_mutex_unlock(&auto_mutex_);
                return MOTION_INTERNAL_FAULT;
            }
        }

        if (group_state_ == STANDBY)
        {
            auto_time_ = 0;
            group_state_ = AUTO;
        }

        FST_INFO("Auto pick-time = %.4f", auto_time_);

        if (target.cnt < 0)
        {
            FST_INFO("Move with 'FINE', group will waiting for sand stable");
            // move by 'FINE'
            if (target.type == MOTION_JOINT)
            {
                waiting_motion_type_ = MOTION_JOINT;
                waiting_joint_ = target.joint_target;
                start_waiting_cnt_ = 20;
                waiting_fine_ = true;
            }
            else if (target.type == MOTION_LINE)
            {
                waiting_motion_type_ = MOTION_LINE;
                waiting_pose_ = target.pose_target;
                start_waiting_cnt_ = 20;
                waiting_fine_ = true;
            }
            else
            {
                FST_ERROR("Move by 'FINE', motion type = %d invalid", target.type);
                pthread_mutex_unlock(&auto_mutex_);
                return MOTION_INTERNAL_FAULT;
            }
        }

        FST_INFO("autoMove: success!");
        pthread_mutex_unlock(&auto_mutex_);
        return SUCCESS;
    }
    else if (err == 0xA000B000C000D000)
    {
        return SUCCESS;
    }
    else
    {
        FST_ERROR("autoMove: failed, code = 0x%llx", err);
        return err;
    }
}

ErrorCode BaseGroup::autoJoint(const Joint &target, double vel, double cnt, int id)
{
    char buffer[LOG_TEXT_SIZE];
    Joint start_joint;
    FST_INFO("autoJoint: ID = %d, vel = %.4f, cnt = %.4f", id, vel, cnt);
    FST_INFO("  target = %s", printDBLine(&target[0], buffer, LOG_TEXT_SIZE));

    if (vel < MINIMUM_E6 || vel > 1 + MINIMUM_E6 || fabs(cnt) > 1 + MINIMUM_E6)
    {
        FST_ERROR("Invalid vel or CNT.");
        return INVALID_PARAMETER;
    }

    if (!soft_constraint_.isJointInConstraint(target))
    {
        FST_ERROR("Target out of soft constraint.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (group_state_ == STANDBY && servo_state_ == SERVO_IDLE)
    {
        getLatestJoint(start_joint);
    }
    else if (group_state_ == AUTO || group_state_ == STANDBY || group_state_ == AUTO_TO_STANDBY)
    {
        start_joint = start_joint_;
    }
    else
    {
        FST_ERROR("Cannot start auto motion in current group state: %d", group_state_);
        return INVALID_SEQUENCE;
    }

    FST_INFO("  start  = %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(start_joint))
    {
        FST_ERROR("Start joint out of soft constraint.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (isSameJoint(start_joint, target))
    {
        FST_WARN("Start joint same as target joint");
        return 0xA000B000C000D000;
    }

    ErrorCode err;
    size_t  index, length;
    double  precision = 0.01;
    Joint   path[MAX_PATH_SIZE];
    planJointPath(start_joint, target, precision, index, path, length);
    //TrajectoryCache *p_cache = auto_pick_ptr_->valid == false ? auto_pick_ptr_ : auto_pick_ptr_->next;
    TrajectoryCache *p_cache = auto_pick_ptr_->next;

    for (size_t i = 0; i < length; i++)
    {
        p_cache->cache[i].path_point.id     = id;
        p_cache->cache[i].path_point.stamp  = i;
        p_cache->cache[i].path_point.type   = MOTION_JOINT;
        p_cache->cache[i].path_point.joint  = path[i];
        p_cache->cache[i].time_from_start   = -1;
        //p_cache->cache[i].command_duration  = expect_duration;
        p_cache->cache[i].forward_duration  = -1;
        p_cache->cache[i].backward_duration = -1;
    }

    p_cache->head = 0;
    p_cache->tail = length;
    p_cache->expect_duration = precision / (axis_vel_[index] * vel * vel_ratio_);
    p_cache->deadline = 999999999.999;

    //FST_WARN("precision = %.4f, omega-limit = %.4f, vel = %.4f, ratio = %.4f => duration = %.4f", precision, axis_vel_[index], vel, vel_ratio_, p_cache->expect_duration);

    FST_INFO("Prepare trajectory cache ...");
    err = prepareCache(*p_cache);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to prepare trajectory, code = 0x%llx", err);
        return err;
    }

    FST_INFO("Preplan trajectory cache ...");
    err = preplanCache(*p_cache, cnt);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to preplan trajectory, code = 0x%llx", err);
        return err;
    }

    p_cache->valid = true;
    start_joint_ = p_cache->cache[p_cache->tail - 1].ending_state.angle;
    FST_INFO("Success.");
    return SUCCESS;
}

ErrorCode BaseGroup::autoLine(const PoseEuler &target, double vel, double cnt, int id)
{
    char buffer[LOG_TEXT_SIZE];
    Joint start_joint;
    FST_INFO("autoLine: ID = %d, vel = %.4f, cnt = %.4f", id, vel, cnt);
    FST_INFO("  target pose = %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", target.position.x, target.position.y, target.position.z, target.orientation.a, target.orientation.b, target.orientation.c);

    if (vel < MINIMUM_E3 || vel > 4000 + MINIMUM_E3 || fabs(cnt) > 1 + MINIMUM_E6)
    {
        FST_ERROR("Invalid vel or CNT.");
        return INVALID_PARAMETER;
    }

    if (group_state_ == STANDBY && servo_state_ == SERVO_IDLE)
    {
        getLatestJoint(start_joint);
    }
    else if (group_state_ == AUTO || group_state_ == STANDBY || group_state_ == AUTO_TO_STANDBY)
    {
        start_joint = start_joint_;
    }
    else
    {
        FST_ERROR("Cannot start auto motion in current group state: %d", group_state_);
        return INVALID_SEQUENCE;
    }

    FST_INFO("  start joint = %s", printDBLine(&start_joint[0], buffer, LOG_TEXT_SIZE));

    if (!soft_constraint_.isJointInConstraint(start_joint))
    {
        FST_ERROR("Start joint out of soft constraint.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    ErrorCode err;
    PoseEuler start;
    size_t  length;
    double  precision = vel < 1000 ? 1 : vel / 1000;     // vel < 1000mm/s -> 1mm, 1500mm/s -> 1.5mm, 2000mm/s -> 2mm
    Pose    path[MAX_PATH_SIZE];

    kinematics_ptr_->forwardKinematicsInUser(start_joint, start);
    FST_INFO("  start pose  = %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", start.position.x, start.position.y, start.position.z, start.orientation.a, start.orientation.b, start.orientation.c);
    planLinePath(start, target, precision, path, length);
    //TrajectoryCache *p_cache = auto_pick_ptr_->valid == false ? auto_pick_ptr_ : auto_pick_ptr_->next;
    TrajectoryCache *p_cache = auto_pick_ptr_->next;

    for (size_t i = 0; i < length; i++)
    {
        p_cache->cache[i].path_point.id     = id;
        p_cache->cache[i].path_point.stamp  = i;
        p_cache->cache[i].path_point.type   = MOTION_LINE;
        p_cache->cache[i].path_point.pose   = path[i];
        p_cache->cache[i].time_from_start   = -1;
        //p_cache->cache[i].command_duration  = expect_duration;
        p_cache->cache[i].forward_duration  = -1;
        p_cache->cache[i].backward_duration = -1;
    }

    p_cache->head = 0;
    p_cache->tail = length;
    p_cache->expect_duration = precision / (vel * vel_ratio_);
    p_cache->deadline = 999999999.999;

    FST_INFO("Prepare trajectory cache ...");
    err = prepareCache(*p_cache);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to prepare trajectory, code = 0x%llx", err);
        return err;
    }

    FST_INFO("Preplan trajectory cache ...");
    err = preplanCache(*p_cache, cnt);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to preplan trajectory, code = 0x%llx", err);
        return err;
    }

    p_cache->valid = true;
    start_joint_ = p_cache->cache[p_cache->tail - 1].ending_state.angle;
    FST_INFO("Success.");
    return SUCCESS;
}

ErrorCode BaseGroup::autoCircle(const PoseEuler &target1, const PoseEuler &target2, double vel, double cnt, int id)
{
    return SUCCESS;
}

ErrorCode BaseGroup::prepareCache(TrajectoryCache &cache)
{
    ErrorCode err = SUCCESS;

    if (cache.tail > cache.head)
    {
        MotionType mtype = cache.cache[cache.head].path_point.type;

        if (mtype == MOTION_JOINT)
        {
            for (size_t i = cache.head; i < cache.tail; i++)
            {
                cache.cache[i].ending_state.angle = cache.cache[i].path_point.joint;
                cache.cache[i + 1].start_state.angle = cache.cache[i].path_point.joint;
            }
        }
        else if (mtype == MOTION_LINE || mtype == MOTION_CIRCLE)
        {
            Joint ref = getLatestJoint();

            for (size_t i = cache.head; i < cache.tail; i++)
            {
                err = kinematics_ptr_->inverseKinematicsInUser(cache.cache[i].path_point.pose, ref, cache.cache[i].ending_state.angle);

                if (err == SUCCESS)
                {
                    ref = cache.cache[i].ending_state.angle;
                    cache.cache[i + 1].start_state.angle = cache.cache[i].ending_state.angle;
                    continue;
                }
                else
                {
                    FST_ERROR("Fail to get IK result, code = 0x%llx", err);
                    return err;
                }
            }
        }
        else
        {
            FST_ERROR("Invalid motion type of point in cache.");
            return MOTION_INTERNAL_FAULT;
        }
    }
    else
    {
        FST_ERROR("prepareCache: cache is empty.");
        return MOTION_INTERNAL_FAULT;
    }

    TrajectorySegment &seg0 = cache.cache[cache.head];
    TrajectorySegment &seg1 = cache.cache[cache.head + 1];
    TrajectorySegment &segn  = cache.cache[cache.tail - 1];

    seg0.start_state.angle = seg0.ending_state.angle;
    memset(&seg0.start_state.omega, 0, sizeof(Joint));
    memset(&seg0.start_state.alpha, 0, sizeof(Joint));
    memset(&seg0.ending_state.omega, 0, sizeof(Joint));
    memset(&seg0.ending_state.alpha, 0, sizeof(Joint));
    memset(&seg1.start_state.omega, 0, sizeof(Joint));
    memset(&seg1.start_state.alpha, 0, sizeof(Joint));

    memset(&segn.ending_state.omega, 0, sizeof(Joint));
    memset(&segn.ending_state.alpha, 0, sizeof(Joint));

    /*
    char buffer[LOG_TEXT_SIZE];

    for (size_t i = cache.head; i < cache.tail; i++)
    {
        FST_INFO("Point-%d: %s", i, printDBLine(&cache.cache[i].ending_state.angle[0], buffer, LOG_TEXT_SIZE));
    }
    */


    char buf[LOG_TEXT_SIZE];

    for (size_t i = cache.head; i < cache.tail; i++)
    {
#ifdef OUTPUT_POUT
        sprintf(buf, "%d,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f", i, cache.cache[i].ending_state.angle[0], cache.cache[i].ending_state.angle[1], cache.cache[i].ending_state.angle[2],
                cache.cache[i].ending_state.angle[3], cache.cache[i].ending_state.angle[4], cache.cache[i].ending_state.angle[5]);
        pout << buf << endl;
        FST_LOG("Point-%s", buf);
#else
        FST_LOG("Point-%d: %s", i, printDBLine(&cache.cache[i].ending_state.angle[0], buf, LOG_TEXT_SIZE));
#endif
    }

    return SUCCESS;
}

//#define PRINT_CACHE
#define CHECK_COEFF

#ifdef CHECK_COEFF

bool BaseGroup::checkCoeff(const TrajSegment (&segment)[NUM_OF_JOINT], const JointPoint &start, const Joint &ending, const Joint &alpha_upper, const Joint &alpha_lower, const Joint &jerk)
{
    double duration[6];

    for (size_t i = 0; i < 6; i++)
    {
        if (segment[i].duration[0] < -MINIMUM_E9 || segment[i].duration[1] < -MINIMUM_E9 || segment[i].duration[2] < -MINIMUM_E9 || segment[i].duration[3] < -MINIMUM_E9)
        {
            FST_ERROR("Duration of joint-%d < 0", i);
            return false;
        }

        if (fabs(segment[i].duration[0]) > 1 || fabs(segment[i].duration[1]) > 1 || fabs(segment[i].duration[2]) > 1 || fabs(segment[i].duration[3]) > 1)
        {
            FST_ERROR("Duration of joint-%d > 1", i);
            return false;
        }

        duration[i] = segment[i].duration[0] + segment[i].duration[1] + segment[i].duration[2] + segment[i].duration[3];
    }

    for (size_t i = 0; i < 5; i++)
    {
        if (fabs(duration[i] - duration[i + 1]) > 0.0001)
        {
            FST_ERROR("total duration not equal, duration[%d] = %.9f duration[%d] = %.9f", i, duration[i], i + 1, duration[i + 1]);
            return false;
        }
    }

    for (size_t i = 0; i < 6; i++)
    {
        if (fabs(segment[i].coeff[0][3]) * 6 > jerk[i] + MINIMUM_E3 || fabs(segment[i].coeff[1][3]) * 6 > jerk[i] + MINIMUM_E3 || fabs(segment[i].coeff[2][3]) * 6 > jerk[i] + MINIMUM_E3 || fabs(segment[i].coeff[3][3]) * 6 > jerk[i] + MINIMUM_E3)
        {
            FST_ERROR("Jerk of joint-%d beyond limit", i);
            return false;
        }

        if (segment[i].coeff[0][2] * 2 > alpha_upper[i] * 10 || segment[i].coeff[1][2] * 2 > alpha_upper[i] * 10 || segment[i].coeff[2][2] * 2 > alpha_upper[i] * 10 || segment[i].coeff[3][2] * 2 > alpha_upper[i] * 10)
        {
            FST_ERROR("Alpha of joint-%d > alpha-upper-limit", i);
            return false;
        }

        if (segment[i].coeff[0][2] * 2 < alpha_lower[i] * 10 || segment[i].coeff[1][2] * 2 < alpha_lower[i] * 10 || segment[i].coeff[2][2] * 2 < alpha_lower[i] * 10 || segment[i].coeff[3][2] * 2 < alpha_lower[i] * 10)
        {
            FST_ERROR("Alpha of joint-%d < alpha-lower-limit", i);
            return false;
        }

        if (fabs(segment[i].coeff[0][1]) > 12 || fabs(segment[i].coeff[1][1]) > 12 || fabs(segment[i].coeff[2][1]) > 12 || fabs(segment[i].coeff[3][1]) > 12)
        {
            FST_ERROR("Omega of joint-%d beyond limit", i);
            return false;
        }
    }

    Joint joint, omega, alpha, end_joint, end_omega, end_alpha;

    if (sampleStartTrajectorySegment(segment, joint, omega, alpha) != SUCCESS || sampleEndingTrajectorySegment(segment, end_joint, end_omega, end_alpha) != SUCCESS)
    {
        FST_ERROR("Fail to sample start and ending joint.");
        return false;
    }

    if (!isSameJoint(start.angle, joint))
    {
        FST_ERROR("Start joint mismatch with coeff");
        return false;
    }

    if (!isSameJoint(start.omega, omega))
    {
        FST_ERROR("Start omega mismatch with coeff");
        return false;
    }

    if (!isSameJoint(ending, end_joint))
    {
        FST_ERROR("Ending joint mismatch with coeff");
        return false;
    }

    return true;
}

bool BaseGroup::checkCoeff(const TrajSegment (&segment)[NUM_OF_JOINT], const Joint &start, const JointPoint &ending, const Joint &alpha_upper, const Joint &alpha_lower, const Joint &jerk)
{
    double duration[6];

    for (size_t i = 0; i < 6; i++)
    {
        if (segment[i].duration[0] < -MINIMUM_E9 || segment[i].duration[1] < -MINIMUM_E9 || segment[i].duration[2] < -MINIMUM_E9 || segment[i].duration[3] < -MINIMUM_E9)
        {
            FST_ERROR("Duration of joint-%d < 0", i);
            return false;
        }

        if (fabs(segment[i].duration[0]) > 1 || fabs(segment[i].duration[1]) > 1 || fabs(segment[i].duration[2]) > 1 || fabs(segment[i].duration[3]) > 1)
        {
            FST_ERROR("Duration of joint-%d > 1", i);
            return false;
        }

        duration[i] = segment[i].duration[0] + segment[i].duration[1] + segment[i].duration[2] + segment[i].duration[3];
    }

    for (size_t i = 0; i < 5; i++)
    {
        if (fabs(duration[i] - duration[i + 1]) > 0.0001)
        {
            FST_ERROR("total duration not equal, duration[%d] = %.9f duration[%d] = %.9f", i, duration[i], i + 1, duration[i + 1]);
            return false;
        }
    }

    for (size_t i = 0; i < 6; i++)
    {
        if (fabs(segment[i].coeff[0][3]) * 6 > jerk[i] + MINIMUM_E3 || fabs(segment[i].coeff[1][3]) * 6 > jerk[i] + MINIMUM_E3 || fabs(segment[i].coeff[2][3]) * 6 > jerk[i] + MINIMUM_E3 || fabs(segment[i].coeff[3][3]) * 6 > jerk[i] + MINIMUM_E3)
        {
            FST_ERROR("Jerk of joint-%d beyond limit", i);
            return false;
        }

        if (segment[i].coeff[0][2] * 2 > alpha_upper[i] * 10 || segment[i].coeff[1][2] * 2 > alpha_upper[i] * 10 || segment[i].coeff[2][2] * 2 > alpha_upper[i] * 10 || segment[i].coeff[3][2] * 2 > alpha_upper[i] * 10)
        {
            FST_ERROR("Alpha of joint-%d > alpha-upper-limit", i);
            return false;
        }

        if (segment[i].coeff[0][2] * 2 < alpha_lower[i] * 10 || segment[i].coeff[1][2] * 2 < alpha_lower[i] * 10 || segment[i].coeff[2][2] * 2 < alpha_lower[i] * 10 || segment[i].coeff[3][2] * 2 < alpha_lower[i] * 10)
        {
            FST_ERROR("Alpha of joint-%d < alpha-lower-limit", i);
            return false;
        }

        if (fabs(segment[i].coeff[0][1]) > 12 || fabs(segment[i].coeff[1][1]) > 12 || fabs(segment[i].coeff[2][1]) > 12 || fabs(segment[i].coeff[3][1]) > 12)
        {
            FST_ERROR("Omega of joint-%d beyond limit", i);
            return false;
        }
    }

    Joint joint, omega, alpha, end_joint, end_omega, end_alpha;

    if (sampleStartTrajectorySegment(segment, joint, omega, alpha) != SUCCESS || sampleEndingTrajectorySegment(segment, end_joint, end_omega, end_alpha) != SUCCESS)
    {
        FST_ERROR("Fail to sample start and ending joint.");
        return false;
    }

    if (!isSameJoint(start, joint))
    {
        FST_ERROR("Start joint mismatch with coeff");
        return false;
    }

    if (!isSameJoint(ending.angle, end_joint))
    {
        FST_ERROR("Ending joint mismatch with coeff");
        return false;
    }

    if (!isSameJoint(ending.omega, end_omega))
    {
        FST_ERROR("Ending omega mismatch with coeff");
        return false;
    }

    return true;
}

bool BaseGroup::checkCoeff(const TrajSegment (&segment)[NUM_OF_JOINT], const JointPoint &start, const JointPoint &ending, const Joint &alpha_upper, const Joint &alpha_lower, const Joint &jerk)
{
    double duration[6];

    for (size_t i = 0; i < 6; i++)
    {
        if (segment[i].duration[0] < -MINIMUM_E9 || segment[i].duration[1] < -MINIMUM_E9 || segment[i].duration[2] < -MINIMUM_E9 || segment[i].duration[3] < -MINIMUM_E9)
        {
            FST_ERROR("Duration of joint-%d < 0", i);
            return false;
        }

        if (fabs(segment[i].duration[0]) > 1 || fabs(segment[i].duration[1]) > 1 || fabs(segment[i].duration[2]) > 1 || fabs(segment[i].duration[3]) > 1)
        {
            FST_ERROR("Duration of joint-%d > 1", i);
            return false;
        }

        duration[i] = segment[i].duration[0] + segment[i].duration[1] + segment[i].duration[2] + segment[i].duration[3];
    }

    for (size_t i = 0; i < 5; i++)
    {
        if (fabs(duration[i] - duration[i + 1]) > 0.0001)
        {
            FST_ERROR("Total duration not equal, duration[%d] = %.9f duration[%d] = %.9f", i, duration[i], i + 1, duration[i + 1]);
            return false;
        }
    }

    for (size_t i = 0; i < 6; i++)
    {
        if (segment[i].coeff[0][2] * 2 > alpha_upper[i] * 10 || segment[i].coeff[1][2] * 2 > alpha_upper[i] * 10 || segment[i].coeff[2][2] * 2 > alpha_upper[i] * 10 || segment[i].coeff[3][2] * 2 > alpha_upper[i] * 10)
        {
            FST_ERROR("Alpha of joint-%d > alpha-upper-limit", i);
            return false;
        }

        if (segment[i].coeff[0][2] * 2 < alpha_lower[i] * 10 || segment[i].coeff[1][2] * 2 < alpha_lower[i] * 10 || segment[i].coeff[2][2] * 2 < alpha_lower[i] * 10 || segment[i].coeff[3][2] * 2 < alpha_lower[i] * 10)
        {
            FST_ERROR("Alpha of joint-%d < alpha-lower-limit", i);
            return false;
        }

        if (fabs(segment[i].coeff[0][1]) > 12 || fabs(segment[i].coeff[1][1]) > 12 || fabs(segment[i].coeff[2][1]) > 12 || fabs(segment[i].coeff[3][1]) > 12)
        {
            FST_ERROR("Omega of joint-%d beyond limit", i);
            return false;
        }
    }

    Joint joint, omega, alpha, end_joint, end_omega, end_alpha;

    if (sampleStartTrajectorySegment(segment, joint, omega, alpha) != SUCCESS || sampleEndingTrajectorySegment(segment, end_joint, end_omega, end_alpha) != SUCCESS)
    {
        FST_ERROR("Fail to sample start and ending joint.");
        return false;
    }

    if (!isSameJoint(start.angle, joint))
    {
        FST_ERROR("Start joint mismatch with coeff");
        return false;
    }

    if (!isSameJoint(start.omega, omega))
    {
        FST_ERROR("Start omega mismatch with coeff");
        return false;
    }

    if (!isSameJoint(ending.angle, end_joint))
    {
        FST_ERROR("Ending joint mismatch with coeff");
        return false;
    }

    if (!isSameJoint(ending.omega, end_omega))
    {
        FST_ERROR("Ending omega mismatch with coeff");
        return false;
    }

    return true;
}
#endif

ErrorCode BaseGroup::preplanCache(TrajectoryCache &cache, double cnt)
{
    ErrorCode   err = SUCCESS;
    size_t  index;
    char    buffer[LOG_TEXT_SIZE];
    double  this_duration, last_duration;
    Joint   tmp;
    Joint   forward_alpha_upper, forward_alpha_lower;
    Joint   backward_alpha_upper, backward_alpha_lower;
    JointPoint  fore_status[MAX_PATH_SIZE];
    JointPoint  back_status[MAX_PATH_SIZE];
    DynamicsProduct     forward_dynamics_product, backward_dynamics_product;
    TrajectorySegment   *pseg;

    computeDynamics(cache.cache[1].start_state.angle, cache.cache[1].start_state.omega, forward_alpha_upper, forward_alpha_lower, forward_dynamics_product);
    computeDynamics(cache.cache[cache.tail - 1].ending_state.angle, cache.cache[cache.tail - 1].ending_state.omega, backward_alpha_upper, backward_alpha_lower, backward_dynamics_product);

#ifdef PRINT_CACHE
    FST_LOG("forward angle: %s", printDBLine(&cache.cache[1].start_state.angle[0], buffer, LOG_TEXT_SIZE));
    FST_LOG("forward omega: %s", printDBLine(&cache.cache[1].start_state.omega[0], buffer, LOG_TEXT_SIZE));
    FST_LOG("forward alpha-upper: %s", printDBLine(&forward_alpha_upper[0], buffer, LOG_TEXT_SIZE));
    FST_LOG("forward alpha-lower: %s", printDBLine(&forward_alpha_lower[0], buffer, LOG_TEXT_SIZE));
    FST_LOG("backward angle: %s", printDBLine(&cache.cache[cache.tail - 1].ending_state.angle[0], buffer, LOG_TEXT_SIZE));
    FST_LOG("backward omega: %s", printDBLine(&cache.cache[cache.tail - 1].ending_state.omega[0], buffer, LOG_TEXT_SIZE));
    FST_LOG("backward alpha-upper: %s", printDBLine(&backward_alpha_upper[0], buffer, LOG_TEXT_SIZE));
    FST_LOG("backward alpha-lower: %s", printDBLine(&backward_alpha_lower[0], buffer, LOG_TEXT_SIZE));
    FST_LOG("jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));
#endif

    size_t joint_num = getNumberOfJoint();

    for (size_t i = 0; i < joint_num; i++)
    {
        forward_alpha_lower[i] = forward_alpha_lower[i] * acc_ratio_;
        forward_alpha_upper[i] = forward_alpha_upper[i] * acc_ratio_;
        backward_alpha_lower[i] = backward_alpha_lower[i] * acc_ratio_;
        backward_alpha_upper[i] = backward_alpha_upper[i] * acc_ratio_;
    }

#ifdef PRINT_CACHE
    FST_LOG("acc ratio: %.12f", acc_ratio_);
    FST_LOG("forward alpha-upper: %s", printDBLine(&forward_alpha_upper[0], buffer, LOG_TEXT_SIZE));
    FST_LOG("forward alpha-lower: %s", printDBLine(&forward_alpha_lower[0], buffer, LOG_TEXT_SIZE));
    FST_LOG("backward alpha-upper: %s", printDBLine(&backward_alpha_upper[0], buffer, LOG_TEXT_SIZE));
    FST_LOG("backward alpha-lower: %s", printDBLine(&backward_alpha_lower[0], buffer, LOG_TEXT_SIZE));
#endif


    memcpy(&fore_status[0], &cache.cache[0].start_state, sizeof(JointPoint));
    memcpy(&fore_status[1], &cache.cache[1].start_state, sizeof(JointPoint));
    memcpy(&back_status[cache.tail - 1], &cache.cache[cache.tail - 1].ending_state, sizeof(JointPoint));

    index = cache.tail - 1;
    this_duration = 99.99;
    last_duration = 99.99;

    while (this_duration > cache.expect_duration + MINIMUM_E6 && index > 1)
    {
        pseg = &cache.cache[index];
        err = backwardCycle(pseg->start_state.angle, back_status[index], cache.expect_duration, backward_alpha_upper, backward_alpha_lower, jerk_, pseg->backward_coeff);

        if (err == SUCCESS)
        {
#ifdef CHECK_COEFF
            if (!checkCoeff(pseg->backward_coeff, pseg->start_state.angle, back_status[index], backward_alpha_upper, backward_alpha_lower, jerk_))
            {
                FST_ERROR("ERROR in back-cycle-%d, this-duration: %.12f, exp-duration: %.12f, last-duration: %.12f", index, this_duration, cache.expect_duration, last_duration);
                FST_ERROR("  start-angle: %s", printDBLine(&pseg->start_state.angle[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  ending-angle: %s", printDBLine(&back_status[index].angle[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  ending-omega: %s", printDBLine(&back_status[index].omega[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  ending-alpha: %s", printDBLine(&back_status[index].alpha[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  alpha-upper: %s", printDBLine(&backward_alpha_upper[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  alpha-lower: %s", printDBLine(&backward_alpha_lower[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));

                for (size_t i = 0; i < 6; i++)
                {
                    FST_ERROR("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", pseg->backward_coeff[i].duration[0], pseg->backward_coeff[i].duration[1], pseg->backward_coeff[i].duration[2], pseg->backward_coeff[i].duration[3],
                                                                                        pseg->backward_coeff[i].duration[0] + pseg->backward_coeff[i].duration[1] + pseg->backward_coeff[i].duration[2] + pseg->backward_coeff[i].duration[3]);
                    FST_ERROR("  coeff = %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[0][3], pseg->backward_coeff[i].coeff[0][2], pseg->backward_coeff[i].coeff[0][1], pseg->backward_coeff[i].coeff[0][0]);
                    FST_ERROR("          %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[1][3], pseg->backward_coeff[i].coeff[1][2], pseg->backward_coeff[i].coeff[1][1], pseg->backward_coeff[i].coeff[1][0]);
                    FST_ERROR("          %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[2][3], pseg->backward_coeff[i].coeff[2][2], pseg->backward_coeff[i].coeff[2][1], pseg->backward_coeff[i].coeff[2][0]);
                    FST_ERROR("          %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[3][3], pseg->backward_coeff[i].coeff[3][2], pseg->backward_coeff[i].coeff[3][1], pseg->backward_coeff[i].coeff[3][0]);
                }

                err = MOTION_INTERNAL_FAULT;
                FST_ERROR("back-work abort");
                break;
            }
#endif
            this_duration = pseg->backward_coeff[0].duration[0] + pseg->backward_coeff[0].duration[1] + pseg->backward_coeff[0].duration[2] + pseg->backward_coeff[0].duration[3];
            pseg->backward_duration = this_duration;
            sampleStartTrajectorySegment(pseg->backward_coeff, tmp, back_status[index - 1].omega, back_status[index - 1].alpha);
            memcpy(&back_status[index - 1].angle, &cache.cache[index - 1].ending_state.angle, sizeof(Joint));

#ifdef PRINT_CACHE
            FST_LOG("back-cycle-%d, this-duration: %.12f, exp-duration: %.12f, last-duration: %.12f", index, this_duration, cache.expect_duration, last_duration);
            FST_LOG("  start-angle: %s", printDBLine(&pseg->start_state.angle[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  ending-angle: %s", printDBLine(&back_status[index].angle[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  ending-omega: %s", printDBLine(&back_status[index].omega[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  ending-alpha: %s", printDBLine(&back_status[index].alpha[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  alpha-upper: %s", printDBLine(&backward_alpha_upper[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  alpha-lower: %s", printDBLine(&backward_alpha_lower[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));

            for (size_t i = 0; i < 6; i++)
            {
                FST_LOG("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", pseg->backward_coeff[i].duration[0], pseg->backward_coeff[i].duration[1], pseg->backward_coeff[i].duration[2], pseg->backward_coeff[i].duration[3],
                                                                                  pseg->backward_coeff[i].duration[0] + pseg->backward_coeff[i].duration[1] + pseg->backward_coeff[i].duration[2] + pseg->backward_coeff[i].duration[3]);
                FST_LOG("  coeff = %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[0][3], pseg->backward_coeff[i].coeff[0][2], pseg->backward_coeff[i].coeff[0][1], pseg->backward_coeff[i].coeff[0][0]);
                FST_LOG("          %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[1][3], pseg->backward_coeff[i].coeff[1][2], pseg->backward_coeff[i].coeff[1][1], pseg->backward_coeff[i].coeff[1][0]);
                FST_LOG("          %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[2][3], pseg->backward_coeff[i].coeff[2][2], pseg->backward_coeff[i].coeff[2][1], pseg->backward_coeff[i].coeff[2][0]);
                FST_LOG("          %.6f, %.6f, %.6f, %.6f", pseg->backward_coeff[i].coeff[3][3], pseg->backward_coeff[i].coeff[3][2], pseg->backward_coeff[i].coeff[3][1], pseg->backward_coeff[i].coeff[3][0]);
            }
#endif

            if (this_duration + MINIMUM_E6 > last_duration)
            {
                break;
            }
            else
            {
                last_duration = this_duration;
            }

            if (this_duration > cache.expect_duration + MINIMUM_E6)
            {
                index --;
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Fail in back cycle %d, err = 0x%llx", index, err);
        return err;
    }

    index = 1;
    this_duration = 99.99;
    last_duration = 99.99;

    while (this_duration > cache.expect_duration + MINIMUM_E6 && index < cache.tail - 1)
    {
        pseg = &cache.cache[index];
        err = forwardCycle(fore_status[index], pseg->ending_state.angle, cache.expect_duration, forward_alpha_upper, forward_alpha_lower, jerk_, pseg->forward_coeff);
        //err = forwardCycleTest(pseg->start_state, pseg->ending_state.angle, 0.01, forward_alpha_upper, forward_alpha_lower, jerk_, pseg->forward_coeff);

        if (err == SUCCESS)
        {
#ifdef CHECK_COEFF
            if (!checkCoeff(pseg->forward_coeff, fore_status[index], pseg->ending_state.angle, forward_alpha_upper, forward_alpha_lower, jerk_))
            {
                FST_ERROR("ERROR in fore-cycle-%d, this-duration: %.12f, exp-duration: %.12f, last-duration: %.12f", index, this_duration, cache.expect_duration, last_duration);
                FST_ERROR("  start-angle: %s", printDBLine(&fore_status[index].angle[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  start-omega: %s", printDBLine(&fore_status[index].omega[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  start-alpha: %s", printDBLine(&fore_status[index].alpha[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  ending-angle: %s", printDBLine(&pseg->ending_state.angle[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  alpha-upper: %s", printDBLine(&forward_alpha_upper[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  alpha-lower: %s", printDBLine(&forward_alpha_lower[0], buffer, LOG_TEXT_SIZE));
                FST_ERROR("  jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));

                for (size_t i = 0; i < 6; i++)
                {
                    FST_ERROR("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", pseg->forward_coeff[i].duration[0], pseg->forward_coeff[i].duration[1], pseg->forward_coeff[i].duration[2], pseg->forward_coeff[i].duration[3],
                                                                                        pseg->forward_coeff[i].duration[0] + pseg->forward_coeff[i].duration[1] + pseg->forward_coeff[i].duration[2] + pseg->forward_coeff[i].duration[3]);
                    FST_ERROR("  coeff = %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[0][3], pseg->forward_coeff[i].coeff[0][2], pseg->forward_coeff[i].coeff[0][1], pseg->forward_coeff[i].coeff[0][0]);
                    FST_ERROR("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[1][3], pseg->forward_coeff[i].coeff[1][2], pseg->forward_coeff[i].coeff[1][1], pseg->forward_coeff[i].coeff[1][0]);
                    FST_ERROR("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[2][3], pseg->forward_coeff[i].coeff[2][2], pseg->forward_coeff[i].coeff[2][1], pseg->forward_coeff[i].coeff[2][0]);
                    FST_ERROR("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[3][3], pseg->forward_coeff[i].coeff[3][2], pseg->forward_coeff[i].coeff[3][1], pseg->forward_coeff[i].coeff[3][0]);
                }

                err = MOTION_INTERNAL_FAULT;
                FST_ERROR("fore-work abort");
                break;
            }
#endif
            this_duration = pseg->forward_coeff[0].duration[0] + pseg->forward_coeff[0].duration[1] + pseg->forward_coeff[0].duration[2] + pseg->forward_coeff[0].duration[3];
            pseg->forward_duration = this_duration;
            sampleEndingTrajectorySegment(pseg->forward_coeff, tmp, fore_status[index + 1].omega, fore_status[index + 1].alpha);
            memcpy(&fore_status[index + 1].angle, &cache.cache[index + 1].start_state.angle, sizeof(Joint));

#ifdef PRINT_CACHE
            FST_LOG("fore-cycle-%d, this-duration: %.12f, exp-duration: %.12f, last-duration: %.12f", index, this_duration, cache.expect_duration, last_duration);
            FST_LOG("  start-angle: %s", printDBLine(&fore_status[index].angle[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  start-omega: %s", printDBLine(&fore_status[index].omega[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  start-alpha: %s", printDBLine(&fore_status[index].alpha[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  ending-angle: %s", printDBLine(&pseg->ending_state.angle[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  alpha-upper: %s", printDBLine(&forward_alpha_upper[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  alpha-lower: %s", printDBLine(&forward_alpha_lower[0], buffer, LOG_TEXT_SIZE));
            FST_LOG("  jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));

            for (size_t i = 0; i < 6; i++)
            {
                FST_LOG("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", pseg->forward_coeff[i].duration[0], pseg->forward_coeff[i].duration[1], pseg->forward_coeff[i].duration[2], pseg->forward_coeff[i].duration[3],
                                                                                  pseg->forward_coeff[i].duration[0] + pseg->forward_coeff[i].duration[1] + pseg->forward_coeff[i].duration[2] + pseg->forward_coeff[i].duration[3]);
                FST_LOG("  coeff = %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[0][3], pseg->forward_coeff[i].coeff[0][2], pseg->forward_coeff[i].coeff[0][1], pseg->forward_coeff[i].coeff[0][0]);
                FST_LOG("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[1][3], pseg->forward_coeff[i].coeff[1][2], pseg->forward_coeff[i].coeff[1][1], pseg->forward_coeff[i].coeff[1][0]);
                FST_LOG("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[2][3], pseg->forward_coeff[i].coeff[2][2], pseg->forward_coeff[i].coeff[2][1], pseg->forward_coeff[i].coeff[2][0]);
                FST_LOG("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[3][3], pseg->forward_coeff[i].coeff[3][2], pseg->forward_coeff[i].coeff[3][1], pseg->forward_coeff[i].coeff[3][0]);
            }
#endif

            if (this_duration + MINIMUM_E6 > last_duration)
            {
                break;
            }
            else
            {
                last_duration = this_duration;
            }

            if (this_duration > cache.expect_duration + MINIMUM_E6)
            {
                index ++;
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    if (err != SUCCESS)
    {
        FST_ERROR("Fail in fore cycle %d, err = 0x%llx", index, err);
        return err;
    }

    size_t forward_index = cache.head + 1;
    size_t backward_index = cache.tail - 1;

    while (forward_index != backward_index)
    {
        if (cache.cache[forward_index].forward_duration > cache.cache[backward_index].backward_duration)
        {
            if (cache.cache[forward_index + 1].forward_duration > 0)
            {
                forward_index ++;
            }
            else
            {
                break;
            }
        }
        else
        {
            if (cache.cache[backward_index - 1].backward_duration > 0)
            {
                backward_index --;
            }
            else
            {
                break;
            }
        }
    }

    if (forward_index == backward_index)
    {
        if (forward_index == cache.head + 1)
        {
            backward_index ++;
        }
        else if (backward_index == cache.tail - 1)
        {
            forward_index --;
        }
        else
        {
            if (cache.cache[forward_index].forward_duration < cache.cache[backward_index].backward_duration)
            {
                forward_index --;
            }
            else
            {
                backward_index ++;
            }
        }
    }

    for (index = 1; index < cache.tail; index++)
    {
        if (cache.cache[index].forward_duration > 0)
        {
            cache.cache[index].dynamics_product = forward_dynamics_product;
            cache.cache[index].start_state = fore_status[index];
            cache.cache[index - 1].ending_state = fore_status[index];

            if (cache.cache[index + 1].forward_duration < 0)
            {
                cache.cache[index + 1].start_state = fore_status[index + 1];
                cache.cache[index].ending_state = fore_status[index + 1];
            }
        }
        else if (cache.cache[index].backward_duration > 0)
        {
            cache.cache[index].dynamics_product = backward_dynamics_product;
            cache.cache[index].ending_state = back_status[index];
            cache.cache[index + 1].start_state = back_status[index];

            if (cache.cache[index - 1].backward_duration < 0)
            {
                cache.cache[index].start_state = back_status[index - 1];
                cache.cache[index - 1].ending_state = back_status[index - 1];
            }
        }
    }

    size_t spd_up = forward_index;
    size_t spd_down = backward_index;

    forward_index ++;
    backward_index --;

    while (cache.cache[forward_index].forward_duration > 0 && forward_index < cache.tail)
    {
        cache.cache[forward_index ++].forward_duration = -1;
    }

    while (cache.cache[backward_index].backward_duration > 0 && backward_index > cache.head)
    {
        cache.cache[backward_index --].backward_duration = -1;
    }

#ifdef PRINT_CACHE
    FST_LOG("spd-up-index=%d, spd-down-index=%d", spd_up, spd_down);

    for (size_t i = cache.head; i < cache.tail; i++)
    {
        FST_LOG("stamp=%d, duration_f=%.12f, duration_b=%.12f, cmd_duration=%.12f",
                 cache.cache[i].path_point.stamp, cache.cache[i].forward_duration,
                 cache.cache[i].backward_duration, cache.expect_duration);
    }
#endif

    if (fabs(cnt - 1) < MINIMUM_E3)
    {
        // CNT = 100%
        cache.smooth_in_stamp = spd_up;
        cache.smooth_out_stamp = spd_down;
    }
    else if (cnt < MINIMUM_E3)
    {
        // CNT = 0% or FINE
        cache.smooth_in_stamp = cache.head;
        cache.smooth_out_stamp = cache.tail;
    }
    else
    {
        // CNT = 0% ~ 100%
        double smooth_duration = cache.expect_duration / cnt + 0.0001;

        for (forward_index = cache.head + 1; forward_index < cache.tail; forward_index++)
        {
            if (cache.cache[forward_index].forward_duration > smooth_duration && forward_index + 1 < cache.tail && cache.cache[forward_index + 1].forward_duration > 0)
            {
                forward_index ++;
            }
            else
            {
                break;
            }
        }

        cache.smooth_in_stamp = forward_index;

        for (backward_index = cache.tail - 1; backward_index > cache.head; backward_index--)
        {
            if (cache.cache[backward_index].backward_duration > smooth_duration && backward_index - 1 > cache.head && cache.cache[backward_index - 1].backward_duration > 0)
            {
                backward_index --;
            }
            else
            {
                break;
            }
        }

        cache.smooth_out_stamp = backward_index;
    }

    return SUCCESS;
}

ErrorCode BaseGroup::abortMove(void)
{
    if (group_state_ == AUTO || group_state_ == PAUSE)
    {
        pthread_mutex_lock(&auto_mutex_);
        group_state_ = STANDBY;
        auto_time_ = 0;
        traj_fifo_.clear();
        bare_core_.clearPointCache();

        while (auto_pick_ptr_->valid)
        {
            auto_pick_ptr_->valid = false;
            auto_pick_ptr_ = auto_pick_ptr_->next;
        }

        pthread_mutex_unlock(&auto_mutex_);
    }

    FST_WARN("Auto move Aborted");
    return SUCCESS;
}

bool BaseGroup::nextMovePermitted(void)
{
    if (waiting_fine_)
    {
        return false;
    }

    pthread_mutex_lock(&auto_mutex_);

    if (group_state_ == AUTO || group_state_ == AUTO_TO_STANDBY || group_state_ == STANDBY_TO_AUTO)
    {
        TrajectoryCache *ptr = auto_pick_ptr_->next->valid ? auto_pick_ptr_->next : auto_pick_ptr_;

        if (ptr->valid && auto_time_ + 0.15 < ptr->deadline)
        {
            FST_LOG("Not-permitted: auto-time = %.4f, deadline = %.4f", auto_time_, ptr->deadline);
            pthread_mutex_unlock(&auto_mutex_);
            return false;
        }
    }

    FST_WARN("Next motion permitted: auto-time = %.4f, deadline = %.4f", auto_time_, auto_pick_ptr_->deadline);
    pthread_mutex_unlock(&auto_mutex_);
    return true;
}

//#define  PRINT_COEFFS

ErrorCode BaseGroup::createTrajectory(void)
{
    static size_t  dynamics_cnt = 0;
    static Joint   alpha_upper, alpha_lower;
    static DynamicsProduct dynamics_product;

    char buffer[LOG_TEXT_SIZE];
    ErrorCode   err = SUCCESS;
    Joint       tmp_joint;
    TrajectoryItem      traj_item;
    TrajectorySegment   *pseg;

    if (group_state_ == AUTO && auto_pick_ptr_->valid)
    {
        while ((traj_fifo_.duration() < 0.15 && !traj_fifo_.full()) || traj_fifo_.size() < 3)
        {
            if (auto_pick_segment_ >= auto_pick_ptr_->tail)
            {
                //FST_WARN("cache ptr = %x is empty", auto_pick_ptr_);

                pthread_mutex_lock(&auto_mutex_);
                if (auto_pick_ptr_->next->valid)
                {
                    auto_pick_ptr_->valid = false;
                    auto_pick_ptr_ = auto_pick_ptr_->next;
                    auto_pick_segment_ = 1;
                    FST_INFO("auto-pick-ptr switch, seg = %d, time-from-start = %.4f", auto_pick_segment_, traj_fifo_.back().time_from_start);
                }
                else
                {
                    auto_pick_ptr_->valid = false;
                    auto_pick_segment_ = 1;
                    FST_INFO("auto-pick-ptr turn to invalid");
                    pthread_mutex_unlock(&auto_mutex_);
                    break;
                }
                pthread_mutex_unlock(&auto_mutex_);
            }

#ifdef PRINT_COEFFS
            FST_LOG("this-seg-%d, fore-duration = %.4f, back-duration = %.4f", auto_pick_segment_, auto_pick_ptr_->cache[auto_pick_segment_].forward_duration, auto_pick_ptr_->cache[auto_pick_segment_].backward_duration);
            FST_LOG("next-seg-%d, fore-duration = %.4f, back-duration = %.4f", auto_pick_segment_ + 1, auto_pick_ptr_->cache[auto_pick_segment_ + 1].forward_duration, auto_pick_ptr_->cache[auto_pick_segment_ + 1].backward_duration);
            FST_LOG("smooth-out = %d, traj-tail = %d", auto_pick_ptr_->smooth_out_stamp, auto_pick_ptr_->tail);
#endif

            if (auto_pick_segment_ == auto_pick_ptr_->smooth_out_stamp)
            {
                if (auto_pick_ptr_->deadline > 9999)
                {
                    auto_pick_ptr_->deadline = traj_fifo_.timeFromStart();
                    FST_WARN("Set deadline: auto-time = %.4f, deadline = %.4f", auto_time_, auto_pick_ptr_->deadline);
                }

                if (auto_pick_ptr_->next->valid)
                {
                    FST_WARN("Next motion is ready, smooth to next motion.");
                    pseg = &auto_pick_ptr_->cache[auto_pick_segment_];

                    if (dynamics_cnt == 0)
                    {
                        computeDynamics(pseg->start_state.angle, pseg->start_state.omega, alpha_upper, alpha_lower, dynamics_product);
                        dynamics_cnt = dynamics_cnt_;

                        size_t joint_num = getNumberOfJoint();

                        for (size_t i = 0; i < joint_num; i++)
                        {
                            alpha_lower[i] = alpha_lower[i] * acc_ratio_;
                            alpha_upper[i] = alpha_upper[i] * acc_ratio_;
                        }
                    }

                    double duration = auto_pick_ptr_->expect_duration * (auto_pick_ptr_->tail - auto_pick_ptr_->smooth_out_stamp) + auto_pick_ptr_->next->expect_duration * auto_pick_ptr_->smooth_in_stamp;
                    smoothPoint2Point(pseg->start_state, auto_pick_ptr_->next->cache[auto_pick_ptr_->next->smooth_in_stamp].ending_state, duration, alpha_upper, alpha_lower, jerk_, traj_item.traj_coeff);
                    dynamics_cnt = dynamics_cnt > 0 ? dynamics_cnt - 1 : 0;

#ifdef OUTPUT_COEFF
                    g_cout[g_cindex].type = SMOOTH_CYCLE;
                    g_cout[g_cindex].start = pseg->start_state;
                    g_cout[g_cindex].ending = auto_pick_ptr_->next->cache[auto_pick_ptr_->next->smooth_in_stamp].ending_state;
                    g_cout[g_cindex].alpha_upper = alpha_upper;
                    g_cout[g_cindex].alpha_lower = alpha_lower;
                    g_cout[g_cindex].exp_duration = duration;
                    g_cout[g_cindex].duration = traj_item.traj_coeff[0].duration[0] + traj_item.traj_coeff[0].duration[1] + traj_item.traj_coeff[0].duration[2] + traj_item.traj_coeff[0].duration[3];
                    g_cout[g_cindex].start_time = traj_fifo_.timeFromStart();
                    memcpy(g_cout[g_cindex].segment, traj_item.traj_coeff, sizeof(traj_item.traj_coeff));
                    g_cindex = (g_cindex + 1) % COEFF_SIZE;
#endif
#ifdef CHECK_COEFF
                    if (!checkCoeff(traj_item.traj_coeff, pseg->start_state, auto_pick_ptr_->next->cache[auto_pick_ptr_->next->smooth_in_stamp].ending_state, alpha_upper, alpha_lower, jerk_))
                    {
                        FST_ERROR("ERROR in smooth, exp-duration: %.12f", duration);
                        FST_ERROR("exp=%.12f, tail=%d, smooth-out=%d", auto_pick_ptr_->expect_duration, auto_pick_ptr_->tail, auto_pick_ptr_->smooth_out_stamp);
                        FST_ERROR("exp=%.12f, smooth-in=%d", auto_pick_ptr_->next->expect_duration, auto_pick_ptr_->smooth_in_stamp);
                        FST_ERROR("  start-angle: %s", printDBLine(&pseg->start_state.angle[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("  start-omega: %s", printDBLine(&pseg->start_state.omega[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("  ending-angle: %s", printDBLine(&auto_pick_ptr_->next->cache[auto_pick_ptr_->next->smooth_in_stamp].ending_state.angle[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("  ending-omega: %s", printDBLine(&auto_pick_ptr_->next->cache[auto_pick_ptr_->next->smooth_in_stamp].ending_state.omega[0], buffer, LOG_TEXT_SIZE));

                        for (size_t i = 0; i < 6; i++)
                        {
                            FST_ERROR("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", traj_item.traj_coeff[i].duration[0], traj_item.traj_coeff[i].duration[1], traj_item.traj_coeff[i].duration[2], traj_item.traj_coeff[i].duration[3],
                                                                                                traj_item.traj_coeff[i].duration[0] + traj_item.traj_coeff[i].duration[1] + traj_item.traj_coeff[i].duration[2] + traj_item.traj_coeff[i].duration[3]);
                            FST_ERROR("  coeff = %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[0][3], traj_item.traj_coeff[i].coeff[0][2], traj_item.traj_coeff[i].coeff[0][1], traj_item.traj_coeff[i].coeff[0][0]);
                            FST_ERROR("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[1][3], traj_item.traj_coeff[i].coeff[1][2], traj_item.traj_coeff[i].coeff[1][1], traj_item.traj_coeff[i].coeff[1][0]);
                            FST_ERROR("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[2][3], traj_item.traj_coeff[i].coeff[2][2], traj_item.traj_coeff[i].coeff[2][1], traj_item.traj_coeff[i].coeff[2][0]);
                            FST_ERROR("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[3][3], traj_item.traj_coeff[i].coeff[3][2], traj_item.traj_coeff[i].coeff[3][1], traj_item.traj_coeff[i].coeff[3][0]);
                        }

                        FST_ERROR("createTrajectory: internal fault !!!");
                        return MOTION_INTERNAL_FAULT;
                    }
#endif
#ifdef PRINT_COEFFS
                    FST_LOG("start angle: %s", printDBLine(&pseg->start_state.angle[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("start omega: %s", printDBLine(&pseg->start_state.omega[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("target angle: %s", printDBLine(&auto_pick_ptr_->next->cache[auto_pick_ptr_->next->smooth_in_stamp].ending_state.angle[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("target omega: %s", printDBLine(&auto_pick_ptr_->next->cache[auto_pick_ptr_->next->smooth_in_stamp].ending_state.omega[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("alpha-upper: %s", printDBLine(&alpha_upper[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("alpha-lower: %s", printDBLine(&alpha_lower[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("start time: %.12f", traj_fifo_.timeFromStart());

                    for (size_t i = 0; i < 6; i++)
                    {
                        FST_LOG("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", traj_item.traj_coeff[i].duration[0], traj_item.traj_coeff[i].duration[1], traj_item.traj_coeff[i].duration[2], traj_item.traj_coeff[i].duration[3],
                                                                                            traj_item.traj_coeff[i].duration[0] + traj_item.traj_coeff[i].duration[1] + traj_item.traj_coeff[i].duration[2] + traj_item.traj_coeff[i].duration[3]);
                        FST_LOG("  coeff = %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[0][3], traj_item.traj_coeff[i].coeff[0][2], traj_item.traj_coeff[i].coeff[0][1], traj_item.traj_coeff[i].coeff[0][0]);
                        FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[1][3], traj_item.traj_coeff[i].coeff[1][2], traj_item.traj_coeff[i].coeff[1][1], traj_item.traj_coeff[i].coeff[1][0]);
                        FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[2][3], traj_item.traj_coeff[i].coeff[2][2], traj_item.traj_coeff[i].coeff[2][1], traj_item.traj_coeff[i].coeff[2][0]);
                        FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[3][3], traj_item.traj_coeff[i].coeff[3][2], traj_item.traj_coeff[i].coeff[3][1], traj_item.traj_coeff[i].coeff[3][0]);
                    }
#endif

                    traj_item.id = pseg->path_point.id;
                    traj_item.dynamics_product = dynamics_product;
                    traj_item.duration = traj_item.traj_coeff[0].duration[0] + traj_item.traj_coeff[0].duration[1] + traj_item.traj_coeff[0].duration[2] + traj_item.traj_coeff[0].duration[3];
                    traj_item.time_from_start = traj_fifo_.timeFromStart() + traj_item.duration;
                    traj_fifo_.push(traj_item);

                    pthread_mutex_lock(&auto_mutex_);
                    auto_pick_ptr_->valid = false;
                    auto_pick_ptr_ = auto_pick_ptr_->next;
                    auto_pick_segment_ = auto_pick_ptr_->smooth_in_stamp + 1;
                    pthread_mutex_unlock(&auto_mutex_);
                    continue;
                }
                else if (auto_time_ + 0.05 < traj_fifo_.timeFromStart())
                {
                    // Next motion is NOT ready, waiting for next motion
                    FST_LOG("Next motion is NOT ready, waiting for next motion");
                    break;
                }
                else
                {
                    FST_WARN("Next motion is NOT ready, auto-time = %.4f, deadline = %.4f, time-from-start of trajectory-fifo = %.4f, smooth-CNT will be abandoned",
                             auto_time_, auto_pick_ptr_->deadline, traj_fifo_.timeFromStart());
                    auto_pick_ptr_->smooth_out_stamp = auto_pick_ptr_->tail;
                }
            }

            pseg = &auto_pick_ptr_->cache[auto_pick_segment_];

            if (pseg->forward_duration > 0)
            {
                dynamics_cnt = 0;
                traj_item.id = pseg->path_point.id;
                traj_item.duration = pseg->forward_duration;
                traj_item.time_from_start = traj_fifo_.timeFromStart() + traj_item.duration;
                traj_item.dynamics_product = pseg->dynamics_product;
                memcpy(traj_item.traj_coeff,  pseg->forward_coeff, sizeof(traj_item.traj_coeff));
                traj_fifo_.push(traj_item);
                sampleEndingTrajectorySegment(traj_item.traj_coeff, tmp_joint, pseg->ending_state.omega, pseg->ending_state.alpha);
                auto_pick_ptr_->cache[auto_pick_segment_ + 1].start_state = pseg->ending_state;

#ifdef PRINT_COEFFS
                FST_LOG("start time: %.12f", traj_fifo_.timeFromStart());

                for (size_t i = 0; i < 6; i++)
                {
                    FST_LOG("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", traj_item.traj_coeff[i].duration[0], traj_item.traj_coeff[i].duration[1], traj_item.traj_coeff[i].duration[2], traj_item.traj_coeff[i].duration[3],
                                                                                      traj_item.traj_coeff[i].duration[0] + traj_item.traj_coeff[i].duration[1] + traj_item.traj_coeff[i].duration[2] + traj_item.traj_coeff[i].duration[3]);
                    FST_LOG("  coeff = %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[0][3], traj_item.traj_coeff[i].coeff[0][2], traj_item.traj_coeff[i].coeff[0][1], traj_item.traj_coeff[i].coeff[0][0]);
                    FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[1][3], traj_item.traj_coeff[i].coeff[1][2], traj_item.traj_coeff[i].coeff[1][1], traj_item.traj_coeff[i].coeff[1][0]);
                    FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[2][3], traj_item.traj_coeff[i].coeff[2][2], traj_item.traj_coeff[i].coeff[2][1], traj_item.traj_coeff[i].coeff[2][0]);
                    FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[3][3], traj_item.traj_coeff[i].coeff[3][2], traj_item.traj_coeff[i].coeff[3][1], traj_item.traj_coeff[i].coeff[3][0]);
                }
#endif
            }
            else
            {
                if (pseg->backward_duration < 0 && auto_pick_segment_ + 1 < auto_pick_ptr_->tail && (pseg + 1)->backward_duration < 0)
                {
                    if (dynamics_cnt == 0)
                    {
                        computeDynamics(pseg->start_state.angle, pseg->start_state.omega, alpha_upper, alpha_lower, dynamics_product);
                        dynamics_cnt = dynamics_cnt_;

                        size_t joint_num = getNumberOfJoint();

                        for (size_t i = 0; i < joint_num; i++)
                        {
                            alpha_lower[i] = alpha_lower[i] * acc_ratio_;
                            alpha_upper[i] = alpha_upper[i] * acc_ratio_;
                        }
                    }

                    err = forwardCycle(pseg->start_state, pseg->ending_state.angle, auto_pick_ptr_->expect_duration, alpha_upper, alpha_lower, jerk_, pseg->forward_coeff);
                    dynamics_cnt = dynamics_cnt > 0 ? dynamics_cnt - 1: 0;

#ifdef OUTPUT_COEFF
                    g_cout[g_cindex].type = FORE_CYCLE;
                    g_cout[g_cindex].start = pseg->start_state;
                    g_cout[g_cindex].ending = pseg->ending_state;
                    g_cout[g_cindex].alpha_upper = alpha_upper;
                    g_cout[g_cindex].alpha_lower = alpha_lower;
                    g_cout[g_cindex].exp_duration = auto_pick_ptr_->expect_duration;
                    g_cout[g_cindex].duration = pseg->forward_coeff[0].duration[0] + pseg->forward_coeff[0].duration[1] + pseg->forward_coeff[0].duration[2] + pseg->forward_coeff[0].duration[3];
                    g_cout[g_cindex].start_time = traj_fifo_.timeFromStart();
                    memcpy(g_cout[g_cindex].segment, pseg->forward_coeff, sizeof(pseg->forward_coeff));
                    g_cindex = (g_cindex + 1) % COEFF_SIZE;
#endif
                    if (err == SUCCESS)
                    {
#ifdef CHECK_COEFF
                        if (!checkCoeff(pseg->forward_coeff, pseg->start_state, pseg->ending_state.angle, alpha_upper, alpha_lower, jerk_))
                        {
                            FST_ERROR("ERROR in fore-cycle-%d, exp-duration: %.12f", auto_pick_segment_, auto_pick_ptr_->expect_duration);
                            FST_ERROR("start-angle: %s", printDBLine(&pseg->start_state.angle[0], buffer, LOG_TEXT_SIZE));
                            FST_ERROR("start-omega: %s", printDBLine(&pseg->start_state.omega[0], buffer, LOG_TEXT_SIZE));
                            FST_ERROR("start-alpha: %s", printDBLine(&pseg->start_state.alpha[0], buffer, LOG_TEXT_SIZE));
                            FST_ERROR("target-angle: %s", printDBLine(&pseg->ending_state.angle[0], buffer, LOG_TEXT_SIZE));
                            FST_ERROR("alpha-upper: %s", printDBLine(&alpha_upper[0], buffer, LOG_TEXT_SIZE));
                            FST_ERROR("alpha-lower: %s", printDBLine(&alpha_lower[0], buffer, LOG_TEXT_SIZE));
                            FST_ERROR("jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));
                            FST_ERROR("exp-duration: %.12f", auto_pick_ptr_->expect_duration);

                            for (size_t i = 0; i < 6; i++)
                            {
                                FST_ERROR("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", pseg->forward_coeff[i].duration[0], pseg->forward_coeff[i].duration[1], pseg->forward_coeff[i].duration[2], pseg->forward_coeff[i].duration[3],
                                                                                                    pseg->forward_coeff[i].duration[0] + pseg->forward_coeff[i].duration[1] + pseg->forward_coeff[i].duration[2] + pseg->forward_coeff[i].duration[3]);
                                FST_ERROR("  coeff = %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[0][3], pseg->forward_coeff[i].coeff[0][2], pseg->forward_coeff[i].coeff[0][1], pseg->forward_coeff[i].coeff[0][0]);
                                FST_ERROR("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[1][3], pseg->forward_coeff[i].coeff[1][2], pseg->forward_coeff[i].coeff[1][1], pseg->forward_coeff[i].coeff[1][0]);
                                FST_ERROR("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[2][3], pseg->forward_coeff[i].coeff[2][2], pseg->forward_coeff[i].coeff[2][1], pseg->forward_coeff[i].coeff[2][0]);
                                FST_ERROR("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[3][3], pseg->forward_coeff[i].coeff[3][2], pseg->forward_coeff[i].coeff[3][1], pseg->forward_coeff[i].coeff[3][0]);
                            }

                            FST_ERROR("createTrajectory: internal fault !!!");
                            return MOTION_INTERNAL_FAULT;
                        }
#endif
#ifdef PRINT_COEFFS
                        FST_LOG("start-angle: %s", printDBLine(&pseg->start_state.angle[0], buffer, LOG_TEXT_SIZE));
                        FST_LOG("start-omega: %s", printDBLine(&pseg->start_state.omega[0], buffer, LOG_TEXT_SIZE));
                        FST_LOG("start-alpha: %s", printDBLine(&pseg->start_state.alpha[0], buffer, LOG_TEXT_SIZE));
                        FST_LOG("target-angle: %s", printDBLine(&pseg->ending_state.angle[0], buffer, LOG_TEXT_SIZE));
                        FST_LOG("alpha-upper: %s", printDBLine(&alpha_upper[0], buffer, LOG_TEXT_SIZE));
                        FST_LOG("alpha-lower: %s", printDBLine(&alpha_lower[0], buffer, LOG_TEXT_SIZE));
                        FST_LOG("jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));
                        FST_LOG("exp-duration: %.12f", auto_pick_ptr_->expect_duration);
                        FST_LOG("start time: %.12f", traj_fifo_.timeFromStart());

                        for (size_t i = 0; i < 6; i++)
                        {
                            FST_LOG("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", pseg->forward_coeff[i].duration[0], pseg->forward_coeff[i].duration[1], pseg->forward_coeff[i].duration[2], pseg->forward_coeff[i].duration[3],
                                                                                              pseg->forward_coeff[i].duration[0] + pseg->forward_coeff[i].duration[1] + pseg->forward_coeff[i].duration[2] + pseg->forward_coeff[i].duration[3]);
                            FST_LOG("  coeff = %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[0][3], pseg->forward_coeff[i].coeff[0][2], pseg->forward_coeff[i].coeff[0][1], pseg->forward_coeff[i].coeff[0][0]);
                            FST_LOG("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[1][3], pseg->forward_coeff[i].coeff[1][2], pseg->forward_coeff[i].coeff[1][1], pseg->forward_coeff[i].coeff[1][0]);
                            FST_LOG("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[2][3], pseg->forward_coeff[i].coeff[2][2], pseg->forward_coeff[i].coeff[2][1], pseg->forward_coeff[i].coeff[2][0]);
                            FST_LOG("          %.6f, %.6f, %.6f, %.6f", pseg->forward_coeff[i].coeff[3][3], pseg->forward_coeff[i].coeff[3][2], pseg->forward_coeff[i].coeff[3][1], pseg->forward_coeff[i].coeff[3][0]);
                        }
#endif
                        pseg->forward_duration = pseg->forward_coeff[0].duration[0] + pseg->forward_coeff[0].duration[1] + pseg->forward_coeff[0].duration[2] + pseg->forward_coeff[0].duration[3];
                        traj_item.id = pseg->path_point.id;
                        traj_item.duration = pseg->forward_duration;
                        traj_item.time_from_start = traj_fifo_.timeFromStart() + traj_item.duration;
                        traj_item.dynamics_product = dynamics_product;
                        memcpy(traj_item.traj_coeff,  pseg->forward_coeff, sizeof(traj_item.traj_coeff));
                        traj_fifo_.push(traj_item);
                        sampleEndingTrajectorySegment(traj_item.traj_coeff, tmp_joint, pseg->ending_state.omega, pseg->ending_state.alpha);
                        auto_pick_ptr_->cache[auto_pick_segment_ + 1].start_state = pseg->ending_state;
                    }
                    else
                    {
                        FST_ERROR("Fail to create trajectory from path, code = 0x%llx", err);
                        FST_ERROR("start-angle: %s", printDBLine(&pseg->start_state.angle[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("start-omega: %s", printDBLine(&pseg->start_state.omega[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("start-alpha: %s", printDBLine(&pseg->start_state.alpha[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("target-angle: %s", printDBLine(&pseg->ending_state.angle[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("alpha-upper: %s", printDBLine(&alpha_upper[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("alpha-lower: %s", printDBLine(&alpha_lower[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("exp-duration: %.12f", auto_pick_ptr_->expect_duration);

                        for (size_t i = 0; i < 6; i++)
                        {
                            FST_ERROR("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", traj_item.traj_coeff[i].duration[0], traj_item.traj_coeff[i].duration[1], traj_item.traj_coeff[i].duration[2], traj_item.traj_coeff[i].duration[3],
                                                                                                traj_item.traj_coeff[i].duration[0] + traj_item.traj_coeff[i].duration[1] + traj_item.traj_coeff[i].duration[2] + traj_item.traj_coeff[i].duration[3]);
                            FST_ERROR("  coeff = %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[0][3], traj_item.traj_coeff[i].coeff[0][2], traj_item.traj_coeff[i].coeff[0][1], traj_item.traj_coeff[i].coeff[0][0]);
                            FST_ERROR("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[1][3], traj_item.traj_coeff[i].coeff[1][2], traj_item.traj_coeff[i].coeff[1][1], traj_item.traj_coeff[i].coeff[1][0]);
                            FST_ERROR("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[2][3], traj_item.traj_coeff[i].coeff[2][2], traj_item.traj_coeff[i].coeff[2][1], traj_item.traj_coeff[i].coeff[2][0]);
                            FST_ERROR("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[3][3], traj_item.traj_coeff[i].coeff[3][2], traj_item.traj_coeff[i].coeff[3][1], traj_item.traj_coeff[i].coeff[3][0]);
                        }

                        FST_ERROR("createTrajectory: internal fault !!!");
                        return MOTION_INTERNAL_FAULT;
                    }
                }
                else if (pseg->backward_duration > 0 && (auto_pick_segment_ + 1 == auto_pick_ptr_->tail || (pseg + 1)->backward_duration > 0))
                {
                    dynamics_cnt = 0;
                    traj_item.id = pseg->path_point.id;
                    traj_item.duration = pseg->backward_duration;
                    traj_item.time_from_start = traj_fifo_.timeFromStart() + traj_item.duration;
                    traj_item.dynamics_product = pseg->dynamics_product;
                    memcpy(traj_item.traj_coeff,  pseg->backward_coeff, sizeof(traj_item.traj_coeff));
                    traj_fifo_.push(traj_item);
#ifdef PRINT_COEFFS
                    FST_LOG("start time: %.12f", traj_fifo_.timeFromStart());

                    for (size_t i = 0; i < 6; i++)
                    {
                        FST_LOG("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", traj_item.traj_coeff[i].duration[0], traj_item.traj_coeff[i].duration[1], traj_item.traj_coeff[i].duration[2], traj_item.traj_coeff[i].duration[3],
                                                                                          traj_item.traj_coeff[i].duration[0] + traj_item.traj_coeff[i].duration[1] + traj_item.traj_coeff[i].duration[2] + traj_item.traj_coeff[i].duration[3]);
                        FST_LOG("  coeff = %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[0][3], traj_item.traj_coeff[i].coeff[0][2], traj_item.traj_coeff[i].coeff[0][1], traj_item.traj_coeff[i].coeff[0][0]);
                        FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[1][3], traj_item.traj_coeff[i].coeff[1][2], traj_item.traj_coeff[i].coeff[1][1], traj_item.traj_coeff[i].coeff[1][0]);
                        FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[2][3], traj_item.traj_coeff[i].coeff[2][2], traj_item.traj_coeff[i].coeff[2][1], traj_item.traj_coeff[i].coeff[2][0]);
                        FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[3][3], traj_item.traj_coeff[i].coeff[3][2], traj_item.traj_coeff[i].coeff[3][1], traj_item.traj_coeff[i].coeff[3][0]);
                    }
#endif
                }
                else if (pseg->backward_duration < 0 && auto_pick_segment_ + 1 < auto_pick_ptr_->tail && (pseg + 1)->backward_duration > 0)
                {
                    //smooth
                    FST_WARN("Smooth fore-cycle with back-cycle, fore-duration = %.12f, back-duration = %.12f", (pseg - 1)->forward_duration, (pseg + 1)->backward_duration);

                    if (dynamics_cnt == 0)
                    {
                        computeDynamics(pseg->start_state.angle, pseg->start_state.omega, alpha_upper, alpha_lower, dynamics_product);
                        dynamics_cnt = dynamics_cnt_;

                        size_t joint_num = getNumberOfJoint();

                        for (size_t i = 0; i < joint_num; i++)
                        {
                            alpha_lower[i] = alpha_lower[i] * acc_ratio_;
                            alpha_upper[i] = alpha_upper[i] * acc_ratio_;
                        }
                    }

                    //smoothPoint2Point(pseg->start_state, (pseg + 1)->start_state, ((pseg - 1)->forward_duration + (pseg + 1)->backward_duration) / 2, alpha_upper, alpha_lower, jerk_, traj_item.traj_coeff);
                    smoothPoint2Point(pseg->start_state, (pseg + 1)->start_state, (pseg - 1)->forward_duration, alpha_upper, alpha_lower, jerk_, traj_item.traj_coeff);
                    dynamics_cnt = dynamics_cnt > 0 ? dynamics_cnt - 1: 0;

#ifdef OUTPUT_COEFF
                    g_cout[g_cindex].type = SMOOTH_CYCLE;
                    g_cout[g_cindex].start = pseg->start_state;
                    g_cout[g_cindex].ending = (pseg + 1)->start_state;
                    g_cout[g_cindex].alpha_upper = alpha_upper;
                    g_cout[g_cindex].alpha_lower = alpha_lower;
                    g_cout[g_cindex].exp_duration = ((pseg - 1)->forward_duration + (pseg + 1)->backward_duration) / 2;
                    g_cout[g_cindex].duration = traj_item.traj_coeff[0].duration[0] + traj_item.traj_coeff[0].duration[1] + traj_item.traj_coeff[0].duration[2] + traj_item.traj_coeff[0].duration[3];
                    g_cout[g_cindex].start_time = traj_fifo_.timeFromStart();
                    memcpy(g_cout[g_cindex].segment, traj_item.traj_coeff, sizeof(traj_item.traj_coeff));
                    g_cindex = (g_cindex + 1) % COEFF_SIZE;

#endif
#ifdef CHECK_COEFF
                    if (!checkCoeff(traj_item.traj_coeff, pseg->start_state, (pseg + 1)->start_state, alpha_upper, alpha_lower, jerk_))
                    {
                        //FST_ERROR("ERROR in smooth, exp-duration: %.12f", ((pseg - 1)->forward_duration + (pseg + 1)->backward_duration) / 2);
                        FST_ERROR("ERROR in smooth, exp-duration: %.12f", (pseg - 1)->forward_duration);
                        FST_ERROR("  start-angle: %s", printDBLine(&pseg->start_state.angle[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("  start-omega: %s", printDBLine(&pseg->start_state.omega[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("  ending-angle: %s", printDBLine(&(pseg + 1)->start_state.angle[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("  ending-omega: %s", printDBLine(&(pseg + 1)->start_state.omega[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("alpha-upper: %s", printDBLine(&alpha_upper[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("alpha-lower: %s", printDBLine(&alpha_lower[0], buffer, LOG_TEXT_SIZE));
                        FST_ERROR("jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));

                        for (size_t i = 0; i < 6; i++)
                        {
                            FST_ERROR("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", traj_item.traj_coeff[i].duration[0], traj_item.traj_coeff[i].duration[1], traj_item.traj_coeff[i].duration[2], traj_item.traj_coeff[i].duration[3],
                                                                                                traj_item.traj_coeff[i].duration[0] + traj_item.traj_coeff[i].duration[1] + traj_item.traj_coeff[i].duration[2] + traj_item.traj_coeff[i].duration[3]);
                            FST_ERROR("  coeff = %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[0][3], traj_item.traj_coeff[i].coeff[0][2], traj_item.traj_coeff[i].coeff[0][1], traj_item.traj_coeff[i].coeff[0][0]);
                            FST_ERROR("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[1][3], traj_item.traj_coeff[i].coeff[1][2], traj_item.traj_coeff[i].coeff[1][1], traj_item.traj_coeff[i].coeff[1][0]);
                            FST_ERROR("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[2][3], traj_item.traj_coeff[i].coeff[2][2], traj_item.traj_coeff[i].coeff[2][1], traj_item.traj_coeff[i].coeff[2][0]);
                            FST_ERROR("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[3][3], traj_item.traj_coeff[i].coeff[3][2], traj_item.traj_coeff[i].coeff[3][1], traj_item.traj_coeff[i].coeff[3][0]);
                        }

                        FST_ERROR("createTrajectory: internal fault !!!");
                        return MOTION_INTERNAL_FAULT;
                    }
#endif
#ifdef PRINT_COEFFS
                    FST_LOG("smooth, exp-duration = %.12f, avg-duration = %.12f", auto_cache_ptr_->expect_duration, ((pseg - 1)->forward_duration + (pseg + 1)->backward_duration) / 2);
                    FST_LOG("start angle: %s", printDBLine(&pseg->start_state.angle[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("start omega: %s", printDBLine(&pseg->start_state.omega[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("target angle: %s", printDBLine(&(pseg + 1)->start_state.angle[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("target omega: %s", printDBLine(&(pseg + 1)->start_state.omega[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("alpha-upper: %s", printDBLine(&alpha_upper[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("alpha-lower: %s", printDBLine(&alpha_lower[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("jerk: %s", printDBLine(&jerk_[0], buffer, LOG_TEXT_SIZE));
                    FST_LOG("start time: %.12f", traj_fifo_.timeFromStart());

                    for (size_t i = 0; i < 6; i++)
                    {
                        FST_LOG("  duration = %.12f, %.12f, %.12f, %.12f, total = %.12f", traj_item.traj_coeff[i].duration[0], traj_item.traj_coeff[i].duration[1], traj_item.traj_coeff[i].duration[2], traj_item.traj_coeff[i].duration[3],
                                                                                          traj_item.traj_coeff[i].duration[0] + traj_item.traj_coeff[i].duration[1] + traj_item.traj_coeff[i].duration[2] + traj_item.traj_coeff[i].duration[3]);
                        FST_LOG("  coeff = %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[0][3], traj_item.traj_coeff[i].coeff[0][2], traj_item.traj_coeff[i].coeff[0][1], traj_item.traj_coeff[i].coeff[0][0]);
                        FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[1][3], traj_item.traj_coeff[i].coeff[1][2], traj_item.traj_coeff[i].coeff[1][1], traj_item.traj_coeff[i].coeff[1][0]);
                        FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[2][3], traj_item.traj_coeff[i].coeff[2][2], traj_item.traj_coeff[i].coeff[2][1], traj_item.traj_coeff[i].coeff[2][0]);
                        FST_LOG("          %.6f, %.6f, %.6f, %.6f", traj_item.traj_coeff[i].coeff[3][3], traj_item.traj_coeff[i].coeff[3][2], traj_item.traj_coeff[i].coeff[3][1], traj_item.traj_coeff[i].coeff[3][0]);
                    }
#endif
                    traj_item.id = pseg->path_point.id;
                    traj_item.duration = traj_item.traj_coeff[0].duration[0] + traj_item.traj_coeff[0].duration[1] + traj_item.traj_coeff[0].duration[2] + traj_item.traj_coeff[0].duration[3];
                    traj_item.time_from_start = traj_fifo_.timeFromStart() + traj_item.duration;
                    traj_item.dynamics_product = dynamics_product;
                    traj_fifo_.push(traj_item);
                    sampleEndingTrajectorySegment(traj_item.traj_coeff, tmp_joint, pseg->ending_state.omega, pseg->ending_state.alpha);
                    auto_pick_ptr_->cache[auto_pick_segment_ + 1].start_state = pseg->ending_state;
                }
                else
                {
                    FST_ERROR("createTrajectory: internal fault !!!");
                    return MOTION_INTERNAL_FAULT;
                }
            }

            auto_pick_segment_ ++;
        }
    }

    return SUCCESS;
}

void BaseGroup::getLatestJoint(Joint &joint)
{
    pthread_mutex_lock(&servo_mutex_);
    joint = current_joint_;
    pthread_mutex_unlock(&servo_mutex_);
}

Joint BaseGroup::getLatestJoint(void)
{
    pthread_mutex_lock(&servo_mutex_);
    Joint joint(current_joint_);
    pthread_mutex_unlock(&servo_mutex_);
    return joint;
}

void BaseGroup::getServoState(ServoState &state)
{
    pthread_mutex_lock(&servo_mutex_);
    state = servo_state_;
    pthread_mutex_unlock(&servo_mutex_);
}

ServoState BaseGroup::getServoState(void)
{
    pthread_mutex_lock(&servo_mutex_);
    ServoState state(servo_state_);
    pthread_mutex_unlock(&servo_mutex_);
    return state;
}

void BaseGroup::getGroupState(GroupState &state)
{
    state = group_state_;
}

GroupState BaseGroup::getGroupState(void)
{
    return group_state_;
}

/*
ErrorCode BaseGroup::getJointFromPose(const PoseEuler &pose, Joint &joint)
{
    ErrorCode err;

    switch (manual_frame_)
    {
        case BASE:
            err = kinematics_ptr_->inverseKinematicsInBase(pose, getLatestJoint(), joint);
            break;
        case USER:
            err = kinematics_ptr_->inverseKinematicsInUser(pose, getLatestJoint(), joint);
            break;
        case WORLD:
            err = kinematics_ptr_->inverseKinematicsInUser(pose, getLatestJoint(), joint);
            break;
        default:
            FST_ERROR("getJointFromPose: motion-frame is invalid: %d", manual_frame_);
            err = INVALID_SEQUENCE;
            break;
    }

    return err;
}

ErrorCode BaseGroup::getPoseFromJoint(const Joint &joint, PoseEuler &pose)
{
    switch (manual_frame_)
    {
        case BASE:
            kinematics_ptr_->forwardKinematicsInBase(joint, pose);
            return SUCCESS;
        case USER:
            kinematics_ptr_->forwardKinematicsInUser(joint, pose);
            return SUCCESS;
        case WORLD:
            kinematics_ptr_->forwardKinematicsInWorld(joint, pose);
            return SUCCESS;
        default:
            FST_ERROR("getPoseFromJoint: motion-frame is invalid: %d", manual_frame_);
            return INVALID_SEQUENCE;
    }
}

ErrorCode BaseGroup::getPoseFromJointInBase(const Joint &joint, PoseEuler &pose)
{
    kinematics_ptr_->forwardKinematicsInBase(joint, pose);
    return SUCCESS;
}

ErrorCode BaseGroup::getPoseFromJointInUser(const Joint &joint, PoseEuler &pose)
{
    kinematics_ptr_->forwardKinematicsInUser(joint, pose);
    return SUCCESS;
}

ErrorCode BaseGroup::getPoseFromJointInWorld(const Joint &joint, PoseEuler &pose)
{
    kinematics_ptr_->forwardKinematicsInWorld(joint, pose);
    return SUCCESS;
}
*/

ErrorCode BaseGroup::setGlobalVelRatio(double ratio)
{
    FST_INFO("Set global velocity ratio: %.4f", ratio);

    if (ratio < 0 || ratio > 1)
    {
        FST_ERROR("Given ratio out of range (0, 1)");
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
    FST_INFO("Set global acceleration ratio: %.4f", ratio);

    if (ratio < 0 || ratio > 1)
    {
        FST_ERROR("Given ratio out of range (0, 1)");
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

ErrorCode BaseGroup::setToolFrame(const PoseEuler &tf)
{
    return kinematics_ptr_->setToolFrame(tf);
}

ErrorCode BaseGroup::setUserFrame(const PoseEuler &uf)
{
    return kinematics_ptr_->setUserFrame(uf);
}

ErrorCode BaseGroup::setWorldFrame(const PoseEuler &wf)
{
    return kinematics_ptr_->setUserFrame(wf);
}

ErrorCode BaseGroup::sendPoint(void)
{
    ErrorCode err;

    if (group_state_ == MANUAL || group_state_ == MANUAL_TO_STANDBY)
    {
        if (bare_core_.isPointCacheEmpty())
        {
            size_t length = 10;
            TrajectoryPoint point[10];
            err = pickFromManual(point, length);

            if (err != SUCCESS)
            {
                FST_ERROR("sendPoint: cannot pick point from manual motion.");
                return err;
            }

            bare_core_.fillPointCache(point, length, POINT_POS);
        }

        return bare_core_.sendPoint() ? SUCCESS : BARE_CORE_TIMEOUT;
    }
    else if (group_state_ == AUTO || group_state_ == AUTO_TO_STANDBY || group_state_ == AUTO_TO_PAUSE)
    {
        if (bare_core_.isPointCacheEmpty())
        {
            size_t length = 10;
            TrajectoryPoint point[10];
            err = pickFromAuto(point, length);

            if (err != SUCCESS)
            {
                FST_ERROR("sendPoint: cannot pick point from auto motion.");
                return err;
            }

            bare_core_.fillPointCache(point, length, POINT_POS_VEL_ACC_EFF);
        }

        return bare_core_.sendPoint() ? SUCCESS : BARE_CORE_TIMEOUT;
    }
    else
    {
        return INVALID_SEQUENCE;
    }
}

ErrorCode BaseGroup::pickFromManual(TrajectoryPoint *point, size_t &length)
{
    return (manual_traj_.frame == JOINT || manual_traj_.mode == APOINT) ? pickFromManualJoint(point, length) : pickFromManualCartesian(point, length);
}

ErrorCode BaseGroup::pickFromManualJoint(TrajectoryPoint *point, size_t &length)
{
    size_t cnt = 0;
    double *angle, *start, *target;
    double tm, omega;

    FST_INFO("Pick from manual joint, manual-time = %.4f", manual_time_);

    for (size_t i = 0 ; i < length; i++)
    {
        point[i].level = manual_time_ > MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
        memset(&point[i].omega, 0, sizeof(Joint));
        memset(&point[i].alpha, 0, sizeof(Joint));
        //memset(&point[i].inertia, 0, sizeof(Joint));
        //memset(&point[i].gravity, 0, sizeof(Joint));
        memset(&point[i].ma_cv_g, 0, sizeof(Joint));

        angle  = (double*)&point[i].angle;
        start  = (double*)&manual_traj_.joint_start;
        target = (double*)&manual_traj_.joint_ending;

        manual_time_ += cycle_time_;

        for (size_t jnt = 0; jnt < JOINT_OF_ARM; jnt++)
        {
            if (manual_time_ < manual_traj_.coeff[jnt].start_time)
            {
                *angle = *start;
            }
            else if (manual_time_ < manual_traj_.coeff[jnt].stable_time)
            {
                tm = manual_time_ - manual_traj_.coeff[jnt].start_time;
                *angle = *start + manual_traj_.coeff[jnt].start_alpha * tm * tm / 2;
            }
            else if (manual_time_ < manual_traj_.coeff[jnt].brake_time)
            {
                tm = manual_traj_.coeff[jnt].stable_time - manual_traj_.coeff[jnt].start_time;
                omega = manual_traj_.coeff[jnt].start_alpha * tm;
                *angle = *start + omega * tm / 2;
                tm = manual_time_ - manual_traj_.coeff[jnt].stable_time;
                *angle = *angle + omega * tm;
            }
            else if (manual_time_ < manual_traj_.coeff[jnt].stop_time)
            {
                tm = manual_traj_.coeff[jnt].stable_time - manual_traj_.coeff[jnt].start_time;
                omega = manual_traj_.coeff[jnt].start_alpha * tm;
                *angle = *start + omega * tm / 2;
                tm = manual_traj_.coeff[jnt].brake_time - manual_traj_.coeff[jnt].stable_time;
                *angle = *angle + omega * tm;
                tm = manual_time_ - manual_traj_.coeff[jnt].brake_time;
                *angle = *angle + omega * tm + manual_traj_.coeff[jnt].brake_alpha * tm * tm / 2;
            }
            else
            {
                *angle = *target;
                *start = *target;
            }

            ++ angle;
            ++ start;
            ++ target;
        }

        cnt ++;

        if (manual_time_ >= manual_traj_.duration)
        {
            point[i].level = POINT_ENDING;
            FST_INFO("%d - %.4f - %.6f %.6f %.6f %.6f %.6f %.6f", point[i].level, manual_time_,
                     point[i].angle.j1, point[i].angle.j2,  point[i].angle.j3,
                     point[i].angle.j4, point[i].angle.j5,  point[i].angle.j6);

            manual_traj_.direction[0] = STANDING;
            manual_traj_.direction[1] = STANDING;
            manual_traj_.direction[2] = STANDING;
            manual_traj_.direction[3] = STANDING;
            manual_traj_.direction[4] = STANDING;
            manual_traj_.direction[5] = STANDING;
            manual_traj_.duration = 0;
            memset(manual_traj_.coeff, 0, JOINT_OF_ARM * sizeof(ManualCoef));
            start_joint_ = manual_traj_.joint_ending;
            manual_time_ = 0;
            group_state_ = MANUAL_TO_STANDBY;
            break;
        }
        else
        {
            //FST_INFO("%d - %.3f - %.6f %.6f %.6f %.6f %.6f %.6f", point[i].level, manual_time_,
            //              point[i].angle.j1, point[i].angle.j2, point[i].angle.j3,
            //              point[i].angle.j4, point[i].angle.j5, point[i].angle.j6);
            continue;
        }
    }

    length = cnt;
    return SUCCESS;
}

ErrorCode BaseGroup::pickFromManualCartesian(TrajectoryPoint *point, size_t &length)
{
    ErrorCode err = SUCCESS;
    PoseEuler pose;
    double tim, vel;
    double *axis, *start, *target;
    size_t cnt = 0;

    FST_INFO("Pick from manual cartesian, manual-time = %.4f", manual_time_);

    for (size_t i = 0; i < length; i++)
    {
        point[i].level = manual_time_ > MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
        memset(&point[i].omega, 0, sizeof(Joint));
        memset(&point[i].alpha, 0, sizeof(Joint));
        memset(&point[i].ma_cv_g, 0, sizeof(Joint));
        //memset(&point[i].inertia, 0, sizeof(Joint));
        //memset(&point[i].gravity, 0, sizeof(Joint));

        axis = (double *) &pose.position.x;
        start = (double *) &manual_traj_.cart_start.position.x;
        target = (double *) &manual_traj_.cart_ending.position.x;

        manual_time_ += cycle_time_;

        for (size_t i = 0; i < 6; i++)
        {
            if (manual_time_ < manual_traj_.coeff[i].start_time)
            {
                *axis = *start;
            }
            else if (manual_time_ < manual_traj_.coeff[i].stable_time)
            {
                tim = manual_time_ - manual_traj_.coeff[i].start_time;
                *axis = *start + manual_traj_.coeff[i].start_alpha * tim * tim / 2;
            }
            else if (manual_time_ < manual_traj_.coeff[i].brake_time)
            {
                tim = manual_traj_.coeff[i].stable_time - manual_traj_.coeff[i].start_time;
                vel = manual_traj_.coeff[i].start_alpha * tim;
                *axis = *start + vel * tim / 2;
                tim = manual_time_ - manual_traj_.coeff[i].stable_time;
                *axis = *axis + vel * tim;
            }
            else if (manual_time_ < manual_traj_.coeff[i].stop_time)
            {
                tim = manual_traj_.coeff[i].stable_time - manual_traj_.coeff[i].start_time;
                vel = manual_traj_.coeff[i].start_alpha * tim;
                *axis = *start + vel * tim / 2;
                tim = manual_traj_.coeff[i].brake_time - manual_traj_.coeff[i].stable_time;
                *axis = *axis + vel * tim;
                tim = manual_time_ - manual_traj_.coeff[i].brake_time;
                *axis = *axis + vel * tim + manual_traj_.coeff[i].brake_alpha * tim * tim / 2;
            }
            else
            {
                *axis = *target;
                *start = *target;
            }

            ++axis;
            ++start;
            ++target;
        }

        Joint ref_joint = getLatestJoint();

        switch (manual_traj_.frame)
        {
            case BASE:
                err = kinematics_ptr_->inverseKinematicsInBase(pose, ref_joint, point[i].angle);
                break;
            case USER:
                err = kinematics_ptr_->inverseKinematicsInUser(pose, ref_joint, point[i].angle);
                break;
            case WORLD:
                err = kinematics_ptr_->inverseKinematicsInWorld(pose, ref_joint, point[i].angle);
                break;
            case TOOL:
                err = kinematics_ptr_->inverseKinematicsInTool(manual_traj_.tool_coordinate, pose, ref_joint, point[i].angle);
                break;
            case JOINT:
            default:
                err = MOTION_INTERNAL_FAULT;
                break;
        }

        if (err == SUCCESS && soft_constraint_.isJointInConstraint(point[i].angle))
        {
            cnt++;

            if (manual_time_ >= manual_traj_.duration)
            {
                point[i].level = POINT_ENDING;
                FST_INFO("%d - %.4f - %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", point[i].level, manual_time_,
                         point[i].angle.j1, point[i].angle.j2, point[i].angle.j3, point[i].angle.j4, point[i].angle.j5, point[i].angle.j6);

                manual_traj_.direction[0] = STANDING;
                manual_traj_.direction[1] = STANDING;
                manual_traj_.direction[2] = STANDING;
                manual_traj_.direction[3] = STANDING;
                manual_traj_.direction[4] = STANDING;
                manual_traj_.direction[5] = STANDING;
                manual_traj_.duration = 0;
                memset(manual_traj_.coeff, 0, 6 * sizeof(ManualCoef));
                start_joint_ = point[i].angle;
                manual_time_ = 0;
                group_state_ = MANUAL_TO_STANDBY;
                break;
            }
            else
            {
                //FST_INFO("%d - %.4f - %.6f %.6f %.6f %.6f %.6f %.6f", point[i].level, manual_time_,
                //         point[i].angle.j1, point[i].angle.j2, point[i].angle.j3,
                //         point[i].angle.j4, point[i].angle.j5, point[i].angle.j6);
                continue;
            }
        }
        else
        {
            if (err != SUCCESS)
            {
                FST_ERROR("pickFromManualCartesian: IK failed.");
                break;
            }
            else
            {
                FST_ERROR("pickFromManualCartesian: IK result out of soft constraint.");
                err = JOINT_OUT_OF_CONSTRAINT;
                break;
            }
        }
    }

    length = cnt;
    return err;
}




ErrorCode BaseGroup::pickFromAuto(TrajectoryPoint *point, size_t &length)
{
    MotionTime  seg_tm;
    ErrorCode   err;
    size_t      pick_num = 0;

    FST_LOG("Pick from auto, auto-time=%.4f, time-form-start of traj-fifo=%.4f", auto_time_, traj_fifo_.timeFromStart());

    for (size_t i = 0; i < length; i++)
    {
        auto_time_ += cycle_time_;

        while (!traj_fifo_.empty() && auto_time_ > traj_fifo_.front().time_from_start)
        {
            traj_fifo_.dropFront();
        }

        err = createTrajectory();

        if (err != SUCCESS)
        {
            FST_ERROR("pickFromAuto: Fail to create trajectory.");
            return err;
        }

        if (!traj_fifo_.empty())
        {
            if (auto_time_ < traj_fifo_.front().time_from_start)
            {
                point[i].level = auto_time_ > cycle_time_ + MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
                seg_tm = auto_time_ - (traj_fifo_.front().time_from_start - traj_fifo_.front().duration);

                err = sampleTrajectorySegment(traj_fifo_.front().traj_coeff, seg_tm, point[i].angle, point[i].omega, point[i].alpha);
                computeCompensate(traj_fifo_.front().dynamics_product, point[i].omega, point[i].alpha, point[i].ma_cv_g);

#ifdef OUTPUT_JOUT
                g_jout[g_jindex].time = auto_time_;
                g_jout[g_jindex].ma_cv_g = point[i].ma_cv_g;
                g_jout[g_jindex].point.angle = point[i].angle;
                g_jout[g_jindex].point.omega = point[i].omega;
                g_jout[g_jindex].point.alpha = point[i].alpha;
                g_jindex = (g_jindex + 1) % JOUT_SIZE;
#endif

                //FST_LOG ("%.4f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", auto_time_,
                //         point[i].angle[0], point[i].angle[1], point[i].angle[2], point[i].angle[3], point[i].angle[4], point[i].angle[5],
                //         point[i].omega[0], point[i].omega[1], point[i].omega[2], point[i].omega[3], point[i].omega[4], point[i].omega[5],
                //         point[i].alpha[0], point[i].alpha[1], point[i].alpha[2], point[i].alpha[3], point[i].alpha[4], point[i].alpha[5]);

                if (err == SUCCESS)
                {
                    pick_num ++;
                }
                else
                {
                    FST_ERROR("pickFromAuto: sampling error! auto-time = %.4f, time-from-start = %.4f, duration = %.4f",
                              auto_time_, traj_fifo_.front().time_from_start, traj_fifo_.front().duration);
                    length = pick_num;
                    return err;
                }
            }
            else
            {
                FST_ERROR("pickFromAuto: auto-time error! auto-time = %.4f, time-from-start = %.4f, duration = %.4f",
                          auto_time_, traj_fifo_.front().time_from_start, traj_fifo_.front().duration);
                length = pick_num;
                return MOTION_INTERNAL_FAULT;
            }
        }
        else
        {
            err = sampleEndingTrajectorySegment(traj_fifo_.front().traj_coeff, point[i].angle, point[i].omega, point[i].alpha);
            computeCompensate(traj_fifo_.front().dynamics_product, point[i].omega, point[i].alpha, point[i].ma_cv_g);

#ifdef OUTPUT_JOUT
            g_jout[g_jindex].time = auto_time_;
            g_jout[g_jindex].ma_cv_g = point[i].ma_cv_g;
            g_jout[g_jindex].point.angle = point[i].angle;
            g_jout[g_jindex].point.omega = point[i].omega;
            g_jout[g_jindex].point.alpha = point[i].alpha;
            g_jindex = (g_jindex + 1) % JOUT_SIZE;
#endif

            //FST_LOG ("%.4f: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", auto_time_,
            //         point[i].angle[0], point[i].angle[1], point[i].angle[2], point[i].angle[3], point[i].angle[4], point[i].angle[5],
            //         point[i].omega[0], point[i].omega[1], point[i].omega[2], point[i].omega[3], point[i].omega[4], point[i].omega[5],
            //         point[i].alpha[0], point[i].alpha[1], point[i].alpha[2], point[i].alpha[3], point[i].alpha[4], point[i].alpha[5]);

            if (err == SUCCESS)
            {
                memset(&point[i].omega, 0, sizeof(point[i].omega));
                memset(&point[i].alpha, 0, sizeof(point[i].alpha));
                point[i].level = POINT_ENDING;
                pick_num ++;
            }
            else
            {
                FST_ERROR("pickFromAuto: sampling error! auto-time = %.4f, time-from-start = %.4f, duration = %.4f",
                          auto_time_, traj_fifo_.front().time_from_start, traj_fifo_.front().duration);
                length = pick_num;
                return err;
            }

            char buffer[LOG_TEXT_SIZE];
            FST_INFO("%d - %.4f - %s", point[i].level, auto_time_, printDBLine(&point[i].angle.j1, buffer, LOG_TEXT_SIZE));
            FST_INFO("Command ID = %d finished", traj_fifo_.front().id);
            //auto_running_ = false;

            if (group_state_ == AUTO)
            {
                group_state_ = AUTO_TO_STANDBY;
            }

            traj_fifo_.clear();

            break;
        }
    }

    length = pick_num;
    return SUCCESS;
}

ErrorCode BaseGroup::computeCompensate(const DynamicsProduct &product, const Joint &omega, const Joint &alpha, Joint &ma_cv_g)
{
    double ma, cv;
    size_t joint_num = getNumberOfJoint();

    for (size_t i = 0; i < joint_num; i++)
    {
        ma = 0;
        cv = 0;

        for (size_t j = 0; j < joint_num; j++)
        {
            ma += product.m[i][j] * alpha[j];
            cv += product.c[i][j] * omega[j];
        }

        ma_cv_g[i] = ma + cv + product.g[i];
    }

    return SUCCESS;
}


ErrorCode BaseGroup::sampleStartTrajectorySegment(const TrajSegment (&segment)[NUM_OF_JOINT], Joint &angle, Joint &omega, Joint &alpha)
{
    size_t ind;
    size_t joint_num = getNumberOfJoint();

    //FST_INFO("sampleStartTrajectorySegment: d0 = %.12f, d1 = %.12f, d2 = %.12f, d3 = %.12f", segment[0].duration[0], segment[0].duration[1], segment[0].duration[2], segment[0].duration[3]);
    for (size_t j = 0; j < joint_num; j++)
    {
        const double (&coeff)[4][4] = segment[j].coeff;
        const double (&duration)[4] = segment[j].duration;

        if (duration[0] > MINIMUM_E9) { ind = 0; goto sample_by_time; }
        if (duration[1] > MINIMUM_E9) { ind = 1; goto sample_by_time; }
        if (duration[2] > MINIMUM_E9) { ind = 2; goto sample_by_time; }
        if (duration[3] > MINIMUM_E9) { ind = 3; goto sample_by_time; }

        FST_ERROR("Time error! d0 = %.12f, d1 = %.12f, d2 = %.12f, d3 = %.12f", duration[0], duration[1], duration[2], duration[3]);
        return MOTION_INTERNAL_FAULT;

sample_by_time:
        angle[j] = coeff[ind][0];
        omega[j] = coeff[ind][1];
        alpha[j] = coeff[ind][2] * 2;
    }

    return SUCCESS;
}

ErrorCode BaseGroup::sampleEndingTrajectorySegment(const TrajSegment (&segment)[NUM_OF_JOINT], Joint &angle, Joint &omega, Joint &alpha)
{
    size_t ind;
    size_t joint_num = getNumberOfJoint();

    //FST_INFO("sampleEndingTrajectorySegment: d0 = %.12f, d1 = %.12f, d2 = %.12f, d3 = %.12f", segment[0].duration[0], segment[0].duration[1], segment[0].duration[2], segment[0].duration[3]);
    for (size_t j = 0; j < joint_num; j++)
    {
        const double (&coeff)[4][4] = segment[j].coeff;
        const double (&duration)[4] = segment[j].duration;

        if (duration[3] > MINIMUM_E9) { ind = 3; goto sample_by_time; }
        if (duration[2] > MINIMUM_E9) { ind = 2; goto sample_by_time; }
        if (duration[1] > MINIMUM_E9) { ind = 1; goto sample_by_time; }
        if (duration[0] > MINIMUM_E9) { ind = 0; goto sample_by_time; }


        FST_ERROR("Time error! d0 = %.12f, d1 = %.12f, d2 = %.12f, d3 = %.12f", duration[0], duration[1], duration[2], duration[3]);
        return MOTION_INTERNAL_FAULT;

sample_by_time:
        double tm_array[4] = {1.0, duration[ind], duration[ind] * duration[ind], duration[ind] * duration[ind] * duration[ind]};
        angle[j] = coeff[ind][3] * tm_array[3] + coeff[ind][2] * tm_array[2] + coeff[ind][1] * tm_array[1] + coeff[ind][0];
        omega[j] = coeff[ind][3] * tm_array[2] * 3 + coeff[ind][2] * tm_array[1] * 2 + coeff[ind][1];
        alpha[j] = coeff[ind][3] * tm_array[1] * 6 + coeff[ind][2] * 2;
    }

    return SUCCESS;
}

ErrorCode BaseGroup::sampleTrajectorySegment(const TrajSegment (&segment)[NUM_OF_JOINT], double time, Joint &angle, Joint &omega, Joint &alpha)
{
    size_t ind;
    size_t joint_num = getNumberOfJoint();

    for (size_t j = 0; j < joint_num; j++)
    {
        double tm = time;
        const double (&coeff)[4][4] = segment[j].coeff;
        const double (&duration)[4] = segment[j].duration;

        if (tm < duration[0] + MINIMUM_E9) { ind = 0; goto sample_by_time; }
        tm -= duration[0];
        if (tm < duration[1] + MINIMUM_E9) { ind = 1; goto sample_by_time; }
        tm -= duration[1];
        if (tm < duration[2] + MINIMUM_E9) { ind = 2; goto sample_by_time; }
        tm -= duration[2];
        if (tm < duration[3] + MINIMUM_E9) { ind = 3; goto sample_by_time; }

        FST_ERROR("Time error! time = %.4f, duration0 = %.4f, duration1 = %.4f, duration2 = %.4f, duration3 = %.4f",
                  time, duration[0], duration[1], duration[2], duration[3]);
        return MOTION_INTERNAL_FAULT;

sample_by_time:
        MotionTime tm_array[4] = {1.0, tm, tm * tm, tm * tm * tm};

        angle[j] = coeff[ind][3] * tm_array[3] + coeff[ind][2] * tm_array[2] + coeff[ind][1] * tm_array[1] + coeff[ind][0];
        omega[j] = coeff[ind][3] * tm_array[2] * 3 + coeff[ind][2] * tm_array[1] * 2 + coeff[ind][1];
        alpha[j] = coeff[ind][3] * tm_array[1] * 6 + coeff[ind][2] * 2;
    }

    return SUCCESS;
}



void BaseGroup::realtimeTask(void)
{
    char buffer[LOG_TEXT_SIZE];
    ErrorCode   err;
    ServoState  barecore_state;
    Joint       barecore_joint;
    PoseEuler   barecore_pose;
    size_t  send_fail_cnt = 0;
    size_t  stable_cnt = 0;
    timeval this_time, last_time;
    timeval t0, t1, t2, t3;

    FST_WARN("Realtime task start.");
    memset(&barecore_joint, 0, sizeof(barecore_joint));

    gettimeofday(&last_time, NULL);
    gettimeofday(&this_time, NULL);

    while (rt_task_active_)
    {
        gettimeofday(&this_time, NULL);
        gettimeofday(&t0, NULL);

        long delay = (this_time.tv_sec - last_time.tv_sec) * 1000 + this_time.tv_usec - last_time.tv_usec;

        if (delay > 90000)
        {
            FST_ERROR(" ++++ RT task delayed %d ms !!!!!!", delay / 1000);
            FST_ERROR("%d.%d - %d.%d", this_time.tv_sec, this_time.tv_usec, last_time.tv_sec, last_time.tv_usec);
            reportError(MOTION_INTERNAL_FAULT);
        }

        last_time = this_time;

        gettimeofday(&t2, NULL);
        if (bare_core_.getLatestJoint(barecore_joint, barecore_state))
        {
            pthread_mutex_lock(&servo_mutex_);
            servo_state_ = barecore_state;
            current_joint_ = barecore_joint;
            pthread_mutex_unlock(&servo_mutex_);
        }
        else
        {
            FST_ERROR("Lost communication with bare core.");
            reportError(BARE_CORE_TIMEOUT);
        }
        gettimeofday(&t3, NULL);

        delay = (t3.tv_sec - t2.tv_sec) * 1000 + t3.tv_usec - t2.tv_usec;

        if (delay > 5000)
        {
            FST_ERROR(" ++++ RT task part1 %d ms !!!!!!", delay / 1000);
            FST_ERROR("%d.%d - %d.%d", t3.tv_sec, t3.tv_usec, t2.tv_sec, t2.tv_usec);
        }

        if (waiting_fine_)
        {
            FST_LOG("waiting fine ... type = %d, cnt = %d, grp-state=%d, servo-state=%d", waiting_motion_type_, start_waiting_cnt_, group_state_, servo_state_);

            if (start_waiting_cnt_ > 0)
            {
                start_waiting_cnt_ --;
            }
            else
            {
                if (group_state_ == STANDBY && servo_state_ == SERVO_IDLE)
                {
                    if (waiting_motion_type_ == MOTION_JOINT)
                    {
                        FST_LOG("waiting joint = %s", printDBLine(&waiting_joint_[0], buffer, LOG_TEXT_SIZE));
                        FST_LOG("current joint = %s", printDBLine(&barecore_joint[0], buffer, LOG_TEXT_SIZE));

                        if (isSameJointForFine(waiting_joint_, barecore_joint))
                        {
                            stable_cnt ++;
                        }
                        else
                        {
                            stable_cnt = 0;
                        }
                    }
                    else
                    {
                        kinematics_ptr_->forwardKinematicsInUser(barecore_joint, barecore_pose);
                        FST_LOG("waiting pose = %s", printDBLine(&waiting_pose_[0], buffer, LOG_TEXT_SIZE));
                        FST_LOG("current pose = %s", printDBLine(&barecore_pose[0], buffer, LOG_TEXT_SIZE));

                        if (getDistance(barecore_pose.position, waiting_pose_.position) < 0.05)
                        {
                            stable_cnt ++;
                        }
                        else
                        {
                            stable_cnt = 0;
                        }
                    }

                    if (stable_cnt > 5)
                    {
                        FST_WARN("Waiting-fine: group is stable.");
                        waiting_fine_ = false;
                    }
                }
            }
        }

        if ((servo_state_ == SERVO_IDLE || servo_state_ == SERVO_RUNNING) &&
            (group_state_ == MANUAL || group_state_ == MANUAL_TO_STANDBY || group_state_ == AUTO || group_state_ == AUTO_TO_STANDBY))
        {
            gettimeofday(&t2, NULL);
            err = sendPoint();
            gettimeofday(&t3, NULL);

            delay = (t3.tv_sec - t2.tv_sec) * 1000 + t3.tv_usec - t2.tv_usec;

            if (delay > 50000)
            {
                FST_ERROR(" ++++ RT task part2 %d ms !!!!!!", delay / 1000);
                FST_ERROR("%d.%d - %d.%d", t3.tv_sec, t3.tv_usec, t2.tv_sec, t2.tv_usec);
            }

            if (err == SUCCESS)
            {
                send_fail_cnt = 0;

                if (group_state_ == MANUAL_TO_STANDBY)
                {
                    group_state_ = STANDBY;
                }
                else if (group_state_ == AUTO_TO_STANDBY)
                {
                    group_state_ = STANDBY;
                }
            }
            else if (err == BARE_CORE_TIMEOUT)
            {
                send_fail_cnt++;

                if (send_fail_cnt > 30)
                {
                    send_fail_cnt = 0;
                    FST_ERROR("Cannot send point to bare core, group-state = %d", group_state_);
                    reportError(err);
                }
            }
            else
            {
                FST_ERROR("realtime-task: Fail to send point, code = %llx", err);
                reportError(err);
            }
        }

        /*
        cnt ++;
        if (cnt > 200)
        {
            cnt = 0;
            FST_INFO("servo-state=%d, joint=%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", servo_state_,
                     current_joint_[0], current_joint_[1], current_joint_[2], current_joint_[3], current_joint_[4], current_joint_[5]);
        }
         */
        gettimeofday(&t1, NULL);

        long tm = (t1.tv_sec - t0.tv_sec) * 1000 + t1.tv_usec - t0.tv_usec;

        if (tm > 60000)
        {
            FST_ERROR(" ----- RT task cycle time: %d ms !!!!!!", tm / 1000);
            FST_ERROR("%d.%d - %d.%d", t1.tv_sec, t1.tv_usec, t0.tv_sec, t0.tv_usec);
            reportError(MOTION_INTERNAL_FAULT);
        }

        usleep(2000);
    }

    FST_WARN("Realtime task quit.");
}

bool BaseGroup::isSameJointForFine(const Joint &joint1, const Joint &joint2)
{
    size_t  joint_num = 5;

    for (size_t i = 0; i < joint_num; i++)
    {
        if (fabs(joint1[i] - joint2[i]) > 0.0001)
        {
            return false;
        }
    }

    return true;
}

bool BaseGroup::isSameJoint(const Joint &joint1, const Joint &joint2)
{
    size_t  joint_num = getNumberOfJoint();

    for (size_t i = 0; i < joint_num; i++)
    {
        if (fabs(joint1[i] - joint2[i]) > 0.0001)
        {
            return false;
        }
    }

    return true;
}

void BaseGroup::inactiveRealtimeTask(void)
{
    rt_task_active_ = false;
}

void BaseGroup::activeRealtimeTask(void)
{
    rt_task_active_ = true;
}

BaseKinematics* BaseGroup::getKinematicsPtr(void)
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
    FST_INFO("Get soft constraint:");
    FST_INFO("  lower = %s", printDBLine(&soft_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&soft_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::getFirmConstraint(JointConstraint &firm_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    firm_constraint_.getConstraint(firm_constraint);
    FST_INFO("Get firm constraint.");
    FST_INFO("  lower = %s", printDBLine(&firm_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&firm_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::getHardConstraint(JointConstraint &hard_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    hard_constraint_.getConstraint(hard_constraint);
    FST_INFO("Get hard constraint.");
    FST_INFO("  lower = %s", printDBLine(&hard_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&hard_constraint.upper[0], buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}


ErrorCode BaseGroup::setSoftConstraint(const JointConstraint &soft_constraint)
{
    Joint lower, upper;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set soft constraint.");

    soft_constraint_.getConstraint(lower, upper);
    FST_INFO("Origin soft constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    FST_INFO("Given soft constraint:");
    FST_INFO("  lower = %s", printDBLine(&soft_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&soft_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    firm_constraint_.getConstraint(lower, upper);
    FST_INFO("Firm constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    if (firm_constraint_.isCoverConstaint(soft_constraint))
    {
        ParamGroup param;
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
            param.dumpParamFile())
        {
            soft_constraint_.setConstraint(soft_constraint);
            FST_INFO("Soft constraint updated successfully.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail dumping soft constraint to config file");
            return param.getLastError();
        }
    }
    else
    {
        FST_ERROR("Given soft constraint out of firm constraint.");
        return INVALID_PARAMETER;
    }
}


ErrorCode BaseGroup::setFirmConstraint(const JointConstraint &firm_constraint)
{
    Joint lower, upper;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set firm constraint.");

    firm_constraint_.getConstraint(lower, upper);
    FST_INFO("Origin firm constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    FST_INFO("Given firm constraint:");
    FST_INFO("  lower = %s", printDBLine(&firm_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&firm_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    hard_constraint_.getConstraint(lower, upper);
    FST_INFO("Hard constraint:");
    FST_INFO("  lower = %s", printDBLine(&lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&upper[0], buffer, LOG_TEXT_SIZE));

    if (hard_constraint_.isCoverConstaint(firm_constraint_))
    {
        ParamGroup param;
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
            param.dumpParamFile())
        {
            firm_constraint_.setConstraint(firm_constraint);
            FST_INFO("Firm constraint updated successfully.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail dumping firm constraint to config file");
            return param.getLastError();
        }
    }
    else
    {
        FST_ERROR("Given firm constraint out of hard constraint.");
        return INVALID_PARAMETER;
    }
}


ErrorCode BaseGroup::setHardConstraint(const JointConstraint &hard_constraint)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set hard constraint.");
    FST_INFO("Origin hard constraint:");
    FST_INFO("  lower = %s", printDBLine(&hard_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&hard_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));

    FST_INFO("Given hard constraint:");
    FST_INFO("  lower = %s", printDBLine(&hard_constraint.lower[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  upper = %s", printDBLine(&hard_constraint.upper[0], buffer, LOG_TEXT_SIZE));

    ParamGroup param;
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
        param.dumpParamFile())
    {
        hard_constraint_.setConstraint(hard_constraint);
        FST_INFO("Hard constraint updated successfully.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail dumping hard constraint to config file");
        return param.getLastError();
    }
}



}

