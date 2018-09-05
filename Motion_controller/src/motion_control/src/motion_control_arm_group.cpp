/*************************************************************************
	> File Name: motion_control_arm_group.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 14时34分07秒
 ************************************************************************/

#include <math.h>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <motion_control_arm_group.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <error_monitor.h>
#include <common_file_path.h>


using namespace std;
using namespace fst_base;
using namespace basic_alg;
using namespace fst_parameter;

namespace fst_mc
{

ErrorCode ArmGroup::initGroup(ErrorMonitor *error_monitor_ptr)
{
    cycle_time_ = 0.001;
    memset(&manual_traj_, 0, sizeof(ManualTrajectory));

    char buf[LOG_ITEM_SIZE];
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    sprintf(buf, "ArmGroup%ld.%ld", time_now.tv_sec, time_now.tv_usec);

    if (!log_ptr_->initLogger(buf))
    {
        FST_ERROR("Lost communication with log server, initGroup abort.");
        return MOTION_INTERNAL_FAULT;
    }

    FST_INFO("Error monitor addr: 0x%x", error_monitor_ptr);
    if (error_monitor_ptr == NULL)
    {
        FST_ERROR("Invalid pointer of error-monitor.");
        return MOTION_INTERNAL_FAULT;
    }
    else
    {
        error_monitor_ptr_ = error_monitor_ptr;
    }

    FST_INFO("Initializing mutex ...");
    if (pthread_mutex_init(&auto_mutex_, NULL) != 0 ||
        pthread_mutex_init(&manual_mutex_, NULL) != 0 ||
        pthread_mutex_init(&servo_mutex_, NULL) != 0)
    {
        FST_ERROR("Fail to initialize mutex.");
        return MOTION_INTERNAL_FAULT;
    }

    FST_INFO("Loading hard constraints ...");
    ParamGroup param;
    vector<double> upper;
    vector<double> lower;

    if (param.loadParamFile(AXIS_GROUP_DIR "hard_constraint.yaml") &&
        param.getParam("hard_constraint/upper", upper) &&
        param.getParam("hard_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            hard_constraint_.initConstraint(*(Joint*)(&lower[0]), *(Joint*)(&upper[0]), JOINT_OF_ARM);
        }
        else
        {
            FST_ERROR("Invalid array size of hard constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading hard constraint from config file");
        return param.getLastError();
    }

    FST_INFO("Loading firm constraints ...");
    param.reset();
    upper.clear();
    lower.clear();

    if (param.loadParamFile(AXIS_GROUP_DIR"firm_constraint.yaml") &&
        param.getParam("firm_constraint/upper", upper) &&
        param.getParam("firm_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            firm_constraint_.initConstraint(*(Joint*)(&lower[0]), *(Joint*)(&upper[0]), JOINT_OF_ARM);
        }
        else
        {
            FST_ERROR("Invalid array size of firm constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading firm constraint from config file");
        return param.getLastError();
    }

    FST_INFO("Loading soft constraints ...");
    param.reset();
    upper.clear();
    lower.clear();

    if (param.loadParamFile(AXIS_GROUP_DIR"soft_constraint.yaml") &&
        param.getParam("soft_constraint/upper", upper) &&
        param.getParam("soft_constraint/lower", lower))
    {
        if (upper.size() == NUM_OF_JOINT && lower.size() == NUM_OF_JOINT)
        {
            soft_constraint_.initConstraint(*(Joint*)(&lower[0]), *(Joint*)(&upper[0]), JOINT_OF_ARM);
        }
        else
        {
            FST_ERROR("Invalid array size of soft constraint.");
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading soft constraint from config file");
        return param.getLastError();
    }

    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Hard/firm/soft constraints from axis-%d to axis-%d:", 0, JOINT_OF_ARM - 1);
    FST_INFO("  hard-lower: %s", printDBLine(&hard_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  firm-lower: %s", printDBLine(&firm_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  soft-lower: %s", printDBLine(&soft_constraint_.lower()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  soft-upper: %s", printDBLine(&soft_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  firm-upper: %s", printDBLine(&firm_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("  hard-upper: %s", printDBLine(&hard_constraint_.upper()[0], buffer, LOG_TEXT_SIZE));


    FST_INFO("Initializing interface to bare core ...");

    if (!bare_core_.initInterface())
    {
        FST_ERROR("Fail to create communication with bare core.");
        return BARE_CORE_TIMEOUT;
    }

    FST_INFO("Initializing calibrator of ArmGroup ...");
    ErrorCode err = calibrator_.initCalibrator(JOINT_OF_ARM, &bare_core_, log_ptr_);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to initialize calibrator, code = 0x%llx", err);
        return err;
    }

    OffsetMask mask[NUM_OF_JOINT] = {OFFSET_UNMASK};
    calibrator_.getOffsetMask(mask);
    size_t index[JOINT_OF_ARM];
    size_t length = 0;

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        if (mask[i] == OFFSET_MASKED)
        {
            index[length++] = i;
        }
    }

    soft_constraint_.setMask(index, length);


    FST_INFO("Initializing kinematics of ArmGroup ...");
    kinematics_ptr_ = new ArmKinematics();

    if (kinematics_ptr_)
    {
        double dh_matrix[NUM_OF_JOINT][4];
        param.reset();

        if (param.loadParamFile(AXIS_GROUP_DIR"arm_dh.yaml"))
        {
            if (param.getParam("dh_parameter/axis-0", dh_matrix[0], 4) &&
                param.getParam("dh_parameter/axis-1", dh_matrix[1], 4) &&
                param.getParam("dh_parameter/axis-2", dh_matrix[2], 4) &&
                param.getParam("dh_parameter/axis-3", dh_matrix[3], 4) &&
                param.getParam("dh_parameter/axis-4", dh_matrix[4], 4) &&
                param.getParam("dh_parameter/axis-5", dh_matrix[5], 4))
            {
                kinematics_ptr_->initKinematics(dh_matrix);
                FST_INFO("DH-matrix:");
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[0][0], dh_matrix[0][1], dh_matrix[0][2], dh_matrix[0][3]);
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[1][0], dh_matrix[1][1], dh_matrix[1][2], dh_matrix[1][3]);
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[2][0], dh_matrix[2][1], dh_matrix[2][2], dh_matrix[2][3]);
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[3][0], dh_matrix[3][1], dh_matrix[3][2], dh_matrix[3][3]);
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[4][0], dh_matrix[4][1], dh_matrix[4][2], dh_matrix[4][3]);
                FST_INFO("  %.6f, %.6f, %.6f, %.6f", dh_matrix[5][0], dh_matrix[5][1], dh_matrix[5][2], dh_matrix[5][3]);
            }
            else
            {
                FST_ERROR("Fail to load dh config, code = 0x%llx", param.getLastError());
                return param.getLastError();
            }
        }
        else
        {
            FST_ERROR("Fail to load dh config, code = 0x%llx", param.getLastError());
            return param.getLastError();
        }
    }
    else
    {
        FST_ERROR("Fail to create kinematics for ArmGroup.");
        return MOTION_INTERNAL_FAULT;
    }

    FST_INFO("Initializing manual teach of ArmGroup ...");
    err = manual_teach_.init(kinematics_ptr_, &soft_constraint_, log_ptr_, AXIS_GROUP_DIR"arm_manual_teach.yaml");

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to initialize manual teach, code = 0x%llx", err);
        return err;
    }

    return SUCCESS;
}



ErrorCode ArmGroup::pickFromManual(TrajectoryPoint *point, size_t &length)
{
    return (manual_traj_.frame == JOINT || manual_traj_.mode == APOINT) ?
           pickFromManualJoint(point, length) : pickFromManualCartesian(point, length);
}

ErrorCode ArmGroup::pickFromManualJoint(TrajectoryPoint *point, size_t &length)
{
    size_t cnt = 0;
    double *angle, *start, *target;
    double tm, omega;

    FST_INFO("Pick from manual joint");
    FST_INFO("manual-time=%.4f", manual_time_);

    for (size_t i = 0 ; i < length; i++)
    {
        point[i].level = manual_time_ > MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
        memset(&point[i].omega, 0, sizeof(Joint));
        memset(&point[i].alpha, 0, sizeof(Joint));
        memset(&point[i].torque, 0, sizeof(Joint));
        memset(&point[i].inertia, 0, sizeof(Joint));
        memset(&point[i].gravity, 0, sizeof(Joint));
        
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
            FST_INFO("%d - %.3f - %.6f %.6f %.6f %.6f %.6f %.6f", point[i].level, manual_time_,
                          point[i].angle.j1, point[i].angle.j2, point[i].angle.j3,
                          point[i].angle.j4, point[i].angle.j5, point[i].angle.j6);
            continue;
        }
    }

    length = cnt;
    return SUCCESS;
}

ErrorCode ArmGroup::pickFromManualCartesian(TrajectoryPoint *point, size_t &length) {
    ErrorCode err = SUCCESS;
    PoseEuler pose;
    double tim, vel;
    double *axis, *start, *target;
    size_t cnt = 0;

    FST_INFO("Pick from manual cartesian");
    FST_INFO("manual-time=%.4f", manual_time_);

    for (size_t i = 0; i < length; i++)
    {
        point[i].level = manual_time_ > MINIMUM_E6 ? POINT_MIDDLE : POINT_START;
        memset(&point[i].omega, 0, sizeof(Joint));
        memset(&point[i].alpha, 0, sizeof(Joint));
        memset(&point[i].torque, 0, sizeof(Joint));
        memset(&point[i].inertia, 0, sizeof(Joint));
        memset(&point[i].gravity, 0, sizeof(Joint));

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
                FST_INFO("%d - %.4f - %.6f %.6f %.6f %.6f %.6f %.6f", point[i].level, manual_time_,
                         point[i].angle.j1, point[i].angle.j2, point[i].angle.j3,
                         point[i].angle.j4, point[i].angle.j5, point[i].angle.j6);
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


ErrorCode ArmGroup::autoMove(void)
{
    return SUCCESS;
}


size_t ArmGroup::getNumberOfJoint(void)
{
    return JOINT_OF_ARM;
}


size_t ArmGroup::getFIFOLength(void)
{
    if (group_state_ == MANUAL)
    {
        return ceil((manual_traj_.duration - manual_time_) * 1000);
    }
    else if (group_state_ == AUTO)
    {
        return 0;
    }
    else
    {
        return 0;
    }
}


bool ArmGroup::isJointInConstraint(Joint joint, JointConstraint constraint)
{
    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        if (joint[i] > constraint.upper[i] + MINIMUM_E9 || joint[i] < constraint.lower[i] - MINIMUM_E9)
        {
            return false;
        }
    }

    return true;
}


char* ArmGroup::printDBLine(const int *data, char *buffer, size_t length)
{
    int len = 0;

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        len += snprintf(buffer + len, length - len, "%d ", data[i]);
    }

    if (len > 0)
    {
        len --;
    }

    buffer[len] = '\0';
    return buffer;
}

char* ArmGroup::printDBLine(const double *data, char *buffer, size_t length)
{
    int len = 0;

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        len += snprintf(buffer + len, length - len, "%.6f ", data[i]);
    }

    if (len > 0)
    {
        len --;
    }

    buffer[len] = '\0';
    return buffer;
}





}




