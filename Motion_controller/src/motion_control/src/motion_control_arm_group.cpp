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
    vel_ratio_ = 0.0625;
    acc_ratio_ = 0.0625;
    cycle_time_ = 0.001;
    memset(&manual_traj_, 0, sizeof(ManualTrajectory));

    /*
    char buf[LOG_ITEM_SIZE];
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    sprintf(buf, "ArmGroup%ld.%ld", time_now.tv_sec, time_now.tv_usec);
*/
    if (!log_ptr_->initLogger("ArmGroup"))
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

    FST_INFO("Initializing cache ...");
    auto_cache_ptr_ = new TrajectoryCache[AUTO_CACHE_SIZE];

    if (auto_cache_ptr_)
    {
        for (size_t i = 0; i < AUTO_CACHE_SIZE; i++)
        {
            auto_cache_ptr_[i].deadline = 0;
            auto_cache_ptr_[i].valid = false;
            auto_cache_ptr_[i].head = 0;
            auto_cache_ptr_[i].tail = 0;
            auto_cache_ptr_[i].smooth_in_stamp = 0;
            auto_cache_ptr_[i].smooth_out_stamp = 0;
            auto_cache_ptr_[i].next = &auto_cache_ptr_[(i + 1) % AUTO_CACHE_SIZE];
            auto_cache_ptr_[i].prev = &auto_cache_ptr_[(i - 1) % AUTO_CACHE_SIZE];
        }

        auto_cache_ptr_[0].prev = &auto_cache_ptr_[AUTO_CACHE_SIZE - 1];
        auto_pick_ptr_ = &auto_cache_ptr_[0];
    }
    else
    {
        FST_ERROR("Fail to create trajectory cache.");
        return MOTION_INTERNAL_FAULT;
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

    param.reset();
    vector<double> data;

    if (param.loadParamFile(COMPONENT_PARAM_FILE_DIR"motion_control.yaml") &&
        param.getParam("joint/omega/limit", data))
    {
        if (data.size() == NUM_OF_JOINT)
        {
            memcpy(axis_vel_, &data[0], NUM_OF_JOINT * sizeof(double));
            FST_INFO("Joint omega: %s", printDBLine(axis_vel_, buffer, LOG_TEXT_SIZE));
        }
        else
        {
            FST_ERROR("Invalid omega array size : %d", data.size());
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail loading motion configuration from config file");
        return param.getLastError();
    }

    int sample_dynamics;

    if (!param.getParam("sample_dynamics", sample_dynamics))
    {
        FST_ERROR("Fail loading sample dynamics from config file");
        return param.getLastError();
    }

    if (sample_dynamics > 0)
    {
        dynamics_cnt_ = sample_dynamics;
    }
    else
    {
        FST_ERROR("Invalid sample dynamics = %d", sample_dynamics);
    }

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

    manual_teach_.setGlobalVelRatio(vel_ratio_);
    manual_teach_.setGlobalAccRatio(acc_ratio_);

    jerk_.j1 = 5.0 * 0.5 / 1.3 * 10000 * 81;
    jerk_.j2 = 3.3 * 0.4 / 0.44 * 10000 * 101;
    jerk_.j3 = 3.3 * 0.4 / 0.44 * 10000 * 81;
    jerk_.j4 = 1.7 * 0.39 / 0.18 * 10000 * 60;
    jerk_.j5 = 1.7 * 0.25 / 0.17 * 10000 * 66.66667;
    jerk_.j6 = 1.7 * 0.25 / 0.17 * 10000 * 44.64286;

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


ErrorCode ArmGroup::computeCompensate(const DynamicsProduct &product, const Joint &omega, const Joint &alpha, Joint &ma_cv_g)
{
    ma_cv_g[0] = product.m[0][0] * alpha[0] + product.m[0][1] * alpha[1] + product.m[0][2] * alpha[2] +
                 product.m[0][3] * alpha[3] + product.m[0][4] * alpha[4] + product.m[0][5] * alpha[5] +
                 product.c[0][0] * omega[0] + product.c[0][1] * omega[1] + product.c[0][2] * omega[2] +
                 product.c[0][3] * omega[3] + product.c[0][4] * omega[4] + product.c[0][5] * omega[5] +
                 product.g[0];

    ma_cv_g[1] = product.m[1][0] * alpha[0] + product.m[1][1] * alpha[1] + product.m[1][2] * alpha[2] +
                 product.m[1][3] * alpha[3] + product.m[1][4] * alpha[4] + product.m[1][5] * alpha[5] +
                 product.c[1][0] * omega[0] + product.c[1][1] * omega[1] + product.c[1][2] * omega[2] +
                 product.c[1][3] * omega[3] + product.c[1][4] * omega[4] + product.c[1][5] * omega[5] +
                 product.g[1];

    ma_cv_g[2] = product.m[2][0] * alpha[0] + product.m[2][1] * alpha[1] + product.m[2][2] * alpha[2] +
                 product.m[2][3] * alpha[3] + product.m[2][4] * alpha[4] + product.m[2][5] * alpha[5] +
                 product.c[2][0] * omega[0] + product.c[2][1] * omega[1] + product.c[2][2] * omega[2] +
                 product.c[2][3] * omega[3] + product.c[2][4] * omega[4] + product.c[2][5] * omega[5] +
                 product.g[2];

    ma_cv_g[3] = product.m[3][0] * alpha[0] + product.m[3][1] * alpha[1] + product.m[3][2] * alpha[2] +
                 product.m[3][3] * alpha[3] + product.m[3][4] * alpha[4] + product.m[3][5] * alpha[5] +
                 product.c[3][0] * omega[0] + product.c[3][1] * omega[1] + product.c[3][2] * omega[2] +
                 product.c[3][3] * omega[3] + product.c[3][4] * omega[4] + product.c[3][5] * omega[5] +
                 product.g[3];

    ma_cv_g[4] = product.m[4][0] * alpha[0] + product.m[4][1] * alpha[1] + product.m[4][2] * alpha[2] +
                 product.m[4][3] * alpha[3] + product.m[4][4] * alpha[4] + product.m[4][5] * alpha[5] +
                 product.c[4][0] * omega[0] + product.c[4][1] * omega[1] + product.c[4][2] * omega[2] +
                 product.c[4][3] * omega[3] + product.c[4][4] * omega[4] + product.c[4][5] * omega[5] +
                 product.g[4];

    ma_cv_g[5] = product.m[5][0] * alpha[0] + product.m[5][1] * alpha[1] + product.m[5][2] * alpha[2] +
                 product.m[5][3] * alpha[3] + product.m[5][4] * alpha[4] + product.m[5][5] * alpha[5] +
                 product.c[5][0] * omega[0] + product.c[5][1] * omega[1] + product.c[5][2] * omega[2] +
                 product.c[5][3] * omega[3] + product.c[5][4] * omega[4] + product.c[5][5] * omega[5] +
                 product.g[5];

    return SUCCESS;
}


char* ArmGroup::printDBLine(const int *data, char *buffer, size_t length)
{
    int len = 0;

    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        len += snprintf(buffer + len, length - len, "%d, ", data[i]);
    }

    if (len > 1)
    {
        len --;
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
        //len += snprintf(buffer + len, length - len, "%.6f ", data[i]);
        len += snprintf(buffer + len, length - len, "%.12f, ", data[i]);
    }

    if (len > 1)
    {
        len --;
        len --;
    }

    buffer[len] = '\0';
    return buffer;
}





}




