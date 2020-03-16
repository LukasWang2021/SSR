/*************************************************************************
	> File Name: motion_control_manual_teach.cpp
	> Author: 冯赟
	> Mail: yun.feng@foresight-robotics.com
	> Created Time: 2018年08月07日 星期二 15时48分54秒
 ************************************************************************/

#include <math.h>
#include <float.h>
#include <iostream>
#include <string>
#include <string.h>
#include <vector>

#include <motion_control_manual_teach.h>
#include <common_log.h>
#include <parameter_manager/parameter_manager_param_group.h>


using namespace std;
using namespace basic_alg;
using namespace fst_parameter;

namespace fst_mc
{

ManualTeach::ManualTeach(void)
{
    joint_num_ = 0;
    vel_ratio_ = 0;
    acc_ratio_ = 0;
    step_position_ = 0;
    step_orientation_ = 0;
    position_vel_reference_ = 0;
    position_acc_reference_ = 0;
    orientation_omega_reference_ = 0;
    orientation_alpha_reference_ = 0;

    memset(step_axis_, 0, sizeof(step_axis_));
    memset(axis_vel_, 0, sizeof(axis_vel_));
    memset(axis_acc_, 0, sizeof(axis_acc_));

    memset(&wf_matrix_.trans_vector_, 0, sizeof(wf_matrix_.trans_vector_));
    memset(&uf_matrix_.trans_vector_, 0, sizeof(uf_matrix_.trans_vector_));
    memset(&tf_matrix_.trans_vector_, 0, sizeof(tf_matrix_.trans_vector_));
    memset(&wf_matrix_inverse_.trans_vector_, 0, sizeof(wf_matrix_inverse_.trans_vector_));
    memset(&uf_matrix_inverse_.trans_vector_, 0, sizeof(uf_matrix_inverse_.trans_vector_));
    memset(&tf_matrix_inverse_.trans_vector_, 0, sizeof(tf_matrix_inverse_.trans_vector_));
    wf_matrix_.rotation_matrix_.eye();
    uf_matrix_.rotation_matrix_.eye();
    tf_matrix_.rotation_matrix_.eye();
    wf_matrix_inverse_.rotation_matrix_.eye();
    uf_matrix_inverse_.rotation_matrix_.eye();
    tf_matrix_inverse_.rotation_matrix_.eye();

    log_ptr_ = NULL;
    joint_constraint_ptr_ = NULL;
}

ManualTeach::~ManualTeach(void)
{}

ErrorCode ManualTeach::init(basic_alg::Kinematics *pkinematics, basic_alg::DynamicAlg* pdynamics, Constraint *pcons, fst_log::Logger *plog, const string &config_file)
{
    if (pkinematics && pdynamics && pcons && plog)
    {
        log_ptr_ = plog;
        kinematics_ptr_ = pkinematics;
        dynamics_ptr_ = pdynamics;
        joint_constraint_ptr_ = pcons;
    }
    else
    {
        return MC_INTERNAL_FAULT;
    }

    int  joint_num;
    char buffer[LOG_TEXT_SIZE];
    double data[NUM_OF_JOINT];
    ParamGroup config;
    manual_config_file_ = config_file;

    if (!config.loadParamFile(manual_config_file_))
    {
        FST_ERROR("Fail to load config file: %s", manual_config_file_.c_str());
        return config.getLastError();
    }

    if (config.getParam("number_of_axis", joint_num))
    {
        if (joint_num > 0 && joint_num <= NUM_OF_JOINT)
        {
            joint_num_ = joint_num;
            FST_INFO("Number of axis: %d", joint_num_);
        }
        else
        {
            FST_ERROR("Invalid number of axis in config file, num-of-joint = %d", joint_num);
            return INVALID_PARAMETER;
        }
    }
    else
    {
        FST_ERROR("Fail to load number of manual axis, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    if (!config.getParam("step/axis", data, joint_num_))
    {
        FST_ERROR("Fail to load manual step, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    memcpy(step_axis_, data, joint_num_ * sizeof(double));
    FST_INFO("Axis step: %s", printDBLine(step_axis_, buffer, LOG_TEXT_SIZE));

    if (config.getParam("step/position", step_position_) &&
        config.getParam("step/orientation", step_orientation_) &&
        config.getParam("reference/position/velocity", position_vel_reference_) &&
        config.getParam("reference/position/acceleration", position_acc_reference_) &&
        config.getParam("reference/position/jerk", position_jerk_reference_) &&
        config.getParam("reference/orientation/omega", orientation_omega_reference_) &&
        config.getParam("reference/orientation/alpha", orientation_alpha_reference_) &&
        config.getParam("reference/orientation/beta", orientation_beta_reference_))
    {
        FST_INFO("Step: position=%.4f, orientation=%.4f", step_position_, step_orientation_);
        FST_INFO("Reference: position-vel=%.4f, position-acc=%.4f, position-jerk=%.4f, orientation-omega=%.4f, orientation-alpha=%.4f, orientation-beta=%.4f",
                 position_vel_reference_, position_acc_reference_, position_jerk_reference_, orientation_omega_reference_, orientation_alpha_reference_, orientation_beta_reference_);
    }
    else
    {
        FST_ERROR("Fail to load manual configuration.");
        return config.getLastError();
    }

    if (!config.getParam("reference/axis/move_to_point_velocity", data, joint_num_))
    {
        FST_ERROR("Fail to load reference velocity in move-to-point of each axis, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    memcpy(move_to_point_vel_, data, joint_num_ * sizeof(double));
    FST_INFO("Axis vel in move-to-point: %s", printDBLine(move_to_point_vel_, buffer, LOG_TEXT_SIZE));

    if (!config.getParam("reference/axis/velocity", data, joint_num_))
    {
        FST_ERROR("Fail to load reference velocity of each axis, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    memcpy(axis_vel_, data, joint_num_ * sizeof(double));
    FST_INFO("Axis vel: %s", printDBLine(axis_vel_, buffer, LOG_TEXT_SIZE));
    
    if(!config.getParam("reference/axis/acceleration", data, joint_num_))
    {
        FST_ERROR("Fail to load reference acceleration of each axis, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    memcpy(axis_acc_, data, joint_num_ * sizeof(double));
    FST_INFO("Axis acc : %s", printDBLine(axis_acc_, buffer, LOG_TEXT_SIZE));

    if(!config.getParam("reference/axis/jerk", data, joint_num_))
    {
        FST_ERROR("Fail to load reference jerk of each axis, code = 0x%llx", config.getLastError());
        return config.getLastError();
    }

    memcpy(axis_jerk_, data, joint_num_ * sizeof(double));
    FST_INFO("Axis jerk : %s", printDBLine(axis_jerk_, buffer, LOG_TEXT_SIZE));

    return SUCCESS;
}

void ManualTeach::getManualStepAxis(double *steps)
{
    for (uint32_t i = 0; i < joint_num_; i++)
    {
        steps[i] = step_axis_[i];
    }
}

double ManualTeach::getManualStepPosition(void)
{
    return step_position_;
}

double ManualTeach::getManualStepOrientation(void)
{
    return step_orientation_;
}

double ManualTeach::getDuration(void)
{
    return total_duration_;
}

void ManualTeach::setGlobalVelRatio(double ratio)
{
    vel_ratio_ = ratio;
}

void ManualTeach::setGlobalAccRatio(double ratio)
{
    acc_ratio_ = ratio;
}

ErrorCode ManualTeach::setManualStepAxis(const double *steps)
{
    vector<double> data;

    for (uint32_t i = 0; i < joint_num_; i++)
    {
        data.push_back(steps[i]);
    }

    ParamGroup param;

    if (param.loadParamFile(manual_config_file_) && param.setParam("step/axis", data) && param.dumpParamFile())
    {
        memcpy(step_axis_, steps, joint_num_ * sizeof(double));
        return SUCCESS;
    }
    else
    {
        return param.getLastError();
    }

    return SUCCESS;
}

ErrorCode ManualTeach::setManualStepPosition(double step)
{
    if (step > MINIMUM_E3 && step < 100)
    {
        ParamGroup param;

        if (param.loadParamFile(manual_config_file_) && param.setParam("step/position", step) && param.dumpParamFile())
        {
            step_position_ = step;
            return SUCCESS;
        }
        else
        {
            return param.getLastError();
        }
    }
    else
    {
        return INVALID_PARAMETER;
    }
}

ErrorCode ManualTeach::setManualStepOrientation(double step)
{
    if (step > MINIMUM_E6 && step < 1)
    {
        ParamGroup param;

        if (param.loadParamFile(manual_config_file_) && param.setParam("step/orientation", step) && param.dumpParamFile())
        {
            step_orientation_ = step;
            return SUCCESS;
        }
        else
        {
            return param.getLastError();
        }
    }
    else
    {
        return INVALID_PARAMETER;
    }
}

void ManualTeach::setWorldFrame(const PoseEuler &wf)
{
	wf.convertToTransMatrix(wf_matrix_);
	wf_matrix_inverse_ = wf_matrix_;
    wf_matrix_inverse_.inverse();
}

void ManualTeach::setUserFrame(const PoseEuler &uf)
{
	uf.convertToTransMatrix(uf_matrix_);
	uf_matrix_inverse_ = uf_matrix_;
    uf_matrix_inverse_.inverse();
}

void ManualTeach::setToolFrame(const PoseEuler &tf)
{
	tf.convertToTransMatrix(tf_matrix_);
	tf_matrix_inverse_ = tf_matrix_;
    tf_matrix_inverse_.inverse();
}

void ManualTeach::setManualFrame(ManualFrame frame)
{
    frame_type_ = frame;
}

ManualMode ManualTeach::getManualMode(void)
{
    return motion_type_;
}

ManualFrame ManualTeach::getManualFrame(void)
{
    return frame_type_;
}

ErrorCode ManualTeach::manualStep(const ManualDirection *directions, const Joint &start)
{
    ErrorCode err = SUCCESS;

    switch (frame_type_)
    {
        case JOINT:
            FST_INFO("Manual step in joint space");
            err = manualStepInJoint(directions, start);
            break;

        case BASE:
            FST_INFO("Manual step in base space");
            err = manualStepInBase(directions, start);
            break;
        case USER:
            FST_INFO("Manual step in user space");
            err = manualStepInUser(directions, start);
            break;
        case WORLD:
            FST_INFO("Manual step in world space");
            err = manualStepInWorld(directions, start);
            break;
        case TOOL:
            FST_INFO("Manual step in tool space");
            err = manualStepInTool(directions, start);
            break;

        default:
            err = MC_FAIL_MANUAL_STEP;
            FST_ERROR("Manual step unsupported frame: %d", frame_type_);
            break;
    }

    if (err == SUCCESS)
    {
        FST_INFO("Manual step trajectory ready.");
        motion_type_ = STEP;
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Manual step failed, err = 0x%llx", err);
        total_duration_ = 0;
        return err;
    }
}

ErrorCode ManualTeach::manualStepInJoint(const ManualDirection *dir, const Joint &start)
{
    char buffer[LOG_TEXT_SIZE];
    Joint target  = start;
    int index = -1;

    for (uint32_t i = 0; i < joint_num_; i++)
    {
        if (dir[i] == STANDING) continue;
        if (dir[i] == INCREASE) {target[i] += step_axis_[i]; index = i; break;}
        if (dir[i] == DECREASE) {target[i] -= step_axis_[i]; index = i; break;}
    }
    
    FST_INFO("Manual step in joint: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("Step-axis = %s", printDBLine(step_axis_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-joint = %s", printDBLine(&start.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Target-joint = %s", printDBLine(&target.j1_, buffer, LOG_TEXT_SIZE));

    if (!joint_constraint_ptr_->isJointInConstraint(start))
    {
        FST_ERROR("Start joint out of soft constraint.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    if (!joint_constraint_ptr_->isJointInConstraint(target))
    {
        FST_ERROR("Target joint out of soft constraint.");
        return JOINT_OUT_OF_CONSTRAINT;
    }

    FST_INFO("  axis-vel = %s", printDBLine(axis_vel_, buffer, LOG_TEXT_SIZE));
    FST_INFO("  axis-acc = %s", printDBLine(axis_acc_, buffer, LOG_TEXT_SIZE));
    FST_INFO("  axis-jerk = %s", printDBLine(axis_jerk_, buffer, LOG_TEXT_SIZE));
    joint_start_ = start;
    joint_end_ = target;
    double trip = fabs(target[index] - start[index]);
    
    if (index < 0 || fabs(target[index] - start[index]) < MINIMUM_E6)
    {
        FST_WARN("Start joint near target joint, do not move");
        total_duration_ = 0;
        return SUCCESS;
    }

    double vel = axis_vel_[index] / trip;
    double acc = axis_acc_[index] / trip;
    double jerk = axis_jerk_[index] / trip;
    FST_INFO("index: %d, trip: %.6f, vel: %.6f, acc: %.6f, jerk: %.6f, create ds curve ...", index, trip, vel, acc, jerk);
    ds_curve_.planDSCurve(0, 1, vel, acc, &jerk);
    total_duration_ = ds_curve_.getDuration();
    FST_INFO("Success, duration: %.6f", total_duration_);
    return SUCCESS;
}

ErrorCode ManualTeach::manualStepInBase(const ManualDirection *dir, const Joint &start)
{
    char buffer[LOG_TEXT_SIZE];
    PoseEuler start_pose;
    TransMatrix matrix;
    kinematics_ptr_->doFK(start, start_pose);
	start_pose.convertToTransMatrix(matrix);
	matrix.rightMultiply(tf_matrix_).convertToPoseEuler(start_pose);
    auxiliary_coord_.euler_ = start_pose.euler_;
    memset(&auxiliary_coord_.point_, 0, sizeof(auxiliary_coord_.point_));
    cart_start_.point_ = start_pose.point_;
    memset(&cart_start_.euler_, 0, sizeof(cart_start_.euler_));
    cart_end_ = cart_start_;
    int index = -1;

    if (dir[0] != STANDING)
    {
        cart_end_.point_.x_ += step_position_ * ((dir[0] == INCREASE) ? 1 : -1);
        index = 0;
    }
    else if (dir[1] != STANDING)
    {
        cart_end_.point_.y_ += step_position_ * ((dir[1] == INCREASE) ? 1 : -1);
        index = 1;
    }
    else if (dir[2] != STANDING)
    {
        cart_end_.point_.z_ += step_position_ * ((dir[2] == INCREASE) ? 1 : -1);
        index = 2;
    }
    else if (dir[3] != STANDING)
    {
        cart_end_.euler_.a_ += step_orientation_ * ((dir[3] == INCREASE) ? 1 : -1);
        index = 3;
    }
    else if (dir[4] != STANDING)
    {
        cart_end_.euler_.b_ += step_orientation_ * ((dir[4] == INCREASE) ? 1 : -1);
        index = 4;
    }
    else if (dir[5] != STANDING)
    {
        cart_end_.euler_.c_ += step_orientation_ * ((dir[5] == INCREASE) ? 1 : -1);
        index = 5;
    }

    FST_INFO("Manual step in base: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-joint = %s", printDBLine(&start.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Auxiliary-coord = %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,", auxiliary_coord_.point_.x_, auxiliary_coord_.point_.y_, auxiliary_coord_.point_.z_, auxiliary_coord_.euler_.a_, auxiliary_coord_.euler_.b_, auxiliary_coord_.euler_.c_);
    FST_INFO("Start-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_start_.point_.x_, cart_start_.point_.y_, cart_start_.point_.z_, cart_start_.euler_.a_, cart_start_.euler_.b_, cart_start_.euler_.c_);
    FST_INFO("End-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_end_.point_.x_, cart_end_.point_.y_, cart_end_.point_.z_, cart_end_.euler_.a_, cart_end_.euler_.b_, cart_end_.euler_.c_);
    double trip, vel, acc, jerk;
    
    if (index >= 0 && index < 3)
    {
        trip = step_position_;
        vel = position_vel_reference_ / trip;
        acc = position_acc_reference_ / trip;
        jerk = position_jerk_reference_ / trip;
    }
    else if (index >= 3 && index < 6)
    {
        trip = step_orientation_;
        vel = orientation_omega_reference_ / trip;
        acc = orientation_alpha_reference_ / trip;
        jerk = orientation_beta_reference_ / trip;
    }
    else
    {
        FST_ERROR("Manual step directions invalid");
        return INVALID_PARAMETER;
    }

    FST_INFO("index: %d, trip: %.6f, vel: %.6f, acc: %.6f, jerk: %.6f, create ds curve ...", index, trip, vel, acc, jerk);
    ds_curve_.planDSCurve(0, 1, vel, acc, &jerk);
    total_duration_ = ds_curve_.getDuration();
    FST_INFO("Success, duration: %.6f", total_duration_);
    return SUCCESS;
}

ErrorCode ManualTeach::manualStepInUser(const ManualDirection *dir, const Joint &start)
{
    char buffer[LOG_TEXT_SIZE];
    PoseEuler start_pose;
    TransMatrix matrix;
    kinematics_ptr_->doFK(start, start_pose);
	start_pose.convertToTransMatrix(matrix);
    matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_).convertToPoseEuler(start_pose);
    auxiliary_coord_.euler_ = start_pose.euler_;
    memset(&auxiliary_coord_.point_, 0, sizeof(auxiliary_coord_.point_));
    cart_start_.point_ = start_pose.point_;
    memset(&cart_start_.euler_, 0, sizeof(cart_start_.euler_));
    cart_end_ = cart_start_;
    int index = -1;

    if (dir[0] != STANDING)
    {
        cart_end_.point_.x_ += step_position_ * ((dir[0] == INCREASE) ? 1 : -1);
        index = 0;
    }
    else if (dir[1] != STANDING)
    {
        cart_end_.point_.y_ += step_position_ * ((dir[1] == INCREASE) ? 1 : -1);
        index = 1;
    }
    else if (dir[2] != STANDING)
    {
        cart_end_.point_.z_ += step_position_ * ((dir[2] == INCREASE) ? 1 : -1);
        index = 2;
    }
    else if (dir[3] != STANDING)
    {
        cart_end_.euler_.a_ += step_orientation_ * ((dir[3] == INCREASE) ? 1 : -1);
        index = 3;
    }
    else if (dir[4] != STANDING)
    {
        cart_end_.euler_.b_ += step_orientation_ * ((dir[4] == INCREASE) ? 1 : -1);
        index = 4;
    }
    else if (dir[5] != STANDING)
    {
        cart_end_.euler_.c_ += step_orientation_ * ((dir[5] == INCREASE) ? 1 : -1);
        index = 5;
    }

    FST_INFO("Manual step in user: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-joint = %s", printDBLine(&start.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Auxiliary-coord = %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,", auxiliary_coord_.point_.x_, auxiliary_coord_.point_.y_, auxiliary_coord_.point_.z_, auxiliary_coord_.euler_.a_, auxiliary_coord_.euler_.b_, auxiliary_coord_.euler_.c_);
    FST_INFO("Start-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_start_.point_.x_, cart_start_.point_.y_, cart_start_.point_.z_, cart_start_.euler_.a_, cart_start_.euler_.b_, cart_start_.euler_.c_);
    FST_INFO("End-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_end_.point_.x_, cart_end_.point_.y_, cart_end_.point_.z_, cart_end_.euler_.a_, cart_end_.euler_.b_, cart_end_.euler_.c_);
    double trip, vel, acc, jerk;
    
    if (index >= 0 && index < 3)
    {
        trip = step_position_;
        vel = position_vel_reference_ / trip;
        acc = position_acc_reference_ / trip;
        jerk = position_jerk_reference_ / trip;
    }
    else if (index >= 3 && index < 6)
    {
        trip = step_orientation_;
        vel = orientation_omega_reference_ / trip;
        acc = orientation_alpha_reference_ / trip;
        jerk = orientation_beta_reference_ / trip;
    }
    else
    {
        FST_ERROR("Manual step directions invalid");
        return INVALID_PARAMETER;
    }

    FST_INFO("index: %d, trip: %.6f, vel: %.6f, acc: %.6f, jerk: %.6f, create ds curve ...", index, trip, vel, acc, jerk);
    ds_curve_.planDSCurve(0, 1, vel, acc, &jerk);
    total_duration_ = ds_curve_.getDuration();
    FST_INFO("Success, duration: %.6f", total_duration_);
    return SUCCESS;
}

ErrorCode ManualTeach::manualStepInWorld(const ManualDirection *dir, const Joint &start)
{
    char buffer[LOG_TEXT_SIZE];
    PoseEuler start_pose;
    TransMatrix matrix;
    kinematics_ptr_->doFK(start, start_pose);
	start_pose.convertToTransMatrix(matrix);
    matrix.rightMultiply(tf_matrix_).leftMultiply(wf_matrix_inverse_).convertToPoseEuler(start_pose);
    auxiliary_coord_.euler_ = start_pose.euler_;
    memset(&auxiliary_coord_.point_, 0, sizeof(auxiliary_coord_.point_));
    cart_start_.point_ = start_pose.point_;
    memset(&cart_start_.euler_, 0, sizeof(cart_start_.euler_));
    cart_end_ = cart_start_;
    int index = -1;

    if (dir[0] != STANDING)
    {
        cart_end_.point_.x_ += step_position_ * ((dir[0] == INCREASE) ? 1 : -1);
        index = 0;
    }
    else if (dir[1] != STANDING)
    {
        cart_end_.point_.y_ += step_position_ * ((dir[1] == INCREASE) ? 1 : -1);
        index = 1;
    }
    else if (dir[2] != STANDING)
    {
        cart_end_.point_.z_ += step_position_ * ((dir[2] == INCREASE) ? 1 : -1);
        index = 2;
    }
    else if (dir[3] != STANDING)
    {
        cart_end_.euler_.a_ += step_orientation_ * ((dir[3] == INCREASE) ? 1 : -1);
        index = 3;
    }
    else if (dir[4] != STANDING)
    {
        cart_end_.euler_.b_ += step_orientation_ * ((dir[4] == INCREASE) ? 1 : -1);
        index = 4;
    }
    else if (dir[5] != STANDING)
    {
        cart_end_.euler_.c_ += step_orientation_ * ((dir[5] == INCREASE) ? 1 : -1);
        index = 5;
    }

    FST_INFO("Manual step in world: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-joint = %s", printDBLine(&start.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Auxiliary-coord = %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,", auxiliary_coord_.point_.x_, auxiliary_coord_.point_.y_, auxiliary_coord_.point_.z_, auxiliary_coord_.euler_.a_, auxiliary_coord_.euler_.b_, auxiliary_coord_.euler_.c_);
    FST_INFO("Start-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_start_.point_.x_, cart_start_.point_.y_, cart_start_.point_.z_, cart_start_.euler_.a_, cart_start_.euler_.b_, cart_start_.euler_.c_);
    FST_INFO("End-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_end_.point_.x_, cart_end_.point_.y_, cart_end_.point_.z_, cart_end_.euler_.a_, cart_end_.euler_.b_, cart_end_.euler_.c_);
    double trip, vel, acc, jerk;
    
    if (index >= 0 && index < 3)
    {
        trip = step_position_;
        vel = position_vel_reference_ / trip;
        acc = position_acc_reference_ / trip;
        jerk = position_jerk_reference_ / trip;
    }
    else if (index >= 3 && index < 6)
    {
        trip = step_orientation_;
        vel = orientation_omega_reference_ / trip;
        acc = orientation_alpha_reference_ / trip;
        jerk = orientation_beta_reference_ / trip;
    }
    else
    {
        FST_ERROR("Manual step directions invalid");
        return INVALID_PARAMETER;
    }

    FST_INFO("index: %d, trip: %.6f, vel: %.6f, acc: %.6f, jerk: %.6f, create ds curve ...", index, trip, vel, acc, jerk);
    ds_curve_.planDSCurve(0, 1, vel, acc, &jerk);
    total_duration_ = ds_curve_.getDuration();
    FST_INFO("Success, duration: %.6f", total_duration_);
    return SUCCESS;
}

ErrorCode ManualTeach::manualStepInTool(const ManualDirection *dir, const Joint &start)
{
    char buffer[LOG_TEXT_SIZE];
    PoseEuler start_pose;
    TransMatrix matrix;
    kinematics_ptr_->doFK(start, start_pose);
	start_pose.convertToTransMatrix(matrix);
    matrix.rightMultiply(tf_matrix_).convertToPoseEuler(start_pose);
    auxiliary_coord_ = start_pose;
    memset(&cart_start_, 0, sizeof(cart_start_));
    cart_end_ = cart_start_;
    int index = -1;

    if (dir[0] != STANDING)
    {
        cart_end_.point_.x_ += step_position_ * ((dir[0] == INCREASE) ? 1 : -1);
        index = 0;
    }
    else if (dir[1] != STANDING)
    {
        cart_end_.point_.y_ += step_position_ * ((dir[1] == INCREASE) ? 1 : -1);
        index = 1;
    }
    else if (dir[2] != STANDING)
    {
        cart_end_.point_.z_ += step_position_ * ((dir[2] == INCREASE) ? 1 : -1);
        index = 2;
    }
    else if (dir[3] != STANDING)
    {
        cart_end_.euler_.a_ += step_orientation_ * ((dir[3] == INCREASE) ? 1 : -1);
        index = 3;
    }
    else if (dir[4] != STANDING)
    {
        cart_end_.euler_.b_ += step_orientation_ * ((dir[4] == INCREASE) ? 1 : -1);
        index = 4;
    }
    else if (dir[5] != STANDING)
    {
        cart_end_.euler_.c_ += step_orientation_ * ((dir[5] == INCREASE) ? 1 : -1);
        index = 5;
    }

    FST_INFO("Manual step in tool: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-joint = %s", printDBLine(&start.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Auxiliary-coord = %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,", auxiliary_coord_.point_.x_, auxiliary_coord_.point_.y_, auxiliary_coord_.point_.z_, auxiliary_coord_.euler_.a_, auxiliary_coord_.euler_.b_, auxiliary_coord_.euler_.c_);
    FST_INFO("Start-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_start_.point_.x_, cart_start_.point_.y_, cart_start_.point_.z_, cart_start_.euler_.a_, cart_start_.euler_.b_, cart_start_.euler_.c_);
    FST_INFO("End-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_end_.point_.x_, cart_end_.point_.y_, cart_end_.point_.z_, cart_end_.euler_.a_, cart_end_.euler_.b_, cart_end_.euler_.c_);
    double trip, vel, acc, jerk;
    
    if (index >= 0 && index < 3)
    {
        trip = step_position_;
        vel = position_vel_reference_ / trip;
        acc = position_acc_reference_ / trip;
        jerk = position_jerk_reference_ / trip;
    }
    else if (index >= 3 && index < 6)
    {
        trip = step_orientation_;
        vel = orientation_omega_reference_ / trip;
        acc = orientation_alpha_reference_ / trip;
        jerk = orientation_beta_reference_ / trip;
    }
    else
    {
        FST_ERROR("Manual step directions invalid");
        return INVALID_PARAMETER;
    }

    FST_INFO("index: %d, trip: %.6f, vel: %.6f, acc: %.6f, jerk: %.6f, create ds curve ...", index, trip, vel, acc, jerk);
    ds_curve_.planDSCurve(0, 1, vel, acc, &jerk);
    total_duration_ = ds_curve_.getDuration();
    FST_INFO("Success, duration: %.6f", total_duration_);
    return SUCCESS;
}

ErrorCode ManualTeach::manualContinuous(const ManualDirection *directions, const Joint &start)
{
    ErrorCode err = SUCCESS;

    for (uint32_t j = 0; j < joint_num_; j++)
    {
        motion_direction_[j] = STANDING;
        axis_ds_curve_valid_[j] = false;
        axis_ds_curve_start_time_[j] = 0;
    }
    
    switch (frame_type_)
    {
        case JOINT:
            FST_INFO("Manual continuous in joint space");
            err = manualContinuousInJoint(directions, start);
            break;

        case BASE:
            FST_INFO("Manual continuous in base space");
            err = manualContinuousInBase(directions, start);
            break;
        case USER:
            FST_INFO("Manual continuous in user space");
            err = manualContinuousInUser(directions, start);
            break;
        case WORLD:
            FST_INFO("Manual continuous in world space");
            err = manualContinuousInWorld(directions, start);
            break;
        case TOOL:
            FST_INFO("Manual continuous in tool space");
            err = manualContinuousInTool(directions, start);
            break;

        default:
            err = MC_FAIL_MANUAL_CONTINUOUS;
            FST_ERROR("Manual continuous unsupported frame: %d", frame_type_);
            break;
    }

    if (err == SUCCESS)
    {
        FST_INFO("Manual continuous trajectory ready.");
        motion_type_ = CONTINUOUS;
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Manual continuous failed, err = 0x%llx", err);
        total_duration_ = 0;
        return err;
    }
}

ErrorCode ManualTeach::manualContinuousInJoint(const ManualDirection *dir, const Joint &start)
{
    Joint target;
    double duration = 0;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual continuous in joint: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-joint = %s", printDBLine(&start.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("  axis-vel = %s", printDBLine(axis_vel_, buffer, LOG_TEXT_SIZE));
    FST_INFO("  axis-acc = %s", printDBLine(axis_acc_, buffer, LOG_TEXT_SIZE));
    FST_INFO("  axis-jerk = %s", printDBLine(axis_jerk_, buffer, LOG_TEXT_SIZE));

    for (uint32_t j = 0; j < joint_num_; j++)
    {
        double trip = 9999.99;

        if (dir[j] == STANDING)
        {
            motion_direction_[j] = STANDING;
            axis_ds_curve_valid_[j] = false;
            axis_ds_curve_start_time_[j] = 0;
            target[j] = start[j];
            continue;
        }

        if (!joint_constraint_ptr_->isMask(j))
        {
            trip = (dir[j] == INCREASE) ? (joint_constraint_ptr_->upper()[j] - start[j]) : (start[j] - joint_constraint_ptr_->lower()[j]);
        }

        if (trip < MINIMUM_E6)
        {
            FST_INFO("index: %d: near soft constraint, cannot manual move", j);
            motion_direction_[j] = STANDING;
            axis_ds_curve_valid_[j] = false;
            axis_ds_curve_start_time_[j] = 0;
            target[j] = start[j];
            continue;
        }

        double vel_ratio = vel_ratio_;
        double acc_ratio = acc_ratio_ > vel_ratio_ ? vel_ratio_ : acc_ratio_;
        double vel = axis_vel_[j] * vel_ratio / trip;
        double acc = axis_acc_[j] * acc_ratio / trip;
        double jerk = axis_jerk_[j] * acc_ratio / trip;
        axis_ds_curve_[j].planDSCurve(0, 1, vel, acc, &jerk);
        motion_direction_[j] = dir[j];
        axis_ds_curve_valid_[j] = true;
        axis_ds_curve_start_time_[j] = 0;
        duration = duration < axis_ds_curve_[j].getDuration() ? axis_ds_curve_[j].getDuration() : duration;
        target[j] = (dir[j] == INCREASE) ? (start[j] + trip) : (start[j] - trip);
        FST_INFO("index: %d, trip: %.6f, vel: %.6f, acc: %.6f, jerk: %.6f, duration: %.6f", j, trip, vel, acc, jerk, axis_ds_curve_[j].getDuration());
    }

    joint_start_ = start;
    joint_end_ = target;
    total_duration_ = duration;
    FST_INFO("Success, duration: %.6f", total_duration_);
    return SUCCESS;
}

ErrorCode ManualTeach::manualContinuousInBase(const ManualDirection *dir, const Joint &start)
{
    char buffer[LOG_TEXT_SIZE];
    PoseEuler start_pose;
    TransMatrix matrix;
    kinematics_ptr_->doFK(start, start_pose);
	start_pose.convertToTransMatrix(matrix);
	matrix.rightMultiply(tf_matrix_).convertToPoseEuler(start_pose);
    auxiliary_coord_.euler_ = start_pose.euler_;
    memset(&auxiliary_coord_.point_, 0, sizeof(auxiliary_coord_.point_));
    cart_start_.point_ = start_pose.point_;
    memset(&cart_start_.euler_, 0, sizeof(cart_start_.euler_));
    cart_end_ = cart_start_;
    int index = -1;
    double position_trip_max = 9999.9999;
    double orientation_trip_max = 99.99;
    double trip, vel, acc, jerk;
    double vel_ratio = vel_ratio_;
    double acc_ratio = acc_ratio_ > vel_ratio_ ? vel_ratio_ : acc_ratio_;

    if (dir[0] != STANDING)
    {
        cart_end_.point_.x_ += position_trip_max * ((dir[0] == INCREASE) ? 1 : -1);
        index = 0;
    }
    else if (dir[1] != STANDING)
    {
        cart_end_.point_.y_ += position_trip_max * ((dir[1] == INCREASE) ? 1 : -1);
        index = 1;
    }
    else if (dir[2] != STANDING)
    {
        cart_end_.point_.z_ += position_trip_max * ((dir[2] == INCREASE) ? 1 : -1);
        index = 2;
    }
    else if (dir[3] != STANDING)
    {
        cart_end_.euler_.a_ += orientation_trip_max * ((dir[3] == INCREASE) ? 1 : -1);
        index = 3;
    }
    else if (dir[4] != STANDING)
    {
        cart_end_.euler_.b_ += orientation_trip_max * ((dir[4] == INCREASE) ? 1 : -1);
        index = 4;
    }
    else if (dir[5] != STANDING)
    {
        cart_end_.euler_.c_ += orientation_trip_max * ((dir[5] == INCREASE) ? 1 : -1);
        index = 5;
    }
    
    if (index >= 0 && index < 3)
    {
        trip = position_trip_max;
        vel = position_vel_reference_ * vel_ratio / trip;
        acc = position_acc_reference_ * acc_ratio / trip;
        jerk = position_jerk_reference_ * acc_ratio / trip;
    }
    else if (index >= 3 && index < 6)
    {
        trip = orientation_trip_max;
        vel = orientation_omega_reference_ * vel_ratio / trip;
        acc = orientation_alpha_reference_ * acc_ratio / trip;
        jerk = orientation_beta_reference_ * acc_ratio / trip;
    }
    else
    {
        FST_ERROR("Manual continuous directions invalid");
        return INVALID_PARAMETER;
    }

    FST_INFO("Manual continuous in base: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-joint = %s", printDBLine(&start.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Auxiliary-coord = %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,", auxiliary_coord_.point_.x_, auxiliary_coord_.point_.y_, auxiliary_coord_.point_.z_, auxiliary_coord_.euler_.a_, auxiliary_coord_.euler_.b_, auxiliary_coord_.euler_.c_);
    FST_INFO("Start-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_start_.point_.x_, cart_start_.point_.y_, cart_start_.point_.z_, cart_start_.euler_.a_, cart_start_.euler_.b_, cart_start_.euler_.c_);
    FST_INFO("End-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_end_.point_.x_, cart_end_.point_.y_, cart_end_.point_.z_, cart_end_.euler_.a_, cart_end_.euler_.b_, cart_end_.euler_.c_);
    FST_INFO("Index: %d, trip: %.6f, vel: %.6f, acc: %.6f, jerk: %.6f, create ds curve ...", index, trip, vel, acc, jerk);
    axis_ds_curve_[index].planDSCurve(0, 1, vel, acc, &jerk);
    motion_direction_[index] = dir[index];
    axis_ds_curve_valid_[index] = true;
    axis_ds_curve_start_time_[index] = 0;
    total_duration_ = axis_ds_curve_[index].getDuration();
    FST_INFO("Success, duration: %.6f", total_duration_);
    return SUCCESS;
}

ErrorCode ManualTeach::manualContinuousInUser(const ManualDirection *dir, const Joint &start)
{
    char buffer[LOG_TEXT_SIZE];
    PoseEuler start_pose;
    TransMatrix matrix;
    kinematics_ptr_->doFK(start, start_pose);
	start_pose.convertToTransMatrix(matrix);
	matrix.rightMultiply(tf_matrix_).leftMultiply(uf_matrix_inverse_).convertToPoseEuler(start_pose);
    auxiliary_coord_.euler_ = start_pose.euler_;
    memset(&auxiliary_coord_.point_, 0, sizeof(auxiliary_coord_.point_));
    cart_start_.point_ = start_pose.point_;
    memset(&cart_start_.euler_, 0, sizeof(cart_start_.euler_));
    cart_end_ = cart_start_;
    int index = -1;
    double position_trip_max = 9999.9999;
    double orientation_trip_max = 99.99;
    double trip, vel, acc, jerk;
    double vel_ratio = vel_ratio_;
    double acc_ratio = acc_ratio_ > vel_ratio_ ? vel_ratio_ : acc_ratio_;

    if (dir[0] != STANDING)
    {
        cart_end_.point_.x_ += position_trip_max * ((dir[0] == INCREASE) ? 1 : -1);
        index = 0;
    }
    else if (dir[1] != STANDING)
    {
        cart_end_.point_.y_ += position_trip_max * ((dir[1] == INCREASE) ? 1 : -1);
        index = 1;
    }
    else if (dir[2] != STANDING)
    {
        cart_end_.point_.z_ += position_trip_max * ((dir[2] == INCREASE) ? 1 : -1);
        index = 2;
    }
    else if (dir[3] != STANDING)
    {
        cart_end_.euler_.a_ += orientation_trip_max * ((dir[3] == INCREASE) ? 1 : -1);
        index = 3;
    }
    else if (dir[4] != STANDING)
    {
        cart_end_.euler_.b_ += orientation_trip_max * ((dir[4] == INCREASE) ? 1 : -1);
        index = 4;
    }
    else if (dir[5] != STANDING)
    {
        cart_end_.euler_.c_ += orientation_trip_max * ((dir[5] == INCREASE) ? 1 : -1);
        index = 5;
    }
    
    if (index >= 0 && index < 3)
    {
        trip = position_trip_max;
        vel = position_vel_reference_ * vel_ratio / trip;
        acc = position_acc_reference_ * acc_ratio / trip;
        jerk = position_jerk_reference_ * acc_ratio / trip;
    }
    else if (index >= 3 && index < 6)
    {
        trip = orientation_trip_max;
        vel = orientation_omega_reference_ * vel_ratio / trip;
        acc = orientation_alpha_reference_ * acc_ratio / trip;
        jerk = orientation_beta_reference_ * acc_ratio / trip;
    }
    else
    {
        FST_ERROR("Manual continuous directions invalid");
        return INVALID_PARAMETER;
    }

    FST_INFO("Manual continuous in user: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-joint = %s", printDBLine(&start.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Auxiliary-coord = %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,", auxiliary_coord_.point_.x_, auxiliary_coord_.point_.y_, auxiliary_coord_.point_.z_, auxiliary_coord_.euler_.a_, auxiliary_coord_.euler_.b_, auxiliary_coord_.euler_.c_);
    FST_INFO("Start-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_start_.point_.x_, cart_start_.point_.y_, cart_start_.point_.z_, cart_start_.euler_.a_, cart_start_.euler_.b_, cart_start_.euler_.c_);
    FST_INFO("End-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_end_.point_.x_, cart_end_.point_.y_, cart_end_.point_.z_, cart_end_.euler_.a_, cart_end_.euler_.b_, cart_end_.euler_.c_);
    FST_INFO("Index: %d, trip: %.6f, vel: %.6f, acc: %.6f, jerk: %.6f, create ds curve ...", index, trip, vel, acc, jerk);
    axis_ds_curve_[index].planDSCurve(0, 1, vel, acc, &jerk);
    motion_direction_[index] = dir[index];
    axis_ds_curve_valid_[index] = true;
    axis_ds_curve_start_time_[index] = 0;
    total_duration_ = axis_ds_curve_[index].getDuration();
    FST_INFO("Success, duration: %.6f", total_duration_);
    return SUCCESS;
}

ErrorCode ManualTeach::manualContinuousInWorld(const ManualDirection *dir, const Joint &start)
{
    char buffer[LOG_TEXT_SIZE];
    PoseEuler start_pose;
    TransMatrix matrix;
    kinematics_ptr_->doFK(start, start_pose);
	start_pose.convertToTransMatrix(matrix);
	matrix.rightMultiply(tf_matrix_).leftMultiply(wf_matrix_inverse_).convertToPoseEuler(start_pose);
    auxiliary_coord_.euler_ = start_pose.euler_;
    memset(&auxiliary_coord_.point_, 0, sizeof(auxiliary_coord_.point_));
    cart_start_.point_ = start_pose.point_;
    memset(&cart_start_.euler_, 0, sizeof(cart_start_.euler_));
    cart_end_ = cart_start_;
    int index = -1;
    double position_trip_max = 9999.9999;
    double orientation_trip_max = 99.99;
    double trip, vel, acc, jerk;
    double vel_ratio = vel_ratio_;
    double acc_ratio = acc_ratio_ > vel_ratio_ ? vel_ratio_ : acc_ratio_;

    if (dir[0] != STANDING)
    {
        cart_end_.point_.x_ += position_trip_max * ((dir[0] == INCREASE) ? 1 : -1);
        index = 0;
    }
    else if (dir[1] != STANDING)
    {
        cart_end_.point_.y_ += position_trip_max * ((dir[1] == INCREASE) ? 1 : -1);
        index = 1;
    }
    else if (dir[2] != STANDING)
    {
        cart_end_.point_.z_ += position_trip_max * ((dir[2] == INCREASE) ? 1 : -1);
        index = 2;
    }
    else if (dir[3] != STANDING)
    {
        cart_end_.euler_.a_ += orientation_trip_max * ((dir[3] == INCREASE) ? 1 : -1);
        index = 3;
    }
    else if (dir[4] != STANDING)
    {
        cart_end_.euler_.b_ += orientation_trip_max * ((dir[4] == INCREASE) ? 1 : -1);
        index = 4;
    }
    else if (dir[5] != STANDING)
    {
        cart_end_.euler_.c_ += orientation_trip_max * ((dir[5] == INCREASE) ? 1 : -1);
        index = 5;
    }
    
    if (index >= 0 && index < 3)
    {
        trip = position_trip_max;
        vel = position_vel_reference_ * vel_ratio / trip;
        acc = position_acc_reference_ * acc_ratio / trip;
        jerk = position_jerk_reference_ * acc_ratio / trip;
    }
    else if (index >= 3 && index < 6)
    {
        trip = orientation_trip_max;
        vel = orientation_omega_reference_ * vel_ratio / trip;
        acc = orientation_alpha_reference_ * acc_ratio / trip;
        jerk = orientation_beta_reference_ * acc_ratio / trip;
    }
    else
    {
        FST_ERROR("Manual continuous directions invalid");
        return INVALID_PARAMETER;
    }

    FST_INFO("Manual continuous in world: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-joint = %s", printDBLine(&start.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Auxiliary-coord = %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,", auxiliary_coord_.point_.x_, auxiliary_coord_.point_.y_, auxiliary_coord_.point_.z_, auxiliary_coord_.euler_.a_, auxiliary_coord_.euler_.b_, auxiliary_coord_.euler_.c_);
    FST_INFO("Start-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_start_.point_.x_, cart_start_.point_.y_, cart_start_.point_.z_, cart_start_.euler_.a_, cart_start_.euler_.b_, cart_start_.euler_.c_);
    FST_INFO("End-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_end_.point_.x_, cart_end_.point_.y_, cart_end_.point_.z_, cart_end_.euler_.a_, cart_end_.euler_.b_, cart_end_.euler_.c_);
    FST_INFO("Index: %d, trip: %.6f, vel: %.6f, acc: %.6f, jerk: %.6f, create ds curve ...", index, trip, vel, acc, jerk);
    axis_ds_curve_[index].planDSCurve(0, 1, vel, acc, &jerk);
    motion_direction_[index] = dir[index];
    axis_ds_curve_valid_[index] = true;
    axis_ds_curve_start_time_[index] = 0;
    total_duration_ = axis_ds_curve_[index].getDuration();
    FST_INFO("Success, duration: %.6f", total_duration_);
    return SUCCESS;
}

ErrorCode ManualTeach::manualContinuousInTool(const ManualDirection *dir, const Joint &start)
{
    char buffer[LOG_TEXT_SIZE];
    PoseEuler start_pose;
    TransMatrix matrix;
    kinematics_ptr_->doFK(start, start_pose);
	start_pose.convertToTransMatrix(matrix);
	matrix.rightMultiply(tf_matrix_).convertToPoseEuler(start_pose);
    auxiliary_coord_ = start_pose;
    memset(&cart_start_, 0, sizeof(cart_start_));
    cart_end_ = cart_start_;
    int index = -1;
    double position_trip_max = 9999.9999;
    double orientation_trip_max = 99.99;
    double trip, vel, acc, jerk;
    double vel_ratio = vel_ratio_;
    double acc_ratio = acc_ratio_ > vel_ratio_ ? vel_ratio_ : acc_ratio_;

    if (dir[0] != STANDING)
    {
        cart_end_.point_.x_ += position_trip_max * ((dir[0] == INCREASE) ? 1 : -1);
        index = 0;
    }
    else if (dir[1] != STANDING)
    {
        cart_end_.point_.y_ += position_trip_max * ((dir[1] == INCREASE) ? 1 : -1);
        index = 1;
    }
    else if (dir[2] != STANDING)
    {
        cart_end_.point_.z_ += position_trip_max * ((dir[2] == INCREASE) ? 1 : -1);
        index = 2;
    }
    else if (dir[3] != STANDING)
    {
        cart_end_.euler_.a_ += orientation_trip_max * ((dir[3] == INCREASE) ? 1 : -1);
        index = 3;
    }
    else if (dir[4] != STANDING)
    {
        cart_end_.euler_.b_ += orientation_trip_max * ((dir[4] == INCREASE) ? 1 : -1);
        index = 4;
    }
    else if (dir[5] != STANDING)
    {
        cart_end_.euler_.c_ += orientation_trip_max * ((dir[5] == INCREASE) ? 1 : -1);
        index = 5;
    }
    
    if (index >= 0 && index < 3)
    {
        trip = position_trip_max;
        vel = position_vel_reference_ * vel_ratio / trip;
        acc = position_acc_reference_ * acc_ratio / trip;
        jerk = position_jerk_reference_ * acc_ratio / trip;
    }
    else if (index >= 3 && index < 6)
    {
        trip = orientation_trip_max;
        vel = orientation_omega_reference_ * vel_ratio / trip;
        acc = orientation_alpha_reference_ * acc_ratio / trip;
        jerk = orientation_beta_reference_ * acc_ratio / trip;
    }
    else
    {
        FST_ERROR("Manual continuous directions invalid");
        return INVALID_PARAMETER;
    }

    FST_INFO("Manual continuous in tool: directions = %s, planning trajectory ...", printDBLine((int*)dir, buffer, LOG_TEXT_SIZE));
    FST_INFO("Start-joint = %s", printDBLine(&start.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Auxiliary-coord = %.4f, %.4f, %.4f,   %.4f, %.4f, %.4f,", auxiliary_coord_.point_.x_, auxiliary_coord_.point_.y_, auxiliary_coord_.point_.z_, auxiliary_coord_.euler_.a_, auxiliary_coord_.euler_.b_, auxiliary_coord_.euler_.c_);
    FST_INFO("Start-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_start_.point_.x_, cart_start_.point_.y_, cart_start_.point_.z_, cart_start_.euler_.a_, cart_start_.euler_.b_, cart_start_.euler_.c_);
    FST_INFO("End-pose = %.4f %.4f %.4f - %.4f %.4f %.4f", cart_end_.point_.x_, cart_end_.point_.y_, cart_end_.point_.z_, cart_end_.euler_.a_, cart_end_.euler_.b_, cart_end_.euler_.c_);
    FST_INFO("Index: %d, trip: %.6f, vel: %.6f, acc: %.6f, jerk: %.6f, create ds curve ...", index, trip, vel, acc, jerk);
    axis_ds_curve_[index].planDSCurve(0, 1, vel, acc, &jerk);
    motion_direction_[index] = dir[index];
    axis_ds_curve_valid_[index] = true;
    axis_ds_curve_start_time_[index] = 0;
    total_duration_ = axis_ds_curve_[index].getDuration();
    FST_INFO("Success, duration: %.6f", total_duration_);
    return SUCCESS;
}

ErrorCode ManualTeach::manualContinuous(const ManualDirection *directions, double time)
{
    switch (frame_type_)
    {
        case JOINT:
            manualContinuousInJoint(directions, time);
            return SUCCESS;

        case BASE:
        case USER:
        case WORLD:
        case TOOL:
            manualContinuousInCartesian(directions, time);
            return SUCCESS;

        default:
            FST_ERROR("Manual continuous unsupported frame: %d", frame_type_);
            return MC_FAIL_MANUAL_CONTINUOUS;
    }
}

ErrorCode ManualTeach::manualContinuousInJoint(const ManualDirection *dir, double time)
{
    for (uint32_t j = 0; j < joint_num_; j++)
    {
        if (motion_direction_[j] == dir[j])
        {
            // direction do not change, do nothing
            continue;
        }

        if (dir[j] != STANDING && motion_direction_[j] == STANDING)
        {
            // start motion
            double p, v, a;
            double last_stop_time = axis_ds_curve_valid_[j] ? axis_ds_curve_start_time_[j] + axis_ds_curve_[j].getDuration() : 0;
            axis_ds_curve_[j].sampleDSCurve(axis_ds_curve_[j].getDuration(), p, v, a);
            double last_stop_pos = (1 - p) * joint_start_[j] + p * joint_end_[j];

            if (time > last_stop_time)
            {
                double trip = 9999.99;

                if (!joint_constraint_ptr_->isMask(j))
                {
                    trip = (dir[j] == INCREASE) ? (joint_constraint_ptr_->upper()[j] - last_stop_pos) : (last_stop_pos - joint_constraint_ptr_->lower()[j]);
                }

                if (trip < MINIMUM_E6)
                {
                    continue;
                }

                double vel_ratio = vel_ratio_;
                double acc_ratio = acc_ratio_ > vel_ratio_ ? vel_ratio_ : acc_ratio_;
                double vel = axis_vel_[j] * vel_ratio / trip;
                double acc = axis_acc_[j] * acc_ratio / trip;
                double jerk = axis_jerk_[j] * acc_ratio / trip;
                axis_ds_curve_[j].planDSCurve(0, 1, vel, acc, &jerk);
                axis_ds_curve_valid_[j] = true;
                motion_direction_[j] = dir[j];
                axis_ds_curve_start_time_[j] = time;
                joint_start_[j] = last_stop_pos;
                joint_end_[j] = (dir[j] == INCREASE) ? (last_stop_pos + trip) : (last_stop_pos - trip);
            }
        }
        else if (motion_direction_[j] != STANDING)
        {
            // stop motion
            double stop_time = time - axis_ds_curve_start_time_[j];

            if (axis_ds_curve_valid_[j] == true && stop_time < axis_ds_curve_[j].getDuration())
            {
                axis_ds_curve_[j].planStopDSCurve(stop_time);
                motion_direction_[j] = STANDING;
            }
        }
    }

    double duration = 0;

    for (uint32_t j = 0; j < joint_num_; j++)
    {
        if (axis_ds_curve_valid_[j] == true && axis_ds_curve_start_time_[j] + axis_ds_curve_[j].getDuration() > duration)
        {
            duration = axis_ds_curve_start_time_[j] + axis_ds_curve_[j].getDuration();
        }
    }

    total_duration_ = duration;
    return SUCCESS;
}

ErrorCode ManualTeach::manualContinuousInCartesian(const ManualDirection *dir, double time)
{
    for (uint32_t j = 0; j < 6; j++)
    {
        if (motion_direction_[j] == dir[j])
        {
            // direction do not change, do nothing
            continue;
        }

        if (dir[j] != STANDING && motion_direction_[j] == STANDING)
        {
            // start motion
            double p, v, a;
            double last_stop_time = axis_ds_curve_valid_[j] ? axis_ds_curve_start_time_[j] + axis_ds_curve_[j].getDuration() : 0;
            axis_ds_curve_[j].sampleDSCurve(axis_ds_curve_[j].getDuration(), p, v, a);
            double start = j < 3 ? cart_start_.point_[j] : cart_start_.euler_[j - 3];
            double end = j < 3 ? cart_end_.point_[j] : cart_end_.euler_[j - 3];
            double last_stop_pos = (1 - p) * start + p * end;

            double trip = j < 3 ? 9999.99 : 99.99;
            double vel = j < 3 ? position_vel_reference_ : orientation_omega_reference_;
            double acc = j < 3 ? position_acc_reference_ : orientation_alpha_reference_;
            double jerk = j < 3 ? position_jerk_reference_ : orientation_beta_reference_;
            double vel_ratio = vel_ratio_;
            double acc_ratio = acc_ratio_ > vel_ratio_ ? vel_ratio_ : acc_ratio_;

            if (time > last_stop_time)
            {
                vel = vel * vel_ratio / trip;
                acc = acc * acc_ratio / trip;
                jerk = jerk * acc_ratio / trip;
                axis_ds_curve_[j].planDSCurve(0, 1, vel, acc, &jerk);
                axis_ds_curve_valid_[j] = true;
                motion_direction_[j] = dir[j];
                axis_ds_curve_start_time_[j] = time;

                if (j < 3)
                {
                    cart_start_.point_[j] = last_stop_pos;
                    cart_end_.point_[j] = (dir[j] == INCREASE) ? (last_stop_pos + trip) : (last_stop_pos - trip);
                }
                else
                {
                    cart_start_.euler_[j - 3] = last_stop_pos;
                    cart_end_.euler_[j - 3] = (dir[j] == INCREASE) ? (last_stop_pos + trip) : (last_stop_pos - trip);
                }
            }
        }
        else
        {
            // stop motion
            double stop_time = time - axis_ds_curve_start_time_[j];

            if (axis_ds_curve_valid_[j] == true && stop_time < axis_ds_curve_[j].getDuration())
            {
                axis_ds_curve_[j].planStopDSCurve(stop_time);
                motion_direction_[j] = STANDING;
            }
        }
    }

    double duration = 0;

    for (uint32_t j = 0; j < joint_num_; j++)
    {
        if (axis_ds_curve_valid_[j] == true && axis_ds_curve_start_time_[j] + axis_ds_curve_[j].getDuration() > duration)
        {
            duration = axis_ds_curve_start_time_[j] + axis_ds_curve_[j].getDuration();
        }
    }

    total_duration_ = duration;
    return SUCCESS;
}

ErrorCode ManualTeach::manualToJoint(const Joint &start, const Joint &end)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Manual move to joint: planning trajectory ...");
    FST_INFO("Start joint = %s", printDBLine(&start.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Target joint = %s", printDBLine(&end.j1_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Axis-vel = %s", printDBLine(move_to_point_vel_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Axis-acc = %s", printDBLine(axis_acc_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Axis-jerk = %s", printDBLine(axis_jerk_, buffer, LOG_TEXT_SIZE));
    FST_INFO("vel-ratio = %.0f%%, acc-ratio = %.0f%%", vel_ratio_ * 100, acc_ratio_ * 100);
    joint_start_ = start;
    joint_end_ = end;
    uint32_t index = 0;
    double duration = -1;

    for (uint32_t j = 0; j < joint_num_; j++)
    {
        if (fabs((end[j] - start[j]) / move_to_point_vel_[j]) > duration)
        {
            duration = fabs((end[j] - start[j]) / move_to_point_vel_[j]);
            index = j;
        }
    }

    double trip = fabs(end[index] - start[index]);
    
    if (trip < MINIMUM_E6)
    {
        FST_WARN("Start joint near target joint, do not move");
        total_duration_ = 0;
        return SUCCESS;
    }

    double vel_ratio = vel_ratio_;
    double acc_ratio = acc_ratio_ > vel_ratio_ ? vel_ratio_ : acc_ratio_;
    double vel = move_to_point_vel_[index] * vel_ratio / trip;
    double acc = axis_acc_[index] * acc_ratio / trip;
    double jerk = axis_jerk_[index] * acc_ratio / trip;
    FST_INFO("index: %d, trip: %.6f, vel: %.6f, acc: %.6f, jerk: %.6f, create ds curve ...", index, trip, vel, acc, jerk);
    ds_curve_.planDSCurve(0, 1, vel, acc, &jerk);
    total_duration_ = ds_curve_.getDuration();
    motion_type_ = APOINT;
    FST_INFO("Success, duration: %.6f", total_duration_);
    return SUCCESS;
}

ErrorCode ManualTeach::sampleTrajectory(double sample_time, const Joint &reference, JointState &state)
{
    double t = sample_time > 0 ? sample_time : 0;
    double p, v, a;
    double trip;
    double delta_time = 0.00001;
    PoseEuler sample, sample_delta1, sample_delta2;

    if (motion_type_ == APOINT)
    {
        ds_curve_.sampleDSCurve(t, p, v, a);

        for (uint32_t j = 0; j < joint_num_; j++)
        {
            trip = joint_end_[j] - joint_start_[j];
            state.angle[j] = joint_start_[j] + p * trip;
            state.omega[j] = v * trip;
            state.alpha[j] = a * trip;
            state.torque[j] = 0;
        }

        dynamics_ptr_->getTorqueInverseDynamics(state.angle, (*(JointVelocity*)(&state.omega)), (*(JointAcceleration*)(&state.alpha)), (*(JointTorque*)(&state.torque)));
        return SUCCESS;
    }

    if (motion_type_ == STEP)
    {
        if (frame_type_ == JOINT)
        {
            ds_curve_.sampleDSCurve(t, p, v, a);

            for (uint32_t j = 0; j < joint_num_; j++)
            {
                trip = joint_end_[j] - joint_start_[j];
                state.angle[j] = joint_start_[j] + p * trip;
                state.omega[j] = v * trip;
                state.alpha[j] = a * trip;
                state.torque[j] = 0;
            }
        }
        else if (frame_type_ == BASE)
        {
            ds_curve_.sampleDSCurve(t, p, v, a);
            sample = interpolatPoseEuler(p, cart_start_, cart_end_);
            ds_curve_.sampleDSCurve(t + delta_time, p, v, a);
            sample_delta1 = interpolatPoseEuler(p, cart_start_, cart_end_);
            ds_curve_.sampleDSCurve(t + delta_time * 2, p, v, a);
            sample_delta2 = interpolatPoseEuler(p, cart_start_, cart_end_);

            Joint joint_delta1, joint_delta2;
            TransMatrix trans, trans_delta1, trans_delta2, trans_auxiliary_coord;
            auxiliary_coord_.convertToTransMatrix(trans_auxiliary_coord);
            sample.convertToTransMatrix(trans);
            sample_delta1.convertToTransMatrix(trans_delta1);
            sample_delta2.convertToTransMatrix(trans_delta2);
            trans.rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta1.rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta2.rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            
            if (!kinematics_ptr_->doIK(trans, reference, state.angle) ||
				!kinematics_ptr_->doIK(trans_delta1, reference, joint_delta1) ||
				!kinematics_ptr_->doIK(trans_delta2, reference, joint_delta2))
            {
                char buffer[LOG_TEXT_SIZE];
                PoseEuler pe0, pe1, pe2;
                trans.convertToPoseEuler(pe0);
                trans_delta1.convertToPoseEuler(pe1);
                trans_delta2.convertToPoseEuler(pe2);
                FST_ERROR("Compute IK failed.");
                FST_ERROR("Reference: %s", printDBLine(&reference.j1_, buffer, LOG_TEXT_SIZE));
                FST_ERROR("Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe0.point_.x_, pe0.point_.y_, pe0.point_.z_, pe0.euler_.a_, pe0.euler_.b_, pe0.euler_.c_);
                FST_ERROR("Pose delta 1: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe1.point_.x_, pe1.point_.y_, pe1.point_.z_, pe1.euler_.a_, pe1.euler_.b_, pe1.euler_.c_);
                FST_ERROR("Pose delta 2: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe2.point_.x_, pe2.point_.y_, pe2.point_.z_, pe2.euler_.a_, pe2.euler_.b_, pe2.euler_.c_);
                return MC_COMPUTE_IK_FAIL;
            }

            for (uint32_t j = 0; j < joint_num_; j++)
            {
                state.omega[j] = (joint_delta1[j] - state.angle[j]) / delta_time;
                state.alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time - state.omega[j]) / delta_time;
                state.torque[j] = 0;
            }
        }
        else if (frame_type_ == USER)
        {
            ds_curve_.sampleDSCurve(t, p, v, a);
            sample = interpolatPoseEuler(p, cart_start_, cart_end_);
            ds_curve_.sampleDSCurve(t + delta_time, p, v, a);
            sample_delta1 = interpolatPoseEuler(p, cart_start_, cart_end_);
            ds_curve_.sampleDSCurve(t + delta_time * 2, p, v, a);
            sample_delta2 = interpolatPoseEuler(p, cart_start_, cart_end_);

            Joint joint_delta1, joint_delta2;
            TransMatrix trans, trans_delta1, trans_delta2, trans_auxiliary_coord;
            auxiliary_coord_.convertToTransMatrix(trans_auxiliary_coord);
            sample.convertToTransMatrix(trans);
            sample_delta1.convertToTransMatrix(trans_delta1);
            sample_delta2.convertToTransMatrix(trans_delta2);
            trans.leftMultiply(uf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta1.leftMultiply(uf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta2.leftMultiply(uf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            
            if (!kinematics_ptr_->doIK(trans, reference, state.angle) ||
				!kinematics_ptr_->doIK(trans_delta1, reference, joint_delta1) ||
				!kinematics_ptr_->doIK(trans_delta2, reference, joint_delta2))
            {
                char buffer[LOG_TEXT_SIZE];
                PoseEuler pe0, pe1, pe2;
                trans.convertToPoseEuler(pe0);
                trans_delta1.convertToPoseEuler(pe1);
                trans_delta2.convertToPoseEuler(pe2);
                FST_ERROR("Compute IK failed.");
                FST_ERROR("Reference: %s", printDBLine(&reference.j1_, buffer, LOG_TEXT_SIZE));
                FST_ERROR("Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe0.point_.x_, pe0.point_.y_, pe0.point_.z_, pe0.euler_.a_, pe0.euler_.b_, pe0.euler_.c_);
                FST_ERROR("Pose delta 1: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe1.point_.x_, pe1.point_.y_, pe1.point_.z_, pe1.euler_.a_, pe1.euler_.b_, pe1.euler_.c_);
                FST_ERROR("Pose delta 2: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe2.point_.x_, pe2.point_.y_, pe2.point_.z_, pe2.euler_.a_, pe2.euler_.b_, pe2.euler_.c_);
                return MC_COMPUTE_IK_FAIL;
            }

            for (uint32_t j = 0; j < joint_num_; j++)
            {
                state.omega[j] = (joint_delta1[j] - state.angle[j]) / delta_time;
                state.alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time - state.omega[j]) / delta_time;
                state.torque[j] = 0;
            }
        }
        else if (frame_type_ == WORLD)
        {
            ds_curve_.sampleDSCurve(t, p, v, a);
            sample = interpolatPoseEuler(p, cart_start_, cart_end_);
            ds_curve_.sampleDSCurve(t + delta_time, p, v, a);
            sample_delta1 = interpolatPoseEuler(p, cart_start_, cart_end_);
            ds_curve_.sampleDSCurve(t + delta_time * 2, p, v, a);
            sample_delta2 = interpolatPoseEuler(p, cart_start_, cart_end_);

            Joint joint_delta1, joint_delta2;
            TransMatrix trans, trans_delta1, trans_delta2, trans_auxiliary_coord;
            auxiliary_coord_.convertToTransMatrix(trans_auxiliary_coord);
            sample.convertToTransMatrix(trans);
            sample_delta1.convertToTransMatrix(trans_delta1);
            sample_delta2.convertToTransMatrix(trans_delta2);
            trans.leftMultiply(wf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta1.leftMultiply(wf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta2.leftMultiply(wf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            
            if (!kinematics_ptr_->doIK(trans, reference, state.angle) ||
				!kinematics_ptr_->doIK(trans_delta1, reference, joint_delta1) ||
				!kinematics_ptr_->doIK(trans_delta2, reference, joint_delta2))
            {
                char buffer[LOG_TEXT_SIZE];
                PoseEuler pe0, pe1, pe2;
                trans.convertToPoseEuler(pe0);
                trans_delta1.convertToPoseEuler(pe1);
                trans_delta2.convertToPoseEuler(pe2);
                FST_ERROR("Compute IK failed.");
                FST_ERROR("Reference: %s", printDBLine(&reference.j1_, buffer, LOG_TEXT_SIZE));
                FST_ERROR("Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe0.point_.x_, pe0.point_.y_, pe0.point_.z_, pe0.euler_.a_, pe0.euler_.b_, pe0.euler_.c_);
                FST_ERROR("Pose delta 1: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe1.point_.x_, pe1.point_.y_, pe1.point_.z_, pe1.euler_.a_, pe1.euler_.b_, pe1.euler_.c_);
                FST_ERROR("Pose delta 2: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe2.point_.x_, pe2.point_.y_, pe2.point_.z_, pe2.euler_.a_, pe2.euler_.b_, pe2.euler_.c_);
                return MC_COMPUTE_IK_FAIL;
            }

            for (uint32_t j = 0; j < joint_num_; j++)
            {
                state.omega[j] = (joint_delta1[j] - state.angle[j]) / delta_time;
                state.alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time - state.omega[j]) / delta_time;
                state.torque[j] = 0;
            }
        }
        else if (frame_type_ == TOOL)
        {
            ds_curve_.sampleDSCurve(t, p, v, a);
            sample = interpolatPoseEuler(p, cart_start_, cart_end_);
            ds_curve_.sampleDSCurve(t + delta_time, p, v, a);
            sample_delta1 = interpolatPoseEuler(p, cart_start_, cart_end_);
            ds_curve_.sampleDSCurve(t + delta_time * 2, p, v, a);
            sample_delta2 = interpolatPoseEuler(p, cart_start_, cart_end_);

            Joint joint_delta1, joint_delta2;
            TransMatrix trans, trans_delta1, trans_delta2, trans_auxiliary_coord;
            auxiliary_coord_.convertToTransMatrix(trans_auxiliary_coord);
            
            sample.convertToTransMatrix(trans);
            sample_delta1.convertToTransMatrix(trans_delta1);
            sample_delta2.convertToTransMatrix(trans_delta2);
            trans.leftMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta1.leftMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta2.leftMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            
            if (!kinematics_ptr_->doIK(trans, reference, state.angle) ||
				!kinematics_ptr_->doIK(trans_delta1, reference, joint_delta1) ||
				!kinematics_ptr_->doIK(trans_delta2, reference, joint_delta2))
            {
                char buffer[LOG_TEXT_SIZE];
                PoseEuler pe0, pe1, pe2;
                trans.convertToPoseEuler(pe0);
                trans_delta1.convertToPoseEuler(pe1);
                trans_delta2.convertToPoseEuler(pe2);
                FST_ERROR("Compute IK failed.");
                FST_ERROR("Reference: %s", printDBLine(&reference.j1_, buffer, LOG_TEXT_SIZE));
                FST_ERROR("Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe0.point_.x_, pe0.point_.y_, pe0.point_.z_, pe0.euler_.a_, pe0.euler_.b_, pe0.euler_.c_);
                FST_ERROR("Pose delta 1: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe1.point_.x_, pe1.point_.y_, pe1.point_.z_, pe1.euler_.a_, pe1.euler_.b_, pe1.euler_.c_);
                FST_ERROR("Pose delta 2: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe2.point_.x_, pe2.point_.y_, pe2.point_.z_, pe2.euler_.a_, pe2.euler_.b_, pe2.euler_.c_);
                return MC_COMPUTE_IK_FAIL;
            }

            for (uint32_t j = 0; j < joint_num_; j++)
            {
                state.omega[j] = (joint_delta1[j] - state.angle[j]) / delta_time;
                state.alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time - state.omega[j]) / delta_time;
                state.torque[j] = 0;
            }
        }
        else
        {
            FST_ERROR("Sample manual trajectory failed, type: %d, frame: %d", motion_type_, frame_type_);
            return MC_MANUAL_FRAME_ERROR;
        }

        dynamics_ptr_->getTorqueInverseDynamics(state.angle, (*(JointVelocity*)(&state.omega)), (*(JointAcceleration*)(&state.alpha)), (*(JointTorque*)(&state.torque)));
        return SUCCESS;
    }
    
    if (motion_type_ == CONTINUOUS)
    {
        if (frame_type_ == JOINT)
        {
            for (uint32_t j = 0; j < joint_num_; j++)
            {
                if (axis_ds_curve_valid_[j] == false)
                {
                    state.angle[j] = joint_end_[j];
                    state.omega[j] = 0;
                    state.alpha[j] = 0;
                    state.torque[j] = 0;
                    continue;
                }

                double t_sample = t - axis_ds_curve_start_time_[j];
                axis_ds_curve_[j].sampleDSCurve(t_sample, p, v, a);
                trip = joint_end_[j] - joint_start_[j];
                state.angle[j] = joint_start_[j] + p * trip;
                state.omega[j] = v * trip;
                state.alpha[j] = a * trip;
                state.torque[j] = 0;
            }
        }
        else if (frame_type_ == BASE)
        {
            for (uint32_t j = 0; j < 3; j++)
            {
                if (axis_ds_curve_valid_[j] == false)
                {
                    sample.point_[j] = cart_end_.point_[j];
                    sample_delta1.point_[j] = cart_end_.point_[j];
                    sample_delta2.point_[j] = cart_end_.point_[j];
                    continue;
                }

                double t_sample = t - axis_ds_curve_start_time_[j];
                trip = cart_end_.point_[j] - cart_start_.point_[j];
                axis_ds_curve_[j].sampleDSCurve(t_sample, p, v, a);
                sample.point_[j] = cart_start_.point_[j] + p * trip;
                axis_ds_curve_[j].sampleDSCurve(t_sample + delta_time, p, v, a);
                sample_delta1.point_[j] = cart_start_.point_[j] + p * trip;
                axis_ds_curve_[j].sampleDSCurve(t_sample + delta_time * 2, p, v, a);
                sample_delta2.point_[j] = cart_start_.point_[j] + p * trip;
            }
            
            for (uint32_t j = 0; j < 3; j++)
            {
                if (axis_ds_curve_valid_[j + 3] == false)
                {
                    sample.euler_[j] = cart_end_.euler_[j];
                    sample_delta1.euler_[j] = cart_end_.euler_[j];
                    sample_delta2.euler_[j] = cart_end_.euler_[j];
                    continue;
                }

                double t_sample = t - axis_ds_curve_start_time_[j + 3];
                trip = cart_end_.euler_[j] - cart_start_.euler_[j];
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample, p, v, a);
                sample.euler_[j] = cart_start_.euler_[j] + p * trip;
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample + delta_time, p, v, a);
                sample_delta1.euler_[j] = cart_start_.euler_[j] + p * trip;
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample + delta_time * 2, p, v, a);
                sample_delta2.euler_[j] = cart_start_.euler_[j] + p * trip;
            }

            Joint joint_delta1, joint_delta2;
            TransMatrix trans, trans_delta1, trans_delta2, trans_auxiliary_coord;
            auxiliary_coord_.convertToTransMatrix(trans_auxiliary_coord);
            sample.convertToTransMatrix(trans);
            sample_delta1.convertToTransMatrix(trans_delta1);
            sample_delta2.convertToTransMatrix(trans_delta2);
            trans.rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta1.rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta2.rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            
            if (!kinematics_ptr_->doIK(trans, reference, state.angle) ||
				!kinematics_ptr_->doIK(trans_delta1, reference, joint_delta1) ||
				!kinematics_ptr_->doIK(trans_delta2, reference, joint_delta2))
            {
                char buffer[LOG_TEXT_SIZE];
                PoseEuler pe0, pe1, pe2;
                trans.convertToPoseEuler(pe0);
                trans_delta1.convertToPoseEuler(pe1);
                trans_delta2.convertToPoseEuler(pe2);
                FST_ERROR("Compute IK failed.");
                FST_ERROR("Reference: %s", printDBLine(&reference.j1_, buffer, LOG_TEXT_SIZE));
                FST_ERROR("Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe0.point_.x_, pe0.point_.y_, pe0.point_.z_, pe0.euler_.a_, pe0.euler_.b_, pe0.euler_.c_);
                FST_ERROR("Pose delta 1: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe1.point_.x_, pe1.point_.y_, pe1.point_.z_, pe1.euler_.a_, pe1.euler_.b_, pe1.euler_.c_);
                FST_ERROR("Pose delta 2: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe2.point_.x_, pe2.point_.y_, pe2.point_.z_, pe2.euler_.a_, pe2.euler_.b_, pe2.euler_.c_);
                return MC_COMPUTE_IK_FAIL;
            }

            for (uint32_t j = 0; j < joint_num_; j++)
            {
                state.omega[j] = (joint_delta1[j] - state.angle[j]) / delta_time;
                state.alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time - state.omega[j]) / delta_time;
                state.torque[j] = 0;
            }
        }
        else if (frame_type_ == USER)
        {
            for (uint32_t j = 0; j < 3; j++)
            {
                if (axis_ds_curve_valid_[j] == false)
                {
                    sample.point_[j] = cart_end_.point_[j];
                    sample_delta1.point_[j] = cart_end_.point_[j];
                    sample_delta2.point_[j] = cart_end_.point_[j];
                    continue;
                }

                double t_sample = t - axis_ds_curve_start_time_[j];
                trip = cart_end_.point_[j] - cart_start_.point_[j];
                axis_ds_curve_[j].sampleDSCurve(t_sample, p, v, a);
                sample.point_[j] = cart_start_.point_[j] + p * trip;
                axis_ds_curve_[j].sampleDSCurve(t_sample + delta_time, p, v, a);
                sample_delta1.point_[j] = cart_start_.point_[j] + p * trip;
                axis_ds_curve_[j].sampleDSCurve(t_sample + delta_time * 2, p, v, a);
                sample_delta2.point_[j] = cart_start_.point_[j] + p * trip;
            }
            
            for (uint32_t j = 0; j < 3; j++)
            {
                if (axis_ds_curve_valid_[j + 3] == false)
                {
                    sample.euler_[j] = cart_end_.euler_[j];
                    sample_delta1.euler_[j] = cart_end_.euler_[j];
                    sample_delta2.euler_[j] = cart_end_.euler_[j];
                    continue;
                }

                double t_sample = t - axis_ds_curve_start_time_[j + 3];
                trip = cart_end_.euler_[j] - cart_start_.euler_[j];
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample, p, v, a);
                sample.euler_[j] = cart_start_.euler_[j] + p * trip;
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample + delta_time, p, v, a);
                sample_delta1.euler_[j] = cart_start_.euler_[j] + p * trip;
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample + delta_time * 2, p, v, a);
                sample_delta2.euler_[j] = cart_start_.euler_[j] + p * trip;
            }

            Joint joint_delta1, joint_delta2;
            TransMatrix trans, trans_delta1, trans_delta2, trans_auxiliary_coord;
            auxiliary_coord_.convertToTransMatrix(trans_auxiliary_coord);
            sample.convertToTransMatrix(trans);
            sample_delta1.convertToTransMatrix(trans_delta1);
            sample_delta2.convertToTransMatrix(trans_delta2);
            trans.leftMultiply(uf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta1.leftMultiply(uf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta2.leftMultiply(uf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            
            if (!kinematics_ptr_->doIK(trans, reference, state.angle) ||
				!kinematics_ptr_->doIK(trans_delta1, reference, joint_delta1) ||
				!kinematics_ptr_->doIK(trans_delta2, reference, joint_delta2))
            {
                char buffer[LOG_TEXT_SIZE];
                PoseEuler pe0, pe1, pe2;
                trans.convertToPoseEuler(pe0);
                trans_delta1.convertToPoseEuler(pe1);
                trans_delta2.convertToPoseEuler(pe2);
                FST_ERROR("Compute IK failed.");
                FST_ERROR("Reference: %s", printDBLine(&reference.j1_, buffer, LOG_TEXT_SIZE));
                FST_ERROR("Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe0.point_.x_, pe0.point_.y_, pe0.point_.z_, pe0.euler_.a_, pe0.euler_.b_, pe0.euler_.c_);
                FST_ERROR("Pose delta 1: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe1.point_.x_, pe1.point_.y_, pe1.point_.z_, pe1.euler_.a_, pe1.euler_.b_, pe1.euler_.c_);
                FST_ERROR("Pose delta 2: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe2.point_.x_, pe2.point_.y_, pe2.point_.z_, pe2.euler_.a_, pe2.euler_.b_, pe2.euler_.c_);
                return MC_COMPUTE_IK_FAIL;
            }

            for (uint32_t j = 0; j < joint_num_; j++)
            {
                state.omega[j] = (joint_delta1[j] - state.angle[j]) / delta_time;
                state.alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time - state.omega[j]) / delta_time;
                state.torque[j] = 0;
            }
        }
        else if (frame_type_ == WORLD)
        {
            for (uint32_t j = 0; j < 3; j++)
            {
                if (axis_ds_curve_valid_[j] == false)
                {
                    sample.point_[j] = cart_end_.point_[j];
                    sample_delta1.point_[j] = cart_end_.point_[j];
                    sample_delta2.point_[j] = cart_end_.point_[j];
                    continue;
                }

                double t_sample = t - axis_ds_curve_start_time_[j];
                trip = cart_end_.point_[j] - cart_start_.point_[j];
                axis_ds_curve_[j].sampleDSCurve(t_sample, p, v, a);
                sample.point_[j] = cart_start_.point_[j] + p * trip;
                axis_ds_curve_[j].sampleDSCurve(t_sample + delta_time, p, v, a);
                sample_delta1.point_[j] = cart_start_.point_[j] + p * trip;
                axis_ds_curve_[j].sampleDSCurve(t_sample + delta_time * 2, p, v, a);
                sample_delta2.point_[j] = cart_start_.point_[j] + p * trip;
            }
            
            for (uint32_t j = 0; j < 3; j++)
            {
                if (axis_ds_curve_valid_[j + 3] == false)
                {
                    sample.euler_[j] = cart_end_.euler_[j];
                    sample_delta1.euler_[j] = cart_end_.euler_[j];
                    sample_delta2.euler_[j] = cart_end_.euler_[j];
                    continue;
                }

                double t_sample = t - axis_ds_curve_start_time_[j + 3];
                trip = cart_end_.euler_[j] - cart_start_.euler_[j];
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample, p, v, a);
                sample.euler_[j] = cart_start_.euler_[j] + p * trip;
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample + delta_time, p, v, a);
                sample_delta1.euler_[j] = cart_start_.euler_[j] + p * trip;
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample + delta_time * 2, p, v, a);
                sample_delta2.euler_[j] = cart_start_.euler_[j] + p * trip;
            }

            Joint joint_delta1, joint_delta2;
            TransMatrix trans, trans_delta1, trans_delta2, trans_auxiliary_coord;
            auxiliary_coord_.convertToTransMatrix(trans_auxiliary_coord);
            sample.convertToTransMatrix(trans);
            sample_delta1.convertToTransMatrix(trans_delta1);
            sample_delta2.convertToTransMatrix(trans_delta2);
            trans.leftMultiply(wf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta1.leftMultiply(wf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta2.leftMultiply(wf_matrix_).rightMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            
            if (!kinematics_ptr_->doIK(trans, reference, state.angle) ||
				!kinematics_ptr_->doIK(trans_delta1, reference, joint_delta1) ||
				!kinematics_ptr_->doIK(trans_delta2, reference, joint_delta2))
            {
                char buffer[LOG_TEXT_SIZE];
                PoseEuler pe0, pe1, pe2;
                trans.convertToPoseEuler(pe0);
                trans_delta1.convertToPoseEuler(pe1);
                trans_delta2.convertToPoseEuler(pe2);
                FST_ERROR("Compute IK failed.");
                FST_ERROR("Reference: %s", printDBLine(&reference.j1_, buffer, LOG_TEXT_SIZE));
                FST_ERROR("Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe0.point_.x_, pe0.point_.y_, pe0.point_.z_, pe0.euler_.a_, pe0.euler_.b_, pe0.euler_.c_);
                FST_ERROR("Pose delta 1: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe1.point_.x_, pe1.point_.y_, pe1.point_.z_, pe1.euler_.a_, pe1.euler_.b_, pe1.euler_.c_);
                FST_ERROR("Pose delta 2: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe2.point_.x_, pe2.point_.y_, pe2.point_.z_, pe2.euler_.a_, pe2.euler_.b_, pe2.euler_.c_);
                return MC_COMPUTE_IK_FAIL;
            }

            for (uint32_t j = 0; j < joint_num_; j++)
            {
                state.omega[j] = (joint_delta1[j] - state.angle[j]) / delta_time;
                state.alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time - state.omega[j]) / delta_time;
                state.torque[j] = 0;
            }
        }
        else if (frame_type_ == TOOL)
        {
            for (uint32_t j = 0; j < 3; j++)
            {
                if (axis_ds_curve_valid_[j] == false)
                {
                    sample.point_[j] = cart_end_.point_[j];
                    sample_delta1.point_[j] = cart_end_.point_[j];
                    sample_delta2.point_[j] = cart_end_.point_[j];
                    continue;
                }

                double t_sample = t - axis_ds_curve_start_time_[j];
                trip = cart_end_.point_[j] - cart_start_.point_[j];
                axis_ds_curve_[j].sampleDSCurve(t_sample, p, v, a);
                sample.point_[j] = cart_start_.point_[j] + p * trip;
                axis_ds_curve_[j].sampleDSCurve(t_sample + delta_time, p, v, a);
                sample_delta1.point_[j] = cart_start_.point_[j] + p * trip;
                axis_ds_curve_[j].sampleDSCurve(t_sample + delta_time * 2, p, v, a);
                sample_delta2.point_[j] = cart_start_.point_[j] + p * trip;
            }
            
            for (uint32_t j = 0; j < 3; j++)
            {
                if (axis_ds_curve_valid_[j + 3] == false)
                {
                    sample.euler_[j] = cart_end_.euler_[j];
                    sample_delta1.euler_[j] = cart_end_.euler_[j];
                    sample_delta2.euler_[j] = cart_end_.euler_[j];
                    continue;
                }

                double t_sample = t - axis_ds_curve_start_time_[j + 3];
                trip = cart_end_.euler_[j] - cart_start_.euler_[j];
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample, p, v, a);
                sample.euler_[j] = cart_start_.euler_[j] + p * trip;
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample + delta_time, p, v, a);
                sample_delta1.euler_[j] = cart_start_.euler_[j] + p * trip;
                axis_ds_curve_[j + 3].sampleDSCurve(t_sample + delta_time * 2, p, v, a);
                sample_delta2.euler_[j] = cart_start_.euler_[j] + p * trip;
            }

            Joint joint_delta1, joint_delta2;
            TransMatrix trans, trans_delta1, trans_delta2, trans_auxiliary_coord;
            auxiliary_coord_.convertToTransMatrix(trans_auxiliary_coord);
            
            sample.convertToTransMatrix(trans);
            sample_delta1.convertToTransMatrix(trans_delta1);
            sample_delta2.convertToTransMatrix(trans_delta2);
            trans.leftMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta1.leftMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            trans_delta2.leftMultiply(trans_auxiliary_coord).rightMultiply(tf_matrix_inverse_);
            
            if (!kinematics_ptr_->doIK(trans, reference, state.angle) ||
				!kinematics_ptr_->doIK(trans_delta1, reference, joint_delta1) ||
				!kinematics_ptr_->doIK(trans_delta2, reference, joint_delta2))
            {
                char buffer[LOG_TEXT_SIZE];
                PoseEuler pe0, pe1, pe2;
                trans.convertToPoseEuler(pe0);
                trans_delta1.convertToPoseEuler(pe1);
                trans_delta2.convertToPoseEuler(pe2);
                FST_ERROR("Compute IK failed.");
                FST_ERROR("Reference: %s", printDBLine(&reference.j1_, buffer, LOG_TEXT_SIZE));
                FST_ERROR("Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe0.point_.x_, pe0.point_.y_, pe0.point_.z_, pe0.euler_.a_, pe0.euler_.b_, pe0.euler_.c_);
                FST_ERROR("Pose delta 1: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe1.point_.x_, pe1.point_.y_, pe1.point_.z_, pe1.euler_.a_, pe1.euler_.b_, pe1.euler_.c_);
                FST_ERROR("Pose delta 2: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", pe2.point_.x_, pe2.point_.y_, pe2.point_.z_, pe2.euler_.a_, pe2.euler_.b_, pe2.euler_.c_);
                return MC_COMPUTE_IK_FAIL;
            }

            for (uint32_t j = 0; j < joint_num_; j++)
            {
                state.omega[j] = (joint_delta1[j] - state.angle[j]) / delta_time;
                state.alpha[j] = ((joint_delta2[j] - joint_delta1[j]) / delta_time - state.omega[j]) / delta_time;
                state.torque[j] = 0;
            }
        }
        else
        {
            FST_ERROR("Sample manual trajectory failed, type: %d, frame: %d", motion_type_, frame_type_);
            return MC_MANUAL_FRAME_ERROR;
        }

        dynamics_ptr_->getTorqueInverseDynamics(state.angle, (*(JointVelocity*)(&state.omega)), (*(JointAcceleration*)(&state.alpha)), (*(JointTorque*)(&state.torque)));
        return SUCCESS;
    }

    FST_ERROR("Internal fault, type: %d, frame: %d, in file: %s line: %d", motion_type_, frame_type_, __FILE__, __LINE__);
    return MC_INTERNAL_FAULT;
}

PoseEuler ManualTeach::interpolatPoseEuler(double p, const PoseEuler &start, const PoseEuler &end)
{
    PoseEuler result;
    result.point_.x_ = (1 - p) * start.point_.x_ + p * end.point_.x_;
    result.point_.y_ = (1 - p) * start.point_.y_ + p * end.point_.y_;
    result.point_.z_ = (1 - p) * start.point_.z_ + p * end.point_.z_;
    result.euler_.a_ = (1 - p) * start.euler_.a_ + p * end.euler_.a_;
    result.euler_.b_ = (1 - p) * start.euler_.b_ + p * end.euler_.b_;
    result.euler_.c_ = (1 - p) * start.euler_.c_ + p * end.euler_.c_;
    return result;
}

ErrorCode ManualTeach::manualStop(MotionTime stop_time)
{
    FST_INFO("stopTeach: stop-time=%.6f, total-duration=%.6f", stop_time, total_duration_);

    if (motion_type_ == CONTINUOUS)
    {
        ManualDirection direction[NUM_OF_JOINT] = {STANDING};
        manualContinuous(direction, stop_time);
        FST_INFO("Success, total-duration=%.6f", total_duration_);
        return SUCCESS;
    }
    else if (motion_type_ == APOINT)
    {
        ds_curve_.planStopDSCurve(stop_time);
        total_duration_ = ds_curve_.getDuration();
        FST_INFO("Success, total duration=%.6f", total_duration_);
        return SUCCESS;
    }
    else
    {
        FST_INFO("Cannot stop manual motion in step mode, total-duration=%.6f", total_duration_);
        return SUCCESS;
    }
}

char* ManualTeach::printDBLine(const int *data, char *buffer, uint32_t length)
{
    int len = 0;

    for (uint32_t i = 0; i < joint_num_; i++)
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

char* ManualTeach::printDBLine(const double *data, char *buffer, uint32_t length)
{
    int len = 0;

    for (uint32_t i = 0; i < joint_num_; i++)
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


