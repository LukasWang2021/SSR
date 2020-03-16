#include <iostream>
#include <vector>
#include <string>
#include "common_file_path.h"
#include "traj_params.h"

using namespace std;

TrajParams::TrajParams()
{
    joint_num_ = 0;
    constraint_file_path_ = constraint_file_path_ + ALGORITHM_DIR + "constraint.yaml";
    config_file_path_ = config_file_path_ + ALGORITHM_DIR + "config.yaml";
}

TrajParams::~TrajParams()
{

}

void TrajParams::setNumOfJoint(uint32_t joint_num)
{
    joint_num_ = joint_num;
}

bool TrajParams::loadConstraint()
{
    vector<double> torque_max;
    vector<double> omega_max; 
    vector<double> alpha_max;
    vector<double> beta_max_acc;
    vector<double> beta_max_running;
    vector<double> beta_max_dec;
    vector<double> cart_omega_max;

    if (!yaml_help_.loadParamFile(constraint_file_path_.c_str())
        || !yaml_help_.getParam("joint/torque_max", torque_max)
        || !yaml_help_.getParam("joint/omega_max", omega_max)
        || !yaml_help_.getParam("joint/alpha_max", alpha_max)
        || !yaml_help_.getParam("joint/beta_max_acc", beta_max_acc)
        || !yaml_help_.getParam("joint/beta_max_running", beta_max_running)
        || !yaml_help_.getParam("joint/beta_max_dec", beta_max_dec)
        || !yaml_help_.getParam("cart/omega_max", cart_omega_max)
        || !yaml_help_.getParam("cart/position/vel_max", position_vel_max_)
        || !yaml_help_.getParam("cart/position/acc_max", position_acc_max_)
        || !yaml_help_.getParam("cart/position/jerk_max_acc", position_jerk_max_[0])
        || !yaml_help_.getParam("cart/position/jerk_max_dec", position_jerk_max_[2])
        || !yaml_help_.getParam("cart/position/jerk_max_running", position_jerk_max_[1])
        || !yaml_help_.getParam("cart/quaternion/vel_max", quaternion_vel_max_)
        || !yaml_help_.getParam("cart/quaternion/acc_max", quaternion_acc_max_)
        || !yaml_help_.getParam("cart/quaternion/jerk_max_acc", quaternion_jerk_max_[0])
        || !yaml_help_.getParam("cart/quaternion/jerk_max_dec", quaternion_jerk_max_[2])
        || !yaml_help_.getParam("cart/quaternion/jerk_max_running", quaternion_jerk_max_[1]))
    {
        std::cout << " Failed load constraint.yaml " << std::endl;
        return false;
    }
    
    /*
    if (!yaml_help_.loadParamFile(constraint_file_path_.c_str()))
    {
        std::cout << " Failed load constraint.yaml " << std::endl;
        return false;
    }

    if (!yaml_help_.getParam("joint/omega_max", omega_max)
        || !yaml_help_.getParam("joint/alpha_max", alpha_max)
        || !yaml_help_.getParam("joint/beta_max_acc", beta_max_acc)
        || !yaml_help_.getParam("joint/beta_max_running", beta_max_running)
        || !yaml_help_.getParam("joint/beta_max_dec", beta_max_dec))
    {
        std::cout << " Failed load joint constraint" << std::endl;
        return false;
    }

    if (!yaml_help_.getParam("cart/position/vel_max", position_vel_max_)
        || !yaml_help_.getParam("cart/position/acc_max", position_acc_max_)
        || !yaml_help_.getParam("cart/position/jerk_max_acc", position_jerk_max_[0])
        || !yaml_help_.getParam("cart/position/jerk_max_dec", position_jerk_max_[2])
        || !yaml_help_.getParam("cart/position/jerk_max_running", position_jerk_max_[1]))
    {
        std::cout << " Failed load position constraint" << std::endl;
        return false;
    }

    if (!yaml_help_.getParam("cart/quaternion/vel_max", quaternion_vel_max_)
        || !yaml_help_.getParam("cart/quaternion/acc_max", quaternion_acc_max_)
        || !yaml_help_.getParam("cart/quaternion/jerk_max_acc", quaternion_jerk_max_[0])
        || !yaml_help_.getParam("cart/quaternion/jerk_max_dec", quaternion_jerk_max_[2])
        || !yaml_help_.getParam("cart/quaternion/jerk_max_running", quaternion_jerk_max_[1]))
    {
        std::cout << " Failed load quaternion constraint" << std::endl;
        return false;
    }
    */

    for (uint32_t j = 0; j < joint_num_; ++j)
    {
        omega_max_of_cart_motion_[j] = cart_omega_max[j];
        torque_max_[j] = torque_max[j];
        omega_max_[j] = omega_max[j];
        alpha_max_[j] = alpha_max[j];
        beta_max_[0][j] =  beta_max_acc[j];
        beta_max_[1][j] =  beta_max_running[j];
        beta_max_[2][j] =  beta_max_dec[j];
    }

    return true;
}

bool TrajParams::saveConstraint()
{
    vector<double> omega_max; 
    vector<double> alpha_max;
    vector<double> beta_max_acc;
    vector<double> beta_max_dec;
    vector<double> beta_max_running;
    
    for (uint32_t j = 0; j < joint_num_; ++j)
    {
        omega_max[j] = omega_max_[j];
        alpha_max[j] = alpha_max_[j];
        beta_max_acc[j] =  beta_max_[0][j];
        beta_max_running[j] =  beta_max_[1][j];
        beta_max_dec[j] =  beta_max_[2][j];
    }

    if (!yaml_help_.setParam("joint/omega_max", omega_max)
        || !yaml_help_.setParam("joint/alpha_max", alpha_max)
        || !yaml_help_.setParam("joint/beta_max_acc", beta_max_acc)
        || !yaml_help_.setParam("joint/beta_max_running", beta_max_running)
        || !yaml_help_.setParam("joint/beta_max_dec", beta_max_dec)
        || !yaml_help_.setParam("cart/position/vel_max", position_vel_max_)
        || !yaml_help_.setParam("cart/position/acc_max", position_acc_max_)
        || !yaml_help_.setParam("cart/position/jerk_max_acc", position_jerk_max_[0])
        || !yaml_help_.setParam("cart/position/jerk_max_running", position_jerk_max_[1])
        || !yaml_help_.setParam("cart/position/jerk_max_dec", position_jerk_max_[2])
        || !yaml_help_.setParam("cart/quaternion/vel_max", quaternion_vel_max_)
        || !yaml_help_.setParam("cart/quaternion/acc_max", quaternion_acc_max_)
        || !yaml_help_.setParam("cart/quaternion/jerk_max_acc", quaternion_jerk_max_[0])
        || !yaml_help_.setParam("cart/quaternion/jerk_max_running", quaternion_jerk_max_[1])
        || !yaml_help_.setParam("cart/quaternion/jerk_max_dec", quaternion_jerk_max_[2])
        || !yaml_help_.dumpParamFile(constraint_file_path_.c_str()))
    {
        std::cout << " Failed save constraint.yaml " << std::endl;
        return false;
    }

    for (uint32_t j = 0; j < joint_num_; ++j)
    {
        omega_max_[j] = omega_max_[j];
        alpha_max_[j] = alpha_max_[j];
        beta_max_[0][j] = beta_max_acc[j];
        beta_max_[1][j] = beta_max_running[j];
        beta_max_[2][j] = beta_max_dec[j];
    }

    return true;
}


bool TrajParams::loadConfig()
{
    if (!yaml_help_.loadParamFile(config_file_path_.c_str())
        || !yaml_help_.getParam("max_jerk_num", max_jerk_num_)
        || !yaml_help_.getParam("inverse_dynamics_check", dynamics_check_)
        || !yaml_help_.getParam("adjust_acc_by_vel", adjust_acc_by_vel_))
    {
        std::cout << " Failed load config.yaml " << std::endl;
        return false;
    }

    return true;
}
