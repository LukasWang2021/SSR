#ifndef TRAJ_PARAMS_H
#define TRAJ_PARAMS_H

#include <string>
#include <joint.h>
#include "yaml_help.h"
#include <ds_planner/ds_planner.h>



class TrajParams
{
public:
    TrajParams();
    ~TrajParams();

    void setNumOfJoint(uint32_t joint_num);

    basic_alg::Joint torque_max_;
    basic_alg::Joint omega_max_;
    basic_alg::Joint alpha_max_;
    basic_alg::Joint beta_max_[MAX_JERK_NUM];

    double position_vel_max_;
    double position_acc_max_;
    double position_acc_max_swift_;
    double position_jerk_max_[MAX_JERK_NUM];
    double position_jerk_max_swift_[MAX_JERK_NUM];

    double quaternion_vel_max_;
    double quaternion_acc_max_;
    double quaternion_jerk_max_[MAX_JERK_NUM];

    basic_alg::Joint  omega_max_of_cart_motion_;

    int max_jerk_num_;
    bool dynamics_check_;
    bool adjust_acc_by_vel_;

    double Tsample_;//touch采样时间间隔
    double GEN_TN_;//生成轨迹间隔
	int N_step_P_;
	int N_step_Q_;
    double NinterpP_;
    double NinterpQ_;
    double trj_ratio_;
    int OnlineRecvTmatrixBuffPackLen_;
    bool loadConstraint();
    bool saveConstraint();
    bool loadConfig();

private:
    uint32_t joint_num_;
    base_space::YamlHelp yaml_help_;
    std::string constraint_file_path_;
    std::string config_file_path_;


};

#endif

