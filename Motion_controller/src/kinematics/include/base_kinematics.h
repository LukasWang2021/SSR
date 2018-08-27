/*************************************************************************
	> File Name: base_kinematics.h
	> Author: Feng Yun
	> Mail:   yun.feng@foresight-robotics.com
	> Created Time: 2018年02月01日 星期四 17时34分50秒
 ************************************************************************/

#ifndef _BASE_KINEMATICS_H
#define _BASE_KINEMATICS_H

#include <base_datatype.h>
#include <basic_matrix.h>

namespace fst_mc
{

class BaseKinematics
{
  public:
    BaseKinematics(void);
    virtual ~BaseKinematics(void) = 0;

    virtual void initKinematics(void) = 0;

    virtual void forwardKinematics(const Joint &joint, Pose      &pose) = 0;
    virtual void forwardKinematics(const Joint &joint, PoseEuler &pose) = 0;

    virtual Pose      getPoseFromJoint(const Joint &joint) = 0;
    virtual PoseEuler getPoseEulerFromJoint(const Joint &joint) = 0;

    virtual ErrorCode   inverseKinematics(const PoseEuler &pose, const Joint &ref, Joint &res) = 0;
    virtual ErrorCode   inverseKinematics(const Pose &pose, const Joint &ref, Joint &res) = 0;

    virtual ErrorCode   chainIK(const Pose &pose, Joint &ref, Joint &res) = 0;
    virtual ErrorCode   chainIK(const PoseEuler &pose, Joint &ref, Joint &res) = 0;

  private:
    double dh_matrix_[NUM_OF_JOINT][4];
    basic_alg::Matrix   user_frame_;
    basic_alg::Matrix   tool_frame_;
    basic_alg::Matrix   inverse_user_frame_;
    basic_alg::Matrix   inverse_tool_frame_;
};





}
#endif
