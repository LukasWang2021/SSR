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
#include <error_code.h>

namespace fst_mc
{

class BaseKinematics
{
  public:
    BaseKinematics(void);
    virtual ~BaseKinematics(void) = 0;

    virtual void initKinematics(double (&dh_matrix)[NUM_OF_JOINT][4]) = 0;

    virtual void forwardKinematicsInBase(const Joint &joint, Pose      &pose) = 0;
    virtual void forwardKinematicsInBase(const Joint &joint, PoseEuler &pose) = 0;
    virtual void forwardKinematicsInUser(const Joint &joint, Pose      &pose) = 0;
    virtual void forwardKinematicsInUser(const Joint &joint, PoseEuler &pose) = 0;
    virtual void forwardKinematicsInWorld(const Joint &joint, Pose      &pose) = 0;
    virtual void forwardKinematicsInWorld(const Joint &joint, PoseEuler &pose) = 0;

    //virtual Pose      getPoseFromJoint(const Joint &joint) = 0;
    //virtual PoseEuler getPoseEulerFromJoint(const Joint &joint) = 0;

    virtual ErrorCode   inverseKinematicsInBase(const PoseEuler &pose, const Joint &ref, Joint &res) = 0;
    virtual ErrorCode   inverseKinematicsInBase(const Pose &pose, const Joint &ref, Joint &res) = 0;

    //virtual ErrorCode   chainIK(const Pose &pose, Joint &ref, Joint &res) = 0;
    //virtual ErrorCode   chainIK(const PoseEuler &pose, Joint &ref, Joint &res) = 0;

    ErrorCode setUserFrame(PoseEuler uf);
    ErrorCode setToolFrame(PoseEuler tf);

  protected:
    virtual void forwardKinematics(const Joint &joint, basic_alg::Matrix &matrix) = 0;
    virtual ErrorCode inverseKinematics(const basic_alg::Matrix &matrix, Joint (&solutions)[8], size_t length) = 0;
    virtual ErrorCode inverseKinematics(const basic_alg::Matrix &matrix, const Joint &ref, Joint &res) = 0;

    double dh_matrix_[NUM_OF_JOINT][4];
    basic_alg::Matrix   user_frame_;
    basic_alg::Matrix   tool_frame_;
    basic_alg::Matrix   world_frame_;
    basic_alg::Matrix   inverse_user_frame_;
    basic_alg::Matrix   inverse_tool_frame_;
    basic_alg::Matrix   inverse_world_frame_;
};





}
#endif
