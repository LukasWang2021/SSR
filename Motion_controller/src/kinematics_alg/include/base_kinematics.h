/*************************************************************************
	> File Name: base_kinematics.h
	> Author: Feng Yun
	> Mail:   yun.feng@foresight-robotics.com
	> Created Time: 2018年02月01日 星期四 17时34分50秒
 ************************************************************************/

#ifndef _BASE_KINEMATICS_H
#define _BASE_KINEMATICS_H

#include <math.h>
#include <basic_alg_datatype.h>
#include <basic_constants.h>
#include <basic_matrix.h>
#include <error_code.h>

using namespace basic_alg;

namespace fst_mc
{


class BaseKinematics
{
  public:
    BaseKinematics(void);
    virtual ~BaseKinematics(void) = 0;

    virtual void initKinematics(double (&dh_matrix)[NUM_OF_JOINT][4]);

    // universal FK/IK API
    virtual void forwardKinematics(const Joint &joint, const PoseEuler &user, const PoseEuler &tool, PoseEuler &pose);
    virtual void forwardKinematics(const Joint &joint, const basic_alg::Matrix &user, const basic_alg::Matrix &tool, PoseEuler &pose);
    virtual ErrorCode inverseKinematics(const PoseEuler &pose, const PoseEuler &user, const PoseEuler &tool, const Joint &ref, Joint &res);
    virtual ErrorCode inverseKinematics(const PoseEuler &pose, const basic_alg::Matrix &user, const basic_alg::Matrix &tool, const Joint &ref, Joint &res);

    // FK/IK using activated frame and tool
    virtual void forwardKinematicsInBase(const Joint &joint, PoseQuaternion &pose);
    virtual void forwardKinematicsInBase(const Joint &joint, PoseEuler &pose);
    virtual void forwardKinematicsInUser(const Joint &joint, PoseQuaternion &pose);
    virtual void forwardKinematicsInUser(const Joint &joint, PoseEuler &pose);
    virtual void forwardKinematicsInWorld(const Joint &joint, PoseQuaternion &pose);
    virtual void forwardKinematicsInWorld(const Joint &joint, PoseEuler &pose);

    virtual ErrorCode inverseKinematicsInBase(const PoseQuaternion &pose, const Joint &ref, Joint &res);
    virtual ErrorCode inverseKinematicsInBase(const PoseEuler &pose, const Joint &ref, Joint &res);
    virtual ErrorCode inverseKinematicsInUser(const PoseQuaternion &pose, const Joint &ref, Joint &res);
    virtual ErrorCode inverseKinematicsInUser(const PoseEuler &pose, const Joint &ref, Joint &res);
    virtual ErrorCode inverseKinematicsInWorld(const PoseQuaternion &pose, const Joint &ref, Joint &res);
    virtual ErrorCode inverseKinematicsInWorld(const PoseEuler &pose, const Joint &ref, Joint &res);
    virtual ErrorCode inverseKinematicsInTool(const basic_alg::Matrix &tool_coordinate, const PoseEuler &pose, const Joint &ref, Joint &res);

    //virtual ErrorCode   chainIK(const Pose &pose, Joint &ref, Joint &res) = 0;
    //virtual ErrorCode   chainIK(const PoseEuler &pose, Joint &ref, Joint &res) = 0;

    const basic_alg::Matrix& getWorldFrame(void) const;
    const basic_alg::Matrix& getUserFrame(void) const;
    const basic_alg::Matrix& getToolFrame(void) const;

    ErrorCode setWorldFrame(const PoseEuler &wf);
    ErrorCode setWorldFrame(const basic_alg::Matrix &wf);
    ErrorCode setUserFrame(const PoseEuler &uf);
    ErrorCode setUserFrame(const basic_alg::Matrix &uf);
    ErrorCode setToolFrame(const PoseEuler &tf);
    ErrorCode setToolFrame(const basic_alg::Matrix &tf);


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


void inline reviseJoint(double &jnt, double ref, double upper_limit, double lower_limit)
{
    jnt = jnt - round((jnt - ref) / 2 / PI) * PI * 2;
    if      (jnt > upper_limit)  jnt = jnt - ceil((jnt - upper_limit) / 2 / PI) * PI * 2;
    else if (jnt < lower_limit)  jnt = jnt - ceil((jnt - lower_limit) / 2 / PI) * PI * 2;
}



}
#endif
