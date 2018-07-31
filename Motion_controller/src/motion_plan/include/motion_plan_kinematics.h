/*************************************************************************
	> File Name: motion_plan_kinematics.h
	> Author: Feng Yun
	> Mail:   yun.feng@foresight-robotics.com
	> Created Time: 2018年02月01日 星期四 17时34分50秒
 ************************************************************************/

#ifndef _MOTION_PLAN_KINEMATICS_H
#define _MOTION_PLAN_KINEMATICS_H

#include <fst_datatype.h>
#include <motion_plan_error_code.h>

#define NUM_OF_JOINT_IN_ALG    6

namespace fst_algorithm
{

void    forwardKinematics(const fst_controller::Joint &jnt, fst_controller::Pose      &pose);
void    forwardKinematics(const fst_controller::Joint &jnt, fst_controller::PoseEuler &pose);

fst_controller::PoseEuler   forwardKinematics2PoseEuler(const fst_controller::Joint &jnt);
fst_controller::Pose        forwardKinematics(const fst_controller::Joint &jnt);

ErrorCode   inverseKinematics(const fst_controller::PoseEuler &pose, const fst_controller::Joint &ref, fst_controller::Joint &res);
ErrorCode   inverseKinematics(const fst_controller::Pose &pose, const fst_controller::Joint &ref, fst_controller::Joint &res);
ErrorCode   inverseKinematics(const fst_controller::Pose &pose, const fst_controller::Angle *ref, fst_controller::Angle *res);
ErrorCode   chainIK(const fst_controller::Pose &pose, fst_controller::Joint &ref, fst_controller::Angle *res);
ErrorCode   chainIK(const fst_controller::Pose &pose, fst_controller::Joint &ref, fst_controller::Joint &res);
ErrorCode   chainIK(const fst_controller::PoseEuler &pose, fst_controller::Joint &ref, fst_controller::Angle *res);
ErrorCode   chainIK(const fst_controller::PoseEuler &pose, fst_controller::Joint &ref, fst_controller::Joint &res);


}
#endif
