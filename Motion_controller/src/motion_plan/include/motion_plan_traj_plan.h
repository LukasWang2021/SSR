/*************************************************************************
	> File Name: motion_plan_traj_plan.h
	> Author: Feng Yun
	> Mail:   yun.feng@foresight-robotics.com
	> Created Time: 2018年02月08日 星期四 18时18分39秒
 ************************************************************************/

#ifndef _MOTION_PLAN_TRAJ_PLAN_H
#define _MOTION_PLAN_TRAJ_PLAN_H

#include <fst_datatype.h>

namespace fst_controller
{


ErrorCode computeAlphaLimit(const double *joint, const double *omega, double *alpha_upper, double *alpha_lower);

ErrorCode createSpeedUpTraj(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                            Alpha *alpha_upper, Alpha *alpha_lower);

ErrorCode createSpeedUpTraj(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration);

ErrorCode createBackwardSpeedUpTraj(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                                    Alpha *alpha_upper, Alpha *alpha_lower);

ErrorCode uniformTrajectory(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration, TrajSegment &seg);



ErrorCode forwardTrajectory(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                            Omega *omega_limit, Alpha *alpha_upper, Alpha *alpha_lower);

ErrorCode backwardTrajectory(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                             Omega *omega_limit, Alpha *alpha_upper, Alpha *alpha_lower);


ErrorCode forwardTrajectory(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                            Omega *omega_limit, TrajSegment &seg);

ErrorCode forwardUniformTrajectory(ControlPoint &prev, ControlPoint &next, MotionTime expect_duration,
                                   Omega *omega_limit, TrajSegment &seg);


}

#endif
