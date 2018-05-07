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

ErrorCode createTrajectoryFromPath(const ControlPoint &prev_point, ControlPoint &this_point);

void computeDurationMax(Angle* start_joint_ptr, Angle* end_joint_ptr, Omega* start_omega_ptr, 
                                Alpha* acc_limit, Omega* velocity_limit, MotionTime& duration_max);

void computeDurationMin(Angle* start_joint_ptr, Angle* end_joint_ptr, Omega* start_omega_ptr, 
                                Alpha* acc_limit, Omega* velocity_limit, MotionTime& duration_min);

void computeLastDurationMin(Angle* start_joint_ptr, Angle* end_joint_ptr, Omega* start_omega_ptr, MotionTime& duration_min);

void computeTrajectory(bool is_pause, bool is_forward, size_t target_tick, Angle* start_joint_ptr, Angle* end_joint_ptr,
                            Omega* start_omega_ptr, MotionTime duration, Alpha* acc_limit, Omega* velocity_limit, ControlPoint* target);

ErrorCode forwardTrajectory(ControlPoint &prev, ControlPoint &next,
                            MotionTime expect_duration, Omega *omega_limit);
ErrorCode backwardTrajectory(ControlPoint &prev, ControlPoint &next,
                             MotionTime expect_duration, Omega *omega_limit);

}

#endif
