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

ErrorCode createTrajFromPath(const ControlPoint &prev_point, ControlPoint &this_point);

ErrorCode foreCycle(const ControlPoint &prev_point, ControlPoint &this_point, int flg);

ErrorCode backCycle(ControlPoint &next_point, ControlPoint &this_point, int flg);

void computeDurationMax(Angle* start_joint_ptr, Angle* end_joint_ptr, Omega* start_omega_ptr, 
                                Alpha* acc_limit, MotionTime& duration_max);

void computeTrajectory(bool is_forward, size_t target_tick, Angle* start_joint_ptr,
                       Angle* end_joint_ptr, Omega* start_omega_ptr, 
                       MotionTime duration_max, ControlPoint* target);


}

#endif
