/*************************************************************************
	> File Name: motion_control_plan_alg.h
	> Author: 
	> Mail: 
	> Created Time: 2018年11月16日 星期五 14时28分58秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_PLAN_ALG_H
#define _MOTION_CONTROL_PLAN_ALG_H

#include <motion_control_datatype.h>

// 前序指令不带平滑时的路径规划
ErrorCode planPathJoint(const Joint &start, const MotionTarget &target, PathCache &path_cache);
ErrorCode planPathLine(const PoseEuler &start, const MotionTarget &target, PathCache &path_cache);
ErrorCode planPathCircle(const PoseEuler &start, const MotionTarget &target, PathCache &path_cache);

// 前序指令带平滑时的路径规划
ErrorCode planPathSmoothJoint(const Joint &start, const MotionTarget &via, const MotionTarget &target, PathCache &path_cache);
ErrorCode planPathSmoothLine(const PoseEuler &start, const MotionTarget &via, const MotionTarget &target, PathCache &path_cache);
ErrorCode planPathSmoothCircle(const PoseEuler &start, const MotionTarget &via, const MotionTarget &target, PathCache &path_cache);

// 前序指令不带平滑时的轨迹规划
ErrorCode planTrajectory(const PathCache &path_cache, const JointState &start_state, double vel_ratio, double acc_ratio, TrajectoryCache &traj_cache);
// 前序指令带平滑时的轨迹规划
ErrorCode planTrajectorySmooth(const PathCache &path_cache, const JointState &start_state, const MotionTarget &via, double vel_ratio, double acc_ratio, TrajectoryCache &traj_cache);
// 暂停轨迹规划
ErrorCode planPauseTrajectory(const PathCache &path_cache, const JointState &start_state, double acc_ratio, rajectoryCache &traj_cache, int &path_stop_index);

#endif
