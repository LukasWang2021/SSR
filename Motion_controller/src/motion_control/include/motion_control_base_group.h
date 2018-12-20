/*************************************************************************
	> File Name: motion_control_base_group.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 10时58分29秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_BASE_GROUP_H
#define _MOTION_CONTROL_BASE_GROUP_H

#include "common_log.h"
#include "error_monitor.h"
#include "error_code.h"
#include <motion_control_datatype.h>
#include <motion_control_constraint.h>
#include <motion_control_core_interface.h>
#include <motion_control_offset_calibrator.h>
#include <motion_control_manual_teach.h>
#include <motion_control_traj_fifo.h>
#include <arm_kinematics.h>
#include <motion_control_cache_pool.h>
#include <dynamics_interface.h>


#define AUTO_CACHE_SIZE     5

namespace fst_mc
{


class FineWaiter
{
  public:
    FineWaiter();
    ~FineWaiter();
    void initFineWaiter(size_t stable_cycle, double threshold);

    void enableWaiter(const Pose &target);
    void disableWaiter(void);
    void checkWaiter(const Pose &pose);
    bool isEnable(void);
    bool isStable(void);

  private:
    bool    enable_;
    double  threshold_;
    size_t  stable_cnt_;
    size_t  stable_cycle_;
    Pose    waiting_pose_;
};

class BaseGroup
{
  public:
    BaseGroup(fst_log::Logger* plog);
    virtual ~BaseGroup();

    virtual ErrorCode initGroup(fst_base::ErrorMonitor *error_monitor_ptr) = 0;
    virtual ErrorCode stopGroup(void);
    virtual ErrorCode resetGroup(void);
    virtual ErrorCode clearGroup(void);

    void  getLatestJoint(Joint &joint);
    void  getServoState(ServoState &state);
    void  getGroupState(GroupState &state);
    Joint getLatestJoint(void);
    GroupState getGroupState(void);
    ServoState getServoState(void);

    // Frame handle APIs:
    virtual ErrorCode setToolFrame(const PoseEuler &tf);
    virtual ErrorCode setUserFrame(const PoseEuler &uf);
    virtual ErrorCode setWorldFrame(const PoseEuler &wf);

    // Off-line trajectory
    virtual ErrorCode moveOffLineTrajectory(int id, const std::string &file_name);

    // Auto move APIs:
    virtual ErrorCode autoMove(int id, const MotionTarget &target);
    virtual ErrorCode abortMove(void);
    virtual bool nextMovePermitted(void);

    // Manual teach APIs:
    virtual ManualFrame getManualFrame(void);
    virtual ErrorCode setManualFrame(ManualFrame frame);
    virtual double getManualStepAxis(void);
    virtual double getManualStepPosition(void);
    virtual double getManualStepOrientation(void);
    virtual ErrorCode setManualStepAxis(double step);
    virtual ErrorCode setManualStepPosition(double step);
    virtual ErrorCode setManualStepOrientation(double step);
    virtual ErrorCode manualMoveStep(const ManualDirection *direction);
    virtual ErrorCode manualMoveContinuous(const ManualDirection *direction);
    virtual ErrorCode manualMoveToPoint(const Joint &joint);
    virtual ErrorCode manualMoveToPoint(const PoseEuler &pose);
    virtual ErrorCode manualStop(void);

    // Constraints handle APIs:
    virtual ErrorCode setSoftConstraint(const JointConstraint &soft_constraint);
    virtual ErrorCode setFirmConstraint(const JointConstraint &firm_constraint);
    virtual ErrorCode setHardConstraint(const JointConstraint &hard_constraint);
    virtual ErrorCode getSoftConstraint(JointConstraint &soft_constraint);
    virtual ErrorCode getFirmConstraint(JointConstraint &firm_constraint);
    virtual ErrorCode getHardConstraint(JointConstraint &hard_constraint);

    // Global velocity and acceleration APIs
    virtual ErrorCode setGlobalVelRatio(double ratio);
    virtual ErrorCode setGlobalAccRatio(double ratio);
    virtual double getGlobalVelRatio(void);
    virtual double getGlobalAccRatio(void);

    // More APIs:
    virtual size_t getNumberOfJoint(void) = 0;
    virtual size_t getFIFOLength(void) = 0;

    virtual BaseKinematics* getKinematicsPtr(void);
    virtual Calibrator* getCalibratorPtr(void);
    virtual Constraint* getSoftConstraintPtr(void);

    virtual void doCommonLoop(void);
    virtual void doPriorityLoop(void);

  protected:
    void sendTrajectoryFlow(void);
    void updateServoStateAndJoint(void);
    void doStateMachine(void);
    void fillTrajectoryFifo(void);
    bool updateStartJoint(void);

    virtual ErrorCode sendAutoTrajectoryFlow(void);
    virtual ErrorCode sendManualTrajectoryFlow(void);

    virtual ErrorCode autoJoint(const Joint &start, const MotionTarget &target, PathCacheList &path, TrajectoryCacheList &trajectory);
    virtual ErrorCode autoStableJoint(const Joint &start, const MotionTarget &target, PathCache &path, TrajectoryCache &trajectory);
    virtual ErrorCode autoSmoothJoint(const JointState &start_state, const MotionTarget &via, const MotionTarget &target, PathCache &path, TrajectoryCache &trajectory);
    
    virtual ErrorCode autoLine(const Joint &start, const MotionTarget &target, PathCacheList &path, TrajectoryCacheList &trajectory);
    virtual ErrorCode autoStableLine(const Joint &start, const MotionTarget &target, PathCache &path, TrajectoryCache &trajectory);
    virtual ErrorCode autoSmoothLine(const JointState &start_state, const MotionTarget &via, const MotionTarget &target, PathCache &path, TrajectoryCache &trajectory);
    
    virtual ErrorCode autoCircle(const Joint &start, const MotionTarget &target, PathCacheList &path, TrajectoryCacheList &trajectory);
    virtual ErrorCode autoStableCircle(const Joint &start, const MotionTarget &target, PathCache &path, TrajectoryCache &trajectory);
    virtual ErrorCode autoSmoothCircle(const JointState &start_state, const MotionTarget &via, const MotionTarget &target, PathCache &path, TrajectoryCache &trajectory);

    virtual ErrorCode computeInverseKinematicsOnPathCache(const Joint &start, PathCache &path);

    virtual ErrorCode pickPointsFromTrajectoryFifo(TrajectoryPoint *points, size_t &length);
    virtual ErrorCode pickPointsFromManualTrajectory(TrajectoryPoint *points, size_t &length);
    virtual ErrorCode pickPointsFromManualJoint(TrajectoryPoint *points, size_t &length);
    virtual ErrorCode pickPointsFromManualCartesian(TrajectoryPoint *points, size_t &length);

    inline  ErrorCode checkMotionTarget(const MotionTarget &target);
    inline  ErrorCode checkStartState(const Joint &start_joint);

    inline void reportError(const ErrorCode &error);
    inline bool isSameJoint(const Joint &joint1, const Joint &joint2, double thres = MINIMUM_E6);
    inline bool isLastMotionSmooth(void);
    inline void resetManualTrajectory(void);
    inline void freeFirstCacheList(void);
    inline void linkCacheList(PathCacheList *path_ptr, TrajectoryCacheList *traj_ptr);
    inline void updateTimeFromStart(TrajectoryCacheList &cache);
    inline void setFineWaiter(void);
    inline void loopFineWaiter(void);
    
    inline void sampleBlockEnding(const TrajectoryBlock &block, JointState &state);
    inline void sampleBlockStart(const TrajectoryBlock &block, JointState &state);
    inline void sampleBlock(const TrajectoryBlock &block, MotionTime time_from_block, JointState &state);

    inline PathCache* getLastPathCachePtr(void);
    inline PathCacheList* getLastPathCacheListPtr(void);
    inline TrajectoryCache* getLastTrajectoryCachePtr(void);
    inline TrajectoryCacheList* getLastTrajectoryCacheListPtr(void);

    virtual char* printDBLine(const int *data, char *buffer, size_t length) = 0;
    virtual char* printDBLine(const double *data, char *buffer, size_t length) = 0;

    
    double  vel_ratio_, acc_ratio_;
    double  cartesian_vel_min_, cartesian_vel_max_;

    Constraint  hard_constraint_;
    Constraint  soft_constraint_;
    Constraint  firm_constraint_;

    Joint       servo_joint_;
    Joint       start_joint_;
    ServoState  servo_state_;
    GroupState  group_state_;
    
    MotionTime  cycle_time_;
    MotionTime  auto_time_;
    MotionTime  manual_time_;
    MotionTime  duration_per_segment_;

    Calibrator  calibrator_;
    FineWaiter  fine_waiter_;
    ManualFrame manual_frame_;
    ManualTeach manual_teach_;

    BaseKinematics          *kinematics_ptr_;
    ManualTrajectory        manual_traj_;
    BareCoreInterface       bare_core_;
    fst_log::Logger         *log_ptr_;
    fst_base::ErrorMonitor  *error_monitor_ptr_;
    fst_algorithm::DynamicsInterface  *dynamics_ptr_;

    CachePool<PathCacheList>        path_cache_pool_;
    CachePool<TrajectoryCacheList>  traj_cache_pool_;

    TrajectoryFifo      traj_fifo_;
    PathCacheList       *path_list_ptr_;
    TrajectoryCacheList *traj_list_ptr_;

    pthread_mutex_t     cache_list_mutex_;
    pthread_mutex_t     manual_traj_mutex_;
    pthread_mutex_t     servo_mutex_;
    
    bool stop_request_;
    bool reset_request_;
    bool abort_request_;
    bool clear_request_;
    bool error_request_;
    bool auto_to_standby_request_;
    bool manual_to_standby_request_;

    size_t  disable_to_standby_timeout_;
    size_t  standby_to_disable_timeout_;
    size_t  standby_to_auto_timeout_;
    size_t  auto_to_standby_timeout_;
    size_t  manual_to_standby_timeout_;
    size_t  trajectory_flow_timeout_;
    size_t  servo_update_timeout_;
};



/*
class PathCachePool
{
  public:
    PathCachePool(void);
    PathCache* getCachePtr(void);
    void freeCachePtr(PathCache *pcache);

  private:
    PathCache   cache_pool_[PATH_CACHE_SIZE];
    PathCache*  free_cache_list_;
    pthread_mutex_t  mutex_;
};
*/



}







#endif
