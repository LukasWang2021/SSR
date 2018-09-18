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
#include <path_plan.h>


#define AUTO_CACHE_SIZE     2

namespace fst_mc
{


class BaseGroup
{
  public:
    BaseGroup(fst_log::Logger* plog);
    virtual ~BaseGroup();

    virtual ErrorCode initGroup(fst_base::ErrorMonitor *error_monitor_ptr) = 0;
    virtual ErrorCode stopGroup(void);
    virtual ErrorCode resetGroup(void);
    virtual ErrorCode clearGroup(void);

    void getLatestJoint(Joint &joint);
    void getServoState(ServoState &state);
    void getGroupState(GroupState &state);
    Joint getLatestJoint(void);
    GroupState getGroupState(void);
    ServoState getServoState(void);


    // Frame handle APIs:
    virtual ErrorCode setToolFrame(const PoseEuler &tf);
    virtual ErrorCode setUserFrame(const PoseEuler &uf);
    virtual ErrorCode setWorldFrame(const PoseEuler &wf);

    // Auto move APIs:
    virtual ErrorCode autoMove(int id, const MotionTarget &target);
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
    ErrorCode setGlobalVelRatio(double ratio);
    ErrorCode setGlobalAccRatio(double ratio);
    double getGlobalVelRatio(void);
    double getGlobalAccRatio(void);

    // More APIs:
    virtual size_t getNumberOfJoint(void) = 0;
    virtual size_t getFIFOLength(void) = 0;

    virtual BaseKinematics* getKinematicsPtr(void);
    virtual Calibrator* getCalibratorPtr(void);
    virtual Constraint* getSoftConstraintPtr(void);

    void realtimeTask(void);
    void activeRealtimeTask(void);
    void inactiveRealtimeTask(void);


  protected:
    virtual ErrorCode autoJoint(const Joint &target, double vel, double cnt, int id);
    virtual ErrorCode autoLine(const PoseEuler &target, double vel, double cnt, int id);
    virtual ErrorCode autoCircle(const PoseEuler &target1, const PoseEuler &target2, double vel, double cnt, int id);

    virtual ErrorCode prepareCache(TrajectoryCache &cache);
    virtual ErrorCode preplanCache(TrajectoryCache &cache);
    virtual ErrorCode smoothJoint2Joint(const JointPoint &ps, const JointPoint &pe, MotionTime smooth_time, TrajSegment &seg);

    virtual ErrorCode sendPoint(void);
    virtual ErrorCode pickFromManual(TrajectoryPoint *point, size_t &length);
    virtual ErrorCode pickFromManualJoint(TrajectoryPoint *point, size_t &length);
    virtual ErrorCode pickFromManualCartesian(TrajectoryPoint *point, size_t &length);
    virtual ErrorCode pickFromAuto(TrajectoryPoint *point, size_t &length);
    virtual ErrorCode createTrajectory(void);
    virtual ErrorCode sampleTrajectorySegment(const TrajSegment (&segment)[NUM_OF_JOINT], double time, Joint &angle, Joint &omega, Joint &alpha);

    virtual char* printDBLine(const int *data, char *buffer, size_t length) = 0;
    virtual char* printDBLine(const double *data, char *buffer, size_t length) = 0;

    inline void reportError(const ErrorCode &error);
    bool updateJointStateFromBareCore(void);

    bool    rt_task_active_;
    double  vel_ratio_, acc_ratio_;
    double  axis_vel_[NUM_OF_JOINT];
    double  axis_acc_[NUM_OF_JOINT];
    size_t  auto_pick_segment_;

    Constraint  hard_constraint_;
    Constraint  soft_constraint_;
    Constraint  firm_constraint_;

    Joint current_joint_;
    ServoState  servo_state_;
    GroupState  group_state_;

    TrajectoryCache     *auto_pick_ptr_;
    TrajectoryCache     *auto_cache_ptr_;

    Joint       jerk_;
    Joint       start_joint_;
    MotionTime  cycle_time_;
    MotionTime  auto_time_;
    MotionTime  manual_time_;
    Calibrator  calibrator_;
    ManualFrame manual_frame_;
    ManualTeach manual_teach_;

    pthread_mutex_t auto_mutex_;
    pthread_mutex_t manual_mutex_;
    pthread_mutex_t servo_mutex_;

    TrajectoryFifo          traj_fifo_;
    BaseKinematics          *kinematics_ptr_;
    ManualTrajectory        manual_traj_;
    BareCoreInterface       bare_core_;
    fst_log::Logger         *log_ptr_;
    fst_base::ErrorMonitor  *error_monitor_ptr_;
};





}







#endif
