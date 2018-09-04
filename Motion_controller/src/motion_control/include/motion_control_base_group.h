/*************************************************************************
	> File Name: motion_control_base_group.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 10时58分29秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_BASE_GROUP_H
#define _MOTION_CONTROL_BASE_GROUP_H

#include <pthread.h>
#include "common_log.h"
#include "error_monitor.h"
#include "error_code.h"
#include <motion_control_datatype.h>
#include <motion_control_constraint.h>
#include <motion_control_core_interface.h>
#include <motion_control_offset_calibrator.h>
#include <motion_control_manual_teach.h>
#include <arm_kinematics.h>


namespace fst_mc
{

enum GroupState
{
    STANDBY = 0,
    MANUAL = 1,
    AUTO = 2,
    PAUSE = 3,

    MANUAL_TO_STANDBY = 110,
    STANDBY_TO_MANUAL = 101,
    AUTO_TO_STANDBY = 120,
    STANDBY_TO_AUTO = 102,
    AUTO_TO_PAUSE = 123,
    PAUSE_TO_AUTO = 132,
    PAUSE_TO_STANDBY = 130,
};


class BaseGroup
{
  public:
    BaseGroup(fst_log::Logger* plog);
    virtual ~BaseGroup();

    virtual ErrorCode initGroup(fst_base::ErrorMonitor *error_monitor_ptr) = 0;

    virtual ErrorCode stopGroup(void);
    virtual ErrorCode resetGroup(void);

    virtual ErrorCode autoMove(void) = 0;

    virtual MotionFrame getMotionFrame(void);
    virtual ErrorCode setMotionFrame(MotionFrame frame);

    virtual ErrorCode setToolFrame(const PoseEuler &tf);
    virtual ErrorCode setUserFrame(const PoseEuler &uf);
    virtual ErrorCode setWorldFrame(const PoseEuler &wf);

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

    virtual size_t getNumberOfJoint(void) = 0;
    virtual size_t getFIFOLength(void) = 0;

    virtual Calibrator* getCalibratorPtr(void) = 0;
    virtual Constraint* getSoftConstraintPtr(void);

    virtual ErrorCode setSoftConstraint(const JointConstraint &soft_constraint);
    virtual ErrorCode setFirmConstraint(const JointConstraint &firm_constraint);
    virtual ErrorCode setHardConstraint(const JointConstraint &hard_constraint);
    virtual ErrorCode getSoftConstraint(JointConstraint &soft_constraint);
    virtual ErrorCode getFirmConstraint(JointConstraint &firm_constraint);
    virtual ErrorCode getHardConstraint(JointConstraint &hard_constraint);

    virtual ErrorCode getJointFromPose(const PoseEuler &pose, Joint &joint);
    virtual ErrorCode getPoseFromJoint(const Joint &joint, PoseEuler &pose);

    void realtimeTask(void);
    void activeRealtimeTask(void);
    void inactiveRealtimeTask(void);

    void getLatestJoint(Joint &joint);
    void getServoState(ServoState &state);
    void getGroupState(GroupState &state);

    Joint getLatestJoint(void);
    GroupState getGroupState(void);
    ServoState getServoState(void);

    ErrorCode setGlobalVelRatio(double ratio);
    ErrorCode setGlobalAccRatio(double ratio);
    double getGlobalVelRatio(void);
    double getGlobalAccRatio(void);

  protected:
    virtual ErrorCode sendPoint(void);
    virtual ErrorCode pickFromManual(TrajectoryPoint *point, size_t &length) = 0;
    virtual ErrorCode pickFromManualJoint(TrajectoryPoint *point, size_t &length) = 0;
    virtual ErrorCode pickFromManualCartesian(TrajectoryPoint *point, size_t &length) = 0;
    virtual bool isJointInConstraint(Joint joint, JointConstraint constraint) = 0;

    virtual char* printDBLine(const int *data, char *buffer, size_t length) = 0;
    virtual char* printDBLine(const double *data, char *buffer, size_t length) = 0;

    inline void reportError(const ErrorCode &error);
    bool updateJointStateFromBareCore(void);

    bool    rt_task_active_;
    double  vel_ratio_, acc_ratio_;

    Constraint  hard_constraint_;
    Constraint  soft_constraint_;
    Constraint  firm_constraint_;

    Joint current_joint_;
    ServoState  servo_state_;
    GroupState  group_state_;

    TrajectorySegment*  auto_cache_;
    TrajectorySegment*  manual_cache_;

    Joint       start_joint_;
    MotionTime  cycle_time_;
    MotionTime  auto_time_;
    MotionTime  manual_time_;
    Calibrator  calibrator_;
    MotionFrame motion_frame_;
    ManualTeach manual_teach_;

    pthread_mutex_t auto_mutex_;
    pthread_mutex_t manual_mutex_;
    pthread_mutex_t servo_mutex_;

    BaseKinematics          *kinematics_ptr_;
    ManualTrajectory        manual_traj_;
    BareCoreInterface       bare_core_;
    fst_log::Logger         *log_ptr_;
    fst_base::ErrorMonitor  *error_monitor_ptr_;
};





}







#endif
