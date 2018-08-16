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
#include <motion_control_datatype.h>
#include <motion_control_error_code.h>
#include <motion_control_core_interface.h>


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
    
    virtual ErrorCode setManualMode(ManualMode mode) = 0;
    virtual ErrorCode setManualFrame(ManualFrame frame) = 0;
    virtual ErrorCode manualMove(const ManualDirection directions[NUM_OF_JOINT]) = 0;
    virtual ErrorCode manualMove(const Joint &joint) = 0;
    virtual ErrorCode manualStop(void) = 0;

    virtual size_t getFIFOLength(void) = 0;

    ErrorCode realtimeTask(void);

    void getLatestJoint(Joint &joint);
    void getServoState(ServoState &state);
    void getGroupState(GroupState &state);

    Joint getLatestJoint(void);
    GroupState getGroupState(void);
    ServoState getServoState(void);





  protected:
    virtual ErrorCode sendPoint(void);
    virtual ErrorCode pickFromManual(TrajectoryPoint *point, size_t &length) = 0;
    virtual ErrorCode pickFromManualJoint(TrajectoryPoint *point, size_t &length) = 0;
    virtual ErrorCode pickFromManualCartesian(TrajectoryPoint *point, size_t &length) = 0;
    virtual bool isJointInConstraint(Joint joint, JointConstraint constraint) = 0;

    inline void reportError(const ErrorCode &error);
    bool updateJointStateFromBareCore(void);

    Joint current_joint_;
    ServoState  servo_state_;
    GroupState  group_state_;

    JointConstraint     soft_constraint_;
    TrajectorySegment*  auto_cache_;
    TrajectorySegment*  manual_cache_;

    Joint       start_joint_;
    MotionTime  cycle_time_;
    MotionTime  auto_time_;
    MotionTime  manual_time_;

    pthread_mutex_t auto_mutex_;
    pthread_mutex_t manual_mutex_;
    pthread_mutex_t servo_mutex_;
    
    BareCoreInterface       bare_core_;
    fst_log::Logger         *log_ptr_;
    fst_base::ErrorMonitor  *error_monitor_ptr_;
};





}







#endif
