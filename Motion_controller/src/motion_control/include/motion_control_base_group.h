/*************************************************************************
	> File Name: motion_control_base_group.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 10时58分29秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_BASE_GROUP_H
#define _MOTION_CONTROL_BASE_GROUP_H

#include <pthread.h>
#include <motion_control_datatype.h>
#include <motion_control_error_code.h>
#include <motion_control_core_interface.h>
#include <log_manager/log_manager_logger.h>


namespace fst_mc
{

enum MotionState
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
    ~BaseGroup();

    virtual ErrorCode stopGroup(void);
    virtual ErrorCode resetGroup(void);
    virtual ErrorCode autoMove(void) = 0;
    virtual ErrorCode manualMove(const ManualDirection directions[NUM_OF_JOINT]) = 0;
    virtual ErrorCode manualMove(const Joint &joint) = 0;
    virtual ErrorCode sendPoint(void) = 0;


  protected:
    virtual bool isJointInConstraint(Joint joint, JointConstraint constraint) = 0;

    MotionState group_state_;

    JointConstraint     soft_constraint_;
    TrajectorySegment*  auto_cache_;
    TrajectorySegment*  manual_cache_;

    Joint       start_joint_;
    MotionTime  cycle_time_;
    MotionTime  auto_time_;
    MotionTime  manual_time_;

    pthread_mutex_t auto_mutex_;
    pthread_mutex_t manual_mutex_;
    
    BareCoreInterface   bare_core_;
    fst_log::Logger*    log_ptr_;
};





}







#endif
