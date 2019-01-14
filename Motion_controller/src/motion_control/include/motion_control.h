#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H


#include <common_log.h>
#include <motion_control_param.h>
#include <device_manager.h>
#include <axis_group_manager.h>
#include <coordinate_manager.h>
#include <tool_manager.h>
#include <motion_control_arm_group.h>
#include <error_monitor.h>
#include <error_code.h>
#include <thread_help.h>
#include <motion_control_ros_basic.h>


namespace fst_mc
{
class MotionControl
{
public:
    MotionControl();
    ~MotionControl();

    ErrorCode init(fst_hal::DeviceManager *device_manager_ptr, AxisGroupManager *axis_group_manager_ptr,
                   fst_ctrl::CoordinateManager *coordinate_manager_ptr, fst_ctrl::ToolManager *tool_manager_ptr,
                   fst_base::ErrorMonitor *error_monitor_ptr);

    // API for teaching
    ManualFrame getManualFrame(void);
    ErrorCode setManualFrame(ManualFrame frame);
    double getRotateManualStep(void);
    double getPrismaticManualStep(void);
    double getPositionManualStep(void);
    double getOrientationManualStep(void);
    ErrorCode setRotateManualStep(double step);
    ErrorCode setPrismaticManualStep(double step);
    ErrorCode setPositionManualStep(double step);
    ErrorCode setOrientationManualStep(double step);

    ErrorCode doStepManualMove(const GroupDirection &direction);
    ErrorCode doContinuousManualMove(const GroupDirection &direction);
    ErrorCode doGotoPointManualMove(const Joint &joint);
    ErrorCode doGotoPointManualMove(const PoseEuler &pose);
    ErrorCode manualStop(void);

    // API for auto run
    ErrorCode autoMove(int id, const MotionTarget &target);
    ErrorCode abortMove(void);
    bool nextMovePermitted(void);

    // API for zero offset and calibrator
    void setOffset(size_t index, double offset);
    void setOffset(const double (&offset)[NUM_OF_JOINT]);
    void getOffset(double (&offset)[NUM_OF_JOINT]);
    void getOffsetMask(OffsetMask (&mask)[NUM_OF_JOINT]);
    CalibrateState getCalibrateState(void);

    ErrorCode saveJoint(void);
    ErrorCode saveOffset(void);
    ErrorCode checkOffset(CalibrateState &cali_stat, OffsetState (&offset_stat)[NUM_OF_JOINT]);
    ErrorCode maskOffsetLostError(void);
    ErrorCode setOffsetState(size_t index, OffsetState stat);

    ErrorCode calibrateOffset(void);
    ErrorCode calibrateOffset(size_t index);
    ErrorCode calibrateOffset(const size_t *pindex, size_t length);

    bool isReferenceAvailable(void);
    ErrorCode deleteReference(void);
    ErrorCode saveReference(void);
    ErrorCode fastCalibrate(void);
    ErrorCode fastCalibrate(size_t index);
    ErrorCode fastCalibrate(const size_t *pindex, size_t length);

    // API for soft and hard constraint
    ErrorCode getSoftConstraint(JointConstraint &soft_constraint);
    ErrorCode getFirmConstraint(JointConstraint &firm_constraint);
    ErrorCode getHardConstraint(JointConstraint &hard_constraint);
    ErrorCode setSoftConstraint(const JointConstraint &soft_constraint);
    ErrorCode setFirmConstraint(const JointConstraint &firm_constraint);
    ErrorCode setHardConstraint(const JointConstraint &hard_constraint);


    // API for Axis Group Enable/Disable/Halt/Stop/Reset
    ErrorCode stopGroup(void);
    ErrorCode resetGroup(void);
    ErrorCode clearGroup(void);

    // more API
    ErrorCode  convertCartToJoint(const PoseEuler &pose, int user_frame_id, int tool_frame_id, Joint &joint);
    ErrorCode  convertJointToCart(const Joint &joint, int user_frame_id, int tool_frame_id, PoseEuler &pose);

    GroupState  getGroupState(void);
    ServoState  getServoState(void);
    PoseEuler   getCurrentPose(void);
    void    getCurrentPose(PoseEuler &pose);
    Joint   getServoJoint(void);
    void    getServoJoint(Joint &joint);

    size_t  getFIFOLength(void);

    ErrorCode setGlobalVelRatio(double ratio);
    ErrorCode setGlobalAccRatio(double ratio);
    double getGlobalVelRatio(void);
    double getGlobalAccRatio(void);


    void getToolFrame(int &id);
    void getUserFrame(int &id);
    ErrorCode setToolFrame(int id);
    ErrorCode setUserFrame(int id);

    // parameter access
    // ...

    void ringCommonTask(void);
    void ringPriorityTask(void);
    
private:
    int  user_frame_id_;
    int  tool_frame_id_;

    bool rt_thread_running_;
    bool non_rt_thread_running_;

    MotionControlParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_hal::DeviceManager* device_manager_ptr_;
    AxisGroupManager* axis_group_manager_ptr_;
    fst_ctrl::CoordinateManager* coordinate_manager_ptr_;
    fst_ctrl::ToolManager* tool_manager_ptr_;
    fst_base::ErrorMonitor *error_monitor_ptr_;
    BaseGroup *group_ptr_;
    fst_base::ThreadHelp rt_thread_;
    fst_base::ThreadHelp non_rt_thread_;

    RosBasic        *ros_basic_ptr_;
};



}

#endif
