/*************************************************************************
	> File Name: motion_control_base_group.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月07日 星期二 10时58分29秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_BASE_GROUP_H
#define _MOTION_CONTROL_BASE_GROUP_H

#include <sys/time.h>
#include <fstream>
#include <vector>
#include <error_queue.h>
#include <common_error_code.h>
#include <joint_constraint.h>
#include <motion_control_datatype.h>
#include <motion_control_core_interface.h>
#include <motion_control_offset_calibrator.h>
#include <motion_control_manual_teach.h>
#include <kinematics_rtm.h>
#include <dynamic_alg.h>
#include <dynamic_alg_rtm.h>
#include <transformation.h>
#include <coordinate_manager.h>
#include <tool_manager.h>
#include <traj_planner.h>
#include <smooth_planner.h>
#include <lock_free_fifo.h>
#include "pause_resume_planner.h"
#include "group.h"
#include "onlineTrj_planner.h"
#include "given_vel_planner.h"

#define TRAJECTORY_CACHE_SIZE     8
#define OFFLINE_TRAJECTORY_CACHE_SIZE  150// 512
#define TRAJECTORY_LOG_CONTROL_SIZE 1024    // 1KB
#define TRAJECTORY_LOG_DATA_SIZE 67108864   // 64MB

namespace group_space
{

struct TrajectoryCache
{
    SmoothPlanner smooth;
    TrajectoryPlanner trajectory;
    double smooth_in_time;
    bool valid;
    bool start_from_smooth;
    bool end_with_smooth;
    double smooth_time;
    double smooth_distance;
    TrajectoryCache *prev;
    TrajectoryCache *next;
};

struct TrajectoryLogControl
{
    char model_name[32];
    uint32_t max_of_points;
    uint32_t num_of_points;
    uint32_t write_index;
};

class BaseGroup
{
  public:
    BaseGroup();
    virtual ~BaseGroup();

    virtual ErrorCode initGroup(fst_ctrl::CoordinateManager *coordinate_manager_ptr, fst_ctrl::ToolManager *tool_manager_ptr, 
        std::map<int32_t, axis_space::Axis*>* axis_group_ptr, GroupSm* sm_ptr,servo_comm_space::ServoCpuCommBase* cpu_comm_ptr,
        system_model_space::GroupModel_t* db_ptr) = 0;
    virtual ErrorCode checkGroupZeroOffset(void) = 0;

    virtual ErrorCode stopGroup(void);
    virtual ErrorCode clearGroup(void);
    virtual ErrorCode clearTeachGroup(void);

    basic_alg::Joint getLatestJoint(void);
    MotionControlState getMotionControlState(void);
    ServoState getServoState(void);

    void setOnlineTrjFirstPointCondition();//检测到在线轨迹起点,设置一些初始条件
    ErrorCode switchToOnlineState();//进入ONLINE状态
    ErrorCode switchOnlineStateToStandby();//从ONLINE状态切换到STANDBY状态
    // Auto move APIs:
    virtual ErrorCode autoMove(const MotionInfo &info);
    virtual ErrorCode pauseMove(void);
    virtual ErrorCode restartMove(void);
    virtual bool isMoving(void);
    virtual bool nextMovePermitted(void);
    //ErrorCode setOnlinePointBufData(double * p_doublePointdata);
    void setOnlinePointLevelBuf(int idx, int value);
    ErrorCode setOnlineTrjPointBufData(double * trj_point_buf, int *level_buf, uint32_t size);//将xyzabc数据逆解为轴角数据后整理为轨迹数据,然后传送给在线轨迹缓存
    ErrorCode setOnlinePoint_TMatrixBufData(double * p_doublePointdata,uint32_t size);
    // API for offline trajectory
    virtual ErrorCode  readEulerTrajectoryFile(const std::string &offline_euler_trajectory_filePath,std::vector<std::vector<double>>& euler_trajArr);
    virtual ErrorCode setOfflineTrajectory(const std::string &offline_trajectory);
    virtual ErrorCode setOfflineViaPoints(const std::vector<PoseEuler> &via_points, bool is_new);
    virtual basic_alg::Joint getStartJointOfOfflineTrajectory(void);
    virtual ErrorCode moveOfflineTrajectory(void);
    virtual ErrorCode planOfflineTrajectory(std::string traj_name, double traj_vel);
    virtual ErrorCode planOfflinePause(void);
    virtual ErrorCode planOfflineResume(void);

    // Manual teach APIs:
    virtual ManualFrame getManualFrame(void);
    virtual ErrorCode setManualFrame(ManualFrame frame);
    virtual void getManualStepAxis(double *steps);
    virtual double getManualStepPosition(void);
    virtual double getManualStepOrientation(void);
    virtual ErrorCode setManualStepAxis(const double *steps);
    virtual ErrorCode setManualStepPosition(double step);
    virtual ErrorCode setManualStepOrientation(double step);
    virtual ErrorCode manualMoveStep(const ManualDirection *direction);
    virtual ErrorCode manualMoveContinuous(const ManualDirection *direction);
    virtual ErrorCode manualMoveToPoint(const IntactPoint &point);
    virtual bool updateContinuousManualMoveRpcTime();
    virtual void handleContinueousManualRpcTimeOut();
 
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
    virtual char* getModelName(char *buffer, size_t length) = 0;
    virtual size_t getNumberOfJoint(void) = 0;
    virtual void getTypeOfAxis(AxisType *types);
    virtual int getID(void);

    // Frame handle APIs:
    void setUserFrame(const basic_alg::PoseEuler &uf);
    void setToolFrame(const basic_alg::PoseEuler &tf);
    void setWorldFrame(const basic_alg::PoseEuler &wf);
    const basic_alg::PoseEuler& getUserFrame(void);
    const basic_alg::PoseEuler& getToolFrame(void);
    const basic_alg::PoseEuler& getWorldFrame(void);

    // payload
    ErrorCode setPayload(int id);
    void getPayload(int &id);
    ErrorCode addPayload(const basic_alg::PayloadInfo& info);
    ErrorCode deletePayload(int id);
    ErrorCode updatePayload(const basic_alg::PayloadInfo& info);
    ErrorCode movePayload(int expect_id, int original_id);
    ErrorCode getPayloadInfoById(int id, basic_alg::PayloadInfo& info);
    std::vector<basic_alg::PayloadSummaryInfo> getAllValidPayloadSummaryInfo(void);
    void getAllValidPayloadSummaryInfo(std::vector<basic_alg::PayloadSummaryInfo>& info_list);

    ErrorCode convertCartToJoint(const PoseAndPosture &pose, const basic_alg::PoseEuler &uf, const basic_alg::PoseEuler &tf, basic_alg::Joint &joint);
    ErrorCode convertCartToJoint(const basic_alg::PoseEuler &pose, const basic_alg::PoseEuler &uf, const basic_alg::PoseEuler &tf, basic_alg::Joint &joint);
    ErrorCode convertJointToCart(const basic_alg::Joint &joint, const basic_alg::PoseEuler &uf, const basic_alg::PoseEuler &tf, basic_alg::PoseEuler &pose);
    ErrorCode convertCartToJoint(const PoseAndPosture &pose, basic_alg::Joint &joint);
    ErrorCode convertCartToJoint(const basic_alg::PoseEuler &pose, basic_alg::Joint &joint);
    ErrorCode convertJointToCart(const basic_alg::Joint &joint, basic_alg::PoseEuler &pose);
    ErrorCode convertJointToTmx(const Joint &joint, TransMatrix &tmx);
    ErrorCode isLinearPathReachable(const IntactPoint &start, const IntactPoint &target);

    double decouplingAxis6ByRad(double fifth_pos, double sixth_pos);
    
    virtual basic_alg::Transformation* getTransformationPtr(void);
    virtual basic_alg::Kinematics* getKinematicsPtr(void);
    virtual Calibrator* getCalibratorPtr(void);
    virtual Constraint* getSoftConstraintPtr(void);

    virtual bool isPostureMatch(const basic_alg::Posture &posture_1, const basic_alg::Posture &posture_2);
    
    virtual void doCommonLoop(void);
    virtual void doPriorityLoop(void);
    virtual void doRealtimeLoop(void);

    virtual char* printDBLine(const int *data, char *buffer, size_t length) = 0;
    virtual char* printDBLine(const double *data, char *buffer, size_t length) = 0;
    
  protected:
    void sendTrajectoryFlow(void);
    void updateServoStateAndJoint(void);

    // state machine
    void doStateMachine(void);
    void doDisableToStandby(const ServoState &servo_state, uint32_t &fail_counter);
    void doStandbyToDisable(const ServoState &servo_state, uint32_t &fail_counter);
    void doAutoToStandby(const ServoState &servo_state, uint32_t &fail_counter, uint32_t &fine_counter);
    void doStandbyToAuto(const ServoState &servo_state, uint32_t &fail_counter);
    void doManualToStandby(const ServoState &servo_state, uint32_t &fail_counter);
    void doPauseManualToPause(const ServoState &servo_state, uint32_t &fail_counter);
    void doStandbyToManual(void);
    void doPauseToManual(void);
    void doOfflineToStandby(const ServoState &servo_state, uint32_t &fail_counter);
    void doStandbyToOffline(void);
    void doPausingToPause(const ServoState &servo_state, uint32_t &fail_counter);
    void doPausingOfflineToPause(const ServoState &servo_state, uint32_t &fail_counter);
    
    ErrorCode checkManualTrajectory(double start_time, double end_time, double step_time, basic_alg::Joint reference);
    ErrorCode planPauseTrajectory(void);
    ErrorCode planResumeTrajectory(void);
    ErrorCode planPauseReturnTrajectory(void);

    void fillTrajectoryFifo(void);
    bool updateStartJoint(void);
    void updateJointRecorder(void);
    void manualStop(void);
    void manualStopWithLock(void);

    virtual ErrorCode sendAutoTrajectoryFlow(void);
    virtual ErrorCode sendManualTrajectoryFlow(void);
    virtual ErrorCode sendOnlineTrajectoryFlow(void);

    virtual ErrorCode fillManualFIFO(void);
    virtual ErrorCode pickManualPoint(TrajectoryPoint &point);
    virtual ErrorCode pickPointsFromTrajectoryFifo(TrajectoryPoint *points, size_t &length);
    virtual ErrorCode pickPointsFromManualTrajectory(TrajectoryPoint *points, size_t &length);

    int fillOnlineFIFO(int startIdx);
    ErrorCode pickOnlinePoint(TrajectoryPoint &point,int pickCnt);
    ErrorCode pickPointsFromOnlineTrajectory(TrajectoryPoint *points, size_t &length);

    

    ErrorCode checkMotionTarget(const MotionInfo &info);
    //ErrorCode checkStartState(const basic_alg::Joint &start_joint);

    void reportError(const ErrorCode &error);
    bool isSameJoint(const basic_alg::Joint &joint1, const basic_alg::Joint &joint2, double thres = MINIMUM_E6);
    bool isSameJoint(const basic_alg::Joint &joint1, const basic_alg::Joint &joint2, const basic_alg::Joint &thres);

    bool fillOfflineCache(void);
    bool fillOfflinePauseCache(void);
    uint32_t getOfflineCacheSize(void);
    ErrorCode sendOfflineTrajectoryFlow(void);
    ErrorCode pickPointsFromOfflineCache(TrajectoryPoint *points, size_t &length);
    
    bool initTrajectoryLogSpace(void);
    std::string getMCServoStatusString(ServoState servo_status);
    std::string getMontionControlStatusString(MotionControlState mc_status);
    
    int id_;
    JointPlanner joint_planner_;
    PauseResumePlanner pause_resume_planner_;
    std::vector<JointState> pause_trajectory_;
    std::vector<JointState> remain_trajectory_;
    std::vector<JointState> resume_trajectory_;
    std::vector<JointState> origin_trajectory_;
    basic_alg::Joint pause_joint_;
    basic_alg::Joint start_joint_before_pause_;
    bool pause_flag_;
    double resume_trajectory_time_stamp_;
    double pause_trajectory_time_stamp_;
    double offline_trajectory_time_stamp_;
    TrajectoryPlanner planner_for_check_;
    TrajectoryCache trajectory_a_;
    TrajectoryCache trajectory_b_;
    TrajectoryCache *plan_traj_ptr_;
    TrajectoryCache *pick_traj_ptr_;
    LockFreeFIFO<TrajectoryPoint> traj_fifo_;
    LockFreeFIFO<TrajectoryPoint> manual_fifo_;
    LockFreeFIFO<TrajectoryPoint> online_fifo_;
    uint32_t traj_fifo_lower_limit_;
    bool filling_points_into_traj_fifo_;
    bool start_of_motion_;
    bool fine_enable_;
    double fine_threshold_;
    uint32_t fine_cycle_;
    basic_alg::PoseQuaternion fine_pose_;
    basic_alg::Joint manaul_reference_;
    bool manual_trajectory_check_fail_;
    
    double  vel_ratio_, acc_ratio_;
    double  cartesian_vel_min_, cartesian_vel_max_;

    Constraint  hard_constraint_;
    Constraint  soft_constraint_;
    Constraint  firm_constraint_;

    basic_alg::PoseEuler world_frame_;
    basic_alg::PoseEuler user_frame_;
    basic_alg::PoseEuler tool_frame_;
    basic_alg::Joint joint_tracking_accuracy_;
    basic_alg::Joint servo_joint_;
    basic_alg::Joint start_joint_;
    ServoState  servo_state_;
    MotionControlState  mc_state_;
    uint32_t    encoder_state_[NUM_OF_JOINT];
    AxisType    type_of_axis_[NUM_OF_JOINT];
    
    
    MotionTime  cycle_time_;
    MotionTime  auto_time_;
    MotionTime  manual_time_;
    MotionTime  online_time_;
    MotionTime  duration_per_segment_;

    Calibrator  calibrator_;
    ManualTeach manual_teach_;

    BareCoreInterface       bare_core_;
    fst_ctrl::CoordinateManager *coordinate_manager_ptr_;
    fst_ctrl::ToolManager   *tool_manager_ptr_;
    basic_alg::Kinematics   *kinematics_ptr_;
    basic_alg::Transformation   transformation_;
    basic_alg::DynamicAlg   *dynamics_ptr_;

    GivenVelocityPlanner offline_planner_;
    std::ifstream offline_trajectory_file_;
    std::ifstream offline_euler_trajectory_file_;
    std::string offline_trajectory_file_name_;
    OnlineTrajectoryPlanner   online_trj_planner_ptr;
    bool offline_trajectory_first_point_;
    bool offline_trajectory_last_point_;
    bool online_trajectory_first_point_;
    bool online_trajectory_last_point_;
    uint32_t offline_trajectory_size_;
    basic_alg::Joint offline_start_joint_;
    TrajectoryPoint offline_trajectory_cache_[OFFLINE_TRAJECTORY_CACHE_SIZE];
    uint32_t offline_trajectory_cache_head_, offline_trajectory_cache_tail_;
    uint32_t offline_traj_point_read_cnt_;
    pthread_mutex_t     planner_list_mutex_;
    pthread_mutex_t     manual_traj_mutex_;
    pthread_mutex_t     manual_rpc_mutex_;
    pthread_mutex_t     online_traj_mutex_;
    pthread_mutex_t     online_rpc_mutex_;
    pthread_mutex_t     servo_mutex_;
    pthread_mutex_t     offline_mutex_;

    bool clear_request_;
    bool stop_barecore_;
    bool clear_teach_request_;

    bool auto_to_pause_request_;
    bool pause_to_auto_request_;
    bool pause_to_manual_request_;
    bool standby_to_auto_request_;
    bool auto_to_standby_request_;
    bool manual_to_pause_request_;
    bool standby_to_manual_request_;
    bool manual_to_standby_request_;

    bool offline_to_pause_request_;
    bool pause_to_offline_request_;
    bool pause_offline_to_standby_request_;
    bool pausing_offline_to_pause_request_;
    bool standby_to_offline_request_;
    bool offline_to_standby_request_;

    bool pause_return_to_pause_request_;
    bool pausing_to_pause_request_;
    bool standby_to_online_request_;
    bool online_to_standby_request_;
    bool online_to_pause_request_;


    size_t  disable_to_standby_timeout_;
    size_t  standby_to_disable_timeout_;
    size_t  standby_to_auto_timeout_;
    size_t  auto_to_standby_timeout_;
    size_t  manual_to_standby_timeout_;
    size_t  offline_to_standby_timeout_;
    size_t  auto_to_pause_timeout_;
    size_t  pause_to_standby_timeout_;
    size_t  trajectory_flow_timeout_;
    size_t  joint_record_update_timeout_;
    size_t  joint_record_update_cycle_;
    bool is_continuous_manual_move_timeout_;
    bool is_continuous_manual_time_count_valid_;
    struct timeval last_continuous_manual_move_rpc_time_; 

    bool traj_log_enable_;
    TrajectoryPoint* traj_log_data_ptr_;
    TrajectoryLogControl *traj_log_ctrl_ptr_;

    std::map<int32_t, axis_space::Axis*>* axis_group_ptr_;  /**< The list of the axes in the group.*/
	  GroupSm* sm_ptr_;                                       /**< The state machine of the group.*/
    servo_comm_space::ServoCpuCommBase* cpu_comm_ptr_;      /**< The pointer to communicate with the other cpu.*/
    system_model_space::GroupModel_t* db_ptr_;              /**< The pointer of the parameters of the group model.*/
};







}







#endif
