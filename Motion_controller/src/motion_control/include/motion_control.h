#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <unistd.h> 
#include <queue>
#include <coordinate_manager.h>
#include <tool_manager.h>
#include <thread_help.h>
#include <motion_control_datatype.h>
#include <motion_control_arm_group.h>
#include "log_manager_producer.h"
#include "common_error_code.h"
#include "group.h"

namespace group_space
{

class MotionControl : public group_space::Group 
{
public:
    MotionControl(int32_t id);
    ~MotionControl();

    ErrorCode initApplication(fst_ctrl::CoordinateManager *coordinate_manager_ptr, fst_ctrl::ToolManager *tool_manager_ptr);

    // API for teaching
    ManualFrame getManualFrame(void);
    ErrorCode setManualFrame(ManualFrame frame);
    void getAxisManualStep(double (&steps)[NUM_OF_JOINT]);
    double getPositionManualStep(void);
    double getOrientationManualStep(void);
    ErrorCode setAxisManualStep(const double (&steps)[NUM_OF_JOINT]);
    ErrorCode setPositionManualStep(double step);
    ErrorCode setOrientationManualStep(double step);

    ErrorCode doStepManualMove(const GroupDirection &direction);
    ErrorCode doContinuousManualMove(const GroupDirection &direction);
    ErrorCode doGotoPointManualMove(const basic_alg::Joint &joint);
    ErrorCode doGotoPointManualMove(const PoseAndPosture &pose, int user_frame_id, int tool_frame_id);

    // API for auto run
    ErrorCode autoMove(const struct Instruction &instruction);
    ErrorCode restartMove(void);
    ErrorCode pauseMove(void);
    bool isMoving(void);
    
    void clearErrorFlag(void);
    bool nextMovePermitted(void);

    ErrorCode moveOnlineTrajectory(void);//从STANDBY状态切换到ONLINE状态,开始传输在线轨迹数据
    ErrorCode MotionStateOnlineToStandby(void); //从ONLINE状态切换到STANDBY状态并做相关变量设置
    //ErrorCode setOnlinePointBufptr(double * ptr);
    ErrorCode setOnlinePointBufptr();
    ErrorCode setOnlineTrajectoryRatio(double ratio);
    ErrorCode setOnlineVpointCache(int num_matrix,int * p_status, double * p_marixArray);
    // API for off line trajectory
    ErrorCode convertEulerTraj2JointTraj(const std::string &offline_euler_trajectory_fileName);
    ErrorCode Fir_Bspline_algorithm_test2(void);
    ErrorCode receive_T_matrix_data(int status, double * p_marixArray);
    void xzc_funTest();
    ErrorCode setOfflineTrajectory(const std::string &offline_trajectory);
    ErrorCode prepareOfflineTrajectory(void);
    ErrorCode moveOfflineTrajectory(void);

    // API for zero offset and calibrator
    ErrorCode setOffset(size_t index, double offset);
    ErrorCode setOffset(const double (&offset)[NUM_OF_JOINT]);
    void getOffset(double (&offset)[NUM_OF_JOINT]);
    void getOffsetMask(OffsetMask (&mask)[NUM_OF_JOINT]);
    CalibrateState getCalibrateState(void);

    ErrorCode maskOffsetLostError(void);
    ErrorCode setOffsetState(size_t index, OffsetState stat);
    void getOffsetState(OffsetState (&offset_stat)[NUM_OF_JOINT]);

    ErrorCode resetEncoderMultiTurnValue(void);
    void getUsingCoord(basic_alg::PoseEuler &usingcool);
    void getUsingTool(basic_alg::PoseEuler &usingtool);

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
    ErrorCode clearTeachGroup(void);

    // more API
    void shiftCoordOfPose(const basic_alg::PoseEuler &old_coord, const basic_alg::PoseEuler &old_pose, 
                          const basic_alg::PoseEuler &new_coord, basic_alg::PoseEuler &new_pose);
    void shiftToolOfPose(const basic_alg::PoseEuler &old_tool, const basic_alg::PoseEuler &old_pose, 
                         const basic_alg::PoseEuler &new_tool, basic_alg::PoseEuler &new_pose);
    ErrorCode isLinearPathReachable(uint32_t group_id, 
                                    int32_t start_coord_id, int32_t start_tool_id, const PoseAndPosture &start, 
                                    int32_t target_coord_id, int32_t target_tool_id, const PoseAndPosture &target);
    ErrorCode isPoseReachable(uint32_t group_id, const basic_alg::Joint &joint);
    ErrorCode isPoseReachable(uint32_t group_id, int32_t coord_id, int32_t tool_id, const PoseAndPosture &pose);
    ErrorCode convertCartToJoint(const PoseAndPosture &pose, int user_frame_id, int tool_frame_id, basic_alg::Joint &joint);
    ErrorCode convertCartToJoint(const basic_alg::PoseEuler &pose, int user_frame_id, int tool_frame_id, basic_alg::Joint &joint);
    ErrorCode convertJointToCart(const basic_alg::Joint &joint, int user_frame_id, int tool_frame_id, basic_alg::PoseEuler &pose);
    basic_alg::Posture getPostureFromJoint(const basic_alg::Joint &joint);
    basic_alg::Turn getTurnFromJoint(const basic_alg::Joint &joint);

    MotionControlState  getMotionControlState(void);
    ServoState  getServoState(void);
    basic_alg::PoseEuler getCurrentPose(void);
    void    getCurrentPose(basic_alg::PoseEuler &pose);
    basic_alg::Joint getServoJoint(void);

    ErrorCode setGlobalVelRatio(double ratio);
    ErrorCode setGlobalAccRatio(double ratio);
    double getGlobalVelRatio(void);
    double getGlobalAccRatio(void);

    std::string getModelName(void);
    size_t getNumberOfAxis(void);
    void getTypeOfAxis(AxisType *types);
    int getGroupID(void);

    void getToolFrame(int &id);
    void getUserFrame(int &id);
    ErrorCode setToolFrame(int id);
    ErrorCode setUserFrame(int id);

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

    //ssr
    void setWorkMode(UserOpMode mode);
    UserOpMode getWorkMode(void);

    // parameter access
    // ...

    void ringCommonTask(void);
    void ringPlannerTask(void);
    void ringRealTimeTask(void);
    void ringPriorityTask(void);
    void ringOnlineTrajTask(void);

    //pure function no realization
    virtual ErrorCode mcGroupHalt(double dec, double jerk);
    virtual ErrorCode mcGroupInterrupt(double dec, double jerk);
    virtual ErrorCode mcGroupContinue(void);
    virtual ErrorCode mcGroupSetPosition(const std::vector<double> &position, bool relative, CoordType_e coord_type);
    virtual ErrorCode mcMoveDirectAbsolute(const std::vector<double> &position, CoordType_e coord_type,
        double vel_pct, double acc_pct, double jerk);
    virtual ErrorCode mcMoveLinearAbsolute(const std::vector<double> &position, CoordType_e coord_type, 
        double velocity, double acc, double dec, double jerk);
    virtual ErrorCode mcGroupReadActualPosition(CoordType_e coord_type, std::vector<double> &position);
    virtual ErrorCode mcGroupReadActualVelocity(CoordType_e coord_type, std::vector<double> &velocity);
    virtual bool initApplication(void);
    virtual bool reloadSystemModel(void);
    virtual bool pushBackFB(void* fb_ptr);
    virtual base_space::FBQueueStatus_e getFBQStatus();
    virtual void processFBQ();
    virtual void processTBQ();
    virtual void clearBQ();
    
private:
    ErrorCode autoMove(const MotionTarget &target);
    ErrorCode offsetToolFrame(int tool_id, const basic_alg::PoseEuler &offset, basic_alg::PoseEuler &tool_frame);
    ErrorCode handlePoint(const TargetPoint &origin, const basic_alg::PoseEuler &user_frame, const basic_alg::PoseEuler &tool_frame, IntactPoint &point);
    ErrorCode offsetPoint(const basic_alg::Joint &offset_joint, IntactPoint &point);
    ErrorCode offsetPoint(const basic_alg::PoseEuler &offset_frame, const basic_alg::PoseEuler &offset, IntactPoint &point);

    int  user_frame_id_;
    int  tool_frame_id_;
    UserOpMode work_mode_;
    bool motion_error_flag_;

    std::queue<struct Instruction> instruction_fifo_;
    pthread_mutex_t  instruction_mutex_;
    uint32_t instructions_recv_counter_;
    uint32_t instructions_handle_counter_;

    std::mutex online_trajData_mutex_;
    bool flag_recv_new_VPMatrix_= false;// false-现在没有收到VP点矩阵, true-收到VP点矩阵
    int *online_vp_status_; //用于暂存在线轨迹接收途经点矩阵状态
    double *online_vp_cache_;//用于暂存在线轨迹接收途经点矩阵数据

    fst_ctrl::CoordinateManager* coordinate_manager_ptr_;
    fst_ctrl::ToolManager* tool_manager_ptr_;
    BaseGroup *group_ptr_;
    OnlineTrajectoryPlanner *online_trj_planner_ptr;
};



}

#endif



