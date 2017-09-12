/**
 * @file robot_motion_.h
 * @brief: changed the defination of state, mode and add ProgramState 
 * @author Wang Wei
 * @version 2.0.3
 * @date 2017-03-13
 */
#ifndef TP_INTERFACE_ROBOT_MOTION_H_
#define	TP_INTERFACE_ROBOT_MOTION_H_

#include "motion_controller/motion_controller_arm_group.h"
#include "parameter_manager/parameter_manager_param_group.h"
#include "robot.h"
#include "motionSL.pb.h"
#include "threadsafe_vector.h"
#include "threadsafe_list.h"
#include "proto_define.h"
#include "share_mem.h"
#include "fst_error.h"
#include "common.h"
#include "safety_interface.h"
#include "io_interface.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <atomic>
#include <map>

using namespace fst_controller;
using namespace fst_parameter;

#define MAX_CNT_VAL		(100)

#define MIN_POINTS_FOR_NEXT_CMD     (150)
#define MAX_PLANNED_POINTS_NUM		(20)
#define NUM_OF_POINTS_TO_SHARE_MEM	(10)

#define TRAJ_LIMIT_NUM              (0)

#define RESET_ERROR_TIMEOUT	        (5000)	//wait until to judge if errors are reset

#define MANUAL_COUNT_PER_STEP		(100)	//the time of TP is 50ms every time

#define DEFAULT_MOVEJ_ACC           (50.0)
#define DEFAULT_ACC			        (7000)

#define IDLE2EXE_DELAY              (50)  //wait from idle to execute(ms)

#define MAX_TIME_IN_PUASE           (20*1000)  //(ms)

#define NON_MOVE_INTERVAL           (10)


/*#define CURVE_MODE_T    (1)*/
/*#define CURVE_MODE_S    (2)*/

typedef enum _MoveStatus
{
    MS_READY = 1,
    MS_WAIT,
    MS_NM_WAIT,
    MS_BUSY
}MoveStatus;

typedef enum _PickStatus
{
    FRESH = 1,
    ONCE,
    USING,
    PICKED,
    NEVER
}PickStatus;

typedef struct _CommandInstruction
{
    PickStatus  pick_status;
    uint32_t	id;
    int         count;
	double		smoothDistance;	
    motion_spec_MOTIONTYPE commandtype;	
	string		command_arguments; //pointer of command
}CommandInstruction;

typedef struct _MoveLParam
{
	double vel_max;
	double acc_max;
	double smooth;
}MoveLParam;

typedef struct _MoveJParam
{
	double vel_max;
	double acc_max;
	double smooth;
}MoveJParam;

typedef struct _MoveCParam
{
	double vel_max;
	double acc_max;
	double smooth;
}MoveCParam;


typedef union _VelocityReq
{
    double jnt_vel[MAX_JOINTS];
    double pose_vel[MAX_JOINTS];
}VelocityReq;

typedef union _PointVal
{
    Joint       jnt_val;
    PoseEuler   pose_val;
}PointVal;

typedef enum _PointType
{
    JOINT_M,
    CART_M
}PointType;

typedef struct _ManuPoint
{
    bool        is_step;
    PointType   type;
    PointVal    point;
 }ManuPoint;

typedef struct _MotionPoint
{
    PointType   type;
    PointVal    point; 
}MotionPoint;

typedef struct _ManualReq
{
    //ManuPoint   pre_target;
    double      vel_max;
    ManuPoint   target;
    double      ref_vmax;
    ManuPoint   ref_target;
}ManualReq;

#define MAX_FIFO_LEN    (1024)
typedef struct _FIFOData
{
    rwmutex rwmux;
    int count;        
    double  data[MAX_FIFO_LEN][MAX_JOINTS];
}FIFOData;


class RobotMotion
{
  public:		    
    ThreadSafeList<CommandInstruction>  non_move_instructions_;
    FIFOData            fifo1_;
    FIFOData            fifo2_;
    int                 dbcount;
    std::atomic<bool>   shutdown_;

	RobotMotion();	
	~RobotMotion();
    
    /**
     * @brief: initial 
     *
     * @return: 0 if success 
     */
    void initial(vector<U64>& err_list);

    /**
     * @brief 
     */
    void destroy();

    /**
     * @brief: state machine 
     *
     * @return: 0 if success 
     */
    U64 checkProgramState();

    U64 stateEStopAction();

    void errorAction(int warning_level);
    /**
     * @brief 
     *
     * @return 
     */
    bool isProgramStateChanged();

    /**
     * @brief 
     *
     * @return 
     */
    IOInterface* getIOInterfacrPtr();
	/**
	 * @brief: get ShareMem object 
	 *
	 * @return: &share_mem_ 
	 */
	ShareMem* getShareMemPtr();
	/**
	 * @brief: get the pointer of ArmGroup 
	 *
	 * @return: arm_group_ 
	 */
	ArmGroup* getArmGroupPtr();
    /**
     * @brief: 
     *
     * @return 
     */
    SafetyInterface* getSafetyInterfacePtr();

    /**
     * @brief: 
     *
     * @return 
     */
    RunningMode getRunningMode();

    /**
     * @brief 
     *
     * @param rm
     */
    void setRunningMode(RunningMode rm);
    /**
     * @brief 
     *
     * @param rm
     */
    void clearRunningMode(RunningMode rm);

    /**
     * @brief 
     *
     * @return 
     */
    int getCurveMode();

    /**
     * @brief 
     *
     * @param c_mode
     */
    void setCurveMode(int c_mode);

    /**
     * @brief 
     *
     * @return 
     */
    ProgramState getNMPrgmState();

    /**
     * @brief 
     *
     * @param prgm_state
     */
    void setNMPrgmState(ProgramState prgm_state);

    /**
     * @brief:get ProgramState 
     *
     * @return 
     */
    ProgramState getProgramState();

    /**
     * @brief: set ProgramState 
     *
     * @param prgm_state: input
     */
    void setProgramState(ProgramState prgm_state);

    /**
     * @brief: getInstructionListSize 
     *
     * @return: 
     */
    int getInstructionListSize();
	/**
	 * @brief: get previous_command_id_ 
	 *
	 * @return: previous_command_id_ 
	 */
	int getPreviousCmdID();
    
    /**
     * @brief: setPreviousCmdID 
     *
     * @param id: input
     */
    void setPreviousCmdID(int id);
	/**
	 * @brief: get current_command_id_ 
	 *
	 * @return: current_command_id_
	 */
	int getCurrentCmdID();

    /**
     * @brief: setCurrentCmdID 
     *
     * @param id: input
     */
    void setCurrentCmdID(int id);
	/**
	 * @brief: get current logic mode
	 *
	 * @return: mode_ 
	 */
	RobotMode getLogicMode();
	/**
	 * @brief: Get current logic state
	 *
	 * @return: state_ 
	 */
	RobotState getLogicState();
	/**
	 * @brief: Get current joints
	 *
	 * @return: joints_
	 */
	Joint getCurJointsValue();
	/**
	 * @brief: get the position
	 *
	 * @return: pose_
	 */
	PoseEuler getCurPosition();

	/**
	 * @brief: get servo state, 0:init, 1:ready, 2:running, 3:error 
	 *
	 * @return: servo_state_ 
	 */
	unsigned int getServoState();

    /**
     * @brief: set servo_state 
     *
     * @param servo_state: input
     */
    void setServoState(unsigned int servo_state);

    /**
     * @brief: jump to the state of ESTOP_S 
     */
    void jumpToEStop();

    /**
     * @brief: update ProgramState 
     *
     * @return: 0 if success 
     */
    U64 updateProgramState();

	/**
	 * @brief update the logic mode
     *
     * @return: true if mode changed
	 */
	 bool updateLogicMode();
     
     /**
      * @brief: update logic state 
      *
      * @return: true if state changed
      */
     bool updateLogicState();
	/**
	 * @brief: update current joints get value from the share memory 
	 *
	 * @return: 0 if success
	 */
	U64 updateJoints();
	/**
	 * @brief: set current joint to library and then get the position
	 *
	 * @return: 0 if success
	 */
	U64 updatePose();
    
    /**
     * @brief: 
     *
     * @return: 0 if no errors
     */
    U64 updateSafetyStatus();    
    /**
     * @brief: setLogicMode  
     *
     * @param mode: input
     */
    void setLogicMode(RobotMode mode);

    /**
     * @brief: setLogicState 
     *
     * @param state:input
     */
    void setLogicState(RobotState state);

    /**
     * @brief: setCurJoints
     *
     * @param joints: input
     */
    void setCurJoints(Joint joints);
    
    /**
     * @brief: setCurPose 
     *
     * @param pose: input
     */
    void setCurPose(PoseEuler pose);
    /**
     * @brief: set running state cmd
     *
     * @param run_cmd: input
     *
     * @return: 0 if success
     */
    U64 setProgramStateCmd(ProgramStateCmd prgm_cmd);
	/**
	 * @brief: set current mode
	 *
	 * @param: mode_cmd mode command
	 *
	 * @return: 0 if set Successfully
	 */
	U64 setLogicModeCmd(RobotModeCmd mode_cmd);
	/**
	 * @brief: set current logic state
	 *
	 * @param state_cmd: input==> state command
	 *
	 * @return: 0 if success 
	 */
	U64 setLogicStateCmd(RobotStateCmd state_cmd);	
    /**
     * @brief: motion resume
     *
     * @return: 0 is success 
     */
    U64 actionResume();
	/**
	 * @brief: motion pause
     *
	 * @return: 0 if success
	 */
	U64 actionPause();	
   
    /**
     * @brief 
     *
     * @param level
     *
     * @return 
     */
    U64 safetyStop(int level);


    /**
     * @brief: servo estop 
     *
     * @return: 0 if success 
     */
    U64 servoEStop();
    /**
     * @brief 
     *
     * @return: 0 if success 
     */
    U64 actionShutdown();

    /**
     * @brief 
     *
     * @return: 0 if success 
     */
    U64 actionCalibrate();

	/**
	 * @brief: add one command in to queue
	 *
	 * @param motion_command: input==>the command to push
	 */
	void addMotionInstruction(CommandInstruction cmd_instruction);	
	/**
	 * @brief :clear instruction list
     * 
	 */
    U64 abortMotion();

    /**
     * @brief: clear instruction list 
     */
	void clearInstructions();
	/**
	 * @brief check start state in auto run mode
     *
     * @return: 0 is success
	 */
	 U64 checkAutoStartState();
    /**
	 * @brief check start state in Manual mode
     *
     * @return: 0 is success
	 */
     U64 checkManualStartState();

     /**
      * @brief: judge if the joints from encoder changed 
      *
      * @return: true if the joints have changed 
      */
     bool isJointsChanged();

     /**
      * @brief: heart beat 
      *
      * @param err_list: output==>error list from motion core1
      *
      * @return: 0 if success 
      */
     int motionHeartBeart(U64 *err_list);

     /**
      * @brief: backupError 
      *
      * @param err: input
      */
     void backupError(U64 err);
    
     /**
      * @brief:clearErrorList 
      */
     void clearErrorList();

     /**
      * @brief 
      *
      * @param err
      *
      * @return 
      */
     bool isUpdatedSameError(U64 err);
     /**
      * @brief:get error state 
      *
      * @return: true if error occurs 
      */
      bool isErrorExist();
 
     /**
     * @brief: clearPathFifo 
     *
     * @return: 0 if success 
     */
    U64 clearPathFifo();

    /**
     * @brief: moveJ in manual mode 
     *
     * @param tech_pose: input
     */
    U64 checkManualJntVel(const motion_spec_TeachPose *tech_pose);
    /**
     * @brief: moveL in manual mode 
     *
     * @param tech_pose: input
     */
    U64 checkManualPoseVel(const motion_spec_TeachPose *tech_pose);   

    /**
     * @brief: set pause in manual mode 
     */
    void zeroManualReqVel();

    /**
	 * @brief check if the joints changed within some values
	 *
	 * @param input==>src_joints
	 * @param input==>dst_joints
	 *
	 * @return 
	 */
	bool compareJoints(Joint src_joints, Joint dst_joints);

    /**
     * @brief: getPoseFromJoint 
     *
     * @param joints: input
     * @param pose: output
     *
     * @return: 0 if success 
     */
    U64 getPoseFromJoint(const Joint &joints, PoseEuler &pose);

    /**
     * @brief: getJointFromPose 
     *
     * @param pose: input
     * @param joints: output
     *
     * @return: 0 if success 
     */
    U64 getJointFromPose(const PoseEuler &pose, Joint &joints, double time_val = 2);

    /**
     * @brief: get user frame 
     *
     *
     * @return:  
     */
    motion_spec_userFrame getUserFrame();
    /**
     * @brief: set user frame 
     *
     * @param user_frame: input==>user frame
     *
     * @return: 0 if success 
     */
    U64 setUserFrame(motion_spec_userFrame *user_frame);

    /**
     * @brief: get tool frame 
     *
     *
     * @return:  
     */
    motion_spec_toolFrame getToolFrame();

    /**
     * @brief: set tool frame 
     *
     * @param tool_frame: input
     *
     * @return: 0 if success 
     */
    U64 setToolFrame(motion_spec_toolFrame *tool_frame);
    /**
     * @brief: getJointConstraint 
     *
     * @return: 0 if success 
     */
     motion_spec_JointConstraint getSoftConstraint();

    /**
     * @brief: setJointConstraint 
     *
     * @param jnt_constraint: input
     *
     * @return: 0 if success 
     */
    U64 setSoftConstraint(motion_spec_JointConstraint *jnt_constraint);

    /**
     * @brief: get hard limit 
     *
     * @return: the hard limit  
     */
    motion_spec_JointConstraint getHardConstraint();
    
    /**
     * @brief: set har limit
     *
     * @param jnt_constraint: input
     *
     * @return 
     */
    U64 setHardConstraint(motion_spec_JointConstraint *jnt_constraint);

    /**
     * @brief: getGlobalVelocity 
     *
     * @return: global_vel_ 
     */
    double getGlobalVelocity();

    /**
     * @brief: setGlobalVelocity 
     *
     * @param velocity: input
     *
     * @return: 0 if success 
     */
    U64 setGlobalVelocity(double factor);

    /**
     * @brief: get DH group 
     *
     * @return: 0 if success 
     */
    motion_spec_DHGroup getDHGroup();

    /**
     * @brief 
     *
     * @param dh_group: intput
     *
     * @return: 0 if success 
     */
    U64 setDHGroup(motion_spec_DHGroup *dh_group);

    /**
     * @brief: set a temporary zero 
     *
     * @return: 0 if success 
     */
    U64 setTempZero();


  private:
	ShareMem			share_mem_;
    SafetyInterface     safety_interface_;
    IOInterface         *io_interface_;
    ParamGroup          *param_group_;
	ArmGroup			*arm_group_;
	Joint			    joints_;				//current joints
	PoseEuler			pose_;					//current pose
    std::atomic<RobotMode>		mode_;					//current mode
    std::atomic<RobotState>		state_;					//current state
	Joint			    prev_joints_;
    //RobotState          prev_state_;
    std::atomic<RobotMode>		prev_mode_;				//previous mode
    std::atomic<int>	previous_command_id_;	//previous command instruction id	
    std::atomic<int>	current_command_id_;	//current instruction id
    std::atomic<ProgramState>   program_state_;
    
    std::atomic<unsigned int>   servo_state_;
    MotionPoint         pre_target_;

    Joint               manu_refer_joints_; //the last target(joints)
    PoseEuler           manu_refer_pose_;   //the last target(pose)
    
    ManualReq           manu_req_;
    
    std::atomic<double>              vel_factor_;        //global velocity for auto run mode
    std::atomic<RunningMode>         run_mode_;

	boost::mutex		mutex_;
    std::atomic<MoveStatus>             move_status_;

    std::atomic<bool>                   servo_ready_wait_;                  //if needs to wait for servo ready
    std::atomic<ProgramState>           nm_prgm_state_;   //non move ProgramState
    ThreadSafeList<CommandInstruction>  instruction_list_;     //store motion instructions
    ThreadSafeList<U64> bak_err_list_;     
    std::atomic<U64>    prev_err_;
    //unsigned int        zero_info_; //record which joint lose zero

    void processNonMove(); 
    MoveStatus getMoveStatus();
    void setMoveStatus(MoveStatus ms);
	/**
	 * @brief: convert unit from params of TP to controller
	 *
	 * @param src_moveL: input==>struct  covert from
	 * @param dst_pose: output==>struct covert to
	 * @param dst_movel_param: output==> struct covert to
	 */
	void unitConvert(const motion_spec_MoveL *src_moveL, PoseEuler& dst_pose, MoveLParam &dst_movel_param);
	/**
	 * @brief convert unit from params of TP to controller
	 *
	 * @param src_moveL: input==>struct  covert from
	 * @param dst_pose: output==>struct covert to
	 * @param dst_movel_param: output==>struct covert to
	 */
	void unitConvert(const motion_spec_MoveJ *src_moveJ, Joint &dst_joints, MoveJParam &dst_moveJ_param);
    /**
	 * @brief convert unit from params of TP to controller
	 *
	 * @param src_moveL: input==>struct  covert from
	 * @param dst_pose: output==>struct covert to
	 * @param dst_movel_param: output==>struct covert to
	 */
    void unitConvert(const motion_spec_MoveC *moveC, PoseEuler& pose1, PoseEuler& pose2, MoveCParam &moveC_param);

    /**
     * @brief: set previous target 
     *
     * @param type: intput
     * @param point: input
     */
    void setPrevTarget(PointType type, PointVal point);

    /**
     * @brief: get start state in joints 
     *
     * @return: the start joints 
     */
    Joint getStartJoint();

    /**
     * @brief: get start state in pose 
     *
     * @return: the start pose 
     */
    PoseEuler getStartPose();

	/**
	 * @brief get max value from a buffer
	 *
	 * @param buffer: input
	 * @param length: input
     * @param id: output
	 *
	 * @return :the max value
	 */
	double getMaxValue(const double *buffer, int length/*, int &id*/);
	/**
	 * @brief: get the ID of max value 
	 *
	 * @param max: input==>the max value
	 * @param buffer: input
	 * @param length: input
	 *
	 * @return: the ID 
	 */
	int getMaxValID(double max, const double *buffer, int length);
	/**
	 * @brief: get the max interval value between two joints
	 *
	 * @param pre_joints: input
	 * @param cur_joints: input
	 *
	 * @return: the max value
	 */
	double getMaxJointInterval(Joint pre_joints, Joint cur_joints);    
	/**
	 * @brief: calculate max velocity between two joints
	 *
	 * @param pre_joints: input
	 * @param cur_joints: input
	 *
	 * @return: max velocity
	 */
//	double calcuManualJointVel(Joint pre_joints, Joint cur_joints);
	/**
	 * @brief: get the max interval value between two pose(x,y,z)
	 *
	 * @param pre_pose: input
	 * @param cur_pose: input
	 *
	 * @return: the max value
	 */
	double getMaxLineInterval(PoseEuler pre_pose, PoseEuler cur_pose);

    /**
     * @brief: get max interval value between two pose(a,b,c) 
     *
     * @param pre_pose: input
     * @param cur_pose: input
     *
     * @return: the max value 
     */
    double getMaxRotateInterval(PoseEuler pre_pose, PoseEuler cur_pose);
	/**
	 * @brief: calculate max velocity between two pose
	 *
	 * @param pre_pose: input
	 * @param cur_pose: input
	 *
	 * @return: max velocity
	 */
	//double calcuManualLineVel(PoseEuler pre_pose, PoseEuler cur_pose);
	/**
	 * @brief: convert unit from params of TP to controller
	 *
	 * @param pose: input&output
	 */
	void uintPoseTP2Ctle(PoseEuler *pose);
	/**
	 * @brief: convert unit from params of TP to controller
	 *
	 * @param pose: input&output
	 */
	void uintPoseCtle2TP(PoseEuler *pose);

    /**
     * @brief 
     *
     * @param point: input
     */
    void setManualPrevTarget(ManuPoint point);

    /**
     * @brief: auto move without next_inst 
     *
     * @param target_inst: input==>target instruction
     *
     * @return: 0 if success
     */
    U64 moveInstructions(CommandInstruction target_inst);

    /**
     * @brief: auto move with next instruction
     *
     * @param target_inst: input==> target instruction
     * @param next_inst: input==> next instruction
     *
     * @return: 0 if success
     */
    U64 moveInstructions(CommandInstruction target_inst, CommandInstruction next_inst);
	/**
	 * @brief: pick one motion instruction from the queue and execute it
	 *
	 * @return: 0 if Success
	 */
	U64 pickInstructions();

    /**
      * @brief: judge if fifo1 and fifo2 is empty 
      *
      * @return: true if empty 
      */
    bool isFifoEmpty();

    /**
     * @brief: plan movement of fifo 
     *
     * @return: 0 if success 
     */
    U64 planFifo();

    /**
     * @brief: process of estop 
     *
     * @return: 0 if success 
     */
    U64 emergencyStop();

    /**
     * @brief: popup instruction from picked_motion_queue_
     */
    void popupInstruction(int id);

    void popupInstEnd();
    /**
     * @brief: resetInstructionList 
     */
    void resetInstructionList();
    /**
     * @brief: pick points from fifo1  
     *
     * @return: 0 if success 
     */
    U64 pickPointsFromPathFifo();

    void clearFIFO1Plot();

    void clearFIFO2Plot();

    void fillInFIFO1(const Pose pose);

    void fillInFIFO2(const double *points);

    void writePosToShm(JointCommand jc_w);

    /**
     * @brief: pick joints and send them to bare metal 
     */
    U64 sendJointsToRemote();    

    void checkInstID(JointCommand joint_command);

    /**
     * @brief: check if manual move is on 
     *
     * @return: true if manual is moving 
     */
    bool hasManualVel();
   
    /**
     *
     * @brief: calculate the target according to src and delta 
     *
     * @param src: input
     * @param delta: input
     * @param dst: output
     */
    void addTargetVal(double *src, double *delta, double *dst);

    /**
     * @brief:  
     *
     * @return 
     */
    ManualReq getManualReq();

    /**
     * @brief: move manual line 
     *
     * @return: 0 if success
     */
    U64 moveManually();
   
    /**
     * @brief: pickMoveInstruction 
     *
     * @param cmd_instruction: output the picked instruction
     *
     * @return: pointer of next instruction, NULL if not exist
     */
    CommandInstruction* pickMoveInstruction();

    /**
     * @brief: pickNextMoveInstruction 
     *
     * @return: pointer of next instruction, NULL if not exist
     */
    CommandInstruction* pickNextMoveInstruction(CommandInstruction *target_inst);
    

    /**
     * @brief: getServoWaitFlag 
     *
     * @return 
     */
    bool getServoWaitFlag();
    
    /**
     * @brief: setServoWaitFlag 
     *
     * @param flag: input
     */
    void setServoWaitFlag(bool flag);

    /**
     * @brief: do after move to the end
     */
    void processEndingMove();
   
    /**
     * @brief: addNonMoveInstruction 
     *
     * @param instruction: input
     *
     * @return: 0 if success 
     */
    U64 addNonMoveInstruction(CommandInstruction instruction);
   
    /**
     * @brief 
     *
     * @return 
     */
    RobotMode getPrevMode();

    /**
     * @brief 
     *
     * @param mode
     */
    void setPrevMode(RobotMode mode);

    /**
     * @brief 
     *
     * @return 
     */
    bool hasMoveCommand();

    /**
     * @brief: check if robot is ready to change to execute 
     *
     * @return: true if ready 
     */
    bool isReadyToMove();

    /**
     * @brief: check if the ProgramState transformed from execute to idle auto 
     *
     * @return: true if has transformed 
     */
    bool hasTransformedToIdle();
    
    /**
     * @brief: clear manu move params 
     */
    void clearManuMove();

    /**
     * @brief: calcuManualJointVel when TP send manual Joint values 
     *
     * @param interval_jnts: input
     *
     * @return: 0 if success 
     */
    double calcuManualJointVel(const double *interval_jnts);
    /**
     * @brief: calcuManualLineVel when TP send manual position values 
     *
     * @param interval_pose: input
     *
     * @return: 0 if success 
     */
    double calcuManualLineVel(const double *interval_pose);

    /**
     * @brief: judge if servo ready  
     *
     * @return: true if ready 
     */
    bool isServoReady();

    /**
     * @brief: get delta value of two rads 
     *
     * @param value1: input
     * @param value2: input
     *
     * @return 
     */
    double get2PIDeltaValue(double value1, double value2);
    
    /**
     * @brief: get line delta between two values
     *
     * @param from: input==>Subtrahend 
     * @param to: input==>minuend
     * @param target: output==>target
     * @param num: number in group
     */
    void getLineDelta(double* from, double* to, double* target, int num);
    /**
     * @brief: get rad delta between two values
     *
     * @param from: input==>Subtrahend 
     * @param to: input==>minuend
     * @param target: output==>target
     * @param num: number in group
     */
    void getRadDelta(double* from, double* to, double* target, int num);
    
    /**
     * @brief: add two value of line 
     *
     * @param src: input==>value1
     * @param delta: input
     * @param dst: output==>the total value
     * @param num: number in group
     */
    void addLineDelta(double *src, double *delta, double *dst, int num);
    /**
     * @brief: add two value of rad 
     *
     * @param src: input==>value1
     * @param delta: input
     * @param dst: output==>the total value
     * @param num: number in group
     */
    void addRadDelta(double *src, double *delta, double *dst, int num);
};


#endif
