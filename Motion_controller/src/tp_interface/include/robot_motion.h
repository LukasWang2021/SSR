/**
 * @file robot_motion_.h
 * @brief 
 * @author Wang Wei
 * @version 1.0.0
 * @date 2016-08-21
 */
#ifndef TP_INTERFACE_ROBOT_MOTION_H_
#define	TP_INTERFACE_ROBOT_MOTION_H_

#include "motion_controller/fst_datatype.h"
#include "motion_controller/arm_group.h"
#include "parameter_manager/parameter_manager_param_group.h"
#include "robot.h"
#include "motionSL.pb.h"
#include "threadsafe_queue.h"
//#include "threadsafe_vector.h"
#include "proto_define.h"
#include "share_mem.h"
#include "fst_error.h"
#include "safety_interface.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <map>

using namespace fst_controller;
using namespace fst_parameter;

#define MAX_CNT_VAL		(100)

#define MAX_PLANNED_POINTS_NUM		(20)
#define NUM_OF_POINTS_TO_SHARE_MEM	(10)

#define TRAJ_LIMIT_NUM              (20)

#define MANUAL_INSTRUCTION_DELAY	(150)	//in manual mode, after the first manual instruction, the delay

#define MANUAL_COUNT_PER_STEP		(60)	//the time of TP is 50ms every time

#define MANUAL_DEFAULT_ACC			(10000)

typedef struct _CommandInstruction
{
	double		smoothDistance;
	uint32_t	id;	
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


typedef struct _TargetPosition
{
	JointValues	joint_values;
	PoseEuler	pose;
}TargetPosition;


class RobotMotion
{
  public:		
	RobotMotion();	
	~RobotMotion();

    U64 initial();
    void destroy();

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

    SafetyInterface* getSafetyInterfacePtr();
	/**
	 * @brief: get previous_command_id_ 
	 *
	 * @return: previous_command_id_ 
	 */
	int getPreviousCmdID();
    void setPreviousCmdID(int id);
	/**
	 * @brief: get current_command_id_ 
	 *
	 * @return: current_command_id_
	 */
	int getCurrentCmdID();
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
	JointValues getCurJointsValue();
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
	 * @brief update the logic mode
     *
	 * @return: 0 if success
	 */
	 U64 updateLogicMode();
     
     /**
      * @brief: update logic state 
      *
      * @return: 0 if success 
      */
     U64 updateLogicState();
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
    void setCurJoints(JointValues joints);
    
    /**
     * @brief: setCurPose 
     *
     * @param pose: input
     */
    void setCurPose(PoseEuler pose);
	/**
	 * @brief: set current mode
	 *
	 * @param: mode_cmd mode command
	 *
	 * @return: true if set Successfully
	 */
	U64 setLogicModeCmd(RobotModeCmd mode_cmd);
	/**
	 * @brief: set current logic state
	 *
	 * @param state_cmd: input==> state command
	 *
	 * @return 
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
	 * @brief: add one manual instruction to the queue
	 *
	 * @param cmd_instruction: input==>the instruction to add
	 * @param arguments: input==> the arguments of instruction
	 */
	void addManualQueue(motion_spec_MOTIONTYPE command_type, string arguments);
	/**
	 * @brief: add one command in to queue
	 *
	 * @param motion_command: input==>the command to push
	 */
	void addMotionQueue(CommandInstruction cmd_instruction);	
	/**
	 * @brief: pick instruction from instruction queue and execute the instruction
     *
     * @return: 0 if success
	 */
	U64 queueProcess();
    /**
	 * @brief :reset motion queue
     *
     * @return: 0 is success 
	 */
    U64 resetMotionQueue();
	/**
	 * @brief :clear motion queue
     *
     * @return: 0 is success 
	 */
	U64 clearMotionQueue();
    /**
     * @brief: clear all manual queue 
     */
    void clearManualQueue();
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
      * @brief: backupErrorList 
      *
      * @param cmd_instruction: input
      */
     void backupErrorList(CommandInstruction cmd_instruction);
    
     /**
      * @brief:clearErrorList 
      */
     void clearErrorList();

     /**
      * @brief:get error state 
      *
      * @return: true if error occurs 
      */
     bool getErrorState();

     /**
      * @brief:setErrorState 
      *
      * @param flag: input
      */
     void setErrorState(bool flag);   
     /**
     * @brief: clearPathFifo 
     *
     * @return: 0 if success 
     */
    U64 clearPathFifo();

  private:
	ShareMem			share_mem_;
    SafetyInterface     safety_interface_;
    ParamGroup          *param_group_;
	ArmGroup			*arm_group_;
	JointValues			joints_;				//current joints
	PoseEuler			pose_;					//current pose
	RobotMode			mode_;					//current mode
	RobotState			state_;					//current state
	JointValues			prev_joints_;
    RobotState          prev_state_;
	RobotMode			prev_mode_;				//previous mode
	int					previous_command_id_;	//previous command instruction id	
	int					current_command_id_;	//current instruction id
//	map<int, PoseEuler> inst_id_map_;			//a map to store the id
	//vector<JointPoint>	joint_traj_;			//store the trajectory joints
//	TargetPosition		previous_target_;		//record the previous target
    map<int, bool>      id_servowait_map_;
	CommandInstruction	cur_instruction_;		//current instruction
	CommandInstruction	next_move_instruction_; //next instruction
	ManualState			manual_state_;          //manual state used in manual mode 
    AutoState           auto_state_;            //auto state used in auto mode
	int					manual_inst_delay_cnt_;
	unsigned int		servo_state_;

	boost::mutex		mutex_;
	boost::mutex	    g_mutex_;

    bool    servo_ready_wait_;                  //if needs to wait for servo ready
	int		next_move_id_;						//record next move id	

	ThreadsafeQueue<CommandInstruction> motion_queue_;		//store motion instructions
	ThreadsafeQueue<CommandInstruction> non_move_queue_;	//store DIO or other instructions
    ThreadsafeQueue<CommandInstruction> picked_motion_queue_;       //store instructions already moved
	ThreadsafeQueue<CommandInstruction> manual_instruction_queue_;	//stroe manual instructions	
    ThreadsafeQueue<CommandInstruction>   bak_err_queue_; 

    bool err_flag_;
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
	void unitConvert(const motion_spec_MoveJ *src_moveJ, JointValues &dst_joints, MoveJParam &dst_moveJ_param);
	/**
	 * @brief get max value from a buffer
	 *
	 * @param buffer: input
	 * @param length: input
	 *
	 * @return :the max value
	 */
	double getMaxValue(const double *buffer, int length);
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
	double getMaxJointInterval(JointValues pre_joints, JointValues cur_joints);    
	/**
	 * @brief: calculate max velocity between two joints
	 *
	 * @param pre_joints: input
	 * @param cur_joints: input
	 *
	 * @return: max velocity
	 */
	double calcuManualJointVel(JointValues pre_joints, JointValues cur_joints);
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
	double calcuManualLineVel(PoseEuler pre_pose, PoseEuler cur_pose);
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
	 * @brief: motion process in auto mode
     *
     * @return: 0 if success
	 */
	U64 autoMotion();

    /**
     * @brief: motion process in manual mode 
     *
     * @return: 0 if success 
     */
    U64 manualMotion();
	/**
	 * @brief: move a line used for manual mode
	 *
	 * @param cur_pose: input==>current pose
	 * @param next_pose: input==>next pose
     *
     * @return: 0 is success
	 */
	U64 moveLine(const PoseEuler* cur_pose, const PoseEuler* next_pose = NULL);
	/**
	 * @brief: move joints in manual mode
	 *
	 * @param cur_joints: input
	 * @param next_joints: input
     *
     * @return: 0 is success
	 */
	U64 moveJoints(const JointValues *cur_joints, const JointValues *next_joints = NULL);
	/**
	 * @brief: pick one manual move instruction
	 *
	 * @return: 0 if success
	 */
	bool pickManualInstruction();
	/**
	 * @brief: pick one motion instruction from the queue and execute it
	 *
	 * @return: 0 if Success
	 */
	bool pickMotionInstruction();
	/**
	 * @brief: find next movable command joint or cart
	 *
	 * @param next_motion_command: output==>the result of the found
	 *
	 * @return: true if found one
	 */
	bool findNextMoveCommand(CommandInstruction &next_cmd_instruction);	
	/**
	 * @brief check if the joints changed within some values
	 *
	 * @param input==>src_joints
	 * @param input==>dst_joints
	 *
	 * @return 
	 */
	bool compareJoints(JointValues src_joints, JointValues dst_joints);
    /**
      * @brief: judge if fifo1 and fifo2 is empty 
      *
      * @return: true if empty 
      */
    bool isFifoEmpty();

    /**
     * @brief: put picked_motion_queue_ to motion_queue_ 
     */
    void resetQueue();    
    /**
     * @brief: clear all auto queue 
     */
    void clearAutoQueue();
    /**
     * @brief: reset the instruction id to -1 
     */
    void resetInstructionID();

    /**
     * @brief: set ManualState 
     *
     * @param state: input
     */
    void setManualState(ManualState state);

    /**
     * @brief: get ManualState 
     *
     * @return: ManualState 
     */
    ManualState getManualState();
    
    /**
     * @brief: setAutoState 
     *
     * @param state: input
     */
    void setAutoState(AutoState state);
    
    /**
     * @brief: getAutoState 
     *
     * @return: AutoState 
     */
    AutoState getAutoState();

    /**
     * @brief: process of estop 
     */
    void emergencyStop();

    /**
     * @brief: popup instruction from picked_motion_queue_
     */
    void popupInstruction();

    /**
     * @brief: pick joints and send them to bare metal 
     */
    void sendJointsToRemote();
    
    void insertIDServoWait();

    void forceChangeMode(RobotMode mode);
};


#endif
