/**
 * @file share_mem_.h
 * @brief: use new API 
 * @author WangWei
 * @version 1.1.0
 * @date 2016-12-20
 */
#ifndef SHARE_MEM_H_
#define SHARE_MEM_H_


#include "comm_interface/core_interface.h"
#include "common.h"
#include "error_code.h"
#include "ros_basic.h"
#include "interpreter_common.h"
#include <atomic>

#define READ_COUNT_LIMIT    (50)
#define WRITE_COUNT_LIMIT   (50)

typedef struct _Shm_Joints_Cmd
{
	JointCommand	    joint_cmd;
    std::atomic_bool	empty;
}ShmjointsCmd;



typedef struct _InterpreterStatus
{
    int line_id;
    int prgm_state;
}InterpreterStatus;


class ShareMem 
{
  public:	
	
	ShareMem(RosBasic *ros_basic);
	~ShareMem();


    static ShareMem* instance();
    /**
     * @brief: initialization 
     *
     * @return: 0 if success 
     */
    void initial();

    bool getLatestJoint(Joint &servo_joint);



    void setEmptyFlag(bool flag);

	/**
	 * @brief: set current share memory JointCommand
	 *
	 */
	void setCurrentJointCmd(JointCommand joint_cmd);
	/**
	 * @brief: get FeedbackJointState from share memory
	 *
	 * @param fbjs: output==> the FeedbackJointState
	 *
	 * @return: 0 if successfullly get the joint state 
	 */
	U64 getFeedbackJoint(Joint &servo_joint);
	/**
	 * @brief: set JointCommand to share memory
	 *
	 * @return: 0 if successfullly set JointCommand 
	 */
	U64 setJointPositions();
	/**
	 * @brief: judge if the JointCommand has successfullly written to Share memory 
	 *
	 * @return: true if success 
	 */
    int getServoState();

    /**
     * @brief: get Instruction from interpreter 
     *
     * @param inst: input==> the Instruction from interpreter
     *
     * @return: true if success 
     */
    bool getInstruction(Instruction &inst);

    /**
     * @brief: not used 
     */
    void setInstEmpty();

    /**
     * @brief: control command to interpreter 
     *
     * @param ctrl: control command
     *
     * @return: true if success 
     */
    bool intprtControl(InterpreterControl ctrl);

    /**
     * @brief: permit interrpreter sending Instruction 
     */
    void sendingPermitted();

    /**
     * @brief: set send flag 
     *
     * @param flag: send flag 
     */
    void setIntprtSendFlag(bool flag);    
    
    /**
     * @brief: set data flag (Reg or IO)
     *
     * @param flag: send flag 
     */
    void setIntprtDataFlag(bool flag); 
	
    /**
     * @brief: get data flag
     *
     * @return: data OK flag 
     */
    bool getIntprtDataFlag();
	
    /**
     * @brief: set user operation mode 
     *
     * @param mode
     */
    void setUserOpMode(UserOpMode mode);
   // void setSysCtrlMode(SysCtrlMode mode);
    
    /**
     * @brief: get current line from interpreter 
     *
     * @return: current line id 
     */
    int getCurLine();

    /**
     * @brief: get interpreter state 
     *
     * @return: interpreter state 
     */
    InterpreterState getIntprtState();

    /**
     * @brief: check if servo is done 
     *
     * @return: true is success 
     */
    bool isServoDone();

    /**
     * @brief: check if joint is updated 
     *
     * @return: true if updated 
     */
    bool isJointUpdated();
     
    /**
     * @brief: get Info of Reg and IO 
     *
     * @param inst: 
     *
     * @return: Info of Reg and IO
     */
    bool getRegInfo(RegMap * info);
	
    /**
     * @brief: get Info of Reg and IO 
     *
     * @param inst: 
     *
     * @return: Info of Reg and IO
     */
    bool getDIOInfo(char * info);
	
  private:	
	fst_core_interface::CoreInterface   core_interface_;
	ShmjointsCmd        shm_jnt_cmd_;		//current JointCommand    
    std::atomic_int     servo_state_;       //current servo state
    std::atomic_int     pre_servo_state_;   //previous servo state
    std::atomic_bool    jnt_updated_;       //joint updated flag
    RosBasic            *ros_basic_;        //
    static ShareMem     *shm_instance_;     //instance own


    /**
     * @brief: check if the intervals between two joints are too big
     *
     * @param src_joints:
     * @param dst_joints:
     *
     * @return: true if the interval is too big
     */
    bool isOutMax(Joint *src_joints, Joint *dst_joints);

    /**
     * @brief: check if the joints has changed 
     *
     * @param src_joints
     * @param dst_joints
     *
     * @return: true if joints changed 
     */
    bool isJointChanged(Joint *src_joints, Joint *dst_joints);

};

#endif //#ifndef SHARE_MEM_H_
