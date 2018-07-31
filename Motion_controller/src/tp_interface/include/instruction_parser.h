/**
 * @file instruction_parser.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-10-13
 */
#ifndef INSTRUCTION_PARSER_H_
#define INSTRUCTION_PARSER_H_

#include <list>
#include "common.h"
#include "motion_plan_arm_group.h"
#include "fst_datatype.h"
#include "instruction.h"
#include "motionSL.pb.h"
#include "base.h"


using namespace fst_controller;

typedef union _PointVal
{
    Joint       jnt_val;
    PoseEuler   pose_val;
}PointVal;

typedef struct _MotionPoint
{
    PointType   type;
    PointVal    point; 
}MotionPoint;

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



class InstructionParser
{
  public:
    InstructionParser(ArmGroup *arm_group);
    ~InstructionParser();
    
    int getLineID();
#if 0
    void updateID(int id);
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
#endif
    void updateTrajRemainCount();

    void pickPoints(JointPoint *jnt);

    /**
     * @brief: getInstructionListSize 
     *
     * @return: 
     */
    int getInstructionListSize();


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
    void setGlobalVelocity(double factor);
    /**
	 * @brief: pick one motion instruction from the queue and execute it
	 *
	 * @return: 0 if Success
	 */
    #if 0
	U64 pickInstructions();


    void setPickWaitFlag(bool flag);

    void addMotionInstruction(CommandInstruction *cmd_instruction);
    void runNMInstructions();

    void rmMoveInstruction(); 
    
    /**
     * @brief: resetInstructionList 
     */
    void resetInstructionList();

    bool isInstructionExist();
#endif
    void motionClear();

    void pause();
    bool resume();
    bool step();
    bool jump();
    bool backwords();
    
  private:    
    ArmGroup		    *arm_group_;
    std::atomic_int     line_id_;
    //std::atomic_int     cur_id_;
    MotionTarget        mot_target_;
    //CommandInstruction  target_inst_;
    //CommandInstruction  next_inst_;
    //std::atomic_bool    pick_wait_flag_;
    //MotionPoint         pre_target_;
    //MoveCommand1        *pre_move_cmd_;
    //int                 smooth_mode_;   //vel or pose

    //std::list<CommandInstruction>::iterator first_move_it_;
   // CommandInstruction cmd_inst_;
    //std::list<CommandInstruction>           instruction_list_; //store motion instructions

    /**
     * @brief: pickNextMoveInstruction 
     *
     * @return: pointer of next instruction, NULL if not exist
     */
  //  CommandInstruction* pickNextMoveInstruction();


   // void* setPulseIO(void* params);

   // void* waitTimeout(void* params);

    U64 moveTarget(); 
    //U64 moveInstruction(CommandInstruction &target_inst);
    /**
     * @brief: auto move without next_inst 
     *
     * @param target_inst: input==>target instruction
     *
     * @return: 0 if success
     */
//    U64 moveInstructions(CommandInstruction target_inst);

    /**
     * @brief: auto move with next instruction
     *
     * @param target_inst: input==> target instruction
     * @param next_inst: input==> next instruction
     *
     * @return: 0 if success
     */
//    U64 moveInstructions(CommandInstruction target_inst, CommandInstruction next_inst);

    /**
	 * @brief: convert unit from params of TP to controller
	 *
	 * @param src_moveL: input==>struct  covert from
	 * @param dst_pose: output==>struct covert to
	 * @param dst_movel_param: output==> struct covert to
	 */
#if 0
	void unitConvert(const motion_spec_MoveL *src_moveL, MotionTarget &target);
	/**
	 * @brief convert unit from params of TP to controller
	 *
	 * @param src_moveL: input==>struct  covert from
	 * @param dst_pose: output==>struct covert to
	 * @param dst_movel_param: output==>struct covert to
	 */
	void unitConvert(const motion_spec_MoveJ *src_moveJ, MotionTarget &target);
    /**
	 * @brief convert unit from params of TP to controller
	 *
	 * @param src_moveL: input==>struct  covert from
	 * @param dst_pose: output==>struct covert to
	 * @param dst_movel_param: output==>struct covert to
	 */
    void unitConvert(const motion_spec_MoveC *moveC, MotionTarget &target);
#endif
    
};

#endif //#ifndef INSTRUCTION_PARSER_H_
