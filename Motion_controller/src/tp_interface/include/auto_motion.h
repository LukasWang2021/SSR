/**
 * @file auto_motion.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2018-01-18
 */

#ifndef AUTO_MOTION_H_
#define AUTO_MOTION_H_

#include "motion_plan_arm_group.h"
#include "fst_datatype.h"
#include "interpreter_common.h"
#include "base.h"
#include <atomic>

class AutoMotion
{
  public:
    AutoMotion(ArmGroup *arm_group);
    ~AutoMotion();

    /**
     * @brief: get current ProgramState 
     *
     * @return: current ProgramState 
     */
    //ProgramState getPrgmState();

    /**
     * @brief: set ProgramState 
     *
     * @param state: ProgramState to set
     */
   // void setPrgmState(ProgramState state);

    /**
     * @brief: get done flag of current Instruction 
     *
     * @return: true if current Instruction is done 
     */
    bool getDoneFlag();
    /**
     * @brief: set done flag  
     *
     * @param flag: flag to set
     */
    void setDoneFlag(bool flag);

    /**
     * @brief: clear motion fifo and status 
     */
    void motionClear();
    /**
     * @brief: get current smooth data of current instruction
     *
     * @return: smooth data 
     */
    int getSmooth();
   /**
     * @brief: pause motion 
     */
    void pause();
    /**
     * @brief: resume motion 
     *
     * @return: true if success 
     */
    bool resume();

    /**
     * @brief: step motion 
     *
     * @return: true if success 
     */
    bool step();

    /**
     * @brief: jump to line 
     *
     * @param id: line to jump
     *
     * @return: tru if success 
     */
    bool jump(int id);

    /**
     * @brief: move backword 
     *
     * @return: true if success 
     */
    bool backwords();

    /**
     * @brief: abort motion 
     *
     * @return: true if success 
     */
    bool abort();

    /**
     * @brief:  
     *
     * @param target
     */
    void moveTarget(MotionTarget target);
  private:
    ArmGroup		    *arm_group_;
    MotionTarget        mot_target_;    //target of this instruction
    bool                is_done_;       //if this Instruction is done

   // std::atomic<ProgramState>   prgm_state_;    //program state
};

#endif
