/**
 * @file manual_motion.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-11-02
 */

#ifndef MANUAL_MOTION_H_
#define MANUAL_MOTION_H_

#include "motionSL.pb.h"
#include "motion_plan_arm_group.h"
#include "robot.h"
#include "interpreter_common.h"

using namespace fst_controller;

#define DEFAULT_MOVEJ_ACC   (50.0)
#define MANUAL_CMD_TIMEOUT  (200)   //ms

class ManualMotion
{
  public:
    ManualMotion(Robot *robot, ArmGroup *arm_group);
    ~ManualMotion();

   // ProgramState getManuState();
    bool isReady();
    double getVelRatio();
    motion_spec_ManualFrame& getManuFrame();
    motion_spec_ManualType& getManuType();
    void setManuCommand(motion_spec_ManualCommand command); //need to be set in idle
    void setTeachTarget(motion_spec_TeachTarget target);
    void pause();
    bool stepCounter();

   // void storeOrginPrgmState();
    //ProgramState getOriginPrgmState();

  private:
    Robot       *robot_;
    ArmGroup    *arm_group_;
    double      vel_ratio_;
    bool        ready_;
   // ProgramState            manu_state_;
    //ProgramState            org_prgm_state_;
    motion_spec_ManualFrame manu_frame_;
    motion_spec_ManualType  manu_type_;
    int         step_counter_;
};

#endif //#ifndef MANUAL_MOTION_H_
