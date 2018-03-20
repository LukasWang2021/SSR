/**
 * @file robot.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-08-04
 */

#ifndef RCS_ROBOT_H_
#define RCS_ROBOT_H_

#include <atomic>
#include "base.h"
#include "motion_plan_arm_group.h"
#include "fst_datatype.h"
#include "motionSL.pb.h"
#include "common.h"

using namespace fst_controller;

class Robot
{
  public:
    Robot(ArmGroup *arm_group);
    ~Robot();

    PoseEuler *getTCPPosePtr();
    PoseEuler *getFlangePosePtr(const Joint &joints);
    void updatePose(const Joint &joints);

    /**
     * @brief: getPoseFromJoint 
     *
     * @param joints: input
     * @param pose: output
     *
     * @return: true if success 
     */
    bool getPoseFromJoint(const Joint &joints, PoseEuler &pose);

    /**
     * @brief: getJointFromPose 
     *
     * @param pose: input
     * @param joints: output
     *
     * @return: true if success 
     */
    bool getJointFromPose(const PoseEuler &pose, Joint &joints, double time_val = 2);

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
     * @return: true if success 
     */
    bool setUserFrame(motion_spec_userFrame *user_frame);

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
     * @return: true if success 
     */
    bool setToolFrame(motion_spec_toolFrame *tool_frame);
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
     * @return: true if success 
     */
    bool setSoftConstraint(motion_spec_JointConstraint *jnt_constraint);

    /**
     * @brief: get hard limit 
     *
     * @return: the hard limit  
     */
    motion_spec_JointConstraint getHardConstraint();

    /**
     * @brief: get DH group 
     *
     * @return: 0 if success 
     */
    motion_spec_DHGroup getDHGroup();

  private:
    ArmGroup    *arm_group_;
    PoseEuler   tcp_pose_;
    PoseEuler   flange_pose_;
};


#endif
