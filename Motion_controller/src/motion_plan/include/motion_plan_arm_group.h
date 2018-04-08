/*************************************************************************
	> File Name: motion_plan_arm_group.h
	> Author: 
	> Mail: 
	> Created Time: 2017年12月11日 星期一 09时14分31秒
 ************************************************************************/

#ifndef _MOTION_PLAN_ARM_GROUP_H
#define _MOTION_PLAN_ARM_GROUP_H

#include <string>
#include <vector>
#include <fst_datatype.h>
#include <motion_plan_error_code.h>
#include <motion_plan_motion_command.h>
#include <motion_plan_manual_teach.h>

#define     MOTION_COMMAND_POOL_CAPACITY    10
#define     PATH_FIFO_CAPACITY              16384       // must be setted to 2~N
#define     TRAJECTORY_FIFO_CAPACITY        512         // must be setted to 2^N

namespace fst_controller
{

class ArmGroup
{
  public:
    ArmGroup();
    ~ArmGroup();

    //------------------------------------------------------------------------------
    // Function:    initArmGroup
    // Summary: Initial all the resources in this class
    // In:      None
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------------------------
    ErrorCode initArmGroup();

    //------------------------------------------------------------
    // Function:    getVersion
    // Summary: To get current version of motion_plan.
    // In:      None
    // Out:     None
    // Return:  version string
    //------------------------------------------------------------
    std::string getVersion(void);

    //------------------------------------------------------------
    // Function:    getCycleTime
    // Summary: To get the cycle time used in interpolation.
    // In:      None
    // Out:     None
    // Return:  cycle time
    //------------------------------------------------------------
    double getCycleTime(void);

    //------------------------------------------------------------
    // Function:    getCycleRadian
    // Summary: To get the cycle radian used in path plan.
    // In:      None
    // Out:     None
    // Return:  cycle radian
    //------------------------------------------------------------
    double getCycleRadian(void);

    //------------------------------------------------------------
    // Function:    getCycleDistance
    // Summary: To get the cycle distance used in path plan.
    // In:      None
    // Out:     None
    // Return:  cycle distance
    //------------------------------------------------------------
    double getCycleDistance(void);

    //------------------------------------------------------------
    // Function:    getCartesianAccMax
    // Summary: To get max acceleration in cartesian space.
    // In:      None
    // Out:     None
    // Return:  value of max acceleration in cartesian space
    //------------------------------------------------------------
    double getCartesianAccMax(void);

    /*
    //------------------------------------------------------------
    // Function:    setCartesianAccMax
    // Summary: To set max acceleration in cartesian space.
    // In:      acc -> max acceleration in cartesian space
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setCartesianAccMax(double acc);
    */

    //------------------------------------------------------------
    // Function:    getCartesianVelMax
    // Summary: To get max velocity in cartesian space.
    // In:      None
    // Out:     None
    // Return:  value of max velocity in cartesian space
    //------------------------------------------------------------
    double getCartesianVelMax(void);

    /*
    //------------------------------------------------------------
    // Function:    setCartesianVelMax
    // Summary: To set max velocity in cartesian space.
    // In:      acc -> max velocity in cartesian space
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setCartesianVelMax(double vel);
    */

    //------------------------------------------------------------
    // Function:    getGlobalVelRatio
    // Summary: To get global velocity ratio in cartesian and joint space.
    // In:      None
    // Out:     None
    // Return:  global velocity ratio in cartesian space
    //          range: 0.0 ~ 1.0
    //------------------------------------------------------------
    double getGlobalVelRatio(void);

    //------------------------------------------------------------
    // Function:    setGlobalVelRatio
    // Summary: To set global velocity ratio in cartesian and joint space.
    // In:      ratio -> global velocity ratio in cartesian space
    //                   range: 0.0 ~ 1.0
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setGlobalVelRatio(double ratio);

    //------------------------------------------------------------
    // Function:    getGlobalAccRatio
    // Summary: To get global acceleration ratio in cartesian and joint space.
    // In:      None
    // Out:     None
    // Return:  global acceleration ratio in cartesian space
    //          range: 0.0 ~ 3.0
    //------------------------------------------------------------
    double getGlobalAccRatio(void);

    //------------------------------------------------------------
    // Function:    setGlobalAccRatio
    // Summary: To set global acceleration ratio in cartesian and joint space.
    // In:      ratio -> global acceleration ratio in cartesian space
    //                   range: 0.0 ~ 3.0
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setGlobalAccRatio(double ratio);

    //------------------------------------------------------------
    // Function:    getCartesianAccDefault
    // Summary: To get default acceleration in cartesian space.
    // In:      None
    // Out:     None
    // Return:  cartesian acc, unit: mm/(s*s)
    //------------------------------------------------------------
    double getCartesianAccDefault(void);

    //------------------------------------------------------------
    // Function:    setCartesianAccDefault
    // Summary: To set default acceleration in cartesian space.
    // In:      acc -> cartesian acceleration, unit: mm/(s*s)
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setCartesianAccDefault(double acc);

    //------------------------------------------------------------
    // Function:    getCartesianVelDefault
    // Summary: To get default velocity in cartesian space.
    // In:      None
    // Out:     None
    // Return:  cartesian vel, unit: mm/s
    //------------------------------------------------------------
    double getCartesianVelDefault(void);

    //------------------------------------------------------------
    // Function:    setCartesianVelDefault
    // Summary: To set default velocity in cartesian space.
    // In:      vel -> cartesian acceleration, unit: mm/s
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setCartesianVelDefault(double vel);

/*
    //------------------------------------------------------------
    // Function:    getCartesianAcc
    // Summary: To get acceleration in cartesian space.
    // In:      None
    // Out:     None
    // Return:  cartesian acc, unit: mm/(s*s)
    //------------------------------------------------------------
    double getCartesianAcc(void);

    //------------------------------------------------------------
    // Function:    setCartesianAcc
    // Summary: To set acceleration in cartesian space.
    // In:      acc -> cartesian acceleration, unit: mm/(s*s)
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setCartesianAcc(double acc);


    //------------------------------------------------------------
    // Function:    getJerk
    // Summary: To get jerk ratio.
    // In:      None
    // Out:     None
    // Return:  jerk ratio
    //------------------------------------------------------------
    double getJerk(void);

    //------------------------------------------------------------
    // Function:    getCurveMode
    // Summary: To get velocity curve mode.
    // In:      None
    // Out:     None
    // Return:  curve mode in current:
    //            T_CURVE -> Trapezoidal curve
    //            S_CURVE -> S-Shape curve
    //------------------------------------------------------------
    const CurveMode& getCurveMode(void);
*/

    //------------------------------------------------------------
    // Function:    getSoftConstraint
    // Summary: To get soft joint constraint from algorithm.
    // In:      None
    // Out:     None
    // Return:  soft constraint
    //------------------------------------------------------------
    const JointConstraint& getSoftConstraint(void);

    //------------------------------------------------------------
    // Function:    setSoftConstraint
    // Summary: To set soft joint constraint to algorithm and config file.
    // In:      cons -> soft joint constraint
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setSoftConstraint(const JointConstraint &cons);

    //------------------------------------------------------------
    // Function:    getHardConstraint
    // Summary: To get hard joint constraint from algorithm.
    // In:      None
    // Out:     None
    // Return:  hard constraint
    //------------------------------------------------------------
    const JointConstraint& getHardConstraint(void);

    //------------------------------------------------------------
    // Function:    getDH
    // Summary: To get DH parameter group from algorithm.
    // In:      None
    // Out:     None
    // Return:  DH parameter group
    //------------------------------------------------------------
    const DHGroup& getDH(void);

    //------------------------------------------------------------
    // Function:    getFIFOLength
    // Summary: To get the length of trajectory FIFO.
    // In:      None
    // Out:     None
    // Return:  length of the FIFO
    //------------------------------------------------------------
    size_t getFIFOLength(void);

    //------------------------------------------------------------
    // Function:    getFIFOCapacity
    // Summary: To get the capacity of trajectory FIFO.
    // In:      None
    // Out:     None
    // Return:  capacity of the FIFO
    //------------------------------------------------------------
    size_t getFIFOCapacity(void);

    //------------------------------------------------------------
    // Function:    getPointFromFIFO
    // Summary: To get points from trajectory FIFO.
    // In:      num -> number of joint points want to get
    // Out:     points -> points from trajectory FIFO
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode getPointFromFIFO(size_t num, std::vector<JointOutput> &points);

    //------------------------------------------------------------
    // Function:    getPointFromFIFO
    // Summary: To get points from trajectory FIFO.
    // In:      num -> number of joint points want to get
    // Out:     points -> points from trajectory FIFO
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode getPointFromFIFO(size_t num, JointOutput *points);

    //------------------------------------------------------------
    // Function:    getPointFromFIFO
    // Summary: To get 10 points from trajectory FIFO.
    // In:      None
    // Out:     points -> points from trajectory FIFO
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode getPointFromFIFO(std::vector<JointOutput> &points);

    //------------------------------------------------------------
    // Function:    pickPointToFIFO
    // Summary: To pick points from motion command and put the points 
    //          into trajectory FIFO.
    // In:      num -> number of points want to pick
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    //ErrorCode pickPointToFIFO(size_t num);

    //------------------------------------------------------------
    // Function:    getToolFrame
    // Summary: To get current tool frame in algorithm.
    // In:      None
    // Out:     None
    // Return:  current tool frame
    //------------------------------------------------------------
    const Transformation getToolFrame(void);

    //------------------------------------------------------------
    // Function:    setToolFrame
    // Summary: To set current tool frame.
    // In:      tf  -> new tool frame
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void setToolFrame(const Transformation &tf);

    //------------------------------------------------------------
    // Function:    getUserFrame
    // Summary: To get current user frame in algorithm.
    // In:      None
    // Out:     None
    // Return:  current user frame
    //------------------------------------------------------------
    const Transformation getUserFrame(void);

    //------------------------------------------------------------
    // Function:    setUserFrame
    // Summary: To set current user frame.
    // In:      uf -> new user frame
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void setUserFrame(const Transformation &uf);

    //------------------------------------------------------------
    // Function:    setStartState
    // Summary: To set robot start state.
    // In:      joint -> robot start state
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setStartState(const Joint &joint);

    //------------------------------------------------------------
    // Function:    getJointFromPose
    // Summary: To compute IK with a given pose in cartesian space,
    //          without reference joint and accessibility check.
    // In:      poes    -> the pose in cartesian space needed to compute IK
    // Out:     joint   -> IK result
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode getJointFromPose(const PoseEuler &pose, Joint &joint);

    //------------------------------------------------------------
    // Function:    getPoseFromJoint
    // Summary: To compute FK with a given point in joint space.
    // In:      joint   -> the point in joint space needed to compute FK.
    // Out:     pose    -> FK result
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode getPoseFromJoint(const Joint &joint, PoseEuler &pose);

    //------------------------------------------------------------
    // Function:    getPoseFromJointInWorld
    // Summary: To get the pose of flange and tcp from a given point
    //          in joint space in world coordinate.
    // In:      joint   -> the point in joint space needed to compute FK.
    // Out:     flange  -> flange pose in world coordinate
    //          tcp     -> tcp pose in world coordinate
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode getPoseFromJointInWorld(const Joint &joint, PoseEuler &flange, PoseEuler &tcp);

    //------------------------------------------------------------
    // Function:    getRemainingTime
    // Summary: To get the remaining time of current motion.
    // In:      None
    // Out:     None
    // Return:  remaining time
    //------------------------------------------------------------
    MotionTime getRemainingTime(void);

    //------------------------------------------------------------
    // Function:    suspendMotion
    // Summary: To replan a slow-down trajectory upon trajectory FIFO
    //          and stop the robot motion. Used when pause event or IK
    //          failure raised etc.
    // In:      None
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode suspendMotion(void);

    //------------------------------------------------------------
    // Function:    declareESTOP
    // Summary: Declare an ESTOP event to arm_group, this function
    //          should be called after the robot is stopped.
    // In:      joint   -> stopped robot is standing on this joint
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode declareESTOP(const Joint &joint);

    //------------------------------------------------------------
    // Function:    resumeMotion
    // Summary: To replan a start-up trajectory and resume robot
    //          motion from suspend or ESTOP state.
    // In:      None
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode resumeMotion(void);

    //------------------------------------------------------------
    // Function:    resetArmGroup
    // Summary: Return wether joint is in soft constraint or not.
    // In:      joint   ->
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode resetArmGroup(const Joint &joint);

    //------------------------------------------------------------
    // Function:    clearArmGroup
    // Summary: To reset resources in arm group, FIFO and state etc.
    // In:      None
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode clearArmGroup(void);

    //------------------------------------------------------------
    // Function:    isJointInConstraint
    // Summary: Retrun true if given joint fall into the soft constraint
    // In:      joint -> joint to be compared with soft constraint
    // Out:     None
    // Return:  true  -> joint in soft constraint
    //          false -> joint not in soft constraint
    //------------------------------------------------------------
    bool isJointInSoftConstraint(const Joint &joint);

    //------------------------------------------------------------
    // Function:    autoMove
    // Summary: Plan an auto move command (moveJ/moveL/moveC) to reach
    //          the target without smooth. If FIFO is empty at the
    //          moment, then fill the FIFO with points from this command.
    // In:      target -> motion target
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode autoMove(const MotionTarget &target, int id);

    //------------------------------------------------------------
    // Function:    pauseMove
    // Summary: Plan a pause trajectory to all axes to stop on the path
    // In:      pick_segment -> the path point index of starting pause action
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode pauseMove(size_t pick_segment);

    //------------------------------------------------------------
    // Function:    continueMove
    // Summary: Plan a trajectory to let all axes continue moving,
    //          first MOVJ to the paused point, then finish the remaining path.
    // In:      None
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode continueMove(void);
    
    //------------------------------------------------------------
    // Function:    manualMove
    // Summary: Plan a manual move trajectory (Joint/Line) with given
    //          direction. If FIFO is empty at the moment, fill it.
    // In:      directions -> manual direction
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode manualMove(const std::vector<ManualDirection> &directions);

    //------------------------------------------------------------
    // Function:    manualMove
    // Summary: Plan a manual move trajectory (Joint/Line) with given
    //          point. If FIFO is empty at the moment, then fill it.
    // In:      joint -> manual target in joint space
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode manualMove(const Joint &joint);

    //------------------------------------------------------------
    // Function:    manualMove
    // Summary: Plan a manual move trajectory (Joint/Line) with given
    //          point. If FIFO is empty at the moment, then fill it.
    // In:      pose  -> manual target in cartesian space
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode manualMove(const PoseEuler &pose);

    //------------------------------------------------------------
    // Function:    setManualFrame
    // Summary: To set manual frame mode.
    // In:      frame   -> manual frame mode, JOINT/WORLD/USER/TOOL
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void setManualFrame(ManualFrame frame);

    //------------------------------------------------------------
    // Function:    setManualMode
    // Summary: To set manual motion mode.
    // In:      motion  -> manual motion mode, STEP/CONTINUOUS/POINT
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void setManualMode(ManualMode mode);

    //------------------------------------------------------------
    // Function:    setManualJointStep
    // Summary: To set step length in manual joint step working mode.
    // In:      step    -> step length, unit: rad
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setManualJointStep(double step);

    //------------------------------------------------------------
    // Function:    setManualCartesianPositionStep
    // Summary: To set step length in manual cartesian step working mode.
    // In:      step    -> step length, unit: mm
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setManualCartesianPositionStep(double step);

    //------------------------------------------------------------
    // Function:    setManualCartesianOrientationStep
    // Summary: To set step length in manual cartesian step working mode.
    // In:      step    -> step length, unit: mm
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setManualCartesianOrientationStep(double step);

    //------------------------------------------------------------
    // Function:    getManualMaxSpeedRatio
    // Summary: To get manual max speed ratio to max-auto-speed.
    // In:      None
    // Out:     None
    // Return:  manual max speed ratio, range: 0.0 ~ 1.0
    //------------------------------------------------------------
    double getManualMaxSpeedRatio(void);

    //------------------------------------------------------------
    // Function:    getManualSpeedRatio
    // Summary: To get manual speed ratio to max-manual-speed.
    // In:      None
    // Out:     None
    // Return:  manual speed ratio, range: 0.0 ~ 1.0
    //------------------------------------------------------------
    double getManualSpeedRatio(void);

    //------------------------------------------------------------
    // Function:    setManualSpeedRatio
    // Summary: To set manual speed ratio to max-manual-speed.
    // In:      ratio   -> manual speed to max-manual-speed
    //                      range: 0.0 ~ 1.0
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setManualSpeedRatio(double ratio);

    //------------------------------------------------------------
    // Function:    getManualMaxAccRatio
    // Summary: To get manual max acc ratio to max-auto-acc.
    // In:      None
    // Out:     None
    // Return:  manual max acc ratio, range: 0.0 ~ 1.0
    //------------------------------------------------------------
    double getManualMaxAccRatio(void);

    //------------------------------------------------------------
    // Function:    getManualAccRatio
    // Summary: To get manual acc ratio to max-manual-acc.
    // In:      None
    // Out:     None
    // Return:  manual acc ratio, range: 0.0 ~ 1.0
    //------------------------------------------------------------
    double getManualAccRatio(void);

    //------------------------------------------------------------
    // Function:    setManualAccRatio
    // Summary: To set manual acc ratio to max-manual-acc.
    // In:      ratio   -> manual acc to max-manual-acc
    //                      range: 0.0 ~ 1.0
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setManualAccRatio(double ratio);
    
private:
    MotionCommand* getFreeMotionCommand(void);
    MotionCommand* releaseMotionCommand(MotionCommand *cmd);

    //------------------------------------------------------------
    // Function:    autoJoint
    // Summary: Plan a path in joint space to reach the target
    //          without smooth.
    // In:      target -> motion target
    //          id     -> motion id
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode autoJoint(const MotionTarget &target, int id);

    //------------------------------------------------------------
    // Function:    autoLine
    // Summary: Plan a line path to reach the target given by auto move command 
    //          without smooth.
    // In:      target -> motion target
    //          id     -> motion id
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode autoLine(const MotionTarget &target, int id);

    ErrorCode fillTrajectoryFIFO(size_t num);

    ErrorCode convertPath2Trajectory(ControlPoint &cp);

    ErrorCode planTraj(void);

    ErrorCode planJointTraj(void);

    bool isJointStop(size_t pick);
  
    void moveFIFO(size_t start_index, int size, int offset);        

    ErrorCode pickFromManual(size_t num, std::vector<JointOutput> &points);
    ErrorCode pickFromAuto(size_t num, std::vector<JointOutput> &points);

    ErrorCode pickManualJoint(size_t num, std::vector<JointOutput> &points);
    ErrorCode pickManualCartesian(size_t num, std::vector<JointOutput> &points);


    MotionCommand   motion_command_pool_[MOTION_COMMAND_POOL_CAPACITY];
    MotionCommand  *free_command_list_ptr_;
    MotionCommand  *used_command_list_ptr_;

    ControlPoint    prev_traj_point_;
    ControlPoint    traj_fifo_[TRAJECTORY_FIFO_CAPACITY];
    size_t          traj_head_;
    size_t          traj_tail_;

    PathPoint       path_fifo_[PATH_FIFO_CAPACITY];
    size_t          path_head_;
    size_t          path_tail_;

    ControlPoint    t_path_[PATH_FIFO_CAPACITY];
    size_t          t_head_;
    size_t          t_tail_;
    size_t          t_real_start_;
    size_t          t_real_end_;

    MotionTime      pick_time_;
    size_t          pick_segment_;
    bool            auto_running_;

    MotionTime      manual_pick_time_;
    bool            manual_running_;
    ManualTeach     manual_;

};




}

#endif
