/**********************************************************************
  Copyright:   Foresight-Robotics
  File:        lib_controller.h
  Author:      Feng Yun
  Data:        Aug.1  2016
  Modify:      Aug.23 2016
  Description: ArmGroup--Class declaration.
**********************************************************************/


#ifndef FST_CONTROLLER_H
#define FST_CONTROLLER_H

#include <vector>
#include <map>
#include <pthread.h>
#include <motion_controller/fst_datatype.h>
#include <trajplan/TrajPlan.h>
#include <log_manager/log_manager_logger.h>
#include <motion_controller/motion_controller_error_code.h>
#include <motion_controller/motion_controller_offset_calibrator.h>
#include <motion_controller/motion_controller_planning_interface.h>

namespace fst_controller {
// Brief class for controller. This class include many default settings and functions to make life easier.
class ArmGroup {
  // -----------------------------public functions---------------------------------------------

  public:
    //------------------------------------------------------------
    // Function:    ArmGroup
    // Summary: The constructor of class
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    ArmGroup();


    //------------------------------------------------------------
    // Function:    ~ArmGroup
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    ~ArmGroup();


    //------------------------------------------------------------------------------
    // Function:    initArmGroup
    // Summary: Initial all the resources in the class
    // In:      joint_values -> initial state of all joints
    // Out:     error_code   -> error code
    // Return:  None
    //------------------------------------------------------------------------------
    bool initArmGroup(ErrorCode &err);


    bool checkZeroOffset(unsigned int &calibrate_result, ErrorCode &err);
    bool calibrateZeroOffset(unsigned int &calibrate_result, ErrorCode &err);


    //------------------------------------------------------------
    // Function:    getCycleTime
    // Summary: To get cycle time of interpolation algorithm.
    // In:      None
    // Out:     None
    // Return:  cycle time
    //------------------------------------------------------------
    double getCycleTime(void);


    //------------------------------------------------------------
    // Function:    getJointConstraints
    // Summary: To get joint constraints from Kinematics algorithm.
    // In:      None
    // Out:     None
    // Return:  joint constraints
    //------------------------------------------------------------
    const JointConstraints& getJointConstraints(void);


    //------------------------------------------------------------
    // Function:    getMaxVelocity
    // Summary: To get max velocity settings.
    // In:      None
    // Out:     None
    // Return:  value of max velocity
    //------------------------------------------------------------
    double getMaxVelocity(void);


    //------------------------------------------------------------
    // Function:    getMaxAcceleration
    // Summary: To get max acceleration settings.
    // In:      None
    // Out:     None
    // Return:  value of max acceleration
    //------------------------------------------------------------
    double getMaxAcceleration(void);


    //------------------------------------------------------------
    // Function:    getVelocityScalingFactor
    // Summary: To get velocity scaling factor value.
    // In:      None
    // Out:     None
    // Return:  global scaling factor of velocity
    //------------------------------------------------------------
    double getVelocityScalingFactor(void);


    //------------------------------------------------------------
    // Function:    getAccelerationScalingFactor
    // Summary: To get acceleration scaling factor value.
    // In:      None
    // Out:     None
    // Return:  global scaling factor of acceleration
    //------------------------------------------------------------
    double getAccelerationScalingFactor(void);


    //------------------------------------------------------------
    // Function:    getCurrentJointValues
    // Summary: To get current values of all joints in the robot.
    // In:      None
    // Out:     None
    // Return:  current values of all six joints
    //------------------------------------------------------------
    const JointValues& getCurrentJointValues(void);


    //------------------------------------------------------------
    // Function:    getCurrentPose
    // Summary: To get current pose of endpoint in the arm group.
    // In:      None
    // Out:     None
    // Return:  current pose of the endpoint
    //------------------------------------------------------------
    const Pose& getCurrentPose(void);


    const JointPoint& getStartJoint(void);

    const Pose& getStartPose(void);


    //------------------------------------------------------------
    // Function:    getPlannedPathFIFOLength
    // Summary: To get the length of planned_path_FIFO.
    // In:      None
    // Out:     None
    // Return:  length of the FIFO
    //------------------------------------------------------------
    int getPlannedPathFIFOLength(void);


    //------------------------------------------------------------
    // Function:    getJointTrajectoryFIFOLength
    // Summary: To get the length of joitn_trajectory_FIFO.
    // In:      None
    // Out:     None
    // Return:  length of the FIFO
    //------------------------------------------------------------
    int getJointTrajectoryFIFOLength(void);


    /*
    //------------------------------------------------------------
    // Function:    getJointTrajectoryFIFOIsLocked
    // Summary: To get joitn_trajectory_FIFO lock state.
    // In:      None
    // Out:     None
    // Return:  true  -> FIFO is locked
    //          false -> FIFO is not locked
    //------------------------------------------------------------
    bool getJointTrajectoryFIFOIsLocked(void);
    */


    //------------------------------------------------------------
    // Function:    getPointsFromJointTrajectoryFIFO
    // Summary: To get points from joitn_trajectory_FIFO.
    // In:      num -> number of joint points want to get
    // Out:     traj-> joint trajectory consisting of joint points
    //          error_code  -> error code
    // Return:  <0  -> joint_trajectory_fifo locked, or any other errors
    //          >=0 -> number of joint points get actually
    //------------------------------------------------------------
    int getPointsFromJointTrajectoryFIFO(std::vector<JointPoint> &traj,
                                         int num, ErrorCode &error_code);


    //------------------------------------------------------------
    // Function:    setCycleTime
    // Summary: To set cycle time of interpolation algorithm.
    // In:      tc  -> desired cycle time
    // Out:     None
    // Return:  true    -> cycle time setted to given value
    //          false   -> cycle time NOT changed
    //------------------------------------------------------------
    bool setCycleTime(double tc);


    //------------------------------------------------------------
    // Function:    setJointConstraints
    // Summary: To set joint constraints in Kinematics algorithm.
    // In:      constraints -> joint constraints
    // Out:     None
    // Return:  true    -> set successfully
    //          false   -> set UNsuccessfully
    //------------------------------------------------------------
    bool setJointConstraints(const JointConstraints &constraints);


    //------------------------------------------------------------
    // Function:    setMaxVelocity
    // Summary: To change max velocity settings.
    // In:      v       -> desired max velocity
    // Out:     None
    // Return:  true    -> max velocity changed to given value
    //          false   -> max velocity NOT changed
    //------------------------------------------------------------
    bool setMaxVelocity(double v);


    //------------------------------------------------------------
    // Function:    setMaxAcceleration
    // Summary: To change max acceleration settings.
    // In:      a       -> desired max acceleration
    // Out:     None
    // Return:  true    -> max acceleration changed to given value
    //          false   -> max acceleration NOT changed
    //------------------------------------------------------------
    bool setMaxAcceleration(double a);


    //------------------------------------------------------------
    // Function:    setVelocityScalingFactor
    // Summary: To set velocity scaling factor.
    // In:      factor  -> desired velocity scaling factor
    // Out:     None
    // Return:  true    -> velocity scaling factor changed to given value
    //          false   -> velocity scaling factor NOT changed
    //------------------------------------------------------------
    bool setVelocityScalingFactor(double factor);


    //------------------------------------------------------------
    // Function:    setAccelerationScalingFactor
    // Summary: To set acceleration scaling factor.
    // In:      factor  -> desired acceleration scaling factor
    // Out:     None
    // Return:  true    -> acceleration scaling factor changed to given value
    //          false   -> acceleration scaling factor NOT changed
    //------------------------------------------------------------
    bool setAccelerationScalingFactor(double factor);


    //------------------------------------------------------------------------------
    // Function:    setJerk
    // Summary: To set the ratio of jerk to acceleration.
    // In:      jerk    -> desired ratio
    // Out:     None
    // Return:  true    -> ratio changed to given value
    //          false   -> ratio NOT changed
    //------------------------------------------------------------------------------
    bool setJerk(double jerk);


    //------------------------------------------------------------------------------
    // Function:    setJointOvershoot
    // Summary: To set joint overshoot.
    // In:      angle   -> desired angle
    // Out:     None
    // Return:  true    -> joint overshoot changed to given value
    //          false   -> joint overshoot NOT changed
    //------------------------------------------------------------------------------
    bool setJointOvershoot(double angle);


    //------------------------------------------------------------------------------
    // Function:    setJointErrorAngle
    // Summary: To set joint error angle.
    // In:      angle   -> desired angle
    // Out:     None
    // Return:  true    -> joint error angle changed to given value
    //          false   -> joint error angle NOT changed
    //------------------------------------------------------------------------------
    bool setJointErrorAngle(double angle);


    //------------------------------------------------------------------------------
    // Function:    setOmegaOverload
    // Summary: To set omega overload.
    // In:      value   -> desired value
    // Out:     None
    // Return:  true    -> omega overload changed to given value
    //          false   -> omega overload NOT changed
    //------------------------------------------------------------------------------
    bool setOmegaOverload(double value);


    //------------------------------------------------------------------------------
    // Function:    setAlphaOverload
    // Summary: To set alpha overload.
    // In:      value   -> desired value
    // Out:     None
    // Return:  true    -> alpha overload changed to given value
    //          false   -> alpha overload NOT changed
    //------------------------------------------------------------------------------
    bool setAlphaOverload(double value);


    //------------------------------------------------------------------------------
    // Function:    setSmoothRadiusCoefficient
    // Summary: To set smooth radius coefficient.
    // In:      coeff   -> desired value
    // Out:     None
    // Return:  true    -> smooth radius coefficient changed to given value
    //          false   -> smooth radius coefficient NOT changed
    //------------------------------------------------------------------------------
    bool setSmoothRadiusCoefficient(double coeff);


    //------------------------------------------------------------------------------
    // Function:    setSmoothCurveMode
    // Summary: To set smooth curve mode.
    // In:      mode    -> desired smooth curve mode
    // Out:     None
    // Return:  true    -> desired smooth curve mode changed to given value
    //          false   -> desired smooth curve mode NOT changed
    //------------------------------------------------------------------------------
    bool setSmoothCurveMode(int mode);


    //------------------------------------------------------------
    // Function:    setToolFrame
    // Summary: To set current tool frame.
    // In:      tool_frame -> current tool frame
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void setToolFrame(const Transformation &tool_frame);


    //------------------------------------------------------------
    // Function:    setUserFrame
    // Summary: To set current user frame.
    // In:      user_frame -> current user frame
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void setUserFrame(const Transformation &user_frame);


    //------------------------------------------------------------
    // Function:    setCurrentJointValues
    // Summary: To set current joint values using encoder data.
    //          Current pose values will be updated automaticly.
    // In:      current_joint-> joint values from the encoder
    // Out:     Error_code  -> error code
    // Return:  true    -> current joint/pose values updated successfully
    //          false   -> either joint or pose values NOT updated
    //------------------------------------------------------------
    bool setCurrentJointValues(const JointValues &current_joint, ErrorCode &error_code);


    //------------------------------------------------------------
    // Function:    setStartState
    // Summary: To set robot start state.
    // In:      joint_start -> robot start state
    // Out:     error_code  -> error code
    // Return:  true    -> robot start state setted to joint_start scucessfully
    //          false   -> failed to set robot start state
    //------------------------------------------------------------
    bool setStartState(const JointValues &joint_start, ErrorCode &error_code);


    //------------------------------------------------------------
    // Function:    clearPlannedPathFIFO
    // Summary: To clear planned_path_fifo.
    // In:      None
    // Out:     err     -> error code
    // Return:  true    -> planned path FIFo is cleared successfully
    //          false   -> FIFO cleared with some errors
    //------------------------------------------------------------
    bool clearPlannedPathFIFO(ErrorCode &err);

    /*
    //------------------------------------------------------------
    // Function:    clearJointTrajectoryFIFO
    // Summary: To clear joint_trajectory_fifo(FIFO2) and planned_path_fifo(FIFO1).
    // In:      None
    // Out:     err     -> error code
    // Return:  true    -> two FIFOs are both cleared successfully
    //          false   -> FIFOs cleared with some errors
    //------------------------------------------------------------
    bool clearJointTrajectoryFIFO(ErrorCode &err);
    */


    //------------------------------------------------------------
    // Function:    computeIK
    // Summary: To compute IK with a given pose in cartesian space.
    // In:      poes    -> cartesian space pose needed to compute IK
    // Out:     joint_result-> IK result
    //          err     -> error code
    // Return:  true    -> IK solution found
    //          false   -> IK solution NOT found
    //------------------------------------------------------------
    bool computeIK(const Pose &pose, JointValues &joint_result, ErrorCode &err);


    //------------------------------------------------------------
    // Function:    computeFK
    // Summary: To compute FK with given joint values.
    // In:      joint_result-> joint values needed to compute FK
    // Out:     poes    -> FK result
    //          err     -> error code
    // Return:  true    -> FK computed successfully
    //          false   -> FK computed UNsuccessfully
    //------------------------------------------------------------
    bool computeFK(const JointValues &joint, Pose &pose, ErrorCode &err);


    //------------------------------------------------------------
    // Function:    MoveJ
    // Summary: To plan a path in joint space to touch target pose, without smooth.
    // In:      joint_target-> target in joint space
    //          v_max   -> max velocity
    //          a_max   -> max acceleration
    //          cnt     -> smooth degree
    //          id      -> command id
    // Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
    //          err     -> error code
    // Return:  true    -> plan successfully
    //          false   -> plan UNsuccessfully
    //------------------------------------------------------------
    bool MoveJ(const JointValues &joint_target, double v_max, double a_max,
               int id, ErrorCode &err);


    //------------------------------------------------------------
    // Function:    MoveJ (smooth to MoveJ)
    // Summary: To plan a path in joint space to touch target pose, with smooth.
    // In:      joint_target-> target in joint space
    //          v_max   -> max velocity
    //          a_max   -> max acceleration
    //          cnt     -> smooth degree
    //          joint_next  -> next target in joint space
    //          v_next  -> max velocity in the next path
    //          a_next  -> max acceleration in the next path
    //          cnt_next    -> smooth degree in the next path
    //          id      -> command id
    // Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
    //          err     -> error code
    // Return:  true    -> plan successfully
    //          false   -> plan UNsuccessfully
    //------------------------------------------------------------
    bool MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
               const JointValues &joint_next, double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);


    //------------------------------------------------------------
    // Function:    MoveJ (smooth to MoveL)
    // Summary: To plan a path in joint space to touch target pose, with smooth.
    // In:      joint_target-> target in joint space
    //          v_max   -> max velocity
    //          a_max   -> max acceleration
    //          cnt     -> smooth degree
    //          pose_next   -> next target in Cartesian space
    //          v_next  -> max velocity in the next path
    //          a_next  -> max acceleration in the next path
    //          cnt_next    -> smooth degree in the next path
    //          id      -> command id
    // Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
    //          err     -> error code
    // Return:  true    -> plan successfully
    //          false   -> plan UNsuccessfully
    //------------------------------------------------------------
    bool MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
               const Pose &pose_next, double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);


    //------------------------------------------------------------
    // Function:    MoveJ (smooth to MoveL)
    // Summary: To plan a path in joint space to touch target pose, with smooth.
    // In:      joint_target-> target in joint space
    //          v_max   -> max velocity
    //          a_max   -> max acceleration
    //          cnt     -> smooth degree
    //          pose_next   -> next target in Cartesian space
    //          v_next  -> max velocity in the next path
    //          a_next  -> max acceleration in the next path
    //          cnt_next    -> smooth degree in the next path
    //          id      -> command id
    // Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
    //          err     -> error code
    // Return:  true    -> plan successfully
    //          false   -> plan UNsuccessfully
    //------------------------------------------------------------
    bool MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
               const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);


    //------------------------------------------------------------------------------
    // Function:    MoveJ (smooth to MoveC)
    // Summary: To plan a path in joint space to touch target pose, with smooth.
    //------------------------------------------------------------------------------
    bool MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
               const Pose &pose2_circle, const Pose &pose3_circle,
               double v_circle, double a_circle, int cnt_circle,
               int id, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveJ (smooth to MoveC)
    // Summary: To plan a path in joint space to touch target pose, with smooth.
    //------------------------------------------------------------------------------
    bool MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
               const PoseEuler &pose2_circle, const PoseEuler &pose3_circle,
               double v_circle, double a_circle, int cnt_circle,
               int id, ErrorCode &err);


    //------------------------------------------------------------
    // Function:    MoveL (without smooth)
    // Summary: To plan a linear path to touch target pose, without smooth.
    // In:      pose_target -> target pose of the linear path
    //          v_max   -> max velocity of endpoint
    //          a_max   -> max acceleration of endpoint
    //          id      -> command id
    // Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
    //          err     -> error code
    // Return:  true    -> plan successfully
    //          false   -> plan UNsuccessfully
    //------------------------------------------------------------
    bool MoveL(const Pose &pose_target, double v_max, double a_max,
               int id, ErrorCode &err);


    //------------------------------------------------------------
    // Function:    MoveL (without smooth)
    // Summary: To plan a linear path to touch target pose, without smooth.
    // In:      pose_target -> target pose of the linear path
    //          v_max   -> max velocity of endpoint
    //          a_max   -> max acceleration of endpoint
    //          id      -> command id
    // Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
    //          err     -> error code
    // Return:  true    -> plan successfully
    //          false   -> plan UNsuccessfully
    //------------------------------------------------------------
    bool MoveL(const PoseEuler &pose_target, double v_max, double a_max,
               int id, ErrorCode &err);

    //------------------------------------------------------------
    // Function:    MoveL (smooth to MoveJ)
    // Summary: To plan a linear path to touch target pose, with smooth.
    // In:      pose_target -> target pose of the linear path
    //          v_max   -> max velocity of endpoint
    //          a_max   -> max acceleration of endpoint
    //          cnt_target  -> smooth degree
    //          joint_next  -> target pose in joint space of the next path
    //          v_next  -> max velocity of endpoint in the next path
    //          a_next  -> max acceleration of endpoint in the next path
    //          cnt_next    -> smooth degree in the next path
    //          id      -> command id
    // Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
    //          err     -> error code
    // Return:  true    -> plan successfully
    //          false   -> plan UNsuccessfully
    //------------------------------------------------------------
    bool MoveL(const Pose &pose_target, double v_max, double a_max, int cnt_target,
               const JointValues &joint_next, double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);

    //------------------------------------------------------------
    // Function:    MoveL (smooth to MoveJ)
    // Summary: To plan a linear path to touch target pose, with smooth.
    // In:      pose_target -> target pose of the linear path
    //          v_max   -> max velocity of endpoint
    //          a_max   -> max acceleration of endpoint
    //          cnt_target  -> smooth degree
    //          joint_next  -> target pose in joint space of the next path
    //          v_next  -> max velocity of endpoint in the next path
    //          a_next  -> max acceleration of endpoint in the next path
    //          cnt_next    -> smooth degree in the next path
    //          id      -> command id
    // Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
    //          err     -> error code
    // Return:  true    -> plan successfully
    //          false   -> plan UNsuccessfully
    //------------------------------------------------------------
    bool MoveL(const PoseEuler &pose_target, double v_max, double a_max, int cnt_target,
               const JointValues &joint_next, double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);


    //------------------------------------------------------------
    // Function:    MoveL (smooth to MoveL)
    // Summary: To plan a linear path to touch target pose, with smooth.
    // In:      pose_target -> target pose of the linear path
    //          v_max   -> max velocity of endpoint
    //          a_max   -> max acceleration of endpoint
    //          cnt_target  -> smooth degree
    //          pose_next   -> target pose of the next path
    //          v_next  -> max velocity of endpoint in the next path
    //          a_next  -> max acceleration of endpoint in the next path
    //          cnt_next-> smooth degree in the next path
    //          id      -> command id
    // Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
    //          err     -> error code
    // Return:  true    -> plan successfully
    //          false   -> plan UNsuccessfully
    //------------------------------------------------------------
    bool MoveL(const Pose &pose_target, double v_max, double a_max, int cnt_target,
               const Pose &pose_next, double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);


    //------------------------------------------------------------
    // Function:    MoveL (smooth to MoveL)
    // Summary: To plan a linear path to touch target pose, with smooth.
    // In:      pose_target -> target pose of the linear path
    //          v_max   -> max velocity of endpoint
    //          a_max   -> max acceleration of endpoint
    //          cnt_target  -> smooth degree
    //          pose_next   -> target pose of the next path
    //          v_next  -> max velocity of endpoint in the next path
    //          a_next  -> max acceleration of endpoint in the next path
    //          cnt_next    -> smooth degree in the next path
    //          id      -> command id
    // Out:     path(hidden)-> outputs added into m_planned_path_FIFO automaticly
    //          err     -> error code
    // Return:  true    -> plan successfully
    //          false   -> plan UNsuccessfully
    //------------------------------------------------------------
    bool MoveL(const PoseEuler &pose_target, double v_max, double a_max, int cnt_target,
               const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveL (smooth to MoveC)
    // Summary: To plan a linear path to touch target pose, with smooth.
    //------------------------------------------------------------------------------
    bool MoveL(const Pose &pose_target, double v_max, double a_max, int cnt_target,
               const Pose &pose2_circle, const Pose &pose3_circle,
               double v_circle, double a_circle, int cnt_circle,
               int id, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveL (smooth to MoveC)
    // Summary: To plan a linear path to touch target pose, with smooth.
    //------------------------------------------------------------------------------
    bool MoveL(const PoseEuler &pose_target, double v_max, double a_max, int cnt_target,
               const PoseEuler &pose2_circle, const PoseEuler &pose3_circle,
               double v_circle, double a_circle, int cnt_circle,
               int id, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveC (without smooth)
    bool MoveC(const Pose pose_2nd, const Pose pose_3rd,
               double v_target, double a_target,
               int id, ErrorCode &err);
    bool MoveC(const PoseEuler pose_2nd, const PoseEuler pose_3rd,
               double v_target, double a_target,
               int id, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveC (smooth to MoveJ)
    // Summary: To plan a circle path to touch target pose, with smooth.
    //------------------------------------------------------------------------------
    bool MoveC(const Pose &pose2_circle, const Pose &pose3_circle,
               double v_max, double a_max, int cnt_target,
               const JointValues &joint_next, double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveC (smooth to MoveJ)
    // Summary: To plan a circle path to touch target pose, with smooth.
    //------------------------------------------------------------------------------
    bool MoveC(const PoseEuler &pose2_circle, const PoseEuler &pose3_circle,
               double v_max, double a_max, int cnt_target,
               const JointValues &joint_next, double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveC (smooth to MoveL)
    // Summary: To plan a circle path to touch target pose, with smooth.
    //------------------------------------------------------------------------------
    bool MoveC(const Pose &pose2_circle, const Pose &pose3_circle,
               double v_max, double a_max, int cnt_target,
               const Pose &pose_next, double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveC (smooth to MoveL)
    // Summary: To plan a circle path to touch target pose, with smooth.
    //------------------------------------------------------------------------------
    bool MoveC(const PoseEuler &pose2_circle, const PoseEuler &pose3_circle,
               double v_max, double a_max, int cnt_target,
               const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveC (smooth to MoveC)
    // Summary: To plan a circle path to touch target pose, with smooth.
    //------------------------------------------------------------------------------
    bool MoveC(const Pose &pose2_circle, const Pose &pose3_circle,
               double v_max, double a_max, int cnt_target,
               const Pose &pose4_circle, const Pose &pose5_circle,
               double v_next, double a_next, int cnt_next,
               int id, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveC (smooth to MoveC)
    // Summary: To plan a circle path to touch target pose, with smooth.
    //------------------------------------------------------------------------------
    bool MoveC(const PoseEuler &pose2_circle, const PoseEuler &pose3_circle,
         double v_max, double a_max, int cnt_target,
         const PoseEuler &pose4_circle, const PoseEuler &pose5_circle,
         double v_next, double a_next, int cnt_next,
         int id, ErrorCode &err);


    //------------------------------------------------------------
    // Function:    convertPathToTrajectory
    // Summary: To convert numbers of posepoint in m_cartesian_path_FIFO
    //          into jointpoint in m_joint_trajectory_FIFO.
    // In:      num -> number of points in planned_path_fifo that needed to be converted
    // Out:     err -> error code
    // Return:  <0  -> ERROR occurred during converting
    //          >=0 -> number of pose that convered actually
    //------------------------------------------------------------
    int convertPathToTrajectory(int num, ErrorCode &err);
    
    
    //------------------------------------------------------------
    // Function:    suspendArmMotion
    // Summary: To replan a slow-down path and stop. Used when pause event
    //          or IK failure raised.
    // In:      None
    // Out:     err     -> error code
    // Return:  true    -> replan successfully
    //          false   -> replan UNsuccessfully
    //------------------------------------------------------------
    bool suspendArmMotion(ErrorCode &err);


    //------------------------------------------------------------
    // Function:    resumeArmMotion
    // Summary: To replan a start-up path and resume arm motion.
    // In:      None
    // Out:     err     -> error code
    // Return:  true    -> replan successfully
    //          false   -> replan UNsuccessfully
    //------------------------------------------------------------
    bool resumeArmMotion(ErrorCode &err);

    bool isArmGroupSuspended(void);

    bool resetArmGroup(const JointValues &joint, ErrorCode &err);

    //------------------------------------------------------------
    // Function:    transformPoseEuler2Pose
    // Summary: To transform a poseEuler point to a pose point.
    // In:      poes_e  -> the poseEuler to be transformed
    // Out:     None
    // Return:  pose point
    //------------------------------------------------------------
    Pose transformPoseEuler2Pose(const PoseEuler &pose_e);


    //------------------------------------------------------------
    // Function:    transformPose2PoseEuler
    // Summary: To transform a pose point to a poseEuler point.
    // In:      poes    -> the pose to be transformed
    // Out:     None
    // Return:  poseEuler point
    //------------------------------------------------------------
    PoseEuler transformPose2PoseEuler(const Pose &pose);



    // -----------------------------private functions---------------------------------------------

  private:
    inline void printJointValues(const JointValues &joint);
    inline void printJointValues(const char *str, const JointValues &joint);
    inline void printJointLimit(const JointLimit &joint_limit);
    inline void printJointLimit(const char *str, const JointLimit &joint_limit);
    inline void printJointConstraints(const JointConstraints &constraint);
    inline void printPose(const Pose &pose);
    inline void printPose(const char *str, const Pose &pose);
    inline void printPoseEuler(const PoseEuler &pose_euler);
    inline void printPoseEuler(const char *str, const PoseEuler &pose_euler);
    inline void printJointPoint(const JointPoint &point);
    inline void printJointPoint(const char *str, const JointPoint &point);
    inline void printPathPoint(const PathPoint &point);
    inline void printPathPoint(const char *str, const PathPoint &point);
    
    void lockArmGroup(void);
    void unlockArmGroup(void);



    //------------------------------------------------------------
    // Function:    setLatestIKReference
    // Summary: To set latest IK reference.
    // In:      joint_reference -> new IK reference
    // Out:     error_code  -> error code
    // Return:  true    -> latest IK reference setted to joint_reference scucessfully
    //          false   -> failed to set latest IK reference
    //------------------------------------------------------------
    bool setLatestIKReference(const JointValues &joint_reference, ErrorCode &error_code);



    bool setArmGroupParameters(XmlRpc::XmlRpcValue &params);

    //------------------------------------------------------------
    // Function:    checkJointBoundary
    // Summary: To check whether a group of joint values are valid according to
    //          joint constraints.
    // In:      joint_values    -> joint_values needed to be checked
    // Out:     None
    // Return   true    -> valid
    //          false   -> INvalid
    //------------------------------------------------------------
    bool checkJointBoundary(const JointValues &joint_values);


    //------------------------------------------------------------
    // Function:    getLatestIKReference
    // Summary: To get latest IK reference values.
    // In:      None
    // Out:     None
    // Return:  a group of joint values used as IK reference
    //------------------------------------------------------------
    const JointValues& getLatestIKReference(void);


    //------------------------------------------------------------
    // Function:    convertPath2Trajectory
    // Summary: To convert numbers of posepoint in m_cartesian_path_FIFO
    //          into jointpoint in m_joint_trajectory_FIFO.
    // In:      num -> number of points in planned_path_fifo that needed to be converted
    // Out:     error_code  -> error code
    // Return:  <0  -> ERROR occurred during converting
    //          >=0 -> number of pose that convered actually
    //------------------------------------------------------------
    int convertPath2Trajectory(int num, ErrorCode &error_code);

    bool isMotionExecutable(MotionType motion_type);
    bool insertAdditionSmooth(ErrorCode &err);
 

    // -----------------------------member variables---------------------------------------------

    int    m_JOINT_FIFO_LEN;


    // intermediate variable used during planning path with smooth
    Pose m_pose_previous;
    Pose m_pose_start;
    Pose m_pose_start_past;     // P_sta_ptc
    JointPoint m_joint_start;
    double m_vu_start;
    double m_v_start;
    MotionType allowed_motion_type_;

    // point level: start-point, middle-point or ending-point
    //PointLevel m_point_level;


    // joint constraints
    JointConstraints m_joint_constraints;

    // latest IK reference
    JointValues m_latest_ik_reference;

    // current state of the arm group in joint space and cartesian space
    JointValues m_joint_state_current;
    Pose m_pose_state_current;

    // tool frame and user frame
    Transformation m_tool_frame;
    Transformation m_user_frame;

    // planned path FIFO (FIFO1)
    std::vector<PathPoint> m_planned_path_fifo;
    // joint space trajectory FIFO (FIFO2)
    std::vector<JointPoint> m_joint_trajectory_fifo;
    
    /*
    // joint FIFO lock. FIFO is locked when pause or IK failure event arised.
    bool m_joint_trajectory_FIFO_islocked;
    */

    enum SuspendPattern{
        e_suspend_pattern_origin = 0,
        e_suspend_pattern_replan = 1,
    };

    struct SuspendState {
        bool is_suspended;
        SuspendPattern pattern;
        JointValues last_point;
        std::vector<JointPoint> fifo2_cache;
        std::vector<JointValues> replan_trajectory;
    } m_suspend_state;

    // algorithm pointer
    fst_controller::PlanningInterface   *planning_interface_;
    fst_controller::Calibrator          *calibrator_;

    unsigned int current_state_;
    bool enable_calibration_;
    

    struct {
        int  id;
        bool addition_smooth;
        MotionType motion_type;
        SmoothType smooth_type;
    } last_motion_;
    
    pthread_mutex_t m_group_mutex;

    fst_log::Logger log;
    // std::string m_log_file_name;
    // std::string m_log_file_content;
};  // class ArmGroup
}   // namespace fst_controller


#endif  // #ifndef FST_CONTROLLER_H
