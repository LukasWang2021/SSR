/*************************************************************************
	> File Name: motion_controller_planning_interface.h
	> Author: 
	> Mail: 
	> Created Time: 2017年02月20日 星期一 10时55分58秒
 ************************************************************************/

#ifndef _MOTION_CONTROLLER_PLANNING_INTERFACE_H
#define _MOTION_CONTROLLER_PLANNING_INTERFACE_H

#include <vector>
#include <map>
#include <motion_controller/fst_datatype.h>
#include <trajplan/TrajPlan.h>
#include <motion_controller/motion_controller_error_code.h>
#include <parameter_manager/parameter_manager_param_group.h>

namespace fst_controller {

class PlanningInterface {

  public:

    PlanningInterface();
    ~PlanningInterface();
    bool initPlanningInterface(fst_parameter::ParamValue &params, ErrorCode &err);

    std::string getAlgorithmVersion(void);

    double getCycleTime(void);

    double getVelocity(void);

    double getAcceleration(void);

    double getVelocityScaling(void);

    double getAccelerationScaling(void);

    double getJerk(void);

    double getJointOvershoot(void);

    double getJointErrorAngle(void);

    double getOmegaOverload(void);

    double getAlphaOverload(void);

    double getSmoothRadiusCoefficient(void);

    double getCurveMode(void);

    const JointConstraints& getJointConstraints(void);

    void clearFIFO(void);

    int getFIFOLength(void);

    unsigned int getTrajectorySegmentLength(void);

    bool setTrajectorySegmentLength(unsigned int length);

    //------------------------------------------------------------
    // Function:    setCycleTime
    // Summary: To set cycle time of interpolation algorithm, default value: 0.001.
    // In:      cycle_time -> desired cycle time
    // Out:     None
    // Return:  true    -> cycle time setted to given value
    //          false   -> cycle time NOT changed
    //------------------------------------------------------------
    bool setCycleTime(double cycle_time);

    bool setVelocity(double vel);

    //------------------------------------------------------------
    // Function:    setAcceleration
    // Summary: To change acceleration in cartesian space.
    // In:      acce    -> desired acceleration
    // Out:     None
    // Return:  true    -> acceleration changed to given value
    //          false   -> acceleration NOT changed
    //------------------------------------------------------------
    bool setAcceleration(double acce);

    bool setVelocityScaling(double scaling);

    bool setAccelerationScaling(double scaling);

    bool setJerk(double jerk);

    bool setJointOvershoot(double angle);

    bool setJointErrorAngle(double angle);

    bool setOmegaOverload(double value);

    bool setAlphaOverload(double value);

    bool setSmoothRadiusCoefficient(double coeff);

    bool setCurveMode(int mode);

    //------------------------------------------------------------------------------
    // Function:    setJointConstraints
    // Summary: To set joint constraints in Kinematics algorithm.
    // In:      constraints -> joint constraints;
    // Out:     None
    // Return:  true    -> joint constraints changed to given value
    //          false   -> joint constraints NOT changed
    //------------------------------------------------------------------------------
    bool setJointConstraints(const JointConstraints &constraints);

    bool setAlphaScaling(double percent);

    bool setDH(const DHGroup &dh);

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

    //------------------------------------------------------------------------------
    // Function:    transformPoseEuler2Pose
    // Summary: To transform a poseEuler point to a pose point.
    // In:      poes_e -> the poseEuler to be transformed
    // Out:     None
    // Return:  pose point
    //------------------------------------------------------------------------------
    Pose transformPoseEuler2Pose(const PoseEuler &pose_e);

    //------------------------------------------------------------------------------
    // Function:    transformPose2PoseEuler
    // Summary: To transform a pose point to a poseEuler point.
    // In:      poes -> the pose to be transformed
    // Out:     None
    // Return:  poseEuler point
    //------------------------------------------------------------------------------
    PoseEuler transformPose2PoseEuler(const Pose &pose);

    bool getJointFromPose(const Pose &pose, const JointValues &reference,
                          JointValues &joint, double time_interval, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    computeInverseKinematics
    // Summary: To compute IK with a given pose in cartesian space.
    // In:      poes         -> cartesian space pose needed to compute IK
    //          joint_reference -> joint reference used during compute IK
    // Out:     joint_result -> IK result
    //          error_code   -> error code
    // Return:  true         -> IK solution found
    //------------------------------------------------------------------------------
    bool computeInverseKinematics(const Pose &pose,
                                  const JointValues &joint_reference,
                                  JointValues &joint_result,
                                  ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    computeForwardKinematics
    // Summary: To compute FK with given joint values.
    // In:      joint_result -> joint values needed to compute FK
    // Out:     poes         -> FK result
    //          error_code   -> error code
    // Return:  true         -> FK computed successfully
    //------------------------------------------------------------------------------
    bool computeForwardKinematics(const JointValues &joint, Pose &pose, ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveJ2J
    // Summary: To plan a path in joint space to touch target pose, with/without smooth.
    // In:      jp_start    -> initial position and omega of six joints
    //          jp_target   -> joint target of the plan
    //          jp_next     -> the target joint of the next plan used for smoothing
    //          v_ref       -> reference velocity
    //          cnt         -> desired cnt value of the plan, range:0-100
    // Out:     jp_end      -> the ending position and omega of six joints in this plan
    //          planned_path-> planned path in joint space
    //          error_code  -> error code
    // Return:  true        -> plan successfully
    //          false       -> plan UNsuccessfully
    //------------------------------------------------------------------------------
    bool MoveJ2J(JointPoint &jp_start,
                 const JointValues &j_target, double v_target, int cnt_target,
                 const JointValues &j_next, double v_next, int cnt_next,
                 ErrorCode &err);

/*
    //------------------------------------------------------------------------------
    // Function:    MoveJ2J
    // Summary: To plan a path in joint space to touch target pose, with/without smooth.
    // In:      jp_start    -> initial position and omega of six joints
    //          pose_target -> pose target of the plan
    //          pose_next   -> the target pose of the next plan used for smoothing 
    //          v_ref       -> reference velocity
    //          cnt         -> desired cnt value of the plan, range:0-100
    // Out:     jp_end      -> the ending position and omega of six joints in this plan
    //          planned_path-> planned path in joint space
    //          error_code  -> error code
    // Return:  true        -> plan successfully
    //          false       -> plan UNsuccessfully
    //------------------------------------------------------------------------------
    bool MoveJ2J(JointPoint &jp_start,
                 const Pose &pose_target, const Pose &pose_next, double v_ref, int cnt,
                 std::vector<JointValues> &planned_path,
                 ErrorCode &err);*/

    //------------------------------------------------------------------------------
    // Function:    MoveJ2L
    // Summary: To plan a path in joint space to touch target pose, with/without smooth.
    // In:      jp_start    -> initial position and omega of six joints
    //          joint_target-> joint target of the plan
    //          v_ref       -> reference velocity
    //          cnt         -> desired cnt value of the plan, range:0-100
    //          pose_next   -> the target pose of the next plan used for smoothing
    //          v_next      -> desired velocity of the next plan used for smoothing
    //          cnt_next    -> desired cnt value of the next plan used for smoothing
    // Out:     pose_start  -> the end pose of this plan, also the initial pose of the next plan
    //          pose_previous -> the pose_previous input in the next plan
    //          v_start     -> the end velocity of this plan, also the initial velocity of the next plan
    //          vu_start    -> the end value of intermediate-variable in this plan, also the initial value in next plan
    //          planned_path-> planned path in joint space
    //          error_code  -> error code
    // Return:  true        -> plan successfully
    //          false       -> plan UNsuccessfully
    //------------------------------------------------------------------------------
    bool MoveJ2L(const JointPoint &jp_start,
                 const JointValues &joint_target, double v_target, int cnt_target,
                 const Pose &pose_next, double v_next, int cnt_next,
                 Pose &pose_start, Pose &pose_previous,
                 double &v_start, double &vu_start,
                 ErrorCode &err);

/*
    //------------------------------------------------------------------------------
    // Function:    MoveJ2L
    // Summary: To plan a path in joint space to touch target pose, with/without smooth.
    // In:      jp_start    -> initial position and omega of six joints
    //          pose_target -> joint target of the plan
    //          v_ref       -> reference velocity
    //          cnt         -> desired cnt value of the plan, range:0-100
    //          pose_next   -> the target pose of the next plan used for smoothing
    //          v_next      -> desired velocity of the next plan used for smoothing
    //          cnt_next    -> desired cnt value of the next plan used for smoothing
    // Out:     pose_start  -> the end pose of this plan, also the initial pose of the next plan
    //          pose_previous -> the pose_previous input in the next plan
    //          v_start     -> the end velocity of this plan, also the initial velocity of the next plan
    //          vu_start    -> the end value of intermediate-variable in this plan, also the initial value in next plan
    //          planned_path-> planned path in joint space
    //          error_code  -> error code
    // Return:  true        -> plan successfully
    //          false       -> plan UNsuccessfully
    //------------------------------------------------------------------------------
    bool MoveJ2L(const JointPoint &jp_start,
                 const Pose &pose_target, double v_ref, int cnt_target,
                 const Pose &pose_next, double v_next, int cnt_next,
                 Pose &pose_start, Pose &pose_previous,
                 double &v_start, double &vu_start,
                 ErrorCode &err);*/

    //------------------------------------------------------------------------------
    // Function:    MoveJ2C
    // Summary: To plan a path in joint space to touch target pose, with/without smooth.
    // In:      jp_start    -> initial position and omega of six joints
    //          j_target    -> joint target of the plan
    //          v_ref       -> reference velocity
    //          cnt         -> desired cnt value of the plan, range:0-100
    //          pose2_circle-> the target pose of the next circle plan used for smoothing 
    //          pose3_circle-> the target pose of the next circle plan used for smoothing 
    // Out:     jp_end      -> the ending position and omega of six joints in this plan
    //          planned_path-> planned path in joint space
    //          error_code  -> error code
    // Return:  true        -> plan successfully
    //------------------------------------------------------------------------------
    bool MoveJ2C(const JointPoint &jp_start,
                 const JointValues &j_target, double v_target, int cnt_target,
                 const Pose &pose1_circle, const Pose &pose2_circle, double v_circle, int cnt_circle,
                 Pose &pose_start, double &v_start,
                 ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    MoveL2J
    // Summary: To plan a linear path to touch target pose, with/without smooth.
    // In:      pose_start  -> initial pose of the plan
    //          v_start     -> initial velocity of the plan
    //          vu_start    -> initial value of the intermediate-variable
    //          pose_target -> target pose of the plan
    //          v_target    -> desired velocity of the plan
    //          cnt_target  -> desired cnt value of the plan
    //          pose_previous-> the previous pose used for interpolation
    // Out:     planned_path-> planned path in cartesian space
    //          jp          -> position and omega of six joints at the ending of this path
    //          error_code  -> error code
    // Return:  true  -> plan successfully
    //------------------------------------------------------------------------------
    bool MoveL2J(const Pose &pose_start, double v_start, double vu_start,
                 const Pose &pose_target, double v_target, int cnt_target,
                 Pose &pose_previous, ErrorCode &err);

    //------------------------------------------------------------
    // Function:    MoveL2L
    // Summary: To plan a linear path to touch target pose, with/without smooth.
    // In:      pose_start  -> initial pose of the plan
    //          v_start -> initial velocity of the plan
    //          vu_start    -> initial value of the intermediate-variable
    //          pose_target -> target pose of the plan
    //          v_target    -> desired velocity of the plan
    //          cnt_target  -> desired cnt value of the plan
    //          pose_next   -> the target pose of the next plan used for smoothing
    //          v_next  ->  desired velocity of the next plan used for smoothing
    //          cnt_next    -> desired cnt value of the next plan used for smoothing
    //          pose_previuos   -> the previous pose used for interpolation
    // Out:     pose_start  -> the end pose of this plan, also the initial pose of the next plan
    //          v_start -> the end velocity of this plan, also the initial velocity of the next plan
    //          vu_start    -> the end value of intermediate-variable in this plan, also the initial value in next plan
    //          pose_previous   -> the pose_previous in the next plan
    //          planned_path-> planned path in cartesian space
    //          error_code  -> error code
    // Return:  true    -> plan successfully
    //          false   -> plan UNsuccessfully
    //------------------------------------------------------------
    bool MoveL2L(Pose &pose_start, double &v_start, double &vu_start,
                 const Pose &pose_target, double v_target, int cnt_target,
                 const Pose &pose_next, double v_next, int cnt_next,
                 Pose &pose_previous, ErrorCode &error_code);

    bool MoveL2C(Pose &pose_start, double &v_start, double &vu_start,
                 const Pose &pose_target, double v_target, int cnt_target,
                 const Pose &pose1_circle, const Pose &pose2_circle, double v_circle, int cnt_circle,
                 Pose &pose_previous, ErrorCode &err);

/*
    bool MoveL2CAdditionSmooth(const JointPoint &jp_start, const Pose &pose_start,
                               const Pose &pose_start_past, std::vector<JointValues> &planned_path,
                               ErrorCode &err);*/

    bool MoveC2J(const Pose &pose_start, double v_start,
                 const Pose &pose1, const Pose &pose2, double v_target, int cnt_target,
                 ErrorCode err);

    bool MoveC2L(Pose &pose_start, double &v_start,
                 const Pose &pose1, const Pose &pose2, double v_target, int cnt_target,
                 const Pose &pose_next, double v_next, int cnt_next,
                 Pose &pose_previous, double &vu_start, ErrorCode &err);

/*
    bool MoveC2LAdditionSmooth(const JointPoint &jp_start, const Pose &pose_start,
                               const Pose &pose_start_past, std::vector<JointValues> &planned_path,
                               ErrorCode &err);*/

    bool MoveC2C(Pose &pose_start, double &v_start,
                 const Pose &pose1_target, const Pose &pose2_target, double v_target, int cnt_target,
                 const Pose &pose1_next, const Pose &pose2_next, double v_next, int cnt_next,
                 ErrorCode &err);

/*
    bool MoveC2CAdditionSmooth(const JointPoint &jp_start, const Pose &pose_start,
                               const Pose &pose_start_past, std::vector<JointValues> &planned_path,
                               ErrorCode &err);*/

    bool pickPoints(vector<Pose> &points, ErrorCode &err);

    bool pickPoints(vector<JointValues> &points, ErrorCode &err);

    int estimateFIFOLength(JointValues joint1, JointValues joint2);

    bool replanPauseTrajectory(std::vector<JointValues> &trajectory, ErrorCode &err);

    bool replanRestartTrajectory(std::vector<JointValues> &trajectory,
                                 JointValues &start_point,
                                 ErrorCode &err);

    bool isPointCoincident(const Pose &pose1, const Pose &pose2);
    bool isPointCoincident(const Pose &pose, const JointValues &joint);
    bool isPointCoincident(const JointValues &joint, const Pose &pose);
    bool isPointCoincident(const JointValues &joint1, const JointValues &joint2);

  private:
    // cycle time between two points in the trajectory, Unit: s
    double  cycle_time_;
    // velocity in cartesian space
    double  velocity_;
    // acceleration in cartesian space
    double  acceleration_;
    // velocity scaling factor
    double  velocity_scaling_;
    // acceleration scaling factor
    double  acceleration_scaling_;

    double  jerk_;
    double  joint_overshoot_;
    double  joint_errorangle_;
    double  omega_overload_;
    double  alpha_overload_;
    double  smooth_radius_coefficient_;
    int     curve_mode_;
    int     fifo_length_;
    unsigned int trajectory_segment_length_;

    JointConstraints joint_constraints_;
    DHGroup dh_parameter_;

    // ultimate values
    struct {
        double min;
        double max;
    }
    trajectory_segment_length_range_,
    cycle_time_range_,
    velocity_range_,
    acceleration_range_,
    velocity_scaling_range_,
    acceleration_scaling_range_,
    jerk_range_,
    joint_overshoot_range_,
    joint_errorangle_range_,
    alpha_overload_range_,
    omega_overload_range_,
    smooth_radius_coefficient_range_;


    TrajPlan  *planner_;

};

}


#endif
