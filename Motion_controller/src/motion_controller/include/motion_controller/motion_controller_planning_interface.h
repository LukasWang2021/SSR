/*************************************************************************
	> File Name: motion_controller_planning_interface.h
	> Author: 
	> Mail: 
	> Created Time: 2017年02月20日 星期一 10时55分58秒
 ************************************************************************/

#ifndef _MOTION_CONTROLLER_PLANNING_INTERFACE_H
#define _MOTION_CONTROLLER_PLANNING_INTERFACE_H

#include <vector>
#include <trajplan/fst_datatype.h>
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

    //double getVelocity(void);

    double getAcceleration(void);

    double getVelocityScaling(void);

    double getAccelerationScaling(void);

    double getJerk(void);

    double getJointOvershoot(void);

    //double getJointErrorAngle(void);

    double getOmegaOverload(void);

    double getAlphaOverload(void);

    double getSmoothRadiusCoefficient(void);

    CurveMode getCurveMode(void);

    const JointConstraint& getJointConstraint(void);

    const DHGroup& getDH(void);

    const Transformation& getToolFrame(void);

    const Transformation& getUserFrame(void);

    void clearMotionList(void);

    //int getFIFOLength(void);

    unsigned int getTrajectorySegmentLength(void);

    int getAllCommandLength(void);

    bool setTrajectorySegmentLength(int length);

    //------------------------------------------------------------
    // Function:    setCycleTime
    // Summary: To set cycle time of interpolation algorithm, default value: 0.001.
    // In:      cycle_time -> desired cycle time
    // Out:     None
    // Return:  true    -> cycle time setted to given value
    //          false   -> cycle time NOT changed
    //------------------------------------------------------------
    bool setCycleTime(double cycle_time);

    //bool setVelocity(double vel);

    //------------------------------------------------------------
    // Function:    setAcceleration
    // Summary: To change acceleration in cartesian space.
    // In:      acce    -> desired acceleration
    // Out:     None
    // Return:  true    -> acceleration changed to given value
    //          false   -> acceleration NOT changed
    //------------------------------------------------------------
    bool setAcceleration(double acce);

    bool setVelocityScaling(double percent);

    bool setAccelerationScaling(double percent);

    bool setJerk(double jerk);

    bool setJointOvershoot(double angle);

    //bool setJointErrorAngle(double angle);

    //bool setOmegaOverload(double value);

    //bool setAlphaOverload(double value);

    bool setSmoothRadiusCoefficient(double coeff);

    void setCurveMode(CurveMode mode);

    //------------------------------------------------------------------------------
    // Function:    setJointConstraint
    // Summary: To set joint constraint in Kinematics algorithm.
    // In:      constraint -> joint constraint;
    // Out:     None
    // Return:  true    -> joint constraint changed to given value
    //          false   -> joint constraint NOT changed
    //------------------------------------------------------------------------------
    void setJointConstraint(const JointConstraint &constraint);

    bool setAlphaScaling(double percent);

    void setDH(const DHGroup &dh);

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

    void setInitialJoint(const Joint &joint);

    void displayMotionList(void);

    void setPickPosition(const JointPoint &point);

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

    bool getJointFromPose(const Pose &pose,
                          const Joint &reference,
                          Joint &joint,
                          double time_interval,
                          ErrorCode &err);

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
                                  const Joint &joint_reference,
                                  Joint &joint_result,
                                  ErrorCode &err);

    //------------------------------------------------------------------------------
    // Function:    computeForwardKinematics
    // Summary: To compute FK with given joint values.
    // In:      joint_result -> joint values needed to compute FK
    // Out:     poes         -> FK result
    //          error_code   -> error code
    // Return:  true         -> FK computed successfully
    //------------------------------------------------------------------------------
    bool computeForwardKinematics(const Joint &joint, Pose &pose, ErrorCode &err);

    MoveCommand* createMotionCommand(int id, MotionTarget &target, MotionTarget &next, ErrorCode &err);

    void deleteFirstMotionCommand(void);

    void deleteLastMotionCommand(void);

    void deleteMotionCommandBefore(MoveCommand *command);

    void deleteMotionCommand(MoveCommand *command);

    bool isMotionCommandListEmpty(void);
    
    ErrorCode pickPoints(vector<PathPoint> &points);

    int estimateFIFOLength(Joint joint1, Joint joint2);

    bool replanPauseTrajectory(std::vector<JointPoint> &traj, JointPoint &joint_end);

    bool resumeFromPause(const Joint &pause_joint, const JointPoint &point, vector<JointPoint> &traj, ErrorCode &err);

    bool resumeFromEstop(const JointPoint &point, vector<JointPoint> &traj, ErrorCode &err);

    bool isMotionExecutable(MotionType motion_t);
    bool isPointCoincident(const Pose &pose1, const Pose &pose2);
    bool isPointCoincident(const Pose &pose, const Joint &joint);
    bool isPointCoincident(const Joint &joint, const Pose &pose);
    bool isPointCoincident(const Joint &joint1, const Joint &joint2);

    bool checkMotionTarget(const MotionTarget &target);
    bool checkJointBoundry(const Joint &joint);
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

    double      jerk_;
    double      joint_overshoot_;
    //double      joint_errorangle_;
    double      omega_overload_;
    double      alpha_overload_;
    double      smooth_radius_coefficient_;
    CurveMode   curve_mode_;
    SmoothMode  smooth_mode_;
    int         trajectory_segment_length_;

    JointConstraint joint_constraint_;
    DHGroup dh_parameter_;

    Transformation tool_frame_;
    Transformation user_frame_;

    // ultimate values
    struct {double min; double max;}
        trajectory_segment_length_range_,
        smooth_radius_coefficient_range_,
        acceleration_scaling_range_,
        velocity_scaling_range_,
        //joint_errorangle_range_,
        joint_overshoot_range_,
        alpha_overload_range_,
        omega_overload_range_,
        acceleration_range_,
        cycle_time_range_,
        velocity_range_,
        jerk_range_;

    bool  initial_joint_valid_;
    Joint initial_joint_;

    TrajPlan    *planner_;
    MoveCommand *motion_list_front_;
    MoveCommand *motion_list_back_;
    MoveCommand *picking_command_;
};

}


#endif
