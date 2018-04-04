/*************************************************************************
	> File Name: motion_plan_motion_command.h
	> Author:   Feng Yun
	> Mail:     yun.feng@foresight-robotics.com
	> Created Time: 2017/12/13-13:27:32************************************************************************/

#ifndef _MOTION_PLAN_MOTION_COMMAND_H
#define _MOTION_PLAN_MOTION_COMMAND_H

#include <vector>
#include <fst_datatype.h>
#include <motion_plan_error_code.h>

namespace fst_controller
{

struct LinePathCoeff {
    double position_coeff_x;
    double position_coeff_y;
    double position_coeff_z;

    bool   spherical_flag;
    Angle  orientation_angle;
};

// for triangle case:
//      J = coeff1 * t^2                                0 < t < duration_max/2
//      J = coeff2 - coeff1 * (duration_max - t)        duration_max/2 < t < duration_max
// where:
//      coeff1 = acc_max/2
//      coeff2 = velocity_actual * duration_max / 2
// for trapezoid case:
//      J = coeff1 * t^2                                0 < t < acc_duration
//      J = coeff3 + velocity_actual * t                acc_duration < t < duration_max - acc_duration
//      J = coeff4 - coeff1 * (duration_max - t)^2      duration_max - acc_duration < t < duration_max
//where:
//      coeff3 = coeff1 * acc_duration^2 - velocity_actual * acc_duration
//      coeff4 = velocity_actual * (duration_max - acc_duration)
struct JointPathCoeff{
    double coeff1;
    double coeff2;
    double coeff3;
    double coeff4;
};

class MotionCommand
{
////////////////////////////////////////////////////////////////////////////////////////////////////
  public:
    //------------------------------------------------------------------------------
    // Function:    MotionCommand
    // Summary: Constructor of class MotionCommand.
    // In:      *log  -> a ptr of log client
    // Out:     None
    // Return:  None
    //------------------------------------------------------------------------------
    MotionCommand(void);

    //------------------------------------------------------------------------------
    // Function:    setTarget
    // Summary: Set motion target.
    // In:      target -> the target point of this command
    //          id      -> the id of this command
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------------------------
    ErrorCode setTarget(const MotionTarget &target, int id);

    //------------------------------------------------------------------------------
    // Function:    setStartJoint
    // Summary: Set motion start from given joint.
    // In:      joint   -> where to start this motion
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------------------------
    ErrorCode setStartJoint(const Joint &joint);

    //------------------------------------------------------------------------------
    // Function:    setTarget
    // Summary: Set motion target.
    // In:      target1 -> the target point of this command
    //          target2 -> the target point of the next command (pre-read)
    //          id      -> the id of this command
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------------------------
    //ErrorCode setTarget(const MotionTarget &target1, const MotionTarget &target2, int id);

    //------------------------------------------------------------------------------
    // Function:    planPath
    // Summary: Plan a path to get the point given through the constructor.
    // In:      None
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------------------------
    ErrorCode planPath(void);

    //------------------------------------------------------------------------------
    // Function:    planPathAndTrajectory
    // Summary: Plan a MOVJ path and trajectory to get the point given through the constructor.
    // In:      None
    // Out:     traj_path  ->  trajectory output fifo
    //          traj_head  ->  trajectory fifo head index
    //          traj_tail  ->  trajectory fifo tail index
    // Return:  error code
    //------------------------------------------------------------------------------
    ErrorCode planPathAndTrajectory(ControlPoint* traj_path, size_t& traj_head, size_t& traj_tail);    

    //------------------------------------------------------------------------------
    // Function:    pickPathPoint
    // Summary: Pick out point on the path using given stamp.
    // In:      None
    // Out:     point  -> point picked out from this command
    // Return:  error code
    //------------------------------------------------------------------------------
    ErrorCode  pickPathPoint(Tick stamp, PathPoint &point);

    //------------------------------------------------------------------------------
    // Function:    pickAllPoint
    // Summary: Pick all of the points on the path.
    // In:      None
    // Out:     points  -> point picked out from this command
    // Return:  error code
    //------------------------------------------------------------------------------
    ErrorCode pickAllPoint(std::vector<PathPoint> &points);

    //------------------------------------------------------------------------------
    // Function:    pickPathPoint
    // Summary: Pick points on the path from this command, with given number of points.
    // In:      num     -> number of points want to pick from the command
    // Out:     points  -> points picked out from the command
    // Return:  error code
    //------------------------------------------------------------------------------
    ErrorCode pickPathPoint(size_t num, std::vector<PathPoint> &points);

    //------------------------------------------------------------------------------
    // Function:    getMotionID
    // Summary: To get id of this command.
    // In:      None
    // Out:     None
    // Return:  id of this command
    //------------------------------------------------------------------------------
    int getMotionID(void);

    //------------------------------------------------------------------------------
    // Function:    getPathLength
    // Summary: To get length of the path from this command, length = MAX_STAMP.
    // In:      None
    // Out:     None
    // Return:  path length of this command
    //------------------------------------------------------------------------------
    size_t getPathLength(void);

    //------------------------------------------------------------------------------
    // Function:    getNextStamp
    // Summary: To get the next stamp that being picked out from this command.
    // In:      None
    // Out:     None
    // Return:  Next stamp being picked out in the futrue
    //------------------------------------------------------------------------------
    Tick getNextStamp(void);

    //------------------------------------------------------------------------------
    // Function:    setNextStamp
    // Summary: To set the next stamp that being picked out in the future.
    //          in next time.
    // In:      stamp -> stamp of the first point picked out in the future
    // Out:     None
    // Return:  true  -> next stamp reset to given stamp successfully
    //          false -> next stamp unchanged, cause of INVALID given stamp
    //------------------------------------------------------------------------------
    bool setNextStamp(Tick stamp);

    //------------------------------------------------------------------------------
    // Function:    isCommandPlanned
    // Summary: To see if this command is planned already.
    // In:      None
    // Out:     None
    // Return:  true  -> this command has been planned before
    //          false -> this command hasn't been planned yet
    //------------------------------------------------------------------------------
    bool isCommandPlanned(void);

    //------------------------------------------------------------------------------
    // Function:    isCommandFinished
    // Summary: To see if all of points in this command have been picked out.
    // In:      None
    // Out:     None
    // Return:  true  -> all of points have been picked out
    //          false -> some of points havn't been picked
    //------------------------------------------------------------------------------
    bool isCommandFinished(void);

    //------------------------------------------------------------------------------
    // Function:    getPathBeginning
    // Summary: To get a copy of the beginning point of this command.
    // In:      None
    // Out:     None
    // Return:  copy of the beginning piont
    //------------------------------------------------------------------------------
    //PathPoint getPathBeginning(void);

    //------------------------------------------------------------------------------
    // Function:    getJointEnding
    // Summary: To get a copy of the ending point of this command.
    // In:      None
    // Out:     None
    // Return:  copy of the ending piont
    //------------------------------------------------------------------------------
    Joint getJointEnding(void);

    //------------------------------------------------------------------------------
    // Function:    getCartesianEnding
    // Summary: To get a copy of the ending point of this command.
    // In:      None
    // Out:     None
    // Return:  copy of the ending piont
    //------------------------------------------------------------------------------
    Pose getCartesianEnding(void);

    //------------------------------------------------------------------------------
    // Function:    getMotionType
    // Summary: To get the motion type of this command.
    // In:      None
    // Out:     None
    // Return:  this commnad's motion type
    //------------------------------------------------------------------------------
    MotionType getMotionType(void);

    //------------------------------------------------------------------------------
    // Function:    getSmoothType
    // Summary: To get the smooth type of this command.
    // In:      None
    // Out:     None
    // Return:  this commnad's smooth type
    //------------------------------------------------------------------------------
    SmoothType getSmoothType(void);

    //------------------------------------------------------------------------------
    // Function:    getCommandVelocity
    // Summary: To get the velocity given by the command.
    // In:      None
    // Out:     None
    // Return:  this commnad's target velocity
    //------------------------------------------------------------------------------
    double getCommandVelocity(void);

    //------------------------------------------------------------------------------
    // Function:    getCommandAcc
    // Summary: To get the accelaration given by the command.
    // In:      None
    // Out:     None
    // Return:  this commnad's target accelaration expressed in percent
    //------------------------------------------------------------------------------
    double getCommandAcc(void);

    //------------------------------------------------------------------------------
    // Function:    getPrevCommandPtr
    // Summary: Get a pointer to the previous command.
    // In:      None
    // Out:     None
    // Return:  pointer to the previous command
    //------------------------------------------------------------------------------
    MotionCommand* getPrevCommandPtr(void);

    //------------------------------------------------------------------------------
    // Function:    getNextCommandPtr
    // Summary: Get a pointer to the next command.
    // In:      None
    // Out:     None
    // Return:  pointer to the next command
    //------------------------------------------------------------------------------
    MotionCommand* getNextCommandPtr(void);

    //------------------------------------------------------------------------------
    // Function:    setPrevCommandPtr
    // Summary: Set the prev-pointer, point to the previous command.
    // In:      ptr     -> pointer to the previous command
    // Out:     None
    // Return:  None
    //------------------------------------------------------------------------------
    void setPrevCommandPtr(MotionCommand *ptr);

    //------------------------------------------------------------------------------
    // Function:    setNextCommandPtr
    // Summary: Set the next-pointer, point to the next command.
    // In:      ptr     -> pointer to the next command
    // Out:     None
    // Return:  None
    //------------------------------------------------------------------------------
    void setNextCommandPtr(MotionCommand *ptr);


////////////////////////////////////////////////////////////////////////////////////////////////////
  private:
    //------------------------------------------------------------------------------
    // Function:    planJointPath
    // Summary: To plan a joint path from starting-joint to ending-joint.
    // In:      None
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------------------------
    ErrorCode planJointPath(void);

    //------------------------------------------------------------------------------
    // Function:    planLinePath
    // Summary: To plan a line path from starting-pose to ending-pose.
    // In:      None
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------------------------
    ErrorCode planLinePath(void);

    //------------------------------------------------------------------------------
    // Function:    interpolateJoint
    // Summary: To interpolate a pose in the joint path planned before.
    // In:      stamp -> to tell which pose you want to pickout from this path
    // Out:     pose  -> pose in the path which pointed out by given stamp
    // Return:  error code
    //------------------------------------------------------------------------------
    void interpolateJoint(Tick stamp, Joint &joint);

    //------------------------------------------------------------------------------
    // Function:    interpolateLine
    // Summary: To interpolate a pose in the line path planned before.
    // In:      stamp -> to tell which pose you want to pickout from this path
    // Out:     pose  -> pose in the path which pointed out by given stamp
    // Return:  error code
    //------------------------------------------------------------------------------
    void interpolateLine(Tick stamp, Pose &pose);

    int         motion_id_;
    MotionType  motion_type_;
    SmoothType  smooth_type_;

    Joint       beginning_joint_;

    Joint       target_joint_;
    Pose        target_pose1_;
    Pose        target_pose2_;
    
    Joint       next_joint_;
    Pose        next_pose1_;
    Pose        next_pose2_;

    double      cnt_, vel_, acc_;
    double      next_cnt_, next_vel_, next_acc_;

    MotionCommand *prev_ptr_;
    MotionCommand *next_ptr_;

    Tick    max_stamp_, pick_stamp_;
    
    bool    is_planned_;
    bool    begin_from_given_joint_;

    //----- MOVE JOINT --------------------------------
    Joint joint_starting_;
    Joint joint_ending_;
    double joint_coeff_[AXIS_IN_ALGORITHM];

    //----- MOVE LINE --------------------------------
    Pose    pose_starting_;
    Pose    pose_ending_;
    LinePathCoeff line_coeff_;
};

}

#endif
