/*************************************************************************
	> File Name: motion_plan_arm_group.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年12月11日 星期一 09时14分31秒
 ************************************************************************/

#include <motion_plan_arm_group.h>

namespace fst_controller
{

static int gs_count = 0;
static int is_paused = false;

ArmGroup::ArmGroup()
{
}
ArmGroup::~ArmGroup()
{
}

    ErrorCode ArmGroup::initArmGroup(){return 0;}

    std::string ArmGroup::getVersion(void){}

    double ArmGroup::getCycleTime(void){}
    //------------------------------------------------------------
    //------------------------------------------------------------

    //------------------------------------------------------------
    // Function:    getCartesianAccMax
    // Summary: To get max acceleration in cartesian space.
    // In:      None
    // Out:     None
    // Return:  value of max acceleration in cartesian space
    //------------------------------------------------------------
    double ArmGroup::getCartesianAccMax(void){}

    /*
    //------------------------------------------------------------
    // Function:    setCartesianAccMax
    // Summary: To set max acceleration in cartesian space.
    // In:      acc -> max acceleration in cartesian space
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setCartesianAccMax(double acc){}
    */

    //------------------------------------------------------------
    // Function:    getCartesianVelMax
    // Summary: To get max velocity in cartesian space.
    // In:      None
    // Out:     None
    // Return:  value of max velocity in cartesian space
    //------------------------------------------------------------
    double ArmGroup::getCartesianVelMax(void){}

    int ArmGroup::getLatestCommandLength(void){}

    /*
    //------------------------------------------------------------
    // Function:    setCartesianVelMax
    // Summary: To set max velocity in cartesian space.
    // In:      acc -> max velocity in cartesian space
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode setCartesianVelMax(double vel){}
    */

    //------------------------------------------------------------
    // Function:    getGlobalVelRatio
    // Summary: To get global velocity ratio in cartesian and joint space.
    // In:      None
    // Out:     None
    // Return:  global velocity ratio in cartesian space
    //          range: 0.0 ~ 1.0
    //------------------------------------------------------------
    double ArmGroup::getGlobalVelRatio(void){}

    //------------------------------------------------------------
    // Function:    setGlobalVelRatio
    // Summary: To set global velocity ratio in cartesian and joint space.
    // In:      ratio -> global velocity ratio in cartesian space
    //                   range: 0.0 ~ 1.0
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::setGlobalVelRatio(double ratio){return 0;}

    //------------------------------------------------------------
    // Function:    getGlobalAccRatio
    // Summary: To get global acceleration ratio in cartesian and joint space.
    // In:      None
    // Out:     None
    // Return:  global acceleration ratio in cartesian space
    //          range: 0.0 ~ 3.0
    //------------------------------------------------------------
 //   double ArmGroup::getGlobalAccRatio(void){}

    //------------------------------------------------------------
    // Function:    setGlobalAccRatio
    // Summary: To set global acceleration ratio in cartesian and joint space.
    // In:      ratio -> global acceleration ratio in cartesian space
    //                   range: 0.0 ~ 3.0
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
 //   ErrorCode ArmGroup::setGlobalVelRatio(double ratio){}

    //------------------------------------------------------------
    // Function:    getCartesianAcc
    // Summary: To get acceleration in cartesian space.
    // In:      None
    // Out:     None
    // Return:  cartesian acc, unit: mm/(s*s)
    //------------------------------------------------------------
    double ArmGroup::getCartesianAcc(void){}

    //------------------------------------------------------------
    // Function:    setCartesianAcc
    // Summary: To set acceleration in cartesian space.
    // In:      acc -> cartesian acceleration, unit: mm/(s*s)
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::setCartesianAcc(double acc){return 0;}

    //------------------------------------------------------------
    // Function:    getJerk
    // Summary: To get jerk ratio.
    // In:      None
    // Out:     None
    // Return:  jerk ratio
    //------------------------------------------------------------
    double ArmGroup::getJerk(void){}

    //------------------------------------------------------------
    // Function:    getCurveMode
    // Summary: To get velocity curve mode.
    // In:      None
    // Out:     None
    // Return:  curve mode in current:
    //            T_CURVE -> Trapezoidal curve
    //            S_CURVE -> S-Shape curve
    //------------------------------------------------------------
    const CurveMode& ArmGroup::getCurveMode(void){}

    //------------------------------------------------------------
    // Function:    getSoftConstraint
    // Summary: To get soft joint constraint from algorithm.
    // In:      None
    // Out:     None
    // Return:  soft constraint
    //------------------------------------------------------------
    const JointConstraint& ArmGroup::getSoftConstraint(void){}

    //------------------------------------------------------------
    // Function:    setSoftConstraint
    // Summary: To set soft joint constraint to algorithm and config file.
    // In:      cons -> soft joint constraint
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::setSoftConstraint(const JointConstraint &cons){return 0;}

    //------------------------------------------------------------
    // Function:    getHardConstraint
    // Summary: To get hard joint constraint from algorithm.
    // In:      None
    // Out:     None
    // Return:  hard constraint
    //------------------------------------------------------------
    const JointConstraint& ArmGroup::getHardConstraint(void){}

    //------------------------------------------------------------
    // Function:    getDH
    // Summary: To get DH parameter group from algorithm.
    // In:      None
    // Out:     None
    // Return:  DH parameter group
    //------------------------------------------------------------
    const DHGroup& ArmGroup::getDH(void){}

/*
    //------------------------------------------------------------
    // Function:    setDH
    // Summary: To set DH parameter group to algorithm and config file.
    // In:      dh -> DH parameter group
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void setDH(const DHGroup &dh){}
*/

    //------------------------------------------------------------
    // Function:    getFIFOLength
    // Summary: To get the length of trajectory FIFO.
    // In:      None
    // Out:     None
    // Return:  length of the FIFO
    //------------------------------------------------------------
    int ArmGroup::getFIFOLength(void)
    {
        if (!is_paused)
            gs_count--;
        if (gs_count < 0)
            gs_count = 0;
        return gs_count;
    }

    //------------------------------------------------------------
    // Function:    getFIFOCapacity
    // Summary: To get the capacity of trajectory FIFO.
    // In:      None
    // Out:     None
    // Return:  capacity of the FIFO
    //------------------------------------------------------------
    int ArmGroup::getFIFOCapacity(void){}

    //------------------------------------------------------------
    // Function:    getPointFromFIFO
    // Summary: To get points from trajectory FIFO.
    // In:      num -> number of joint points want to get
    // Out:     points -> points from trajectory FIFO
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::getPointFromFIFO(std::vector<JointPoint> &points){return 0;}

    //------------------------------------------------------------
    // Function:    getPointFromFIFO
    // Summary: To get points from trajectory FIFO.
    // In:      num -> number of joint points want to get
    // Out:     points -> points from trajectory FIFO
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::getPointFromFIFO(int &num, JointPoint *points){return 0;}

    //------------------------------------------------------------
    // Function:    pickPointToFIFO
    // Summary: To pick points from motion command and put the points 
    //          into trajectory FIFO.
    // In:      num -> number of points want to pick
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::pickPointToFIFO(int num){return 0;}

    //------------------------------------------------------------
    // Function:    getToolFrame
    // Summary: To get current tool frame in algorithm.
    // In:      None
    // Out:     None
    // Return:  current tool frame
    //------------------------------------------------------------
    const Transformation& ArmGroup::getToolFrame(void){}

    //------------------------------------------------------------
    // Function:    setToolFrame
    // Summary: To set current tool frame.
    // In:      frame -> current tool frame
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void ArmGroup::setToolFrame(const Transformation &frame){}

    //------------------------------------------------------------
    // Function:    getUserFrame
    // Summary: To get current user frame in algorithm.
    // In:      None
    // Out:     None
    // Return:  current user frame
    //------------------------------------------------------------
    const Transformation& ArmGroup::getUserFrame(void){}

    //------------------------------------------------------------
    // Function:    setUserFrame
    // Summary: To set current user frame.
    // In:      frame -> current user frame
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void ArmGroup::setUserFrame(const Transformation &frame){}

    //------------------------------------------------------------
    // Function:    setStartState
    // Summary: To set robot start state.
    // In:      joint -> robot start state
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::setStartState(const Joint &joint){return 0;}

    //------------------------------------------------------------
    // Function:    getJointFromPose
    // Summary: To compute IK with a given pose in cartesian space,
    //          without reference joint and accessibility check.
    // In:      poes    -> the pose in cartesian space needed to compute IK
    // Out:     joint   -> IK result
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::getJointFromPose(const PoseEuler &pose, Joint &joint){return 0;}

    //------------------------------------------------------------
    // Function:    getPoseFromJoint
    // Summary: To compute FK with a given point in joint space.
    // In:      joint   -> the point in joint space needed to compute FK.
    // Out:     pose    -> FK result
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::getPoseFromJoint(const Joint &joint, PoseEuler &pose){return 0;}

    //------------------------------------------------------------
    // Function:    getPoseFromJointInWorld
    // Summary: To get the pose of flange and tcp from a given point
    //          in joint space in world coordinate.
    // In:      joint   -> the point in joint space needed to compute FK.
    // Out:     flange  -> flange pose in world coordinate
    //          tcp     -> tcp pose in world coordinate
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::getPoseFromJointInWorld(const Joint &joint, PoseEuler &flange, PoseEuler &tcp){return 0;}

    //------------------------------------------------------------
    // Function:    suspendMotion
    // Summary: To replan a slow-down trajectory upon trajectory FIFO
    //          and stop the robot motion. Used when pause event or IK
    //          failure raised etc.
    // In:      None
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::suspendMotion(void){
        is_paused = true;
        return 0;}

    //------------------------------------------------------------
    // Function:    declareESTOP
    // Summary: Declare an ESTOP event to arm_group, this function
    //          should be called after the robot is stopped.
    // In:      joint   -> stopped robot is standing on this joint
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::declareESTOP(/*const Joint &joint*/){return 0;}

    //------------------------------------------------------------
    // Function:    resumeMotion
    // Summary: To replan a start-up trajectory and resume robot
    //          motion from suspend or ESTOP state.
    // In:      None
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::resumeMotion(void){
        is_paused = false;
        return 0;}

    //------------------------------------------------------------
    // Function:    resetArmGroup
    // Summary: To reset resources in arm group.
    // In:      None
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::resetArmGroup(){return 0;}
    ErrorCode ArmGroup::clearArmGroup(void){
        gs_count = 0;
        is_paused = false;
        return 0;}

    //------------------------------------------------------------
    // Function:    isJointInConstraint
    // Summary: Retrun true if given joint fall into the soft constraint
    // In:      joint -> joint to be compared with soft constraint
    // Out:     None
    // Return:  true  -> joint in soft constraint
    //          false -> joint not in soft constraint
    //------------------------------------------------------------
    bool ArmGroup::isJointInSoftConstraint(const Joint &joint){}

    //------------------------------------------------------------
    // Function:    autoMove
    // Summary: Plan an auto move command (moveJ/moveL/moveC) to reach
    //          the target without smooth. If FIFO is empty at the
    //          moment, then fill the FIFO with points from this command.
    // In:      target -> motion target
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::autoMove(const MotionTarget &target, int id){
        printf("auto move...");
        gs_count = 1000;
        return 0;}

    //------------------------------------------------------------
    // Function:    manualMove
    // Summary: Plan a manual move trajectory (Joint/Line) with given
    //          direction. If FIFO is empty at the moment, fill it.
    // In:      button -> manual direction
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::manualMove(const std::vector<ManualDirection> &button){return 0;}

    //------------------------------------------------------------
    // Function:    manualMove
    // Summary: Plan a manual move trajectory (Joint/Line) with given
    //          point. If FIFO is empty at the moment, then fill it.
    // In:      joint -> manual target in joint space
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::manualMove(const Joint &joint){return 0;}

    //------------------------------------------------------------
    // Function:    manualMove
    // Summary: Plan a manual move trajectory (Joint/Line) with given
    //          point. If FIFO is empty at the moment, then fill it.
    // In:      pose  -> manual target in cartesian space
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::manualMove(const PoseEuler &pose){return 0;}

    //------------------------------------------------------------
    // Function:    setManualFrameMode
    // Summary: To set manual frame mode.
    // In:      frame   -> manual frame mode, JOINT/WORLD/USER/TOOL
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void ArmGroup::setManualFrameMode(ManualFrameMode frame){}

    //------------------------------------------------------------
    // Function:    setManualMotionMode
    // Summary: To set manual motion mode.
    // In:      motion  -> manual motion mode, STEP/CONTINUOUS/POINT
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    void ArmGroup::setManualMotionMode(ManualMotionMode mode){}

    //------------------------------------------------------------
    // Function:    setManualJointStepLength
    // Summary: To set step length in manual joint step working mode.
    // In:      step    -> step length, unit: rad
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::setManualJointStepLength(double step){return 0;}

    //------------------------------------------------------------
    // Function:    setManualCartesianStepLength
    // Summary: To set step length in manual cartesian step working mode.
    // In:      step    -> step length, unit: mm
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::setManualCartesianStepLength(double step){return 0;}

    //------------------------------------------------------------
    // Function:    getManualMaxSpeedRatio
    // Summary: To get manual max speed ratio to max-auto-speed.
    // In:      None
    // Out:     None
    // Return:  manual max speed ratio, range: 0.0 ~ 1.0
    //------------------------------------------------------------
    double ArmGroup::getManualMaxSpeedRatio(void){}


    //------------------------------------------------------------
    // Function:    getManualSpeedRatio
    // Summary: To get manual speed ratio to max-manual-speed.
    // In:      None
    // Out:     None
    // Return:  manual speed ratio, range: 0.0 ~ 1.0
    //------------------------------------------------------------
    double ArmGroup::getManualSpeedRatio(void){}

    //------------------------------------------------------------
    // Function:    setManualSpeedRatio
    // Summary: To set manual speed ratio to max-manual-speed.
    // In:      ratio   -> manual speed to max-manual-speed
    //                      range: 0.0 ~ 1.0
    // Out:     None
    // Return:  error code
    //------------------------------------------------------------
    ErrorCode ArmGroup::setManualSpeedRatio(double ratio){return 0;}

}




