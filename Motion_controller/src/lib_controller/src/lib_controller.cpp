/**********************************************************************
    Copyright:  Foresight-Robotics
    File:       fst_controller.cpp
    Author:     Feng Yun
    Data:       Aug.1  2016
    Modify:     Aug.30 2016
    Description:ArmGroup--Source code.
**********************************************************************/
			

#include <lib_controller/lib_controller.h>
#include <ros/ros.h>


//-----------------------------public functions--------------------------------------------


//------------------------------------------------------------
// Function:    ArmGroup
// Summary: The constructor of class
// In:      joint_values -> initial state of all joints
// Out:     error_code   -> error code
// Return:  None
//------------------------------------------------------------
fst_controller::ArmGroup::ArmGroup( const JointValues &joint_values,
                                    ErrorCode &error_code)
{
    ROS_INFO("Initializing ArmGroup...");

    m_VMAX = 4000;
    m_AMAX = 48000;
    m_CYCLE_TIME_UPPER = 0.1;
    m_CYCLE_TIME_LOWER = 0.0005;
    m_JOINT_FIFO_LEN = 10000;

    error_code = Error::Success;

    m_planner = new fst_controller::TrajPlan();
    if(m_planner==NULL) {
        error_code = Error::Fail_Init_Algerithm;
        ROS_ERROR("Can not construct algorithm space, error code=%d", error_code);
        return;
    }

    setCycleTime(0.001);

    setMaxVelocity(m_VMAX);
    setMaxAcceleration(m_AMAX);
    setVelocityScalingFactor(1.0);
    setAccelerationScalingFactor(1.0);
    setJumpThresholdScalingFactor(1.0);

    ROS_INFO("Loading joint constraints from: ");
    if(!loadJointConstraints()) {
        ROS_ERROR("Can not load joint constraints from: , using default values");
        error_code = Error::Fail_Load_Constraint;
        ROS_ERROR("Error code: %d",error_code);
        //return;
    }

    if(setCurrentJointValues(joint_values,error_code)) {
        m_pose_start = getCurrentPose();
        m_pose_previous = m_pose_start;
	m_vu_start = 0.0;
	m_v_start = 0.0;
        ROS_INFO("Current joint: %f,%f,%f,%f,%f,%f",
                joint_values.j1, joint_values.j2,
                joint_values.j3, joint_values.j4,
                joint_values.j5, joint_values.j6);
        m_latest_ik_reference = joint_values;
        m_joint_start.joints = joint_values;
        m_joint_start.omegas.j1 = 0;
        m_joint_start.omegas.j2 = 0;
        m_joint_start.omegas.j3 = 0;
        m_joint_start.omegas.j4 = 0;
        m_joint_start.omegas.j5 = 0;
        m_joint_start.omegas.j6 = 0;
        m_point_level = enum_Level_Start;
        m_next_cmd_type = enum_Type_Other;
    }
    else {
        ROS_ERROR("Fail to init start joint values: %f,%f,%f,%f,%f,%f",
                joint_values.j1, joint_values.j2,
                joint_values.j3, joint_values.j4,
                joint_values.j5, joint_values.j6);
        ROS_ERROR("Error code: %d",error_code);
        return;
    }

    ROS_INFO("Constructing planned_path_FIFO and joint_trajectory_FIFO ...");
    m_planned_path_fifo.reserve(5000);
    m_joint_trajectory_fifo.reserve(200);
    m_joint_trajectory_FIFO_islocked = false;

    ROS_INFO("ArmGroup is ready to make life easier. Have a good time!");
    ROS_INFO("**********************************************************************************************");
}


//------------------------------------------------------------
// Function:    ~ArmGroup
// Summary: The destructor of class
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
fst_controller::ArmGroup::~ArmGroup(void)
{
    delete m_planner;
}


//------------------------------------------------------------
// Function:    getCycleTime
// Summary: To get cycle time of interpolation algorithm.
// In:      None
// Out:     None
// Return:  cycle time
//------------------------------------------------------------
double fst_controller::ArmGroup::getCycleTime(void)
{
    return m_cycle_time;
}


//------------------------------------------------------------
// Function:    getJointConstraints
// Summary: To get joint constraints from Kinematics algorithm.
// In:      None
// Out:     None
// Return:  joint constraints
//------------------------------------------------------------
const fst_controller::JointConstraints& fst_controller::ArmGroup::getJointConstraints(void)
{
    return m_joint_constraints;
}


//------------------------------------------------------------
// Function:    getMaxVelocity
// Summary: To get max velocity settings.
// In:      None
// Out:     None
// Return:  value of max velocity
//------------------------------------------------------------
double fst_controller::ArmGroup::getMaxVelocity(void)
{
    return m_vmax;
}


//------------------------------------------------------------
// Function:    getMaxAcceleration
// Summary: To get max acceleration settings.
// In:      None
// Out:     None
// Return:  value of max acceleration
//------------------------------------------------------------
double fst_controller::ArmGroup::getMaxAcceleration(void)
{
    return m_amax;
}


//------------------------------------------------------------
// Function:    getVelocityScalingFactor
// Summary: To get velocity scaling factor value.
// In:      None
// Out:     None
// Return:  global scaling factor for velocity
//------------------------------------------------------------
double fst_controller::ArmGroup::getVelocityScalingFactor(void)
{
    return m_vmax_scaling_factor;
}


//------------------------------------------------------------
// Function:    getAccelerationScalingFactor
// Summary: To get acceleration scaling factor value.
// In:      None
// Out:     None
// Return:  global scaling factor for acceleration
//------------------------------------------------------------
double fst_controller::ArmGroup::getAccelerationScalingFactor(void)
{
    return m_amax_scaling_factor;
}


//------------------------------------------------------------
// Function:    getCurrentJointValues
// Summary: To get current values of all joints in the robot.
// In:      None
// Out:     None
// Return:  current values of all six joints
//------------------------------------------------------------
const fst_controller::JointValues& fst_controller::ArmGroup::getCurrentJointValues(void)
{
    return m_joint_state_current;
}


//------------------------------------------------------------
// Function:    getCurrentPose
// Summary: To get current pose of endpoint in the arm group.
// In:      None
// Out:     None
// Return:  current pose of the endpoint
//------------------------------------------------------------
const fst_controller::Pose& fst_controller::ArmGroup::getCurrentPose(void)
{
    return m_pose_state_current;
}


//------------------------------------------------------------
// Function:    getPlannedPathFIFOLength
// Summary: To get the length of planned_path_FIFO.
// In:      None
// Out:     None
// Return:  length of the FIFO
//------------------------------------------------------------
int fst_controller::ArmGroup::getPlannedPathFIFOLength(void)
{
    return m_planned_path_fifo.size();
}


//------------------------------------------------------------
// Function:    getJointTrajectoryFIFOLength
// Summary: To get the length of joitn_trajectory_FIFO.
// In:      None
// Out:     None
// Return:  length of the FIFO
//------------------------------------------------------------
int fst_controller::ArmGroup::getJointTrajectoryFIFOLength(void)
{
    return m_joint_trajectory_fifo.size();
}


//------------------------------------------------------------
// Function:    getJointTrajectoryFIFOIsLocked
// Summary: To get joitn_trajectory_FIFO lock state.
// In:      None
// Out:     None
// Return:  true  -> FIFO is locked
//	    false -> FIFO is not locked
//------------------------------------------------------------
bool fst_controller::ArmGroup::getJointTrajectoryFIFOIsLocked(void)
{
    return m_joint_trajectory_FIFO_islocked;
}


//------------------------------------------------------------
// Function:    getPointsFromJointTrajectoryFIFO
// Summary: To get points from joitn_trajectory_FIFO.
// In:      num  -> number of joint points want to get
// Out:     traj -> joint trajectory consisting of joint points
//	    error_code -> error code
// Return:   <0	 -> joint_trajectory_fifo locked, or any other errors
//	    >=0  -> number of joint points get actually
//------------------------------------------------------------
int fst_controller::ArmGroup::getPointsFromJointTrajectoryFIFO(	std::vector<JointPoint> &traj,
								int num, ErrorCode &error_code)
{
    error_code = Error::Success;
    if(num<0) {
	error_code = Error::Cmd_Parameter_Invalid;
	return num-1;
    }
    else if(num==0) {
	return 0;
    }

    if(getJointTrajectoryFIFOIsLocked()) {
	error_code = Error::Trajectory_FIFO_Locked;
	return 0;
    }
    
    int cnt;
    std::vector<fst_controller::JointPoint>::iterator itr=m_joint_trajectory_fifo.begin();
    
    for(cnt=0; cnt<num; ++cnt) {
	if( itr!=m_joint_trajectory_fifo.end() ) {
	    traj.push_back(*itr);
	    ++itr;
	}
	else {
	    error_code = Error::No_Enough_Points;
	    break;
	}
    }

    if(cnt!=0) {
	itr = m_joint_trajectory_fifo.begin();
	m_joint_trajectory_fifo.erase(itr, itr+cnt);
    }
    
    return cnt;
}


//------------------------------------------------------------
// Function:    setCycleTime
// Summary: To set cycle time of interpolation algorithm.
// In:	    tc -> desired cycle time
// Out:     None
// Return:  true  -> cycle time setted to given value
//          false -> cycle time NOT changed
//------------------------------------------------------------
bool fst_controller::ArmGroup::setCycleTime(double tc)
{
    if( tc>m_CYCLE_TIME_UPPER || tc<m_CYCLE_TIME_LOWER )
	return false;
    else {
	m_cycle_time = tc;
	setCycleTime();
	ROS_INFO("Set cycle time to %.4fs",m_cycle_time);
	return true;
    }
}


//------------------------------------------------------------
// Function:	setJointConstraints
// Summary: To set joint constraints in Kinematics algorithm.
// In:	    constraints -> joint constraints
// Out:	    None
// Return:  true  -> set successfully
//	    false -> set UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::setJointConstraints(const JointConstraints constraints)
{
    ROS_INFO("Set joint constraints:");
    ROS_INFO("  Lower limit: %f,%f,%f,%f,%f,%f",constraints.j1.lower,
						constraints.j2.lower,
						constraints.j3.lower,
						constraints.j4.lower,
						constraints.j5.lower,
						constraints.j6.lower);
    ROS_INFO("  Joint home:  %f,%f,%f,%f,%f,%f",constraints.j1.home,
						constraints.j2.home,
						constraints.j3.home,
						constraints.j4.home,
						constraints.j5.home,
						constraints.j6.home);
    ROS_INFO("  Upper limit: %f,%f,%f,%f,%f,%f",constraints.j1.upper,
						constraints.j2.upper,
						constraints.j3.upper,
						constraints.j4.upper,
						constraints.j5.upper,
						constraints.j6.upper);
    ROS_INFO("  Max omega:   %f,%f,%f,%f,%f,%f",constraints.j1.max_omega,
						constraints.j2.max_omega,
						constraints.j3.max_omega,
						constraints.j4.max_omega,
						constraints.j5.max_omega,
						constraints.j6.max_omega);
    ROS_INFO("  Max alpha:   %f,%f,%f,%f,%f,%f",constraints.j1.max_alpha,
						constraints.j2.max_alpha,
						constraints.j3.max_alpha,
						constraints.j4.max_alpha,
						constraints.j5.max_alpha,
						constraints.j6.max_alpha);

    m_joint_constraints = constraints;
    setJointConstraints();
}


//------------------------------------------------------------
// Function:    setMaxVelocity
// Summary: To change max velocity settings.
// In:      max_v -> desired max velocity
// Out:     None
// Return:  true  -> max velocity changed to given value
//          false -> max velocity NOT changed
//------------------------------------------------------------
bool fst_controller::ArmGroup::setMaxVelocity(double max_v)
{
    if( max_v>m_VMAX || max_v<0 )
	return false;
    else {
	m_vmax = max_v;
	ROS_INFO("Set max velocity to %.2fmm/s",m_vmax);
	return true;
    }
}


//------------------------------------------------------------
// Function:    setMaxAcceleration
// Summary: To change max acceleration settings.
// In:      max_a -> desired max acceleration, unit: mm/s*s
// Out:     None
// Return:  true  -> max acceleration changed to given value
//          false -> max acceleration NOT changed
//------------------------------------------------------------
bool fst_controller::ArmGroup::setMaxAcceleration(double max_a)
{
    if( max_a>m_AMAX || max_a<0 )
	return false;
    else {
	m_amax = max_a;
	setMaxAcceleration();
	ROS_INFO("Set max acceleration to %.2fmm/(s*s)",m_amax);
	return true;
    }
}


//------------------------------------------------------------
// Function:    setVelocityScalingFactor
// Summary: To set velocity scaling factor.
// In:      v_factor -> desired velocity scaling factor
// Out:     None
// Return:  true  -> velocity scaling factor changed to given value
//          false -> velocity scaling factor NOT changed
//------------------------------------------------------------
bool fst_controller::ArmGroup::setVelocityScalingFactor(double v_factor)
{
    if( v_factor<0.0 || v_factor>1.0)	
	return false;
    else {
	m_vmax_scaling_factor = v_factor;
	ROS_INFO("Set velocity scaling factor to %.2f%%",m_vmax_scaling_factor*100);
	return true;
    }
}


//------------------------------------------------------------
// Function:    setAccelerationScalingFactor
// Summary: To set acceleration scaling factor.
// In:      a_factor -> desired acceleration scaling factor
// Out:     None
// Return:  true  -> acceleration scaling factor changed to given value
//          false -> acceleration scaling factor NOT changed
//------------------------------------------------------------
bool fst_controller::ArmGroup::setAccelerationScalingFactor(double a_factor)
{
    if( a_factor<0.0 || a_factor>1.0)	
	return false;
    else {
	m_amax_scaling_factor = a_factor;
	setMaxAcceleration();
	ROS_INFO("Set acceleration scaling factor to %.2f%%", m_amax_scaling_factor*100);
	return true;
    }
}


//------------------------------------------------------------
// Function:	setJumpThresholdScalingFactor
// Summary: To set jump threshold scaling factor.
// In:	    j_factor -> desired jump threshold scaling factor
// Out:	    None
// Return:  true  -> jump threshold scaling factor changed to given value
//	    false -> jump threshold scaling factor NOT changed
//------------------------------------------------------------
bool fst_controller::ArmGroup::setJumpThresholdScalingFactor(double j_factor)
{
    if( j_factor<0.0 || j_factor>1.0 )
	return false;
    else {
	m_jump_threshold_scaling = j_factor;
	setJumpThresholdScalingFactor();
	ROS_INFO("Set jump threshold scaling factor to %.2f%%", m_jump_threshold_scaling*100);
	return true;
    }
}


//------------------------------------------------------------
// Function:    setToolFrame
// Summary: To set current tool frame.
// In:      tool_frame -> current tool frame
// Out:     None
// Return:  None
//------------------------------------------------------------
void fst_controller::ArmGroup::setToolFrame(const Transformation &tool_frame)
{
    m_tool_frame = tool_frame;
    m_planner->setToolFrame(m_tool_frame.position,m_tool_frame.orientation);
}


//------------------------------------------------------------
// Function:    setUserFrame
// Summary: To set current user frame.
// In:      user_frame -> current user frame
// Out:     None
// Return:  None
//------------------------------------------------------------
void fst_controller::ArmGroup::setUserFrame(const Transformation &user_frame)
{
    m_user_frame = user_frame;
    m_planner->setUserFrame(m_user_frame.position,m_user_frame.orientation);
}


//------------------------------------------------------------
// Function:    setCurrentJointValues
// Summary: To set current joint values using encoder data. 
//          Current pose values will be updated automaticly.
// In:      current_joint -> joint values from the encoder
// Out:     error_code -> error code
// Return:  true  -> current joint/pose values updated successfully
//          false -> either joint or pose values NOT updated
//------------------------------------------------------------
bool fst_controller::ArmGroup::setCurrentJointValues(const JointValues &current_joint, ErrorCode &error_code)
{
    if(checkJointBoundary(current_joint)) {
	m_joint_state_current = current_joint;
	if(computeFK(m_joint_state_current, m_pose_state_current, error_code))
	    return true;
	else
	    return false;
    }
    else {
	error_code = Error::Joint_Out_Of_Limit;
	ROS_ERROR("setCurrentJOintValues get a joint group out of boundary");
	return false;
    }
}


//------------------------------------------------------------
// Function:    setLatestIKReference
// Summary: To set latest IK reference.
// In:      joint_reference -> new IK reference
// Out:     error_code -> error code
// Return:  true  -> latest IK reference setted to joint_reference scucessfully
//	    false -> failed to set latest IK reference
//------------------------------------------------------------
bool fst_controller::ArmGroup::setLatestIKReference(const JointValues &joint_reference, ErrorCode &error_code)
{
    if(checkJointBoundary(joint_reference)) {
	m_latest_ik_reference = joint_reference;
	//error_code = Error::Success;
	return true;
    }
    else {
	error_code = Error::Joint_Out_Of_Limit;
	return false;
    }
}


//------------------------------------------------------------
// Function:    setStartState
// Summary: To set robot start state.
// In:      joint_start -> robot start state
// Out:     error_code -> error code
// Return:  true  -> robot start state setted to joint_start scucessfully
//	    false -> failed to set robot start state
//------------------------------------------------------------
bool fst_controller::ArmGroup::setStartState(const JointValues &joint_start, ErrorCode &error_code)
{
    if(setCurrentJointValues(joint_start,error_code)) {
        m_pose_start = getCurrentPose();
        m_pose_previous = getCurrentPose();
	m_vu_start = 0.0;
	m_v_start = 0.0;

	m_joint_start.joints = joint_start;
        m_joint_start.omegas.j1 = 0;
        m_joint_start.omegas.j2 = 0;
        m_joint_start.omegas.j3 = 0;
        m_joint_start.omegas.j4 = 0;
        m_joint_start.omegas.j5 = 0;
        m_joint_start.omegas.j6 = 0;
        m_point_level = enum_Level_Start;
        m_next_cmd_type = enum_Type_Other;

	setLatestIKReference(joint_start,error_code);
	//m_latest_ik_reference = joint_values;
    }
    else {
        ROS_ERROR("Fail to init start joint values: %f,%f,%f,%f,%f,%f",
                joint_start.j1, joint_start.j2,
                joint_start.j3, joint_start.j4,
                joint_start.j5, joint_start.j6);
        ROS_ERROR("Error code: %d",error_code);
        return false;
    }
}


//------------------------------------------------------------
// Function:    clearPlannedPathFIFO
// Summary: To clear planned_path_fifo.
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
void fst_controller::ArmGroup::clearPlannedPathFIFO(void)
{
    m_planned_path_fifo.clear();
}


//------------------------------------------------------------
// Function:    clearJointTrajectoryFIFO
// Summary: To clear joint_trajectory_fifo.
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
void fst_controller::ArmGroup::clearJointTrajectoryFIFO(void)
{
    m_joint_trajectory_fifo.clear();
}


//------------------------------------------------------------
// Function:    computeIK
// Summary: To compute IK with a given pose in cartesian space.
// In:      poes -> cartesian space pose needed to compute IK
// Out:     joint_result -> IK result
//	    error_code -> error code
// Return:  true  -> IK solution found
//          false -> IK solution NOT found
//------------------------------------------------------------
bool fst_controller::ArmGroup::computeIK(   const Pose &pose,
					    JointValues &joint_result,
					    ErrorCode &error_code)
{
    return computeInverseKinematics(pose, getLatestIKReference(), joint_result, error_code);
}


//------------------------------------------------------------
// Function:    computeFK
// Summary: To compute FK with given joint values.
// In:      joint_result -> joint values needed to compute FK
// Out:     poes -> FK result
//	    error_code -> error code
// Return:  true  -> FK computed successfully
//          false -> FK computed UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::computeFK(   const JointValues &joint,
					    Pose &pose,
					    ErrorCode &error_code)
{
    return computeForwardKinematics(joint, pose, error_code);
}


//------------------------------------------------------------
// Function:	MoveJ
// Summary: To plan a path in joint space to touch target pose, without smooth.
// In:	    joint_target -> target in joint space
//	    v_max	-> max velocity
//	    a_max	-> max acceleration
//	    id		-> command id
// Out:	    path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ( const JointValues &joint_target, double v_max, double a_max,
				      int id, ErrorCode &error_code)
{
    if( m_next_cmd_type!=enum_Type_MoveJ && m_next_cmd_type!=enum_Type_Other ) {
	error_code = Error::Cmd_Sequence_Error;
	return false;
    }

    if( !m_planned_path_fifo.empty() && 
        (m_planned_path_fifo.back().type==enum_Type_MoveL ||
	 m_planned_path_fifo.back().type==enum_Type_MoveC ) ) 
    {
	error_code = Error::Cartesian_Path_Exist;
	return false;
    }

    ROS_INFO("MoveJ request accepted, planning joint path...");

    v_max = v_max/m_VMAX*100;
    int v_percent = v_max>100?100:(int)v_max;
    std::vector<JointValues> planned_path;
    JointPoint jp_end,jp_target;
    jp_target.joints = joint_target;

    bool res = MoveJ2J( m_joint_start, 
		        jp_target, v_percent, 0,
			planned_path,
			jp_end, 
			error_code);
    
    if( res ) {
	if(0==planned_path.size()) {
	    ROS_INFO("Joint path generated with 0 point, are we standing on the target point already?");
	    return true;
	}

	if(setLatestIKReference(planned_path.back(),error_code)) {
	    PathPoint pp;
	    pp.type = enum_Type_MoveJ;

	    if( enum_Level_Start==m_point_level ) {
		pp.id = (id<<2) + m_point_level ;
		pp.joints = planned_path.front();
		m_planned_path_fifo.push_back(pp);
		m_point_level = enum_Level_Middle;
		pp.id = (id<<2) + m_point_level;
		
		for(std::vector<JointValues>::iterator itr=planned_path.begin()+1; itr!=planned_path.end(); ++itr) {
		    pp.joints = *itr;
		    m_planned_path_fifo.push_back(pp);
		}
	    }
	    else if( enum_Level_Middle==m_point_level ) {
		pp.id = (id<<2) + m_point_level ;
		
		for(std::vector<JointValues>::iterator itr=planned_path.begin(); itr!=planned_path.end(); ++itr) {
		    pp.joints = *itr;
		    m_planned_path_fifo.push_back(pp);
		}
	    }
	    else {
		error_code = Error::Undefined_Error;
		return false;
	    }
	    
	    m_planned_path_fifo.back().id = (id<<2) + enum_Level_Ending;
	    m_point_level = enum_Level_Start;
	    m_next_cmd_type = enum_Type_Other;

	    m_joint_start = jp_end;
	    computeFK(planned_path.back(), m_pose_start, error_code);
	    m_pose_previous = m_pose_start;
	    m_vu_start = 0;
	    m_v_start = 0;
	    
	    ROS_INFO("Joint path generated successfully with %d points, and added into planned_path_FIFO.",
			    (int)planned_path.size());

	    return true;
	}
	else
	    return false;
    }
}


//------------------------------------------------------------
// Function:	MoveJ (smooth to MoveJ)
// Summary: To plan a path in joint space to touch target pose, with smooth.
// In:	    joint_target -> target in joint space
//	    v_max	-> max velocity
//	    a_max	-> max acceleration
//	    cnt		-> smooth degree
//	    joint_next	-> next target in joint space
//	    v_next	-> max velocity in the next path
//	    a_next	-> max acceleration in the next path
//	    cnt_next	-> smooth degree in the next path
//	    id		-> command id
// Out:	    path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
                                     const JointValues &joint_next, double v_next, double a_next, int cnt_next,
				     int id, ErrorCode &error_code)
{
    if( m_next_cmd_type!=enum_Type_MoveJ && m_next_cmd_type!=enum_Type_Other ) {
	error_code = Error::Cmd_Sequence_Error;
	return false;
    }

    if( cnt<=0 ) {
	error_code = Error::Cmd_Parameter_Invalid;
	return false;
    }

    if( !m_planned_path_fifo.empty() && 
        (m_planned_path_fifo.back().type==enum_Type_MoveL ||
	 m_planned_path_fifo.back().type==enum_Type_MoveC) ) 
    {
	error_code = Error::Cartesian_Path_Exist;
	return false;
    }

    ROS_INFO("MoveJ request accepted, planning joint path...");

    v_max = v_max/m_VMAX*100;
    int v_percent = v_max>100?100:(int)v_max;
    std::vector<JointValues> planned_path;
    JointPoint jp_end,jp_target;
    jp_target.joints = joint_target;

    bool res = MoveJ2J( m_joint_start, 
		        jp_target, v_percent, cnt,
			planned_path,
			jp_end, 
			error_code);

    if( res ) {
	if(setLatestIKReference(planned_path.back(),error_code)) {
	    PathPoint pp;
	    pp.type = enum_Type_MoveJ;

	    if( enum_Level_Start==m_point_level ) {
		pp.id = (id<<2) + m_point_level ;
		pp.joints = planned_path.front();
		m_planned_path_fifo.push_back(pp);
		m_point_level = enum_Level_Middle;
		pp.id = (id<<2) + m_point_level;
		
		for(std::vector<JointValues>::iterator itr=planned_path.begin()+1; itr!=planned_path.end(); ++itr) {
		    pp.joints = *itr;
		    m_planned_path_fifo.push_back(pp);
		}
	    }
	    else if( enum_Level_Middle==m_point_level ) {
		pp.id = (id<<2) + m_point_level ;
		
		for(std::vector<JointValues>::iterator itr=planned_path.begin(); itr!=planned_path.end(); ++itr) {
		    pp.joints = *itr;
		    m_planned_path_fifo.push_back(pp);
		}
	    }
	    else {
		error_code = Error::Undefined_Error;
		return false;
	    }
	    
	    m_joint_start = jp_end;
	    m_next_cmd_type = enum_Type_MoveJ;

	    ROS_INFO("Joint path generated successfully with %d points, and added into planned_path_FIFO.",
			    (int)planned_path.size());

	    return true;
	}
	else
	    return false;
    }
    else {
	ROS_ERROR("ERROR occurred during generating joint path, error code:%d", error_code);
	ROS_ERROR("invalid path obtained and dropped.");
	return false;
    }
}


//------------------------------------------------------------
// Function:	MoveJ (smooth to MoveL)
// Summary: To plan a path in joint space to touch target pose, with smooth.
// In:	    joint_target -> target in joint space
//	    v_max	-> max velocity
//	    a_max	-> max acceleration
//	    cnt		-> smooth degree
//	    pose_next	-> next target in Cartesian space
//	    v_next	-> max velocity in the next path
//	    a_next	-> max acceleration in the next path
//	    cnt_next	-> smooth degree in the next path
//	    id		-> command id
// Out:	    path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ(const JointValues &joint_target, double v_max, double a_max, int cnt,
                                     const Pose &pose_next, double v_next, double a_next, int cnt_next,
				     int id, ErrorCode &error_code)
{
    if( m_next_cmd_type!=enum_Type_MoveJ && m_next_cmd_type!=enum_Type_Other ) {
	error_code = Error::Cmd_Sequence_Error;
	return false;
    }

    if( cnt<=0 ) {
	error_code = Error::Cmd_Parameter_Invalid;
	return false;
    }

    if( !m_planned_path_fifo.empty() && 
        (m_planned_path_fifo.back().type==enum_Type_MoveL ||
	 m_planned_path_fifo.back().type==enum_Type_MoveC) ) 
    {
	error_code = Error::Cartesian_Path_Exist;
	return false;
    }
    
    ROS_INFO("MoveJ request accepted, planning joint path...");

    v_max = v_max/m_VMAX*100;
    int v_percent = v_max>100?100:(int)v_max;
    std::vector<JointValues> planned_path;
    JointValues j_target = joint_target;

    bool res = MoveJ2L( m_joint_start, 
		        j_target, v_percent, cnt,
		        pose_next, v_next, cnt_next, planned_path,
		        m_pose_start, m_pose_previous,
		        m_v_start, m_vu_start, 
		        error_code);

    if( res ) {
	if(setLatestIKReference(planned_path.back(),error_code)) {
	    PathPoint pp;
	    pp.type = enum_Type_MoveJ;

	    if( enum_Level_Start==m_point_level ) {
		pp.id = (id<<2) + m_point_level ;
		pp.joints = planned_path.front();
		m_planned_path_fifo.push_back(pp);
		m_point_level = enum_Level_Middle;
		pp.id = (id<<2) + m_point_level;
		
		for(std::vector<JointValues>::iterator itr=planned_path.begin()+1; itr!=planned_path.end(); ++itr) {
		    pp.joints = *itr;
		    m_planned_path_fifo.push_back(pp);
		}
	    }
	    else if( enum_Level_Middle==m_point_level ) {
		pp.id = (id<<2) + m_point_level ;
		
		for(std::vector<JointValues>::iterator itr=planned_path.begin(); itr!=planned_path.end(); ++itr) {
		    pp.joints = *itr;
		    m_planned_path_fifo.push_back(pp);
		}
	    }
	    else {
		error_code = Error::Undefined_Error;
		return false;
	    }

	    m_next_cmd_type = enum_Type_MoveL;
	
	    ROS_INFO("Joint path generated successfully with %d points, and added into planned_path_FIFO.",
			    (int)planned_path.size());
	    
	    return true;
	}
	else
	    return false;
    }
    else {
	ROS_ERROR("ERROR occurred during generating joint path, error code:%d", error_code);
	ROS_ERROR("invalid path obtained and dropped.");
	return false;
    }
}


//------------------------------------------------------------
// Function:	MoveJ (smooth to MoveL)
// Summary: To plan a path in joint space to touch target pose, with smooth.
// In:	    joint_target -> target in joint space
//	    v_max	-> max velocity
//	    a_max	-> max acceleration
//	    cnt		-> smooth degree
//	    pose_next	-> next target in Cartesian space
//	    v_next	-> max velocity in the next path
//	    a_next	-> max acceleration in the next path
//	    cnt_next	-> smooth degree in the next path
//	    id		-> command id
// Out:	    path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ( const JointValues &joint_target, double v_max, double a_max, int cnt,
            const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
	    int id, ErrorCode &error_code)
{
    return MoveJ( joint_target, v_max, a_max, cnt,
		  transformPoseEuler2Pose(pose_next), v_next, a_next, cnt_next,
		  id, error_code);
}


//------------------------------------------------------------
// Function:    MoveL
// Summary: To plan a linear path to touch target pose, without smooth.
// In:      pose_target -> target pose of the linear path
//          v_max       -> max velocity of endpoint
//          a_max       -> max acceleration of endpoint
//	    id		-> command id
// Out:     path(hidden)-> outputs added into m_cartesian_path_FIFO automaticly
//	    error_code	-> error code
// Return:  true  -> plan successfully
//          false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL( const Pose &pose_target, double v_max, double a_max, 
                                      int id, ErrorCode &error_code)
{
    if( m_next_cmd_type!=enum_Type_MoveL && m_next_cmd_type!=enum_Type_Other ) {
	error_code = Error::Cmd_Sequence_Error;
	return false;
    }

    ROS_INFO("MoveL request accepted, planning cartesian path...");
    setMaxAcceleration(a_max);
    setMaxVelocity(v_max);

    double v_target = m_vmax * m_vmax_scaling_factor;
    std::vector<fst_controller::Pose> planned_path;
    
    bool res = MoveL2L(	m_pose_start, m_v_start, m_vu_start, 
			pose_target, v_target, 0, 
			pose_target, v_target, 0, 
			m_pose_previous,  planned_path, error_code);
    if(res) {
	PathPoint pp;
	pp.type = enum_Type_MoveL;
	
	if( enum_Level_Start==m_point_level ) {
	    pp.id = (id<<2) + m_point_level ;
	    pp.pose = planned_path.front();
	    m_planned_path_fifo.push_back(pp);
	    m_point_level = enum_Level_Middle;
	    pp.id = (id<<2) + m_point_level;
	    
	    for(std::vector<Pose>::iterator itr=planned_path.begin()+1; itr!=planned_path.end(); ++itr) {
		/*
		//------------------- FOR TEST ONLY --------------------
		ROS_INFO("Pose :%f,%f,%f,%f,%f,%f,%f", itr->position.x,itr->position.y,itr->position.z,itr->orientation.w,itr->orientation.x,itr->orientation.y,itr->orientation.z);
		//^^^^^^^^^^^^^^^^^^^ FOR TEST ONLY ^^^^^^^^^^^^^^^^^^^^
		*/
	   	pp.pose = *itr;
		m_planned_path_fifo.push_back(pp);
	    }
	}
	else if( enum_Level_Middle==m_point_level ) {
	    pp.id = (id<<2) + m_point_level;
	    
	    for(std::vector<Pose>::iterator itr=planned_path.begin(); itr!=planned_path.end(); ++itr) {
	   	pp.pose = *itr;
		m_planned_path_fifo.push_back(pp);
	    }
	}
	else {
	    error_code = Error::Undefined_Error;
	    return false;
	}

	m_planned_path_fifo.back().id = (id<<2) + enum_Level_Ending;
	m_point_level = enum_Level_Start;
	m_next_cmd_type = enum_Type_Other;

	ROS_INFO("Cartesian path generated successfully with %d points, and added into planned_path_FIFO.",
		(int)planned_path.size());

	return true;
    }
    else {
	ROS_ERROR("ERROR occurred during generating cartesian path, error code:%d", error_code);
	ROS_ERROR("invalid path obtained and dropped.");
	return false;
    }
}


//------------------------------------------------------------
// Function:    MoveL
// Summary: To plan a linear path to touch target pose, without smooth.
// In:      pose_target -> target pose of the linear path
//          v_max       -> max velocity of endpoint
//          a_max       -> max acceleration of endpoint
//	    id		-> command id
// Out:     path(hidden)-> outputs added into m_cartesian_path_FIFO automaticly
//	    error_code	-> error code
// Return:  true  -> plan successfully
//          false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL( const PoseEuler &pose_target, double v_max, double a_max, 
                                      int id, ErrorCode &error_code)
{
    return MoveL(transformPoseEuler2Pose(pose_target), v_max, a_max, id, error_code);
    //return true;
}


//------------------------------------------------------------
// Function:    MoveL (smooth to MoveL)
// Summary: To plan a linear path to touch target pose, with smooth.
// In:      pose_target -> target pose of the linear path
//          v_max       -> max velocity of endpoint
//          a_max       -> max acceleration of endpoint
//          cnt_target  -> smooth degree
//          pose_next   -> target pose of the next path
//          v_next      -> max velocity of endpoint in the next path
//          a_next      -> max acceleration of endpoint in the next path
//          cnt_next    -> smooth degree in the next path
//	    id		-> command id
// Out:     path(hidden)-> outputs added into m_cartesian_path_FIFO automaticly
//	    error_code	-> error code
// Return:  true  -> plan successfully
//          false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL(	const Pose &pose_target, double v_max, double a_max, int cnt_target,
					const Pose &pose_next, double v_next, double a_next, int cnt_next,
					int id, ErrorCode &error_code)
{
    if( m_next_cmd_type!=enum_Type_MoveL && m_next_cmd_type!=enum_Type_Other ) {
	error_code = Error::Cmd_Sequence_Error;
	return false;
    }

    if( 0>=cnt_target ) {
	error_code = Error::Cmd_Parameter_Invalid;
	return false;
    }

    ROS_INFO("MoveL request accepted, planning cartesian path...");
    setMaxAcceleration(a_max);
    setMaxVelocity(v_max);
    
    v_max   = m_vmax * m_vmax_scaling_factor;
    v_next  = v_next * m_vmax_scaling_factor;
    std::vector<fst_controller::Pose> planned_path;
    
    bool res = MoveL2L(	m_pose_start, m_v_start, m_vu_start, 
			pose_target, v_max, cnt_target, 
			pose_next, v_next, cnt_next, 
			m_pose_previous, planned_path, error_code);
    /*
    if( !res && Error::MoveL_Unsmoothable == error_code ) {
	res = MoveL2L(	m_pose_start, m_v_start, m_vu_start, 
			pose_target, v_max, 0, 
			pose_next, v_next, cnt_next, 
			m_pose_previous,  planned_path, error_code);
    }*/

    if(res) {
	PathPoint pp;
	pp.type = enum_Type_MoveL;
	
	if( enum_Level_Start==m_point_level ) {
	    pp.id = (id<<2) + m_point_level;
	    pp.pose = planned_path.front();
	    m_planned_path_fifo.push_back(pp);
	    m_point_level = enum_Level_Middle;
	    pp.id = (id<<2) + m_point_level;

	    for(std::vector<Pose>::iterator itr=planned_path.begin()+1; itr!=planned_path.end(); ++itr) {
		pp.pose = *itr;
		m_planned_path_fifo.push_back(pp);
	    }
	}
	else if( enum_Level_Middle==m_point_level ) {
	    pp.id = (id<<2) + m_point_level;
	    
	    for(std::vector<Pose>::iterator itr=planned_path.begin(); itr!=planned_path.end(); ++itr) {
		pp.pose = *itr;
		m_planned_path_fifo.push_back(pp);
	    }
	}
	else {
	    error_code = Error::Undefined_Error;
	    return false;
	}
	
	m_next_cmd_type = enum_Type_MoveL;

	ROS_INFO("Cartesian path generated successfully with %d points, and added into planned_path_FIFO.",
		(int)planned_path.size());
	
	return true;
    }
    else if( Error::MoveL_Unsmoothable==error_code ) {
	ROS_ERROR("Path unsmoothable using cnt=%d, replanning using cnt=0...", cnt_target);
	return MoveL(pose_target,v_max/m_vmax_scaling_factor,a_max,id,error_code);
    }
    else {
	ROS_ERROR("ERROR occurred during generating cartesian path, error code:%d", error_code);
	ROS_ERROR("invalid path obtained and dropped.");
    }
}


//------------------------------------------------------------
// Function:    MoveL (smooth to MoveL)
// Summary: To plan a linear path to touch target pose, with smooth.
// In:      pose_target -> target pose of the linear path
//          v_max       -> max velocity of endpoint
//          a_max       -> max acceleration of endpoint
//          cnt_target  -> smooth degree
//          pose_next   -> target pose of the next path
//          v_next      -> max velocity of endpoint in the next path
//          a_next      -> max acceleration of endpoint in the next path
//          cnt_next    -> smooth degree in the next path
//	    id		-> command id
// Out:     path(hidden)-> outputs added into m_cartesian_path_FIFO automaticly
//	    error_code	-> error code
// Return:  true  -> plan successfully
//          false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL(	const PoseEuler &pose_target, double v_max, double a_max, int cnt_target,
					const PoseEuler &pose_next, double v_next, double a_next, int cnt_next,
					int id, ErrorCode &error_code)
{
    return MoveL(   transformPoseEuler2Pose(pose_target),v_max,a_max,cnt_target,
    		    transformPoseEuler2Pose(pose_next),v_next,a_next,cnt_next,
    		    id, error_code);
}


//------------------------------------------------------------
// Function:	MoveL (smooth to MoveJ)
// Summary: To plan a linear path to touch target pose, with smooth.
// In:	    pose_target -> target pose of the linear path
//	    v_max	-> max velocity of endpoint
//	    a_max	-> max acceleration of endpoint
//	    cnt_target	-> smooth degree
// 	    joint_next	-> target pose in joint space of the next path
//	    v_next	-> max velocity of endpoint in the next path
//	    a_next	-> max acceleration of endpoint in the next path
//	    cnt_next	-> smooth degree in the next path
//	    id		-> command id
// Out:	    path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL(	const Pose &pose_target, double v_max, double a_max, int cnt_target,
					const JointValues &joint_next, double v_next, double a_next, int cnt_next,
					int id, ErrorCode &error_code)
{
    if( m_next_cmd_type!=enum_Type_MoveL && m_next_cmd_type!=enum_Type_Other ) {
	error_code = Error::Cmd_Sequence_Error;
	return false;
    }

    if( 0>=cnt_target ) {
	error_code = Error::Cmd_Parameter_Invalid;
	return false;
    }

    ROS_INFO("MoveL request accepted, planning cartesian path...");
    setMaxAcceleration(a_max);
    setMaxVelocity(v_max);
    
    v_max   = m_vmax * m_vmax_scaling_factor;
    v_next  = v_next * m_vmax_scaling_factor;
    std::vector<fst_controller::Pose> planned_path;
    
    bool res = MoveL2J( m_pose_start, m_v_start, m_vu_start,
		        pose_target, v_max, cnt_target,
		        m_pose_previous, planned_path,
		        m_joint_start, error_code);

    if(res) {
	fst_controller::PathPoint pp;
	pp.type = enum_Type_MoveL;

	if( enum_Level_Start==m_point_level ) {
	    pp.id = (id<<2) + m_point_level;
	    pp.pose = planned_path.front();
	    m_planned_path_fifo.push_back(pp);
	    m_point_level = enum_Level_Middle;
	    pp.id = (id<<2) + m_point_level;

	    for(std::vector<Pose>::iterator itr=planned_path.begin()+1; itr!=planned_path.end(); ++itr) {
		pp.pose = *itr;
		m_planned_path_fifo.push_back(pp);
	    }
	}
	else if( enum_Level_Middle==m_point_level ) {
	    pp.id = (id<<2) + m_point_level;
	    
	    for(std::vector<Pose>::iterator itr=planned_path.begin(); itr!=planned_path.end(); ++itr) {
		pp.pose = *itr;
		m_planned_path_fifo.push_back(pp);
	    }
	}
	else {
	    error_code = Error::Undefined_Error;
	    return false;
	}

	m_next_cmd_type = enum_Type_MoveJ;
	
	ROS_INFO("Cartesian path generated successfully with %d points, and added into planned_path_FIFO.",
		(int)planned_path.size());
	return true;
    }
    else {
	ROS_ERROR("ERROR occurred during generating cartesian path, error code:%d", error_code);
	ROS_ERROR("invalid path obtained and dropped.");
	return false;
    }
}


//------------------------------------------------------------
// Function:	MoveL (smooth to MoveJ)
// Summary: To plan a linear path to touch target pose, with smooth.
// In:	    pose_target -> target pose of the linear path
//	    v_max	-> max velocity of endpoint
//	    a_max	-> max acceleration of endpoint
//	    cnt_target	-> smooth degree
// 	    joint_next	-> target pose in joint space of the next path
//	    v_next	-> max velocity of endpoint in the next path
//	    a_next	-> max acceleration of endpoint in the next path
//	    cnt_next	-> smooth degree in the next path
//	    id		-> command id
// Out:	    path(hidden)-> outputs added into m_planned_path_FIFO automaticly
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL(	const PoseEuler &pose_target, double v_max, double a_max, int cnt_target,
					const JointValues &joint_next, double v_next, double a_next, int cnt_next,
					int id, ErrorCode &error_code)
{
    return MoveL( transformPoseEuler2Pose(pose_target), v_max, a_max, cnt_target,
		  joint_next, v_next, a_next, cnt_next, id, error_code);
}


//------------------------------------------------------------
// Function:    convertPathToTrajectory
// Summary: To convert numbers of posepoint in m_cartesian_path_FIFO 
//          into jointpoint in m_joint_trajectory_FIFO.
// In:      num -> number of pose that needed to be converted
// Out:     error_code -> error code
// Return:  <0	-> ERROR occurred during converting
//	    >=0	-> number of pose that convered actually
//------------------------------------------------------------
int fst_controller::ArmGroup::convertPathToTrajectory(int num, ErrorCode &error_code)
{
    error_code = Error::Success;
    
    if( num<0 ) {
	error_code = Error::Cmd_Parameter_Invalid;
	return num-1;
    }
    else if( num==0 ) {
	return 0;
    }

    if(m_joint_trajectory_FIFO_islocked) {
	error_code = Error::Trajectory_FIFO_Locked;
	return 0;
    }

    if(num > m_JOINT_FIFO_LEN - getJointTrajectoryFIFOLength()) {
	error_code = Error::Trajectory_FIFO_Full;
	num = m_JOINT_FIFO_LEN - getJointTrajectoryFIFOLength();
    }
    
    int conv_cnt;
    std::vector<fst_controller::PathPoint>::iterator itr = m_planned_path_fifo.begin();
    fst_controller::JointPoint jp;

    for( conv_cnt=0; conv_cnt<num; ++conv_cnt ) {
	if(itr==m_planned_path_fifo.end()) {
	    error_code = Error::No_Enough_Points;
	    break;
	}
	
	if( itr->type==enum_Type_MoveL || itr->type==enum_Type_MoveC ) {
	    if(computeIK(itr->pose,jp.joints,error_code)) {
	    //computeIK(itr->pose,jp.joints,error_code); {
		/*
		//------------------- FOR TEST ONLY --------------------
		JointValues j=m_latest_ik_reference;
		//Pose p = itr->pose;
		ErrorCode err1;
		//computeFK(j,p,err1);
		//ROS_INFO("reference:%f,%f,%f,%f,%f,%f", j.j1,j.j2,j.j3,j.j4,j.j5,j.j6);
		j=jp.joints;
		ROS_INFO("IK result:%f,%f,%f,%f,%f,%f", j.j1,j.j2,j.j3,j.j4,j.j5,j.j6);
		//ROS_INFO("Pose plann:%f,%f,%f,%f,%f,%f,%f", p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
		//computeFK(jp.joints,p,err1);
		//ROS_INFO("FK-%d:%f,%f,%f,%f,%f,%f,%f",error_code, p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
		//^^^^^^^^^^^^^^^^^^^ FOR TEST ONLY ^^^^^^^^^^^^^^^^^^^^
		*/
		if( itr==m_planned_path_fifo.end()-1 ) {
		    JointValues joints = getLatestIKReference();
		    m_joint_start.joints = jp.joints;
		    m_joint_start.omegas.j1 = (jp.joints.j1-joints.j1)/m_cycle_time;
		    m_joint_start.omegas.j2 = (jp.joints.j2-joints.j2)/m_cycle_time;
		    m_joint_start.omegas.j3 = (jp.joints.j3-joints.j3)/m_cycle_time;
		    m_joint_start.omegas.j4 = (jp.joints.j4-joints.j4)/m_cycle_time;
		    m_joint_start.omegas.j5 = (jp.joints.j5-joints.j5)/m_cycle_time;
		    m_joint_start.omegas.j6 = (jp.joints.j6-joints.j6)/m_cycle_time;
		}
		jp.id = itr->id;
		//jp.velocity = itr->velocity;
		m_joint_trajectory_fifo.push_back(jp);
		m_latest_ik_reference = jp.joints;
	    }
	    else {
		
		//------------------- FOR TEST ONLY --------------------
		ROS_ERROR("converted=%d, Error code=%d",conv_cnt,error_code);
		JointValues j=m_latest_ik_reference;
		//Pose p;
		//ErrorCode err1;
		//computeFK(j,p,err1);
		ROS_INFO("reference:%f,%f,%f,%f,%f,%f", j.j1,j.j2,j.j3,j.j4,j.j5,j.j6);
		j=jp.joints;
		ROS_INFO("IK result:%f,%f,%f,%f,%f,%f", j.j1,j.j2,j.j3,j.j4,j.j5,j.j6);
		//ROS_INFO("Pose reference:%f,%f,%f,%f,%f,%f,%f", p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
		//computeFK(j,p,err1);
		//ROS_INFO("Pose result:   %f,%f,%f,%f,%f,%f,%f", p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
		//^^^^^^^^^^^^^^^^^^^ FOR TEST ONLY ^^^^^^^^^^^^^^^^^^^^
		
		break;
	    }
	}
	else if(itr->type==enum_Type_MoveJ) {
	    jp.id = itr->id;
	    //jp.velocity = itr->velocity;
	    jp.joints = itr->joints;
	    m_joint_trajectory_fifo.push_back(jp);
	    m_latest_ik_reference = jp.joints;
	}

	++itr;
    }

    if(conv_cnt!=0) {
	itr = m_planned_path_fifo.begin();
	m_planned_path_fifo.erase(itr,itr+conv_cnt);
    }

    return conv_cnt;
    
}


//------------------------------------------------------------
// Function:    suspendArmMotion
// Summary: To replan a slow-down path and stop. Used when pause event
//          or IK failure raised.
// In:      None
// Out:     None
// Return:  true  -> replan successfully
//          false -> replan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::suspendArmMotion(void)
{
    /*
    ROS_INFO("Suspend request accepted.");
    m_joint_trajectory_FIFO_islocked = true;
    ROS_INFO("joint_trajectory_FIFO is locded, replanning slow-down path in joint space...");
    

    m_joint_trajectory_FIFO_islocked = false;
    ROS_INFO("Replan successfully, joint_trajectory_FIFO is unlocked.");
    */
    return true;
}


//------------------------------------------------------------
// Function:	resumeArmMotion
// Summary: To replan a start-up path and resume arm motion.
// In:	    None
// Out:	    None
// Return:  true  -> replan successfully
//	    false -> replan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::resumeArmMotion(void)
{
    return true;
}





//-----------------------------private function---------------------------------------------


//------------------------------------------------------------
// Function:    setCycleTime
// Summary: To set cycle time of interpolation algorithm.
// In:	    None
// Out:     None
// Return:  None
//------------------------------------------------------------
void fst_controller::ArmGroup::setCycleTime(void)
{
    m_planner->setCycleTime(m_cycle_time);
}


//------------------------------------------------------------
// Function:    setMaxAcceleration
// Summary: To set the max acceleration in algorithm
// In:      None
// Out:     None
// Return:  None
//------------------------------------------------------------
void fst_controller::ArmGroup::setMaxAcceleration(void)
{
    m_planner->setAcceleration(m_amax*m_amax_scaling_factor);
}


//------------------------------------------------------------
// Function:	setJumpThresholdScalingFactor
// Summary: To set jump threshold scaling factor.
// In:	    None
// Out:	    None
// Return:  None
//------------------------------------------------------------
void fst_controller::ArmGroup::setJumpThresholdScalingFactor(void)
{
    m_planner->setLimitScale(m_jump_threshold_scaling);
}


//------------------------------------------------------------
// Function:	setJointConstraints
// Summary: To set joint constraints in Kinematics algorithm.
// In:	    None
// Out:	    None
// Return:  None
//------------------------------------------------------------
void fst_controller::ArmGroup::setJointConstraints(void)
{
    m_planner->setAxisLimit(m_joint_constraints);
}


//------------------------------------------------------------
// Function:    loadJointConstraints
// Summary: To load joint constraints from parameter server.
// In:      None
// Out:     None
// Return:  true  -> load parameter successfully
//          false -> load parameter UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::loadJointConstraints(void)
{
    fst_controller::JointConstraints constraint;

    constraint.j1.home  = 0.0;
    constraint.j1.upper = 3.1416;
    constraint.j1.lower = -3.1416;
    constraint.j1.max_omega = 5.5851;
    constraint.j1.max_alpha = 6.9813;
    constraint.j2.home  = 0.0;
    constraint.j2.upper = 1.309;
    constraint.j2.lower = -2.2689;
    constraint.j2.max_omega = 6.2832;
    constraint.j2.max_alpha = 7.8540;
    constraint.j3.home  = 0.0;
    constraint.j3.upper = 3.2289;
    constraint.j3.lower = -3.1416;
    constraint.j3.max_omega = 6.9813;
    constraint.j3.max_alpha = 13.9626;
    constraint.j4.home  = 0.0;
    constraint.j4.upper = 3.3161;
    constraint.j4.lower = -3.3161;
    constraint.j4.max_omega = 6.2832;
    constraint.j4.max_alpha = 12.5664;
    constraint.j5.home  = 0.0;
    constraint.j5.upper = 2.2;
    constraint.j5.lower = -2.2;
    constraint.j5.max_omega = 4.1888;
    constraint.j5.max_alpha = 10.4720;
    constraint.j6.home  = 0.0;
    constraint.j6.upper = 6.2832;
    constraint.j6.lower = -6.2832;
    constraint.j6.max_omega = 12.5664;
    constraint.j6.max_alpha = 31.4159;

    setJointConstraints(constraint);
    return true;
}


//------------------------------------------------------------
// Function:    checkJointBoundary
// Summary: To check whether a group of joint values are valid according to 
//          joint constraints.
// In:      joint_values -> joint_values needed to be checked
// Out:     None
// Return:  true  -> valid
//          false -> INvalid
//------------------------------------------------------------
bool fst_controller::ArmGroup::checkJointBoundary(const JointValues &joint_values)
{
    return  joint_values.j1>m_joint_constraints.j1.lower &&
	    joint_values.j1<m_joint_constraints.j1.upper &&
	    joint_values.j2>m_joint_constraints.j2.lower &&
	    joint_values.j2<m_joint_constraints.j2.upper &&
	    joint_values.j3>m_joint_constraints.j3.lower &&
	    joint_values.j3<m_joint_constraints.j3.upper &&
	    joint_values.j4>m_joint_constraints.j4.lower &&
	    joint_values.j4<m_joint_constraints.j4.upper &&
	    joint_values.j5>m_joint_constraints.j5.lower &&
	    joint_values.j5<m_joint_constraints.j5.upper &&
	    joint_values.j6>m_joint_constraints.j6.lower &&
	    joint_values.j6<m_joint_constraints.j6.upper;
}


//------------------------------------------------------------
// Function:    getLatestIKReference
// Summary: To get latest IK reference values.
// In:      None
// Out:     None
// Return:  a group of joint values used as IK reference
//------------------------------------------------------------
const fst_controller::JointValues& fst_controller::ArmGroup::getLatestIKReference(void)
{   
    return m_latest_ik_reference;
}


//------------------------------------------------------------
// Function:    transformPoseEuler2Pose
// Summary: To transform a poseEuler point to a pose point.
// In:      poes_e -> the poseEuler to be transformed
// Out:     None
// Return:  pose point
//------------------------------------------------------------
fst_controller::Pose fst_controller::ArmGroup::transformPoseEuler2Pose(const PoseEuler &pose_e)
{
    fst_controller::Pose pose;
    double matrix_tmp[4][4];
    double position[3];
    double orientation[4];
    
    m_planner->Euler2Matrix(pose_e.position.x, pose_e.position.y, pose_e.position.z, 
			    pose_e.orientation.a, pose_e.orientation.b, pose_e.orientation.c,
			    matrix_tmp);
    m_planner->MATRIX2POSE(matrix_tmp,position,orientation);
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    pose.orientation.w = orientation[0];
    pose.orientation.x = orientation[1];
    pose.orientation.y = orientation[2];
    pose.orientation.z = orientation[3];

    return pose;
}


//------------------------------------------------------------
// Function:    transformPose2PoseEuler
// Summary: To transform a pose point to a poseEuler point.
// In:      poes -> the pose to be transformed
// Out:     None
// Return:  poseEuler point
//------------------------------------------------------------
fst_controller::PoseEuler fst_controller::ArmGroup::transformPose2PoseEuler(const Pose &pose)
{
    fst_controller::PoseEuler pose_e;
    double matrix_tmp[4][4];
    double position[3];
    double orientation[4];
    
    position[0] = pose.position.x;
    position[1] = pose.position.y;
    position[2] = pose.position.z;
    orientation[0] = pose.orientation.w;
    orientation[1] = pose.orientation.x;
    orientation[2] = pose.orientation.y;
    orientation[3] = pose.orientation.z;
    m_planner->POSE2MATRIX(position,orientation,matrix_tmp);
    m_planner->Matrix2Euler(matrix_tmp,	pose_e.position.x,
					pose_e.position.y,
					pose_e.position.z,
					pose_e.orientation.a,
					pose_e.orientation.b,
					pose_e.orientation.c);

    return pose_e;
}


//------------------------------------------------------------
// Function:    computeInverseKinematics
// Summary: To compute IK with a given pose in cartesian space.
// In:      poes -> cartesian space pose needed to compute IK
//	    joint_reference -> joint reference used during compute IK
// Out:     joint_result -> IK result
//	    error_code -> error code
// Return:  true  -> IK solution found
//          false -> IK solution NOT found
//------------------------------------------------------------
bool fst_controller::ArmGroup::computeInverseKinematics(   
					    const Pose &pose,
					    const JointValues &joint_reference,
					    JointValues &joint_result,
					    ErrorCode &error_code)
{
    int res = m_planner->InverseKinematics(pose, joint_reference, joint_result);
    //ROS_INFO("IK Ref:%f,%f,%f,%f,%f,%f",joint_reference.j1,joint_reference.j2,joint_reference.j3,joint_reference.j4,joint_reference.j5,joint_reference.j6);
    //ROS_INFO("IK Res:%f,%f,%f,%f,%f,%f",joint_result.j1,joint_result.j2,joint_result.j3,joint_result.j4,joint_result.j5,joint_result.j6);
    switch( res ) {
	case 0:
	    error_code = Error::Success;
	    return true;
	case 1001:
	    error_code = Error::IK_Out_Of_Workspace;
	    return false;
	case 1002:
	    error_code = Error::IK_Joint_Out_Of_Limit;
	    return false;
	case 1003: 
	    error_code = Error::IK_Excessive_Distance;
	    return false;
	default:
	    error_code = Error::Undefined_Error;
	    return false;
    }
}


//------------------------------------------------------------
// Function:    computeForwardKinematics
// Summary: To compute FK with given joint values.
// In:      joint_result -> joint values needed to compute FK
// Out:     poes -> FK result
//	    error_code -> error code
// Return:  true  -> FK computed successfully
//          false -> FK computed UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::computeForwardKinematics(
					    const JointValues &joint,
					    Pose &pose,
					    ErrorCode &error_code)
{
    int res = m_planner->ForwardKinematics(joint, pose);
    switch( res ) {
	case 0:
	    error_code = Error::Success;
	    return true;
	case 1011:
	    error_code = Error::FK_Out_Of_Joint_Limit;
	    return false;
	default:
	    error_code = Error::Undefined_Error;
	    return false;
    }
}


//------------------------------------------------------------
// Function:	MoveJ2L
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:	    jp_start	-> initial position and omega of six joints
//	    joint_target-> joint target of the plan
//	    v_percent	-> percentage of velocity, range:1-100
//	    cnt		-> desired cnt value of the plan, range:0-100
// 	    pose_next	-> the target pose of the next plan used for smoothing
//	    v_next	-> desired velocity of the next plan used for smoothing
//	    cnt_next	-> desired cnt value of the next plan used for smoothing
// Out:	    pose_start	-> the end pose of this plan, also the initial pose of the next plan
//	    pose_previous -> the pose_previous input in the next plan
//	    v_start	-> the end velocity of this plan, also the initial velocity of the next plan
//	    vu_start	-> the end value of intermediate-variable in this plan, also the initial value in next plan
//	    planned_path-> planned path in joint space
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ2L(	const JointPoint &jp_start, 
					JointValues &joint_target, int v_percent, int cnt,
					const Pose &pose_next, double v_next, int cnt_next,
					std::vector<JointValues> &planned_path,
					Pose &pose_start, Pose &pose_previous,
					double &v_start, double &vu_start,
					ErrorCode &err)
{

    int res = m_planner->MoveJ2L_Jspace( jp_start,
					 joint_target, v_percent, cnt,
					 pose_next, v_next,
					 planned_path,
					 pose_start, v_start, vu_start,
					 pose_previous
					);

    switch( res ) {
	case 0:
	    err = Error::Success;
	    return true;
	case 1031:
	    err = Error::MoveJ_Axis_Overshoot;
	    return false;
	case 1032:
	    err = Error::MoveJ_Axis_Near_Limit;
	    return false;
	default:
	    err = Error::Undefined_Error;
	    return false;
    }
}


//------------------------------------------------------------
// Function:	MoveJ2L
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:	    jp_start	-> initial position and omega of six joints
//	    pose_target -> joint target of the plan
//	    v_percent	-> percentage of velocity, range:1-100
//	    cnt		-> desired cnt value of the plan, range:0-100
// 	    pose_next	-> the target pose of the next plan used for smoothing
//	    v_next	-> desired velocity of the next plan used for smoothing
//	    cnt_next	-> desired cnt value of the next plan used for smoothing
// Out:	    pose_start	-> the end pose of this plan, also the initial pose of the next plan
//	    pose_previous -> the pose_previous input in the next plan
//	    v_start	-> the end velocity of this plan, also the initial velocity of the next plan
//	    vu_start	-> the end value of intermediate-variable in this plan, also the initial value in next plan
//	    planned_path-> planned path in joint space
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ2L(	const JointPoint &jp_start, 
					const Pose &pose_target, int v_percent, int cnt,
					const Pose &pose_next, double v_next, int cnt_next,
					std::vector<JointValues> &planned_path,
					Pose &pose_start, Pose &pose_previous,
					double &v_start, double &vu_start,
					ErrorCode &err)
{
    int res = m_planner->MoveJ2L_Cspace( jp_start,
					 pose_target, v_percent, cnt,
					 pose_next, v_next,
					 planned_path,
					 pose_start, v_start, vu_start,
					 pose_previous
					);

    switch( res ) {
	case 0:
	    err = Error::Success;
	    return true;
	case 1031:
	    err = Error::MoveJ_Axis_Overshoot;
	    return false;
	case 1032:
	    err = Error::MoveJ_Axis_Near_Limit;
	    return false;
	default:
	    err = Error::Undefined_Error;
	    return false;
    }
}


//------------------------------------------------------------
// Function:	MoveJ2J
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:	    jp_start	-> initial position and omega of six joints
//	    jp_target	-> joint target of the plan
//	    v_percent	-> percentage of velocity, range:1-100
//	    cnt		-> desired cnt value of the plan, range:0-100
// Out:	    jp_end	-> the ending position and omega of six joints in this plan
//	    planned_path-> planned path in joint space
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ2J( const JointPoint &jp_start,
					JointPoint &jp_target, int v_percent, int cnt,
					std::vector<JointValues> &planned_path,
					JointPoint &jp_end,
					ErrorCode &err)
{
    int res = m_planner->MoveJ2J_Jspace( jp_start, jp_target, v_percent, cnt,
					 planned_path, jp_end);
    switch( res ) {
	case 0:
	    err = Error::Success;
	    return true;
	case 1031:
	    err = Error::MoveJ_Axis_Overshoot;
	    return false;
	case 1032:
	    err = Error::MoveJ_Axis_Near_Limit;
	    return false;
	default:
	    err = Error::Undefined_Error;
	    return false;
    }
}


//------------------------------------------------------------
// Function:	MoveJ2J
// Summary: To plan a path in joint space to touch target pose, with/without smooth.
// In:	    jp_start	-> initial position and omega of six joints
//	    pose_target	-> pose target of the plan
//	    v_percent	-> percentage of velocity, range:1-100
//	    cnt		-> desired cnt value of the plan, range:0-100
// Out:	    jp_end	-> the ending position and omega of six joints in this plan
//	    planned_path-> planned path in joint space
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveJ2J( const JointPoint &jp_start,
					const Pose &pose_target, int v_percent, int cnt,
					std::vector<JointValues> &planned_path,
					JointPoint &jp_end,
					ErrorCode &err)
{
    int res = m_planner->MoveJ2J_Cspace( jp_start, pose_target, v_percent, cnt,
					 planned_path, jp_end);
    switch( res ) {
	case 0:
	    err = Error::Success;
	    return true;
	case 1031:
	    err = Error::MoveJ_Axis_Overshoot;
	    return false;
	case 1032:
	    err = Error::MoveJ_Axis_Near_Limit;
	    return false;
	default:
	    err = Error::Undefined_Error;
	    return false;
    }
}


//------------------------------------------------------------
// Function:	MoveL2L
// Summary: To plan a linear path to touch target pose, with/without smooth.
// In:	    pose_start	-> initial pose of the plan
//	    v_start	-> initial velocity of the plan
//	    vu_start	-> initial value of the intermediate-variable
// 	    pose_target	-> target pose of the plan
//	    v_target	-> desired velocity of the plan
//	    cnt_target	-> desired cnt value of the plan
// 	    pose_next	-> the target pose of the next plan used for smoothing
//	    v_next	->  desired velocity of the next plan used for smoothing
//	    cnt_next	-> desired cnt value of the next plan used for smoothing
//	    pose_previuos -> the previous pose used for interpolation
// Out:	    pose_start	-> the end pose of this plan, also the initial pose of the next plan
//	    v_start	-> the end velocity of this plan, also the initial pose of the next plan
//	    vu_start	-> the end value of intermediate-variable in this plan, also the initial value in next plan
//	    pose_previous -> the pose_previous in the next plan
//	    planned_path-> planned path in cartesian space
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL2L(	Pose &pose_start, double &v_start, double &vu_start,
					const Pose &pose_target, double v_target, int cnt_target,
					const Pose &pose_next, double v_next, int cnt_next,
					Pose &pose_previous,
					std::vector<Pose> &planned_path,
					ErrorCode &error_code)
{
    int res = m_planner->MoveL(	pose_start, v_start, vu_start,
				pose_target, v_target, cnt_target,
				pose_next, v_next, cnt_next,
				pose_previous, planned_path);
    /*
    // For test 
    // print all planned points 
    int cnt=0;
    std::vector<fst_controller::Pose>::iterator itr=planned_path.begin();
    for(;itr!=planned_path.end();++itr) {
	ROS_INFO("Pose-%d: position(%f,%f,%f), orientation(%f,%f,%f,%f)",++cnt,
		itr->position.x,itr->position.y,itr->position.z,
		itr->orientation.w,itr->orientation.x,itr->orientation.y,itr->orientation.z);
    }*/
    switch( res ) {
	case 0:
	    error_code = Error::Success;
	    return true;
	case 1021:
	    error_code = Error::MoveL_Unsmoothable;
	    return false;
	case 1022:
	    error_code = Error::MoveL_Zero_Distance;
	    return false;
	default:
	    error_code = Error::Undefined_Error;
	    return false;
    }
}


//------------------------------------------------------------
// Function:	MoveL2J
// Summary: To plan a linear path to touch target pose, with/without smooth.
// In:	    pose_start	-> initial pose of the plan
//	    v_start	-> initial velocity of the plan
//	    vu_start	-> initial value of the intermediate-variable
// 	    pose_target	-> target pose of the plan
//	    v_target	-> desired velocity of the plan
//	    cnt_target	-> desired cnt value of the plan
//	    pose_previuos -> the previous pose used for interpolation
// Out:	    planned_path-> planned path in cartesian space
//	    jp		-> position and omega of six joints at the ending of this path
//	    error_code	-> error code
// Return:  true  -> plan successfully
//	    false -> plan UNsuccessfully
//------------------------------------------------------------
bool fst_controller::ArmGroup::MoveL2J( const Pose &pose_start, double v_start, double vu_start,
                                        const Pose &pose_target, double v_target, int cnt_target,
                                        Pose &pose_previous, std::vector<Pose> &planned_path,
                                        JointPoint &jp, ErrorCode &error_code)
{
    int res = m_planner->MoveL2J( pose_start, v_start, vu_start,
                                  pose_target, v_target, cnt_target,
				  pose_previous, planned_path,
				  jp);
    
    switch( res ) {
      case 0:
        error_code = Error::Success;
        return true;
      case 1021:
	error_code = Error::MoveL_Unsmoothable;
	return false;
      case 1022:
	error_code = Error::MoveL_Zero_Distance;
	return false;
      default:
	error_code = Error::Undefined_Error;
	return false;
    }
}











