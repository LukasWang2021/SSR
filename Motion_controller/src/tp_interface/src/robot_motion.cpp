/**
 * @file robot_motion_.cpp
 * @brief: use count instead of wait xx ms in updateLogicState
 * @author WangWei
 * @version 2.0.5
 * @date 2017-4-18
 */
#include "robot_motion.h"
#include "stdio.h"
#include "common.h"
#include "sub_functions.h"
#include "safety/safety.h" 
#include "rt_timer.h"
#include <boost/algorithm/string.hpp>

#define MAX(a,b) ((a) > (b) ? (a) : (b))


void processNonMove(void* parameter)
{
    RobotMotion *rob_motion = (RobotMotion*)parameter;
    while(1)
    {
        ProgramState g_prgm_state = rob_motion->getProgramState();
        switch (g_prgm_state)
        {
            case IDLE_R: //this thread should exit
                return; 
            case PAUSE_TO_EXECUTE_T:
            case IDLE_TO_EXECUTE_T:
            case EXECUTE_R:
                if (!rob_motion->non_move_instructions_.empty())
                {
                    rob_motion->setNMPrgmState(EXECUTE_R);
                    ThreadSafeList<CommandInstruction>::iterator it;
                    for (it = rob_motion->non_move_instructions_.begin(); \
                        it != rob_motion->non_move_instructions_.end(); it++)
                    {
                        if (it->commandtype == motion_spec_MOTIONTYPE_WAIT)
                        {
                            rtMsSleep(it->msecond);
                        }
                        else if (it->commandtype == motion_spec_MOTIONTYPE_SET)
                        {
                            motion_spec_Set *set = (motion_spec_Set*)it->command_arguments.c_str();
                            
                            
                            if (it->msecond > 0)
                            {
                            
                            }
                            else
                            {
                               //rob_motion->getIOInterfacrPtr()->setIO( 
                            }
                        }
                    }
                }
                break;
            case EXECUTE_TO_PAUSE_T:          
                break;
            case PAUSE_TO_IDLE_T:
                break;
            case PAUSED_R:
                break;
            default:
                break;
        }
        usleep(10*1000); //sleep 10ms
    }
}



RobotMotion::RobotMotion()
{
    servo_ready_wait_ = false;
	previous_command_id_ = -1;
	current_command_id_ = -1;

    vel_factor_ = 100;
    run_mode_ = NORMAL_R;
    non_move_prgm_state_ = IDLE_R;

    prev_err_ = TPI_SUCCESS;							
	prev_mode_ = INIT_M;
	mode_ = INIT_M;
    prev_state_ = ESTOP_S;
    state_ = ESTOP_S;
    program_state_ = IDLE_R;
    memset((char*)&manu_req_, 0, sizeof(ManualReq));

    io_interface_ = new IOInterface();

    arm_group_ = new ArmGroup();
    curve_mdoe_ = CURVE_MODE_T;
        	
	Transformation tool_frame;
	memset(&tool_frame, 0, sizeof(tool_frame));
	arm_group_->setToolFrame(tool_frame);  
}


RobotMotion::~RobotMotion()
{
    //==before exit, first estop to fault===========
    servoEStop();
    setLogicStateCmd(EMERGENCY_STOP_E);
    //==============================================
    if (arm_group_)
    {
        delete arm_group_;
    }
    if (io_interface_ != NULL)
        delete io_interface_;
}


U64 RobotMotion::initial()
{
    U64 result;

    if((result = share_mem_.initial()) != TPI_SUCCESS)
    {
        FST_ERROR("ShareMem init failed");
        return result;
    }

    share_mem_.stopBareMetal();

	arm_group_->initArmGroup(result);
	if (result != TPI_SUCCESS)
    {
        FST_ERROR("create arm_group_ failed");
        return result;
    }

    if ((result = io_interface_->initial()) != TPI_SUCCESS)
    {
        FST_ERROR("io_interface_ init failed");
        return result;
    }


    return TPI_SUCCESS;
}

void RobotMotion::destroy()
{
}



U64 RobotMotion::checkProgramState()
{
    U64 result = TPI_SUCCESS;
    
    
    ProgramState prgm_state = getProgramState();

    switch (prgm_state)
    {
        case IDLE_R: 
            if (getLogicState() != ENGAGED_S)
            {
                break;
            }
            if (hasMoveCommand())
            {
                FST_INFO("IDLE_TO_EXECUTE_T");
                setProgramState(IDLE_TO_EXECUTE_T);
            }
            break;
        case PAUSE_TO_EXECUTE_T:
        case IDLE_TO_EXECUTE_T:
            //usleep(50*1000);   //sleep 50ms to wait until next program in auto mode
            result = planFifo();
            if (result == TPI_SUCCESS)
            {
                usleep(IDLE2EXE_DELAY * 1000); //sleep 50ms to wait until next state
                setProgramState(EXECUTE_R);                
            }
            else
            {
                setProgramState(PAUSED_R);
            }
            break;
        case EXECUTE_TO_PAUSE_T:
            sendJointsToRemote();     
            //FST_INFO("servo state:%d", getServoState());
            if ((getServoState() == STATE_READY) 
            || (getServoState() == STATE_ERROR))
            {
                setProgramState(PAUSED_R);
            }            
            break;
        case PAUSE_TO_IDLE_T:
            clearInstructionList();
            setProgramState(IDLE_R);
            break;
        case EXECUTE_R:
            if (getLogicState() != ENGAGED_S)
            {
                break;
            }
            if (getLogicMode() == AUTO_RUN_M)
            {
                processEndingMove();
            }
            if (hasTransformedToIdle())
            {
                FST_INFO("EXECUTE_TO_idle");
                setProgramState(IDLE_R);
                boost::mutex::scoped_lock lock(mutex_);
                manu_req_.vel_max = 0;
            }
            else
            {
                sendJointsToRemote();
                result = pickPointsFromPathFifo();
                if (result != TPI_SUCCESS)
                    return result;    
                result = planFifo();
                if (result != TPI_SUCCESS)
                    return result;
            }
            break;
        case PAUSED_R:       
        {
            static const int max_count = MAX_TIME_IN_PUASE / INTERVAL_PROCESS_UPDATE;
            static int count = 0;
            if (getLogicState() == ENGAGED_S)
            {
                count++;
                if (count >= max_count) //wait
                {
                    count = 0;
                    setLogicStateCmd(EMERGENCY_STOP_E);
                    // then we need to return this info
                }
            }
            else
            {
                
                    count = 0;
            }
            //timeout to set logic state estop
            RobotMode mode = getLogicMode();
            if (mode == AUTO_RUN_M)
            {
                if (instruction_list_.empty())
                {
                    count = 0;
                    setProgramState(IDLE_R);
                }
            }
            else if (mode == MANUAL_MODE_M)
            {
                count = 0;
                clearPathFifo();
                setProgramState(IDLE_R);
                boost::mutex::scoped_lock lock(mutex_);
                manu_req_.vel_max = 0;
            }
            else
            {
                setProgramState(IDLE_R);
            }
            break;  
        }
        default:
            break;
    }

    return result;
}

bool RobotMotion::isProgramStateChanged()
{
    static ProgramState pre_prgm_state;

    ProgramState ps = getProgramState();
    if (pre_prgm_state != ps)
    {
        pre_prgm_state = ps;
        return true;
    }
    else
    {
        return false;
    }
}
IOInterface* RobotMotion::getIOInterfacrPtr()
{
    boost::mutex::scoped_lock lock(mutex_);
	return this->io_interface_;
}
/**
 * @brief: get ShareMem object 
 *
 * @return: &share_mem_ 
 */
ShareMem* RobotMotion::getShareMemPtr()
{
	boost::mutex::scoped_lock lock(mutex_);
	return &(this->share_mem_);
}

/**
 * @brief: get the pointer of ArmGroup 
 *
 * @return: arm_group_ 
 */
ArmGroup* RobotMotion::getArmGroupPtr()
{
	boost::mutex::scoped_lock lock(mutex_);
	return this->arm_group_;
}

SafetyInterface* RobotMotion::getSafetyInterfacePtr()
{
    boost::mutex::scoped_lock lock(mutex_);
    return &(this->safety_interface_);
}

RunningMode RobotMotion::getRunningMode()
{
    boost::mutex::scoped_lock lock(mutex_);
    return this->run_mode_;
}

void RobotMotion::setRunningMode(RunningMode rm)
{
    boost::mutex::scoped_lock lock(mutex_);
    if (rm == NORMAL_R)
    {
        run_mode_ = NORMAL_R;
    }
    else
    {
        run_mode_ = (RunningMode)(run_mode_ | rm);
    }
}

int RobotMotion::getCurveMode()
{
    boost::mutex::scoped_lock lock(mutex_);
    return this->curve_mdoe_;
}

bool RobotMotion::setCurveMode(int c_mode)
{
    boost::mutex::scoped_lock lock(mutex_);
    if (curve_mdoe_ != c_mode)
    {
        if (arm_group_->setCurveMode(c_mode))
        {
            curve_mdoe_ = c_mode;
            return true;
        }
        else
        {
            return false;
        }
    }
    return true;
}

ProgramState RobotMotion::getNMPrgmState()
{
    boost::mutex::scoped_lock lock(mutex_);
    return non_move_prgm_state_; 
}

void RobotMotion::setNMPrgmState(ProgramState prgm_state)
{
    boost::mutex::scoped_lock lock(mutex_);
    non_move_prgm_state_ = prgm_state;
}


ProgramState RobotMotion::getProgramState()
{
    boost::mutex::scoped_lock lock(mutex_);
    return this->program_state_;
}
void RobotMotion::setProgramState(ProgramState prgm_state)
{
    boost::mutex::scoped_lock lock(mutex_);
    this->program_state_ = prgm_state;
}

int RobotMotion::getInstructionListSize()
{
    int size = instruction_list_.size();
    if (size == 1)
    {
        CommandInstruction instruction = instruction_list_.front();
        if (instruction.smoothDistance >= 0)
        {
            if (arm_group_->getPlannedPathFIFOLength() <= MIN_POINTS_FOR_NEXT_CMD)
            {
                size = -1;
            }
        } 
    }

    return size;
}
/**
 * @brief: get previous_command_id_ 
 *
 * @return: previous_command_id_ 
 */
int RobotMotion::getPreviousCmdID()
{
	boost::mutex::scoped_lock lock(mutex_);
	return this->previous_command_id_;
}

void RobotMotion::setPreviousCmdID(int id)
{
	boost::mutex::scoped_lock lock(mutex_);
	this->previous_command_id_ = id;
}

/**
 * @brief: get current_command_id_ 
 *
 * @return: current_command_id_ 
 */
int RobotMotion::getCurrentCmdID()
{
	boost::mutex::scoped_lock lock(mutex_);
	return this->current_command_id_;
}

void RobotMotion::setCurrentCmdID(int id)
{
	boost::mutex::scoped_lock lock(mutex_);
	this->current_command_id_ = id;
}


/**
 * @brief: get current logic mode
 *
 * @return: mode_ 
 */
RobotMode RobotMotion::getLogicMode()
{
	boost::mutex::scoped_lock lock(mutex_);
	return this->mode_;
}

/**
 * @brief: Get current logic state
 *
 * @return: state_
 */
RobotState RobotMotion::getLogicState()
{
	boost::mutex::scoped_lock lock(mutex_);
	return this->state_;
}

/**
 * @brief: Get current joints
 *
 * @return: joints_
 */
JointValues RobotMotion::getCurJointsValue()
{
	boost::mutex::scoped_lock lock(mutex_);
	return this->joints_;
}

/**
 * @brief: get the position
 *
 * @return: pose_
 */
PoseEuler RobotMotion::getCurPosition()
{
	boost::mutex::scoped_lock lock(mutex_);
	return this->pose_;
}

/**
 * @brief: get servo state, 0:init, 1:ready, 2:running, 3:error 
 *
 * @return: servo_state_ 
 */
unsigned int RobotMotion::getServoState()
{
    boost::mutex::scoped_lock lock(mutex_);
	return this->servo_state_;
}

void RobotMotion::setServoState(unsigned int servo_state)
{
    boost::mutex::scoped_lock lock(mutex_);
    this->servo_state_ = servo_state;
}

/**
 * @brief update the logic mode
 */
bool RobotMotion::updateLogicMode()
{
    static RobotMode pre_mode;
	RobotMode mode = getLogicMode();
 
	switch(mode)
	{
        case AUTO_RUN_TO_MANUAL_T:
            setLogicMode(MANUAL_MODE_M);
			break;
		case MANUAL_TO_AUTO_RUN_T:
			setLogicMode(AUTO_RUN_M);
			break;
		default:
			break;
	}
    
    if (pre_mode != mode)
    {
        pre_mode = mode;
        return true;
    }
    else
    {
        return false;
    }
}


bool RobotMotion::updateLogicState()
{
    static RobotState pre_state;
	RobotState state = getLogicState();
    
    switch (state)
    {
        /*case DISENGAGED_TO_OFF_T:*/
            //setLogicState(OFF_S);
            /*break;*/
        /*case OFF_TO_DISENGAGED_T:            */
            //state = DISENGAGED_S;
            /*break;*/
        /*case OFF_TO_CALIBRATE_T:*/
            //setLogicState(OFF_S);
            //break;
        //case ENGAGED_TO_DISENGAGED_T:                        
            //setLogicState(DISENGAGED_S);
            /*break;*/
        /*case DISENGAGED_TO_ENGAGED_T:*/
            //if (getServoState() == STATE_READY)
            //{
                //setLogicState(ENGAGED_S);
            //}
            //else
            //{
                //setLogicState(DISENGAGED_S);
            //}
            /*break;*/
        case TO_ESTOP_T:
        {
            ProgramState ps = getProgramState();
            FST_INFO("cur ProgramState:%d", ps);
            if ((ps == IDLE_R) 
            || (ps == PAUSED_R))
            {
                setLogicState(ESTOP_S);
            }
            break;
        }
        case RESET_ESTOP_T:
        {            
            //===wait RESET_ERROR_DELAY ms=========================
            static int count = RESET_ERROR_TIMEOUT / INTERVAL_PROPERTY_UPDATE;
            //FST_INFO("the count is :%d", count);
            if (getServoState() != STATE_READY)
            {
                if (isErrorExist() || count <= 1)
                {
                    FST_INFO("cur state:%d", getServoState());
                    share_mem_.stopBareMetal();
                    setLogicState(ESTOP_S);
                    count = RESET_ERROR_TIMEOUT / INTERVAL_PROPERTY_UPDATE;
                    break;
                }
                if (count-- > 0)
                    break;
                count = RESET_ERROR_TIMEOUT / INTERVAL_PROPERTY_UPDATE;
            }

           /*while(1)*/
            //{
                //usleep(10*1000);
                //if (getServoState() == STATE_READY)
                    //break;
                //else
                   //FST_INFO("state:%d",getServoState());
            /*}*/            
            //===================================================
            //FST_INFO("after delay...");
            //if there are still errors can't change state
            //FST_INFO("cur state:%d", getServoState());
            /*if ((!isErrorExist()) //no errors exist*/
            //&& (getServoState() == STATE_READY)) //servo ready
            /*{   */             
            setLogicState(ENGAGED_S);
            /*}*/
            //else
            //{
                //FST_INFO("cur state:%d", getServoState());
                //share_mem_.stopBareMetal();
                //setLogicState(ESTOP_S);
            /*}*/
            break;
        }
        default:
            break;
    }

    if (pre_state != state)
    {
        pre_state = state;
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief: update current joints get value from the share memory 
 *
 * @return: 0 if success
 */
U64 RobotMotion::updateJoints()
{
	FeedbackJointState fbjs;
	JointValues joints_val;

//	int count = 0;
    U64 result = share_mem_.getFeedbackJointState(fbjs);
	if (result == TPI_SUCCESS)
	{
        joints_val.j1 = fbjs.position[0];
        joints_val.j2 = fbjs.position[1];
        joints_val.j3 = fbjs.position[2];
        joints_val.j4 = fbjs.position[3];
        joints_val.j5 = fbjs.position[4];
        joints_val.j6 = fbjs.position[5];

        unsigned int servo_state = getServoState();
        /*static int cnt = 40;*/
        //if (cnt-- <= 0)
        //{
            //FST_INFO("update joints:%d", fbjs.state); 
            //cnt = 40;
        /*}*/
        if (fbjs.state == STATE_READY) 
        {
            if (servo_state == STATE_RUNNING)
            {
                FST_INFO("state translate from running to ready");
            }    

        //	FST_INFO("cur_joints:%f,%f,%f,%f,%f,%f", joints_.j1,joints_.j2,joints_.j3\
                    ,joints_.j4,joints_.j5,joints_.j6);
        }
        else if (fbjs.state == STATE_ERROR)
        {
            if (servo_state == STATE_RUNNING)
            {
                FST_INFO("state translate from running to error");
            }   
        }
        setCurJoints(joints_val);
	}
    setServoState(fbjs.state);
	
	return result;
}

/**
 * @brief: set current joint to library and then get the position
 *
 * @return: true if success
 */
U64 RobotMotion::updatePose()
{
	U64 result;
	PoseEuler pose;
	//boost::recursive_mutex::scoped_lock lock(recu_mutex_);
    boost::mutex::scoped_lock lock(mutex_);
	  //set the current joint to lib
	if (arm_group_->setCurrentJointValues(joints_, result) == false)
    {
		return result;
    }
    
	pose = arm_group_->transformPose2PoseEuler(arm_group_->getCurrentPose()); // get current position from lib
	uintPoseCtle2TP(&pose);
	this->pose_ = pose;
    return TPI_SUCCESS;
}


U64 RobotMotion::updateSafetyStatus()
{
    if (!safety_interface_.isSafetyValid())
    {
        return TPI_SUCCESS;
    }
    if (safety_interface_.getDIExtEStop()
    || safety_interface_.getDILimitedStop()
    || safety_interface_.getDIEStop()
    || safety_interface_.getDIDeadmanPanic()
    || safety_interface_.getDIServoAlarm()
    || safety_interface_.getDISafetyDoorStop()) 
    {        
        //FST_ERROR("detected estop...");
        return SERVO_ESTOP;
    }


    return TPI_SUCCESS;
}



void RobotMotion::setLogicMode(RobotMode mode)
{
    boost::mutex::scoped_lock lock(mutex_);
    this->mode_ = mode;
}

void RobotMotion::setLogicState(RobotState state)
{
    boost::mutex::scoped_lock lock(mutex_);
    /*if (state_ != state)*/
        /*FST_INFO("state_:%d",state);*/
    this->state_ = state;
}

void RobotMotion::setCurJoints(JointValues joints)
{
    boost::mutex::scoped_lock lock(mutex_);
    this->joints_ = joints;
}
void RobotMotion::setCurPose(PoseEuler pose)
{
    boost::mutex::scoped_lock lock(mutex_);
    this->pose_ = pose;
}

U64 RobotMotion::setProgramStateCmd(ProgramStateCmd prgm_cmd)
{
    U64 result = TPI_SUCCESS;
    
    ProgramState prgm_state = getProgramState();
    switch (prgm_cmd)
    {
        case GOTO_IDLE_E:    
            if (prgm_state == EXECUTE_R)
            {
               result = SET_STATE_FAILED; 
            }
            else if (prgm_state == PAUSED_R)
            {
                setProgramState(PAUSE_TO_IDLE_T);                
            }
            break;
        case GOTO_EXECUTE_E:
            if (getLogicState() != ENGAGED_S)
            {
                return SET_STATE_FAILED;
            }
            if (prgm_state == IDLE_R)
            {
                setProgramState(IDLE_TO_EXECUTE_T);
            }
            else if (prgm_state == PAUSED_R)
            {
                setProgramState(PAUSE_TO_EXECUTE_T);
            }
            break;
        case GOTO_PAUSED_E:
            if (getLogicState() != ENGAGED_S)
            {
                //the robot can't be moving in this state
                setProgramState(PAUSED_R); 
                break;
            }
            if (prgm_state == EXECUTE_R)
            {
                setProgramState(EXECUTE_TO_PAUSE_T);
                result = actionPause();
            }
            else if (prgm_state == IDLE_TO_EXECUTE_T)
            {                
                setProgramState(IDLE_R);
                result = actionPause();
            }
            else if (prgm_state == PAUSE_TO_EXECUTE_T)
            {
                setProgramState(PAUSED_R);
                result = actionPause();
            }
            /*else if (prgm_state == IDLE_R)*/
            //{
               //result = SET_STATE_FAILED; 
            /*}*/
            break;
        default:
            break;
    }

    FST_INFO("current Program State:%d", getProgramState());

    return result;
}

/**
 * @brief: set current mode
 *
 * @param: mode_cmd mode command
 *
 * @return: 0 if set successfully
 */
U64 RobotMotion::setLogicModeCmd(RobotModeCmd mode_cmd)
{
    U64 result = TPI_SUCCESS;

    ProgramState prgm_state = getProgramState();
    if ((prgm_state != IDLE_R) && (prgm_state != PAUSED_R))
    {
        FST_ERROR("cant change mode!!");
       return SET_MODE_FAILED;
    }

	RobotMode mode = getLogicMode();

	switch (mode_cmd)
	{
		case GOTO_AUTO_RUN_E:
            if (mode == MANUAL_MODE_M)
            {
                setLogicMode(MANUAL_TO_AUTO_RUN_T);
            }
            else
            {
                setLogicMode(AUTO_RUN_M);
            }
			break;
		case GOTO_MANUAL_MODE_E:
            if (mode == AUTO_RUN_M)
            {
                setLogicMode(AUTO_RUN_TO_MANUAL_T);
            }
            else
            {
                setLogicMode(MANUAL_MODE_M);
            }
			break;
		default:
			result = SET_MODE_FAILED;
			break;
	}

	FST_INFO("current mode:%d", mode);

	return result;
}


/**
 * @brief: set current logic state
 *
 * @param state_cmd: input==> state command
 *
 * @return 
 */
U64 RobotMotion::setLogicStateCmd(RobotStateCmd state_cmd)
{
    U64 result = TPI_SUCCESS;

	RobotState state = getLogicState();
    if ((state == TO_ESTOP_T) 
    || (state == RESET_ESTOP_T))
        return TPI_SUCCESS;
        
    //FST_INFO("state_cmd:%d",state_cmd);
	switch (state_cmd)
	{
        /*case GOTO_OFF:*/
            //if (state == OFF_S)
            //{
                //break;
            //}
            //else if (state == DISENGAGED_S)
            //{
                //setLogicState(DISENGAGED_TO_OFF_T); 
            //}
            //else
            //{
                //result = SET_STATE_FAILED;
            //}
            //break;
        //case GOTO_DISENGAGED_E:
            //if (state == DISENGAGED_S)
            //{
                //break;
            //}
            //else if (state == OFF_S)
            //{
                //setLogicState(OFF_TO_DISENGAGED_T);
            //}
            //else if (state ==  ENGAGED_S)
            //{
                //setLogicState(ENGAGED_TO_DISENGAGED_T);
                //emergencyStop();                
            //}
            //else
            //{
                //result = SET_STATE_FAILED;
            //}
            //break;
        /*case GOTO_ENGAGED_E:*/
            //if (state == ENGAGED_S)
            //{
                //break;
            //}
            //else if (state == LIMITED_RUNNING_S)
            //{
                //setLogicState(DISENGAGED_TO_ENGAGED_T);
                //result = share_mem_.resetBareMetal(); //servo reset
            //}
            //else 
            //{
                //result = SET_STATE_FAILED;
            //}
            /*break;*/
        case EMERGENCY_STOP_E:
            if (state == ESTOP_S)
            {
                break;
            }
            result = emergencyStop();//some status need to set in ENGAGED_S
            setLogicState(TO_ESTOP_T);                        
            break; 
        case ACKNOWLEDGE_ERROR:
            if (state == ENGAGED_S)
            {
                break;
            }
            else if (state == ESTOP_S)
            {
                //FST_INFO("reset controller...");
                setLogicState(RESET_ESTOP_T);
                share_mem_.resetBareMetal();
                safety_interface_.setSoftwareReset(1);  //reset safety board
                usleep(200*1000);       //sleep 200ms for safety reset
                safety_interface_.setSoftwareReset(0);  //clear softwareReset
                clearErrorList(); 
            }
            else 
            {
                result = SET_STATE_FAILED;
            }
            break;
        default:
            break;
    }

    return result;
}

U64 RobotMotion::actionResume()
{
    U64 result = TPI_SUCCESS;
    boost::mutex::scoped_lock lock(mutex_);

	int traj_len = arm_group_->getPlannedPathFIFOLength();
	int joints_len = arm_group_->getJointTrajectoryFIFOLength();
	if ((traj_len != 0) && (joints_len == 0))
	{        
		arm_group_->resumeArmMotion(result);
	}
    //do not process the event when joints_len is not zero

	return result;
}

/**
 * @brief: motion pause
 * @return: 0 if success
 */
U64 RobotMotion::actionPause()
{
    U64 result = TPI_SUCCESS;
	int traj_len = arm_group_->getPlannedPathFIFOLength();
	int joints_len = arm_group_->getJointTrajectoryFIFOLength();
	if ((traj_len != 0) || (joints_len != 0))
	{
		if ((traj_len != 0) && (joints_len == 0))
		{
			FST_ERROR("traj_len is %d, but joints_len is 0 !", traj_len);
			return WRONG_FIFO_STATE;
		}
        FST_INFO("pause:traj_len:%d, joint_len:%d",traj_len, joints_len);             
        
        if (!arm_group_->suspendArmMotion(result))
        {
            FST_ERROR("suspendArmMotion error:%llx", result);
        }

        clearPathFifo();        
        resetInstructionList();
    }

	return result;
}

U64 RobotMotion::servoEStop()
{
    U64 result;
    //stop bare metal, can't write fifo since
    result = share_mem_.stopBareMetal();
    usleep(500* 1000); //wait for brake on
    FST_INFO("wait for brake on");
}


U64 RobotMotion::emergencyStop()
{
    U64 result = TPI_SUCCESS;
    
    setProgramStateCmd(GOTO_PAUSED_E); //set mode to pause
    clearManuMove();
    //clear all the fifos
    arm_group_->resetArmGroup(getCurJointsValue(), result);
    if (result != TPI_SUCCESS)
    {
        FST_ERROR("reset ArmGroup failed:%llx", result);
    }
    share_mem_.setWritenFlag(true); //don't write share_mem any more
          
    //==record current joints for calibrate==
    arm_group_->recordLastJoint(result);
    if (result != TPI_SUCCESS)
    {
        FST_ERROR("recordLastJoint failed:%llx", result);
    }

    return result;
}

U64 RobotMotion::actionShutdown()
{
    return TPI_SUCCESS;
}

U64 RobotMotion::actionCalibrate()
{
    U64 result;    
    unsigned int caliborate_val;
    arm_group_->calibrateZeroOffset(caliborate_val, result); 

    return result;
}

/**
 * @brief: add one command in to queue
 *
 * @param motion_command: input==>the command to push
 */
void RobotMotion::addMotionInstruction(CommandInstruction cmd_instruction)
{
   instruction_list_.push_back(cmd_instruction);
   FST_INFO("instruction num:%d", instruction_list_.size());
}

void RobotMotion::clearInstructionList()
{
    FST_INFO("clear instruction list...");
    while (!instruction_list_.empty())
    {
       instruction_list_.pop_front();
    }
    setPreviousCmdID(-1);
    setCurrentCmdID(-1);
}


/**
 * @brief check the robot joint state judge if they are running
 * @param start_state_cnt: input==>the count of set start state
 *
 * @return: 0 is success
 */
U64 RobotMotion::checkAutoStartState()
{
	U64 result = TPI_SUCCESS;   

    if ((getServoState() == STATE_READY)//ready
    && isFifoEmpty()                    //fifo is empty
    && (instruction_list_.empty()))     //instruction queue empty
    {
        boost::mutex::scoped_lock lock(mutex_);
	    arm_group_->setStartState(getCurJointsValue(), result);
    }

    return result;
}

U64 RobotMotion::checkManualStartState()
{
	U64 result = TPI_SUCCESS;   

    ManualReq manu_req = getManualReq();
    if ((getServoState() == STATE_READY)
    && isFifoEmpty()
    && (manu_req.vel_max == 0))    
    {
        FST_INFO("setStartState...");
       // printDbLine("start joints_", (double*)&joints_,6);
        boost::mutex::scoped_lock lock(mutex_);
	    arm_group_->setStartState(joints_, result);
        manu_refer_joints_ = joints_;
        manu_refer_pose_ = arm_group_->transformPose2PoseEuler(arm_group_->getStartPose());
    }

    return result;
}


bool RobotMotion::isJointsChanged()
{
    static int cnt = 0;
    static JointValues prev_joints = {-1,-1,-1,-1,-1,-1};

    JointValues cur_joints = getCurJointsValue();
    if (compareJoints(prev_joints, cur_joints) == false)
	{
		cnt++;
		prev_joints = cur_joints;
		FST_INFO("cnt:%d, joint:%f,%f,%f,%f,%f,%f", cnt,\
			joints_.j1,joints_.j2,joints_.j3,joints_.j4,joints_.j5,joints_.j6);

        return true;
	}
    
    return false;
}

int RobotMotion::motionHeartBeart(U64 *err_list)
{
    boost::mutex::scoped_lock lock(mutex_);    
    int err_size = share_mem_.monitorHearBeat(err_list);
    U64 result = safety_interface_.setSafetyHeartBeat();
    if (result != TPI_SUCCESS)
    {
        err_list[err_size] = result;
        err_size += 1;        
    }

    return err_size;
}

void RobotMotion::backupError(U64 err)
{    
    bak_err_list_.push_back(err);
    boost::mutex::scoped_lock lock(mutex_);
    prev_err_ = err;
}

void RobotMotion::clearErrorList()
{
    while (!bak_err_list_.empty())
    {
        bak_err_list_.pop_front();
    }

    boost::mutex::scoped_lock lock(mutex_);
    prev_err_ = TPI_SUCCESS;
}

bool RobotMotion::isUpdatedSameError(U64 err)
{
    boost::mutex::scoped_lock lock(mutex_);
    if (prev_err_ != err)
    {
        return false;
    }

    return true;
}

bool RobotMotion::isErrorExist()
{
    boost::mutex::scoped_lock lock(mutex_);
    if (bak_err_list_.empty())
        return false;
    else
        return true;
}

U64 RobotMotion::clearPathFifo()
{
    U64 result = TPI_SUCCESS;

	int traj_len = arm_group_->getPlannedPathFIFOLength();
	//int joints_len = arm_group_->getJointTrajectoryFIFOLength();
//	FST_ERROR("current traj_len:%d, joints_len:%d", traj_len, joints_len);
    
    if ((traj_len != 0) /*|| (joints_len != 0)*/)
    {		
        //maybe wrong when setStartState failed
        FST_ERROR("clear fifo....");
        arm_group_->clearPlannedPathFIFO(result); 
    }

    return result;
}

U64 RobotMotion::checkManualJntVel(const motion_spec_TeachPose *tech_pose)
{
    U64 result;    
    
    JointValues *joints = (JointValues*)tech_pose->pose.coordinates;

    printDbLine("manual joint values:", (double*)joints, MAX_JOINTS);    

    result = checkManualStartState();
    if (result != TPI_SUCCESS)
        return result;

    boost::mutex::scoped_lock lock(mutex_);
    if (tech_pose->has_velocity) //must be step mode
    {
        if (run_mode_ != NORMAL_R)  //only normal mode can use this function
            return INVALID_ACTION_IN_LIMITED_STATE;

        if (tech_pose->velocity <= 0)
            return TPI_SUCCESS;
        manu_req_.target.is_step = true;
        manu_req_.target.type = JOINT_M;
        manu_req_.target.point.jnt_val = *joints;
        manu_req_.vel_max = tech_pose->velocity;
        manu_req_.ref_vmax = 0;
    }
    else
    {
        if ((manu_req_.vel_max > 0) && (manu_req_.target.type == CART_M))
        {
            FST_INFO("manul pose to manual joints...");
            result = getJointFromPose(getCurPosition(), manu_refer_joints_);
            if (result != TPI_SUCCESS)
                return result;
        }

        double interval_jnts[MAX_JOINTS];
        getRadDelta((double*)&manu_refer_joints_, (double*)joints, interval_jnts, MAX_JOINTS);   
        //printDbLine("manu_refer_joints_:", (double*)&manu_refer_joints_, 6);
        //printDbLine("interval_jnts:", interval_jnts, 6);        
        JointValues start_joints = arm_group_->getStartJoint().joints;  //get the start state
        if ((tech_pose->has_step) && (tech_pose->step == true))  //in step mode
        {
            if (manu_req_.target.is_step == false) 
            {
                manu_req_.target.is_step = true;
                if (manu_req_.vel_max > 0) 
                {
                    return TPI_SUCCESS;
                }
                else
                {
                    addTargetVal((double*)&start_joints, (double*)interval_jnts, (double*)&(manu_req_.target.point.jnt_val)); 
                    manu_req_.target.type = JOINT_M;
                    manu_req_.vel_max = calcuManualJointVel((double*)interval_jnts);
                    FST_INFO("max vel is:%f", manu_req_.vel_max);
                }
            }//end if (manu_req_.target.is_step == false)
            else //target is in step mode
            {
                manu_req_.ref_target.is_step = true;
                manu_req_.ref_target.type = JOINT_M;
                if (manu_req_.ref_vmax > 0)
                {
                    return TPI_SUCCESS;
                }
                else
                {
                    addTargetVal((double*)&manu_req_.target.point.jnt_val, (double*)interval_jnts, (double*)&manu_req_.ref_target.point.jnt_val);                
                    manu_req_.ref_vmax = calcuManualJointVel((double*)interval_jnts);
                }
            }
        }//end if ((tech_pose->has_step) && (tech_pose->step == true))    
        else 
        {
            if (manu_req_.vel_max > 0)  //use manu_refer_joints_ instead of start state
            {
                manu_req_.ref_target.is_step = false;
                addTargetVal((double*)&manu_req_.target.point.jnt_val, (double*)interval_jnts, (double*)&manu_req_.ref_target.point.jnt_val); 
                manu_req_.ref_target.type = JOINT_M;
                manu_req_.ref_vmax = calcuManualJointVel((double*)interval_jnts);
                //don't need to set manu_refer_joints_ and manu_refer_pose_
            }
            else
            {
                manu_req_.target.is_step = false;
                manu_req_.target.type = JOINT_M;
                addTargetVal((double*)&start_joints, (double*)interval_jnts, (double*)&manu_req_.target.point.jnt_val); 
                manu_req_.vel_max = calcuManualJointVel((double*)interval_jnts);     
                //===set ref_target================
                manu_req_.ref_target.is_step = false;
                manu_req_.ref_target.type = JOINT_M;
                addTargetVal((double*)&manu_req_.target.point.jnt_val, (double*)interval_jnts, (double*)&manu_req_.ref_target.point.jnt_val);
                manu_req_.ref_vmax = manu_req_.vel_max;
            }
        }//end else
    }//end else

    manu_refer_joints_ = *joints;
    //FST_INFO("step:%d",manu_req_.target.is_step);
    return result;
}

U64 RobotMotion::checkManualPoseVel(const motion_spec_TeachPose *tech_pose)
{
    U64 result;    
    static PoseEuler pre_pose = getCurPosition();
    PoseEuler *pose = (PoseEuler*)tech_pose->pose.coordinates;

    printDbLine("manual pose values:", (double*)pose, 6);  
    
    PoseEuler start_pose =  arm_group_->transformPose2PoseEuler(arm_group_->getStartPose());
    result = checkManualStartState();
    if (result != TPI_SUCCESS)
        return result;
    

    boost::mutex::scoped_lock lock(mutex_);
    if (tech_pose->has_velocity) //must be step mode
    {
        if (tech_pose->velocity <= 0)
            return TPI_SUCCESS;
        PoseEuler tmp_pose = *pose;
        tmp_pose.position.x= pose->position.x * 1000;
        tmp_pose.position.y= pose->position.y * 1000;
        tmp_pose.position.z= pose->position.z * 1000;
        JointValues tmp_joints;
        result = getJointFromPose(tmp_pose, tmp_joints);
        if (result != TPI_SUCCESS)
            return result;
        manu_req_.target.is_step = true;
        manu_req_.target.type = JOINT_M;    
        manu_req_.target.point.jnt_val = tmp_joints;
        manu_req_.vel_max = tech_pose->velocity*1000/MAX_LINE_SPEED*100; //speed to percentage
        manu_req_.ref_vmax = 0;
        manu_refer_pose_ = tmp_pose;
        manu_refer_joints_ = tmp_joints;
        printDbLine("pose:",(double*)&tmp_pose, 6);
        printDbLine("joints:",(double*)&tmp_joints, 6);
        return TPI_SUCCESS;
    }
    if ((manu_req_.vel_max > 0) && (manu_req_.target.type == JOINT_M))
    {
        //====set manu_refer_joints_ and manu_refer_pose_ again======      
        FST_INFO("manul joints to manual pose...");
        result = getPoseFromJoint(getCurJointsValue(), manu_refer_pose_); 
        if (result != TPI_SUCCESS)
            return result;        
    }

    double interval_pose[MAX_JOINTS];
    getLineDelta((double*)&manu_refer_pose_, (double*)pose, interval_pose, 3);
    getRadDelta((double*)&manu_refer_pose_.orientation, (double*)&pose->orientation,\
            &interval_pose[3], 3);
    printDbLine("manu_refer_pose_:", (double*)&manu_refer_pose_, 6);
    printDbLine("interval_pose:", interval_pose, 6);

    if ((tech_pose->has_step) && (tech_pose->step == true))  //in step mode
    {
        if (manu_req_.target.is_step == false) 
        {
            manu_req_.target.is_step = true;
            if (manu_req_.vel_max > 0) 
            {
                manu_req_.ref_vmax = 0;
                return TPI_SUCCESS;
            }
            else
            {
                addTargetVal((double*)&start_pose, (double*)interval_pose, (double*)&manu_req_.target.point.pose_val);
                manu_req_.target.type = CART_M;
                manu_req_.vel_max = calcuManualLineVel((double*)interval_pose);
               // FST_INFO("max vel is:%f", manu_req_.vel_max);                
            }
        }
        else
        {
            manu_req_.ref_target.is_step = true;
            if (manu_req_.ref_vmax > 0)
            {
                return TPI_SUCCESS;
            }
            else
            {     
                addTargetVal((double*)&manu_req_.target.point.pose_val, (double*)interval_pose, (double*)&manu_req_.ref_target.point.pose_val); 
                manu_req_.ref_target.type = CART_M;
                manu_req_.ref_vmax = calcuManualLineVel((double*)interval_pose);
            }
        }
    }
    else
    {
        if (manu_req_.vel_max > 0)  //use manu_refer_joints_ instead of start state
        {
            manu_req_.ref_target.is_step = false;
            addTargetVal((double*)&manu_req_.target.point.pose_val, (double*)interval_pose, (double*)&manu_req_.ref_target.point.pose_val); 
            manu_req_.ref_target.type = CART_M;
            manu_req_.ref_vmax = calcuManualLineVel((double*)interval_pose);
            //don't need to set manu_refer_joints_ and manu_refer_pose_
        }
        else
        {
            manu_req_.target.is_step = false;
            manu_req_.target.type = CART_M;
            addTargetVal((double*)&start_pose, (double*)interval_pose, (double*)&manu_req_.target.point.pose_val); 
            manu_req_.vel_max = calcuManualLineVel((double*)interval_pose);
            //===set ref_target================
            manu_req_.ref_target.is_step = false;
            manu_req_.ref_target.type = CART_M;
            addTargetVal((double*)&manu_req_.target.point.pose_val, (double*)interval_pose, (double*)&manu_req_.ref_target.point.pose_val);
            manu_req_.ref_vmax = manu_req_.vel_max;
        }
    }

    //====set manu_refer_joints_ and manu_refer_pose_ again======
    manu_refer_pose_ = *pose;
    manu_refer_pose_.position.x= pose->position.x * 1000;  //m --> mm
    manu_refer_pose_.position.y= pose->position.y * 1000;
    manu_refer_pose_.position.z= pose->position.z * 1000;

    return result;
}


void RobotMotion::zeroManualReqVel()
{
    boost::mutex::scoped_lock lock(mutex_);
    memset(&manu_req_, 0, sizeof(manu_req_)); 
}

/**
 * @brief check if the joints changed within some values
 *
 * @param input==>src_joints
 * @param input==>dst_joints
 *
 * @return true if they are the same
 */
bool RobotMotion::compareJoints(JointValues src_joints, JointValues dst_joints)
{
	if (fabs(src_joints.j1 - dst_joints.j1) > MIN_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j2 - dst_joints.j2) > MIN_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j3 - dst_joints.j3) > MIN_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j4 - dst_joints.j4) > MIN_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j5 - dst_joints.j5) > MIN_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j6 - dst_joints.j6) > MIN_ACCURATE_VALUE) return false;

	return true;
}	

U64 RobotMotion::getPoseFromJoint(const JointValues &joints, PoseEuler &pose)
{
    U64 result;
    Pose p;
    if (arm_group_->computeFK(joints, p, result))        
        pose = arm_group_->transformPose2PoseEuler(p);

    return result;
}

U64 RobotMotion::getJointFromPose(const PoseEuler &pose, JointValues &joints)
{
    U64 result;
    Pose p = arm_group_->transformPoseEuler2Pose(pose);
    
    arm_group_->computeIK(p, joints, result);
    //printDbLine("new is:", (double*)&joints, 6);
    if (result != TPI_SUCCESS)
    {
        if (result == IK_EXCESSIVE_DISTANCE) //need to be changed later
            return TPI_SUCCESS;
        else
        {
            FST_ERROR("computeIK failed");
            return result;
        }
    }

    //return TPI_SUCCESS;
}

U64 RobotMotion::getUserFrame(motion_spec_userFrame *user_frame)
{
    return TPI_SUCCESS;
}

U64 RobotMotion::setUserFrame(motion_spec_userFrame *user_frame)
{
    Transformation transform;
    transform.position.x = user_frame->X;
    transform.position.y = user_frame->Y;
    transform.position.z = user_frame->Z;
    
    transform.orientation.a = user_frame->A;
    transform.orientation.b = user_frame->B;
    transform.orientation.c = user_frame->C;

    arm_group_->setUserFrame(transform);

    return TPI_SUCCESS;
}

U64 RobotMotion::getToolFrame(motion_spec_toolFrame *tool_frame)
{
    return TPI_SUCCESS;
}

U64 RobotMotion::setToolFrame(motion_spec_toolFrame *tool_frame)
{
    Transformation transform;
    transform.position.x = tool_frame->X;
    transform.position.y = tool_frame->Y;
    transform.position.z = tool_frame->Z;
    
    transform.orientation.a = tool_frame->A;
    transform.orientation.b = tool_frame->B;
    transform.orientation.c = tool_frame->C;

    arm_group_->setToolFrame(transform);    

    return TPI_SUCCESS;
}


U64 RobotMotion::getJointConstraint(motion_spec_JointConstraint *jnt_constraint)
{
    JointConstraints jc;
    jnt_constraint->jnt_lmt_count = MAX_JOINTS;

    jnt_constraint->jnt_lmt[0].has_zero = true;
    jnt_constraint->jnt_lmt[0].zero = jc.j1.home;
    jnt_constraint->jnt_lmt[0].has_upper = true;
    jnt_constraint->jnt_lmt[0].upper = jc.j1.upper;
    jnt_constraint->jnt_lmt[0].has_lower = true;
    jnt_constraint->jnt_lmt[0].lower = jc.j1.lower;
    jnt_constraint->jnt_lmt[0].has_max_omega = true;
    jnt_constraint->jnt_lmt[0].max_omega = jc.j1.max_omega;
    jnt_constraint->jnt_lmt[0].has_max_alpha = true;
    jnt_constraint->jnt_lmt[0].max_alpha = jc.j1.max_alpha;

    jnt_constraint->jnt_lmt[1].has_zero = true;
    jnt_constraint->jnt_lmt[1].zero = jc.j2.home;
    jnt_constraint->jnt_lmt[1].has_upper = true;
    jnt_constraint->jnt_lmt[1].upper = jc.j2.upper;
    jnt_constraint->jnt_lmt[1].has_lower = true;
    jnt_constraint->jnt_lmt[1].lower = jc.j2.lower;
    jnt_constraint->jnt_lmt[1].has_max_omega = true;
    jnt_constraint->jnt_lmt[1].max_omega = jc.j2.max_omega;
    jnt_constraint->jnt_lmt[1].has_max_alpha = true;
    jnt_constraint->jnt_lmt[1].max_alpha = jc.j2.max_alpha;

    jnt_constraint->jnt_lmt[2].has_zero = true;
    jnt_constraint->jnt_lmt[2].zero = jc.j3.home;
    jnt_constraint->jnt_lmt[2].has_upper = true;
    jnt_constraint->jnt_lmt[2].upper = jc.j3.upper;
    jnt_constraint->jnt_lmt[2].has_lower = true;
    jnt_constraint->jnt_lmt[2].lower = jc.j3.lower;
    jnt_constraint->jnt_lmt[2].has_max_omega = true;
    jnt_constraint->jnt_lmt[2].max_omega = jc.j3.max_omega;
    jnt_constraint->jnt_lmt[2].has_max_alpha = true;
    jnt_constraint->jnt_lmt[2].max_alpha = jc.j3.max_alpha;

    jnt_constraint->jnt_lmt[3].has_zero = true;
    jnt_constraint->jnt_lmt[3].zero = jc.j4.home;
    jnt_constraint->jnt_lmt[3].has_upper = true;
    jnt_constraint->jnt_lmt[3].upper = jc.j4.upper;
    jnt_constraint->jnt_lmt[3].has_lower = true;
    jnt_constraint->jnt_lmt[3].lower = jc.j4.lower;
    jnt_constraint->jnt_lmt[3].has_max_omega = true;
    jnt_constraint->jnt_lmt[3].max_omega = jc.j4.max_omega;
    jnt_constraint->jnt_lmt[3].has_max_alpha = true;
    jnt_constraint->jnt_lmt[3].max_alpha = jc.j4.max_alpha;

    jnt_constraint->jnt_lmt[4].has_zero = true;
    jnt_constraint->jnt_lmt[4].zero = jc.j5.home;
    jnt_constraint->jnt_lmt[4].has_upper = true;
    jnt_constraint->jnt_lmt[4].upper = jc.j5.upper;
    jnt_constraint->jnt_lmt[4].has_lower = true;
    jnt_constraint->jnt_lmt[4].lower = jc.j5.lower;
    jnt_constraint->jnt_lmt[4].has_max_omega = true;
    jnt_constraint->jnt_lmt[4].max_omega = jc.j5.max_omega;
    jnt_constraint->jnt_lmt[4].has_max_alpha = true;
    jnt_constraint->jnt_lmt[4].max_alpha = jc.j5.max_alpha;

    jnt_constraint->jnt_lmt[5].has_zero = true;
    jnt_constraint->jnt_lmt[5].zero = jc.j6.home;
    jnt_constraint->jnt_lmt[5].has_upper = true;
    jnt_constraint->jnt_lmt[5].upper = jc.j6.upper;
    jnt_constraint->jnt_lmt[5].has_lower = true;
    jnt_constraint->jnt_lmt[5].lower = jc.j6.lower;
    jnt_constraint->jnt_lmt[5].has_max_omega = true;
    jnt_constraint->jnt_lmt[5].max_omega = jc.j6.max_omega;
    jnt_constraint->jnt_lmt[5].has_max_alpha = true;
    jnt_constraint->jnt_lmt[5].max_alpha = jc.j6.max_alpha;

    return TPI_SUCCESS;
}
U64 RobotMotion::setJointConstraint(motion_spec_JointConstraint *jnt_constraint)
{
    JointConstraints jc;
    if (jnt_constraint->jnt_lmt_count < 6)
        return INVALID_PARAM_FROM_TP;

    if (jnt_constraint->jnt_lmt[0].has_zero)
        jc.j1.home = jnt_constraint->jnt_lmt[0].zero;
    if (jnt_constraint->jnt_lmt[0].has_upper)
        jc.j1.upper = jnt_constraint->jnt_lmt[0].upper;
    if (jnt_constraint->jnt_lmt[0].has_lower)
        jc.j1.lower = jnt_constraint->jnt_lmt[0].lower;
    if (jnt_constraint->jnt_lmt[0].has_max_omega)
        jc.j1.max_omega = jnt_constraint->jnt_lmt[0].max_omega;
    if (jnt_constraint->jnt_lmt[0].has_max_alpha)
        jc.j1.max_alpha = jnt_constraint->jnt_lmt[0].max_alpha;

    if (jnt_constraint->jnt_lmt[1].has_zero)
        jc.j2.home = jnt_constraint->jnt_lmt[1].zero;
    if (jnt_constraint->jnt_lmt[1].has_upper)
        jc.j2.upper = jnt_constraint->jnt_lmt[1].upper;
    if (jnt_constraint->jnt_lmt[1].has_lower)
        jc.j2.lower = jnt_constraint->jnt_lmt[1].lower;
    if (jnt_constraint->jnt_lmt[1].has_max_omega)
        jc.j2.max_omega = jnt_constraint->jnt_lmt[1].max_omega;
    if (jnt_constraint->jnt_lmt[1].has_max_alpha)
        jc.j2.max_alpha = jnt_constraint->jnt_lmt[1].max_alpha;

    if (jnt_constraint->jnt_lmt[2].has_zero)
        jc.j3.home = jnt_constraint->jnt_lmt[2].zero;
    if (jnt_constraint->jnt_lmt[2].has_upper)
        jc.j3.upper = jnt_constraint->jnt_lmt[2].upper;
    if (jnt_constraint->jnt_lmt[2].has_lower)
        jc.j3.lower = jnt_constraint->jnt_lmt[2].lower;
    if (jnt_constraint->jnt_lmt[2].has_max_omega)
        jc.j3.max_omega = jnt_constraint->jnt_lmt[2].max_omega;
    if (jnt_constraint->jnt_lmt[2].has_max_alpha)
        jc.j3.max_alpha = jnt_constraint->jnt_lmt[2].max_alpha;

    if (jnt_constraint->jnt_lmt[3].has_zero)
        jc.j4.home = jnt_constraint->jnt_lmt[3].zero;
    if (jnt_constraint->jnt_lmt[3].has_upper)
        jc.j4.upper = jnt_constraint->jnt_lmt[3].upper;
    if (jnt_constraint->jnt_lmt[3].has_lower)
        jc.j4.lower = jnt_constraint->jnt_lmt[3].lower;
    if (jnt_constraint->jnt_lmt[3].has_max_omega)
        jc.j4.max_omega = jnt_constraint->jnt_lmt[3].max_omega;
    if (jnt_constraint->jnt_lmt[3].has_max_alpha)
        jc.j4.max_alpha = jnt_constraint->jnt_lmt[3].max_alpha;

    if (jnt_constraint->jnt_lmt[4].has_zero)
        jc.j5.home = jnt_constraint->jnt_lmt[4].zero;
    if (jnt_constraint->jnt_lmt[4].has_upper)
        jc.j5.upper = jnt_constraint->jnt_lmt[4].upper;
    if (jnt_constraint->jnt_lmt[4].has_lower)
        jc.j5.lower = jnt_constraint->jnt_lmt[4].lower;
    if (jnt_constraint->jnt_lmt[4].has_max_omega)
        jc.j5.max_omega = jnt_constraint->jnt_lmt[4].max_omega;
    if (jnt_constraint->jnt_lmt[4].has_max_alpha)
        jc.j5.max_alpha = jnt_constraint->jnt_lmt[4].max_alpha;

    if (jnt_constraint->jnt_lmt[5].has_zero)
        jc.j6.home = jnt_constraint->jnt_lmt[5].zero;
    if (jnt_constraint->jnt_lmt[5].has_upper)
        jc.j6.upper = jnt_constraint->jnt_lmt[5].upper;
    if (jnt_constraint->jnt_lmt[5].has_lower)
        jc.j6.lower = jnt_constraint->jnt_lmt[5].lower;
    if (jnt_constraint->jnt_lmt[5].has_max_omega)
        jc.j6.max_omega = jnt_constraint->jnt_lmt[5].max_omega;
    if (jnt_constraint->jnt_lmt[5].has_max_alpha)
        jc.j6.max_alpha = jnt_constraint->jnt_lmt[5].max_alpha;

    return TPI_SUCCESS;
}

U64 RobotMotion::getDHGroup(motion_spec_DHGroup *dh_group)
{
    DHGroup dh;
    dh_group->coord_offset_count = 6;
    dh_group->coord_offset[0].alpha = dh.j1.alpha;
    dh_group->coord_offset[0].a = dh.j1.a;
    dh_group->coord_offset[0].d = dh.j1.d;
    dh_group->coord_offset[0].theta = dh.j1.theta;

    dh_group->coord_offset[1].alpha = dh.j2.alpha;
    dh_group->coord_offset[1].a = dh.j2.a;
    dh_group->coord_offset[1].d = dh.j2.d;
    dh_group->coord_offset[1].theta = dh.j2.theta;

    dh_group->coord_offset[2].alpha = dh.j3.alpha;
    dh_group->coord_offset[2].a = dh.j3.a;
    dh_group->coord_offset[2].d = dh.j3.d;
    dh_group->coord_offset[2].theta = dh.j3.theta;

    dh_group->coord_offset[3].alpha = dh.j4.alpha;
    dh_group->coord_offset[3].a = dh.j4.a;
    dh_group->coord_offset[3].d = dh.j4.d;
    dh_group->coord_offset[3].theta = dh.j4.theta;
    
    dh_group->coord_offset[4].alpha = dh.j5.alpha;
    dh_group->coord_offset[4].a = dh.j5.a;
    dh_group->coord_offset[4].d = dh.j5.d;
    dh_group->coord_offset[4].theta = dh.j5.theta;

    dh_group->coord_offset[5].alpha = dh.j6.alpha;
    dh_group->coord_offset[5].a = dh.j6.a;
    dh_group->coord_offset[5].d = dh.j6.d;
    dh_group->coord_offset[5].theta = dh.j6.theta;

    return TPI_SUCCESS;
}
U64 RobotMotion::setDHGroup(motion_spec_DHGroup *dh_group)
{
    DHGroup dh;
    dh.j1.alpha = dh_group->coord_offset[0].alpha;
    dh.j1.a = dh_group->coord_offset[0].a;
    dh.j1.d = dh_group->coord_offset[0].d;
    dh.j1.theta = dh_group->coord_offset[0].theta;

    dh.j2.alpha = dh_group->coord_offset[1].alpha;
    dh.j2.a = dh_group->coord_offset[1].a;
    dh.j2.d = dh_group->coord_offset[1].d;
    dh.j2.theta = dh_group->coord_offset[1].theta;

    dh.j3.alpha = dh_group->coord_offset[2].alpha;
    dh.j3.a = dh_group->coord_offset[2].a;
    dh.j3.d = dh_group->coord_offset[2].d;
    dh.j3.theta = dh_group->coord_offset[2].theta;

    dh.j4.alpha = dh_group->coord_offset[3].alpha;
    dh.j4.a = dh_group->coord_offset[3].a;
    dh.j4.d = dh_group->coord_offset[3].d;
    dh.j4.theta = dh_group->coord_offset[3].theta;

    dh.j5.alpha = dh_group->coord_offset[4].alpha;
    dh.j5.a = dh_group->coord_offset[4].a;
    dh.j5.d = dh_group->coord_offset[4].d;
    dh.j5.theta = dh_group->coord_offset[4].theta;

    dh.j6.alpha = dh_group->coord_offset[5].alpha;
    dh.j6.a = dh_group->coord_offset[5].a;
    dh.j6.d = dh_group->coord_offset[5].d;
    dh.j6.theta = dh_group->coord_offset[5].theta;

    return TPI_SUCCESS;
}

U64 RobotMotion::getHWLimit(double *hw_limit)
{
   return TPI_SUCCESS; 
}

U64 RobotMotion::setHWLimit(double *hw_limit)
{
    return TPI_SUCCESS;
}

double RobotMotion::getGlobalVelocity()
{
    boost::mutex::scoped_lock lock(mutex_);
    return vel_factor_;
}
U64 RobotMotion::setGlobalVelocity(double factor)
{
    if ((factor <= 0) || (factor > 100))
        return INVALID_PARAM_FROM_TP;

    boost::mutex::scoped_lock lock(mutex_);
    vel_factor_ = factor;

    return TPI_SUCCESS;
}


U64 setTempZero()
{
    return TPI_SUCCESS;
}

/**
 * @brief: convert unit from params of TP to controller
 *
 * @param src_moveL: input==>st\ruct  covert from
 * @param dst_pose: output==>struct covert to
 * @param dst_movel_param: output==> struct covert to
 */
void RobotMotion::unitConvert(const motion_spec_MoveL *src_moveL, PoseEuler& dst_pose, MoveLParam &dst_movel_param)
{
	motion_spec_WayPoint way_point = src_moveL->waypoints[0];
	motion_spec_Pose pose = way_point.pose;
	dst_pose.position.x = pose.coordinates[0] * 1000;
	dst_pose.position.y = pose.coordinates[1] * 1000;
	dst_pose.position.z = pose.coordinates[2] * 1000;
	dst_pose.orientation.a = pose.coordinates[3];
	dst_pose.orientation.b = pose.coordinates[4];
	dst_pose.orientation.c = pose.coordinates[5];

	dst_movel_param.vel_max = src_moveL->vMax * getGlobalVelocity() * 10; //global_vel/100*1000
	dst_movel_param.acc_max = src_moveL->aMax * 1000;

	dst_movel_param.smooth = way_point.smoothPercent;
}
/**
 * @brief convert unit from params of TP to controller
 *
 * @param src_moveL: input==>struct  covert from
 * @param dst_pose: output==>struct covert to
 * @param dst_movel_param: output==>struct covert to
 */
void RobotMotion::unitConvert(const motion_spec_MoveJ *src_moveJ, JointValues &dst_joints, MoveJParam &dst_movej_param)
{
	dst_joints.j1 = src_moveJ->targetJointCoordinates[0];
	dst_joints.j2 = src_moveJ->targetJointCoordinates[1];
	dst_joints.j3 = src_moveJ->targetJointCoordinates[2];
	dst_joints.j4 = src_moveJ->targetJointCoordinates[3];
	dst_joints.j5 = src_moveJ->targetJointCoordinates[4];
	dst_joints.j6 = src_moveJ->targetJointCoordinates[5];

	dst_movej_param.vel_max = src_moveJ->vMax * getGlobalVelocity() / 100; //percent
	dst_movej_param.acc_max = src_moveJ->aMax;
	
	dst_movej_param.smooth = src_moveJ->smoothPercent;
}

void RobotMotion::unitConvert(const motion_spec_MoveC *moveC, PoseEuler& pose1, PoseEuler& pose2, MoveCParam &moveC_param)
{
    pose1.position.x = moveC->pose1.coordinates[0] * 1000;
    pose1.position.y = moveC->pose1.coordinates[1] * 1000;
    pose1.position.z = moveC->pose1.coordinates[2] * 1000;
    pose1.orientation.a = moveC->pose1.coordinates[3];
    pose1.orientation.b = moveC->pose1.coordinates[4];
    pose1.orientation.c = moveC->pose1.coordinates[5];

    pose2.position.x = moveC->pose2.coordinates[0] * 1000;
    pose2.position.y = moveC->pose2.coordinates[1] * 1000;
    pose2.position.z = moveC->pose2.coordinates[2] * 1000;
    pose2.orientation.a = moveC->pose2.coordinates[3];
    pose2.orientation.b = moveC->pose2.coordinates[4];
    pose2.orientation.c = moveC->pose2.coordinates[5];

    moveC_param.vel_max = moveC->vMax * getGlobalVelocity() * 10; //global_vel/100*1000
    moveC_param.acc_max = moveC->aMax * 1000;
    moveC_param.smooth = moveC->smoothPercent;
}

/**
 * @brief get max value from a buffer
 *
 * @param buffer: input
 * @param length: input
 *
 * @return :the max value
 */
/*double RobotMotion::getMaxValue(const double *buffer, int length, int &id)*/
//{
	//double cmp1, cmp2;

	//double max = 0;

	//for (int i = 0; i < length; i++)
	//{
		//cmp1 = max;
		//cmp2 = fabs(buffer[i]);

		//if (cmp1 <= cmp2)
        //{
            //id = i;
            //max = cmp2;
        //}
        //else
        //{
            //max = cmp1;
        //}
    //}

	
	//return max;
/*}*/
double RobotMotion::getMaxValue(const double *buffer, int length)
{
	double cmp1, cmp2;
	cmp1 = fabs(buffer[0]);
	cmp2 = fabs(buffer[1]);

	double max = MAX(cmp1, cmp2);

	for (int i = 2; i < length; i++)
	{
		cmp1 = max;
		cmp2 = fabs(buffer[i]);

		max = MAX(cmp1, cmp2);
	}

	
	return max;
}
/**
 * @brief: get the ID of max value 
 *
 * @param max: input==>the max value
 * @param buffer: input
 * @param length: input
 *
 * @return: the ID 
 */
int RobotMotion::getMaxValID(double max, const double *buffer, int length)
{
	int id;
	for (int i = 0; i < length; i++)
	{
		if (max == fabs(buffer[i]))
		{
			id = i;
			break;
		}
	}

	return id;
}


/**
 * @brief: get the max interval value between two pose
 *
 * @param pre_pose: input
 * @param cur_pose: input
 *
 * @return: the max value
 */
double RobotMotion::getMaxLineInterval(PoseEuler pre_pose, PoseEuler cur_pose)
{
	double interval[3];
	interval[0] = fabs(pre_pose.position.x - cur_pose.position.x);
	interval[1] = fabs(pre_pose.position.y - cur_pose.position.y);
	interval[2] = fabs(pre_pose.position.z - cur_pose.position.z);

	return getMaxValue(interval, 3);
}

double getRotInterval(double data1, double data2)
{
    double tmp_interval = fabs(data1 - data2);
    while(tmp_interval > PI)
    {
        tmp_interval = fabs(tmp_interval - 2*PI);
    }
    return tmp_interval;
}

double RobotMotion::getMaxRotateInterval(PoseEuler pre_pose, PoseEuler cur_pose)
{

	double interval[3];
	interval[0] = getRotInterval(pre_pose.orientation.a , cur_pose.orientation.a);
	interval[1] = getRotInterval(pre_pose.orientation.b , cur_pose.orientation.b);
	interval[2] = getRotInterval(pre_pose.orientation.c , cur_pose.orientation.c);

    FST_INFO("pre:a:%f,%f,%f,",pre_pose.orientation.a, pre_pose.orientation.b, pre_pose.orientation.c);
    FST_INFO("post:a:%f,%f,%f",cur_pose.orientation.a, cur_pose.orientation.b, cur_pose.orientation.c);
    FST_INFO("interval:%f,%f,%f", interval[0], interval[1], interval[2]);

	return getMaxValue(interval, 3);
}



/**
 * @brief: convert unit from params of TP to controller
 *
 * @param pose: input&output
 */
void RobotMotion::uintPoseTP2Ctle(PoseEuler *pose)
{
	//printf("unit convert==>tp2ctrl:%f,%f,%f\n",pose->position.x,pose->position.y,pose->position.z);
    pose->position.x = pose->position.x * 1000;
	pose->position.y = pose->position.y * 1000;
	pose->position.z = pose->position.z * 1000;

}
/**
 * @brief: convert unit from params of TP to controller
 *
 * @param pose: input&output
 */
void RobotMotion::uintPoseCtle2TP(PoseEuler *pose)
{
//	printf("unit convert==>ctrl2tp:%d,%d,%d\n",pose->position.x,pose->position.y,pose->position.z);
    pose->position.x = pose->position.x / 1000;
	pose->position.y = pose->position.y / 1000;
	pose->position.z = pose->position.z / 1000;

}


U64 RobotMotion::moveInstructions(CommandInstruction target_inst)
{
    U64 result;
    
    switch (target_inst.commandtype)
    {
        case motion_spec_MOTIONTYPE_JOINTMOTION:
        {                
            motion_spec_MoveJ* moveJ =  (motion_spec_MoveJ*)target_inst.command_arguments.c_str();
            moveJ->smoothPercent = target_inst.smoothDistance; //first rewrite this value
            JointValues target_jnts;
            MoveJParam movej_param;
            unitConvert(moveJ, target_jnts, movej_param);
            FST_INFO("target joints:%f,%f,%f,%f,%f,%f", target_jnts.j1, target_jnts.j2, target_jnts.j3, target_jnts.j4, target_jnts.j5, target_jnts.j6);                
            if (target_inst.smoothDistance >= 0)
            {
                JointValues start_jnts = arm_group_->getStartJoint().joints;  //get the start state
                JointValues new_target;
                new_target.j1 = (start_jnts.j1 + target_jnts.j1) / 2;
                new_target.j2 = (start_jnts.j2 + target_jnts.j2) / 2;
                new_target.j3 = (start_jnts.j3 + target_jnts.j3) / 2;
                new_target.j4 = (start_jnts.j4 + target_jnts.j4) / 2;
                new_target.j5 = (start_jnts.j5 + target_jnts.j5) / 2;
                new_target.j6 = (start_jnts.j6 + target_jnts.j6) / 2;

                arm_group_->MoveJ(new_target,movej_param.vel_max, movej_param.acc_max,\
                        MAX_CNT_VAL, target_jnts, movej_param.vel_max, movej_param.acc_max,\
                        MAX_CNT_VAL, target_inst.id, result);

                setServoWaitFlag(false);
            }//end if (target_inst.smoothDistance >= 0)
            else
            {
                arm_group_->MoveJ(target_jnts, movej_param.vel_max, \
                            movej_param.acc_max, target_inst.id, result);
                setServoWaitFlag(true);

            }//end else
            break;
        }
        case motion_spec_MOTIONTYPE_CARTMOTION:
        {
            motion_spec_MoveL* moveL = (motion_spec_MoveL*)target_inst.command_arguments.c_str();
            moveL->waypoints[0].smoothPercent = target_inst.smoothDistance; //first rewrite this value
            PoseEuler start_pose =  arm_group_->transformPose2PoseEuler(arm_group_->getStartPose());
            printDbLine("start_pose:", (double*)&start_pose, 6);
            PoseEuler target_pose;
            MoveLParam movel_param;
            unitConvert(moveL, target_pose, movel_param);
            FST_INFO("target pose:%f,%f,%f,%f,%f,%f", target_pose.position.x,target_pose.position.y,target_pose.position.z, target_pose.orientation.a, target_pose.orientation.b, target_pose.orientation.c);
            if (target_inst.smoothDistance >= 0)
            {
                PoseEuler new_target;               
                new_target.position.x = (start_pose.position.x + target_pose.position.x) / 2;
                new_target.position.y = (start_pose.position.y + target_pose.position.y) / 2;
                new_target.position.z = (start_pose.position.z + target_pose.position.z) / 2;
                double tmp1,tmp2 = 0;
                tmp1 = start_pose.orientation.a + target_pose.orientation.a;
                tmp2 = get2PIDeltaValue(start_pose.orientation.a, target_pose.orientation.a);
                new_target.orientation.a = (tmp1 - tmp2) / 2;
                tmp1 = start_pose.orientation.b + target_pose.orientation.b;
                tmp2 = get2PIDeltaValue(start_pose.orientation.b, target_pose.orientation.b);
                new_target.orientation.b = (tmp1 - tmp2) / 2;
                tmp1 = start_pose.orientation.c + target_pose.orientation.c;
                tmp2 = get2PIDeltaValue(start_pose.orientation.c, target_pose.orientation.c);
                new_target.orientation.c = (tmp1 - tmp2) / 2;
                
                printDbLine("new_target:", (double*)&new_target, 6);
                arm_group_->MoveL(new_target, movel_param.vel_max, movel_param.acc_max,\
                        MAX_CNT_VAL, target_pose, movel_param.vel_max, movel_param.acc_max,\
                       MAX_CNT_VAL, target_inst.id, result);

                setServoWaitFlag(false);
            }//end if (target_inst.smoothDistance >= 0)
            else
            {
                arm_group_->MoveL(target_pose, movel_param.vel_max, \
                            movel_param.acc_max, target_inst.id, result);
                setServoWaitFlag(true);
            }
            break;
        }
        case motion_spec_MOTIONTYPE_CIRCLEMOTION:
        {
            motion_spec_MoveC *moveC = (motion_spec_MoveC*)target_inst.command_arguments.c_str();
            PoseEuler target_pose1, target_pose2;
            MoveCParam target_param;
            unitConvert(moveC, target_pose1, target_pose2, target_param); 
            printDbLine("movec pose2:", (double*)&target_pose2, 6);
            arm_group_->MoveC(target_pose1, target_pose2, target_param.vel_max, \
                            target_param.acc_max, target_inst.id, result);
            setServoWaitFlag(true);
            break;
        }
        default:
            break;
    }

    return result;
}

U64 RobotMotion::moveInstructions(CommandInstruction target_inst, CommandInstruction next_inst)
{
    U64 result;
    
    switch (target_inst.commandtype)
    {
        case motion_spec_MOTIONTYPE_JOINTMOTION:
        {                
            motion_spec_MoveJ* moveJ =  (motion_spec_MoveJ*)target_inst.command_arguments.c_str();
            moveJ->smoothPercent = target_inst.smoothDistance; //first rewrite this value
            JointValues target_jnts;
            MoveJParam target_param;
            unitConvert(moveJ, target_jnts, target_param);
            FST_INFO("target joints:%f,%f,%f,%f,%f,%f, smmoth:%f", target_jnts.j1, target_jnts.j2, target_jnts.j3, target_jnts.j4, target_jnts.j5, target_jnts.j6, target_inst.smoothDistance);     
            
            if (next_inst.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
            {
                motion_spec_MoveJ* moveJ_next =  (motion_spec_MoveJ*)next_inst.command_arguments.c_str();
                JointValues next_jnts;
                MoveJParam next_param;
                unitConvert(moveJ_next, next_jnts, next_param);

                arm_group_->MoveJ(target_jnts,target_param.vel_max, target_param.acc_max,\
                        target_param.smooth, next_jnts, next_param.vel_max, next_param.acc_max,\
                        next_param.smooth, target_inst.id, result);
            }
            else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
            {
                motion_spec_MoveL* moveL_next =  (motion_spec_MoveL*)next_inst.command_arguments.c_str();
                PoseEuler next_pose;
                MoveLParam next_param;
                unitConvert(moveL_next, next_pose, next_param);

                arm_group_->MoveJ(target_jnts,target_param.vel_max, target_param.acc_max,\
                        target_param.smooth, next_pose, next_param.vel_max, next_param.acc_max,\
                        next_param.smooth, target_inst.id, result);
            }      
            else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION)
            {
                motion_spec_MoveC *moveC_next = (motion_spec_MoveC*)next_inst.command_arguments.c_str();
                PoseEuler next_pose1, next_pose2;
                MoveCParam next_param;
                unitConvert(moveC_next, next_pose1, next_pose2, next_param); 
                arm_group_->MoveJ(target_jnts,target_param.vel_max, target_param.acc_max,\
                        target_param.smooth, next_pose1, next_pose2, next_param.vel_max, \
                        next_param.acc_max, next_param.smooth, target_inst.id, result);
            }

            setServoWaitFlag(false);
            break;
        }//end case motion_spec_MOTIONTYPE_JOINTMOTION:     
        case motion_spec_MOTIONTYPE_CARTMOTION:
        {
            motion_spec_MoveL* moveL = (motion_spec_MoveL*)target_inst.command_arguments.c_str();
            moveL->waypoints[0].blendInDistance = target_inst.smoothDistance; //first rewrite this value
            PoseEuler start_pose =  arm_group_->transformPose2PoseEuler(arm_group_->getStartPose());
            PoseEuler target_pose;
            MoveLParam target_param;
            unitConvert(moveL, target_pose, target_param);
            FST_INFO("target pose:%f,%f,%f,%f,%f,%f,smooth:%f", target_pose.position.x,target_pose.position.y,target_pose.position.z, target_pose.orientation.a, target_pose.orientation.b, target_pose.orientation.c, target_inst.smoothDistance);
            if (next_inst.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
            {
                motion_spec_MoveJ* moveJ_next =  (motion_spec_MoveJ*)next_inst.command_arguments.c_str();
                JointValues next_jnts;
                MoveJParam next_param;
                unitConvert(moveJ_next, next_jnts, next_param);

                arm_group_->MoveL(target_pose,target_param.vel_max, target_param.acc_max,\
                        target_param.smooth, next_jnts, next_param.vel_max, next_param.acc_max,\
                        next_param.smooth, target_inst.id, result);
            }
            else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
            {
                motion_spec_MoveL* moveL_next =  (motion_spec_MoveL*)next_inst.command_arguments.c_str();
                PoseEuler next_pose;
                MoveLParam next_param;
                unitConvert(moveL_next, next_pose, next_param);

                arm_group_->MoveL(target_pose,target_param.vel_max, target_param.acc_max,\
                        target_param.smooth, next_pose, next_param.vel_max, next_param.acc_max,\
                        next_param.smooth, target_inst.id, result);
            }  
            else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION)
            {
                motion_spec_MoveC *moveC_next = (motion_spec_MoveC*)next_inst.command_arguments.c_str();
                PoseEuler next_pose1, next_pose2;
                MoveCParam next_param;
                unitConvert(moveC_next, next_pose1, next_pose2, next_param); 
                arm_group_->MoveL(target_pose,target_param.vel_max, target_param.acc_max,\
                        target_param.smooth,next_pose1, next_pose2, next_param.vel_max, \
                        next_param.acc_max, next_param.smooth, target_inst.id, result);
            }
            setServoWaitFlag(false);
            break;
        }//end case motion_spec_MOTIONTYPE_CARTMOTION:
        case motion_spec_MOTIONTYPE_CIRCLEMOTION:
        {
            motion_spec_MoveC *moveC = (motion_spec_MoveC*)target_inst.command_arguments.c_str();
            PoseEuler target_pose1, target_pose2;
            MoveCParam target_param;
            unitConvert(moveC, target_pose1, target_pose2, target_param); 
            printDbLine("movec pose2:", (double*)&target_pose2, 6);
            
            if (next_inst.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
            {
                motion_spec_MoveJ* moveJ_next =  (motion_spec_MoveJ*)next_inst.command_arguments.c_str();
                JointValues next_jnts;
                MoveJParam next_param;
                unitConvert(moveJ_next, next_jnts, next_param);
                arm_group_->MoveC(target_pose1, target_pose2, target_param.vel_max, \
                        target_param.acc_max, target_param.smooth, next_jnts, next_param.vel_max,\
                        next_param.acc_max, next_param.smooth, target_inst.id, result);             
            }
            else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
            {
                motion_spec_MoveL* moveL_next =  (motion_spec_MoveL*)next_inst.command_arguments.c_str();
                PoseEuler next_pose;
                MoveLParam next_param;
                unitConvert(moveL_next, next_pose, next_param);
                arm_group_->MoveC(target_pose1, target_pose2, target_param.vel_max, \
                        target_param.acc_max, target_param.smooth, next_pose, next_param.vel_max,\
                        next_param.acc_max, next_param.smooth, target_inst.id, result);           }  
            else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION)
            {
                motion_spec_MoveC *moveC_next = (motion_spec_MoveC*)next_inst.command_arguments.c_str();
                PoseEuler next_pose1, next_pose2;
                MoveCParam next_param;
                unitConvert(moveC_next, next_pose1, next_pose2, next_param); 
                arm_group_->MoveC(target_pose1, target_pose2, target_param.vel_max, \
                        target_param.acc_max, target_param.smooth, next_pose1, next_pose2, \
                        next_param.vel_max, next_param.acc_max, next_param.smooth, target_inst.id, result);
            }
            setServoWaitFlag(false);
            break;
        }//end case motion_spec_MOTIONTYPE_CIRCLEMOTION:
        default:
            break;
    }//end switch


    return result;
}




/**
 * @brief: pick one motion instruction from the queue and execute it
 *
 * @return: 0 if Success
 */
U64 RobotMotion::pickInstructions()
{
	U64 result;

    if ((getServoWaitFlag() == true)    //do not need to wait for servo ready
    || (arm_group_->getPlannedPathFIFOLength() > TRAJ_LIMIT_NUM))     //judge when to pick next instruction
    {
        return TPI_SUCCESS;
    }

    CommandInstruction *target_instruction = pickMoveInstruction();
    if (target_instruction == NULL)
    {
        return TPI_SUCCESS;
    }

    
    if (target_instruction->smoothDistance >= 0)
    {
        CommandInstruction *next_instruction = pickNextMoveInstruction();
        if (next_instruction) //find next_instruction
        {
            result = moveInstructions(*target_instruction, *next_instruction);      
        }
        else
        {
            //after moving to the middle still no next instruction
            //move as fine

            //push front target_instruction to use it next time
            result = moveInstructions(*target_instruction);
            if (result == TARGET_REPEATED)
            {
                result = TPI_SUCCESS;
                instruction_list_.pop_front();
                setPreviousCmdID(target_instruction->id);
                setCurrentCmdID(-1);
            }
        }
    }//end if (target_instruction->smoothDistance >= 0)
    else
    {
        result = moveInstructions(*target_instruction);         
        if (result == TARGET_REPEATED)
        {
            result = TPI_SUCCESS;
            FST_INFO("8888888888");
            if (isServoReady())  //it's the first one
            {FST_INFO("ooooooooooooooo");
                instruction_list_.pop_front();
                setPreviousCmdID(target_instruction->id);
                setCurrentCmdID(-1);

            }
            else 
            {
                if ((instruction_list_.size() == 1) //it's the last one
                && isServoReady())
                {
                    instruction_list_.pop_front();
                    setPreviousCmdID(target_instruction->id);
                    setCurrentCmdID(-1);
                }
                else
                {
                    target_instruction->pick_status = FRESH; //not execute this instruction
                }
            }//end else            
        }//end if (result == TARGET_REPEATED)            
    }//end else    
    if (result != TPI_SUCCESS)
    {
        target_instruction->pick_status = FRESH;
    }

    //setInstruction(target_instruction);
    //FST_INFO("pick result:%llx", result);

	return result;
}


bool RobotMotion::isFifoEmpty()
{
    boost::mutex::scoped_lock lock(mutex_);

    int traj_len = arm_group_->getPlannedPathFIFOLength();
	int joints_len = arm_group_->getJointTrajectoryFIFOLength();

    if ((traj_len == 0) && (joints_len == 0))
    {
        return true;
    }

    return false;
}

U64 RobotMotion::planFifo()
{
    U64 result = TPI_SUCCESS;
    RobotMode mode = getLogicMode();

    if (mode == MANUAL_MODE_M) 
    {
        result = moveManually();        
    }
    else if (mode == AUTO_RUN_M)
    {
        result = pickInstructions();
    }
    return result;
}


void RobotMotion::popupInstruction()
{
    int id, cur_id;
    while (!instruction_list_.empty())
    {
        CommandInstruction inst = instruction_list_.front();
        id = inst.id;
        //instruction_list_.pop_front();        
        cur_id = getCurrentCmdID();
        FST_INFO("poped instruction id:%d, cur_id:%d", id, cur_id);
        if(id != cur_id)
        {
            FST_INFO("non move instructions...");
            //this instruction should setio or other non move instruction
            //===run this instruction===     
            if ((inst.commandtype != motion_spec_MOTIONTYPE_JOINTMOTION) 
            && (inst.commandtype != motion_spec_MOTIONTYPE_CARTMOTION)
            && (inst.commandtype != motion_spec_MOTIONTYPE_CIRCLEMOTION))
            {    
                setPreviousCmdID(getCurrentCmdID());            
                setCurrentCmdID(id);
                addNonMoveInstruction(inst);
                setPreviousCmdID(id);
            }
            instruction_list_.pop_front();
        }
        else
        {
            setCurrentCmdID(cur_id); //set id to the next move id
            break;
        }        
    }

    cur_id = getCurrentCmdID();
    if(id != cur_id)
    {
        FST_ERROR("pop instruction id:%d is not the same as current id:%d",id, cur_id);
    }
}

void RobotMotion::resetInstructionList()
{
    ThreadSafeList<CommandInstruction>::iterator it = instruction_list_.begin();
    for (; it != instruction_list_.end(); it++)
    {
        if (it->pick_status != FRESH)
            it->pick_status = FRESH;
    }
    //setPreviousCmdID(-1);
    //setCurrentCmdID(-1);
}

U64 RobotMotion::pickPointsFromPathFifo()
{
    U64 result = TPI_SUCCESS;

    /*int traj_len = arm_group_->getPlannedPathFIFOLength();*/
	//int joints_len = arm_group_->getJointTrajectoryFIFOLength();
    //if (traj_len > 0) //there are points left for us to pick
	//{
		//if (joints_len < MAX_CNT_VAL)
		//{
			//int joints_spare = MAX_CNT_VAL -joints_len;
		////	FST_INFO("points fifo length:%d,joints_spare:%d", traj_len, joints_spare);
			//int points_num;
			//if (joints_spare > traj_len)
				//points_num = (traj_len < MAX_PLANNED_POINTS_NUM)?traj_len:MAX_PLANNED_POINTS_NUM;
			//else
				//points_num = (joints_spare < MAX_PLANNED_POINTS_NUM)?joints_spare:MAX_PLANNED_POINTS_NUM;
		////	FST_INFO("points_num:%d",points_num);
			//arm_group_->convertPathToTrajectory(points_num, result);
			//if (result != TPI_SUCCESS)
			//{
				//FST_ERROR("convertPathToTrajectory failed:%llx, servo_state:%d", result,getServoState());   
				//return result;
			//}
		//}// end if (joints_len < MAX_CNT_VAL)
	/*}//end if (traj_len > 0) */ 

    arm_group_->convertPathToTrajectory(MAX_PLANNED_POINTS_NUM, result);
    return result;
}

void RobotMotion::sendJointsToRemote()
{
    U64 result;
    if (share_mem_.isJointCommandWritten() == false)
    {
        share_mem_.setJointPositions(share_mem_.getCurrentJointCmd().joint_cmd);
        return;
    }//end if (share_mem_.isJointCommandWritten() == false)

    int joints_len = arm_group_->getJointTrajectoryFIFOLength();    
    if (joints_len > 0)
    {
    //	FST_INFO("joints fifo length:%d,points:%d", joints_len, arm_group_->getPlannedPathFIFOLength());
        int joints_in = (joints_len < NUM_OF_POINTS_TO_SHARE_MEM)?joints_len:NUM_OF_POINTS_TO_SHARE_MEM;

        vector<JointPoint> joint_traj;
        joints_in = arm_group_->getPointsFromJointTrajectoryFIFO(joint_traj, joints_in, result);
        //FST_INFO("joint in :%d",joints_in);		
        if (joints_in > 0)
        {
            //joints_len = arm_group_->getJointTrajectoryFIFOLength();
            //FST_INFO("joints fifo length:%d", joints_len);
            JointCommand joint_command;
            joint_command.total_points = joints_in;
            for (int i = 0; i < joints_in; i++)
            {
                joint_command.points[i].positions[0] = joint_traj[i].joints.j1;
                joint_command.points[i].positions[1] = joint_traj[i].joints.j2;
                joint_command.points[i].positions[2] = joint_traj[i].joints.j3;
                joint_command.points[i].positions[3] = joint_traj[i].joints.j4;
                joint_command.points[i].positions[4] = joint_traj[i].joints.j5;
                joint_command.points[i].positions[5] = joint_traj[i].joints.j6;
                joint_command.points[i].point_position = joint_traj[i].id & 0x03;  //last three bits as point position

            //	FST_INFO("%f,%f,%f,%f,%f,%f",\
            joint_traj_[i].joints.j1, joint_traj_[i].joints.j2,\
            joint_traj_[i].joints.j3, joint_traj_[i].joints.j4,\
            joint_traj_[i].joints.j5, joint_traj_[i].joints.j6);
                if (getLogicMode() == AUTO_RUN_M)
                {
                    int traj_id = joint_traj[i].id >> 2;   
                               
                    int cur_id = getCurrentCmdID();
                    if (cur_id == -1)
                    {                    
                        setCurrentCmdID(traj_id);
                       // FST_ERROR("prev_id:%d,traj_id:%d", previous_command_id_, traj_id);
                    }                
                    else if (cur_id != traj_id) //id changed
                    {  
                        setPreviousCmdID(cur_id);
                        setCurrentCmdID(traj_id);
                        popupInstruction();
                    }    
                }
               // FST_PRINT("position:%d===", joint_command.points[i].point_position);
              //  printDbLine("joints:", joint_command.points[i].positions, 6);
                if (joint_command.points[i].point_position == END_POINT)
                {
                    printDbLine("the last joints:", joint_command.points[i].positions, 6);
                    //FST_INFO("traj_id is :%d, cur_id:%d", traj_id, cur_id);
                } 
            }// end for (int i = 0; i < joints_in; i++)
        	//FST_INFO("=====>position:%d--%d,id:%d",joint_command.points[0].point_position, joint_command.points[joints_in-1].point_position,joint_traj[0].id >> 2);
            share_mem_.setCurrentJointCmd(joint_command); //store this command in case it can't write Success
            share_mem_.setJointPositions(joint_command); //send joints to bare metal         
        }//end if (joints_in > 0)
    }//end  if (joints_len > 0)

}

bool RobotMotion::hasManualVel()
{
    boost::mutex::scoped_lock lock(mutex_);
    if (manu_req_.vel_max > 0)
    {
        return true;
    }
    else 
    {
        return false;
    }
}


void RobotMotion::addTargetVal(double *src, double *delta, double *dst)
{
    for (int i = 0; i < MAX_JOINTS; i++)
    {
        dst[i] = src[i] + delta[i];
    }
}

ManualReq RobotMotion::getManualReq()
{
    boost::mutex::scoped_lock lock(mutex_);
    return manu_req_;
}

/*ManualReq RobotMotion::getNextManualReq()*/
//{
    //boost::mutex::scoped_lock lock(mutex_);
    //return manu_req_next_;
/*}*/


U64 RobotMotion::moveManually()
{
    U64 result = TPI_SUCCESS;
    int traj_len = arm_group_->getPlannedPathFIFOLength();
    if (traj_len > TRAJ_LIMIT_NUM)//judge when to pick instruction
    {
        return TPI_SUCCESS;
    }   
    
    ManualReq manu_req = getManualReq();
    manu_req.pre_target = manu_req.target;
    if (manu_req.vel_max == 0) 
    {
        manu_req.target.is_step = false;
        return TPI_SUCCESS;
    }
    else if (manu_req.ref_vmax == 0)
    {
        manu_req.target.is_step = true;
        /*setProgramStateCmd(GOTO_PAUSED_E);*/
        boost::mutex::scoped_lock lock(mutex_);
        manu_req_.vel_max = 0;
        /*return TPI_SUCCESS;*/
    }
    printDbLine("manu target:", (double*)&manu_req_.target.point, 6);
    FST_INFO("vel:%f", manu_req.vel_max);
    if (manu_req.target.is_step) //step 
    {
        if (manu_req.target.type == JOINT_M)
        {
            arm_group_->MoveJ(manu_req.target.point.jnt_val, manu_req.vel_max,\
                    DEFAULT_MOVEJ_ACC, 0, result);
            
        }
        else if (manu_req.target.type == CART_M)
        {
            arm_group_->MoveL(manu_req.target.point.pose_val, manu_req.vel_max,\
                    DEFAULT_ACC, 0, result);
        }             
    }//end if (manu_req.target.is_step)
    else
    {
        if (manu_req.target.type == JOINT_M)
        {
            if (manu_req.ref_target.type == JOINT_M)
            {
                arm_group_->MoveJ(manu_req.target.point.jnt_val, manu_req.vel_max,\
                    DEFAULT_MOVEJ_ACC, MAX_CNT_VAL, manu_req.ref_target.point.jnt_val, \
                    manu_req.ref_vmax, DEFAULT_MOVEJ_ACC, MAX_CNT_VAL, 0, result);
            }
            else if (manu_req.ref_target.type == CART_M)
            {
                arm_group_->MoveJ(manu_req.target.point.jnt_val, manu_req.vel_max,\
                    DEFAULT_MOVEJ_ACC, MAX_CNT_VAL, manu_req.ref_target.point.pose_val, \
                    manu_req.ref_vmax, DEFAULT_ACC, MAX_CNT_VAL, 0, result);
            }
        }//end if (manu_req.target.type == JOINT_M)
        else if (manu_req.target.type == CART_M)
        {
            if (manu_req.ref_target.type == JOINT_M)
            {
                arm_group_->MoveL(manu_req.target.point.pose_val, manu_req.vel_max,\
                    DEFAULT_ACC, MAX_CNT_VAL, manu_req.ref_target.point.jnt_val, \
                    manu_req.ref_vmax, DEFAULT_MOVEJ_ACC, MAX_CNT_VAL, 0, result);
            }
            else if (manu_req.ref_target.type == CART_M)
            {
                arm_group_->MoveL(manu_req.target.point.pose_val, manu_req.vel_max,\
                    DEFAULT_ACC, MAX_CNT_VAL, manu_req.ref_target.point.pose_val, \
                    manu_req.ref_vmax, DEFAULT_ACC, MAX_CNT_VAL, 0, result);
            }
        }//end else if (manu_req.target.type == CART_M)           
    }//end else
    if (result != TPI_SUCCESS)
    {
        FST_ERROR("manual move failed:%llx",result);
    }

    boost::mutex::scoped_lock lock(mutex_);
    if (manu_req_.ref_vmax > 0)    
    {
        manu_req_.vel_max = manu_req_.ref_vmax;
        manu_req_.target = manu_req_.ref_target;
        manu_req_.ref_vmax = 0;
    }   
    else
    {
        manu_req_.target.is_step = false;
        manu_req_.vel_max = 0;
    }

    return result;
}

CommandInstruction* RobotMotion::pickMoveInstruction()
{
    CommandInstruction *instruction;
    static ThreadSafeList<CommandInstruction>::iterator it;
    for (it = instruction_list_.begin(); it != instruction_list_.end(); it++)
    {
        if ((it->commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
        || (it->commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
        || (it->commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION))
        {
            if (it->pick_status != USED)
            {
                FST_INFO("find one:id==>%d, smooth:%f", it->id, it->smoothDistance);
                
                if (it-> pick_status == FRESH)
                {
                    ThreadSafeList<CommandInstruction>::iterator next_it = it;
                    if ((it->smoothDistance > 0) && (++next_it == instruction_list_.end()))
                    {
                        it->pick_status = ONCE;
                    }
                    else
                    {
                        it->pick_status = USED;
                    }
                }
                else if (it->pick_status == ONCE)
                {
                    it->pick_status = USED;
                    it->smoothDistance = -1;
                }
                instruction = &(*it);
                return instruction;
            }//end if (it->pick_status != USED)
        }// end if ((it->commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
        else
        {
            addNonMoveInstruction(*it); //process starting non move instructions
        }
    }// end for (it = instruction_list_.begin(); it != instruction_list_.end(); it++)

    return NULL;
}    

CommandInstruction* RobotMotion::pickNextMoveInstruction()
{
    CommandInstruction *instruction;
    static ThreadSafeList<CommandInstruction>::iterator it;
    for (it = instruction_list_.begin(); it != instruction_list_.end(); it++)
    {
        if ((it->commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
        || (it->commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
        || (it->commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION))
        {
            if (it->pick_status == FRESH)
            {
                FST_INFO("find next one:id==>%d, smooth:%f", it->id, it->smoothDistance);
                instruction = &(*it);
                return instruction;
            }
        }
        else if (it->commandtype == motion_spec_MOTIONTYPE_WAIT) //if wait, then must be fine
        {
            return NULL;
        }
    }

    return NULL;
}  


bool RobotMotion::getServoWaitFlag()
{
    boost::mutex::scoped_lock lock(mutex_);
    return servo_ready_wait_;
}

void RobotMotion::setServoWaitFlag(bool flag)
{
    FST_INFO("servo flag :%d", flag);
    boost::mutex::scoped_lock lock(mutex_);    
    servo_ready_wait_ = flag;
}

void RobotMotion::processEndingMove()
{
    if (getServoWaitFlag())
    {
        if (isServoReady() && isFifoEmpty())                     
        {
            int id;
            if (getCurrentCmdID() != (int)instruction_list_.front().id)
            {
                FST_ERROR("current_command_id_ is not the same as list front");
            }
            else
            {
                instruction_list_.pop_front();
            }
            while (!instruction_list_.empty())
            {
                CommandInstruction inst = instruction_list_.front();
                id = inst.id;                
                if ((inst.commandtype != motion_spec_MOTIONTYPE_JOINTMOTION) 
                && (inst.commandtype != motion_spec_MOTIONTYPE_CARTMOTION)
                && (inst.commandtype != motion_spec_MOTIONTYPE_CIRCLEMOTION))
                {    
                    setPreviousCmdID(getCurrentCmdID());            
                    setCurrentCmdID(id);
                    addNonMoveInstruction(inst);
                    instruction_list_.pop_front();                                    
                }
                else
                {
                    break;
                }                
            }//end while (!instruction_list_.empty())
            setPreviousCmdID(getCurrentCmdID());            
            setCurrentCmdID(-1);
            setServoWaitFlag(false);
        }// end 
    }
    //FST_INFO("prev_id:%d, cur_id:%d", previous_command_id_, current_command_id_);
}


U64 RobotMotion::addNonMoveInstruction(CommandInstruction instruction)
{
    non_move_instructions_.push_back(instruction); 
    return TPI_SUCCESS;
}

RobotMode RobotMotion::getPrevMode()
{
    boost::mutex::scoped_lock lock(mutex_);
    return prev_mode_;
}

void RobotMotion::setPrevMode(RobotMode mode)
{
    boost::mutex::scoped_lock lock(mutex_);
    prev_mode_ = mode;
}


bool RobotMotion::hasMoveCommand()
{
    RobotMode mode = getLogicMode();

    if (mode == MANUAL_MODE_M) 
    {
        ManualReq req = getManualReq();
        if (req.vel_max > 0)
            return true;
    }
    else if (mode == AUTO_RUN_M)
    {
        if (!instruction_list_.empty())
        {
            boost::thread thrd_non_move(boost::bind(processNonMove, this));
            return true;
        }
    }

    return false;
}

bool RobotMotion::isReadyToMove()
{
    RobotMode mode = getLogicMode();

    if (mode == MANUAL_MODE_M) 
    {
        ManualReq req = getManualReq();
        if (req.target.is_step == true)
            return true;
        else if (req.ref_vmax > 0)
            return true;
    }
    else if (mode == AUTO_RUN_M)
    {
        if (!instruction_list_.empty())
            return true;
    }

    return false;
}

bool RobotMotion::hasTransformedToIdle()
{
    RobotMode mode = getLogicMode();

    if (mode == MANUAL_MODE_M) 
    {
        ManualReq req = getManualReq();
        if ((getServoState() == STATE_READY)
        && isFifoEmpty()
        && (req.vel_max == 0))
        {
            clearPathFifo();
            return true;   
        }
    }
    else if (mode == AUTO_RUN_M)
    {
        if ((getServoState() == STATE_READY)
        && isFifoEmpty()
        && instruction_list_.empty())
        {
            //processEndingMove();
            return true;
        }
    }

    return false;
}


/*void RobotMotion::checkNextManualRequest()*/
//{
    //boost::mutex::scoped_lock lock(mutex_);
    //if (manu_req_next_.is_valid == false)
    //{
        //manu_req_next_.is_valid = true;
        //addTargetVal((double*)&manu_req_.target, (double*)&manu_req_.vel_req, (double*)&manu_req_next_.target);
    //}
/*}*/


void RobotMotion::clearManuMove()
{
    boost::mutex::scoped_lock lock(mutex_);
    memset((char*)&manu_req_, 0, sizeof(ManualReq));
}

double RobotMotion::calcuManualJointVel(const double *interval_jnts)
{          
    double max_interval = getMaxValue(interval_jnts, MAX_JOINTS);
    int id = getMaxValID(max_interval, interval_jnts, MAX_JOINTS);
    
    JointConstraints jnt_constraints = arm_group_->getJointConstraints();
    double constraint_vel;
    switch (id)
    {
        case 0:
            constraint_vel = jnt_constraints.j1.max_omega;
            break;
        case 1:
            constraint_vel = jnt_constraints.j2.max_omega;
            break;
        case 2:
            constraint_vel = jnt_constraints.j3.max_omega;
            break;
        case 3:
            constraint_vel = jnt_constraints.j4.max_omega;
            break;
        case 4:
            constraint_vel = jnt_constraints.j5.max_omega;
            break;
        case 5:
            constraint_vel = jnt_constraints.j6.max_omega;
            break;
    }//end switch(id)

    double vmax = max_interval / MANUAL_COUNT_PER_STEP * 1000; // rad/ms --> rad/s
    //vmax = (vmax<constraint_vel) ? vmax : constraint_vel;
    //vmax = vmax / constraint_vel * MAX_LINE_SPEED;
    vmax = vmax / constraint_vel * 100; //rad/s-->percentage

    return vmax;
}

double RobotMotion::calcuManualLineVel(const double *interval_pose)
{
    int id;
    double vmax;
    
    double max_line = getMaxValue(interval_pose, 3);    //x,y,z  m--> mm
    int line_id = getMaxValID(max_line, interval_pose, 3);
    double max_rot = getMaxValue(&interval_pose[3], 3); //a,b,c
    int rot_id = getMaxValID(max_rot, &interval_pose[3], 3);

    //FST_INFO("max_line:%f,max_rot:%f", max_line, max_rot);
    if ((max_line * MAX_RAD_SPEED) > (max_rot * MAX_LINE_SPEED)) //need to calcu according to rotation
    {
        id = line_id;
        vmax = max_line / MANUAL_COUNT_PER_STEP * 1000; // mm/ms --> mm/s
    }
    else
    {
        id = 3 + rot_id;
        vmax = (max_rot * MAX_LINE_SPEED) / (MANUAL_COUNT_PER_STEP * MAX_RAD_SPEED) * 1000; // c/t* (v_ref/omega_ref)
        FST_INFO("rot_interval:%f, vmax:%f", max_rot, vmax);
    }
 
    return vmax;
}

bool RobotMotion::isServoReady()
{
    if ((getServoState() == STATE_READY)
    && isFifoEmpty())
    {
        return true;
    }
    else
    {
        return false;
    }
}

double RobotMotion::get2PIDeltaValue(double value1, double value2)
{
    //FST_INFO("value1:%f, value2:%f", value1, value2);
    return round((value1 - value2) / (2 * PI)) * (2 * PI); 
}

void RobotMotion::getLineDelta(double* from, double* to, double* target, int num)
{
    for (int i = 0; i < num; i++)
    {
        //FST_INFO("to:%f, from:%f", to[i],from[i]);
        target[i] = to[i] * 1000 - from[i];
    }
}

void RobotMotion::getRadDelta(double* from, double* to, double* target, int num)
{
    for (int i = 0; i < num; i++)
    {
        target[i] = to[i] - from[i] - get2PIDeltaValue(to[i], from[i]);
        //FST_PRINT("%f ", get2PIDeltaValue(to[i], from[i]));
    }
    //printf("\n");
}

void RobotMotion::addLineDelta(double *src, double *delta, double *dst, int num)
{
    for (int i = 0; i < num; i++)
    {
        FST_INFO("src:%f,delta:%f", src[i], delta[i]);
        dst[i] = (src[i] + delta[i]) * 1000; //m --> mm
    }
}

void RobotMotion::addRadDelta(double *src, double *delta, double *dst, int num)
{
    for (int i = 0; i < num; i++)
    {
        dst[i] = src[i] + delta[i] - get2PIDeltaValue(src[i], dst[i]);
    }
}
