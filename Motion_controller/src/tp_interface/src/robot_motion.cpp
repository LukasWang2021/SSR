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
#include <future>
#include <boost/algorithm/string.hpp>

#define MAX(a,b) ((a) > (b) ? (a) : (b))

extern bool gs_running_flag;

RobotMotion::RobotMotion()
{
    servo_ready_wait_ = false;
	previous_command_id_ = -1;
	current_command_id_ = -1;

    vel_factor_ = 100;
    run_mode_ = NORMAL_R;
    nm_prgm_state_ = IDLE_R;

    prev_err_ = TPI_SUCCESS;							
	prev_mode_ = INIT_M;
	mode_ = INIT_M;
    //prev_state_ = ESTOP_S;
    state_ = ESTOP_S;
    program_state_ = IDLE_R;
    memset((char*)&manu_req_, 0, sizeof(ManualReq));

    io_interface_ = new IOInterface();

    arm_group_ = new ArmGroup();

	Transformation tool_frame;
	memset(&tool_frame, 0, sizeof(tool_frame));
	arm_group_->setToolFrame(tool_frame);  
    
    fifo1_.count = 0;
    fifo2_.count = 0;
    dbcount = 0;
    shutdown_ = false;

    boost::thread thrd_non_move(boost::bind(&RobotMotion::processNonMove, this));
}


RobotMotion::~RobotMotion()
{
    //==before exit, first estop to fault===========
    /*servoEStop();*/
    //setLogicStateCmd(EMERGENCY_STOP_E);
    /*usleep(100*1000);*/  
    //==============================================
    if (arm_group_)
    {
        delete arm_group_;
    }
    if (io_interface_ != NULL)
        delete io_interface_;
}


void RobotMotion::initial(vector<U64>& err_list)
{
    U64 result;

    if((result = share_mem_.initial()) != TPI_SUCCESS)
    {
        FST_ERROR("ShareMem init failed");
        err_list.push_back(result);
    }

    share_mem_.stopBareMetal(); //disable baremetal first

	if (arm_group_->initArmGroup(err_list) == false)
    {
        FST_ERROR("init arm_group_ failed");
        //err_list.push_back(result);
    }

    if ((result = io_interface_->initial()) != TPI_SUCCESS)
    {
        FST_ERROR("io_interface_ init failed");
        err_list.push_back(result);
    }

}

void RobotMotion::destroy()
{
}



U64 RobotMotion::checkProgramState()
{
    U64 result = TPI_SUCCESS;
    
    static int pause_cnt = 0;
    
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
            pause_cnt = 0;
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
                abortMotion(); //abort the planned motion 
                clearInstructions();
                setProgramState(IDLE_R);
                boost::mutex::scoped_lock lock(mutex_);
                manu_req_.vel_max = 0;
            }
            break;
        case EXECUTE_TO_PAUSE_T:
            result = sendJointsToRemote();     
            //FST_INFO("servo state:%d", getServoState());
            if ((getServoState() == STATE_READY) 
            || (getServoState() == STATE_ERROR))
            {
                if (0 == arm_group_->getTrajectoryFIFOLength())
                {
                    fifo1_.count = 0;
                    fifo2_.count = 0;
                    setProgramState(PAUSED_R);
                    pause_cnt = 0;
                }
            }            
            break;
        case PAUSE_TO_IDLE_T:
            //FST_INFO("pause to idle");
            pause_cnt = 0;
            result = abortMotion();
            clearInstructions();
            setProgramState(IDLE_R);
            break;
        case EXECUTE_R:
            if (getLogicState() != ENGAGED_S)
            {
                break;
            }
            //processEndingMove();              
            if (hasTransformedToIdle())
            {
                FST_INFO("EXECUTE_TO_idle");
                fifo1_.count = 0;
                fifo2_.count = 0;
                setProgramState(IDLE_R);
                boost::mutex::scoped_lock lock(mutex_);
                manu_req_.vel_max = 0;
            }
            else
            {
                result = sendJointsToRemote();
                if (result != TPI_SUCCESS)
                {
                    //FST_ERROR("======pick point failed====");
                    return result;    
                }
                result = planFifo();
                if (result != TPI_SUCCESS)
                    return result;
            }
            break;
        case PAUSED_R:       
        {
            static const int max_count = MAX_TIME_IN_PUASE / INTERVAL_PROCESS_UPDATE;
            
            if (getLogicState() == ENGAGED_S)
            {
                pause_cnt++;
                if (pause_cnt >= max_count) //wait
                {
                    pause_cnt = 0;
                    setLogicStateCmd(EMERGENCY_STOP_E);
                    // then we need to return this info
                }
            }
            //timeout to set logic state estop
            RobotMode mode = getLogicMode();
            /*if (mode == AUTO_RUN_M)*/
            //{
                //if (instruction_list_.empty())
                //{
                    //count = 0;
                    //setProgramState(IDLE_R);
                //}
            //}
            /*else*/ if (mode == MANUAL_MODE_M)
            {
                pause_cnt = 0;
                result = abortMotion();
                //setServoWaitFlag(false);
                setProgramState(IDLE_R);
                boost::mutex::scoped_lock lock(mutex_);
                manu_req_.vel_max = 0;
            }
            /*else*/
            //{
                //setProgramState(IDLE_R);
            /*}*/
            break;  
        }//end switch (prgm_state)
        default:
            break;
    }

    return result;
}

U64 RobotMotion::stateEStopAction()
{
    FST_INFO("in estop action...");
    servoEStop();
    setProgramStateCmd(GOTO_PAUSED_E); //set mode to pause
    clearManuMove();

    return TPI_SUCCESS;
}

void RobotMotion::errorAction(int warning_level)
{
    FST_INFO("warning_level is:%d", warning_level);
    if (warning_level > 4)
    {
        FST_INFO("in estop process...");
        safetyStop(warning_level);
        servoEStop();
        setProgramStateCmd(GOTO_PAUSED_E); //set mode to pause
        clearManuMove();
        setLogicStateCmd(EMERGENCY_STOP_E);
    }
    else if (warning_level > 2)
    {
        FST_INFO("in pause process...");
        setProgramStateCmd(GOTO_PAUSED_E);
        setLogicStateCmd(EMERGENCY_STOP_E);
    }
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
	return this->io_interface_;
}
/**
 * @brief: get ShareMem object 
 *
 * @return: &share_mem_ 
 */
ShareMem* RobotMotion::getShareMemPtr()
{
	return &(this->share_mem_);
}

/**
 * @brief: get the pointer of ArmGroup 
 *
 * @return: arm_group_ 
 */
ArmGroup* RobotMotion::getArmGroupPtr()
{
	return this->arm_group_;
}

SafetyInterface* RobotMotion::getSafetyInterfacePtr()
{
    return &(this->safety_interface_);
}

RunningMode RobotMotion::getRunningMode()
{
    return this->run_mode_;
}

void RobotMotion::setRunningMode(RunningMode rm)
{
    if (rm == NORMAL_R)
    {
        return;
        //run_mode_ = NORMAL_R;
    }
    else
    {
        run_mode_ = (RunningMode)(run_mode_ | rm);
    }
}

void RobotMotion::clearRunningMode(RunningMode rm)
{
    if (rm == NORMAL_R)
    {
        return;
    }
    else
    {
        run_mode_ = (RunningMode)(run_mode_ & (~rm));
    }
}

int RobotMotion::getCurveMode()
{
    return (int)arm_group_->getCurveMode();
}

void RobotMotion::setCurveMode(int c_mode)
{
    arm_group_->setCurveMode((CurveMode)c_mode);
}

ProgramState RobotMotion::getNMPrgmState()
{
    return nm_prgm_state_; 
}

void RobotMotion::setNMPrgmState(ProgramState prgm_state)
{
    nm_prgm_state_ = prgm_state;
}


ProgramState RobotMotion::getProgramState()
{
    return this->program_state_;
}
void RobotMotion::setProgramState(ProgramState prgm_state)
{
    this->program_state_ = prgm_state;
}

int RobotMotion::getInstructionListSize()
{
    boost::mutex::scoped_lock lock(mutex_);
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
    else if (size > 0)
    {
        CommandInstruction instruction = instruction_list_.back();
        if ((instruction.pick_status == PICKED) && (instruction.smoothDistance >= 0))
        {
            size = -1;
        }
    }

    return size;
}

bool RobotMotion::updateInstructionSize()
{
    static int pre_size = 0;

    if (getLogicMode() != AUTO_RUN_M)
        return false;

    int size = getInstructionListSize();
    if (size != pre_size)
        return true;

    return false;
}
/**
 * @brief: get previous_command_id_ 
 *
 * @return: previous_command_id_ 
 */
int RobotMotion::getPreviousCmdID()
{
	return this->previous_command_id_;
}

void RobotMotion::setPreviousCmdID(int id)
{
	this->previous_command_id_ = id;
}

/**
 * @brief: get current_command_id_ 
 *
 * @return: current_command_id_ 
 */
int RobotMotion::getCurrentCmdID()
{
	return this->current_command_id_;
}

void RobotMotion::setCurrentCmdID(int id)
{
	this->current_command_id_ = id;
}


/**
 * @brief: get current logic mode
 *
 * @return: mode_ 
 */
RobotMode RobotMotion::getLogicMode()
{
	return this->mode_;
}

/**
 * @brief: Get current logic state
 *
 * @return: state_
 */
RobotState RobotMotion::getLogicState()
{
	return this->state_;
}

/**
 * @brief: Get current joints
 *
 * @return: joints_
 */
Joint RobotMotion::getCurJointsValue()
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

PoseEuler RobotMotion::getFlangePose()
{
	boost::mutex::scoped_lock lock(mutex_);
	return flange_pose_;
}

/**
 * @brief: get servo state, 0:init, 1:ready, 2:running, 3:error 
 *
 * @return: servo_state_ 
 */
unsigned int RobotMotion::getServoState()
{
	return this->servo_state_;
}

void RobotMotion::setServoState(unsigned int servo_state)
{
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
        case ESTOP_S:
            if (gs_running_flag == false)
            {
                shutdown_ = true;
            }
            break;
        case ENGAGED_S:
            if (gs_running_flag == false)
            {
                setLogicStateCmd(EMERGENCY_STOP_E);
            }
            break;
        case TO_ESTOP_T:
        {
            ProgramState ps = getProgramState();
            FST_INFO("cur ProgramState:%d", ps);
            if ((ps == IDLE_R) 
            || (ps == PAUSED_R))
            {
                //==record current joints for calibrate==
                U64 ret;
                arm_group_->recordLastJoint(ret);
                if (ret != TPI_SUCCESS)
                {
                    FST_ERROR("recordLastJoint failed:%llx", ret);
                }
                setLogicState(ESTOP_S);
            }
            break;
        }
        case RESET_ESTOP_T:
        {
            // FST_INFO("estop to off");            
            //===wait RESET_ERROR_DELAY ms=========================
            static int count = RESET_ERROR_TIMEOUT / INTERVAL_PROPERTY_UPDATE;
            //FST_INFO("the count is :%d", count);
            if (count-- > 0) //hasn't arrived timeout, during wait
            {
                //===check if error exist again====
                //if error accurs again, return ESTOP_S==
                if (isErrorExist())
                {
                    count = 0;
                    //share_mem_.stopBareMetal();
                    //setLogicState(ESTOP_S);
                    //====set the count to default===
                    //count = RESET_ERROR_TIMEOUT / INTERVAL_PROPERTY_UPDATE;
                    //break;
                }
                else
                {
                    //=====servo hasn't ready========
                    //===need keep on wait============
                    if (getServoState() != STATE_READY)   
                    {
                        break;
                    }
                    if (safety_interface_.getDIAlarm())
                    {
                        break;
                    }
                }
            }// if (count-- > 0)

            FST_INFO("count:%d", count);
            if (count <= 0)
            {
                //FST_ERROR("error: reset timeout");
                share_mem_.stopBareMetal();
                setLogicState(ESTOP_S);
            }            
            else
            {
                setLogicState(ENGAGED_S);
            }
            //====set the count to default===
            count = RESET_ERROR_TIMEOUT / INTERVAL_PROPERTY_UPDATE;
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

#define MAX_ACCURATE_VALUE 0.1
bool isOutMax(Joint src_joints, Joint dst_joints)
{
	if (fabs(src_joints.j1 - dst_joints.j1) > MAX_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j2 - dst_joints.j2) > MAX_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j3 - dst_joints.j3) > MAX_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j4 - dst_joints.j4) > MAX_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j5 - dst_joints.j5) > MAX_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j6 - dst_joints.j6) > MAX_ACCURATE_VALUE) return false;

	return true;
}	

/**
 * @brief: update current joints get value from the share memory 
 *
 * @return: 0 if success
 */
U64 RobotMotion::updateJoints()
{
	FeedbackJointState fbjs;
	Joint joints_val;
  //  static Joint prev_joints = {-1,-1,-1,-1,-1,-1};

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
        /*if (isOutMax(prev_joints, joints_val) == false)*/
        //{
            //printDbLine("+++prev_joints:", (double*)&prev_joints, 6);
            //printDbLine("+++cur_joints:", (double*)&joints_val, 6);
        //}
        /*prev_joints = joints_val;*/

        unsigned int servo_state = getServoState();
        /*static int cnt = 40;*/
        //if (cnt-- <= 0)
        //{
            //FST_INFO("update joints:%d", fbjs.state); 
            //cnt = 40;
        /*}*/
        if (getRunningMode() != NORMAL_R)
        {
            if (arm_group_->isJointFallInConstraint(joints_val, arm_group_->getSoftConstraint()))
            {
                clearRunningMode(SOFTLIMITED_R);
            }
        }
        if (fbjs.state == STATE_READY) 
        {
            if (servo_state != fbjs.state)
            {
                FST_INFO("==========diffrent===========current:%d, pre:%d", fbjs.state, servo_state);
            }
            if ((servo_state == STATE_RUNNING) || (servo_state == STATE_WAIT_SERVOREADY))
            {
                FST_INFO("state translate from running/(wait running) to ready");
                //==set servo wait ready flag to false========
                if (arm_group_->isTrajectoryTotallyFinished())
                {
                    setServoWaitFlag(false);
                }
                else
                {
                    FST_INFO("====this is not end of trajectory====");
                }
            }    

        	//FST_INFO("cur_joints:%f,%f,%f,%f,%f,%f", joints_.j1,joints_.j2,joints_.j3
             //       ,joints_.j4,joints_.j5,joints_.j6);
        }//if (fbjs.state == STATE_READY)
        else if (fbjs.state == STATE_ERROR)
        {
            if ((servo_state == STATE_RUNNING) || (servo_state == STATE_WAIT_SERVODOWN))
            {
                FST_INFO("state translate from running/(wait running) to error");
                if (arm_group_->isTrajectoryTotallyFinished())
                {
                    setServoWaitFlag(false);
                }
                else
                {
                    FST_INFO("====this is not end of trajectory====");
                }
            }   
        }// else if (fbjs.state == STATE_ERROR)
        setCurJoints(joints_val);

        setServoState(fbjs.state);
	}// if (result == TPI_SUCCESS)
    
	
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
	  //set the current joint to lib
    result = getPoseFromJoint(getCurJointsValue(), pose);
	if (result != TPI_SUCCESS)
        return result;
    
	uintPoseCtle2TP(&pose);
    setCurPose(pose);
    return TPI_SUCCESS;
}

U64 RobotMotion::updateFlangePose()
{
	U64 result;
	PoseEuler pose;
    PoseEuler pose_tcp;
    if (false == arm_group_->computeFKInWorldCoordinate(getCurJointsValue(), pose, pose_tcp, result))
        return result;
    
	uintPoseCtle2TP(&pose);
    //printDbLine("flange_pose_:", (double*)&pose, 6);
    setFlangePose(pose);
    return TPI_SUCCESS;
}



U64 RobotMotion::updateSafetyStatus()
{
    if (!safety_interface_.isSafetyValid())
    {
        return TPI_SUCCESS;
    }
    if (safety_interface_.isSafetyAlarm())
    {
        return SERVO_ESTOP;
    }

    return TPI_SUCCESS;
}



void RobotMotion::setLogicMode(RobotMode mode)
{
    this->mode_ = mode;
}

void RobotMotion::setLogicState(RobotState state)
{
    /*if (state_ != state)*/
        /*FST_INFO("state_:%d",state);*/
    this->state_ = state;
}

void RobotMotion::setCurJoints(Joint joints)
{
    boost::mutex::scoped_lock lock(mutex_);
    this->joints_ = joints;
}
void RobotMotion::setCurPose(PoseEuler pose)
{
    boost::mutex::scoped_lock lock(mutex_);
    this->pose_ = pose;
}

void RobotMotion::setFlangePose(PoseEuler pose)
{
    boost::mutex::scoped_lock lock(mutex_);
    flange_pose_ = pose;
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
                result = actionResume();
                if (result != TPI_SUCCESS)
                {
                    setProgramState(PAUSED_R);
                    break;
                }
            }
            setNMPrgmState(EXECUTE_R);
            break;
        case GOTO_PAUSED_E:
            if (getLogicState() != ENGAGED_S)
            {
                //the robot can't be moving in this state
                //setProgramState(PAUSED_R); 
                break;
            }
            if (prgm_state == IDLE_R)
            {
                break;
            }
            if (prgm_state == EXECUTE_R)
            {                
                setProgramState(EXECUTE_TO_PAUSE_T);
            }
            else if (prgm_state == IDLE_TO_EXECUTE_T)
            {                
                setProgramState(IDLE_R);
            }
            else if (prgm_state == PAUSE_TO_EXECUTE_T)
            {
                setProgramState(PAUSED_R);                
            }
            else if (prgm_state == EXECUTE_TO_PAUSE_T)
            {
                result = INVALID_ACTION_IN_CURRENT_STATE;
                break;
            }
            setNMPrgmState(PAUSED_R);
            result = actionPause();
            break;
        default:
            break;
    }//end switch (prgm_cmd)

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
            arm_group_->setVelocityScalingFactor(getGlobalVelocity());
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
            arm_group_->setVelocityScalingFactor(100);
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
	}//end switch (mode_cmd)

    //==========set previous target=============
    PointVal point;
    point.jnt_val = getCurJointsValue();
    setPrevTarget(JOINT_M, point);
    //////////////////////////////////////////

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
        
    //FST_INFO("state_cmd:%d",state_cmd);
	switch (state_cmd)
	{
        case EMERGENCY_STOP_E:
            if (state == ESTOP_S)
            {
                break;
            }
            //result = emergencyStop();//some status need to set in ENGAGED_S                
            setLogicState(TO_ESTOP_T);                        
            break; 
        case ACKNOWLEDGE_ERROR:
            if (state == ENGAGED_S)
            {
                break;
            }
            else if (state == ESTOP_S)
            {
                setLogicState(RESET_ESTOP_T);
                //first clear error list 
                clearErrorList();
                //====reset core1==========
                share_mem_.resetBareMetal();
                if (result != TPI_SUCCESS)
                {
                    setLogicState(ESTOP_S);
                    break;
                }                
                //=======reset safety board==================
                resetSafetyBoard();
                /*result = safety_interface_.reset();  //reset safety board*/
                //if (result != TPI_SUCCESS)
                //{
                    //setLogicState(ESTOP_S);
                    //break;
                /*}*/
                //==========reset ArmGroup=============
                arm_group_->resetArmGroup(result);
                if (result != TPI_SUCCESS)
                {
                    if (result == CURRENT_JOINT_OUT_OF_CONSTRAINT)
                    {
                        result = TPI_SUCCESS;
                        setRunningMode(SOFTLIMITED_R);
                    }
                    else
                    {
                        FST_ERROR("reset ArmGroup failed:%llx", result);        
                        setLogicState(ESTOP_S);
                        break;
                    }
                }                  
            }//end else if (state == ESTOP_S)
            else 
            {
                FST_ERROR("invalid action in state:%d", state);
                result = SET_STATE_FAILED;
            }
            break;
        default:
            break;
    }//end switch (state_cmd)
    FST_INFO("current state:%d", getLogicState());

    return result;
}

U64 RobotMotion::actionResume()
{
    U64 result = TPI_SUCCESS;

    FST_INFO("resumeArmMotion...");
	arm_group_->resumeArmMotion(result);
    int fifo2_len = arm_group_->getTrajectoryFIFOLength();
    /*if (fifo2_len == 0) //in case last time almost ready, then disable servo */
    //{
        //setServoWaitFlag(false); 
    /*}*/
    //FST_INFO("wait flag:%d, traj_len:%d, fifo2:%d", getServoWaitFlag(), arm_group_->getPlannedPathFIFOLength(), fifo2_len);
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
        
    if (arm_group_->suspendArmMotion(result))
    {
        //resetInstructionList();            
    }

    
	return result;
}

U64 RobotMotion::safetyStop(int level)
{
    FST_INFO("safetyStop ...");
    switch (level)
    {
        case 5:
        case 6:
            safety_interface_.setDOType1Stop(1);
            break;
        case 7:
            safety_interface_.setDOType0Stop(1);
            break;
        case 8:
        case 9:
            safety_interface_.setDOType2Stop(1);
            break;
        case 10:
            safety_interface_.setDOType0Stop(1);
            break;
        case 11:
            safety_interface_.setDOType0Stop(1);
            break;
        default:
            break;
    }
    return TPI_SUCCESS;
}

U64 RobotMotion::servoEStop()
{    
    //stop bare metal, can't write fifo since
    U64 result = share_mem_.stopBareMetal();
    if (result == TPI_SUCCESS)
    {
        //if there is no point, then do not call declareEstop
        if (getProgramState() != IDLE_R)
        //if (getServoState() != STATE_READY)
            arm_group_->declareEstop();
    }

    share_mem_.setWritenFlag(true);
    usleep(500*1000);   //wait for brake on

    return result;
}


U64 RobotMotion::emergencyStop()
{
    U64 result = TPI_SUCCESS;
    
    setProgramStateCmd(GOTO_PAUSED_E); //set mode to pause
    clearManuMove();
    
    

    
    return result;
}

U64 RobotMotion::actionShutdown()
{
    gs_running_flag = false;
    FST_INFO("shutdown ...");
    setProgramStateCmd(GOTO_PAUSED_E);
    setLogicStateCmd(EMERGENCY_STOP_E);
    return TPI_SUCCESS;
}

U64 RobotMotion::actionCalibrate()
{
    U64 result;    
    unsigned int caliborate_val;
    if (arm_group_->calibrateZeroOffset(caliborate_val, result))
    {
        clearRunningMode(CALIBRATE_R);
        setProgramStateCmd(GOTO_IDLE_E);
    }
    return result;
}

/**
 * @brief: add one command in to queue
 *
 * @param motion_command: input==>the command to push
 */
void RobotMotion::addMotionInstruction(CommandInstruction cmd_instruction)
{
    boost::mutex::scoped_lock lock(mutex_);
    instruction_list_.push_back(cmd_instruction);
    FST_INFO("instruction num:%d", instruction_list_.size());
}

U64 RobotMotion::abortMotion()
{
    U64 result;
    //=====clear all the fifos=======
    arm_group_->clearArmGroup(result);
    if (result != TPI_SUCCESS)
    {
        FST_ERROR("reset ArmGroup failed:%llx", result);
    }
    share_mem_.setWritenFlag(true); //don't write share_mem any more
    setServoWaitFlag(false);    //don't wait any flag

    return result;
}

void RobotMotion::clearInstructions()
{
    FST_INFO("clear instruction list...");
    boost::mutex::scoped_lock lock(mutex_);
    while (!instruction_list_.empty())
    {
       instruction_list_.pop_front();
    }
    while (!non_move_instructions_.empty())
    {
        non_move_instructions_.pop_front();
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
	    arm_group_->setStartState(getCurJointsValue(), result);
        dbcount = 0;
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
        //printDbLine("start joints_", (double*)&joints_,6);
        boost::mutex::scoped_lock lock(mutex_);
	    arm_group_->setStartState(joints_, result);
        manu_refer_joints_ = joints_;
        pre_target_.point.jnt_val = joints_;
        pre_target_.type = JOINT_M;
        manu_refer_pose_ = pose_;
        manu_refer_pose_.position.x= pose_.position.x * 1000;  //m --> mm
        manu_refer_pose_.position.y= pose_.position.y * 1000;
        manu_refer_pose_.position.z= pose_.position.z * 1000;
        dbcount = 0;
    }
    if (result != TPI_SUCCESS)
        FST_ERROR("ffffffffffffffffffff");

    return result;
}


bool RobotMotion::isJointsChanged()
{
    static int cnt = 0;
    static Joint prev_joints = {-1,-1,-1,-1,-1,-1};

    Joint cur_joints = getCurJointsValue();
    if (isOutMax(prev_joints, cur_joints) == false)
    {
        printDbLine("prev_joints:", (double*)&prev_joints, 6);
        printDbLine("cur_joints:", (double*)&cur_joints, 6);
    }
    if (compareJoints(prev_joints, cur_joints) == false)
	{
		cnt++;
		prev_joints = cur_joints;
		//FST_INFO("cnt:%d, joint:%f,%f,%f,%f,%f,%f", cnt,\
			joints_.j1,joints_.j2,joints_.j3,joints_.j4,joints_.j5,joints_.j6);

        return true;
	}
    
    return false;
}
#include <thread> 
#include <chrono>
int RobotMotion::motionHeartBeart(U64 *err_list)
{
   // auto t1 = std::chrono::system_clock::now();
    int err_size = share_mem_.monitorHearBeat(err_list);
    /*auto t2 = std::chrono::system_clock::now();*/
        //int x = std::chrono::duration_cast<std::chrono::milliseconds>( t2-t1 ).count();
        //if (x > 10)
            /*FST_ERROR("222delay too much=====+:%d", x);*/

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
    prev_err_ = err;
}

void RobotMotion::clearErrorList()
{
    while (!bak_err_list_.empty())
    {
        if (bak_err_list_.front() & 0x0010000000000000)
            return;
        bak_err_list_.pop_front();
    }

    prev_err_ = TPI_SUCCESS;
}

bool RobotMotion::isUpdatedSameError(U64 err)
{
    if (prev_err_ != err)
    {
        return false;
    }

    return true;
}

bool RobotMotion::isErrorExist()
{
    if (bak_err_list_.empty())
        return false;
    else
    {
        ThreadSafeList<U64>::iterator it = bak_err_list_.begin();
        for(; it != bak_err_list_.end(); ++it)
        {
            int warning_level = (*it & 0x0000FFFF00000000) >> 32;
            if (warning_level > 2)
                return true;
        }
        return false;
    }
}

U64 RobotMotion::clearPathFifo()
{
    U64 result = TPI_SUCCESS;

	//int traj_len = arm_group_->getPlannedPathFIFOLength();
	//int joints_len = arm_group_->getTrajectoryFIFOLength();
//	FST_ERROR("current traj_len:%d, joints_len:%d", traj_len, joints_len);
    
    /*if ((traj_len != 0) [>|| (joints_len != 0)<])*/
    //{		
        ////maybe wrong when setStartState failed
        //FST_ERROR("clear fifo....");
       //// arm_group_->clearPlannedPathFIFO(result); 
    /*}*/

    return result;
}

U64 RobotMotion::checkManualJntVel(const motion_spec_TeachPose *tech_pose)
{
    U64 result;    
    
    if (getServoWaitFlag())
        return TPI_SUCCESS;

    Joint *joints = (Joint*)tech_pose->pose.coordinates;

    printDbLine("manual joint values:", (double*)joints, MAX_JOINTS);    

    result = checkManualStartState();
    if (result != TPI_SUCCESS)
        return result;

    Joint start_joints = getStartJoint();
    boost::mutex::scoped_lock lock(mutex_);
    if (tech_pose->has_velocity) //must be step mode
    {
        FST_INFO("cur run_mode_:%d", getRunningMode());
        if (run_mode_ != NORMAL_R)  //only normal mode can use this function
            return INVALID_ACTION_IN_LIMITED_STATE;

        if (tech_pose->velocity <= 0)
            return TPI_SUCCESS;
        manu_req_.target.is_step = true;
        manu_req_.target.type = JOINT_M;
        manu_req_.target.point.jnt_val = *joints;
        manu_req_.vel_max = tech_pose->velocity;
        manu_req_.ref_vmax = 0;
    }//end if (tech_pose->has_velocity)
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
            }//end else
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
    if (getServoWaitFlag())
        return TPI_SUCCESS;

    static PoseEuler pre_pose = getCurPosition();
    PoseEuler *pose = (PoseEuler*)tech_pose->pose.coordinates;

    printDbLine("manual pose values:", (double*)pose, 6);  
    
    result = checkManualStartState();
    if (result != TPI_SUCCESS)
        return result;
    
    PoseEuler start_pose = getStartPose();
    boost::mutex::scoped_lock lock(mutex_);
    if (tech_pose->has_velocity) //must be step mode
    {
        if (tech_pose->velocity <= 0)
            return TPI_SUCCESS;
        PoseEuler tmp_pose = *pose;
        tmp_pose.position.x= pose->position.x * 1000;
        tmp_pose.position.y= pose->position.y * 1000;
        tmp_pose.position.z= pose->position.z * 1000;
        Joint tmp_joints;
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
        //printDbLine("pose:",(double*)&tmp_pose, 6);
      //  printDbLine("joints:",(double*)&tmp_joints, 6);
        return TPI_SUCCESS;
    }//end if (tech_pose->has_velocity)
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
    //printDbLine("manu_refer_pose_:", (double*)&manu_refer_pose_, 6);
    //printDbLine("interval_pose:", interval_pose, 6);

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
        }//end if if (manu_req_.target.is_step == false)
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
        }//end else
    }//end if ((tech_pose->has_step) && (tech_pose->step == true))
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
        }//end else
    }//end else

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
bool RobotMotion::compareJoints(Joint src_joints, Joint dst_joints)
{
	if (fabs(src_joints.j1 - dst_joints.j1) > MIN_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j2 - dst_joints.j2) > MIN_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j3 - dst_joints.j3) > MIN_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j4 - dst_joints.j4) > MIN_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j5 - dst_joints.j5) > MIN_ACCURATE_VALUE) return false;
	if (fabs(src_joints.j6 - dst_joints.j6) > MIN_ACCURATE_VALUE) return false;

	return true;
}	

U64 RobotMotion::getPoseFromJoint(const Joint &joints, PoseEuler &pose)
{
    U64 result;
    Pose p;
    if (arm_group_->getPoseFromJoint(joints, p, result))        
        pose = arm_group_->transformPose2PoseEuler(p);

    return result;
}

U64 RobotMotion::getJointFromPose(const PoseEuler &pose, Joint &joints, double time_val)
{
    U64 result;
    Pose p = arm_group_->transformPoseEuler2Pose(pose);
    
    arm_group_->getJointFromPose(p, joints, time_val, result);
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

    return TPI_SUCCESS;
}

motion_spec_userFrame RobotMotion::getUserFrame()
{
    Transformation transform = arm_group_->getUserFrame();
    motion_spec_userFrame user_frame;
    user_frame.X = transform.position.x;
    user_frame.Y = transform.position.y;
    user_frame.Z = transform.position.z;
    
    user_frame.A = transform.orientation.a;
    user_frame.B = transform.orientation.b;
    user_frame.C = transform.orientation.c;

    return user_frame;
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

motion_spec_toolFrame RobotMotion::getToolFrame()
{
    Transformation transform = arm_group_->getToolFrame();
    motion_spec_toolFrame too_frame;
    too_frame.X = transform.position.x;
    too_frame.Y = transform.position.y;
    too_frame.Z = transform.position.z;
    
    too_frame.A = transform.orientation.a;
    too_frame.B = transform.orientation.b;
    too_frame.C = transform.orientation.c;

    return too_frame;
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


motion_spec_JointConstraint RobotMotion::getSoftConstraint()
{
    JointConstraint jc = arm_group_->getSoftConstraint();
    //FST_INFO("J1=>maxv:%f, maxa:%f", jc.j1.max_omega, jc.j1.max_alpha);
    motion_spec_JointConstraint jnt_constraint;
    jnt_constraint.jnt_lmt_count = MAX_JOINTS;

    jnt_constraint.jnt_lmt[0].has_zero = true;
    jnt_constraint.jnt_lmt[0].zero = jc.j1.home;
    jnt_constraint.jnt_lmt[0].has_upper = true;
    jnt_constraint.jnt_lmt[0].upper = jc.j1.upper;
    jnt_constraint.jnt_lmt[0].has_lower = true;
    jnt_constraint.jnt_lmt[0].lower = jc.j1.lower;
    jnt_constraint.jnt_lmt[0].has_max_omega = true;
    jnt_constraint.jnt_lmt[0].max_omega = jc.j1.max_omega;
    jnt_constraint.jnt_lmt[0].has_max_alpha = true;
    jnt_constraint.jnt_lmt[0].max_alpha = jc.j1.max_alpha;

    jnt_constraint.jnt_lmt[1].has_zero = true;
    jnt_constraint.jnt_lmt[1].zero = jc.j2.home;
    jnt_constraint.jnt_lmt[1].has_upper = true;
    jnt_constraint.jnt_lmt[1].upper = jc.j2.upper;
    jnt_constraint.jnt_lmt[1].has_lower = true;
    jnt_constraint.jnt_lmt[1].lower = jc.j2.lower;
    jnt_constraint.jnt_lmt[1].has_max_omega = true;
    jnt_constraint.jnt_lmt[1].max_omega = jc.j2.max_omega;
    jnt_constraint.jnt_lmt[1].has_max_alpha = true;
    jnt_constraint.jnt_lmt[1].max_alpha = jc.j2.max_alpha;

    jnt_constraint.jnt_lmt[2].has_zero = true;
    jnt_constraint.jnt_lmt[2].zero = jc.j3.home;
    jnt_constraint.jnt_lmt[2].has_upper = true;
    jnt_constraint.jnt_lmt[2].upper = jc.j3.upper;
    jnt_constraint.jnt_lmt[2].has_lower = true;
    jnt_constraint.jnt_lmt[2].lower = jc.j3.lower;
    jnt_constraint.jnt_lmt[2].has_max_omega = true;
    jnt_constraint.jnt_lmt[2].max_omega = jc.j3.max_omega;
    jnt_constraint.jnt_lmt[2].has_max_alpha = true;
    jnt_constraint.jnt_lmt[2].max_alpha = jc.j3.max_alpha;

    jnt_constraint.jnt_lmt[3].has_zero = true;
    jnt_constraint.jnt_lmt[3].zero = jc.j4.home;
    jnt_constraint.jnt_lmt[3].has_upper = true;
    jnt_constraint.jnt_lmt[3].upper = jc.j4.upper;
    jnt_constraint.jnt_lmt[3].has_lower = true;
    jnt_constraint.jnt_lmt[3].lower = jc.j4.lower;
    jnt_constraint.jnt_lmt[3].has_max_omega = true;
    jnt_constraint.jnt_lmt[3].max_omega = jc.j4.max_omega;
    jnt_constraint.jnt_lmt[3].has_max_alpha = true;
    jnt_constraint.jnt_lmt[3].max_alpha = jc.j4.max_alpha;

    jnt_constraint.jnt_lmt[4].has_zero = true;
    jnt_constraint.jnt_lmt[4].zero = jc.j5.home;
    jnt_constraint.jnt_lmt[4].has_upper = true;
    jnt_constraint.jnt_lmt[4].upper = jc.j5.upper;
    jnt_constraint.jnt_lmt[4].has_lower = true;
    jnt_constraint.jnt_lmt[4].lower = jc.j5.lower;
    jnt_constraint.jnt_lmt[4].has_max_omega = true;
    jnt_constraint.jnt_lmt[4].max_omega = jc.j5.max_omega;
    jnt_constraint.jnt_lmt[4].has_max_alpha = true;
    jnt_constraint.jnt_lmt[4].max_alpha = jc.j5.max_alpha;

    jnt_constraint.jnt_lmt[5].has_zero = true;
    jnt_constraint.jnt_lmt[5].zero = jc.j6.home;
    jnt_constraint.jnt_lmt[5].has_upper = true;
    jnt_constraint.jnt_lmt[5].upper = jc.j6.upper;
    jnt_constraint.jnt_lmt[5].has_lower = true;
    jnt_constraint.jnt_lmt[5].lower = jc.j6.lower;
    jnt_constraint.jnt_lmt[5].has_max_omega = true;
    jnt_constraint.jnt_lmt[5].max_omega = jc.j6.max_omega;
    jnt_constraint.jnt_lmt[5].has_max_alpha = true;
    jnt_constraint.jnt_lmt[5].max_alpha = jc.j6.max_alpha;

    return jnt_constraint;
}
U64 RobotMotion::setSoftConstraint(motion_spec_JointConstraint *jnt_constraint)
{
    JointConstraint jc = arm_group_->getSoftConstraint();
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

    if (arm_group_->setSoftConstraint(jc) != TPI_SUCCESS)
        return INVALID_PARAM_FROM_TP; 

    return TPI_SUCCESS;
}

motion_spec_JointConstraint RobotMotion::getHardConstraint()
{
    JointConstraint jc = arm_group_->getHardConstraint();
    motion_spec_JointConstraint jnt_constraint;
    jnt_constraint.jnt_lmt_count = MAX_JOINTS;

    jnt_constraint.jnt_lmt[0].has_upper = true;
    jnt_constraint.jnt_lmt[0].upper = jc.j1.upper;
    jnt_constraint.jnt_lmt[0].has_lower = true;
    jnt_constraint.jnt_lmt[0].lower = jc.j1.lower;

    jnt_constraint.jnt_lmt[1].has_upper = true;
    jnt_constraint.jnt_lmt[1].upper = jc.j2.upper;
    jnt_constraint.jnt_lmt[1].has_lower = true;
    jnt_constraint.jnt_lmt[1].lower = jc.j2.lower;

    jnt_constraint.jnt_lmt[2].has_upper = true;
    jnt_constraint.jnt_lmt[2].upper = jc.j3.upper;
    jnt_constraint.jnt_lmt[2].has_lower = true;
    jnt_constraint.jnt_lmt[2].lower = jc.j3.lower;

    jnt_constraint.jnt_lmt[3].has_upper = true;
    jnt_constraint.jnt_lmt[3].upper = jc.j4.upper;
    jnt_constraint.jnt_lmt[3].has_lower = true;
    jnt_constraint.jnt_lmt[3].lower = jc.j4.lower;

    jnt_constraint.jnt_lmt[4].has_upper = true;
    jnt_constraint.jnt_lmt[4].upper = jc.j5.upper;
    jnt_constraint.jnt_lmt[4].has_lower = true;
    jnt_constraint.jnt_lmt[4].lower = jc.j5.lower;

    jnt_constraint.jnt_lmt[5].has_upper = true;
    jnt_constraint.jnt_lmt[5].upper = jc.j6.upper;
    jnt_constraint.jnt_lmt[5].has_lower = true;
    jnt_constraint.jnt_lmt[5].lower = jc.j6.lower;

    return jnt_constraint;
}


/**
 * @brief : right now this function isn't used 
 *
 * @param jnt_constraint
 *
 * @return 
 */
U64 RobotMotion::setHardConstraint(motion_spec_JointConstraint *jnt_constraint)
{
    JointConstraint jc;
    if (jnt_constraint->jnt_lmt_count < 6)
        return INVALID_PARAM_FROM_TP;

    if (jnt_constraint->jnt_lmt[0].has_upper)
        jc.j1.upper = jnt_constraint->jnt_lmt[0].upper;
    if (jnt_constraint->jnt_lmt[0].has_lower)
        jc.j1.lower = jnt_constraint->jnt_lmt[0].lower;

    if (jnt_constraint->jnt_lmt[1].has_upper)
        jc.j2.upper = jnt_constraint->jnt_lmt[1].upper;
    if (jnt_constraint->jnt_lmt[1].has_lower)
        jc.j2.lower = jnt_constraint->jnt_lmt[1].lower;

    if (jnt_constraint->jnt_lmt[2].has_upper)
        jc.j3.upper = jnt_constraint->jnt_lmt[2].upper;
    if (jnt_constraint->jnt_lmt[2].has_lower)
        jc.j3.lower = jnt_constraint->jnt_lmt[2].lower;

    if (jnt_constraint->jnt_lmt[3].has_upper)
        jc.j4.upper = jnt_constraint->jnt_lmt[3].upper;
    if (jnt_constraint->jnt_lmt[3].has_lower)
        jc.j4.lower = jnt_constraint->jnt_lmt[3].lower;

    if (jnt_constraint->jnt_lmt[4].has_upper)
        jc.j5.upper = jnt_constraint->jnt_lmt[4].upper;
    if (jnt_constraint->jnt_lmt[4].has_lower)
        jc.j5.lower = jnt_constraint->jnt_lmt[4].lower;

    if (jnt_constraint->jnt_lmt[5].has_upper)
        jc.j6.upper = jnt_constraint->jnt_lmt[5].upper;
    if (jnt_constraint->jnt_lmt[5].has_lower)
        jc.j6.lower = jnt_constraint->jnt_lmt[5].lower;

    return TPI_SUCCESS;
}


motion_spec_DHGroup RobotMotion::getDHGroup()
{
    DHGroup dh = arm_group_->getDH();
    motion_spec_DHGroup dh_group;
    dh_group.coord_offset_count = 6;
    dh_group.coord_offset[0].alpha = dh.j1.alpha;
    dh_group.coord_offset[0].a = dh.j1.a;
    dh_group.coord_offset[0].d = dh.j1.d;
    dh_group.coord_offset[0].theta = dh.j1.theta;

    dh_group.coord_offset[1].alpha = dh.j2.alpha;
    dh_group.coord_offset[1].a = dh.j2.a;
    dh_group.coord_offset[1].d = dh.j2.d;
    dh_group.coord_offset[1].theta = dh.j2.theta;

    dh_group.coord_offset[2].alpha = dh.j3.alpha;
    dh_group.coord_offset[2].a = dh.j3.a;
    dh_group.coord_offset[2].d = dh.j3.d;
    dh_group.coord_offset[2].theta = dh.j3.theta;

    dh_group.coord_offset[3].alpha = dh.j4.alpha;
    dh_group.coord_offset[3].a = dh.j4.a;
    dh_group.coord_offset[3].d = dh.j4.d;
    dh_group.coord_offset[3].theta = dh.j4.theta;
    
    dh_group.coord_offset[4].alpha = dh.j5.alpha;
    dh_group.coord_offset[4].a = dh.j5.a;
    dh_group.coord_offset[4].d = dh.j5.d;
    dh_group.coord_offset[4].theta = dh.j5.theta;

    dh_group.coord_offset[5].alpha = dh.j6.alpha;
    dh_group.coord_offset[5].a = dh.j6.a;
    dh_group.coord_offset[5].d = dh.j6.d;
    dh_group.coord_offset[5].theta = dh.j6.theta;

    return dh_group;
}


/**
 * @brief : right now this function isn't used 
 *
 * @param dh_group
 *
 * @return 
 */
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



double RobotMotion::getGlobalVelocity()
{
    return vel_factor_;
}
U64 RobotMotion::setGlobalVelocity(double factor)
{
    /*if ((factor <= 0) || (factor > 100))*/
        //return INVALID_PARAM_FROM_TP;

    //boost::mutex::scoped_lock lock(mutex_);
    //vel_factor_ = factor;

    /*return TPI_SUCCESS;*/
    if (arm_group_->setVelocityScalingFactor(factor) == false)
    {
        return INVALID_PARAM_FROM_TP;
    }
    
    vel_factor_ = factor;
    return TPI_SUCCESS;
}


U64 RobotMotion::setTempZero()
{
    U64 result;
    if(arm_group_->setTempZeroOffset(result))
    {
        setRunningMode(CALIBRATE_R);
        setProgramStateCmd(GOTO_IDLE_E);
    }
    return result;
}


void RobotMotion::processNonMove()
{
    
    while(1)
    {        
        switch (getNMPrgmState())
        {
            case IDLE_R: //this thread should exit
                if (getLogicMode() != AUTO_RUN_M)
                    break;
                if (!non_move_instructions_.empty())
                {
                    FST_INFO("non move goto EXECUTE_R");
                    setNMPrgmState(EXECUTE_R);
                }
                break; 
            case PAUSED_R:
                if (non_move_instructions_.empty()) 
                {
                    setNMPrgmState(IDLE_R);
                    break;
                }
                if (non_move_instructions_.front().commandtype != motion_spec_MOTIONTYPE_WAIT)
                {
                    setNMPrgmState(EXECUTE_R);
                }
                break;
            case EXECUTE_R:
                if (non_move_instructions_.empty())
                {
                    setNMPrgmState(IDLE_R);
                    FST_INFO("non move changed IDLE_R");
                    break;
                }
               // if (!non_move_instructions_.empty())
                {                    
                    
                    ThreadSafeList<CommandInstruction>::iterator it;
                    for (it = non_move_instructions_.begin(); it != non_move_instructions_.end(); it++)
                    {
                        if (it->commandtype == motion_spec_MOTIONTYPE_WAIT)
                        {
                            if (it->pick_status == PICKED)
                                continue;
                            motion_spec_Wait *wait = (motion_spec_Wait*)it->command_arguments.c_str();
                            if (wait->has_value)
                            {
                                unsigned char value;
                                int len;
                                io_interface_->getDIO(wait->path, &value, 1, len); 
                                //FST_INFO("setval:%d, value:%d", wait->value, value);
                                if (wait->value == value)
                                {
                                    goto WAIT_END;
                                }
                            }
                            if ((wait->has_timeout) && (it->count-- <= 0))
                            {
                                goto WAIT_END;                                
                            }
                            break;
                   WAIT_END:it->pick_status = PICKED;
                            setServoWaitFlag(false);
                        }
                        else if (it->commandtype == motion_spec_MOTIONTYPE_SET)
                        {
                            if (it->pick_status == PICKED)
                                continue;
                            if (it->count-- > 0)
                                continue;
                            
                            motion_spec_Set *set = (motion_spec_Set*)it->command_arguments.c_str();
                            io_interface_->setDO(set->path, set->value);
                            it->pick_status = PICKED;
                        }
                    }//end for (it = rob_motion
                }//end if (!non_move_instructions_.empty())
                //////////////////////////////////////////////////////////////////////
                //delete all the used instructions
                //////////////////////////////////////////////////////////////////////
                while(!non_move_instructions_.empty())
                {
                    if (non_move_instructions_.front().pick_status == PICKED)
                    {
                        non_move_instructions_.pop_front();
                    }
                    else
                    {
                        break;
                    }
                }
                break;
            //case EXECUTE_TO_PAUSE_T:          
            //    break;
            //case PAUSE_TO_IDLE_T:
            //    break;
            default:
                break;
        }
        usleep(NON_MOVE_INTERVAL*1000); //sleep 10ms
    }
}

void RobotMotion::resetSafetyBoard()
{
    safety_interface_.setDOType0Stop(0);
    safety_interface_.setDOType1Stop(0);
    safety_interface_.setDOType2Stop(0);
}

/**
 * @brief: convert unit from params of TP to controller
 *
 * @param src_moveL: input==>struct  covert from
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
void RobotMotion::unitConvert(const motion_spec_MoveJ *src_moveJ, Joint &dst_joints, MoveJParam &dst_movej_param)
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

void RobotMotion::setPrevTarget(PointType type, PointVal point)
{
    boost::mutex::scoped_lock lock(mutex_);
    pre_target_.type = type;
    pre_target_.point = point;
}

Joint RobotMotion::getStartJoint()
{
    Joint start_joints;
    boost::mutex::scoped_lock lock(mutex_);
    if (pre_target_.type == JOINT_M)    
    {
        start_joints = pre_target_.point.jnt_val;  //get the start joints
    }
    else
    {
        getJointFromPose(pre_target_.point.pose_val, start_joints);
    }
    return start_joints;
}

PoseEuler RobotMotion::getStartPose()
{
    PoseEuler start_pose; 
    boost::mutex::scoped_lock lock(mutex_);
    if (pre_target_.type == JOINT_M)    
    {
        getPoseFromJoint(pre_target_.point.jnt_val, start_pose);
    }
    else
    {
        start_pose = pre_target_.point.pose_val;
    } 

    return start_pose;
}


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
            Joint target_jnts;
            MoveJParam movej_param;
            unitConvert(moveJ, target_jnts, movej_param);
            FST_INFO("target joints:%f,%f,%f,%f,%f,%f", target_jnts.j1, target_jnts.j2, target_jnts.j3, target_jnts.j4, target_jnts.j5, target_jnts.j6);                
            
            if (target_inst.smoothDistance >= 0)
            {
                Joint start_jnts = getStartJoint();
                Joint new_target;
                new_target.j1 = (start_jnts.j1 + target_jnts.j1) / 2;
                new_target.j2 = (start_jnts.j2 + target_jnts.j2) / 2;
                new_target.j3 = (start_jnts.j3 + target_jnts.j3) / 2;
                new_target.j4 = (start_jnts.j4 + target_jnts.j4) / 2;
                new_target.j5 = (start_jnts.j5 + target_jnts.j5) / 2;
                new_target.j6 = (start_jnts.j6 + target_jnts.j6) / 2;

                arm_group_->MoveJ(new_target,movej_param.vel_max, movej_param.acc_max,\
                        MAX_CNT_VAL, target_jnts, movej_param.vel_max, movej_param.acc_max,\
                        MAX_CNT_VAL, target_inst.id, result);

                //setServoWaitFlag(false);
            }//end if (target_inst.smoothDistance >= 0)
            else
            {
                arm_group_->MoveJ(target_jnts, movej_param.vel_max, \
                            movej_param.acc_max, target_inst.id, result);
                target_inst.smoothDistance = -1;
                setServoWaitFlag(true);

            }//end else
            //==========set previous target=============
            PointVal point;
            point.jnt_val = target_jnts;
            setPrevTarget(JOINT_M, point);
            ////////////////////////////////////////// 
            break;
        }//end case motion_spec_MOTIONTYPE_JOINTMOTION:
        case motion_spec_MOTIONTYPE_CARTMOTION:
        {
            motion_spec_MoveL* moveL = (motion_spec_MoveL*)target_inst.command_arguments.c_str();
            moveL->waypoints[0].smoothPercent = target_inst.smoothDistance; //first rewrite this value
            PoseEuler start_pose = getStartPose();
            //printDbLine("start_pose:", (double*)&start_pose, 6);
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

               // setServoWaitFlag(false);
            }//end if (target_inst.smoothDistance >= 0)
            else
            {
                arm_group_->MoveL(target_pose, movel_param.vel_max, \
                            movel_param.acc_max, target_inst.id, result);
                target_inst.smoothDistance = -1;
                setServoWaitFlag(true);
            }
            //==========set previous target=============
            PointVal point;
            point.pose_val = target_pose;
            setPrevTarget(CART_M, point);
            ////////////////////////////////////////// 
            break;
        }//end case motion_spec_MOTIONTYPE_CARTMOTION:
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
            target_inst.smoothDistance = -1;
            //==========set previous target=============
            PointVal point;
            point.pose_val = target_pose2;
            setPrevTarget(CART_M, point);
            ////////////////////////////////////////// 
            break;
        }//end case motion_spec_MOTIONTYPE_CIRCLEMOTION:
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
            Joint target_jnts;
            MoveJParam target_param;
            unitConvert(moveJ, target_jnts, target_param);
            FST_INFO("target joints:%f,%f,%f,%f,%f,%f, smmoth:%f", target_jnts.j1, target_jnts.j2, target_jnts.j3, target_jnts.j4, target_jnts.j5, target_jnts.j6, target_inst.smoothDistance);                 
            
            if (next_inst.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
            {
                motion_spec_MoveJ* moveJ_next =  (motion_spec_MoveJ*)next_inst.command_arguments.c_str();
                Joint next_jnts;
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

            //setServoWaitFlag(false);
            break;
        }//end case motion_spec_MOTIONTYPE_JOINTMOTION:     
        case motion_spec_MOTIONTYPE_CARTMOTION:
        {
            motion_spec_MoveL* moveL = (motion_spec_MoveL*)target_inst.command_arguments.c_str();
            moveL->waypoints[0].blendInDistance = target_inst.smoothDistance; //first rewrite this value
            //PoseEuler start_pose = getStartPose();
            PoseEuler target_pose;
            MoveLParam target_param;
            unitConvert(moveL, target_pose, target_param);
            FST_INFO("target pose:%f,%f,%f,%f,%f,%f,smooth:%f", target_pose.position.x,target_pose.position.y,target_pose.position.z, target_pose.orientation.a, target_pose.orientation.b, target_pose.orientation.c, target_inst.smoothDistance);
            if (next_inst.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
            {
                motion_spec_MoveJ* moveJ_next =  (motion_spec_MoveJ*)next_inst.command_arguments.c_str();
                Joint next_jnts;
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
            //setServoWaitFlag(false);
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
                Joint next_jnts;
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
                        next_param.acc_max, next_param.smooth, target_inst.id, result);
            }  
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
           // setServoWaitFlag(false);
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

    CommandInstruction *target_instruction; 

    target_instruction = pickMoveInstruction();
    if (target_instruction == NULL) 
    {
        return TPI_SUCCESS;
    }


    if (target_instruction->smoothDistance >= 0)
    {
        CommandInstruction *next_instruction = pickNextMoveInstruction(target_instruction);
        if (next_instruction) //find next_instruction
        {
            result = moveInstructions(*target_instruction, *next_instruction); 
        }
        else 
        {
            //after moving to the middle still no next instruction
            //move as fine
            result = moveInstructions(*target_instruction);
        }
    }//end if (target_instruction->smoothDistance >= 0)
    else
    {
        result = moveInstructions(*target_instruction);     
    }//end else
    
    if (result != TPI_SUCCESS)
    {
        FST_ERROR("error move instruction:%d, %llx", target_instruction->id, result);
        target_instruction->pick_status = FRESH;          
    }

    //FST_INFO("planFifo end");
    return result;
}


bool RobotMotion::isFifoEmpty()
{
    int traj_len = arm_group_->getPlannedPathFIFOLength();
	int joints_len = arm_group_->getTrajectoryFIFOLength();

    //FST_INFO("the fifo size:%d, joints:%d", traj_len, joints_len);
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

//pop until current id
void RobotMotion::popupInstruction(int id)
{
    int cur_id = id, next_id;
    FST_INFO("pop until id:%d", id);
    boost::mutex::scoped_lock lock(mutex_);
    while (!instruction_list_.empty())
    {
        CommandInstruction inst = instruction_list_.front();
        next_id = inst.id;        
        if(id != next_id)
        {            

            FST_INFO("1poped instruction id:%d", next_id);
            instruction_list_.pop_front();
            cur_id = next_id;
        }
        else
        {
            break;
        }
  
    }//end while (!instruction_list_.empty())

    if(id != next_id)
    {
        FST_ERROR("pop instruction id:%d is not the same as current id:%d",id, cur_id);
    }
    else
    {
        if (!instruction_list_.empty())
        {
            if (instruction_list_.front().pick_status != ONCE)
            {
                FST_INFO("pop up id:%d", instruction_list_.front().id);
                instruction_list_.pop_front();
            }
        }
    }
}

 void RobotMotion::popupInstEnd()
{
    int cur_id = getCurrentCmdID(), next_id;

    while (!instruction_list_.empty())
    {
        CommandInstruction inst = instruction_list_.front();
        next_id = inst.id;        

        //===pop up zero instructions=====
        if (inst.pick_status == NEVER)
        {    
            FST_INFO("poped instruction id:%d", next_id);
            setPreviousCmdID(cur_id);
            setCurrentCmdID(next_id);  
            instruction_list_.pop_front();
            cur_id = next_id;
            continue;
        }
    
        //this instruction should setio or other non move instruction
        //===run this instruction===     
        if ((inst.commandtype != motion_spec_MOTIONTYPE_JOINTMOTION) 
        && (inst.commandtype != motion_spec_MOTIONTYPE_CARTMOTION)
        && (inst.commandtype != motion_spec_MOTIONTYPE_CIRCLEMOTION))
        {    
            FST_INFO("non move instructions...");                
            addNonMoveInstruction(inst);
            setPreviousCmdID(cur_id);
            setCurrentCmdID(next_id);
            instruction_list_.pop_front();
            cur_id = next_id;
            continue;
        }
        else
        {
            break;
        }
    }//end while (!instruction_list_.empty())

    setPreviousCmdID(getCurrentCmdID());            
    setCurrentCmdID(-1);
    //setServoWaitFlag(false);
}


void RobotMotion::resetInstructionList()
{
    boost::mutex::scoped_lock lock(mutex_);
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
	//int joints_len = arm_group_->getTrajectoryFIFOLength();
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

    //arm_group_->convertPathToTrajectory(MAX_PLANNED_POINTS_NUM, result);
    return result;
}

void RobotMotion::clearFIFO1Plot()
{
    
}

void RobotMotion::clearFIFO2Plot()
{

}

void RobotMotion::fillInFIFO1(const Pose pose)
{
    PoseEuler epose;
	//set the current joint to lib
    epose = arm_group_->transformPose2PoseEuler(pose);
    int count = fifo1_.count++;
    writeLock wlock(fifo1_.rwmux); 
    memcpy(fifo1_.data+count, (char*)&epose, sizeof(PoseEuler));    
}


void RobotMotion::fillInFIFO2(const double *points)
{
    writeLock wlock(fifo2_.rwmux);
    int count = fifo2_.count++;
    if (count >= MAX_FIFO_LEN)
    {
        FST_INFO("fifo2_ full");
        fifo2_.count = 0;
        return;
    }
     
    memcpy(fifo2_.data+count, points, sizeof(double)*MAX_JOINTS);    
}

void RobotMotion::writePosToShm(JointCommand jc_w)
{
    if (TPI_SUCCESS == share_mem_.setJointPositions(jc_w))
    {
        if (jc_w.total_points < JC_POINT_NUM)   //in case can't detect running state
        {
            if (getServoState() == STATE_READY)
            {
                setServoState(STATE_RUNNING);
            }
        }
    }
}


U64 RobotMotion::sendJointsToRemote()
{
    U64 result = TPI_SUCCESS;

    if (share_mem_.isJointCommandWritten() == false)
    {
        writePosToShm(share_mem_.getCurrentJointCmd().joint_cmd);

        return TPI_SUCCESS;
    }//end if (share_mem_.isJointCommandWritten() == false)
            
    int joints_len = arm_group_->getTrajectoryFIFOLength();          

    if (joints_len > 0)
    {
    	//FST_INFO("joints fifo length:%d, traj_len:%d", joints_len, arm_group_->getPlannedPathFIFOLength());

        vector<JointOutput> joint_traj;
        U64 result = arm_group_->getPointsFromJointTrajectoryFIFO(joint_traj);
        if (result != TPI_SUCCESS)
        {
           // FST_ERROR("=======pick point failed ==========");
            return result;
        }
        //FST_INFO("joint in :%d",joints_len);		
        //if (result == TPI_SUCCESS)
        //{
            int joints_in = joint_traj.size();
            dbcount+=joints_in;
            //printDbLine("joints:", (double*)&joint_traj[joints_in-1].joint, 6);
            //joints_len = arm_group_->getTrajectoryFIFOLength();
            //FST_INFO("joints fifo length:%d", joints_len);
            JointCommand joint_command;
            joint_command.total_points = joints_in;
            for (int i = 0; i < joints_in; i++)
            {
                joint_command.points[i].positions[0] = joint_traj[i].joint.j1;
                joint_command.points[i].positions[1] = joint_traj[i].joint.j2;
                joint_command.points[i].positions[2] = joint_traj[i].joint.j3;
                joint_command.points[i].positions[3] = joint_traj[i].joint.j4;
                joint_command.points[i].positions[4] = joint_traj[i].joint.j5;
                joint_command.points[i].positions[5] = joint_traj[i].joint.j6;
#ifdef PLOT
                fillInFIFO2(joint_command.points[i].positions);
#endif
                joint_command.points[i].point_position = joint_traj[i].level; //point position: start\middle\ending
                
                //printDbLine("joints:", joint_command.points[i].positions, 6);
                if (getLogicMode() == AUTO_RUN_M)
                {
                    int traj_id = joint_traj[i].id;
                    int cur_id = getCurrentCmdID();
                    if (cur_id != traj_id)
                    {
                        setPreviousCmdID(cur_id);
                        setCurrentCmdID(traj_id);  
                    }

                    int level = joint_command.points[i].point_position;
                    if (level == POINT_LAST)
                    {
                        printDbLine("the last joints:", joint_command.points[i].positions, 6);                   
                    }
                    if (level == POINT_ENDING)  
                    {
                        if (getProgramState() == EXECUTE_R) //in case for pausing
                        {
                            popupInstruction(traj_id);
                        }
                        printDbLine("the end joints:", joint_command.points[i].positions, 6);
                                 
                        FST_INFO("traj_id is :%d", cur_id);
                    } 
                    else if (joint_command.points[i].point_position == POINT_START)
                    {
                        printDbLine("the start joints:", joint_command.points[i].positions, 6);
                        //FST_INFO("traj_id is :%d, cur_id:%d", traj_id, cur_id);
                    }
                    else if (joint_command.points[i].point_position == POINT_FIRST)
                    {
                        if (traj_id != cur_id) //in case in middle of path
                        {
                            popupInstruction(cur_id);
                        }
                        printDbLine("the first joints:", joint_command.points[i].positions, 6);
                        //FST_INFO("traj_id is :%d, cur_id:%d", traj_id, cur_id);
                    }
                }
            }// end for (int i = 0; i < joints_in; i++)



            share_mem_.setCurrentJointCmd(joint_command); //store this command in case it can't write Success
            writePosToShm(joint_command); //send joints to bare metal
        //}//end if (joints_in > 0)]
        
    }//end  if (joints_len > 0)


    return result;
}

void RobotMotion::checkInstID(JointCommand joint_command)
{
    if (getLogicMode() != AUTO_RUN_M)
        return;
    int cur_id = getCurrentCmdID();
    for (int i = 0; i < joint_command.total_points; i++)
    {
        int level = joint_command.points[i].point_position;   
                   
        if (level == POINT_LAST)
        {
            printDbLine("the last joints:", joint_command.points[i].positions, 6);
            popupInstruction(cur_id);
            setPreviousCmdID(cur_id);
            setCurrentCmdID(cur_id);
            
        }
        if (level == POINT_ENDING)  
        {
            printDbLine("the end joints:", joint_command.points[i].positions, 6);
            popupInstruction(cur_id);
            //setServoWaitFlag(false); 
            setPreviousCmdID(cur_id);
            setCurrentCmdID(cur_id);            
            FST_INFO("traj_id is :%d", cur_id);
        } 
        else if (joint_command.points[i].point_position == POINT_START)
        {
            printDbLine("the start joints:", joint_command.points[i].positions, 6);
            //FST_INFO("traj_id is :%d, cur_id:%d", traj_id, cur_id);
        }
        else if (joint_command.points[i].point_position == POINT_FIRST)
        {
            printDbLine("the first joints:", joint_command.points[i].positions, 6);
            //FST_INFO("traj_id is :%d, cur_id:%d", traj_id, cur_id);
        }

    }//end if (getLogicMode() == AUTO_RUN_M)
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
    //========setPrevTarget====================================
    setPrevTarget(manu_req.target.type, manu_req.target.point);
    //////////////////////////////////////////////////////////
    if (manu_req.vel_max == 0) 
    {
        manu_req.target.is_step = false;
        return TPI_SUCCESS;
    }
    else if (manu_req.ref_vmax == 0)
    {
        manu_req.target.is_step = true;
        /*setProgramStateCmd(GOTO_PAUSED_E);*/
        setServoWaitFlag(true);
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

    if (instruction_list_.empty())
    {
       /*if (isServoReady() && isFifoEmpty())*/
       //{
            //setPreviousCmdID(getCurrentCmdID());            
            //setCurrentCmdID(-1);
            //setServoWaitFlag(false);
       /*}*/
       return NULL;
    }
    {
        static ThreadSafeList<CommandInstruction>::iterator it;
        boost::mutex::scoped_lock lock(mutex_);
        for (it = instruction_list_.begin(); it != instruction_list_.end(); it++)
        {
            if (it->pick_status == USING) 
            {
                //FST_INFO("this has picked:%d", it->id);
                continue;
            }
            if ((it->commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
            || (it->commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
            || (it->commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION))
            {
                if ((it->pick_status == PICKED) /*|| (it->pick_status == NEVER)*/)
                {
                    continue;
                }
                FST_INFO("find one:id==>%d, smooth:%f, pick_status:%d", it->id, it->smoothDistance, it->pick_status);
                
                if (it-> pick_status == FRESH)
                {
                    ThreadSafeList<CommandInstruction>::iterator next_it = it;
                    if ((it->smoothDistance > 0) 
                    && (++next_it == instruction_list_.end())
                    && (it->commandtype != motion_spec_MOTIONTYPE_CIRCLEMOTION))
                    {
                        it->pick_status = ONCE;
                    }
                    else
                    {
                        it->pick_status = USING;
                    }
                }
                else if (it->pick_status == ONCE)
                {
                    it->pick_status = USING;
                    it->smoothDistance = -1;
                }
                FST_INFO("pick status is:%d", it->pick_status);
                instruction = &(*it);
                return instruction;
            }// end if ((it->commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
            else
            {
                addNonMoveInstruction(*it); //process starting non move instructions
                it->pick_status = USING;
                setPreviousCmdID(getCurrentCmdID());
                setCurrentCmdID(it->id); 
                FST_INFO("non move instruction type:%d", it->commandtype);
                if (it->commandtype == motion_spec_MOTIONTYPE_WAIT) //
                {
                    setServoWaitFlag(true);
                    return NULL;
                }
            }
        }// end for (it = instruction_list_.begin(); it != instruction_list_.end(); it++)
    }
    if (getNMPrgmState() == IDLE_R)
        popupInstruction(getCurrentCmdID());
    return NULL;
}    

CommandInstruction* RobotMotion::pickNextMoveInstruction(CommandInstruction *target_inst)
{
    CommandInstruction *instruction;
    boost::mutex::scoped_lock lock(mutex_);
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
            target_inst->pick_status = USING;
            target_inst->smoothDistance = -1;
            return NULL;
        }
    }//end for (it = instruction_list_.begin()

    FST_INFO("can't find next id");
    return NULL;
}  


bool RobotMotion::getServoWaitFlag()
{
    return servo_ready_wait_;
}

void RobotMotion::setServoWaitFlag(bool flag)
{
    //FST_INFO("servo flag :%d", flag);
    servo_ready_wait_ = flag;
}

void RobotMotion::processEndingMove()
{
    if (getLogicMode() != AUTO_RUN_M)
        return;    
    /*if (!getServoWaitFlag())*/
        /*return;*/
    if (!isServoReady())
        return;
    if (!isFifoEmpty()) 
        return;
    
    int cur_id = getCurrentCmdID(), next_id;

    while (!instruction_list_.empty())
    {
        CommandInstruction inst = instruction_list_.front();
        next_id = inst.id;        

        //===pop up zero instructions=====
        if (inst.pick_status == NEVER)
        {    
            FST_INFO("2poped instruction id:%d", next_id);
            setPreviousCmdID(cur_id);
            setCurrentCmdID(next_id);  
            instruction_list_.pop_front();
            cur_id = next_id;
            continue;
        }
    
        //this instruction should setio or other non move instruction
        //===run this instruction===     
        if ((inst.commandtype != motion_spec_MOTIONTYPE_JOINTMOTION) 
        && (inst.commandtype != motion_spec_MOTIONTYPE_CARTMOTION)
        && (inst.commandtype != motion_spec_MOTIONTYPE_CIRCLEMOTION))
        {    
            FST_INFO("non move instructions...");                
            addNonMoveInstruction(inst);
            setPreviousCmdID(cur_id);
            setCurrentCmdID(next_id);
            instruction_list_.pop_front();
            cur_id = next_id;
            continue;
        }
        else
        {
            break;
        }
    }//end while (!instruction_list_.empty())
    if (instruction_list_.empty())
    {
        setPreviousCmdID(getCurrentCmdID());            
        setCurrentCmdID(-1);
    }
    //setServoWaitFlag(false);
    //FST_INFO("prev_id:%d, cur_id:%d", previous_command_id_, current_command_id_);
}


U64 RobotMotion::addNonMoveInstruction(CommandInstruction instruction)
{
    if (instruction.commandtype == motion_spec_MOTIONTYPE_SET)
    {
        motion_spec_Set *set = (motion_spec_Set*)instruction.command_arguments.c_str();                            
        //FST_INFO("set do:path:%s, value:%d, count:%d", set->path, set->value, instruction.count);
        io_interface_->setDO(set->path,  set->value);
        if (instruction.count > 0)
        {
            set->value = !set->value;
        }
        instruction.pick_status = USING;
    }
    non_move_instructions_.push_back(instruction); 

    return TPI_SUCCESS;
}

RobotMode RobotMotion::getPrevMode()
{
    return prev_mode_;
}

void RobotMotion::setPrevMode(RobotMode mode)
{
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
        boost::mutex::scoped_lock lock(mutex_);
        if (!instruction_list_.empty())
        {
            //boost::thread thrd_non_move(boost::bind(processNonMove, this));
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
        boost::mutex::scoped_lock lock(mutex_);
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
            //clearPathFifo();
            setServoWaitFlag(false);
            return true;   
        }
    }
    else if (mode == AUTO_RUN_M)
    {
        if ((getServoState() == STATE_READY)
        && isFifoEmpty()
        && instruction_list_.empty()
        && (getNMPrgmState() == IDLE_R))
        {
            setPreviousCmdID(getCurrentCmdID());
            setCurrentCmdID(-1);
            //in case pused motion changes to idle,
            //next motion need to be clear or resume
            abortMotion();   
            return true;
        }
    }

    return false;
}


void RobotMotion::clearManuMove()
{
    boost::mutex::scoped_lock lock(mutex_);
    memset((char*)&manu_req_, 0, sizeof(ManualReq));
}

double RobotMotion::calcuManualJointVel(const double *interval_jnts)
{          
    double max_interval = getMaxValue(interval_jnts, MAX_JOINTS);
    int id = getMaxValID(max_interval, interval_jnts, MAX_JOINTS);
    
    JointConstraint jnt_constraints = arm_group_->getSoftConstraint();
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
