/**
 * @file robot_motion_.cpp
 * @brief: seperate the operation of cnt= 0 and fine(cnt=-1)
 * @author WangWei
 * @version 1.2.0
 * @date 2016-12-20
 */
#include "robot_motion.h"
#include "stdio.h"
#include "common.h"
#include "safety/safety.h" 


#define MAX(a,b) ((a) > (b) ? (a) : (b))


RobotMotion::RobotMotion()
{
    servo_ready_wait_ = false;
	previous_command_id_ = -1;
	current_command_id_ = -1;
	next_move_id_  = -1; //unknown next move id
	next_move_instruction_.id = -1;
	manual_inst_delay_cnt_ = MANUAL_INSTRUCTION_DELAY / INTERVAL_PROCESS_UPDATE;
							
	prev_mode_ = INIT_M;
	mode_ = INIT_M;
    prev_state_ = ESTOP_S;
    state_ = ESTOP_S;
    err_flag_ = false;
    arm_group_ = new ArmGroup();
    
}

RobotMotion::~RobotMotion()
{
    if (arm_group_)
    {
        delete arm_group_;
    }
}


U64 RobotMotion::initial()
{
    U64 result;
    param_group_ = new fst_parameter::ParamGroup("share/tp_interface/config/motion_controller.yaml");
    if (param_group_ == NULL)
    {
        FST_ERROR("create arm_group_ failed");
        return CREATE_PARAM_GROUP_FAILED;
    }
    
    if (false == param_group_->uploadParam())
    {
        FST_ERROR("uploadParam failed");
        return param_group_->getLastError();
    }

    if((result = share_mem_.initial()) != FST_SUCCESS)
    {
        FST_ERROR("ShareMem init failed");
        return result;
    }

	arm_group_->initArmGroup(result);
	if (result != FST_SUCCESS)
    {
        FST_ERROR("create arm_group_ failed");
        return result;
    }

    /*result = share_mem_.resetBareMetal();*/
    //if (result != FST_SUCCESS)
    //{
        //FST_ERROR("resetBareMetal failed");
        //return result;
    /*}*/

    unsigned int zero_offset;
    if (true == arm_group_->checkZeroOffset(zero_offset, result))
    {
        if (FST_SUCCESS != result)
        {
            FST_ERROR("checkZeroOffset unsuccessful");
            return result;
        }
    }
    else
    {
        FST_ERROR("checkZeroOffset failed");
        return result;
    }
        	
	Transformation tool_frame;
	memset(&tool_frame, 0, sizeof(tool_frame));
	arm_group_->setToolFrame(tool_frame);

	prev_mode_ = PAUSE_M;
	mode_ = PAUSE_M;


    return FST_SUCCESS;
}

void RobotMotion::destroy()
{
    if (param_group_)
    {
        delete param_group_;
    }
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
U64 RobotMotion::updateLogicMode()
{
    U64 result = FST_SUCCESS;

	RobotMode mode = getLogicMode();
    RobotState state = getLogicState();

	switch(mode)
	{
		case AUTO_RUN_TO_PAUSE_T:
			result = actionPause();
			mode = PAUSE_M;
			break;
		case PAUSE_TO_AUTO_RUN_T:
            if(prev_mode_ != AUTO_RUN_M)
            {
                result = clearMotionQueue();
            }
            else
            {
                result = actionResume();
            }
			mode = AUTO_RUN_M;
			break;
		case PAUSE_TO_MANUAL_JOINT_T:
			mode = MANUAL_JOINT_MODE_M;
			break;
		case PAUSE_TO_MANUAL_CART_T:
			mode = MANUAL_CART_MODE_M;
			break;
		case MANUAL_CART_TO_PAUSE_T:
			result = actionPause();
			mode = PAUSE_M;
			break;
		case MANUAL_JOINT_TO_PAUSE_T:
			result = actionPause();
			mode = PAUSE_M;
			break;
		case MANUAL_CART_TO_MANUAL_JOINT_T:
			result = actionPause();
			mode = MANUAL_JOINT_MODE_M;
			break;
		case MANUAL_JOINT_TO_MANUAL_CART_T:
			result = actionPause();
			mode = MANUAL_CART_MODE_M;
			break;
		default:
            result = PARAMETER_NOT_UPDATED;
			break;
	}

    /*boost::mutex::scoped_lock lock(mutex_);*/
    /*if (mode != this->mode_)*/
    //{
      //  FST_INFO("update mode:%d,result:%lx", mode,result);
        //this->mode_ = mode;
    /*}*/
    setLogicMode(mode);
    
    return result;
}

U64 RobotMotion::updateLogicState()
{
    U64 result = FST_SUCCESS;

	RobotState state = getLogicState();
    
    switch (state)
    {
        case DISENGAGED_TO_OFF_T:
            state = OFF_S;
            break;
        case OFF_TO_DISENGAGED_T:            
            state = DISENGAGED_S;
            break;
        case OFF_TO_CALIBRATE_T:
            unsigned int caliborate_val;
            arm_group_->calibrateZeroOffset(caliborate_val, result);  //
            state = OFF_S;
            break;
        case ENGAGED_TO_DISENGAGED_T:
            //right now the mode should be in pause
            emergencyStop();            
            state = DISENGAGED_S;
            break;
        case DISENGAGED_TO_ENGAGED_T:
            result = share_mem_.resetBareMetal();
            resetInstructionID();
            resetQueue();
            state = ENGAGED_S;
            break;
        case TO_ESTOP_T:
            //right now the mode should be in pause 
            emergencyStop();
            state = ESTOP_S;
            break;
        case RESET_ESTOP_T:
            setErrorState(false);
            share_mem_.stopBareMetal();
            usleep(200*1000); //wait 200ms for self check
            //if there are still errors can't change state
            if (getErrorState() == false) //no errors exist
            {                
                resetInstructionID();
                resetQueue();
                state = OFF_S;
            }
            else
            {
                state = ESTOP_S;
            }
            break;
        default:
            result = PARAMETER_NOT_UPDATED;
            break;
    }

    setLogicState(state);

    return result;
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
 //   boost::mutex::scoped_lock lock(mutex_);
    U64 result = share_mem_.getFeedbackJointState(fbjs);
//    lock.unlock();
	if (result == FST_SUCCESS)
	{
        joints_val.j1 = fbjs.position[0];
        joints_val.j2 = fbjs.position[1];
        joints_val.j3 = fbjs.position[2];
        joints_val.j4 = fbjs.position[3];
        joints_val.j5 = fbjs.position[4];
        joints_val.j6 = fbjs.position[5];

        
        if (fbjs.state == STATE_READY)
        {
            if (getServoState() == STATE_RUNNING)
            {
                FST_INFO("state translate from running to ready");
            }
            
        //	FST_INFO("cur_joints:%f,%f,%f,%f,%f,%f", joints_.j1,joints_.j2,joints_.j3\
                    ,joints_.j4,joints_.j5,joints_.j6);
        }
        setServoState(fbjs.state);
        setCurJoints(joints_val);

	}
	
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
    return FST_SUCCESS;
}


U64 RobotMotion::updateSafetyStatus()
{
    U64 result;
    if (safety_interface_.isSafetyValid() == false)
    {
        return FST_SUCCESS;
    }

    char third_frm = getSafety(SAFETY_INPUT_THIRDFRAME, &result);
    if (result = FST_SUCCESS)
    {
        if (third_frm & 0xE0) //estop D5:ESTOP	D6:limited Estop D7:extEstop
        {
            result = SERVO_ESTOP;
            setLogicStateCmd(EMERGENCY_STOP_E);
        }
    }
    return result;
}

/**
 * @brief: set current mode
 *
 * @param: mode_cmd mode command
 *
 * @return: true if set successfully
 */
bool RobotMotion::setLogicModeCmd(RobotModeCmd mode_cmd)
{
	bool ret;

	RobotMode mode = getLogicMode();
    RobotState state = getLogicState();

	switch (mode_cmd)
	{
		case GOTO_PAUSE_E:
			switch(mode)
			{
				case AUTO_RUN_M:
					prev_mode_ = mode;
					mode = AUTO_RUN_TO_PAUSE_T;
					ret = true;
					break;
				case MANUAL_JOINT_MODE_M:
					prev_mode_ = mode;
					mode = MANUAL_JOINT_TO_PAUSE_T;
					ret = true;
					break;
				case MANUAL_CART_MODE_M:
					prev_mode_ = mode;
					mode = MANUAL_CART_TO_PAUSE_T;
					ret = true;
					break;
				case INIT_M:
				case AUTO_RESET_M:
					prev_mode_ = mode;
					mode = PAUSE_M;
					ret = true;
					break;
				case PAUSE_M:
					ret = true;
					break;
				default:
					ret = false;
					break;
			}
			break;
		case GOTO_AUTO_RUN_E:
            if (state != ENGAGED_S)
            {
                ret = false;
            }
            else
            {
                if (mode == AUTO_RUN_M)
                {
                    ret = true;
                }
                else if (mode == PAUSE_M)
                {
                    mode = PAUSE_TO_AUTO_RUN_T;
                    ret = true;
                }
                else 
                    ret = false;
            }
			break;
		case GOTO_MANUAL_JOINT_MODE_E:
            if (state != ENGAGED_S)
            {
                ret = false;
            }
            else
            {
                clearMotionQueue();
                if (mode == MANUAL_JOINT_MODE_M)
                {
                    ret = true;
                }
                else if (mode == PAUSE_M)
                {
                    mode = PAUSE_TO_MANUAL_JOINT_T;
                    ret = true;
                }
                else if (mode == MANUAL_CART_MODE_M)
                {
                    mode = MANUAL_CART_TO_MANUAL_JOINT_T;
                    ret = true;
                }
                else 
                    ret = false;
            }
			break;
		case GOTO_MANUAL_CART_MODE_E:
            if (state != ENGAGED_S)
            {
                ret = false;
            }
            else
            {
                clearMotionQueue();
                if (mode == MANUAL_CART_MODE_M)
                {
                    ret = true;
                }
                else if (mode == PAUSE_M)
                {
                    mode = PAUSE_TO_MANUAL_CART_T;
                    ret = true;
                }
                else if (mode == MANUAL_JOINT_MODE_M)
                {
                    mode = MANUAL_JOINT_TO_MANUAL_CART_T;
                    ret = true;
                }
                else 
                    ret = false;
            }
			break;
		case GOTO_AUTO_RESET_E:
            if (mode == AUTO_RESET_M)
            {
                ret = true;
            }
			else if (mode == PAUSE_M)
			{
				mode = AUTO_RESET_M;
				clearMotionQueue();
				ret = true;
			}
			else 
				ret = false;
			break;
		default:
			ret = false;
			break;
	}

	FST_INFO("current mode:%d", mode);
    setLogicMode(mode);

	return ret;
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


/**
 * @brief: set current logic state
 *
 * @param state_cmd: input==> state command
 *
 * @return 
 */
bool RobotMotion::setLogicStateCmd(RobotStateCmd state_cmd)
{
    bool ret = true;

	RobotState state = getLogicState();
        
    //FST_INFO("state_cmd:%d",state_cmd);
	switch (state_cmd)
	{
        case GOTO_OFF:
            if (state == OFF_S)
            {
                ret = true;
            }
            else if (state == DISENGAGED_S)
            {
                state = DISENGAGED_TO_OFF_T; 
            }
            else
            {
                ret = false;
            }
            break;
        case GOTO_DISENGAGED_E:
            if (state == DISENGAGED_S)
            {
                ret = true;
            }
            else if (state == OFF_S)
            {
                state = OFF_TO_DISENGAGED_T;
            }
            else if (state ==  ENGAGED_S)
            {
                setLogicModeCmd(GOTO_PAUSE_E); //set mode to pause
                state = ENGAGED_TO_DISENGAGED_T;
            }
            else
            {
                ret = false;
            }
            break;
        case GOTO_ENGAGED_E:
            if (state == ENGAGED_S)
            {
                ret = true;
            }
            else if (state == DISENGAGED_S)
            {
                state = DISENGAGED_TO_ENGAGED_T;
            }
            else 
            {
                ret = false;
            }
            break;
        case GOTO_REFERENCING_E:
            if (state == OFF_S)
            {
                state = OFF_TO_CALIBRATE_T;
            }
            else
            {
                ret = false;
            }
            break;
        /*case GOTO_CALIBRATE:*/
            /*break;*/
        case EMERGENCY_STOP_E:
            setLogicModeCmd(GOTO_PAUSE_E); //set mode to pause
            state = TO_ESTOP_T;
            ret = true;
            break;
        case ACKNOWLEDGE_ERROR:
            if (state == OFF_S)
            {
                ret = true;
            }
            else if (state == ESTOP_S)
            {
                state = RESET_ESTOP_T;
            }
            else 
            {
                ret = false;
            }
            break;
        default:
            break;
    }

    /*if (ret)*/
    //{
        //this->prev_state_ = this->state_;
    /*}*/

    FST_INFO("cur state:%d",state);
    
    setLogicState(state);

    return ret;
}

U64 RobotMotion::actionResume()
{
    U64 result = FST_SUCCESS;
   // boost::recursive_mutex::scoped_lock lock(recu_mutex_);
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
    U64 result = FST_SUCCESS;
//	boost::recursive_mutex::scoped_lock lock(recu_mutex_);
    boost::mutex::scoped_lock lock(g_mutex_);
    if (getManualState() == MANUAL_SUSPEND_S)
    {
        return result;
    }
	int traj_len = arm_group_->getPlannedPathFIFOLength();
	int joints_len = arm_group_->getJointTrajectoryFIFOLength();
	if ((traj_len != 0) || (joints_len != 0))
	{
		if ((traj_len != 0) && (joints_len == 0))
		{
			FST_ERROR("traj_len is %d, but joints_len is 0 !", traj_len);
			return WRONG_FIFO_STATE;
		}
        //FST_INFO("traj_len:%d, joint_len:%d",traj_len, joints_len);             
        if (arm_group_->suspendArmMotion(result))
        {
            //FST_ERROR("manual state:%d", getManualState());
            FST_ERROR("auto state:%d", getAutoState());
            if (getManualState() == MANUAL_RUNNING_S)
            {
                setManualState(MANUAL_SUSPEND_S);
                clearPathFifo();
            }
            if (getAutoState() == AUTO_RUNNING_S)
            {
                setAutoState(AUTO_SUSPEND_S);
            }
        }
        else
        {
            FST_ERROR("suspendArmMotion error");
        }
	}

	return result;
}

/**
 * @brief: add one manual instruction to the queue
 *
 * @param cmd_instruction: input==>the instruction to add
 * @param arguments: input==> the arguments of instruction
 */
void RobotMotion::addManualQueue(motion_spec_MOTIONTYPE command_type, string arguments)
{
	//boost::mutex::scoped_lock lock(mutex_);
	CommandInstruction cmd_instruction;
	cmd_instruction.id = 0;
	cmd_instruction.smoothDistance = MAX_CNT_VAL;
	cmd_instruction.commandtype = command_type;
	cmd_instruction.command_arguments = arguments;

	manual_instruction_queue_.push(cmd_instruction);
}


/**
 * @brief: add one command in to queue
 *
 * @param motion_command: input==>the command to push
 */
void RobotMotion::addMotionQueue(CommandInstruction cmd_instruction)
{
    motion_queue_.push(cmd_instruction);
}

/**
 * @brief: pick instruction from instruction queue and execute the instruction
 *
 * @return: 0 if success
 */
U64 RobotMotion::queueProcess()
{
	U64 result = FST_SUCCESS;
	motion_spec_MoveL *moveL, *moveL_next;
	PoseEuler *cur_pose, *next_pose = NULL;
	PoseEuler pose_cur, pose_next;
	JointValues *cur_joints, *next_joints;	
	static int manual_count = 0;	   

    boost::mutex::scoped_lock lock(g_mutex_);

    RobotMode mode = getLogicMode();
	if ((mode == INIT_M) || (mode == AUTO_RESET_M))
	{
		//FST_ERROR("invalid mode %d", mode);
		return FST_SUCCESS;
	}
	static int total_cnt = 0;    

	if (share_mem_.isJointCommandWritten() == false)
	{
		result = share_mem_.setJointPositions(share_mem_.getCurrentJointCmd().joint_cmd);
		if (result != FST_SUCCESS)
		{
			//FST_ERROR("set share_mem_ positions failed");
            return result;
		}
	}//end if (share_mem_.isJointCommandWritten() == false)
    
	int traj_len = arm_group_->getPlannedPathFIFOLength();
	int joints_len = arm_group_->getJointTrajectoryFIFOLength();
//	FST_INFO("traj_len:%d,joints_len:%d",traj_len, joints_len);
	/*if (joints_len == 0) //last instruction*/
	//{
		//if (current_command_id_ >= 0)
		//{
			////popupInstruction();
            //current_command_id_ = -1; //stopped
		//}		
	//} 
	/*else */if (joints_len > 0)
	{
	//	FST_INFO("joints fifo length:%d", joints_len);
		int joints_in = (joints_len < NUM_OF_POINTS_TO_SHARE_MEM)?joints_len:NUM_OF_POINTS_TO_SHARE_MEM;

        vector<JointPoint> joint_traj;
        joints_in = arm_group_->getPointsFromJointTrajectoryFIFO(joint_traj, joints_in, result);
        if(result != FST_SUCCESS)
        {
            //this error come up when joints_in is not properly set
            FST_ERROR("getJointTrajectoryFIFOLength failed:%llx", result);
        }
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
				joint_command.points[i].point_position = joint_traj[i].id & 0x03;  //last two bits as point position

			//	FST_INFO("%f,%f,%f,%f,%f,%f",joint_traj_[i].joints.j1, joint_traj_[i].joints.j2,\
			joint_traj_[i].joints.j3, joint_traj_[i].joints.j4, joint_traj_[i].joints.j5,\
			joint_traj_[i].joints.j6);

		
				if (joint_command.points[i].point_position == END_POINT)
				{
                    if (0 == arm_group_->getJointTrajectoryFIFOLength())
                    {
                        if (current_command_id_ >= 0)
                        {
                            popupInstruction();
                            current_command_id_ = -1; //stopped
                        }	
                    }
					FST_PRINT("the last joints:");
					for (int m = 0; m < MAX_JOINTS; m++)
					{
						FST_PRINT("%f ", joint_command.points[i].positions[m]);
					}
					FST_PRINT("\n");
				}
                int traj_id = joint_traj[i].id >> 2;
                
                if (current_command_id_ == -1)
                {                    
                    current_command_id_ = traj_id;
                    //FST_ERROR("prev_id:%d,traj_id:%d", previous_command_id_, traj_id);
                }
                else if (current_command_id_ != traj_id) //id changed
                {  
                    popupInstruction();
                    current_command_id_ = traj_id;
                    
                }      
                                
			}// end for (int i = 0; i < joints_in; i++)
		//	FST_INFO("=====>traj_id:%x,position:%d--%d,id:%d",joint_traj_[0].id, joint_command.points[0].point_position, joint_command.points[joints_in-1].point_position,joint_traj_[0].id >> 2);
			share_mem_.setCurrentJointCmd(joint_command); //store this command in case it can't write Success
			result = share_mem_.setJointPositions(joint_command);
            
			if (result != FST_SUCCESS)
            {
                //do nothing if unsuccessful
                //FST_ERROR("set share_mem_ positions failed");
            }
		}//end if (arm_group_->getPointsFromJointTrajectoryFIFO(joint_traj_, joints_in, result) >= 0)
		//FST_INFO("cur id:%d, prev id:%d",current_command_id_,previous_command_id_);

	}//end else if (joints_len > 0)
    
	
	if (mode == PAUSE_M)
	{
		return FST_SUCCESS;
	}
    else if ((mode == MANUAL_JOINT_MODE_M) || (mode == MANUAL_CART_MODE_M))
    {
        if (manual_state_ == MANUAL_SUSPEND_S) 
        {
            //clear instruction queue
            while (!manual_instruction_queue_.empty())
            {
                CommandInstruction cmd_instruction;
                manual_instruction_queue_.waitAndPop(cmd_instruction);
            } 
            if (getServoState() == STATE_READY)
            {
                manual_state_ = MANUAL_IDLE_S;
            }
            return FST_SUCCESS;
        }
    }
    else if (mode == AUTO_RUN_M)
    {
        if (getServoState() == STATE_READY)
        {
            auto_state_ = AUTO_READY_S;
        }
        if (auto_state_ == AUTO_SUSPEND_S)
        {
            return FST_SUCCESS;
        }
    }

	if (traj_len > 0) //there are points left for us to pick
	{
		if (joints_len < MAX_CNT_VAL)
		{
			int joints_spare = MAX_CNT_VAL -joints_len;
		//	FST_INFO("points fifo length:%d,joints_spare:%d", traj_len, joints_spare);
			int points_num;
			if (joints_spare > traj_len)
				points_num = (traj_len < MAX_PLANNED_POINTS_NUM)?traj_len:MAX_PLANNED_POINTS_NUM;
			else
				points_num = (joints_spare < MAX_PLANNED_POINTS_NUM)?joints_spare:MAX_PLANNED_POINTS_NUM;
		//	FST_INFO("points_num:%d",points_num);
			arm_group_->convertPathToTrajectory(points_num, result);
			if (result != FST_SUCCESS)
			{
				FST_ERROR("convertPathToTrajectory failed:%llx", result);   
                setLogicStateCmd(EMERGENCY_STOP_E);
                //=========================================================================
                //in this situation we need to go into another mode to wait for reset error
                //=========================================================================
			    return result;
			}
		}// end if (joints_len < MAX_CNT_VAL)
	}//end if (traj_len > 0)    
    if ((servo_ready_wait_ == false)  //do not need to wait for servo ready
    || ((servo_ready_wait_ == true)&&(getServoState() == STATE_READY))) //wait until servo ready
    {
        if (traj_len < TRAJ_LIMIT_NUM) //judge when to pick next instruction
        {
            if (mode == AUTO_RUN_M)
            {
                if (pickMotionInstruction())
                {
                    FST_INFO("pick one instruction...");
                    if (auto_state_ == AUTO_READY_S)
                    {
                        auto_state_ = AUTO_RUNNING_S;
                    }

                    return autoMotion();
                }
            }
            else if ((mode == MANUAL_JOINT_MODE_M) || (mode == MANUAL_CART_MODE_M))
            {
                if (manual_instruction_queue_.empty() //manual instruction queue is empty
                && (next_move_instruction_.id == -1)
                && (getServoState() == STATE_READY))  
                {
                    manual_state_ = MANUAL_IDLE_S;
                }
                if (manual_state_ == MANUAL_IDLE_S)
                {
                    if ((!manual_instruction_queue_.empty()) || (next_move_instruction_.id != -1)) //manual instruction queue is not empty
                    {
                        manual_state_ = MANUAL_READY_S;
                        FST_INFO("state:ready");
                    }
                }
                if (manual_state_ == MANUAL_READY_S)
                {
                    manual_count++;
                //	FST_INFO("manual_count:%d",manual_count);
                    if (manual_count >= manual_inst_delay_cnt_) //this value can be fixed
                    {
                        manual_count = 0;
                        manual_state_ = MANUAL_RUNNING_S;
                        FST_INFO("state:running");
                    }
                }
                if (manual_state_ == MANUAL_RUNNING_S)
                {
                    if (pickManualInstruction())
                    {
                        FST_INFO("pick one manual instruction...");
                        switch (cur_instruction_.commandtype)
                        {
                        case motion_spec_MOTIONTYPE_JOINTMOTION:
                            cur_joints = (JointValues*)cur_instruction_.command_arguments.c_str();
                            if (next_move_instruction_.id == -1)
                                next_joints = NULL;
                            else
                                next_joints = (JointValues*)next_move_instruction_.command_arguments.c_str();
                            result = moveJoints(cur_joints, next_joints);
                            break;
                        case motion_spec_MOTIONTYPE_CARTMOTION:
                            pose_cur = *(PoseEuler*)cur_instruction_.command_arguments.c_str();
                            uintPoseTP2Ctle(&pose_cur);
                            cur_pose = &pose_cur;
                            if (next_move_instruction_.id == -1)
                                next_pose = NULL;
                            else
                            {
                                pose_next = *(PoseEuler*)next_move_instruction_.command_arguments.c_str();
                                uintPoseTP2Ctle(&pose_next);
                                next_pose = &pose_next;
                            }
                            result = moveLine(cur_pose, next_pose);
                            break;
                        case motion_spec_MOTIONTYPE_SET:
                            break;
                        case motion_spec_MOTIONTYPE_WAIT:
                            break;
                        default:
                            break;
                        }
                    }//end if (pickManualInstruction())
                }//end if (manual_state_ == MANUAL_RUNNING_S)
            }// end else if ((mode == MANUAL_JOINT_MODE_M) || (mode == MANUAL_CART_MODE_M))
        }// end if(traj_len < TRAJ_LIMIT_NUM) 
    }//end if (servo_ready_wait_)

    return result;
}

/**
 * @brief :reset command queue
 *
 * @return: 0 is success 
 */
U64 RobotMotion::clearMotionQueue()
{    
    U64 result = clearPathFifo();

	resetQueue();

	resetInstructionID();

    //reset share_mem_
    share_mem_.setWritenFlag(true);

    return result;
}

/**
 * @brief check the robot joint state judge if they are running
 * @param start_state_cnt: input==>the count of set start state
 *
 * @return: 0 is success
 */
U64 RobotMotion::checkAutoStartState()
{
	U64 result = FST_SUCCESS;   

    if ((getServoState() == STATE_READY)  //ready
    && isFifoEmpty()                    //fifo is empty
    && (motion_queue_.empty())          //instruction queue empty
    && (picked_motion_queue_.empty()))
    {
        boost::mutex::scoped_lock lock(mutex_);
	    arm_group_->setStartState(joints_, result);
    }

    return result;
}

U64 RobotMotion::checkManualStartState()
{
	U64 result = FST_SUCCESS;   

    if ((getServoState() == STATE_READY)          
    && isFifoEmpty() 
    && (manual_instruction_queue_.empty()))
    {
        boost::mutex::scoped_lock lock(mutex_);
	    arm_group_->setStartState(joints_, result);
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
    if (result != FST_SUCCESS)
    {
        err_list[err_size] = result;
        err_size += 1;        
    }

    return err_size;
}

void RobotMotion::backupErrorList(CommandInstruction cmd_instruction)
{
    bak_err_queue_.push(cmd_instruction);
}

void RobotMotion::clearErrorList()
{
    CommandInstruction cmd_instruction;
    while (!bak_err_queue_.empty())
    {
        bak_err_queue_.waitAndPop(cmd_instruction);
    }
}

bool RobotMotion::getErrorState()
{
    boost::mutex::scoped_lock lock(mutex_);
    return err_flag_;
}

void RobotMotion::setErrorState(bool flag)
{
    boost::mutex::scoped_lock lock(mutex_);
    err_flag_ = flag;
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
	dst_pose.position.x = pose.coordinates[0]*1000;
	dst_pose.position.y = pose.coordinates[1]*1000;
	dst_pose.position.z = pose.coordinates[2]*1000;
	dst_pose.orientation.a = pose.coordinates[3];
	dst_pose.orientation.b = pose.coordinates[4];
	dst_pose.orientation.c = pose.coordinates[5];

	dst_movel_param.vel_max = src_moveL->vMax*1000;
	dst_movel_param.acc_max = src_moveL->aMax*1000;

	if (way_point.has_blendInDistance)
		dst_movel_param.smooth = way_point.blendInDistance;
	else
		dst_movel_param.smooth = -1;
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

	dst_movej_param.vel_max = src_moveJ->vMax*1000;
	dst_movej_param.acc_max = src_moveJ->aMax*1000;
#if 0
	if (src_moveJ->has_smoothDistance)
		dst_movej_param.smooth = src_moveJ->smoothDistance;
	else
		dst_movej_param.smooth = -1;
#endif

}

/**
 * @brief get max value from a buffer
 *
 * @param buffer: input
 * @param length: input
 *
 * @return :the max value
 */
double RobotMotion::getMaxValue(const double *buffer, int length)
{
	double cmp1, cmp2;
	cmp1 = buffer[0];
	cmp2 = buffer[1];

	double max = MAX(cmp1, cmp2);

	for (int i = 2; i < length; i++)
	{
		cmp1 = max;
		cmp2 = buffer[i];

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
		if (max == buffer[i])
		{
			id = i;
			break;
		}
	}

	return id;
}

/**
 * @brief: get the max interval value between two joints
 *
 * @param pre_joints: input
 * @param cur_joints: input
 *
 * @return: the max value
 */
double RobotMotion::getMaxJointInterval(JointValues pre_joints, JointValues cur_joints)
{
return 0;
	
}

/**
 * @brief: calculate max velocity between two joints
 *
 * @param pre_joints: input
 * @param cur_joints: input
 *
 * @return: max velocity
 */
double RobotMotion::calcuManualJointVel(JointValues pre_joints, JointValues cur_joints)
{
	double interval[6];
	interval[0] = fabs(pre_joints.j1 - cur_joints.j1);
	interval[1] = fabs(pre_joints.j2 - cur_joints.j2);
	interval[2] = fabs(pre_joints.j3 - cur_joints.j3);
	interval[3] = fabs(pre_joints.j4 - cur_joints.j4);
	interval[4] = fabs(pre_joints.j5 - cur_joints.j5);
	interval[5] = fabs(pre_joints.j6 - cur_joints.j6);

	double max_interval = getMaxValue(interval, 6); 

	int id = getMaxValID(max_interval, interval, 6);

    boost::mutex::scoped_lock lock(mutex_);

	JointConstraints jnt_constraints = arm_group_->getJointConstraints();
	JointVelocity start_jnt_vel = arm_group_->getStartJoint().omegas;
	double constraint_vel;
	double constraint_acc;
	double start_vel;	 
	switch (id)
	{
		case 0:
			constraint_vel = jnt_constraints.j1.max_omega;
			constraint_acc = jnt_constraints.j1.max_alpha;
			start_vel = start_jnt_vel.j1;
			break;
		case 1:
			constraint_vel = jnt_constraints.j2.max_omega;
			constraint_acc = jnt_constraints.j2.max_alpha;
			start_vel = start_jnt_vel.j2;
			break;
		case 2:
			constraint_vel = jnt_constraints.j3.max_omega;
			constraint_acc = jnt_constraints.j3.max_alpha;
			start_vel = start_jnt_vel.j3;
			break;
		case 3:
			constraint_vel = jnt_constraints.j4.max_omega;
			constraint_acc = jnt_constraints.j4.max_alpha;
			start_vel = start_jnt_vel.j4;
			break;
		case 4:
			constraint_vel = jnt_constraints.j5.max_omega;
			constraint_acc = jnt_constraints.j5.max_alpha;
			start_vel = start_jnt_vel.j5;
			break;
		case 5:
			constraint_vel = jnt_constraints.j6.max_omega;
			constraint_acc = jnt_constraints.j6.max_alpha;
			start_vel = start_jnt_vel.j6;
			break;
	}//end switch(id)

	double vRef = sqrt(2*constraint_acc*max_interval + start_vel*start_vel);  //get the reference velocity

	int time_interval = MANUAL_COUNT_PER_STEP; //need to fix

	double vmax = max_interval / time_interval * 1000; // rad/s
	FST_INFO("ref:%f, vmax:%f, constraint_vel:%f\n",vRef, vmax, constraint_vel);

	vmax = (vmax<constraint_vel) ? vmax : constraint_vel;
	vmax = (vmax<vRef) ? vmax : vRef;

	return (vmax / constraint_vel * MAX_MANUAL_SPEED);
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
 * @brief: calculate max velocity between two pose
 *
 * @param pre_pose: input
 * @param cur_pose: input
 *
 * @return: max velocity
 */
double RobotMotion::calcuManualLineVel(PoseEuler pre_pose, PoseEuler cur_pose)
{
    
    double vmax;
    int time_interval = MANUAL_COUNT_PER_STEP; //need to fix;
	double line_interval = getMaxLineInterval(pre_pose, cur_pose);
    double rot_interval = getMaxRotateInterval(pre_pose, cur_pose);

    FST_INFO("line_interval:%f,rot_interval:%f",line_interval, rot_interval);

    if ((line_interval * 10) > (rot_interval * MAX_LINE_SPEED)) //need to calcu according to rotation
    {
	    vmax = line_interval / time_interval * 1000; // m/ms --> mm/s
    }
    else
    {
        vmax = (rot_interval * MAX_LINE_SPEED) / (time_interval * 10) * 1000; // c/t* (v_ref/omega_ref)
        FST_INFO("rot_interval:%f, vmax:%f", rot_interval, vmax);
    }
	
	return vmax;
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


/**
 * @brief: motion process in auto mode
 */
U64 RobotMotion::autoMotion()
{
	U64 result = FST_SUCCESS;
    servo_ready_wait_ = false;

	FST_INFO("===>cur id:%d", cur_instruction_.id);
	switch (cur_instruction_.commandtype)
	{
		case motion_spec_MOTIONTYPE_JOINTMOTION:
        {
			motion_spec_MoveJ* moveJ =  (motion_spec_MoveJ*)cur_instruction_.command_arguments.c_str();
#if 0
			moveJ->has_smoothDistance = cur_instruction_.has_smooth; //first rewrite this value
#endif
            JointValues cur_jnts;
            MoveJParam cur_movej_param;
			unitConvert(moveJ, cur_jnts, cur_movej_param);
			FST_INFO("target joints:%f,%f,%f,%f,%f,%f, smmoth:%f", cur_jnts.j1, cur_jnts.j2, cur_jnts.j3, cur_jnts.j4, cur_jnts.j5, cur_jnts.j6, cur_instruction_.smoothDistance);
			if (cur_instruction_.smoothDistance > 0)
			{
				if (next_move_instruction_.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION) //joint-->joint
				{	
					motion_spec_MoveJ* moveJ_next =  (motion_spec_MoveJ*)next_move_instruction_.command_arguments.c_str();
                    JointValues next_jnts;
                    MoveJParam next_movej_param;
					unitConvert(moveJ_next, next_jnts, next_movej_param);
                    boost::mutex::scoped_lock lock(mutex_);
					arm_group_->MoveJ(cur_jnts, cur_movej_param.vel_max, cur_movej_param.acc_max, \
							cur_movej_param.smooth, next_jnts, next_movej_param.vel_max, next_movej_param.acc_max,\
							next_movej_param.smooth, cur_instruction_.id, result);
                }
				else if (next_move_instruction_.commandtype == motion_spec_MOTIONTYPE_CARTMOTION) //joint-->line
				{
					motion_spec_MoveL* moveL_next =  (motion_spec_MoveL*)next_move_instruction_.command_arguments.c_str();
                    PoseEuler next_pose;
                    MoveLParam next_movel_param;
					unitConvert(moveL_next, next_pose, next_movel_param);
                    boost::mutex::scoped_lock lock(mutex_);
					arm_group_->MoveJ(cur_jnts, cur_movej_param.vel_max, cur_movej_param.acc_max, \
							cur_movej_param.smooth, next_pose, next_movel_param.vel_max, next_movel_param.acc_max,\
							next_movel_param.smooth, cur_instruction_.id, result);
				}
			}//end if (cur_instruction_.has_smooth == trueo+)
			else
			{
                FST_INFO("1ret:");
                if (cur_instruction_.smoothDistance < 0)
                {
                    servo_ready_wait_ = true;
                }
                boost::mutex::scoped_lock lock(mutex_);
				bool ret = arm_group_->MoveJ(cur_jnts, cur_movej_param.vel_max, \
							cur_movej_param.acc_max, cur_instruction_.id, result);
                FST_INFO("ret:%d, result:%x", ret, result);

			}//end else
			break;
        }
		case motion_spec_MOTIONTYPE_CARTMOTION:
        {
			motion_spec_MoveL* moveL = (motion_spec_MoveL*)cur_instruction_.command_arguments.c_str();
			moveL->waypoints[0].blendInDistance = cur_instruction_.smoothDistance; //first rewrite this value
            PoseEuler cur_pose;
            MoveLParam cur_movel_param;
			unitConvert(moveL, cur_pose, cur_movel_param);
			FST_INFO("current pose:%f,%f,%f,%f,%f,%f,smooth:%f", cur_pose.position.x,cur_pose.position.y,cur_pose.position.z, cur_pose.orientation.a, cur_pose.orientation.b, cur_pose.orientation.c, cur_instruction_.smoothDistance);
			if (cur_instruction_.smoothDistance > 0)
			{
				FST_INFO("smooth...");
				if (next_move_instruction_.commandtype == motion_spec_MOTIONTYPE_CARTMOTION) //line-->line
				{
					motion_spec_MoveL* moveL_next =  (motion_spec_MoveL*)next_move_instruction_.command_arguments.c_str();
                    PoseEuler next_pose;
                    MoveLParam next_movel_param;
					unitConvert(moveL_next, next_pose, next_movel_param);
                    boost::mutex::scoped_lock lock(mutex_);
					arm_group_->MoveL(cur_pose, cur_movel_param.vel_max, cur_movel_param.acc_max, \
							cur_movel_param.smooth, next_pose, next_movel_param.vel_max, next_movel_param.acc_max,\
							next_movel_param.smooth, cur_instruction_.id, result);
				}
				else if (next_move_instruction_.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION) //line-->joint
				{
					motion_spec_MoveJ* moveJ_next =  (motion_spec_MoveJ*)next_move_instruction_.command_arguments.c_str();
                    JointValues next_jnts;
                    MoveJParam next_movej_param;
					unitConvert(moveJ_next, next_jnts, next_movej_param);
                    boost::mutex::scoped_lock lock(mutex_);
					arm_group_->MoveL(cur_pose, cur_movel_param.vel_max, cur_movel_param.acc_max, \
							cur_movel_param.smooth, next_jnts, next_movej_param.vel_max, next_movej_param.acc_max,\
							next_movej_param.smooth, cur_instruction_.id, result);
				}
			}//end if (cur_instruction_.has_smooth == true)
			else
			{
				FST_INFO("do not smmoth");
                if (cur_instruction_.smoothDistance < 0)
                {
                    servo_ready_wait_ = true;
                }
                boost::mutex::scoped_lock lock(mutex_);
				arm_group_->MoveL(cur_pose, cur_movel_param.vel_max, \
							cur_movel_param.acc_max, cur_instruction_.id, result);

			}
			break;
        }
		case motion_spec_MOTIONTYPE_SET:
			break;
		case motion_spec_MOTIONTYPE_WAIT:
			break;
		default:
			break;

	}

    if (result != FST_SUCCESS)
    {
        FST_ERROR("auto motion failed:%llx",result);
    }

    return result;
}

/**
 * @brief: move a line used for manual mode
 *
 * @param cur_pose: input==>current pose
 * @param next_pose: input==>next pose
 *
 * @return: 0 is success
 */
U64 RobotMotion::moveLine(const PoseEuler* cur_pose, const PoseEuler* next_pose)
{
	U64 result;
    servo_ready_wait_ = false;
	
	boost::mutex::scoped_lock lock(mutex_);

    PoseEuler start_pose =  arm_group_->transformPose2PoseEuler(arm_group_->getStartPose());
    lock.unlock();
	double vMax = calcuManualLineVel(start_pose, *cur_pose);
	double aMax = MANUAL_DEFAULT_ACC;

   // FST_INFO("manual start pose:%f,%f,%f,%f,%f,%f", s_pose.position.x,s_pose.position.y,s_pose.position.z, s_pose.orientation.a, s_pose.orientation.b, s_pose.orientation.c);
	FST_INFO("manual target pose:%f,%f,%f,%f,%f,%f", cur_pose->position.x,cur_pose->position.y,cur_pose->position.z, cur_pose->orientation.a, cur_pose->orientation.b, cur_pose->orientation.c);
	FST_INFO("Manual line==>vMax:%f,aMax:%f", vMax, aMax);
    if(vMax > 2000)
        FST_ERROR("the velocity is too big");

	if (next_pose == NULL) // don't use smooth
	{
		FST_INFO("do not smooth");
        lock.lock();
        bool ret = arm_group_->MoveL(*cur_pose, vMax, aMax, 0, result);
        lock.unlock();
		if (ret == false)
		{
            FST_ERROR("manual moveL failed:%llx",result);
		}
	}
	else
	{
		FST_INFO("smooth ...");
		FST_INFO("manual next target pose:%f,%f,%f,%f,%f,%f", next_pose->position.x,next_pose->position.y,next_pose->position.z, next_pose->orientation.a, next_pose->orientation.b, next_pose->orientation.c);
		int cnt = MAX_CNT_VAL;
		int cnt_next = MAX_CNT_VAL;
        
        lock.lock();
        bool ret = arm_group_->MoveL(*cur_pose, vMax, aMax, MAX_CNT_VAL,\
				*next_pose, vMax, aMax, MAX_CNT_VAL, 0, result);
        lock.unlock();
		if (ret == false)
		{
            FST_ERROR("manual moveL failed:%llx",result);
		}

	}

    return result;
}


/**
 * @brief: move joints in manual mode
 *
 * @param cur_joints: input
 * @param next_joints: input
 *
 * @return: 0 is success
 */
U64 RobotMotion::moveJoints(const JointValues *cur_joints, const JointValues *next_joints)
{
	U64 result;
    servo_ready_wait_ = false;

    boost::mutex::scoped_lock lock(mutex_);
	JointValues start_joints = arm_group_->getStartJoint().joints;  //get the start state
    lock.unlock();

	double vMax = calcuManualJointVel(start_joints, *cur_joints);
	double aMax = MANUAL_DEFAULT_ACC;

	FST_INFO("manual target joint:%f,%f,%f,%f,%f,%f", cur_joints->j1,cur_joints->j2,cur_joints->j3,cur_joints->j4,cur_joints->j5,cur_joints->j6);
	FST_INFO("Manual joints===>vMax:%f,aMax:%f", vMax, aMax);

	if (next_joints == NULL) // don't use smooth
	{
		FST_INFO("do not smooth");		
        lock.lock();
        bool ret = arm_group_->MoveJ(*cur_joints, vMax, aMax, 0, result);
        lock.unlock();
		if (ret == false)
		{
            FST_ERROR("manual MoveJ failed:%llx",result);
		}
	}
	else
	{
		FST_INFO("smooth ...");
		int cnt = MAX_CNT_VAL;
		int cnt_next = MAX_CNT_VAL;

		FST_INFO("manual next target joint:%f,%f,%f,%f,%f,%f", next_joints->j1,next_joints->j2,next_joints->j3,next_joints->j4,next_joints->j5,next_joints->j6);

        lock.lock();
        bool ret = arm_group_->MoveJ(*cur_joints, vMax, aMax, MAX_CNT_VAL,\
				*next_joints, vMax, aMax, MAX_CNT_VAL, 0, result);
        lock.unlock();
		if (ret == false)
		{
            FST_ERROR("manual moveJ failed:%llx",result);
		}
	}
    
    return result;
}


/**
 * @brief: pick one manual move instruction
 *
 * @return: 0 if success
 */
bool RobotMotion::pickManualInstruction()
{
    boost::mutex::scoped_lock lock(mutex_);
//	FST_INFO("next move id is:%d", next_move_instruction_.id);
	if (next_move_instruction_.id != -1)  //need to fix 0928
	{		
		cur_instruction_ = next_move_instruction_;
	}
	else if (!manual_instruction_queue_.empty())
		manual_instruction_queue_.waitAndPop(cur_instruction_);
	else
	{
		next_move_instruction_.id = -1;
		return false;
	}

	if (!manual_instruction_queue_.empty())
	{
		FST_INFO("find next manual instruction");
		manual_instruction_queue_.waitAndPop(next_move_instruction_);
	}
	else
	{
		FST_INFO("can't find next manual instruction");
		cur_instruction_.smoothDistance = MAX_CNT_VAL;
		next_move_instruction_.id = -1;
	}
    

	return true;
}



/**
 * @brief: pick one motion instruction from the queue and execute it
 *
 * @return: 0 if Success
 */
bool RobotMotion::pickMotionInstruction()
{
	bool ret;
	U64 result;

    boost::mutex::scoped_lock lock(mutex_);
//	CommandInstruction cmd_instruction;
	//before move, do other actions like set DI and DO etc...
	if (!non_move_queue_.empty())
	{
		//FST_INFO("non move queue not empty");
		//execute non move instruction
		//ret = true;
	}
	else
	{
		if ((next_move_id_ == next_move_instruction_.id) && (next_move_id_ >= 0)) //the instruction has been already decoded
		{
			cur_instruction_ = next_move_instruction_;
			FST_INFO("cur instruction id:%d", cur_instruction_.id);
		}
		else if (!motion_queue_.empty()) //start to move
		{
			FST_INFO("motion queue not empty");
			//make sure current instruction
			motion_queue_.waitAndPop(cur_instruction_);
            picked_motion_queue_.push(cur_instruction_);
   /*         if ((motion_command.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)*/
			//|| (motion_command.commandtype == motion_spec_MOTIONTYPE_CARTMOTION))
			//{
				//result = parseMotionCommand(motion_command, cur_instruction_);
			/*}*/
		}
		else
		{
			return false;//no motion instruction left
		}

		if (cur_instruction_.smoothDistance > 0)
		{
			FST_INFO("findNextMoveCommand");
		//	motion_spec_MotionCommand next_motion_command;
			if (findNextMoveCommand(next_move_instruction_)) //find a next move instruction
			{
			//	result = parseMotionCommand(next_motion_command, next_move_instruction_); //move smooth
				next_move_id_ = next_move_instruction_.id;
				//then we can moveJ or moveL
			}
			else // can't find next move instruction
			{
				FST_INFO("can't find next move ");
				cur_instruction_.smoothDistance = -1;
				next_move_id_ = -1;
			}
		}//end if (cur_instruction_.has_smooth == true)
		else
		{
			next_move_id_ = -1;
		}
	}//end else

    /*FST_ERROR("curID:%d,PreID:%d", current_command_id_, previous_command_id_);*/
	//if ((previous_command_id_ == -1) && (current_command_id_ == -1))//the first command instruction
	//{
		//current_command_id_ = cur_instruction_.id;
        //FST_ERROR("1111curID:%d,PreID:%d", current_command_id_, previous_command_id_);
	/*}*/

		return true;
}

/**
 * @brief: find next movable command joint or cart
 *
 * @param next_motion_command: output==>the result of the found
 *
 * @return: true if found one
 */
bool RobotMotion::findNextMoveCommand(CommandInstruction &next_cmd_instruction)
{
    CommandInstruction cmd_instruction;
	while (!motion_queue_.empty())
	{
		motion_queue_.waitAndPop(cmd_instruction);
        picked_motion_queue_.push(cmd_instruction);
		if ((cmd_instruction.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
		|| (cmd_instruction.commandtype == motion_spec_MOTIONTYPE_CARTMOTION))
		{
			next_cmd_instruction = cmd_instruction;  //next move command	found
			return true;
		}
		else
		{
			non_move_queue_.push(cmd_instruction);
		}

	}//end while (!motion_queue_.empty())

	return false;
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

void RobotMotion::resetQueue()
{
    CommandInstruction command_instruction;
    while (!non_move_queue_.empty())
    {
       non_move_queue_.waitAndPop(command_instruction);
       picked_motion_queue_.push(command_instruction);
    }

    while (!motion_queue_.empty())
    {
       motion_queue_.waitAndPop(command_instruction);
       picked_motion_queue_.push(command_instruction);
    }

    while (!picked_motion_queue_.empty())
    {
       picked_motion_queue_.waitAndPop(command_instruction);
       motion_queue_.push(command_instruction);
    }

    while (!manual_instruction_queue_.empty())
	{
		manual_instruction_queue_.waitAndPop(command_instruction);
	}

}

U64 RobotMotion::clearPathFifo()
{
    U64 result = FST_SUCCESS;
    boost::mutex::scoped_lock lock(mutex_);

	int traj_len = arm_group_->getPlannedPathFIFOLength();
	int joints_len = arm_group_->getJointTrajectoryFIFOLength();
//	FST_ERROR("current traj_len:%d, joints_len:%d", traj_len, joints_len);
    
    if ((traj_len != 0) || (joints_len != 0))
    {		
        //maybe wrong when setStartState failed
        arm_group_->clearPlannedPathFIFO(result); 
    }

    return result;
}

void RobotMotion::resetInstructionID()
{
    boost::mutex::scoped_lock lock(mutex_);
    previous_command_id_ = -1;
	current_command_id_ = -1;
	next_move_id_  = -1; //unknown next move id

	cur_instruction_.id = -1;
	next_move_instruction_.id = -1; //used in manual mode	
}

void RobotMotion::setManualState(ManualState state)
{
    boost::mutex::scoped_lock lock(mutex_);
    this->manual_state_ = state;
}
ManualState RobotMotion::getManualState()
{
    boost::mutex::scoped_lock lock(mutex_);
    return this->manual_state_;
}

void RobotMotion::setAutoState(AutoState state)
{
    boost::mutex::scoped_lock lock(mutex_);
    this->auto_state_ = state;
}
AutoState RobotMotion::getAutoState()
{
    boost::mutex::scoped_lock lock(mutex_);
    return this->auto_state_;
}

void RobotMotion::emergencyStop()
{
    U64 result;
    //clear all the fifos
    arm_group_->resetArmGroup(getCurJointsValue(), result);
    share_mem_.setWritenFlag(true); //don't write share_mem any more
    //stop bare metal, can't write fifo since
    share_mem_.stopBareMetal();  
}


void RobotMotion::popupInstruction()
{
    boost::mutex::scoped_lock lock(mutex_);

    CommandInstruction cmd_instruction;
    while(!picked_motion_queue_.empty())
    {                        
        picked_motion_queue_.waitAndPop(cmd_instruction);
        //FST_ERROR("picked_motion_queue_ id:%d", cmd_instruction.id);
        if(cmd_instruction.id == current_command_id_)
        {
            break;
        }
    }

    if(cmd_instruction.id != current_command_id_)
    {
        FST_ERROR("picked_motion_queue_ id:%d is not the same as current_command_id:%d",cmd_instruction.id, current_command_id_);
    }

    //id_changed_flag_ = false;
    previous_command_id_ = current_command_id_;

}

