/**
 * @file robot_motion.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-21
 */
#include "robot_motion.h"
#include "stdio.h"
#include <pb_decode.h>
#include <boost/thread/mutex.hpp>

 boost::mutex io_mutex;

bool Robot_Motion::Get_Logic_Mode(FST_ROBOT_MODE &mode)
{
	mode = this->mode;
}
bool Robot_Motion::Get_Logic_State(FST_ROBOT_STATE &state)
{
	state =  this->state;
}
JointValues Robot_Motion::Get_Cur_Joints_Value()
{
	FeedbackJointState fbjs;
//	int count = 0;
	while(share_mem.Get_Feedback_Joint_State(fbjs) == false){
	//	count++;
		usleep(1);
	}
//	printf("===count is%d\n",count);
	joint_values.j1 = fbjs.position[0];
	joint_values.j2 = fbjs.position[1];
	joint_values.j3 = fbjs.position[2];
	joint_values.j4 = fbjs.position[3];
	joint_values.j5 = fbjs.position[4];
	joint_values.j6 = fbjs.position[5];

	return joint_values;
}

/**
 * @brief set current joint to library and then get the position
 *
 * @param pose:position to return 
 *
 * @return :0 if success 
 */
ErrorCode Robot_Motion::Get_Cur_Position(PoseEuler &pose)
{
	ErrorCode error_code;
	arm_group->setCurrentJointValues(joint_values, error_code);  //set the current joint to lib
	if(error_code != Success)
		return error_code;

	pose = arm_group->transformPose2PoseEuler(arm_group->getCurrentPose()); // get current position from lib
	pose.position.x = pose.position.x/1000;
	pose.position.y = pose.position.y/1000;
	pose.position.z = pose.position.z/1000;
	
	this->pose = pose;

	return error_code;
}


/**
 * @brief set current mode
 *
 * @param mode_cmd mode command 
 *
 * @return true if set Successfully
 */
bool Robot_Motion::Set_Logic_Mode(FST_ROBOT_MODE_CMD mode_cmd)
{
	bool ret;
	if(mode_cmd == GOTO_PAUSE_E)
	{
		if(mode == PAUSE_M)  //from pause to pause
		{
			ret = true;
		}
		else
		{
			ret = arm_group->suspendArmMotion();
			if(ret == true)
			{
				prev_mode = mode;
				mode = PAUSE_M;
			}
		}
	}
	else
	{
		if(mode == PAUSE_M)
		{
			switch(mode_cmd)
			{
				case GOTO_AUTO_RUN_E:
					if(mode != AUTO_RUN_M)
					{
						//printf("restart again...\n");
						Clear_Motion_Queue();
						mode = AUTO_RUN_M;						
					}
					ret = true;
					break;
				case GOTO_MANUAL_JOINT_MODE_E:
					if(mode != MANUAL_JOINT_MODE_M)
					{
						Clear_Motion_Queue();
						mode = MANUAL_JOINT_MODE_M;						
					}
					ret = true;
					break;
				case GOTO_MANUAL_CART_MODE_E:
					if(mode != MANUAL_CART_MODE_M)
					{
						Clear_Motion_Queue();
						mode = MANUAL_CART_MODE_M;
					}
					ret = true;
					break;
				case GOTO_AUTO_RESET_E:
					if(mode != AUTO_RESET_M)
					{
						Clear_Motion_Queue();
						mode = AUTO_RESET_M;
					}
					ret = true;
					break;
				default:
					break;
			}
		}
		else if(mode == MANUAL_JOINT_MODE_M)
		{
			if(mode_cmd == GOTO_MANUAL_JOINT_MODE_E) 
			{
				mode = MANUAL_JOINT_MODE_M;
				ret = true;
			}
			else if((mode_cmd == GOTO_MANUAL_CART_MODE_E))
			{
				Clear_Motion_Queue();
				mode = MANUAL_CART_MODE_M;
				ret = true;
			}
			else 
				ret = false;
		}
		else if(mode == GOTO_MANUAL_CART_MODE_E)
		{
			if(mode_cmd == GOTO_MANUAL_JOINT_MODE_E) 
			{
				Clear_Motion_Queue();
				mode = MANUAL_JOINT_MODE_M;
				ret = true;
			}
			else if((mode_cmd == GOTO_MANUAL_CART_MODE_E))
			{				
				mode = MANUAL_CART_MODE_M;
				ret = true;
			}
			else
				ret = false;
		}
		else 
			ret = false;
	}	
	printf("current mode:%d\n", mode);
	return ret;
}

bool Robot_Motion::Set_Logic_State(FST_ROBOT_STATE_CMD state_cmd)
{
	//this->state = state;
}

void Robot_Motion::Manual_Pause()
{
	//prev_mode = mode;
	//mode = PAUSE_M; 
}

void Robot_Motion::Unit_Convert(motion_spec_MoveL *src_moveL, PoseEuler& dst_pose, MoveL_Param &dst_movel_param)
{
	dst_pose.position.x = src_moveL->targetPose[0]*1000;
	dst_pose.position.y = src_moveL->targetPose[1]*1000;
	dst_pose.position.z = src_moveL->targetPose[2]*1000;
	dst_pose.orientation.a = src_moveL->targetPose[3];
	dst_pose.orientation.b = src_moveL->targetPose[4];
	dst_pose.orientation.c = src_moveL->targetPose[5];

	dst_movel_param.vel_max = src_moveL->vMax*1000;
	dst_movel_param.acc_max = src_moveL->aMax*1000;

	if(src_moveL->has_smoothDistance)
		dst_movel_param.smooth = src_moveL->smoothDistance;
	else
		dst_movel_param.smooth = -1;
}

void Robot_Motion::Unit_Convert(motion_spec_MoveJ *src_moveJ, JointValues &dst_joints, MoveJ_Param &dst_movej_param)
{
	joint_values.j1 = src_moveJ->targetPose[0];
	joint_values.j2 = src_moveJ->targetPose[1];
	joint_values.j3 = src_moveJ->targetPose[2];
	joint_values.j4 = src_moveJ->targetPose[3];
	joint_values.j5 = src_moveJ->targetPose[4];
	joint_values.j6 = src_moveJ->targetPose[5];

	dst_movej_param.vel_max = src_moveJ->vMax*1000;
	dst_movej_param.acc_max = src_moveJ->aMax*1000;

	if(src_moveJ->has_smoothDistance)
		dst_movej_param.smooth = src_moveJ->smoothDistance;
	else
		dst_movej_param.smooth = -1;

}

/**
 * @brief move a line
 *
 * @param current_moveL: current move instruction 
 * @param id:current move id
 * @param next_moveL: next move instruction
 *
 * @return 0 if Success
 */
ErrorCode Robot_Motion::Move_Line(motion_spec_MoveL* current_moveL, int id, motion_spec_MoveL* next_moveL)
{
	bool ret;
	ErrorCode error_code;
	PoseEuler cur_pose, next_pose;
	/*ret = arm_group->setMaxVelocity(current_moveL->vMax);*/
	//if(ret != Success)
		//return ret;
	//ret = arm_group->setMaxAcceleration(current_moveL->aMax);
	//if(ret != Success)
		/*return ret;*/

	cur_pose.position.x = current_moveL->targetPose[0]*1000;
	cur_pose.position.y = current_moveL->targetPose[1]*1000;
	cur_pose.position.z = current_moveL->targetPose[2]*1000;
	cur_pose.orientation.a = current_moveL->targetPose[3];
	cur_pose.orientation.b = current_moveL->targetPose[4];
	cur_pose.orientation.c = current_moveL->targetPose[5];
	printf("current pose:%f,%f,%f,%f,%f,%f\n", cur_pose.position.x,cur_pose.position.y,cur_pose.position.z, cur_pose.orientation.a, cur_pose.orientation.b, cur_pose.orientation.c);

	if(next_moveL == NULL) // don't use smooth
	{	
		ret = arm_group->MoveL(cur_pose, current_moveL->vMax*1000,\
				current_moveL->aMax*1000, id, error_code);
		if(error_code != Success)
			return error_code;
	}
	else
	{
		int cnt = current_moveL->smoothDistance;
		int cnt_next;
		if(current_moveL->has_smoothDistance == true)
			cnt_next = next_moveL->smoothDistance;
		else
			cnt_next = -1;  //no smooth
		//printf("cnt:%d, cnt_next:%d\n", cnt, cnt_next);
		next_pose.position.x = next_moveL->targetPose[0]*1000;
		next_pose.position.y = next_moveL->targetPose[1]*1000;
		next_pose.position.z = next_moveL->targetPose[2]*1000;
		next_pose.orientation.a = next_moveL->targetPose[3];
		next_pose.orientation.b = next_moveL->targetPose[4];
		next_pose.orientation.c = next_moveL->targetPose[5];
		printf("next pose:%f,%f,%f,%f,%f,%f\n", next_pose.position.x,next_pose.position.y,next_pose.position.z, next_pose.orientation.a, next_pose.orientation.b, next_pose.orientation.c);

		ret = arm_group->MoveL(cur_pose, current_moveL->vMax*1000,\
			current_moveL->aMax*1000, cnt, next_pose, next_moveL->vMax*1000, \
			next_moveL->aMax*1000, cnt_next, id, error_code);

		if(error_code != Success)
			return error_code;
	}

	return error_code;

}

ErrorCode Robot_Motion::Auto_Motion()
{
	bool ret;
	ErrorCode error_code;
	motion_spec_MoveL *moveL, *moveL_next;
	PoseEuler cur_pose, next_pose;
	MoveL_Param cur_movel_param, next_movel_param;

	//printf("===>cur id:%d\n", cur_instruction.id);
	switch(cur_instruction.commandtype)
	{
		case motion_spec_MOTIONTYPE_JOINTMOTION:
			break;
		case motion_spec_MOTIONTYPE_CARTMOTION:
			moveL = (motion_spec_MoveL*)cur_instruction.command_arguments.c_str();
			moveL->has_smoothDistance = cur_instruction.has_smooth; //first rewrite this value
			Unit_Convert(moveL, cur_pose, cur_movel_param);
			printf("current pose:%f,%f,%f,%f,%f,%f\n", cur_pose.position.x,cur_pose.position.y,cur_pose.position.z, cur_pose.orientation.a, cur_pose.orientation.b, cur_pose.orientation.c);
			if(cur_instruction.has_smooth == true)
			{
				printf("smooth...\n");
				if(next_move_instruction.commandtype == motion_spec_MOTIONTYPE_CARTMOTION) //line-->line
				{
					moveL_next =  (motion_spec_MoveL*)next_move_instruction.command_arguments.c_str();
					Unit_Convert(moveL_next, next_pose, next_movel_param);
					ret = arm_group->MoveL(cur_pose, cur_movel_param.vel_max, cur_movel_param.acc_max, \
							cur_movel_param.smooth, next_pose, next_movel_param.vel_max, next_movel_param.acc_max,\
							next_movel_param.smooth, cur_instruction.id, error_code);

					//if(ret != Success)
						/*return error_code;*/

				}
				else if(next_move_instruction.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION) //line-->joint
				{
					
				}
			}
			else
			{
				ret = arm_group->MoveL(cur_pose, cur_movel_param.vel_max, \
							cur_movel_param.acc_max, cur_instruction.id, error_code);
			}
		//	if(error_code)
		//		printf("movel failed %d\n", error_code);
			break;
		case motion_spec_MOTIONTYPE_SETOUTPUT:
			break;
		case motion_spec_MOTIONTYPE_WAIT:
			break;
		default:
			break;

	}
	
}

/**
 * @brief move a line used for manual mode
 *
 * @param cur_pose: current pose 
 * @param next_pose: next pose
 *
 * @return 0 if Success
 */
ErrorCode Robot_Motion::Move_Line(PoseEuler* cur_pose, PoseEuler* next_pose)
{
	bool ret;
	ErrorCode error_code;
	double vMax, aMax;
	vMax = arm_group->getMaxVelocity();
	aMax = arm_group->getMaxAcceleration();

//	printf("manual current pose:%f,%f,%f,%f,%f,%f\n", cur_pose->position.x,cur_pose->position.y,cur_pose->position.z, cur_pose->orientation.a, cur_pose->orientation.b, cur_pose->orientation.c);
	printf("Manual==>vMax:%f,aMax:%f\n", vMax, aMax);

	vMax = 500;
	aMax = 16000;

	if(next_pose == NULL) // don't use smooth
	{	
		ret = arm_group->MoveL(*cur_pose, vMax, aMax, 0, error_code);
		if(error_code != Success)
			return error_code;
	}
	else
	{
		int cnt = MAX_CNT_VAL;
		int cnt_next = MAX_CNT_VAL;

		ret = arm_group->MoveL(*cur_pose, vMax, aMax, MAX_CNT_VAL,\
				*next_pose, vMax, aMax, MAX_CNT_VAL, 0, error_code);
		if(error_code != Success)
			return error_code;
	}

	return error_code;

}


ErrorCode Robot_Motion::Move_Joints(JointValues *cur_joints, JointValues *next_joints)
{
	bool ret;
	ErrorCode error_code;
	double vMax, aMax;
	vMax = 500;
	aMax = 16000;

//	printf("Manual==>vMax:%d,aMax:%d\n", vMax, aMax);

	if(next_joints == NULL) // don't use smooth
	{	
		ret = arm_group->MoveJ(*cur_joints, vMax, aMax, 0, error_code);
		if(error_code != Success)
			return error_code;
	}
	else
	{
		int cnt = MAX_CNT_VAL;
		int cnt_next = MAX_CNT_VAL;

		ret = arm_group->MoveJ(*cur_joints, vMax, aMax, MAX_CNT_VAL,\
				*next_joints, vMax, aMax, MAX_CNT_VAL, 0, error_code);
		if(error_code != Success)
			return error_code;
	}

	return error_code;

}

/**
 * @brief :reset command queue
 */
void Robot_Motion::Clear_Motion_Queue()
{
	boost::mutex mu;
	motion_spec_MotionCommand motion_command;
	Command_Instruction command_instruction;
	while(!non_move_queue.empty())
	{		
		motion_queue.wait_and_pop(motion_command);	
	}
	while(!motion_queue.empty())
	{
		non_move_queue.wait_and_pop(motion_command);	
	}
	while(!manual_instruction_queue.empty())
	{
		manual_instruction_queue.wait_and_pop(command_instruction);
	}

	previous_command_id = -1;
	current_command_id = -1;
	next_move_id  = -1; //unknown next move id

	cur_instruction.id = -1;
	next_move_instruction.id = -1; //used in manual mode

	boost::mutex::scoped_lock lock(io_mutex);

	int traj_len = arm_group->getPlannedPathFIFOLength();
	
	int joints_len = arm_group->getJointTrajectoryFIFOLength();	
	//printf("current traj_len:%d, joints_len:%d\n", traj_len, joints_len);
	//if(mode == AUTO_RESET_M)
	{		
		if(traj_len != 0)
		{
			arm_group->clearPlannedPathFIFO();
		}
		if(joints_len != 0)
		{
			arm_group->clearJointTrajectoryFIFO();
		}
		return;
	}
	lock.unlock();

	int traj_len1 = arm_group->getPlannedPathFIFOLength();	
	int joints_len1 = arm_group->getJointTrajectoryFIFOLength();
	printf("traj_len:%d,joints_len:%d\n",traj_len1, joints_len1);

}

/**
 * @brief add one manual instruction to the queue
 *
 * @param cmd_instruction the instruction to add
 */
void Robot_Motion::Add_Manual_Queue(motion_spec_MOTIONTYPE command_type, string arguments)
{
	Command_Instruction cmd_instruction;
	cmd_instruction.has_smooth = true;
	cmd_instruction.commandtype = command_type;
	cmd_instruction.command_arguments = arguments;

	manual_instruction_queue.push(cmd_instruction);
}


/**
 * @brief :add one command in to queue
 *
 * @param motion_command :the command to push
 */
void Robot_Motion::Add_Motion_Queue(motion_spec_MotionCommand &motion_command)
{
	motion_queue.push(motion_command);
}

/**
 * @brief pick one manual move instruction
 *
 * @return 0 if Success
 */
bool Robot_Motion::Pick_Manual_Instruction()
{
	if(next_move_instruction.id == 0)
		cur_instruction = next_move_instruction;
	else if(!manual_instruction_queue.empty())
		manual_instruction_queue.wait_and_pop(cur_instruction);
	else
		return false;

	if(!manual_instruction_queue.empty())
		manual_instruction_queue.wait_and_pop(next_move_instruction);
	else 
	{
		cur_instruction.has_smooth = false;
		next_move_instruction.id = -1;
	}

	return true;
}


/**
 * @brief :pick one motion instruction from the queue and execute it
 *
 * @return 0 if Success
 */
bool Robot_Motion::Pick_Motion_Instruction()
{
	bool ret;
	ErrorCode error_code;
	motion_spec_MotionCommand motion_command;
	//before move, do other actions like set DI and DO etc...
	if(!non_move_queue.empty())
	{
		printf("non move queue not empty\n");
		//execute non move instruction
		//ret = true;
	}
	else
	{
		if((next_move_id == next_move_instruction.id) && (next_move_id >= 0)) //the instruction has been already decoded 
		{				
			cur_instruction = next_move_instruction;
			printf("cur instruction id:%d\n", cur_instruction.id);
		}
		else if(!motion_queue.empty()) //start to move
		{
			//printf("motion queue not empty\n");
			//make sure current instruction
			motion_queue.wait_and_pop(motion_command);
			if((motion_command.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
			|| (motion_command.commandtype == motion_spec_MOTIONTYPE_CARTMOTION))
			{		
				error_code = Parse_Motion_Command(motion_command, cur_instruction);
			}
		}
		else
		{
			return false;//no motion instruction left
		}

		if(cur_instruction.has_smooth == true)
		{
			printf("==Find_Next_Move_Command\n");
			motion_spec_MotionCommand next_motion_command;
			if(Find_Next_Move_Command(next_motion_command)) //find a next move instruction
			{	
				error_code = Parse_Motion_Command(next_motion_command, next_move_instruction); //move smooth
				next_move_id = next_move_instruction.id;
				//then we can moveJ or moveL
			}
			else // can't find next move instruction
			{
				printf("can't find next move \n");
				cur_instruction.has_smooth = false;
				next_move_id = -1;
			}					
		}
		else
		{
			next_move_id = -1;
		}
	}
		
		if((previous_command_id == -1) && (current_command_id == -1))//the first command instruction
		{
			current_command_id = cur_instruction.id;
		}

		return true;
}

/**
 * @brief find next movable command joint or cart
 *
 * @param next_motion_command :the result of the found
 *
 * @return :true if found one
 */
bool Robot_Motion::Find_Next_Move_Command(motion_spec_MotionCommand &next_motion_command)
{
    motion_spec_MotionCommand motion_command;
	while(!motion_queue.empty())
	{
		motion_queue.wait_and_pop(motion_command);
		if((motion_command.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
		|| (motion_command.commandtype == motion_spec_MOTIONTYPE_CARTMOTION))
		{
			next_motion_command = motion_command;  //next move command	found
			return true;			
		}
		else
		{
			non_move_queue.push(motion_command);
		}

	}

	return false;
}


/**
 * @brief 
 *
 * @param motion_command :the command to parse
 * @param cmd_instruction : it is an output argument and is used to store the result of parsing
 *
 * @return 0 if Success
 */
ErrorCode Robot_Motion::Parse_Motion_Command(motion_spec_MotionCommand motion_command, Command_Instruction &cmd_instruction)
{
	bool ret;
	ErrorCode error_code = Success;
    
	cmd_instruction.id = motion_command.id;
	cmd_instruction.commandtype = motion_command.commandtype;	

	motion_spec_MoveL moveL1;
	switch(motion_command.commandtype)
	{
		case motion_spec_MOTIONTYPE_JOINTMOTION:
			motion_spec_MoveJ moveJ;
			PARSE_FIELD(moveJ, motion_spec_MoveJ,\
					motion_command.commandarguments.bytes,\
					motion_command.commandarguments.size, ret);
			if(ret == false)
			{
				printf("error parse motion command\n");
				error_code = Proto_Parse_Motion_Cmd_Failed;
				return error_code;
			}
			printf("MoveJ: count:%d, target pose:%f,%f,%f,%f,%f,%f, vmax:%f, amax:%f,has_smth:%d,smth:%f\n", moveJ.targetPose_count,moveJ.targetPose[0],moveJ.targetPose[1],moveJ.targetPose[2],moveJ.targetPose[3],moveJ.targetPose[4],moveJ.targetPose[5],moveJ.vMax,moveJ.aMax,moveJ.has_smoothDistance,moveJ.smoothDistance);
			if((moveJ.has_smoothDistance) && (moveJ.smoothDistance > 0))
				cmd_instruction.has_smooth = true;
			else
				cmd_instruction.has_smooth = false;
			cmd_instruction.command_arguments = string((const char*)&moveJ, sizeof(moveJ));
			//memcpy((char*)cmd_instruction.command_arguments.c_str(), (const char*)&moveJ, sizeof(moveJ));
			break;
		case motion_spec_MOTIONTYPE_CARTMOTION:
			motion_spec_MoveL moveL;
			PARSE_FIELD(moveL, motion_spec_MoveL,\
					motion_command.commandarguments.bytes,\
					motion_command.commandarguments.size, ret);			
			printf("MoveL: count:%d, target pose:%f,%f,%f,%f,%f,%f, vmax:%f, amax:%f,has_smth:%d,smth:%f\n", moveL.targetPose_count,moveL.targetPose[0],moveL.targetPose[1],moveL.targetPose[2],moveL.targetPose[3],moveL.targetPose[4],moveL.targetPose[5],moveL.vMax,moveL.aMax,moveL.has_smoothDistance,moveL.smoothDistance);
			if((moveL.has_smoothDistance) && (moveL.smoothDistance > 0))
				cmd_instruction.has_smooth = true;
			else
				cmd_instruction.has_smooth = false;
			cmd_instruction.command_arguments = string((const char*)&moveL, sizeof(moveL));
			//memcpy((char*)cmd_instruction.command_arguments.c_str(), (const char*)&moveL, sizeof(moveL));			
			break;
		case motion_spec_MOTIONTYPE_SETOUTPUT:
			break;
		case motion_spec_MOTIONTYPE_WAIT:
			break;
		default:
			break;
	}
	

	
	return error_code;
}

/*JointCommand former_jnt_cmd;*/
//Robot_Motion::Compare_Joints_Cmd()
//(
 
//)

/**
 * @brief: pick instruction from instruction queue and execute the instruction 
 */
void Robot_Motion::Queue_Process()
{
	ErrorCode error_code;
	motion_spec_MoveL *moveL, *moveL_next;
	PoseEuler *cur_pose, *next_pose = NULL;
	JointValues *cur_joints, *next_joints;


	if((mode == INIT_M) || (mode ==PAUSE_M) || (mode == AUTO_RESET_M))
	{
		//printf("invalid mode %d\n", mode);
		return;
	}
	static int total_cnt = 0;

	if(share_mem.Is_Joint_Command_Written() == false)
	{
		bool ret = share_mem.Set_Joint_Positions(share_mem.shm_jnt_cmd.joint_cmd);
		if(ret == false)
		{
			//printf("set share_mem positions failed\n");
		}
		else
		{
			total_cnt++;
		//	printf("total_points1:%d\n", total_cnt);
		}
		return;
	}

	int traj_len = arm_group->getPlannedPathFIFOLength();
	
	int joints_len = arm_group->getJointTrajectoryFIFOLength();	

	//Check_Command_ID(joints_len);
	if(joints_len == 0)
	{		
		if(current_command_id >= 0)
		{
			previous_command_id = current_command_id;
		}

		current_command_id = -1; //stopped 
	}
	else if(joints_len > 0)
	{
		//printf("joints fifo length:%d\n", joints_len);
		int joints_in = (joints_len < NUM_OF_POINTS_TO_SHARE_MEM)?joints_len:NUM_OF_POINTS_TO_SHARE_MEM;
		//	printf("joint in :%d\n",joints_in);

		joint_traj.resize(0);
		if(arm_group->getPointsFromJointTrajectoryFIFO(joint_traj, joints_in, error_code) >= 0)
		{
			JointCommand joint_command;
			joint_command.total_points = joints_in;
			for(int i = 0; i < joints_in; i++)
			{
				joint_command.points[i].positions[0] = joint_traj[i].joints.j1;
				joint_command.points[i].positions[1] = joint_traj[i].joints.j2;
				joint_command.points[i].positions[2] = joint_traj[i].joints.j3;
				joint_command.points[i].positions[3] = joint_traj[i].joints.j4;
				joint_command.points[i].positions[4] = joint_traj[i].joints.j5;
				joint_command.points[i].positions[5] = joint_traj[i].joints.j6;
				joint_command.points[i].point_position = joint_traj[i].id & 0x03;  //last two bits as point position
 
				/*if(joint_command.points[i].point_position == START_POINT)*/
				//{
					//printf("|||=====>traj_id:");
					//for(int m = 0; m < joints_in; m++)
						//printf("%x ",joint_traj[m].id >> 2);
					//printf("\n");
					//printf("======the first joints:");
					//for(int m = 0; m < 6; m++)
					//{
						//printf("%f ", joint_command.points[i].positions[m]);
					//}
					//printf("\n");
				//}

				if(joint_command.points[i].point_position == END_POINT)
				{
					/*printf("=====>traj_id:");*/
					//for(int m = 0; m < joints_in; m++)
						//printf("%x ",joint_traj[m].id >> 2);
					/*printf("\n");*/
					printf("=======the last joints:");

					for(int m = 0; m < 6; m++)
					{
						printf("%f ", joint_command.points[i].positions[m]);
					}
					printf("\n");
				}
				int traj_id = joint_traj[i].id >> 2;
				if(current_command_id != traj_id) //id changed
				{
					id_changed_flag = false;
					previous_command_id = current_command_id;
					current_command_id = traj_id;
				}
			}
		//	printf("=====>traj_id:%x,position:%d--%d,id:%d\n",joint_traj[0].id, joint_command.points[0].point_position, joint_command.points[joints_in-1].point_position,joint_traj[0].id >> 2);
			share_mem.shm_jnt_cmd.joint_cmd = joint_command;
			bool result = share_mem.Set_Joint_Positions(joint_command);
			if(result == false)
			{
				//printf("set share_mem positions failed\n");
			}
			else
			{
				total_cnt++;
				//printf("total_points:%d\n", total_cnt);
			}

		}
		//printf("cur id:%d, prev id:%d\n",current_command_id,previous_command_id);

	}
	
	if(traj_len > 0)
	{
		if(joints_len < MAX_CNT_VAL)
		{
			int joints_spare = MAX_CNT_VAL -joints_len;
			//printf("points fifo length:%d,joints_spare:%d\n", traj_len, joints_spare);
			int points_num;
			if(joints_spare > traj_len)
				points_num = (traj_len < MAX_PLANNED_POINTS_NUM)?traj_len:MAX_PLANNED_POINTS_NUM;
			else
				points_num = (joints_spare < MAX_PLANNED_POINTS_NUM)?joints_spare:MAX_PLANNED_POINTS_NUM;
			arm_group->convertPathToTrajectory(points_num, error_code);
			if(error_code != Success)
			{
				printf("convertPathToTrajectory failed:%d\n", error_code);
				arm_group->suspendArmMotion();
				Clear_Motion_Queue();
				arm_group->setStartState(Get_Cur_Joints_Value(), error_code);
	//			return;
			}
		}
	}
	else
	{
		if(mode == AUTO_RUN_M)			
		{
			if(Pick_Motion_Instruction())
			{
				printf("pick one instruction\n");
				
				Auto_Motion();
			}
		}
		else if((mode == MANUAL_JOINT_MODE_M) || (mode == MANUAL_CART_MODE_M))
		{
			if(Pick_Manual_Instruction())
			{
				printf("pick one manual instruction\n");
				switch(cur_instruction.commandtype)
				{
				case motion_spec_MOTIONTYPE_JOINTMOTION:
					cur_joints = (JointValues*)cur_instruction.command_arguments.c_str();
					if(next_move_instruction.id == -1)
						next_joints = NULL;
					else
						next_joints = (JointValues*)next_move_instruction.command_arguments.c_str();
					error_code = Move_Joints(cur_joints, next_joints);
					break;
				case motion_spec_MOTIONTYPE_CARTMOTION:
					cur_pose = (PoseEuler*)cur_instruction.command_arguments.c_str();
					cur_pose->position.x = cur_pose->position.x*1000;
					cur_pose->position.y = cur_pose->position.y*1000;
					cur_pose->position.z = cur_pose->position.z*1000;
					if(next_move_instruction.id == -1)
						next_pose = NULL;
					else
					{
						next_pose = (PoseEuler*)next_move_instruction.command_arguments.c_str();
						next_pose->position.x = next_pose->position.x*1000;
						next_pose->position.y = next_pose->position.y*1000;
						next_pose->position.z = next_pose->position.z*1000;
					}
					error_code = Move_Line(cur_pose, next_pose);
					break;
				case motion_spec_MOTIONTYPE_SETOUTPUT:
					break;
				case motion_spec_MOTIONTYPE_WAIT:
					break;
				default:
					break;

				}
			}
		}
	}
	
	
}

bool Robot_Motion::Check_Command_ID(int joints_len)
{
	bool ret = false;
//	int joints_len = arm_group->getJointTrajectoryFIFOLength();
	if(joints_len == 0)
	{		
		if(previous_command_id >= 0)
		{
			previous_command_id = current_command_id;
			ret = true;
		} 
		else
		{
			ret = false;
		}

		current_command_id = -1; //stopped 
		instruction_flag = true; //need to pick instruction===> situation 1
	}
	else if(joint_traj.size() > 0)
	{
		for(int i = 0; i < joint_traj.size(); i++)
		{
			int tmp_id = joint_traj[i].id;
			if(current_command_id != tmp_id)
			{
				previous_command_id = current_command_id;
				current_command_id = tmp_id;
				instruction_flag = true; //need to pick instruction===> situation 2
				ret = true;
				break;
			}
		}
	}
	else
		ret = false;

	return ret;
}


