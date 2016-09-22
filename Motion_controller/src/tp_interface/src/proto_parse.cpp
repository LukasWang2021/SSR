/**
 * @file proto_parse.cpp
 * @brief 
 * @author Wang Wei
 * @version 1.0.0
 * @date 2016-08-21
 */
#include "proto_parse.h"
#include "motionSL.pb.h"
#include "base_types.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <fstream>


#define PATH_LOGIC_MODE		("root/Logic/mode")
#define PATH_LOGIC_STATE	("root/Logic/state")
#define PATH_LOGIC_MODECMD	("root/Logic/modeCommand")
#define PATH_LOGIC_STATECMD	("root/Logic/stateCommand")

#define PATH_CTL_ACTUAL_TOOL_COORD		("root/Control/actualToolCoordinates")
#define PATH_CTL_ACTUAL_JOINT_POS		("root/Control/actualJointPositionsFiltered")
#define PATH_CTL_HOSTIN_TOOL_TRAJ		("root/Control/hostInToolTrajectory")
#define PATH_CTL_HOSTIN_JOINT_TRAJ		("root/Control/hostInJointTrajectory")
#define PATH_CTL_KINEMATICS_TOOL_COORD	("root/Control/forwardKinematics/toolCoordinates")

#define PATH_INTERP_PRE_CMD_ID			("root/MotionInterpreter/previous_command_id")
#define PATH_INTERP_CUR_CMD_ID			("root/MotionInterpreter/current_command_id")
#define PATH_INTERP_MOTION_PROGRAM		("root/MotionInterpreter/motion_program")


/**
 * @brief: stop in manuall mode
 *
 * @param:  parameters input
 */
void  Manual_Stop(void* params)
{
	Proto_Parse* p_proto_parse = (Proto_Parse*)params;
	//printf("=====pause!\n");
	//p_proto_parse->robot_motion.Manual_Pause();
	p_proto_parse->robot_motion.Clear_Motion_Queue();
	p_proto_parse->wdt_start_flag = false;
}

/**
 * @brief check if watchdog started in manual mode
 */
void Proto_Parse::Check_Manual_Wdt()
{
	if((robot_motion.mode == MANUAL_JOINT_MODE_M)
	|| (robot_motion.mode == MANUAL_CART_MODE_M))
	{
		if(robot_motion.manual_instruction_queue.empty())
		{
			if(wdt_start_flag == false)
			{
				wdt_start_flag = true;
				std::function<void(void*)> callback = Manual_Stop;
				wdt.Start(2000, callback, this);
			}
		}
	}

}

/**
 * @brief 
 *
 * @param buffer: buffer to store decoded data
 * @param count : the buffer size
 *
 * @return 
 */
ErrorCode Proto_Parse::Parse_Buffer(uint8_t *buffer, int count)
{
	bool ret;
	ErrorCode error_code;
	uint8_t *field_buffer = buffer+hash_size;
	int field_size = count - hash_size;
	
	/*Timer::Timeout time_out = Manual_Stop;*/
	//Timer timer(time_out);
	//timer.setSingleShot(true);
	/*timer.setInterval(Timer::Interval(200));*/

	if(HASH_CMP(ParameterSetMsg, buffer) == true)
	{
		BaseTypes_ParameterSetMsg param_set_msg;
		PARSE_FIELD(param_set_msg, BaseTypes_ParameterSetMsg, field_buffer, field_size, ret);
		if(ret == false)
			return Proto_Parse_Set_Msg_Failed;
		printf("set_param_msg:%s, elements:%d\n", param_set_msg.path, param_set_msg.number_of_elements);
		//===parse motion program================= 
		if(strcmp(PATH_INTERP_MOTION_PROGRAM, param_set_msg.path) == 0)
		{
			BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;
			//check if current mode id auto run
			if(robot_motion.mode != AUTO_RUN_M)
			{
				printf("error:wrong mode\n");
				status_code = BaseTypes_StatusCode_FAILED;
				Ret_Status(PATH_INTERP_MOTION_PROGRAM, status_code);
				nn_socket->NN_socket_Reply(sending_buffer, cmd_bytes_written);
				return Motion_In_Wrong_Mode;
			}
			error_code = Parse_Motion_Program(param_set_msg.param.bytes, param_set_msg.param.size);			
			if(error_code != Success)
				status_code = BaseTypes_StatusCode_FAILED;
			Ret_Status(PATH_INTERP_MOTION_PROGRAM, status_code);
			/*motion_spec_MotionCommand motion_command;*/
			//while(!robot_motion.motion_queue.empty()){
				//robot_motion.motion_queue.wait_and_pop(motion_command);
				//robot_motion.Parse_Motion_Command(motion_command, error_code);
				//if(error_code != Success)
					//return error_code
		}
		else if(strcmp(PATH_LOGIC_MODECMD, param_set_msg.path) == 0)
		{
			FST_ROBOT_MODE_CMD mode_cmd = *(FST_ROBOT_MODE_CMD*)param_set_msg.param.bytes;
			robot_motion.Set_Logic_Mode(mode_cmd);
			Ret_Param_Msg(PATH_LOGIC_MODECMD, robot_motion.mode);			
		}
		else if(strcmp(PATH_LOGIC_STATECMD, param_set_msg.path) == 0)
		{
			FST_ROBOT_STATE_CMD state_cmd = *(FST_ROBOT_STATE_CMD*)param_set_msg.param.bytes;
			robot_motion.Set_Logic_State(state_cmd);
			Ret_Param_Msg(PATH_LOGIC_STATECMD, robot_motion.state);
		}
		else if(strcmp(PATH_CTL_HOSTIN_JOINT_TRAJ, param_set_msg.path) == 0)
		{
			BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;
			
			if(robot_motion.mode != MANUAL_JOINT_MODE_M)
			{
			//	printf("wrong mode:%d\n", robot_motion.mode);
				status_code = BaseTypes_StatusCode_FAILED;
			}
			else
			{
				wdt.Pet(); //no matter there is timer or not
				string arguments = string((const char*)param_set_msg.param.bytes, param_set_msg.param.size);
				//memcpy(arguments, param_set_msg.param.bytes, param_set_msg.param.size);
				robot_motion.Add_Manual_Queue(motion_spec_MOTIONTYPE_JOINTMOTION, arguments);
				//JointValues joint = *(JointValues*)param_set_msg.param.bytes;
			//	robot_motion.joint_values = joint;
			}
			Ret_Status(PATH_CTL_HOSTIN_JOINT_TRAJ, status_code);
		}
		else if(strcmp(PATH_CTL_HOSTIN_TOOL_TRAJ, param_set_msg.path) == 0)
		{
			BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;			
			
			if(robot_motion.mode != MANUAL_CART_MODE_M)
			{
			//	printf("wrong mode:%d\n", robot_motion.mode);
				status_code = BaseTypes_StatusCode_FAILED;
			}
			else
			{
				wdt.Pet(); //no matter there is timer or not
				string arguments = string((const char*)param_set_msg.param.bytes, param_set_msg.param.size);
				robot_motion.Add_Manual_Queue(motion_spec_MOTIONTYPE_CARTMOTION, arguments);
				//PoseEuler pose = *(PoseEuler*)param_set_msg.param.bytes;
				//robot_motion.pose = pose;
			}
			Ret_Status(PATH_CTL_HOSTIN_TOOL_TRAJ, status_code);
		}
		
		// if success return status_code ok

		nn_socket->NN_socket_Reply(sending_buffer, cmd_bytes_written);	
	}
	else if(HASH_CMP(ParameterCmdMsg, buffer) == true)
	{
	    BaseTypes_ParameterCmdMsg param_cmd_msg;
		
		PARSE_FIELD(param_cmd_msg,BaseTypes_ParameterCmdMsg, field_buffer, field_size, ret);
	//	printf("the cmd msg path:%s\n", param_cmd_msg.path);

		if(param_cmd_msg.cmd == BaseTypes_CommandType_LIST)
		{
			//return all the parameter infomation
			map<string,BaseTypes_ParamInfo>::iterator it;
			int i = 0;
			BaseTypes_ParameterListMsg param_list_msg;
			param_list_msg.has_header = false;
			param_list_msg.params_count = json_parse->param_info_list.size();
			for(it=json_parse->param_info_list.begin(); it != json_parse->param_info_list.end(); it++)
			{				
				param_list_msg.params[i].has_info = true;
				printf("type:%d,size:%d\n",it->second.data_type, it->second.data_size);
				param_list_msg.params[i].info = it->second;
				int number_of_elements = param_list_msg.params[i].info.number_of_elements;
				int data_size = param_list_msg.params[i].info.data_size;
				int total_size = number_of_elements*data_size;
				char * tmp = new char[total_size];
				memset(tmp, 0, total_size);
				memcpy(param_list_msg.params[i].param.bytes, tmp, total_size);
				param_list_msg.params[i].param.size = total_size;
				delete tmp;

				i++;
			}

			SET_FIELD(param_list_msg, BaseTypes_ParameterListMsg, sending_buffer, sizeof(sending_buffer), hash_size, cmd_bytes_written, ret);
		//	printf("cmd_bytes_written:%d\n",bytes_written);
			/*std::ofstream fp("data1.bin",std::ofstream::binary);*/
			//fp.write( (char*)sending_buffer, cmd_bytes_written) ;
			/*fp.close();*/
		}
		else if(param_cmd_msg.cmd == BaseTypes_CommandType_ADD) //add an publisher in controller
		{
			printf("the add path:%s\n", param_cmd_msg.path);
			BaseTypes_StatusCode status_code;
			if(strcmp(param_cmd_msg.path, PATH_CTL_KINEMATICS_TOOL_COORD) == 0)
			{				
				//BaseTypes_ParamInfo parm_info = json_parse->param_info_list["root/Control/forwardKinematics/toolCoordinates"];
				if(param_cmd_msg.has_update_frq_devider == false)
				{
					status_code = BaseTypes_StatusCode_FAILED;
				}
				else
				{
					coord_publish_tm = param_cmd_msg.update_frq_devider/MIN_ROBOT_PUBLISH_TIME; //the unit is 10ms
					// if success return status_code ok
					status_code = BaseTypes_StatusCode_OK;
				}
				Ret_Status(PATH_CTL_KINEMATICS_TOOL_COORD, status_code);			
			}
			else if(strcmp(param_cmd_msg.path, PATH_CTL_ACTUAL_JOINT_POS) == 0)
			{
				if(param_cmd_msg.has_update_frq_devider == false)
				{
					status_code = BaseTypes_StatusCode_FAILED;
				}
				else
				{
					joint_publish_tm = param_cmd_msg.update_frq_devider/MIN_ROBOT_PUBLISH_TIME; //the unit is 10ms
					// if success return status_code ok
					status_code = BaseTypes_StatusCode_OK;
				}
				Ret_Status(PATH_CTL_ACTUAL_JOINT_POS, status_code);

			}
			else if(strcmp(param_cmd_msg.path, PATH_LOGIC_STATE) == 0)
			{
				if(param_cmd_msg.has_update_frq_devider == false)
				{
					status_code = BaseTypes_StatusCode_FAILED;
				}
				else
				{
					state_publish_tm = param_cmd_msg.update_frq_devider/MIN_ROBOT_PUBLISH_TIME; //the unit is 10ms
					// if success return status_code ok
					status_code = BaseTypes_StatusCode_OK;
				}
				Ret_Status(PATH_LOGIC_STATE, status_code);
			}
			else if(strcmp(param_cmd_msg.path, PATH_LOGIC_MODE) == 0)
			{
				printf("has freq:%d, freq:%d\n", param_cmd_msg.has_update_frq_devider, param_cmd_msg.update_frq_devider);
				if(param_cmd_msg.has_update_frq_devider == false)
				{
					status_code = BaseTypes_StatusCode_FAILED;
				}
				else
				{
					mode_publish_tm = param_cmd_msg.update_frq_devider/MIN_ROBOT_PUBLISH_TIME; //the unit is 10ms
					printf("mode tm:%d\n", mode_publish_tm);
					// if success return status_code ok
					status_code = BaseTypes_StatusCode_OK;					
				}
				Ret_Status(PATH_LOGIC_MODE, status_code);
			}
			

		}
		nn_socket->NN_socket_Reply(sending_buffer, cmd_bytes_written);
	}
	else if(HASH_CMP(ParameterGetMsg, buffer) == true)
	{
		BaseTypes_ParameterGetMsg param_get_msg;
		PARSE_FIELD(param_get_msg, BaseTypes_ParameterGetMsg, field_buffer, field_size, ret);
		printf("get path:%s\n", param_get_msg.path);

		if(strcmp(param_get_msg.path, PATH_INTERP_PRE_CMD_ID) == 0)
		{
			Ret_Param_Msg(PATH_INTERP_PRE_CMD_ID, robot_motion.previous_command_id);
		}
		if(strcmp(param_get_msg.path, PATH_INTERP_CUR_CMD_ID) == 0)
		{
			Ret_Param_Msg(PATH_INTERP_PRE_CMD_ID, robot_motion.current_command_id);
		}

		nn_socket->NN_socket_Reply(sending_buffer, cmd_bytes_written);
	}

//	printf("mode:%d\n",robot_motion.mode);

	return error_code;
}

/**
 * @brief return status to TP
 *
 * @param: status_code :status code
 *
 * @return: true if Success
 */
bool Proto_Parse::Ret_Status(string path, BaseTypes_StatusCode status_code)
{
	bool ret;
	BaseTypes_StatusMsg status_msg;
	status_msg.status = status_code;
	status_msg.has_info = true;
	status_msg.info = json_parse->param_info_list[path];

	SET_FIELD(status_msg, BaseTypes_StatusMsg, sending_buffer, sizeof(sending_buffer), hash_size, cmd_bytes_written, ret);

	return ret;
}


/**
 * @brief :return previous_command_id to TP
 *
 * @param previous_command_id :input 
 *
 * @return :true if Success
 */
bool Proto_Parse::Ret_Previous_Command_Id(int previous_command_id)
{
	bool ret;
	BaseTypes_ParameterMsg param_msg;
	param_msg.has_info = true;
	param_msg.info = json_parse->param_info_list[PATH_INTERP_PRE_CMD_ID];

	memcpy(param_msg.param.bytes, (char*)&previous_command_id, param_msg.info.data_size);
	SET_FIELD(param_msg, BaseTypes_ParameterMsg, sending_buffer, sizeof(sending_buffer), hash_size, cmd_bytes_written, ret);	

	return ret;
}


/**
 * @brief return current_command_id to TP
 *
 * @param current_command_id :input
 *
 * @return :true if Success
 */
bool Proto_Parse::Ret_Current_Command_Id(int current_command_id)
{
	bool ret;
	BaseTypes_ParameterMsg param_msg;
	param_msg.has_info = true;
	param_msg.info = json_parse->param_info_list[PATH_INTERP_CUR_CMD_ID];

	memcpy(param_msg.param.bytes, (char*)&current_command_id, param_msg.info.data_size);
	SET_FIELD(param_msg, BaseTypes_ParameterMsg, sending_buffer, sizeof(sending_buffer), hash_size, cmd_bytes_written, ret);

	return ret;
}

/**
 * @brief :return parameter to TP with a template name T
 *
 * @tparam T : the template
 * @param param :input
 *
 * @return :true if Success
 */
template<typename T> 
bool Proto_Parse::Ret_Param_Msg(string path, T param)
{
	bool ret;
	BaseTypes_ParameterMsg param_msg;
	param_msg.has_info = true;
	param_msg.info = json_parse->param_info_list[path];
	
	int length = param_msg.info.data_size*param_msg.info.number_of_elements;
	memcpy(param_msg.param.bytes, (char*)&param, length);
	param_msg.param.size = length;
	SET_FIELD(param_msg, BaseTypes_ParameterMsg, sending_buffer, sizeof(sending_buffer), hash_size, cmd_bytes_written, ret);

	return ret;
}


/**
 * @brief :return parameter to TP with a template name T
 *
 * @param  T: template, the parameter to publish
 * @path : parameter path
 * @param param :input
 *
 * @return :true if Success
 */
template<typename T>
bool Proto_Parse::Pub_Param_Msg(string path, T param)
{
	bool ret;
	BaseTypes_ParameterMsg param_msg;
	param_msg.has_info = false;
	BaseTypes_ParamInfo info = json_parse->param_info_list[path];

	param_msg.has_header =true;
	param_msg.header.frameCounter = 0;
	param_msg.header.timestamp = 0;
	
	int length = info.data_size*info.number_of_elements;
	memcpy(param_msg.param.bytes, (char*)&param, length);
	param_msg.param.size = length;

	int id_len = sizeof(int);
	memcpy(publish_buffer, (char*)&info.id, id_len);

	pb_ostream_t stream = {0};
	stream = pb_ostream_from_buffer(publish_buffer+id_len, sizeof(publish_buffer)-hash_size);
	ret = pb_encode(&stream, BaseTypes_ParameterMsg_fields, &param_msg);
	state_bytes_written = stream.bytes_written+id_len;
	//SET_FIELD(param_msg, BaseTypes_ParameterMsg, publish_buffer, sizeof(publish_buffer), hash_size, state_bytes_written, ret);

	if(ret == true)
		int result = nn_socket->NN_Socket_Publish(publish_buffer, state_bytes_written);
	/*memset(publish_buffer, 0, sizeof(publish_buffer));*/
	//BaseTypes_ParamInfo info = json_parse->param_info_list[path];
	
	//publish_buffer[id_len] = 10;
	//publish_buffer[id_len+1] = 8;
	//publish_buffer[id_len+2] = 8;
	//publish_buffer[id_len+3] = 0;
	//publish_buffer[id_len+4] = 16;
	//publish_buffer[id_len+10] = 26;
	//publish_buffer[id_len+11] = 4;
	//memcpy(publish_buffer+12+id_len, (char*)&param, length);
	//state_bytes_written = id_len+length+12;
	/*nn_socket->NN_Socket_Publish(publish_buffer, state_bytes_written);*/
	
	return ret;
}


bool Proto_Parse::Pub_Cur_State()
{
	return Pub_Param_Msg(PATH_LOGIC_STATE, robot_motion.state);
}

bool Proto_Parse::Pub_Cur_Mode()
{
	return Pub_Param_Msg(PATH_LOGIC_MODE, robot_motion.mode);
}

bool Proto_Parse::Pub_Cur_Joint()
{
	return Pub_Param_Msg(PATH_CTL_ACTUAL_JOINT_POS, robot_motion.joint_values);
}

bool Proto_Parse::Pub_Cur_Coord()
{
	PoseEuler pose;	
	robot_motion.Get_Cur_Position(pose);
	//printf("x:%f,y:%f,z:%f,a:%f,b:%f,c:%f\n",pose.position.x,pose.position.y,\
						pose.position.z,pose.orientation.a,pose.orientation.b,pose.orientation.c);
	return Pub_Param_Msg(PATH_CTL_KINEMATICS_TOOL_COORD, pose);
}


/**
 * @brief :parse the motion program 
 *
 * @param buffer: the buffer to parse
 * @param count : the count of buffer
 *
 * @return :0 if Success
 */
ErrorCode Proto_Parse::Parse_Motion_Program(uint8_t *buffer, int count)
{
	bool ret;
	motion_spec_MotionProgram motion_program;
	motion_spec_MotionCommand motion_command;
	PARSE_FIELD(motion_program, motion_spec_MotionProgram, buffer, count, ret);
	if(ret == false)
		return Proto_Parse_Motion_Prgm_Failed;
	printf("motion program: name:%s, id:%d, cmd_list:%d\n", motion_program.name, motion_program.id, motion_program.commandlist_count);
	for(int j = 0; j < motion_program.commandlist_count; j++)
	{	 		
		robot_motion.Add_Motion_Queue(motion_program.commandlist[j]);  // add to the queue of motion instructions
		//robot_motion.Parse_Motion_Command(motion_program.commandlist[j]);
	}

	return Success;
}

/*bool Proto_Parse::Execute_Motion_Program()*/
/*{}*/
