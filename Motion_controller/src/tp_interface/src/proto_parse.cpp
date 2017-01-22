/**
 * @file proto_parse.cpp
 * @brief: use the new moveJ and moveL proto 
 * @author Wang Wei
 * @version 1.2.0
 * @date 2016-12-18
 */
#include "proto_parse.h"
#include "motionSL.pb.h"
#include "base_types.pb.h"
#include "common.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <fstream>
#include "rt_timer.h"


/**
 * @brief: stop in manuall mode
 *
 * @param: input==>parameters input
 */
void manualStop(void* params)
{
	ProtoParse* p_proto_parse = (ProtoParse*)params;
    p_proto_parse->manual_start_time_ = getCurTime();
	//FST_INFO("====pause!");
    boost::mutex mutex;
    boost::mutex::scoped_lock lock(mutex);
    RobotMotion *robot_motion = p_proto_parse->getRobotMotionPtr();
	robot_motion->actionPause();
	robot_motion->clearManualQueue();
    robot_motion->clearPathFifo();
	
	p_proto_parse->wdt_start_flag_ = false;

}


ProtoParse::ProtoParse(RosBasic *ros_basic):ros_basic_(ros_basic)
{
    wdt_start_flag_ = false;
   
	manual_start_time_  = getCurTime();

	string str_addr = getLocalIP();

	hash_size_ = get_hash_size_();
	NNSockType type = NN_SOCK_WS;
	nn_socket_ = new NNSocket(str_addr, type, type, COMMAND_PORT, STATE_PORT);
	FST_ASSERT(nn_socket_);	
	
	json_parse_ = new JsonParse(FILE_API_PATH);
	FST_ASSERT(json_parse_);

    removeAllPubParams();    

    U64 result = robot_motion_.initial();
    if(result != FST_SUCCESS)
    {
        FST_ERROR("initial robbot motion failed");
        storeErrorCode(result);
    }
}
 
ProtoParse::~ProtoParse()
{
    if (nn_socket_)
    {
	    delete nn_socket_;
    }
    if (json_parse_)
    {
	    delete json_parse_;
    }

    robot_motion_.destroy();
}


/**
 * @brief: get pointer of RobotMotion object
 *
 * @return: the object 
 */
RobotMotion* ProtoParse::getRobotMotionPtr()
{
	return &robot_motion_;
}

/**
 * @brief: the main process of parser 
 */
void ProtoParse::StartParser()
{
	nn_socket_->nnPoll(2);
	if(nn_socket_->nnSocketRecieve() == true)
	{
		parseBuffer(nn_socket_->getRequestBufPtr(), nn_socket_->getRequestBufLen());
	}
}
	
/**
 * @brief: check if watchdog started in manual mode
 */
void ProtoParse::checkManualWdt()
{
	RobotMode mode = robot_motion_.getLogicMode();
	if ((mode == MANUAL_JOINT_MODE_M)
	|| (mode == MANUAL_CART_MODE_M))
	{
	//	if (robot_motion_.manual_instruction_queue_.empty())
		{
			if (wdt_start_flag_ == false)
			{
				wdt_start_flag_ = true;
				std::function<void(void*)> callback = manualStop;
				wdt_.Start(MANUAL_EXPIRE_TIME, callback, this);
			}
		}
	}

}

void ProtoParse::updateParams()
{
    U64 result;
    static U64 prev_err;
    /*static int global_update_cnt = 0;*/
    //if (++global_update_cnt >= 500)
    //{
        //global_update_cnt = 0;
        //map<int, PublishUpdate>::iterator it = param_pub_map_.begin();
        //for (; it != param_pub_map_.end(); ++it)
        //{
            //it->second.update_flag = true;
        //}

    /*}*/
    if ((result = robot_motion_.updateJoints()) == FST_SUCCESS) //get current joints from share memory
    {        
        if (robot_motion_.isJointsChanged())
        {
            setUpdateFlagByID(ACTUAL_JOINT_POS_ID, true);
            if ((result = robot_motion_.updatePose()) == FST_SUCCESS)
            {
                setUpdateFlagByID(FK_TOOL_COORD_ID, true);
            }
            else
            {
                if (!isUpdatedSameError(result))
                {
                    storeErrorCode(result);
                }
            }
#ifdef SIMMULATION
            // ===update joints and publish to rviz===================
            JointValues cur_joints = robot_motion_.getCurJointsValue();
            ros_basic_->pubJointState(cur_joints);
#endif
        }        
    }
    else
    {
        if (!isUpdatedSameError(result))                
        {
            storeErrorCode(result);
        }
    }
    if ((result = robot_motion_.updateLogicMode()) != PARAMETER_NOT_UPDATED)
    {
        setUpdateFlagByID(LOGIC_MODE_ID, true);
        if (result != FST_SUCCESS)
        {
            if (!isUpdatedSameError(result))                
            {
                storeErrorCode(result);
            }
        }
    }
    if ((result = robot_motion_.updateLogicState()) != PARAMETER_NOT_UPDATED)
    {
/*        if (robot_motion_.getLogicState() == RESET_ESTOP_T)*/
        //{
            //if (robot_motion_.getErrorState() == false)
            //{
                //clearUpdatedError();
            //}
        /*}*/
        setUpdateFlagByID(LOGIC_STATE_ID, true);
        if (result != FST_SUCCESS)
        {
            if (!isUpdatedSameError(result))                
            {
                storeErrorCode(result);
            }
        }
    }

    if ((result = robot_motion_.updateSafetyStatus()) != FST_SUCCESS)
    {
        if (!isUpdatedSameError(result))                
        {
            storeErrorCode(result);
        }
    }
  
    if (!error_queue_.empty())
    {
       // U64 err;
        /*error_queue_.waitAndPop(err);*/
        /*FST_INFO("err:%llx",err);*/        
        setUpdateFlagByID(ERROR_WARNINGS_ID, true);
    }

}


/**
 * @brief publish the parameter list
 */
void ProtoParse::pubParamList()
{
	boost::mutex::scoped_lock lock(mutex_);
    RobotMode mode = robot_motion_.getLogicMode();

    //FST_INFO("pubsize:%d",param_pub_map_.size());
	map<int, PublishUpdate>::iterator it = param_pub_map_.begin();
	for (; it != param_pub_map_.end(); ++it)
	{
		int msg_id = it->first;
		PublishUpdate pub_update = it->second;
		if(pub_update.relative_time)
		{
			pub_update.update_cnt++;
            if (pub_update.relative_time <= pub_update.update_cnt)
            {
                pub_update.update_cnt = 0;
               // FST_INFO("id:%d,update_flag:%d",msg_id, pub_update.update_flag);
             //   if (pub_update.update_flag == true)
                {
                    pubParamByID(msg_id);								
             //       pub_update.update_flag = false;
                }
            }			
        }
		it->second = pub_update;
	}
}

void ProtoParse::storeErrorCode(U64 err_code)
{
   // static U64 prev_err;

    if (err_code & 0x0001000000000000)
    {
        //FST_ERROR("err_code:%llx",err_code);        
        int warning_level = (err_code & 0x0000FFFF00000000) >> 32;
        if (warning_level > 4) //bigger than INFO level
        {
            FST_ERROR("warning_level:%d",warning_level);
            robot_motion_.setErrorState(true);
            robot_motion_.setLogicStateCmd(EMERGENCY_STOP_E);
        }
        else if (warning_level > 2)
        {
            robot_motion_.setLogicModeCmd(GOTO_PAUSE_E);
        }
       // if (prev_err != err_code)
        {
            error_queue_.push(err_code);
          //  prev_err = err_code;
        }
    }
}

bool ProtoParse::isUpdatedSameError(U64 err)
{
    boost::mutex::scoped_lock lock(mutex_);
    if (err & 0x0001000000000000)
    {
        if (prev_err_ != err)
        {
            prev_err_ = err;
            return false;
        }
        else
        {
            return true;
        }
    }
}


/**
 * @brief: parse the struct of set message
 *
 * @param field_buffer: input==>the buffer pointer
 * @param field_size: input==>the length of buffer
 */
void ProtoParse::parseParamSetMsg(const uint8_t *field_buffer, int field_size)
{
	uint32_t msg_id = 0;
	bool result;

	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;
	BaseTypes_ParameterSetMsg param_set_msg;
	PARSE_FIELD(param_set_msg, BaseTypes_ParameterSetMsg, field_buffer, field_size, result);
	if (result == false)
	{
		FST_ERROR("failed to parse param_set_msg");
        storeErrorCode(DECODE_MESSAGE_FAILED);
		retErrorStatus();
		return;  //need to define this error
	}
	
	if(checkPathAndID(param_set_msg, msg_id) == false)
	{
		return;	
	}
	FST_INFO("set_param_msg:%s, id:%d ,has_path:%d, has_id:%d, param_id:%d",\
			param_set_msg.path, msg_id, param_set_msg.has_path, param_set_msg.has_id, param_set_msg.id);
	//===parse motion program================= 
	switch (msg_id)
	{
		case MODE_COMMAND_ID:
		{
			RobotModeCmd mode_cmd = *(RobotModeCmd*)param_set_msg.param.bytes;
			setProtoLogicMode(msg_id, mode_cmd);	
			break;
		}			
		case STATE_COMMAND_ID:
		{
			RobotStateCmd state_cmd = *(RobotStateCmd*)param_set_msg.param.bytes;
			setProtoLogicState(msg_id, state_cmd);
			break;
		}
		case HOSTIN_JOINT_TRAJ_ID:
		{
			string joints = string((const char*)param_set_msg.param.bytes, param_set_msg.param.size);
			setProtoJiontTraj(msg_id, joints);
			break;
		}
		case HOSTIN_TOOL_COORD_ID:
		{
			string coordinates = string((const char*)param_set_msg.param.bytes, param_set_msg.param.size);
			setProtoToolCoord(msg_id, coordinates);
			break;
		}
        case CTL_COMMAND_ID:
        {
            RobotCtrlCmd cmd = *(RobotCtrlCmd*)param_set_msg.param.bytes;
            if(cmd == SHUTDOWN_CMD) //reset error
            {
                robot_motion_.setLogicStateCmd(EMERGENCY_STOP_E);
                retStatus(param_set_msg.path, param_set_msg.id, BaseTypes_StatusCode_OK);
            }
            break;
        }
        case ID_PREVIOUS_COMMAND_ID:
        {
            int prev_id = *(int*)param_set_msg.param.bytes;
            robot_motion_.setPreviousCmdID(prev_id);
			retStatus(param_set_msg.path, param_set_msg.id, BaseTypes_StatusCode_OK);
            break;
        }
		default:
		{
			//no id exist
			if (strcmp(PATH_INTERP_MOTION_PROGRAM, param_set_msg.path) == 0)
			{						
				//check if current mode id auto run
				if (robot_motion_.getLogicMode() != AUTO_RUN_M)
				{
					FST_ERROR("error:wrong mode");
                    storeErrorCode(INVALID_ACTION_IN_CURRENT_MODE);
					status_code = BaseTypes_StatusCode_FAILED;				
				}
				else
				{
					if (parseMotionProgram(param_set_msg.param.bytes, param_set_msg.param.size) == false)
                    {
                        storeErrorCode(DECODE_MESSAGE_FAILED);
						status_code = BaseTypes_StatusCode_FAILED;
                    }
				}

				retStatus(param_set_msg.path, param_set_msg.id, status_code);
			}//end if (strcmp(PATH_INTERP_MOTION_PROGRAM, param_set_msg.path) == 0)
			else
			{
				FST_ERROR("Invalid path:%s", param_set_msg.path);
				retStatus(param_set_msg.path, param_set_msg.id, BaseTypes_StatusCode_FAILED);
			}
			break;
		}//end default
	}//end 	switch (msg_id)
}

/**
 * @brief: parse the struct of command message
 *
 * @param field_buffer: input==>the buffer pointer
 * @param field_size: input==>the length of buffer
 */
void ProtoParse::parseParamCmdMsg(const uint8_t *field_buffer, int field_size)
{
	uint32_t msg_id = 0;
	bool result;
	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;

	BaseTypes_ParameterCmdMsg param_cmd_msg;		
	PARSE_FIELD(param_cmd_msg,BaseTypes_ParameterCmdMsg, field_buffer, field_size, result);
	if(result == false)
	{
		FST_ERROR("parse param_cmd_msg failed");
        storeErrorCode(DECODE_MESSAGE_FAILED);
        retErrorStatus();
        return;
	}

	if(checkPathAndID(param_cmd_msg, msg_id) == false)
	{
		return;
	}

    //FST_INFO("path:%s,param_id:%d", param_cmd_msg.path, param_cmd_msg.id);
	switch (param_cmd_msg.cmd)
	{
		case BaseTypes_CommandType_LIST:
		{
			FST_INFO("list...\n");
			//return all the parameter infomation			
			BaseTypes_ParameterListMsg param_list_msg = json_parse_->getParamListMsg();
			retParamListMsg(param_list_msg);
			break;
		}			
		case BaseTypes_CommandType_ADD:
		{
			addPubParameter(param_cmd_msg.path, msg_id, param_cmd_msg.has_update_frq_devider,param_cmd_msg.update_frq_devider);
			break;
		}
		case BaseTypes_CommandType_REMOVE:
		{
			removePubParameter(param_cmd_msg.path, msg_id);
			break;
		} 
		case BaseTypes_CommandType_REMOVE_ALL:
		{
			removeAllPubParams();
			break;
		} 
		default:
        {
			FST_ERROR("ERROR command:%d", param_cmd_msg.cmd);
            storeErrorCode(INVALID_PARAM_FROM_TP);
			break;
        }
	}//end switch (param_cmd_msg.cmd)

}
/**
 * @brief: parse the struct of get message
 *
 * @param field_buffer: input==>the buffer pointer
 * @param field_size: input==>the length of buffer
 *
 * @return:true if success 
 */
void ProtoParse::parseParamGetMsg(const uint8_t *field_buffer, int field_size)
{
	uint32_t msg_id = 0;
	bool ret;
	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;

	BaseTypes_ParameterGetMsg param_get_msg;
	PARSE_FIELD(param_get_msg, BaseTypes_ParameterGetMsg, field_buffer, field_size, ret);
	if (json_parse_->getIDFromPath(param_get_msg.path, msg_id) == false)
	{
		FST_ERROR("ERROR:get_param_msg:%s,", param_get_msg.path);
        storeErrorCode(DECODE_MESSAGE_FAILED);
		retErrorStatus();
		return;  //need to define this error
	}
//	FST_INFO("get_param_msg:%s, id:%d, param_id:%d", param_get_msg.path, msg_id, param_get_msg.id);

	switch(msg_id)
	{
        case SAFETY_INPUT_FRAME1_ID:
			retParamMsg(param_get_msg.path, msg_id, robot_motion_.getPreviousCmdID());
			break;
		case SAFETY_INPUT_FRAME2_ID:
			retParamMsg(param_get_msg.path, msg_id, robot_motion_.getCurrentCmdID());
			break;
		case ID_PREVIOUS_COMMAND_ID:
			retParamMsg(param_get_msg.path, msg_id, robot_motion_.getPreviousCmdID());
			break;
		case ID_CURRENT_COMMAND_ID:
			retParamMsg(param_get_msg.path, msg_id, robot_motion_.getCurrentCmdID());
			break;
        case ETHERCAT_SMNT_DIN3_ID:
        {
            U64 result;
            retParamMsg(param_get_msg.path, msg_id, robot_motion_.getSafetyInterfacePtr()->getDITPManual());
            break;
        }
        case ETHERCAT_SMNT_DIN4_ID:
        {
            U64 result;
            retParamMsg(param_get_msg.path, msg_id, robot_motion_.getSafetyInterfacePtr()->getDITPAuto());
            break;
        }
        case LOGIC_STATE_ID:		
		{
			RobotState state = robot_motion_.getLogicState();
			retParamMsg(param_get_msg.path, msg_id, state);
			break;
		}
		case LOGIC_MODE_ID:
		{
			RobotMode mode = robot_motion_.getLogicMode();
			retParamMsg(param_get_msg.path, msg_id, mode);
			break;
		}
		case ACTUAL_JOINT_POS_ID:
		{
			JointValues jnt_val = robot_motion_.getCurJointsValue();
			retParamMsg(param_get_msg.path, msg_id, jnt_val);
			break;
		}
		case FK_TOOL_COORD_ID:
		{
            PoseEuler pose = robot_motion_.getCurPosition();
			retParamMsg(param_get_msg.path, msg_id, pose);
			break;
		}
        case ERROR_WARNINGS_ID:
        {
            U64 err[ERROR_NUM] = {0};
            for (int i = 0; i < ERROR_NUM; ++i)
            {
                if (error_queue_.empty())
                {
                    break;
                }
                error_queue_.waitAndPop(err[i]);
            }
            retParamMsg(param_get_msg.path, msg_id, err[0]);
            break;
        }

		default:
            storeErrorCode(INVALID_PARAM_FROM_TP);
			break;
	}

}

void ProtoParse::parseParamOverwriteMsg(const uint8_t *field_buffer, int field_size)
{
	uint32_t msg_id = 0;
	bool ret;
	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;

	BaseTypes_ParameterOverwriteMsg param_ovwrt_msg;
	PARSE_FIELD(param_ovwrt_msg, BaseTypes_ParameterOverwriteMsg, field_buffer, field_size, ret);
    if (json_parse_->getIDFromPath(param_ovwrt_msg.path, msg_id) == false)
	{
		FST_ERROR("ERROR:overwrite_param_msg:%s,", param_ovwrt_msg.path);
        storeErrorCode(DECODE_MESSAGE_FAILED);
		retErrorStatus();

		return;  //need to define this error
	}
    switch(msg_id)
	{
		case SAFETY_INPUT_FRAME1_ID:
			retParamMsg(param_ovwrt_msg.path, msg_id, robot_motion_.getPreviousCmdID());
			break;
		case SAFETY_INPUT_FRAME2_ID:
			retParamMsg(param_ovwrt_msg.path, msg_id, robot_motion_.getCurrentCmdID());
			break;
        case ETHERCAT_SMNT_DIN3_ID:
        {
            U64 result;
            retParamMsg(param_ovwrt_msg.path, msg_id, robot_motion_.getSafetyInterfacePtr()->getDITPManual());
            break;
        }
        case ETHERCAT_SMNT_DIN4_ID:
        {
            U64 result;
            retParamMsg(param_ovwrt_msg.path, msg_id, robot_motion_.getSafetyInterfacePtr()->getDITPAuto());
            break;
        }
		default:
            storeErrorCode(INVALID_PARAM_FROM_TP);
			break;
	}

}

/**
 * @brief: parse the buffer input 
 *
 * @param buffer: input==>buffer to store decoded data
 * @param count: input==>the buffer size
 */
void ProtoParse::parseBuffer(const uint8_t *buffer, int count)
{
	const uint8_t *field_buffer = buffer+hash_size_;
	int field_size = count - hash_size_;
	

	if (HASH_CMP(ParameterSetMsg, buffer) == true)
	{
		parseParamSetMsg(field_buffer, field_size);
	}//end if (HASH_CMP(ParameterSetMsg, buffer) == true)
	else if (HASH_CMP(ParameterCmdMsg, buffer) == true)
	{
	    parseParamCmdMsg(field_buffer, field_size);
	}//end else if (HASH_CMP(ParameterCmdMsg, buffer) == true)
	else if (HASH_CMP(ParameterGetMsg, buffer) == true)
	{
		parseParamGetMsg(field_buffer, field_size);
	}//end else if (HASH_CMP(ParameterGetMsg, buffer) == true)
    else if (HASH_CMP(ParameterOverwriteMsg, buffer) == true)
	{
		parseParamOverwriteMsg(field_buffer, field_size);
	}//
    else
	{
		retErrorStatus();	
        storeErrorCode(INVALID_PARAM_FROM_TP);
	}
}

/**
 * @brief: return param_list_msg to TP 
 *
 * @param param_list_msg: input==>parameter list store in API.txt
 *
 * @return: true if success 
 */
bool ProtoParse::retParamListMsg(BaseTypes_ParameterListMsg param_list_msg)
{
	bool ret;

	uint8_t *buffer = nn_socket_->getReplyBufPtr();
	int length;
	SET_FIELD(param_list_msg, BaseTypes_ParameterListMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

    int bytes = nn_socket_->nnSocketReply(buffer, length);
	FST_INFO("ret param list size:%d",bytes);

	return ret;
}


/**
 * @brief: return status to TP
 *
 * @param: status_code: input==>status code
 *
 * @return: true if success
 */
bool ProtoParse::retStatus(const char *path, int id, BaseTypes_StatusCode status_code)
{
	bool ret;
	BaseTypes_StatusMsg status_msg;
	status_msg.status = status_code;

	FST_INFO("path:%s, id:%d",path, id);

	if(json_parse_->getParamInfo(status_msg.info, path, id) == false)
	{
		status_msg.has_info = false;
		//return false;
	}
	else
	{
		status_msg.has_info = true;
	}

	uint8_t *buffer = nn_socket_->getReplyBufPtr();
	int length;
	SET_FIELD(status_msg, BaseTypes_StatusMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

	//FST_INFO("len:%d",length);
	nn_socket_->nnSocketReply(buffer, length);

	return ret;
}


/**
 * @brief :return previous_command_id_ to TP
 *
 * @param previous_command_id_: input 
 *
 * @return: true if Success
 */
bool ProtoParse::retPreviousCommandId(int id)
{
	bool ret;
   /* BaseTypes_ParameterMsg param_msg;*/
	//param_msg.has_info = true;
	//ParamProperty param_property;
	//if(json_parse_->getParamFromID(id, param_property) == false)
		//return false;
	//param_msg.info = param_property.param_info;

	//memcpy(param_msg.param.bytes, (char*)&id, param_msg.info.data_size);

	//uint8_t *buffer = nn_socket_->getReplyBufPtr();
	//int length;
	//SET_FIELD(param_msg, BaseTypes_ParameterMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);	

	
	/*nn_socket_->nnSocketReply(buffer, length);*/

	return ret;
}


/**
 * @brief: return current_command_id_ to TP
 *
 * @param current_command_id_: input
 *
 * @return: true if Success
 */
bool ProtoParse::retCurrentCommandId(int id)
{
	bool ret;
	/*BaseTypes_ParameterMsg param_msg;*/
	//param_msg.has_info = true;
	//ParamProperty param_property;
	//if(json_parse_->getParamFromID(id, param_property) == false)
		//return false;
	//param_msg.info = param_property.param_info;

	//memcpy(param_msg.param.bytes, (char*)&id, param_msg.info.data_size);

	//uint8_t *buffer = nn_socket_->getReplyBufPtr();
	//int length;
	//SET_FIELD(param_msg, BaseTypes_ParameterMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

	/*nn_socket_->nnSocketReply(buffer, length);*/

	return ret;
}

/**
 * @brief :return parameter to TP with a template name T
 *
 * @tparam T: input ==>the template
 * @param param: input
 */
template<typename T> 
void ProtoParse::retParamMsg(const char *path, int id, T param)
{
	bool result;
	BaseTypes_ParameterMsg param_msg;
	
	if(json_parse_->getParamInfo(param_msg.info, path, id) == false)
	{
		param_msg.has_info = false;
		//return false;
	}
	else
	{
		param_msg.has_info = true;
		int into_bytes = param_msg.info.data_size*param_msg.info.number_of_elements;
		memcpy(param_msg.param.bytes, (char*)&param, into_bytes);
		param_msg.param.size = into_bytes;
	}	
	

	uint8_t *buffer = nn_socket_->getReplyBufPtr();
	int length;
	SET_FIELD(param_msg, BaseTypes_ParameterMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, result);

   // if (result)
    {
	    nn_socket_->nnSocketReply(buffer, length);
    }
   /* else*/
    //{
        //FST_ERROR("encode parameter message failed");
    /*}*/

}

/**
 * @brief: return parameter to TP with a template name T
 *
 * @param  T: input==>template, the parameter to publish
 * @path: input==>parameter path
 * @param param: input
 *
 * @return: true if success
 */
bool ProtoParse::pubParamMsg(int id, const char *param)
{
	bool result;
	if (param == NULL)
		return false;
	BaseTypes_ParameterMsg param_msg;
	param_msg.has_info = false;
	ParamProperty property;
	if(json_parse_->getParamFromID(id, property) == false)
		return false;
//	FST_INFO("path:%s, %d", info.path, id);

	param_msg.has_header =true;
	param_msg.header.frameCounter = 0;
	param_msg.header.timestamp = 0;
	
	int info_bytes = property.data_size*property.number_of_elements;
	memcpy(param_msg.param.bytes, param, info_bytes);
	param_msg.param.size = info_bytes;

	int id_len = sizeof(int);
	uint8_t *buffer = nn_socket_->getPublishBufPtr();
	memcpy(buffer, (char*)&id, id_len);

	pb_ostream_t stream = {0};	
	stream = pb_ostream_from_buffer(buffer+id_len, MAX_BUFFER_SIZE-hash_size_);
	result = pb_encode(&stream, BaseTypes_ParameterMsg_fields, &param_msg);
	int length = stream.bytes_written+id_len;
	//SET_FIELD(param_msg, BaseTypes_ParameterMsg, publish_buffer_, sizeof(publish_buffer_), hash_size_, state_bytes_written_, ret);

	//if (result == true)
    {
		nn_socket_->nnSocketPublish(buffer, length);
    }
	/*else*/
    //{
        //FST_ERROR("encode publish message failed");
    /*}*/

	return result;
}

/**
 * @brief: publish a parameter by the id
 *
 * @param id:input==>the uniq id of the parameter
 */
void ProtoParse::pubParamByID(int id)
{	
	char *param_ptr = NULL;
	switch (id)
	{
		case LOGIC_STATE_ID:		
		{
			RobotState state = robot_motion_.getLogicState();
			param_ptr = (char*)&state;
			break;
		}
		case LOGIC_MODE_ID:
		{
			RobotMode mode = robot_motion_.getLogicMode();
			param_ptr = (char*)&mode;
			break;
		}
		case ACTUAL_JOINT_POS_ID:
		{
			JointValues jnt_val = robot_motion_.getCurJointsValue();
			param_ptr = (char*)&jnt_val;
			break;
		}
		case FK_TOOL_COORD_ID:
		{
            PoseEuler pose = robot_motion_.getCurPosition();
			param_ptr = (char*)&pose;
			break;
		}
        case ERROR_WARNINGS_ID:
        {
            U64 err[ERROR_NUM] = {0};
            for (int i = 0; i < ERROR_NUM; ++i)
            {
                if (error_queue_.empty())
                {
                    break;
                }
                error_queue_.waitAndPop(err[i]);
            }
            param_ptr = (char*)err;
            break;
        }
		case DIN_USER1_ID:
		{
			break;
		}
		case DIN_USER2_ID:
		{
			break;
		}
		case DIN_USER3_ID:
		{
			break;
		}
		case DIN_USER4_ID:
		{
			break;
		}
		case DOUT_USER1_ID:
		{
			break;
		}
		case DOUT_USER2_ID:
		{
			break;
		}
		case DOUT_USER3_ID:
		{
			break;
		}
		case DOUT_USER4_ID:
		{
			break;
		}
        case SAFETY_INPUT_FRAME1_ID:
        {
            U32 frm_data = robot_motion_.getSafetyInterfacePtr()->getDIFrm1();
            param_ptr = (char*)&frm_data;
            break;
        }
        case SAFETY_INPUT_FRAME2_ID:
        {
            U32 frm_data = robot_motion_.getSafetyInterfacePtr()->getDIFrm2();
            param_ptr = (char*)&frm_data;
            break;
        }
		default:
			break;
	}	
    if(param_ptr != NULL)
    {
	    pubParamMsg(id, param_ptr);
    }
}

void ProtoParse::setUpdateFlagByID(int id, bool flag)
{
    boost::mutex::scoped_lock lock(mutex_);
    map<int, PublishUpdate>::iterator it = param_pub_map_.find(id);
    if (it != param_pub_map_.end())
    {
        it->second.update_flag = flag;
    }
}


/**
 * @brief: parse the motion program 
 *
 * @param buffer: input==>the buffer to parse
 * @param count: input==>the count of buffer
 *
 * @return: true if success
 */
bool ProtoParse::parseMotionProgram(const uint8_t *buffer, int count)
{
	bool ret;
	motion_spec_MotionProgram motion_program;
	PARSE_FIELD(motion_program, motion_spec_MotionProgram, buffer, count, ret);
	if (ret == false)
    {
        FST_ERROR("error parse motion program");
		return false;
    }
	FST_INFO("motion program: name:%s, id:%d, cmd_list:%d", motion_program.name, motion_program.id, motion_program.commandlist_count);
	for (int j = 0; j < motion_program.commandlist_count; ++j)
	{	 		
        CommandInstruction cmd_instruction;
        if (parseMotionCommand(motion_program.commandlist[j], cmd_instruction) == false)
        { 
            FST_ERROR("error parse motion command");
		    continue; //continue to parse next command
        }
        U64 result = robot_motion_.checkAutoStartState();
		robot_motion_.addMotionQueue(cmd_instruction);  // add to the queue of motion instructions
	}

	return true;
}

/**
 * @brief
 *
 * @param motion_command: input==>the command to parse
 * @param cmd_instruction: output==>store the result of parsing
 *
 * @return true if Success
 */
bool ProtoParse::parseMotionCommand(motion_spec_MotionCommand motion_command, CommandInstruction &cmd_instruction)
{
	bool result;	

    cmd_instruction.smoothDistance = 0; //0 is default
	cmd_instruction.id = motion_command.id;
	cmd_instruction.commandtype = motion_command.commandtype;

	switch (motion_command.commandtype)
	{
		case motion_spec_MOTIONTYPE_JOINTMOTION:
        {
			motion_spec_MoveJ moveJ;
			PARSE_FIELD(moveJ, motion_spec_MoveJ,\
					motion_command.commandarguments.bytes,\
					motion_command.commandarguments.size, result);
			if (result == false)
			{
				//FST_ERROR("error parse motion command");
				return false;
			}
			//FST_INFO("MoveJ: count:%d, target pose:%f,%f,%f,%f,%f,%f, vmax:%f, amax:%f,has_smth:%d,smth:%f", moveJ.targetPose_count,moveJ.targetPose[0],moveJ.targetPose[1],moveJ.targetPose[2],moveJ.targetPose[3],moveJ.targetPose[4],moveJ.targetPose[5],moveJ.vMax,moveJ.aMax,moveJ.has_smoothDistance,moveJ.smoothDistance);
#if 0
			if ((moveJ.has_smoothDistance) && (moveJ.smoothDistance > 0))
				cmd_instruction.has_smooth = true;
			else
				cmd_instruction.has_smooth = false;
#endif
			cmd_instruction.command_arguments = string((const char*)&moveJ, sizeof(moveJ));
			break;
        }
		case motion_spec_MOTIONTYPE_CARTMOTION:
        {
			motion_spec_MoveL moveL;
			PARSE_FIELD(moveL, motion_spec_MoveL,\
					motion_command.commandarguments.bytes,\
					motion_command.commandarguments.size, result);
            if (result == false)
			{
			//	FST_ERROR("error parse motion command");
				return false;
			}
		//	FST_INFO("MoveL: count:%d, target pose:%f,%f,%f,%f,%f,%f, vmax:%f, amax:%f,has_smth:%d,smth:%f\n", moveL.targetPose_count,moveL.targetPose[0],moveL.targetPose[1],moveL.targetPose[2],moveL.targetPose[3],moveL.targetPose[4],moveL.targetPose[5],moveL.vMax,moveL.aMax,moveL.has_smoothDistance,moveL.smoothDistance);
			if (moveL.waypoints_count == 0)
			{
				return false; // no points
			}
			else if (moveL.waypoints_count == 1)
			{
                motion_spec_WayPoint waypoint = moveL.waypoints[0];
				if (waypoint.has_blendInDistance) 
                {
                    cmd_instruction.smoothDistance = waypoint.blendInDistance;
                }
                else
                {
                    cmd_instruction.smoothDistance = -1;
                }
			}	
            
            cmd_instruction.command_arguments = string((const char*)&moveL, sizeof(moveL));
			break;
        }
		case motion_spec_MOTIONTYPE_SET:
			break;
		case motion_spec_MOTIONTYPE_WAIT:
			break;
		default:
			break;
	}//end switch (motion_command.commandtype)

	return true;
}


/**
 * @brief: compare two strings in the form of int
 *
 * @param cmp_a: input
 * @param cmp_b: input
 *
 * @return: true if they are the same 
 */
bool ProtoParse::compareInt(const unsigned char *cmp_a, const uint8_t *cmp_b)
{
	int *p_a = (int*)cmp_a;
	int *p_b = (int*)cmp_b;
	for (int i = 0; i < get_hash_size_()/sizeof(int); ++i)
	{
		if (p_a[i] != p_b[i])
		{
			//printf("a:%d, b:%d, %d\n", cmp_a[i], cmp_b[i],i);
			return false;
		}
	}

	return true;
}
template<typename T>
bool ProtoParse::checkPathAndID(T msg, uint32_t &id)
{
	if(msg.has_path)
	{
		if(json_parse_->getIDFromPath(msg.path, id))
		{
			if(msg.has_id)
			{
				if(id != msg.id)
				{
					FST_ERROR("the id is not the same as current id:%d", id);
					// in this situation, what can I do?
					retStatus(msg.path, -1, BaseTypes_StatusCode_FAILED);
					return false;  //need to define this error
				}
			}
		}		
	}
	return true;
}
/**
 * @brief: set logic mode  
 *
 * @param id: input==>the parameter id
 * @param mode_cmd: input==>the mode command
 */
void ProtoParse::setProtoLogicMode(int id, RobotModeCmd mode_cmd)
{
	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;

	boost::mutex::scoped_lock lock(mutex_);

    U64 result = robot_motion_.setLogicModeCmd(mode_cmd);
	if (result != FST_SUCCESS)
    {
        storeErrorCode(result);
    }
    
	//==============================================
	retParamMsg(PATH_LOGIC_MODE, id, robot_motion_.getLogicMode());
}
/**
 * @brief: set logic state 
 *
 * @param id: input==>the parameter id
 * @param state_cmd: input==>the state command
 */
void ProtoParse::setProtoLogicState(int id, RobotStateCmd state_cmd)
{
	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;

	boost::mutex::scoped_lock lock(mutex_);

    U64 result = robot_motion_.setLogicStateCmd(state_cmd);
	if (result != FST_SUCCESS)
    {
        //FST_ERROR("set state:%llx", result);
        storeErrorCode(SET_STATE_FAILED);
    }
    if (state_cmd == ACKNOWLEDGE_ERROR)
    {
       FST_INFO("clear prev_err_..."); 
       prev_err_ = FST_SUCCESS;
    }
	retParamMsg(PATH_LOGIC_STATE, id, robot_motion_.getLogicState());
}
/**
 * @brief: set joints trajectory 
 *
 * @param id: input==>the parameter id
 * @param state_cmd: input==>joints pointer
 */
void ProtoParse::setProtoJiontTraj(int id, string joints)
{
	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;

	boost::mutex::scoped_lock lock(mutex_);

	if (robot_motion_.getLogicMode() != MANUAL_JOINT_MODE_M)
	{
		status_code = BaseTypes_StatusCode_FAILED;
        storeErrorCode(INVALID_ACTION_IN_CURRENT_MODE);
	}
	else
	{
		wdt_.Pet(); //no matter there is timer or not
		double *db_param = (double*)joints.c_str();
		FST_PRINT("time:%d,joints params:",getCurTime()-manual_start_time_);
		for (int i = 0; i < MAX_JOINTS; i++)
		{
			FST_PRINT("%f ", db_param[i]);
		}
		FST_PRINT("\n");		
		
        U64 result = robot_motion_.checkManualStartState();
		robot_motion_.addManualQueue(motion_spec_MOTIONTYPE_JOINTMOTION, joints);
	}
	retStatus(PATH_CTL_HOSTIN_JOINT_TRAJ, id, status_code);
}
/**
 * @brief: set coordinates trajectory 
 *
 * @param id: input==>the parameter id
 * @param state_cmd: input==>coordinates pointer
 */
void ProtoParse::setProtoToolCoord(int id, string coordinates)
{
	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;

	boost::mutex::scoped_lock lock(mutex_);

	if (robot_motion_.getLogicMode() != MANUAL_CART_MODE_M)
	{
	//	printf("wrong mode:%d\n", robot_motion_.mode);
		status_code = BaseTypes_StatusCode_FAILED;
        storeErrorCode(INVALID_ACTION_IN_CURRENT_MODE);
	}
	else
	{
		wdt_.Pet(); //no matter there is timer or not
		double *db_param = (double*)coordinates.c_str();
		FST_PRINT("time:%d,tool params:",getCurTime()-manual_start_time_);
		for (int i = 0; i < MAX_JOINTS; i++)
		{
			FST_PRINT("%f ", db_param[i]);
		}
		FST_PRINT("\n");
        U64 result = robot_motion_.checkManualStartState();
		robot_motion_.addManualQueue(motion_spec_MOTIONTYPE_CARTMOTION, coordinates);
	}
	retStatus(PATH_CTL_ACTUAL_TOOL_COORD, id, status_code);

}
/**
 * @brief: add a parameter to publish list
 *
 * @param id: input==>the parameter id
 * @param has_freq: input==> if has frequency
 * @param update_freq: input==> the frequency value
 */
void ProtoParse::addPubParameter(const char * path, int id, bool has_freq, int update_freq)
{
	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;	
	if (has_freq == false)
	{
		status_code = BaseTypes_StatusCode_FAILED;
	}
	else 
	{
		int pub_tm = update_freq / INTERVAL_PROPERTY_UPDATE; //the unit is 10ms
		if (pub_tm == 0)
		{
			status_code = BaseTypes_StatusCode_FAILED;
		}
		else
		{
			PublishUpdate pub_update;
			pub_update.relative_time = pub_tm;
			pub_update.update_cnt = 0;
            pub_update.update_flag = true;

            boost::mutex::scoped_lock lock(mutex_);
			param_pub_map_.insert(map<int, PublishUpdate>::value_type(id, pub_update));
        }
	}
	retStatus(path, id, status_code);
}
/**
 * @brief: remove a paramet from the publish list 
 *
 * @param  id: input==>the parameter id
 */
void ProtoParse::removePubParameter(const char *path, int id)
{
	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;

	boost::mutex::scoped_lock lock(mutex_);

	map<int, PublishUpdate>::iterator it = param_pub_map_.begin();
	for (it; it != param_pub_map_.end(); ++it)
	{
		if (id == it->first)
		{
			param_pub_map_.erase(it);
			break;
		}
	}
	retStatus(path, id, status_code);
}
/**
 * @brief: remove all publish list 
 */
void ProtoParse::removeAllPubParams()
{
	boost::mutex::scoped_lock lock(mutex_);
	param_pub_map_.erase(param_pub_map_.begin(), param_pub_map_.end());
}

void ProtoParse::retErrorStatus()
{
    retStatus(PATH_ERROR_WARNINGS, ERROR_WARNINGS_ID, BaseTypes_StatusCode_FAILED);
}

void ProtoParse::clearUpdatedError()
{
     boost::mutex::scoped_lock lock(mutex_);

     FST_INFO("clearUpdatedError.....");
     prev_err_ = FST_SUCCESS;
}
