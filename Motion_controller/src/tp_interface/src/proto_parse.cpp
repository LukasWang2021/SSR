/**
 * @file proto_parse.cpp
 * @brief: use the new moveJ and moveL proto 
 * @author Wang Wei
 * @version 2.0.2
 * @date 2017-3-8
 */
#include "proto_parse.h"
#include "motionSL.pb.h"
#include "base_types.pb.h"
#include "common.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "rt_timer.h"
#include "sub_functions.h"


ProtoParse::ProtoParse(RosBasic *ros_basic):ros_basic_(ros_basic)
{  
	string str_addr = getLocalIP();

	hash_size_ = get_hash_size();
	NNSockType type = NN_SOCK_WS;
	nn_socket_ = new NNSocket(str_addr, type, type, COMMAND_PORT, STATE_PORT);
	FST_ASSERT(nn_socket_);	
	
	json_parse_ = new JsonParse(FILE_API_PATH);
	FST_ASSERT(json_parse_);

    removeAllPubParams();    

    vector<U64> err_list;
    robot_motion_.initial(err_list);
    for (vector<U64>::iterator it = err_list.begin(); it != err_list.end(); it++)
    {
        if(*it != TPI_SUCCESS)
        {
            storeErrorCode(*it);
            if (*it == CURRENT_JOINT_OUT_OF_CONSTRAINT)
            {
               robot_motion_.setRunningMode(SOFTLIMITED_R); 
            }
        }
    }
    //=========use the same IOInterface=========
    json_parse_->io_interface_ = robot_motion_.getIOInterfacrPtr(); 
}
 
ProtoParse::~ProtoParse()
{
    if (nn_socket_ != NULL)
    {
	    delete nn_socket_;
    }
    if (json_parse_ != NULL)
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

JsonParse* ProtoParse::getJsonParserPtr()
{
    return json_parse_;
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
	


void ProtoParse::updateParams()
{
    U64 result;
    if (robot_motion_.isProgramStateChanged())
    {
        setUpdateFlagByID(LOGIC_PROGRAMSTATE_ID, true);
    }
    if ((result = robot_motion_.updateJoints()) == TPI_SUCCESS) //get current joints from share memory
    {        
        if (robot_motion_.isJointsChanged())
        {
            setUpdateFlagByID(ACTUAL_JOINT_POS_ID, true);
            if ((result = robot_motion_.updatePose()) == TPI_SUCCESS)
            {
                setUpdateFlagByID(FK_TOOL_COORD_ID, true);
            }
            else
            {
               // if (!robot_motion_.isUpdatedSameError(result))
                {
                    storeErrorCode(result);
                }
            }
            if ((result = robot_motion_.updateFlangePose()) == TPI_SUCCESS)
            {
                setUpdateFlagByID(FK_FLGE_COORD_ID, true);
            }
            else
            {
                storeErrorCode(result);
            }
#ifdef SIMMULATION
            // ===update joints and publish to rviz===================
            Joint cur_joints = robot_motion_.getCurJointsValue();
            ros_basic_->pubJointState(cur_joints);
#endif
        }//end if (robot_motion_.isJointsChanged())         
    }//end if ((result = robot_motion_.updateJoints()) == TPI_SUCCESS)
    else
    {
        //if (!robot_motion_.isUpdatedSameError(result))                
        {
            storeErrorCode(result);
        }
    }
    if (robot_motion_.updateLogicMode())
    {
        setUpdateFlagByID(LOGIC_MODE_ID, true);
        if (result != TPI_SUCCESS)
        {
            //if (!robot_motion_.isUpdatedSameError(result))                
            {
                storeErrorCode(result);
            }
        }
    }
    if (robot_motion_.updateLogicState()) 
    {
        setUpdateFlagByID(LOGIC_STATE_ID, true);
        if (result != TPI_SUCCESS)
        {
            //if (!robot_motion_.isUpdatedSameError(result))                
            {
                storeErrorCode(result);
            }
        }
    }
    if (robot_motion_.updateInstructionSize())
    {
        setUpdateFlagByID(INSTRUCTION_NUM_ID, true);
    }
    if ((result = robot_motion_.updateSafetyStatus()) != TPI_SUCCESS)
    {
        //if (!robot_motion_.isUpdatedSameError(result))                
        {
            storeErrorCode(result);
        }
    }
  
    if (!error_list_.empty())
    {
        /*U64 err;*/
        /*FST_INFO("err:%llx",err);*/        
        setUpdateFlagByID(ERROR_WARNINGS_ID, true);
    }
    else
    {
        
    }

}


/**
 * @brief publish the parameter list
 */
void ProtoParse::pubParamList()
{
    string pub_str;
    motion_spec_SignalGroup sig_group;
    
    motion_spec_Signal sig;
    int cnt = 0;
    //FST_INFO("pubsize:%d",param_pub_map_.size());
	map<int, PublishUpdate>::iterator it = param_pub_map_.begin();
	for (; it != param_pub_map_.end(); ++it)
	{
		sig.id = it->first;
		PublishUpdate pub_update = it->second;
        //FST_INFO("id:%d, count:%d", sig.id, pub_update.count);
        if (pub_update.count++ >= pub_update.basic_freq)
		{
            if ((pub_update.count >= pub_update.max_freq)
            || (pub_update.update_flag == true))
            {
                pub_update.count = 0;
                pub_str = getParamBytes(sig.id);
                if ((sig.param.size = pub_str.size()) > 0)
                {
                    memcpy(sig.param.bytes, pub_str.c_str(), pub_str.size());
                    sig_group.sig_param[cnt++] = sig;
                }
                pub_update.update_flag = false;
            }
        }// if (pub_update.count++ >= pub_update.basic_freq)
        it->second = pub_update;
	}// for (; it != param_pub_map_.end(); ++it)
    //====================================
    sig_group.sig_param_count = cnt;
    if (cnt > 0)
    {
        for (it = param_pub_map_.begin(); it != param_pub_map_.end(); ++it)
        {
            it->second.count = 0;
        }
        pubGroupMsg(sig_group);
    }// if (cnt > 0)

    //===add plot msg==================
    cnt = 0;
    boost::mutex::scoped_lock lock(mutex_);
    map<int, PlotPub>::iterator itr = plot_pub_.begin();
    for(;itr != plot_pub_.end(); ++itr)
    {  
        if (itr->second.count-- <= 0)
        {
            itr->second.count = itr->second.frq_devider / INTERVAL_PROPERTY_UPDATE; 
            vector<uint32_t> vc_ids = itr->second.param_ids;
            vector<uint32_t>::iterator iter = vc_ids.begin();
            for (; iter != vc_ids.end(); ++iter)
            {                
                int id = *iter;
                //FST_INFO("the id is :%d", id);                
                if (id == PLOT_FIFO1_ID)
                {
                    if (robot_motion_.fifo1_.count > 0)
                    {
                        sig.id = id;
                        int fifo_cnt =  robot_motion_.fifo1_.count;
                        robot_motion_.fifo1_.count = 0;
                        sig.param.size = fifo_cnt * sizeof(PoseEuler);  
                        {
                            readLock rLock(robot_motion_.fifo1_.rwmux);
                            memcpy(sig.param.bytes, (char*)robot_motion_.fifo1_.data, sig.param.size);
                        }
                        sig_group.sig_param[cnt++] = sig;                     
                    }
                    
                }
                else if (id == PLOT_FIFO2_ID)
                {
                    writeLock wlock(robot_motion_.fifo2_.rwmux);
                    int fifo_cnt =  robot_motion_.fifo2_.count;
                    if (fifo_cnt > 0)
                    { 
                        sig.id = id;
                        
                       // FST_INFO("pub total count:%d", robot_motion_.dbcount);
                        //FST_INFO("pub %d fifo2...", fifo_cnt);
                        robot_motion_.fifo2_.count = 0;
                        sig.param.size = fifo_cnt * sizeof(double) * MAX_JOINTS;
                        //FST_INFO("size:%d", sig.param.size);
                        {
                            //readLock rLock(robot_motion_.fifo2_.rwmux);
                            memcpy(sig.param.bytes, (char*)robot_motion_.fifo2_.data, sig.param.size);
                           // printDbLine("firo2:", (double*)(sig.param.bytes+1200), 24);
                        }
                        sig_group.sig_param[cnt++] = sig; 
                    }
                }
            }// for (; iter != vc_ids.end(); ++iter)
        }// if (itr->second.count-- <= 0)
    }// for(;itr != plot.end(); ++itr)
    sig_group.sig_param_count = cnt;
    if (cnt > 0)
    {
        //FST_INFO("publish %d signal", cnt);
        pubGroupMsg(sig_group);
    }// if (cnt > 0)

}

void ProtoParse::storeErrorCode(U64 err_code)
{
    if (robot_motion_.isUpdatedSameError(err_code))
    {
        return;
    }
    if (err_code & 0x0001000000000000)
    {
        FST_ERROR("err_code:%llx",err_code);        
        int warning_level = (err_code & 0x0000FFFF00000000) >> 32;
        
        robot_motion_.errorAction(warning_level);
        
        error_list_.push_back(err_code);
        robot_motion_.backupError(err_code);
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
	bool ret;
    U64 result;

	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;
	BaseTypes_ParameterSetMsg param_set_msg;
	PARSE_FIELD(param_set_msg, BaseTypes_ParameterSetMsg, field_buffer, field_size, ret);
	if (ret == false)
	{
		FST_ERROR("failed to parse param_set_msg");		
        return retStatus(BaseTypes_StatusCode_FAILED);  //need to define this error
	}

    if (param_set_msg.has_id)
    {
        msg_id = param_set_msg.id;
        if (msg_id >= IO_BASE_ADDRESS)
        {
            result = robot_motion_.getIOInterfacrPtr()->setDO(msg_id, param_set_msg.param.bytes[0]);
            if (result != TPI_SUCCESS)
                status_code = BaseTypes_StatusCode_FAILED;
            return retStatus(status_code);
        }
    }
    else if (param_set_msg.has_path)
    {
        if (json_parse_->getIDFromPath(param_set_msg.path, msg_id) == false)
        {
            FST_INFO("set path:%s", param_set_msg.path);
            result = robot_motion_.getIOInterfacrPtr()->setDO(param_set_msg.path, param_set_msg.param.bytes[0]);            
            if (result != TPI_SUCCESS)
            {
                FST_ERROR("set dio failed:%llx", result);
                status_code = BaseTypes_StatusCode_FAILED;
            }
            return retStatus(status_code);
        }//end if (json_parse_->getIDFromPath(param_set_msg.path, msg_id) == false)
    }//end else if (param_set_msg.has_path)

	FST_INFO("set_param_msg:%s, id:%d ,has_path:%d, has_id:%d, param_id:%d",\
			param_set_msg.path, msg_id, param_set_msg.has_path, param_set_msg.has_id, param_set_msg.id);
	//===parse motion program================= 
	switch (msg_id)
	{
		case MODE_COMMAND_ID:
		{
			RobotModeCmd mode_cmd = *(RobotModeCmd*)param_set_msg.param.bytes;
			setProtoLogicMode(mode_cmd);	
			break;
		}			
		case STATE_COMMAND_ID:
		{
			RobotStateCmd state_cmd = *(RobotStateCmd*)param_set_msg.param.bytes;
			setProtoLogicState(state_cmd);
			break;
		}
		case HOSTIN_JOINT_TRAJ_ID:
		{
			setProtoJiontTraj(param_set_msg.param.bytes, param_set_msg.param.size);
			break;
		}
		case HOSTIN_TOOL_COORD_ID:
		{
			setProtoToolCoord(param_set_msg.param.bytes, param_set_msg.param.size);
			break;
		}
        case CTL_COMMAND_ID:
        {
            U64 result = TPI_SUCCESS;
            RobotCtrlCmd cmd = *(RobotCtrlCmd*)param_set_msg.param.bytes;
            switch (cmd)
            {
                case SHUTDOWN_CMD:
                    result = robot_motion_.actionShutdown();
                    break;
                case PAUSE_CMD:         
                    if (robot_motion_.getLogicMode() != AUTO_RUN_M)
                    {
                        result = INVALID_ACTION_IN_CURRENT_MODE;
                        FST_ERROR("INVALID_ACTION_IN_CURRENT_MODE");
                        break;
                    }
                    if (robot_motion_.getProgramState() == EXECUTE_R)
                    {
                        result = robot_motion_.setProgramStateCmd(GOTO_PAUSED_E);
                    }
                    else
                    {
                        result = INVALID_ACTION_IN_CURRENT_STATE;
                    }
                    break;
                case CONTINUE_CMD:
                    if ((robot_motion_.getLogicMode() == AUTO_RUN_M)
                    && (robot_motion_.getProgramState() == PAUSED_R))
                    {
                        result = robot_motion_.setProgramStateCmd(GOTO_EXECUTE_E);
                    }
                    else
                    {
                        result = INVALID_ACTION_IN_CURRENT_STATE;
                    }
                    break;
                case ABORT_CMD:
                    if (robot_motion_.getProgramState() == PAUSED_R)
                    {
                        result = robot_motion_.setProgramStateCmd(GOTO_IDLE_E);
                    }
                    else
                    {
                        result = INVALID_ACTION_IN_CURRENT_STATE;
                    }
                    break;
                case CALIBRATE_CMD:
                    if (robot_motion_.getLogicState() != ESTOP_S)
                    {
                        result = INVALID_ACTION_IN_CURRENT_STATE;
                    }
                    else
                    {
                        result = robot_motion_.actionCalibrate();
                    }
                    break;
                case SETTMPZERO_CMD:
                    if (robot_motion_.getLogicState() != ESTOP_S)
                    {
                        result = INVALID_ACTION_IN_CURRENT_STATE;
                    }
                    else
                    {
                        result = robot_motion_.setTempZero();
                    }
                    break;
                default:
                    status_code = BaseTypes_StatusCode_FAILED;
                    break;
            }//end switch (cmd)
            if (result != TPI_SUCCESS)
            {
                storeErrorCode(result);
                status_code = BaseTypes_StatusCode_FAILED;
            }
            break;
        }//end case CTL_COMMAND_ID:
        case ID_PREVIOUS_COMMAND_ID:
        {
            int prev_id = *(int*)param_set_msg.param.bytes;
            robot_motion_.setPreviousCmdID(prev_id);
            break;
        }
        case MOTION_PROGRAM_ID:
        {
            if (robot_motion_.getLogicState() != ENGAGED_S)
            {
                FST_ERROR("error:wrong state");
                storeErrorCode(INVALID_ACTION_IN_CURRENT_STATE);
                status_code = BaseTypes_StatusCode_FAILED;
            }
            //check if current mode id auto run
            else if (robot_motion_.getLogicMode() != AUTO_RUN_M)
            {
                FST_ERROR("error:wrong mode");
                storeErrorCode(INVALID_ACTION_IN_CURRENT_MODE);
                status_code = BaseTypes_StatusCode_FAILED;				
            }
            else if (robot_motion_.getRunningMode() != NORMAL_R)
            {
                FST_ERROR("error:limited running mode:%d", robot_motion_.getRunningMode());
                storeErrorCode(INVALID_ACTION_IN_LIMITED_STATE);
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
            break;
        }//end case MOTION_PROGRAM_ID:
        case TOOLFRAME_ID:
        {
            if (parseToolFrame(param_set_msg.param.bytes, param_set_msg.param.size) != TPI_SUCCESS)
            {
                    storeErrorCode(DECODE_MESSAGE_FAILED);
                    status_code = BaseTypes_StatusCode_FAILED;
            }
            break;
        }
        case USERFRAME_ID:
        {
            if (parseUserFrame(param_set_msg.param.bytes, param_set_msg.param.size) != TPI_SUCCESS)
            {
                    storeErrorCode(DECODE_MESSAGE_FAILED);
                    status_code = BaseTypes_StatusCode_FAILED;
            }
            break;
        }
        case LOGIC_CURVEMODE_ID:
        {
            int curve_mode = *(int*)param_set_msg.param.bytes;
            robot_motion_.setCurveMode(curve_mode);
            break;
        }
        case PARAMS_LOCALTIME_ID:
        {
            long time = *(long*)param_set_msg.param.bytes;
            setTimeSecond(time);
            break;
        }
        case PARAMS_SOFTLIMIT_ID:
        {
            if (parseSoftConstraint(param_set_msg.param.bytes, param_set_msg.param.size) != TPI_SUCCESS)
            {
                    storeErrorCode(DECODE_MESSAGE_FAILED);
                    status_code = BaseTypes_StatusCode_FAILED;
            } 
            break;
        }
        case PARAMS_DH_ID:
        {
            if (parseDHGroup(param_set_msg.param.bytes, param_set_msg.param.size) != TPI_SUCCESS)
            {
                    storeErrorCode(DECODE_MESSAGE_FAILED);
                    status_code = BaseTypes_StatusCode_FAILED;
            } 
            break;
        }
        case CTL_GLOBAL_VELOCITY_ID:
        {
            if (robot_motion_.getLogicMode() != AUTO_RUN_M)
            {
                storeErrorCode(INVALID_ACTION_IN_CURRENT_MODE);
                status_code = BaseTypes_StatusCode_FAILED;
                break;
            }
            double factor = *(double*)param_set_msg.param.bytes;
            robot_motion_.setGlobalVelocity(factor);
            break;
        }
		default:
		{
			//no id exist
            FST_ERROR("Invalid path:%s", param_set_msg.path);
            status_code = BaseTypes_StatusCode_FAILED;
			break;
		}//end default        
	}//end 	switch (msg_id)

    retStatus(status_code);
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
    int buf_len = 0;
	//BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;

	BaseTypes_ParameterCmdMsg param_cmd_msg;		
	PARSE_FIELD(param_cmd_msg,BaseTypes_ParameterCmdMsg, field_buffer, field_size, result);
	if(result == false)
	{
		FST_ERROR("parse param_cmd_msg failed");
        return retStatus(BaseTypes_StatusCode_FAILED);
    }

	if (param_cmd_msg.has_id)
    {
        msg_id = param_cmd_msg.id;
        if (msg_id >= IO_BASE_ADDRESS)
        {
            result = robot_motion_.getIOInterfacrPtr()->checkIO(msg_id, buf_len);
            if (result != TPI_SUCCESS)
            {   
                storeErrorCode(result);
                return retStatus(BaseTypes_StatusCode_FAILED);
            }
        }
    }
    else if (param_cmd_msg.has_path)
    {
        if (json_parse_->getIDFromPath(param_cmd_msg.path, msg_id) == false)
        {
            result = robot_motion_.getIOInterfacrPtr()->checkIO(param_cmd_msg.path, buf_len, msg_id);
            if (result != TPI_SUCCESS)
            {   
                storeErrorCode(result);
                return retStatus(BaseTypes_StatusCode_FAILED);
            }
        }//end if (json_parse_->getIDFromPath(param_cmd_msg.path, msg_id) == false)
    }//end else if (param_cmd_msg.has_path)

    FST_INFO("path:%s,param_id:%d", param_cmd_msg.path, msg_id);
    
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
            FST_INFO("add:freq:%d, min_freq:%d",param_cmd_msg.update_frq_devider,param_cmd_msg.minimum_frq_devider);
			addPubParameter(msg_id, param_cmd_msg.update_frq_devider, param_cmd_msg.minimum_frq_devider, buf_len);
            ParamProperty* param_property = json_parse_->getParamProperty(msg_id);
            BaseTypes_ParamInfo* param_info = json_parse_->getParamInfo(param_cmd_msg.path, msg_id, param_property);
            retStatus(param_info, BaseTypes_StatusCode_OK);
			break;
		}
		case BaseTypes_CommandType_REMOVE:
		{
            FST_INFO("remove...");
			removePubParameter(msg_id);
            retStatus(BaseTypes_StatusCode_OK);
			break;
		} 
		case BaseTypes_CommandType_REMOVE_ALL:
		{
            FST_INFO("remove all...");
			removeAllPubParams();
            retStatus(BaseTypes_StatusCode_OK);
			break;
		} 
		default:
        {
			FST_ERROR("ERROR command:%d", param_cmd_msg.cmd);
            storeErrorCode(INVALID_PARAM_FROM_TP);
            retStatus(BaseTypes_StatusCode_FAILED);
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
	U64 result;

	BaseTypes_ParameterGetMsg param_get_msg;
	PARSE_FIELD(param_get_msg, BaseTypes_ParameterGetMsg, field_buffer, field_size, ret);
	if(ret == false)
	{
		FST_ERROR("parse param_get_msg failed");
        return retStatus(BaseTypes_StatusCode_FAILED);
    }
    if (param_get_msg.has_id)
    {
        msg_id = param_get_msg.id;
        if (msg_id >= IO_BASE_ADDRESS)
        {
            BaseTypes_ParameterMsg param_msg;
            int io_bytes_len;
            result = robot_motion_.getIOInterfacrPtr()->getDIO(msg_id, \
                param_msg.param.bytes, sizeof(param_msg.param.bytes), io_bytes_len);
            if (result != TPI_SUCCESS)
            {
                storeErrorCode(result);
                return retStatus(BaseTypes_StatusCode_FAILED);
            }
            param_msg.param.size = io_bytes_len;
            retIOMsg(&param_msg);
        }//end if (msg_id >= IO_BASE_ADDRESS)
    }//end if (param_get_msg.has_id)
    else if (param_get_msg.has_path)
    {
        if (json_parse_->getIDFromPath(param_get_msg.path, msg_id) == false)
        {                                
            BaseTypes_ParameterMsg param_msg;                                        
            int io_bytes_len;
            result = robot_motion_.getIOInterfacrPtr()->getDIO(param_get_msg.path,\
                (unsigned char*)param_msg.param.bytes, sizeof(param_msg.param.bytes), io_bytes_len);            
            if (result != TPI_SUCCESS)
            {
                storeErrorCode(result);
                return retStatus(BaseTypes_StatusCode_FAILED);
            }
            param_msg.param.size = io_bytes_len;
            retIOMsg(&param_msg);
            return;
        }//end if (json_parse_->getIDFromPath(param_get_msg.path, msg_id) == false) 
    }//end else if (param_set_msg.has_path)


//	FST_INFO("get_param_msg:%s, id:%d, param_id:%d", param_get_msg.path, msg_id, param_get_msg.id);
    ParamProperty* param_property = json_parse_->getParamProperty(msg_id);
    BaseTypes_ParamInfo* param_info = json_parse_->getParamInfo(param_get_msg.path, msg_id, param_property);

   // string str_param = getParamBytes(msg_id);
   // retParamMsg(param_info, str_param.c_str());
    switch(msg_id)
    {
        case SAFETY_INPUT_FRAME1_ID:
        {
            U32 frm_data = robot_motion_.getSafetyInterfacePtr()->getDIFrm1();
            retParamMsg(param_info, &frm_data);
            break;
        }
        case SAFETY_INPUT_FRAME2_ID:
        {
            U32 frm_data = robot_motion_.getSafetyInterfacePtr()->getDIFrm2();
            retParamMsg(param_info, &frm_data);
            break;
        }
        case INSTRUCTION_NUM_ID:
        {
            int num = robot_motion_.getInstructionListSize();
            retParamMsg(param_info, &num);
            break;
        }
        case ID_PREVIOUS_COMMAND_ID:
        {
            int prev_id = robot_motion_.getPreviousCmdID();
            retParamMsg(param_info, &prev_id);
            break;
        }
        case ID_CURRENT_COMMAND_ID:
        {
            int cur_id = robot_motion_.getCurrentCmdID();
            retParamMsg(param_info, &cur_id);
            break;
        }
        case ETHERCAT_SMNT_DIN3_ID:
        {
            uint8_t io = robot_motion_.getSafetyInterfacePtr()->getDITPManual();
            retParamMsg(param_info, &io);
            break;
        }
        case ETHERCAT_SMNT_DIN4_ID:
        {
            uint8_t io = robot_motion_.getSafetyInterfacePtr()->getDITPAuto();
            retParamMsg(param_info, &io);
            break;
        }
        case LOGIC_PROGRAMSTATE_ID:
        {
            ProgramState prgm_state = robot_motion_.getProgramState();
            retParamMsg(param_info, &prgm_state);
            break;
        }
        case LOGIC_STATE_ID:		
        {
            RobotState state = robot_motion_.getLogicState();
            retParamMsg(param_info, &state);
            break;
        }
        case LOGIC_MODE_ID:
        {
            RobotMode mode = robot_motion_.getLogicMode();
            retParamMsg(param_info, &mode);
            break;
        }
        case ACTUAL_JOINT_POS_ID:
        {
            Joint jnts = robot_motion_.getCurJointsValue();
            retParamMsg(param_info, &jnts);
            break;
        }
        case FK_TOOL_COORD_ID:
        {
            PoseEuler pose = robot_motion_.getCurPosition();
            retParamMsg(param_info, &pose);
            break;
        }
        case FK_FLGE_COORD_ID:
        {
            PoseEuler pose = robot_motion_.getFlangePose();
            retParamMsg(param_info, &pose);
            break;
        }
        case ERROR_WARNINGS_ID:
        {
            /*U64 err[ERROR_NUM] = {0};*/
            //for (int i = 0; i < ERROR_NUM; ++i)
            //{
                //if (error_list_.empty())
                //{
                    //break;
                //}
                //err[i] = error_list_.front();
                ////error_list_.pop_front();
            //}
            /*retParamMsg(param_info, err);*/
            break;
        }
        case CONFIG_IO_INFO_ID:
        {
            retDeviceInfo(param_info);
            break;
        }
        case CALCU_FK_ID:
        {
            if ((param_get_msg.has_param == false)
            ||(param_get_msg.param.size == 0))
            {
                storeErrorCode(INVALID_PARAM_FROM_TP);
                retStatus(BaseTypes_StatusCode_FAILED);
                break;
            }
            else
            {
                Joint joints;
                
                memcpy((void*)&joints, param_get_msg.param.bytes, param_get_msg.param.size);
                PoseEuler pose;
                robot_motion_.getPoseFromJoint(joints, pose);
                retParamMsg(param_info, &pose);         
            }
            break;
        }
        case CALCU_IK_ID:
        {
            if ((param_get_msg.has_param == false)
            ||(param_get_msg.param.size == 0))
            {
                storeErrorCode(INVALID_PARAM_FROM_TP);
                retStatus(BaseTypes_StatusCode_FAILED);
                break;
            }
            else
            {
                PoseEuler pose;
                memcpy((void*)&pose, param_get_msg.param.bytes, param_get_msg.param.size);
                Joint joints;
                robot_motion_.getJointFromPose(pose, joints);
                retParamMsg(param_info, &joints);
            }
            break;
        }
        case TOOLFRAME_ID:
        {            
            retToolFrame(param_info);
            break;
        }
        case USERFRAME_ID:
        {
            retUserFrame(param_info);
            break;
        }
        case LOGIC_CURVEMODE_ID:
        {
            int curve_mode = robot_motion_.getCurveMode();
            retParamMsg(param_info, &curve_mode);
            break;
        }
        case PARAMS_LOCALTIME_ID:
        {
            long time = getCurTimeSecond();
            retParamMsg(param_info, &time);
            break;
        }
        case PARAMS_SOFTLIMIT_ID:
        {
            retSoftConstrait(param_info); 
            break;
        }
        case PARAMS_HW_LIMIT_ID:
        {
            retHardConstrait(param_info); 
            break;
        }
        case PARAMS_DH_ID:
        {
            retDHParams(param_info);
            break;
        }
        case CTL_GLOBAL_VELOCITY_ID:
        {
            double factor = robot_motion_.getGlobalVelocity();
            retParamMsg(param_info, &factor);
            break;
        }
        default:
            storeErrorCode(INVALID_PARAM_FROM_TP);
            retStatus(BaseTypes_StatusCode_FAILED);
            break;
    }

}

void ProtoParse::parseParamOverwriteMsg(const uint8_t *field_buffer, int field_size)
{
	bool ret;

	BaseTypes_ParameterOverwriteMsg param_ovwrt_msg;
	PARSE_FIELD(param_ovwrt_msg, BaseTypes_ParameterOverwriteMsg, field_buffer, field_size, ret);
    if (param_ovwrt_msg.has_path == false)
    {
        return retStatus(BaseTypes_StatusCode_FAILED);
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
    else if (HASH_CMP(CreatePlotMsg, buffer) == true)
    {
        parseCreatePlotMsg(field_buffer, field_size); 
    }
    else if (HASH_CMP(RemovePlotMsg, buffer) == true)
    {
        parseRemovePlotMsg(field_buffer, field_size);
    }
    else
	{
        storeErrorCode(INVALID_PARAM_FROM_TP);
        retStatus(BaseTypes_StatusCode_FAILED);
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
	int length=3;
	SET_FIELD(param_list_msg, BaseTypes_ParameterListMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

    int bytes = nn_socket_->nnSocketReply(buffer, length);
	FST_INFO("ret param list size:%d",bytes);

	return ret;
}


/**
 * @brief: return status to TP
 *
 * @param: param_info: input
 * @param: status_code: input==>status code
 *
 * @return: true if success
 */
void ProtoParse::retStatus(BaseTypes_ParamInfo *param_info, BaseTypes_StatusCode status_code)
{
	bool ret;
	BaseTypes_StatusMsg status_msg;
	status_msg.status = status_code;


	if(param_info == NULL)
	{
		status_msg.has_info = false;
	}
	else
	{
		status_msg.has_info = true;
        status_msg.info = *param_info;
	}

	uint8_t *buffer = nn_socket_->getReplyBufPtr();
	int length;
	SET_FIELD(status_msg, BaseTypes_StatusMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

//	FST_INFO("len:%d",length);
	nn_socket_->nnSocketReply(buffer, length);

}

void ProtoParse::retStatus(BaseTypes_StatusCode status_code)
{
	bool ret;
	BaseTypes_StatusMsg status_msg;
	status_msg.status = status_code;
	status_msg.has_info = false;

	uint8_t *buffer = nn_socket_->getReplyBufPtr();
	int length;
	SET_FIELD(status_msg, BaseTypes_StatusMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

//	FST_INFO("len:%d",length);
	nn_socket_->nnSocketReply(buffer, length);
}


template<typename T> 
void ProtoParse::retParamMsg(BaseTypes_ParamInfo* param_info, T params)
{
    bool ret;
	BaseTypes_ParameterMsg param_msg;
    if (param_info == NULL)
    {
        param_msg.has_info = false;
        memcpy(param_msg.param.bytes, (char*)params, sizeof(params));
		param_msg.param.size = sizeof(params);
    }
    else
    {
        param_msg.has_info = true;
        param_msg.info = *param_info;
		int into_bytes = param_info->data_size*param_info->number_of_elements;
		memcpy(param_msg.param.bytes, (char*)params, into_bytes);
		param_msg.param.size = into_bytes;
    }
    uint8_t *buffer = nn_socket_->getReplyBufPtr();

    int length;
	SET_FIELD(param_msg, BaseTypes_ParameterMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

    nn_socket_->nnSocketReply(buffer, length);
}

void ProtoParse::retIOMsg(BaseTypes_ParameterMsg *param_msg)
{
    bool ret;
    //param_msg->has_info = false;
    uint8_t *buffer = nn_socket_->getReplyBufPtr();
    param_msg->has_info = true;
    param_msg->info.overwrite_active = 0; //not active
    param_msg->info.data_type = 2;  // uint8
    param_msg->info.data_size = 1;  // 1 byte
    param_msg->info.number_of_elements = 1;  
    param_msg->info.param_type = BaseTypes_ParamType_INPUT_SIGNAL;  //input
    param_msg->info.permission = BaseTypes_Permission_permission_undefined;
    param_msg->info.user_level = BaseTypes_UserLevel_user_level_undefined;
    param_msg->info.unit = BaseTypes_Unit_unit_undefined;

    int length;
	SET_FIELD(*param_msg, BaseTypes_ParameterMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

    nn_socket_->nnSocketReply(buffer, length);
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
template<typename T>
bool ProtoParse::pubParamMsg(int id, T params, int len)
{
	bool ret;

	BaseTypes_ParameterMsg param_msg;
	param_msg.has_info = false;
//	FST_INFO("path:%s, %d", info.path, id);

	param_msg.has_header =true;
	param_msg.header.frameCounter = 0;
	param_msg.header.timestamp = 0;
	
	memcpy(param_msg.param.bytes, (char*)params, len);
	param_msg.param.size = len;

	int id_len = sizeof(int);
	uint8_t *buffer = nn_socket_->getPublishBufPtr();
	memcpy(buffer, (char*)&id, id_len);

	pb_ostream_t stream = {0};	
	stream = pb_ostream_from_buffer(buffer+id_len, MAX_BUFFER_SIZE-hash_size_);
	ret = pb_encode(&stream, BaseTypes_ParameterMsg_fields, &param_msg);
	int length = stream.bytes_written+id_len;
	//SET_FIELD(param_msg, BaseTypes_ParameterMsg, publish_buffer_, sizeof(publish_buffer_), hash_size_, state_bytes_written_, ret);

	nn_socket_->nnSocketPublish(buffer, length);

	return ret;
}

void ProtoParse::pubGroupMsg(motion_spec_SignalGroup sig_group)
{
    pb_ostream_t stream = {0};

    uint8_t *buffer = nn_socket_->getPublishBufPtr();
    stream = pb_ostream_from_buffer(buffer, MAX_BUFFER_SIZE);
	bool ret = pb_encode(&stream, motion_spec_SignalGroup_fields, &sig_group);	
    if (ret != true)
    {
        FST_INFO("encode failed");
    }

	int length = stream.bytes_written;

	nn_socket_->nnSocketPublish(buffer, length);
}
/**
 * @brief: publish a parameter by the id
 *
 * @param id:input==>the uniq id of the parameter
 */
string ProtoParse::getParamBytes(int id)
{	
    string param_bytes;
	switch (id)
	{
        case LOGIC_PROGRAMSTATE_ID:
        {
            ProgramState prgm_state = robot_motion_.getProgramState();
            param_bytes = string((char*)&prgm_state, sizeof(prgm_state));
            break;
        }
		case LOGIC_STATE_ID:		
		{
			RobotState state = robot_motion_.getLogicState();
            param_bytes = string((char*)&state, sizeof(state));
			break;
		}
		case LOGIC_MODE_ID:
		{
			RobotMode mode = robot_motion_.getLogicMode();
            param_bytes = string((char*)&mode, sizeof(mode));
			break;
		}
		case ACTUAL_JOINT_POS_ID:
		{
			Joint jnt_val = robot_motion_.getCurJointsValue();
            param_bytes = string((char*)&jnt_val, sizeof(jnt_val));
			break;
		}
		case FK_TOOL_COORD_ID:
		{
            PoseEuler pose = robot_motion_.getCurPosition();
            param_bytes = string((char*)&pose, sizeof(pose));
			break;
		}
        case FK_FLGE_COORD_ID:
		{
            PoseEuler pose = robot_motion_.getFlangePose();
            param_bytes = string((char*)&pose, sizeof(pose));
			break;
		}
        case ID_PREVIOUS_COMMAND_ID:
        {
            int prev_id = robot_motion_.getPreviousCmdID();
            param_bytes = string((char*)&prev_id, sizeof(prev_id));
            break;
        }
        case ID_CURRENT_COMMAND_ID:
        {
            int cur_id = robot_motion_.getCurrentCmdID();
            param_bytes = string((char*)&cur_id, sizeof(cur_id));
            break;
        }
        case INSTRUCTION_NUM_ID:
        {
            int num = robot_motion_.getInstructionListSize();
            param_bytes = string((char*)&num, sizeof(num));
        }
        case ERROR_WARNINGS_ID:
        {
            U64 err[ERROR_NUM] = {0};
            int i;
            if (error_list_.empty())
            {
                param_bytes.resize(0);
                break;
            }
            
            for (i = 0; i < error_list_.size(); ++i)
            {
                err[i] = error_list_.front();
                error_list_.pop_front();
            }
            param_bytes = string((char*)err, ERROR_NUM*sizeof(U64));
            //FST_INFO("param_bytes size:%d", param_bytes.size());
            break;
        }
        case SAFETY_INPUT_FRAME1_ID:
        {
            U32 frm_data = robot_motion_.getSafetyInterfacePtr()->getDIFrm1();
            param_bytes = string((char*)&frm_data, sizeof(frm_data));
            break;
        }
        case SAFETY_INPUT_FRAME2_ID:
        {
            U32 frm_data = robot_motion_.getSafetyInterfacePtr()->getDIFrm2();
            param_bytes = string((char*)&frm_data, sizeof(frm_data));
            break;
        }
		default:
        {
            if (id >= IO_BASE_ADDRESS)
            {
                //FST_INFO("pub id:%d", id);
                map<int, PublishUpdate>::iterator it = param_pub_map_.find(id);
                int bytes;
                robot_motion_.getIOInterfacrPtr()->getDIO(id, it->second.buffer, it->second.buf_len, bytes);
                param_bytes = string((char*)it->second.buffer, bytes);
            }
			break;
        }
	}	
    return param_bytes;
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
	for (unsigned int j = 0; j < motion_program.commandlist_count; ++j)
	{	 		
        CommandInstruction cmd_instruction;
        if (parseMotionCommand(motion_program.commandlist[j], cmd_instruction) == false)
        { 
            FST_ERROR("error parse motion command");
		    continue; //continue to parse next command
        }
        
		robot_motion_.addMotionInstruction(cmd_instruction);  // add to the queue of motion instructions
	}
    U64 result = robot_motion_.checkAutoStartState(); 
    if (result != TPI_SUCCESS)
    {
        storeErrorCode(result);
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
	bool ret;	

    cmd_instruction.pick_status = FRESH;
    cmd_instruction.smoothDistance = 0; //0 is default
	cmd_instruction.id = motion_command.id;
	cmd_instruction.commandtype = motion_command.commandtype;
    cmd_instruction.count = 0;

	switch (motion_command.commandtype)
	{
		case motion_spec_MOTIONTYPE_JOINTMOTION:
        {
			motion_spec_MoveJ moveJ;
			PARSE_FIELD(moveJ, motion_spec_MoveJ,\
					motion_command.commandarguments.bytes,\
					motion_command.commandarguments.size, ret);
			if (ret == false)
			{
				//FST_ERROR("error parse motion command");
				return false;
			}
			FST_INFO("MoveJ:vmax:%f, amax:%f,has_smth:%d,smth:%f",moveJ.vMax,moveJ.aMax,moveJ.has_smoothPercent,moveJ.smoothPercent);
            if (moveJ.has_aMax == false)
                moveJ.aMax = DEFAULT_ACC;
			if (moveJ.has_smoothPercent) 
				cmd_instruction.smoothDistance = moveJ.smoothPercent;
			else
				cmd_instruction.smoothDistance = -1;
			cmd_instruction.command_arguments = string((const char*)&moveJ, sizeof(moveJ));
			break;
        }//end case motion_spec_MOTIONTYPE_JOINTMOTION:
		case motion_spec_MOTIONTYPE_CARTMOTION:
        {
			motion_spec_MoveL moveL;
			PARSE_FIELD(moveL, motion_spec_MoveL,\
					motion_command.commandarguments.bytes,\
					motion_command.commandarguments.size, ret);
            if (ret == false)
			{
			//	FST_ERROR("error parse motion command");
				return false;
			}
			FST_INFO("MoveL:vmax:%f, amax:%f,has_smth:%d,smth:%f\n",moveL.vMax,moveL.aMax,moveL.waypoints[0].has_smoothPercent,moveL.waypoints[0].smoothPercent);
            if (moveL.has_aMax == false)
                moveL.aMax = DEFAULT_ACC;

			if (moveL.waypoints_count == 0)
			{
				return false; // no points
			}            
            else if (moveL.waypoints_count == 1)
			{
                motion_spec_WayPoint waypoint = moveL.waypoints[0];
				if (waypoint.has_smoothPercent) 
                {
                    cmd_instruction.smoothDistance = waypoint.smoothPercent;
                }
                else
                {
                    cmd_instruction.smoothDistance = -1;
                }
			}	
            
            cmd_instruction.command_arguments = string((const char*)&moveL, sizeof(moveL));
			break;
        }//end case motion_spec_MOTIONTYPE_CARTMOTION:
		case motion_spec_MOTIONTYPE_CIRCLEMOTION:
        {
            motion_spec_MoveC moveC;
            PARSE_FIELD(moveC, motion_spec_MoveC,\
					motion_command.commandarguments.bytes,\
					motion_command.commandarguments.size, ret);
            if (ret == false)
			{
			//	FST_ERROR("error parse motion command");
				return false;
			}
            FST_PRINT("MoveC:vmax:%f, amax:%f,has_smth:%d,smth:%f\n",moveC.vMax,moveC.aMax,moveC.has_smoothPercent,moveC.smoothPercent);
            printDbLine("pose1:", moveC.pose1.coordinates, 6);
            printDbLine("pose2:", moveC.pose2.coordinates, 6);
            if (moveC.has_aMax == false)
                moveC.aMax = DEFAULT_ACC;
			if (moveC.has_smoothPercent) 
				cmd_instruction.smoothDistance = moveC.smoothPercent;
			else
				cmd_instruction.smoothDistance = -1;
			cmd_instruction.command_arguments = string((const char*)&moveC, sizeof(moveC));
            break;
        }//end case motion_spec_MOTIONTYPE_CIRCLEMOTION:
		case motion_spec_MOTIONTYPE_WAIT:
        {
            motion_spec_Wait wait;
            PARSE_FIELD(wait, motion_spec_Wait,\
					motion_command.commandarguments.bytes,\
					motion_command.commandarguments.size, ret);
            if (ret == false)
			{
				FST_ERROR("error parse motion wait");
				return false;
			}
           
            //FST_INFO("wait has_timeout:%d, timeout:%f", wait.has_timeout, wait.timeout);
            if (((wait.has_path) && (!wait.has_value))
            || ((!wait.has_path) && (wait.has_value)))
                return false;
            if ((wait.has_timeout) && (wait.timeout > 0))
                 cmd_instruction.count = (wait.timeout+NON_MOVE_INTERVAL-1) / NON_MOVE_INTERVAL;
            //FST_INFO("wait count:%d", cmd_instruction.count);
            cmd_instruction.command_arguments = string((const char*)&wait, sizeof(wait));
			break;
        }//end case motion_spec_MOTIONTYPE_WAIT:
        case motion_spec_MOTIONTYPE_SET:
        {
            motion_spec_Set set;
            PARSE_FIELD(set, motion_spec_Set,\
					motion_command.commandarguments.bytes,\
					motion_command.commandarguments.size, ret);
            if (ret == false)
			{
			//	FST_ERROR("error parse motion command");
				return false;
			}
            if ((set.has_time_sec) && (set.time_sec > 0))
                cmd_instruction.count = (set.time_sec+NON_MOVE_INTERVAL-1) / NON_MOVE_INTERVAL;
            cmd_instruction.command_arguments = string((const char*)&set, sizeof(set));
			break;
        }//end case motion_spec_MOTIONTYPE_SET:
		default:
			break;
	}//end switch (motion_command.commandtype)

	return true;
}


template<typename T>
bool ProtoParse::checkPathAndID(T msg, uint32_t &id)
{
    if (msg.has_id)
    {
        id = msg.id;
        return true;
    }
    else if (msg.has_path)
    {
        return json_parse_->getIDFromPath(msg.path, id);
    }
	return false;
}
/**
 * @brief: set logic mode  
 *
 * @param id: input==>the parameter id
 * @param mode_cmd: input==>the mode command
 */
void ProtoParse::setProtoLogicMode(RobotModeCmd mode_cmd)
{
    U64 result = robot_motion_.setLogicModeCmd(mode_cmd);
	if (result != TPI_SUCCESS)
    {
        storeErrorCode(result);
    }
}
/**
 * @brief: set logic state 
 *
 * @param id: input==>the parameter id
 * @param state_cmd: input==>the state command
 */
void ProtoParse::setProtoLogicState(RobotStateCmd state_cmd)
{
    if (state_cmd == EMERGENCY_STOP_E)
    {
        robot_motion_.stateEStopAction();   //TP command 

    }
    U64 result = robot_motion_.setLogicStateCmd(state_cmd);
	if (result != TPI_SUCCESS)
    {
        storeErrorCode(result);
    }

}
/**
 * @brief: set joints trajectory 
 *
 * @param id: input==>the parameter id
 * @param state_cmd: input==>joints pointer
 */
void ProtoParse::setProtoJiontTraj(const uint8_t *buffer, int count)
{
    bool ret;
    if ((robot_motion_.getLogicState() != ENGAGED_S))
    {
        storeErrorCode(INVALID_ACTION_IN_CURRENT_STATE);
    }
	else if (robot_motion_.getLogicMode() != MANUAL_MODE_M)
	{
        storeErrorCode(INVALID_ACTION_IN_CURRENT_MODE);
	}
	else
	{   
        motion_spec_TeachPose tech_pose;
        PARSE_FIELD(tech_pose, motion_spec_TeachPose, buffer, count, ret);
        if (ret == false)
        {
            FST_ERROR("failed to parse tech_pose");
            return;
        } 
        if ((tech_pose.has_step) && (robot_motion_.getProgramState() != IDLE_R))
        {
            return;
        }

		U64 result = robot_motion_.checkManualJntVel(&tech_pose);
        if (result != TPI_SUCCESS)
        {
            storeErrorCode(result);
        }
	}
}
/**
 * @brief: set coordinates trajectory 
 *
 * @param id: input==>the parameter id
 * @param state_cmd: input==>coordinates pointer
 */
void ProtoParse::setProtoToolCoord(const uint8_t *buffer, int count)
{
    bool ret;
	if (robot_motion_.getLogicState() != ENGAGED_S)
    {
        storeErrorCode(INVALID_ACTION_IN_CURRENT_STATE);
    }
	else if (robot_motion_.getLogicMode() != MANUAL_MODE_M)
	{
	//	printf("wrong mode:%d\n", robot_motion_.mode);
        storeErrorCode(INVALID_ACTION_IN_CURRENT_MODE);
	}
    else if (robot_motion_.getRunningMode() != NORMAL_R)
    {
        storeErrorCode(INVALID_ACTION_IN_LIMITED_STATE);
    }
	else
	{
        motion_spec_TeachPose tech_pose;
        PARSE_FIELD(tech_pose, motion_spec_TeachPose, buffer, count, ret);
        if (ret == false)
        {
            FST_ERROR("failed to parse tech_pose");
            return;
        }
		U64 result = robot_motion_.checkManualPoseVel(&tech_pose);
        if (result != TPI_SUCCESS)
        {
            storeErrorCode(result);
        }
	}
}
/**
 * @brief: add a parameter to publish list
 *
 * @param id: input==>the parameter id
 * @param update_freq: input==> the frequency value
 * @param min_freq: input==> the min frequency value
 */
bool ProtoParse::addPubParameter(int id, int update_freq, int min_freq, int buf_len)
{

    int pub_tm = update_freq / INTERVAL_PROPERTY_UPDATE; //the unit is 10ms
    if (pub_tm == 0)
    {
        return false;
    }
    else
    {
        int max_tm = min_freq / INTERVAL_PROPERTY_UPDATE;
        if (max_tm <= pub_tm)
        {
            return false;
        }
        boost::mutex::scoped_lock lock(mutex_);
        map<int, PublishUpdate>::iterator it = param_pub_map_.find(id);
        if (it != param_pub_map_.end())
        {
            it->second.basic_freq = pub_tm;
            it->second.max_freq = max_tm;
            it->second.count = max_tm;                
            it->second.update_flag = true;
        }
        else
        {
            FST_INFO("add id:%d", id);
            PublishUpdate pub_update;
            if (buf_len > 0)
            {
                pub_update.buffer = new uint8_t[buf_len];
                pub_update.buf_len = buf_len;
            }
            else
            {
                pub_update.buffer = NULL;
                pub_update.buf_len = 0;
            }
            pub_update.basic_freq = pub_tm;
            pub_update.max_freq = max_tm;
            pub_update.count = max_tm;
            pub_update.update_flag = true;
            
            param_pub_map_.insert(map<int, PublishUpdate>::value_type(id, pub_update));
        }
        return true;
    }
}
/**
 * @brief: remove a paramet from the publish list 
 *
 * @param  id: input==>the parameter id
 */
void ProtoParse::removePubParameter(int id)
{
	boost::mutex::scoped_lock lock(mutex_);
   // FST_INFO("remove id:%d", id);

	map<int, PublishUpdate>::iterator it = param_pub_map_.find(id);
    if (it != param_pub_map_.end())
    {
        PublishUpdate up = it->second;
        if (it->second.buffer != NULL)
        {
            delete [] it->second.buffer;
        }
        param_pub_map_.erase(it);
    }
}
/**
 * @brief: remove all publish list 
 */
void ProtoParse::removeAllPubParams()
{
	boost::mutex::scoped_lock lock(mutex_);
    map<int, PublishUpdate>::iterator it = param_pub_map_.begin();
    for(; it != param_pub_map_.end(); it++)
    {
        if (it->second.buffer != NULL)
            delete it->second.buffer;
    }
	param_pub_map_.erase(param_pub_map_.begin(), param_pub_map_.end());
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
	for (unsigned int i = 0; i < get_hash_size()/sizeof(int); ++i)
	{
		if (p_a[i] != p_b[i])
		{
			//printf("a:%d, b:%d, %d\n", cmp_a[i], cmp_b[i],i);
			return false;
		}
	}

	return true;
}

U64 ProtoParse::parseToolFrame(const uint8_t *buffer, int count)
{
    bool ret;
	motion_spec_toolFrame tool_frame;
	PARSE_FIELD(tool_frame, motion_spec_toolFrame, buffer, count, ret);
	if (ret == false)
    {
        FST_ERROR("error parse tool frame");
		return DECODE_MESSAGE_FAILED;
    } 
    //printDbLine("setToolFrame:", (double*)&tool_frame, 6);

    U64 result = robot_motion_.setToolFrame(&tool_frame);
    //==calculate the new position after Transformation==
    if (result == TPI_SUCCESS)
    {
        if ((robot_motion_.updatePose()) == TPI_SUCCESS)
        {
            setUpdateFlagByID(FK_TOOL_COORD_ID, true);
        }
    }
    return result;

}

U64 ProtoParse::parseUserFrame(const uint8_t *buffer, int count)
{
    bool ret;
	motion_spec_userFrame user_frame;
	PARSE_FIELD(user_frame, motion_spec_userFrame, buffer, count, ret);
	if (ret == false)
    {
        FST_ERROR("error parse tool frame");
		return DECODE_MESSAGE_FAILED;
    } 
    
    U64 result = robot_motion_.setUserFrame(&user_frame);
    //==calculate the new position after Transformation==
    if (result == TPI_SUCCESS)
    {
        if ((robot_motion_.updatePose()) == TPI_SUCCESS)
        {
            setUpdateFlagByID(FK_TOOL_COORD_ID, true);
        }
    }
    return result;
}

U64 ProtoParse::parseSoftConstraint(const uint8_t *buffer, int count)
{
    bool ret;
    if (robot_motion_.getLogicState() != ESTOP_S)
    {
        return INVALID_ACTION_IN_CURRENT_STATE;
    }
	motion_spec_JointConstraint jnt_constraint;
	PARSE_FIELD(jnt_constraint, motion_spec_JointConstraint, buffer, count, ret);
	if (ret == false)
    {
        FST_ERROR("error parse joint constraint");
		return DECODE_MESSAGE_FAILED;
    } 
    return robot_motion_.setSoftConstraint(&jnt_constraint);
}

U64 ProtoParse::parseHardConstraint(const uint8_t *buffer, int count)
{
    bool ret;
	motion_spec_JointConstraint jnt_constraint;
	PARSE_FIELD(jnt_constraint, motion_spec_JointConstraint, buffer, count, ret);
	if (ret == false)
    {
        FST_ERROR("error parse joint constraint");
		return DECODE_MESSAGE_FAILED;
    } 
    return robot_motion_.setHardConstraint(&jnt_constraint);
}


U64 ProtoParse::parseDHGroup(const uint8_t *buffer, int count)
{
    bool ret;
	motion_spec_DHGroup dh_group;
	PARSE_FIELD(dh_group, motion_spec_DHGroup, buffer, count, ret);
	if (ret == false)
    {
        FST_ERROR("error parse dh group");
		return DECODE_MESSAGE_FAILED;
    } 
    return robot_motion_.setDHGroup(&dh_group);
}


bool ProtoParse::retDeviceInfo(BaseTypes_ParamInfo* param_info)
{
    bool ret;
    BaseTypes_ParameterMsg param_msg;
    param_msg.has_info = true;
    param_msg.info = *param_info;

    motion_spec_DeviceList dev_list;
    json_parse_->getDeviceList(dev_list);

    pb_ostream_t ostream = pb_ostream_from_buffer(param_msg.param.bytes, sizeof(param_msg.param.bytes));
	ret = pb_encode(&ostream, motion_spec_DeviceList_fields, &dev_list);
	if(ret != true)
		return ret;
    param_msg.param.size = ostream.bytes_written;

	uint8_t *buffer = nn_socket_->getReplyBufPtr();
	int length;
	SET_FIELD(param_msg, BaseTypes_ParameterMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

    nn_socket_->nnSocketReply(buffer, length);

	return ret;
}

bool ProtoParse::retToolFrame(BaseTypes_ParamInfo* param_info)
{    
    bool ret;
    BaseTypes_ParameterMsg param_msg;
    param_msg.has_info = true;
    param_msg.info = *param_info;

    motion_spec_toolFrame tool_frame = robot_motion_.getToolFrame();

    pb_ostream_t ostream = pb_ostream_from_buffer(param_msg.param.bytes, sizeof(param_msg.param.bytes));
	ret = pb_encode(&ostream, motion_spec_toolFrame_fields, &tool_frame);
	if(ret != true)
		return ret;
    param_msg.param.size = ostream.bytes_written;

	uint8_t *buffer = nn_socket_->getReplyBufPtr();
	int length;
	SET_FIELD(param_msg, BaseTypes_ParameterMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

    nn_socket_->nnSocketReply(buffer, length);

	return ret;
}

bool ProtoParse::retUserFrame(BaseTypes_ParamInfo* param_info)
{
    bool ret;
    BaseTypes_ParameterMsg param_msg;
    param_msg.has_info = true;
    param_msg.info = *param_info;

    motion_spec_userFrame user_frame = robot_motion_.getUserFrame();

    pb_ostream_t ostream = pb_ostream_from_buffer(param_msg.param.bytes, sizeof(param_msg.param.bytes));
	ret = pb_encode(&ostream, motion_spec_userFrame_fields, &user_frame);
	if(ret != true)
		return ret;
    param_msg.param.size = ostream.bytes_written;

	uint8_t *buffer = nn_socket_->getReplyBufPtr();
	int length;
	SET_FIELD(param_msg, BaseTypes_ParameterMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

    nn_socket_->nnSocketReply(buffer, length);

	return ret;
}

bool ProtoParse::retSoftConstrait(BaseTypes_ParamInfo* param_info)
{
    bool ret;
    BaseTypes_ParameterMsg param_msg;
    param_msg.has_info = true;
    param_msg.info = *param_info;

    motion_spec_JointConstraint jnt_constraint = robot_motion_.getSoftConstraint();

    pb_ostream_t ostream = pb_ostream_from_buffer(param_msg.param.bytes, sizeof(param_msg.param.bytes));
	ret = pb_encode(&ostream, motion_spec_JointConstraint_fields, &jnt_constraint);
	if(ret != true)
		return ret;
    param_msg.param.size = ostream.bytes_written;

	uint8_t *buffer = nn_socket_->getReplyBufPtr();
	int length;
	SET_FIELD(param_msg, BaseTypes_ParameterMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

    nn_socket_->nnSocketReply(buffer, length);

	return ret;
}

bool ProtoParse::retHardConstrait(BaseTypes_ParamInfo* param_info)
{
    bool ret;
    BaseTypes_ParameterMsg param_msg;
    param_msg.has_info = true;
    param_msg.info = *param_info;

    motion_spec_JointConstraint jnt_constraint = robot_motion_.getHardConstraint();

    pb_ostream_t ostream = pb_ostream_from_buffer(param_msg.param.bytes, sizeof(param_msg.param.bytes));
	ret = pb_encode(&ostream, motion_spec_JointConstraint_fields, &jnt_constraint);
	if(ret != true)
		return ret;
    param_msg.param.size = ostream.bytes_written;

	uint8_t *buffer = nn_socket_->getReplyBufPtr();
	int length;
	SET_FIELD(param_msg, BaseTypes_ParameterMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

    nn_socket_->nnSocketReply(buffer, length);

	return ret;
}

bool ProtoParse::retDHParams(BaseTypes_ParamInfo* param_info)
{
    bool ret;
    BaseTypes_ParameterMsg param_msg;
    param_msg.has_info = true;
    param_msg.info = *param_info;

    motion_spec_DHGroup dh = robot_motion_.getDHGroup();

    pb_ostream_t ostream = pb_ostream_from_buffer(param_msg.param.bytes, sizeof(param_msg.param.bytes));
	ret = pb_encode(&ostream, motion_spec_DHGroup_fields, &dh);
	if(ret != true)
		return ret;
    param_msg.param.size = ostream.bytes_written;

	uint8_t *buffer = nn_socket_->getReplyBufPtr();
	int length;
	SET_FIELD(param_msg, BaseTypes_ParameterMsg, buffer, MAX_BUFFER_SIZE, hash_size_, length, ret);

    nn_socket_->nnSocketReply(buffer, length);

	return ret;
}


void ProtoParse::parseCreatePlotMsg(const uint8_t *field_buffer, int field_size)
{
    bool ret;

	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;
	BaseTypes_CreatePlotMsg create_plot;
	PARSE_FIELD(create_plot, BaseTypes_CreatePlotMsg, field_buffer, field_size, ret);
	if (ret == false)
	{
		FST_ERROR("failed to parse create_plot");		
        return retStatus(BaseTypes_StatusCode_FAILED);  //need to define this error
	}
    FST_INFO("add plot msg:id:%d", create_plot.id);

    uint32_t msg_id;       
    if (create_plot.paths_count <= 0)
    {
        FST_ERROR("invalid create_plot");		
        return retStatus(BaseTypes_StatusCode_FAILED);  //need to define this error
    }
    
    PlotPub pp;

    pp.frq_devider = create_plot.frq_devider;
    pp.count = create_plot.frq_devider / INTERVAL_PROPERTY_UPDATE; 
    for (int i = 0; i < create_plot.paths_count; ++i)
    {
        if (json_parse_->getIDFromPath(create_plot.paths[i], msg_id))
        {
            FST_INFO("push_back id:%d", msg_id);
            pp.param_ids.push_back(msg_id);
        }  
    }
    
    {
        boost::mutex::scoped_lock lock(mutex_);
        map<int, PlotPub>::iterator it = plot_pub_.find(create_plot.id);
        if (it != plot_pub_.end())
            plot_pub_[create_plot.id] = pp;
        else
            plot_pub_.insert(map<int, PlotPub>::value_type(create_plot.id, pp));
    }
    retStatus(BaseTypes_StatusCode_OK);
}
void ProtoParse::parseRemovePlotMsg(const uint8_t *field_buffer, int field_size)
{
    bool ret;

	BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;
	BaseTypes_RemovePlotMsg remove_plot;
	PARSE_FIELD(remove_plot, BaseTypes_RemovePlotMsg, field_buffer, field_size, ret);
	if (ret == false)
	{
		FST_ERROR("failed to parse create_plot");		
        return retStatus(BaseTypes_StatusCode_FAILED);  //need to define this error
	}
    FST_INFO("remove plot msg:id:%d", remove_plot.id);
    {
        boost::mutex::scoped_lock lock(mutex_);
        map<int, PlotPub>::iterator it = plot_pub_.find(remove_plot.id);
        if (it != plot_pub_.end())
        {
            plot_pub_.erase(it);
        }
    }
    retStatus(BaseTypes_StatusCode_OK);
}

