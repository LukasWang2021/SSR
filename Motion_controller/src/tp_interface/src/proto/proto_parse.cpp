/**
 * @file proto_parse.cpp
 * @brief: use the new moveJ and moveL proto 
 * @author Wang Wei
 * @version 2.0.2
 * @date 2017-3-8
 */
#include "proto_parse.h"
#include "base.h"
#include "common.h"
#include <pb_decode.h>
#include "sub_functions.h"
#include "tp_params.h"
#include "io_interface.h"
#include "tpif_data.h"


ProtoParse::ProtoParse()
{  
    hash_size_ = get_hash_size();
}
 
ProtoParse::~ProtoParse()
{

}



ProtoParse* ProtoParse::instance()
{
    static ProtoParse proto_parse;

    return &proto_parse;
}	

BaseTypes_ParamInfo* ProtoParse::getIOInfo()
{
    static BaseTypes_ParamInfo io_info = {"", 0, 0, 2, 1, 1, 
                    BaseTypes_ParamType_INPUT_SIGNAL,
                    BaseTypes_Permission_permission_undefined,
                    BaseTypes_UserLevel_user_level_undefined,
                    BaseTypes_Unit_unit_undefined};
    return &io_info;
}

BaseTypes_ParamInfo* ProtoParse::getRegInfo()
{
    static BaseTypes_ParamInfo io_info = {"", 0, 0, 2, 1, 1, 
                    BaseTypes_ParamType_INPUT_SIGNAL,
                    BaseTypes_Permission_permission_undefined,
                    BaseTypes_UserLevel_user_level_undefined,
                    BaseTypes_Unit_unit_undefined};
    return &io_info;
}

BaseTypes_ParamInfo* ProtoParse::getInfoByID(uint32_t id)
{
    int num = sizeof(g_param_info) / sizeof(BaseTypes_ParamInfo);
	for (int i = 0; i < num; i++)    
    {
        if (g_param_info[i].id == id)
            return &g_param_info[i];
    }

    return NULL;
}

void ProtoParse::decDefault(const uint8_t *in_buf, int in_len, void *out_buf)
{
    TPIFReqData<>   *req = (TPIFReqData<>*)out_buf;
    req->fillData((char*)in_buf, in_len);
}
void ProtoParse::encDefault(const uint8_t *in_buf, int in_len, void *out_buf)
{
    BaseTypes_ParameterMsg_param_t *param = (BaseTypes_ParameterMsg_param_t*)out_buf; 
    memcpy(param->bytes, in_buf, in_len);
    param->size = in_len;
}


void ProtoParse::decSoftConstraint(const uint8_t *in_buf, int in_len, void *out_buf)
{
    bool ret;
    TPIFReqData<>   *req = (TPIFReqData<>*)out_buf;
	motion_spec_JointConstraint jnt_constraint;
	PARSE_FIELD(jnt_constraint, motion_spec_JointConstraint, in_buf, in_len, ret);
	if (ret == false)
    {
        FST_ERROR("error decode soft constraint");
		return;
    } 
    req->fillData((char*)&jnt_constraint, sizeof(jnt_constraint));
}

void ProtoParse::decManualCmd(const uint8_t *in_buf, int in_len, void *out_buf)
{
    bool ret;
    TPIFReqData<>   *req = (TPIFReqData<>*)out_buf;
	motion_spec_ManualCommand command;
	PARSE_FIELD(command, motion_spec_ManualCommand, in_buf, in_len, ret);
	if (ret == false)
    {
        FST_ERROR("error decode manual command");
		return;
    } 
    req->fillData((char*)&command, sizeof(command));
}

void ProtoParse::decTeachTarget(const uint8_t *in_buf, int in_len, void *out_buf)
{
    bool ret;
    TPIFReqData<>   *req = (TPIFReqData<>*)out_buf;
	motion_spec_TeachTarget target;
	PARSE_FIELD(target, motion_spec_TeachTarget, in_buf, in_len, ret);
	if (ret == false)
    {
        FST_ERROR("error decode teach target");
		return;
    } 
    req->fillData((char*)&target, sizeof(target));
}

int ProtoParse::checkPath(char *path)
{
    int i;
    char p_buf[256];

    if(path == NULL) {
        FST_ERROR("Command with NULL path\n");
        return -1;
    }
    if(strlen(path)<9 || strlen(path)>511) {
        FST_ERROR("Command path too short or too long\n");
        return -1;
    }
    //compare the path in head file;
    i= sizeof(g_param_info)/sizeof(g_param_info[0]);
    for(;i>=0;i--){
        if(strcmp(path,g_param_info[i].path) ==0) break;
        strcpy(p_buf,path);
        p_buf[7] = '\0';
        if(strcmp(p_buf,"root/IO") == 0 
                || strcmp(p_buf,"root/re") ) break;
    }
    if(i<0){
        printf("No such path named:%s",path);
        FST_ERROR("Command with unkown path\n");
        return -1;
    }

    return 0;

}




bool ProtoParse::decParamSetMsg(const uint8_t *in_buf, int in_len, BaseTypes_ParameterSetMsg &param_set_msg)
{
	bool ret;

	PARSE_FIELD(param_set_msg, BaseTypes_ParameterSetMsg, in_buf, in_len, ret);

    return ret;
}
bool ProtoParse::decParamGetMsg(const uint8_t *in_buf, int in_len, BaseTypes_ParameterGetMsg &param_get_msg)
{
	bool ret;

	PARSE_FIELD(param_get_msg, BaseTypes_ParameterGetMsg, in_buf, in_len, ret);
    return ret;
}
bool ProtoParse::decParamCmdMsg(const uint8_t *in_buf, int in_len, BaseTypes_ParameterCmdMsg &param_cmd_msg)
{
	bool ret;
	PARSE_FIELD(param_cmd_msg, BaseTypes_ParameterCmdMsg, in_buf, in_len, ret);
    
    return ret;
}

void ProtoParse::encIOInfo(const uint8_t *in_buf, int in_len, void *out_buf)
{
    motion_spec_DeviceList *dev_list = (motion_spec_DeviceList*)in_buf;
    BaseTypes_ParameterMsg_param_t *param = (BaseTypes_ParameterMsg_param_t*)out_buf; 
    pb_ostream_t ostream = pb_ostream_from_buffer(param->bytes, sizeof(param->bytes));
	bool ret = pb_encode(&ostream, motion_spec_DeviceList_fields, dev_list);
	if(ret != true)
    {
        FST_ERROR("error encode io info");
		return;
    }
    param->size = ostream.bytes_written;
    FST_INFO("bytes_written:%d", ostream.bytes_written);
}




void ProtoParse::encSoftConstraint(const uint8_t *in_buf, int in_len, void *out_buf)
{
    motion_spec_JointConstraint *jnt_constraint = (motion_spec_JointConstraint*)in_buf;

    BaseTypes_ParameterMsg_param_t *param = (BaseTypes_ParameterMsg_param_t*)out_buf; 
    pb_ostream_t ostream = pb_ostream_from_buffer(param->bytes, sizeof(param->bytes));
	bool ret = pb_encode(&ostream, motion_spec_JointConstraint_fields, jnt_constraint);
	if(ret != true)
    {
        FST_ERROR("error encode soft limit");
		return;
    }
    param->size = ostream.bytes_written;    
}

void ProtoParse::encDHParameters(const uint8_t *in_buf, int in_len, void *out_buf)
{
    motion_spec_DHGroup *dh_group = (motion_spec_DHGroup*)in_buf;

    BaseTypes_ParameterMsg_param_t *param = (BaseTypes_ParameterMsg_param_t*)out_buf; 
    pb_ostream_t ostream = pb_ostream_from_buffer(param->bytes, sizeof(param->bytes));
    bool ret = pb_encode(&ostream, motion_spec_DHGroup_fields, dh_group);
    if(ret != true)
    {
        FST_ERROR("error encode dh params");
        return;
    }
    param->size = ostream.bytes_written;
}

void ProtoParse::encHardConstraint(const uint8_t *in_buf, int in_len, void *out_buf)
{
    motion_spec_JointConstraint *jnt_constraint = (motion_spec_JointConstraint*)in_buf;

    BaseTypes_ParameterMsg_param_t *param = (BaseTypes_ParameterMsg_param_t*)out_buf; 
    pb_ostream_t ostream = pb_ostream_from_buffer(param->bytes, sizeof(param->bytes));
	bool ret = pb_encode(&ostream, motion_spec_JointConstraint_fields, jnt_constraint);
	if(ret != true)
    {
        FST_ERROR("error encode hard limit");
		return;
    }
    param->size = ostream.bytes_written;
}

/**
 * @brief: return status to TP
 *
 * @param: param_info: input
 * @param: status_code: input==>status code
 *
 * @return: true if success
 */
bool ProtoParse::encStatus(BaseTypes_StatusCode status_code, BaseTypes_ParamInfo *param_info, uint8_t *buf_out, int buf_len, int& bytes_written)
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

	SET_FIELD(status_msg, BaseTypes_StatusMsg, buf_out, buf_len, hash_size_, bytes_written, ret);

    return ret;
}



bool ProtoParse::encParamMsg(const BaseTypes_ParameterMsg &param_msg, uint8_t *buf_out, int buf_len, int& bytes_written)
{
   bool ret;

	SET_FIELD(param_msg, BaseTypes_ParameterMsg, buf_out, buf_len, hash_size_, bytes_written, ret); 

    return ret;
}

bool ProtoParse::encPubGroupMsg(motion_spec_SignalGroup *sig_group, uint8_t *buf_out, int buf_len, int& bytes_written)
{
    pb_ostream_t stream = {0};
    stream = pb_ostream_from_buffer(buf_out, buf_len);
	bool ret = pb_encode(&stream, motion_spec_SignalGroup_fields, sig_group);	
    bytes_written = stream.bytes_written;

    return ret;
}

bool encListCallback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
bool ProtoParse::encParamListMsg(uint8_t *buf_out, int buf_len, int& bytes_written)
{
    bool ret;
	BaseTypes_ParameterListMsg param_list_msg;
	param_list_msg.has_header = false;
	param_list_msg.params.funcs.encode = encListCallback;//
    param_list_msg.params.arg = this;

	SET_FIELD(param_list_msg, BaseTypes_ParameterListMsg, buf_out, buf_len, hash_size_, bytes_written, ret);

	return ret;
}



/**
 * @brief: decode the motion program 
 *
 * @param buffer: input==>the buffer to parse
 * @param count: input==>the count of buffer
 *
 * @return: true if success
 */
#if 0
void ProtoParse::decMotionProgram(const uint8_t *in_buf, int in_len, void *out_buf)
{
	bool ret;
    TPIFReqData<>   *req = (TPIFReqData<>*)out_buf;

	static motion_spec_MotionProgram motion_program;
	PARSE_FIELD(motion_program, motion_spec_MotionProgram, in_buf, in_len, ret);
	if (ret == false)
    {
        FST_ERROR("error parse motion program");
		return;
    }
	FST_INFO("motion program: name:%s, id:%d, cmd_list:%d", motion_program.name, motion_program.id, motion_program.commandlist_count);
    CommandInstruction cmd_instruction;    
    unsigned int j = 0;
    //CommandInstruction *cmd_inst = (CommandInstruction*)req->getParamBufPtr();
    int size = sizeof(cmd_instruction);
	for (; j < motion_program.commandlist_count; ++j)
	{	 		
        
        if (decMotionCommand(motion_program.commandlist[j], cmd_instruction) == false)
        { 
            FST_ERROR("error parse motion command");
		    break; //won't parse next command
        }
        memcpy(req->getParamBufPtr()+j*size, (void*)&cmd_instruction, size);
    }
    

    req->setParamLen(j);
}

/**
 * @brief
 *
 * @param motion_command: input==>the command to parse
 * @param cmd_instruction: output==>store the result of parsing
 *
 * @return true if Success
 */
bool ProtoParse::decMotionCommand(motion_spec_MotionCommand motion_command, CommandInstruction &cmd_instruction)
{
	bool ret;	

    cmd_instruction.pick_status = FRESH;
    cmd_instruction.smoothDistance = 0; //0 is default
	cmd_instruction.id = motion_command.id;
	cmd_instruction.commandtype = motion_command.commandtype;
    cmd_instruction.count = 0;
    cmd_instruction.timesout = 0;

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
				FST_ERROR("error parse motion command");
				return false;
			}
			FST_INFO("MoveJ:vmax:%f, amax:%f,has_smth:%d,smth:%f",moveJ.vMax,moveJ.aMax,moveJ.has_smoothPercent,moveJ.smoothPercent);
            if (moveJ.has_aMax == false)
                moveJ.aMax = DEFAULT_ACC;
			if (moveJ.has_smoothPercent) 
				cmd_instruction.smoothDistance = moveJ.smoothPercent;
			else
				cmd_instruction.smoothDistance = -1;
			cmd_instruction.command_arguments.movej = moveJ;
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
            
            cmd_instruction.command_arguments.movel = moveL;
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
			cmd_instruction.command_arguments.movec = moveC;
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
			//	FST_ERROR("error parse motion command");
				return false;
			}
           
            //cmd_instruction.timeout = wait.timeout;
            if ((wait.has_timeout) && (wait.timeout > 0))
                 cmd_instruction.timesout = (wait.timeout+STATE_MACHINE_INTERVAL-1) / STATE_MACHINE_INTERVAL;
            cmd_instruction.command_arguments.wait = wait;
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
            //cmd_instruction.timeout = set.time_sec;
            if ((set.has_time_sec) && (set.time_sec > 0))
                cmd_instruction.timesout = (set.time_sec+STATE_MACHINE_INTERVAL-1) / STATE_MACHINE_INTERVAL;
            cmd_instruction.command_arguments.set = set;
			break;
        }//end case motion_spec_MOTIONTYPE_SET:
		default:
			break;
	}//end switch (motion_command.commandtype)

	return true;
}

#endif

bool encListCallback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
    BaseTypes_ParameterMsg param_msg = {};
    
    int num = sizeof(g_param_info) / sizeof(BaseTypes_ParamInfo);
	for (int i = 0; i < num; i++)    
    {
        param_msg.has_info = true;
        param_msg.info = g_param_info[i];        
        /* This encodes the header for the field, based on the constant info
         * from pb_field_t. */
        if (!pb_encode_tag_for_field(stream, field))
            return false;
        
        /* This encodes the data for the field, based on our FileInfo structure. */
        if (!pb_encode_submessage(stream, BaseTypes_ParameterMsg_fields, &param_msg))
            return false;
    }
    
    return IOInterface::instance()->encDevList(&param_msg, stream, field);
}

void ProtoParse::encVersionInfo(const uint8_t *in_buf, int in_len, void *out_buf)
{
    base_types_VersionInfo *version = (base_types_VersionInfo*)in_buf;

    BaseTypes_ParameterMsg_param_t *param = (BaseTypes_ParameterMsg_param_t*)out_buf; 

    FST_INFO("param is : %d", param);

    pb_ostream_t ostream = pb_ostream_from_buffer(param->bytes, sizeof(param->bytes));

    bool ret = pb_encode(&ostream, base_types_VersionInfo_fields, version);

    if(ret != true)
    {
        FST_ERROR("error encode io info");
        return;
    }

    param->size = ostream.bytes_written;

    FST_INFO("bytes_written:%d", ostream.bytes_written);
}


void ProtoParse::decFrame(const uint8_t *in_buf, int in_len, void *out_buf)
{
    frame_spec_Interface frame_interface = *(frame_spec_Interface*)in_buf;

    TPIFReqData<> *req = (TPIFReqData<>*)out_buf;

    req->fillData((char*)&frame_interface, sizeof(frame_interface));
}


void ProtoParse::encFrame(const uint8_t *in_buf, int in_len, void *out_buf)
{
    frame_spec_Interface *frame_interface = (frame_spec_Interface*)in_buf;

    BaseTypes_ParameterMsg_param_t *param = (BaseTypes_ParameterMsg_param_t*)out_buf;
    pb_ostream_t ostream = pb_ostream_from_buffer(param->bytes, sizeof(param->bytes));

	bool ret = pb_encode(&ostream, frame_spec_Interface_fields, frame_interface);

	if(!ret)
    {
        FST_ERROR("error encode frame");
        return;
    }

    param->size = ostream.bytes_written;
}


void ProtoParse::decActivateFrame(const uint8_t *in_buf, int in_len, void *out_buf)
{
    frame_spec_ActivateInterface activate_frame = *(frame_spec_ActivateInterface*)in_buf;

    TPIFReqData<>   *req = (TPIFReqData<>*)out_buf;

    req->fillData((char*)&activate_frame, sizeof(activate_frame));
}


void ProtoParse::encActivateFrame(const uint8_t *in_buf, int in_len, void *out_buf)
{
    frame_spec_ActivateInterface *activate_frame = (frame_spec_ActivateInterface*)in_buf;

    BaseTypes_ParameterMsg_param_t *param = (BaseTypes_ParameterMsg_param_t*)out_buf;
    pb_ostream_t ostream = pb_ostream_from_buffer(param->bytes, sizeof(param->bytes));

    bool ret = pb_encode(&ostream, frame_spec_ActivateInterface_fields, activate_frame);

    if(!ret)
    {
        FST_ERROR("error encode activate frame");
        return;
    }

    param->size = ostream.bytes_written;
}

void ProtoParse::encRegister(const uint8_t *in_buf, int in_len, void *out_buf)
{
    register_spec_RegMap *reg_param = (register_spec_RegMap*)in_buf;

    BaseTypes_ParameterMsg_param_t *param = (BaseTypes_ParameterMsg_param_t*)out_buf; 

    FST_INFO("param is : %d", param);

    pb_ostream_t ostream = pb_ostream_from_buffer(param->bytes, sizeof(param->bytes));

    bool ret = pb_encode(&ostream, register_spec_RegMap_fields, reg_param);

    if(ret != true)
    {
        FST_ERROR("error encode io info");
        return;
    }

    param->size = ostream.bytes_written;

    FST_INFO("bytes_written:%d", ostream.bytes_written);
}


void ProtoParse::decRegister(const uint8_t *in_buf, int in_len, void *out_buf)
{
    register_spec_RegMap register_interface = *(register_spec_RegMap*)in_buf;

    TPIFReqData<> *req = (TPIFReqData<>*)out_buf;

    req->fillData((char*)&register_interface, sizeof(register_interface));
}


void ProtoParse::encString(const uint8_t *in_buf, int in_len, void *out_buf)
{
    BaseTypes_CommonString *string_data = (BaseTypes_CommonString*)in_buf;

    BaseTypes_ParameterMsg_param_t *param = (BaseTypes_ParameterMsg_param_t*)out_buf; 

    FST_INFO("param is : %d", param);

    pb_ostream_t ostream = pb_ostream_from_buffer(param->bytes, sizeof(param->bytes));

    bool ret = pb_encode(&ostream, BaseTypes_CommonString_fields, string_data);

    if(ret != true)
    {
        FST_ERROR("error encode io info");
        return;
    }

    param->size = ostream.bytes_written;

    FST_INFO("bytes_written:%d", ostream.bytes_written);
}