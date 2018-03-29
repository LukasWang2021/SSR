/**
 * @file tp_interface.cpp
 * @brief: use the new moveJ and moveL proto 
 * @author Wang Wei
 * @version 2.0.2
 * @date 2017-3-8
 */
#include "tp_interface.h"
#include "base_types_hash.h"
#include "ip_address.h"
#include "error_monitor.h"
#include "io_interface.h"
#include "error_code.h"
#include "hash_map.h"
#include "proto_func.h"


TPInterface::TPInterface()
{  
    string str_addr = getLocalIP();
	NNSockType type = NN_SOCK_WS;
	nn_socket_ = new NNSocket(str_addr, type, type, COMMAND_PORT, STATE_PORT);
	FST_ASSERT(nn_socket_);

    proto_parser_ = new ProtoParse;
    
    task_ = new rcs::Task(10);
    task_->function(std::bind(&TPInterface::updateInterface, this, (void*)nn_socket_));
    task_->run();
}
 
TPInterface::~TPInterface()
{
    if (nn_socket_ != NULL)
        delete nn_socket_;
    
    if (proto_parser_ != NULL)
        delete proto_parser_;
}

void TPInterface::destroy()
{
    task_->stop();
    if (task_ != NULL)
        delete task_;
}

TPInterface* TPInterface::instance()
{
    static TPInterface tpif;

    return &tpif;
}

TPIFReqData<>* TPInterface::getReqDataPtr()
{
    return &request_;
}
TPIFRepData* TPInterface::getRepDataPtr()
{
    return &reply_;
}


TPIPubData* TPInterface::getPubDataPtr()
{
    return &publish_;
}


void* TPInterface::updateInterface(void *params)
{
    NNSocket *sock = (NNSocket*)params;
    sock->nnPoll(2);
	if(sock->nnSocketRecieve() == true)
	{
        //FST_INFO("get recieve");
		parseTPCommand(sock->getRequestBufPtr(), sock->getRequestBufLen());
        //FST_INFO("send reply");
        //==reply to TP===========
        sendReply();
	}
    
    sendPublish();

    return NULL;
}

#include "base.h"
void TPInterface::parseTPCommand(const uint8_t* buffer, int len)
{
    char *path;
    int id;
    int i;
    int hash_size = get_hash_size();
    const uint8_t *field_buffer = buffer+hash_size;
	int field_size = len - hash_size;
	
	if (HASH_CMP(ParameterSetMsg, buffer) == true)
	{        
        BaseTypes_ParameterSetMsg param_set_msg;
        
        if (!proto_parser_->decParamSetMsg(field_buffer, field_size, param_set_msg))
        {
            goto exception;
        }
        //do not allow request using id*/
        if (!param_set_msg.has_path)
        {
            goto exception;
        }

        path = param_set_msg.path;
        //qianjin: add path check 20180329
        if(proto_parser_->checkPath(path)!=0){
            goto exception;
        }

        id = ELFHash(path, strlen(path));
        FST_INFO("setMsg:path:%s, %d", path, *(int*)param_set_msg.param.bytes);
        if (request_.update(SET, path, id))
        {
            std::map<int, ProtoFunctions>::iterator it = g_proto_funcs_mp.find(id);
            if (it != g_proto_funcs_mp.end())
            {
                (proto_parser_->*it->second.setMsg)(param_set_msg.param.bytes, param_set_msg.param.size, &request_);
            }            
            else
            {
                FST_ERROR("can't find this proto functions");
            }
        }
        else
        {
            setReply(BaseTypes_StatusCode_FAILED);
        }
    }//end if (HASH_CMP(ParameterSetMsg, buffer) == true)
	else if (HASH_CMP(ParameterCmdMsg, buffer) == true)
	{
        //FST_INFO("cmdMsg");
        BaseTypes_ParameterCmdMsg param_cmd_msg;
        if(!proto_parser_->decParamCmdMsg(field_buffer, field_size, param_cmd_msg))
        {
            goto exception;
        }
        CmdParams cmd_param;
        cmd_param.cmd = param_cmd_msg.cmd;
        //string str_param(0);
        if (param_cmd_msg.cmd == BaseTypes_CommandType_ADD)
        {
            //FST_INFO("add");
            PublishUpdate pub_update;
            if (param_cmd_msg.has_update_frq_devider)
                pub_update.base_interval = param_cmd_msg.update_frq_devider;
            else
                goto exception;                
            if (param_cmd_msg.has_minimum_frq_devider)
            {
                pub_update.max_interval = param_cmd_msg.minimum_frq_devider;   
                if (pub_update.max_interval < pub_update.base_interval)
                    goto exception;
            }

            //FST_INFO("freq:%d,%d", pub_update.base_interval, pub_update.max_interval);
            cmd_param.pub_update = pub_update;
        }// if (param_cmd_msg.cmd == BaseTypes_CommandType_ADD)      
        path = param_cmd_msg.path;
        id = ELFHash(path, strlen(path));
        //FST_INFO("cmd:path:%s, id:%d", path, id);
        if (request_.update(CMD, path, id))
        {
            request_.fillData((char*)&cmd_param, sizeof(cmd_param));
        }
        else
        {
            setReply(BaseTypes_StatusCode_FAILED);
        }
        //setRequest(CMD, param_cmd_msg.path, str_param);
	}//end else if (HASH_CMP(ParameterCmdMsg, buffer) == true)
	else if (HASH_CMP(ParameterGetMsg, buffer) == true)
	{
        
        BaseTypes_ParameterGetMsg param_get_msg;
        if (!proto_parser_->decParamGetMsg(field_buffer, field_size, param_get_msg))
        {
            goto exception;
        }
        if (!param_get_msg.has_path)
        {
            goto exception;
        }
        path = param_get_msg.path;
        //qianjin: add path check 20180329
        if(proto_parser_->checkPath(path)!=0){
            goto exception;
        }
        id = ELFHash(path, strlen(path));
        //FST_INFO("getMsg_path:%s", path);
        if (request_.update(GET, path, id))
        {
            if (param_get_msg.has_param)
            {
                request_.fillData((char*)param_get_msg.param.bytes, param_get_msg.param.size);
            }            
        }
        else
        {
            setReply(BaseTypes_StatusCode_FAILED);
        }

	}//end else if (HASH_CMP(ParameterGetMsg, buffer) == true)
    else if (HASH_CMP(ParameterOverwriteMsg, buffer) == true)
	{
        //BaseTypes_ParameterOverwriteMsg param_ow_msg;
        //proto_parser_->decParamOwMsg(field_buffer, field_size, param_ow_msg);
        //fillDataTimeout(OVERWRITE, param_ow_msg);

	}//
    else
	{
        goto exception;
    }

    return;
exception:
    FST_ERROR("invalid tp command");
    rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
    setReply(BaseTypes_StatusCode_FAILED);

}

void TPInterface::sendReply()
{    
    while (!reply_.isFilled()) //wait until reply
        usleep(100);
    int buf_len;
    TPRepType type = reply_.getType();
    switch (type)
    {
        case STATUS:
        {
            BaseTypes_StatusCode *status_code = (BaseTypes_StatusCode*)reply_.getParamBufPtr();
            BaseTypes_ParamInfo *info = proto_parser_->getInfoByID(reply_.getID());
            proto_parser_->encStatus(*status_code, info, nn_socket_->getReplyBufPtr(), MAX_BUFFER_SIZE, buf_len);            
            break;
        }
        case PARAM:
        {
            int id = reply_.getID();
            BaseTypes_ParameterMsg param_msg;
            BaseTypes_ParamInfo *info;
            if (id < IO_BASE_ADDRESS)
            {
                (proto_parser_->*g_proto_funcs_mp[id].getMsg)(reply_.getParamBufPtr(), reply_.getParamLen(), &param_msg.param);  
                info = proto_parser_->getInfoByID(id);
            }
            else
            {
                proto_parser_->encDefault(reply_.getParamBufPtr(), reply_.getParamLen(), &param_msg.param);
                info = proto_parser_->getIOInfo();
            }
            
            if (info == NULL)
            {
                param_msg.has_info = false;
            }
            else
            {
                param_msg.has_info = true;
                param_msg.info = *info;
            }

            proto_parser_->encParamMsg(param_msg, nn_socket_->getReplyBufPtr(), MAX_BUFFER_SIZE, buf_len);            
            break;
        }
        case LIST:
        {
            bool ret = proto_parser_->encParamListMsg(nn_socket_->getReplyBufPtr(), MAX_BUFFER_SIZE, buf_len);
            FST_INFO("list buf_len:%d",buf_len);
            break;
        }
        default:
            break;
    }
    nn_socket_->nnSocketReply(nn_socket_->getReplyBufPtr(), buf_len);
    reply_.setFilledFlag(false);
}

void TPInterface::sendPublish()
{
    if (!publish_.isFilled())
        return;
    int len;
    proto_parser_->encPubGroupMsg(publish_.getParamBufPtr(), nn_socket_->getPublishBufPtr(), MAX_BUFFER_SIZE, len);
    motion_spec_SignalGroup *sig_gp = publish_.getParamBufPtr();
    //FST_INFO("len:%d, size:%d", len, sig_gp->sig_param_count);
    nn_socket_->nnSocketPublish(nn_socket_->getPublishBufPtr(), len);
    publish_.setFilledFlag(false);
}

void TPInterface::setReply(BaseTypes_StatusCode status_code, int id)
{
    if (id > 0)
    {
        reply_.setID(id);
    }
    reply_.fillData((char*)&status_code, sizeof(status_code));
    reply_.setType(STATUS);
    reply_.setFilledFlag(true);
}

void TPInterface::setReply(TPRepType type, int id)
{    
    reply_.setID(id);
    reply_.setType(type);
    reply_.setFilledFlag(true);
}
/**
 * @brief: compare two strings in the form of int
 *
 * @param cmp_a: input
 * @param cmp_b: input
 *
 * @return: true if they are the same 
 */
bool TPInterface::compareInt(const unsigned char *cmp_a, const uint8_t *cmp_b)
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
