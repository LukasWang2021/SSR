/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       comm_interface.cpp
Author:     Feng.Wu 
Create:     04-Nov-2016
Modify:     08-Dec-2016
Summary:    lib to communicate between processes
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_COMM_INTERFACE_CPP_
#define MIDDLEWARE_TO_MEM_COMM_INTERFACE_CPP_

#include "comm_interface/comm_interface.h"
#include <iostream>
#include <sstream>
#include <string.h>
#include <nanomsg/nn.h>
#include <nanomsg/reqrep.h>
#include <nanomsg/pubsub.h>

namespace fst_comm_interface
{

//------------------------------------------------------------
// Function:  CommInterface
// Summary: The constructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
CommInterface::CommInterface()
{
    fd_ = -1;
    error_flag_ = CREATE_CHANNEL_FAIL;
}

//------------------------------------------------------------
// Function:  ~CommInterface
// Summary: The destructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
CommInterface::~CommInterface()
{
    closeChannel();
}

//------------------------------------------------------------
// Function:  createChannel
// Summary: Create the channel for IPC communication
// In:      type -> REQ:request, REP:response, PUB:publisher, SUB:subscriber
//          name -> name for each channel
// Out:     None
// Return:  0 -> succeed to get handle.
//          CREATE_CHANNEL_FAIL -> failed to create channel 
//------------------------------------------------------------
ERROR_CODE_TYPE CommInterface::createChannel(int type, const char *name)
{
    if(type != IPC_REQ && type != IPC_REP && type != IPC_PUB && type != IPC_SUB)
    {
        std::cout<<"Error in CommInterface::createChannel(): Please enter a type(IPC_REQ, IPC_REP, IPC_PUB, IPC_SUB)."<<std::endl;
        return CREATE_CHANNEL_FAIL;
    }
    if (strlen(name) >= NAME_SIZE)
    {
        std::cout<<"Error in  CommInterface::createChannel(): Name exceeds 64 characters."<<std::endl;
        return CREATE_CHANNEL_FAIL;
    }

    //create a socket.
    fd_ = nn_socket(AF_SP, type);
    if (fd_ < 0)
    {
        std::cout<<"Error in CommInterface::createChannel() for <"<<name<<"> socket: "<<nn_strerror(nn_errno())<<std::endl;
        return CREATE_CHANNEL_FAIL;
    }
   
    //bind or connect to url.
/*    char url[NAME_SIZE];
    sprintf(url, "ipc:///tmp/msg_%s.ipc", name);
*/
    std::ostringstream oss;
    oss << "ipc:///tmp/msg_" << name << ".ipc";
    std::string ss = oss.str();
    const char *url = ss.c_str();
 
    if (type == NN_REP || type == NN_PUB)
    {
        if (nn_bind(fd_, url) < 0)
        {
            std::cout<<"Error in CommInterface::createChannel() to bind <"<<name<<"> : "<<nn_strerror(nn_errno())<<std::endl;
            return CREATE_CHANNEL_FAIL;
        }
    } 
    else if (type == NN_REQ || type == NN_SUB)
    {
        if (nn_connect(fd_, url) < 0)
        {
            std::cout<<"Error in CommInterface::createChannel() to connect <"<<name<<"> : "<<nn_strerror(nn_errno())<<std::endl;
            return CREATE_CHANNEL_FAIL;
        }
    } 
    //set sockopt specially for NN_SUB, otherwise can't receive.
    if (type == NN_SUB)
    {
        if (nn_setsockopt(fd_, NN_SUB, NN_SUB_SUBSCRIBE, "", 0) < 0)
        {
            std::cout<<"Error in CommInterface::createChannel() to set <"<<name<<"> SUB: "<<nn_strerror(nn_errno())<<std::endl;
            return CREATE_CHANNEL_FAIL;
        };
    }

    error_flag_ = 0;
    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  send
// Summary: send data
// In:      buff -> data to be sent.
//          buff_size -> date size
//          flag -> block or not(IPC_WAIT/IPC_DONTWAIT).
// Out:     None
// Return:  0 -> succeed to send data.
//          SEND_MSG_FAIL -> failed to send data.
//          CREATE_CHANNEL_FAIL -> didn't or failed to create channel 
//------------------------------------------------------------
ERROR_CODE_TYPE CommInterface::send(const void *buf, int buf_size, int flag)
{
    if (error_flag_ != 0)
        return error_flag_;

    if(flag != IPC_WAIT && flag != IPC_DONTWAIT)
    {
        std::cout<<"Error in CommInterface::send(): Please enter a type(REQ, REP, PUB, SUB)."<<std::endl;
        return SEND_MSG_FAIL;
    }

    if (buf_size > MAX_MSG_SIZE)
    {
        std::cout<<"Warning in CommInterface::send(): The size value exceeds maximum message size."<<std::endl;
    }

    int result = nn_send(fd_, buf, buf_size, flag);
    if (result < 0)
    {
        std::cout<<"Error in CommInterface::send(): "<<nn_strerror(nn_errno())<<std::endl;
        return SEND_MSG_FAIL;
    }
    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  send
// Summary: send data by string
// In:      str -> string to be sent.
//          flag -> block or not(IPC_WAIT/IPC_DONTWAIT).
// Out:     None
// Return:  0 -> succeed to send data.
//          SEND_MSG_FAIL -> failed to send data.
//          CREATE_CHANNEL_FAIL -> didn't or failed to create channel 
//------------------------------------------------------------
ERROR_CODE_TYPE CommInterface::send(std::string str, int flag)
{
    if (error_flag_ != 0)
        return error_flag_;

    char *send_buf = new char[str.length() + 1];
    strcpy(send_buf, str.c_str());

    ERROR_CODE_TYPE result = send(send_buf, str.length() + 1, flag);

    delete[] send_buf;
    return result;
}

//------------------------------------------------------------
// Function:  recv
// Summary: receive data
// In:      buff -> data to be read.
//          buff_size -> date size
//          flag -> block or not(IPC_WAIT/IPC_DONTWAIT).
// Out:     None
// Return:  0 -> succeed to get data.
//          RECV_MSG_FAIL -> failed to get data.
//          CREATE_CHANNEL_FAIL -> didn't or failed to create channel
//------------------------------------------------------------
ERROR_CODE_TYPE CommInterface::recv(void *buf, int buf_size, int flag)
{
    if (error_flag_ != 0)
        return error_flag_;

    if(flag != IPC_WAIT && flag != IPC_DONTWAIT)
    {
        std::cout<<"Error in CommInterface::recv(): Please enter a type(REQ, REP, PUB, SUB)."<<std::endl;
        return RECV_MSG_FAIL;
    }
    if (buf_size > MAX_MSG_SIZE)
    {
        std::cout<<"Warning in CommInterface::recv(): The size value exceeds maximum message size."<<std::endl;
    } 
    int result = nn_recv(fd_, buf, buf_size, flag);
    if (result < 0)
    {
        if (nn_errno() != EAGAIN)
        {
            std::cout<<"Error in CommInterface::recv(): "<<nn_strerror(nn_errno())<<std::endl;
        }
        return RECV_MSG_FAIL;
    }    
    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  recv
// Summary: receive data
// In:      string -> string to be read.
//          flag -> block or not(IPC_WAIT/IPC_DONTWAIT).
// Out:     None
// Return:  0 -> succeed to get data.
//          RECV_MSG_FAIL -> failed to get data.
//          CREATE_CHANNEL_FAIL -> didn't or failed to create channel
//------------------------------------------------------------
ERROR_CODE_TYPE CommInterface::recv(std::string *str, int flag)
{
    if (error_flag_ != 0)
        return error_flag_;

    if(flag != IPC_WAIT && flag != IPC_DONTWAIT)
    {
        std::cout<<"Error in CommInterface::recv(): Please enter a type(REQ, REP, PUB, SUB)."<<std::endl;
        return RECV_MSG_FAIL;
    }

    char *buf = NULL;
    int result = nn_recv(fd_, &buf, NN_MSG, flag);
    if (result < 0)
    {
        if (nn_errno() != EAGAIN)
        {
            std::cout<<"Error in CommInterface::recv(): "<<nn_strerror(nn_errno())<<std::endl;
        }
        return RECV_MSG_FAIL;
    }    
    *str = buf;
    nn_freemsg(buf);
    return FST_SUCCESS;

}

//------------------------------------------------------------
// Function:  close
// Summary: closeChannel
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void CommInterface::closeChannel(void)
{
    nn_close(fd_);
}


} //namespace fst_comm_interface

#endif //MIDDLEWARE_TO_MEM_COMM_INTERFACE_CPP_
