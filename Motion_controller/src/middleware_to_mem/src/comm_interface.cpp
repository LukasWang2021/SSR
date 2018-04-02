/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       comm_interface.cpp
Author:     Feng.Wu 
Create:     04-Nov-2016
Modify:     09-Jun-2017
Summary:    lib to communicate between processes
**********************************************/
#ifndef MIDDLEWARE_TO_MEM_COMM_INTERFACE_CPP_
#define MIDDLEWARE_TO_MEM_COMM_INTERFACE_CPP_

#include "comm_interface/comm_interface.h"
#include <unistd.h>
#include <string.h>
#include <net/if.h>  // socket()
#include <sys/ioctl.h> // ioctl()
#include <arpa/inet.h> // inet_ntoa()
#include <iostream>
#include <sstream>
#include <nanomsg/nn.h>  
#include <nanomsg/reqrep.h>
#include <nanomsg/pubsub.h>
#include "middleware_to_mem_version.h"

namespace fst_comm_interface
{

int CommInterface::obj_num_;

//------------------------------------------------------------
// Function:  getVersion
// Summary: get the version. 
// In:      None
// Out:     None
// Return:  std::string -> the version.
//------------------------------------------------------------
std::string getVersion(void)
{
     std::stringstream ss;
    ss<<middleware_to_mem_VERSION_MAJOR<<"."
        <<middleware_to_mem_VERSION_MINOR<<"."
        <<middleware_to_mem_VERSION_PATCH;
    std::string s = ss.str();

    return s;
}

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
    max_msg_size = MAX_MSG_SIZE;
    obj_num_++;
    error_flag_ = CREATE_CHANNEL_FAIL;
    if (obj_num_ == 1)
        std::cout<<"lib_comm_interface version:"<<getVersion()<<std::endl;
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
// In:      type -> COMM_REQ:request, COMM_REP:response, COMM_PUB:publisher, COMM_SUB:subscriber
//          transport -> COMM_IPC, COMM_TCP, COMM_INPROC.
//          name -> name for each channel
// Out:     None
// Return:  FST_SUCCESS -> succeed to get handle.
//          CREATE_CHANNEL_FAIL -> failed to create channel 
//------------------------------------------------------------
ERROR_CODE_TYPE CommInterface::createChannel(int protocol, int transport, const char *name)
{
    if (strlen(name) >= NAME_SIZE || strlen(name) == 0)
    {
        std::cout<<"Error in  CommInterface::createChannel(): Name exceeds 32 characters or empty."<<std::endl;
        return CREATE_CHANNEL_FAIL;
    }

    int prot = 0 ;
    char url[URL_SIZE];
    switch (protocol)
    {
        case COMM_REQ:
            prot = NN_REQ;
            break;
        case COMM_REP:
            prot = NN_REP;
            break;
        case COMM_PUB:
            prot = NN_PUB;
            break;
        case COMM_SUB:
            prot = NN_SUB;
            break;
        default:
            std::cout<<"Error in CommInterface::createChannel(): Please enter a right protocol(COMM_REQ, COMM_REP, COMM_PUB, COMM_SUB)."<<std::endl;
            return CREATE_CHANNEL_FAIL;
    }

    switch (transport)
    {
        case COMM_IPC:
            convertIpcUrl(name, url);
            break;
        case COMM_TCP:
            convertTcpUrl(name, url);
            break;
        case COMM_INPROC:
            convertInprocUrl(name, url);
            break;
        default:
            std::cout<<"Error in CommInterface::createChannel(): Please enter a right transport(COMM_IPC, COMM_TCP, COMM_INPROC)."<<std::endl;
            return CREATE_CHANNEL_FAIL;
    }

    //create a socket.
    fd_ = nn_socket(AF_SP, prot);
    if (fd_ < 0)
    {
        std::cout<<"Error in CommInterface::createChannel() for <"<<name<<"> socket: "<<nn_strerror(nn_errno())<<std::endl;
        return CREATE_CHANNEL_FAIL;
    }
    switch (prot)
    {
        case NN_REP:
        case NN_PUB:
            if (!bind(url))
                return CREATE_CHANNEL_FAIL;
            break;
        case NN_REQ:
            if (!connect(url))
                return CREATE_CHANNEL_FAIL;
            break;
        case NN_SUB:
            if (!connect(url))
                return CREATE_CHANNEL_FAIL;
            if (!setSubOpt())
                return CREATE_CHANNEL_FAIL;
            break;
        default:
            break;
    }

    error_flag_ = 0;

    if (!setMaxMsgSize(max_msg_size))
    {
        std::cout<<"Error in CommInterface::createChannel() to set the max message size: "<<std::endl;
        error_flag_ = CREATE_CHANNEL_FAIL;
        return CREATE_CHANNEL_FAIL;
    };

    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  send
// Summary: send data
// In:      buff -> data to be sent.
//          buff_size -> date size
//          flag -> block or not(IPC_WAIT/IPC_DONTWAIT).
// Out:     None
// Return:  FST_SUCCESS -> succeed to send data.
//          SEND_MSG_FAIL -> failed to send data.
//          CREATE_CHANNEL_FAIL -> didn't or failed to create channel 
//------------------------------------------------------------
ERROR_CODE_TYPE CommInterface::send(const void *buf, int buf_size, int flag)
{
    if (error_flag_ != 0)
        return error_flag_;

    if(flag != COMM_WAIT && flag != COMM_DONTWAIT)
    {
        std::cout<<"Error in CommInterface::send(): Please enter a type(IPC_DONTWAIT, IPC_WAIT)."<<std::endl;
        return SEND_MSG_FAIL;
    }

    if (buf_size > max_msg_size)
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
// Return:  FST_SUCCESS -> succeed to send data.
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
// Return:  FST_SUCCESS -> succeed to get data.
//          RECV_MSG_FAIL -> failed to get data.
//          CREATE_CHANNEL_FAIL -> didn't or failed to create channel
//------------------------------------------------------------
ERROR_CODE_TYPE CommInterface::recv(void *buf, int buf_size, int flag)
{
    if (error_flag_ != 0)
        return error_flag_;

    if(flag != COMM_WAIT && flag != COMM_DONTWAIT)
    {
        std::cout<<"Error in CommInterface::recv(): Please enter a type(IPC_DONTWAIT, IPC_WAIT)."<<std::endl;
        return RECV_MSG_FAIL;
    }
    if (buf_size > max_msg_size)
    {
        std::cout<<"Warning in CommInterface::recv(): The size value exceeds maximum message size."<<std::endl;
    } 
    int result = nn_recv(fd_, buf, buf_size, flag);
    if (result < 0)
    {
        //printf("error:%d\n", result);
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
// Return:  FST_SUCCESS -> succeed to get data.
//          RECV_MSG_FAIL -> failed to get data.
//          CREATE_CHANNEL_FAIL -> didn't or failed to create channel
//------------------------------------------------------------
ERROR_CODE_TYPE CommInterface::recv(std::string *str, int flag)
{
    if (error_flag_ != 0)
        return error_flag_;
    if(flag != COMM_WAIT && flag != COMM_DONTWAIT)
    {
        std::cout<<"Error in CommInterface::recv(): Please enter a type(IPC_DONTWAIT, IPC_WAIT)."<<std::endl;
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
// Function:  getLocalIP
// Summary: get the local IP address
// In:      iface_name -> interface name such as "eth0", "wlan0" or "lo".
// Out:     *ip -> the IP adress according to the interface name.
// Return:  true -> succeed to get IP address.
//          false -> failed to get IP address.
//------------------------------------------------------------
bool CommInterface::getLocalIP(char **ip, const char *iface_name)
{
    if (strcmp(iface_name, "eth0") != 0 && strcmp(iface_name, "wlan0") != 0 && strcmp(iface_name, "lo") != 0)
    {
        std::cout<<"Error in CommInterface::getLocalIp(): invalid interface name."<<std::endl;
        return false;
    }
    struct ifreq ifr; // to save the data of one interface. used in ioctl().

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1)
    {
        std::cout<<"Error in CommInterface::getLocalIp(): creating socket has error."<<std::endl;
        return false;
    }
    
    memset(&ifr, 0, sizeof(struct ifreq));
    strncpy(ifr.ifr_name, iface_name, IFNAMSIZ); // set the interface name.

    // The command SIOCGIGADDR means getting the address of interface. 
    // The output data type is struct ifreq.
    if (ioctl(sockfd, SIOCGIFADDR, &ifr) == -1)
    {
        std::cout<<"Error in CommInterface::getLocalIp(): ioctl has error."<<std::endl;
        return false;
    }

    *ip = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr); //convert ip to char*.
    std::cout<<"Host :"<<*ip<<std::endl;

    close(sockfd);
    return true;
}

//------------------------------------------------------------
// Function:  setMaxMsgSize
// Summary: set the maximum size of the message if you have a message
//          bigger than 1024*1024 bytes.
//          This function can be called after createChannel().
// In:      size -> the unit is byte. 1024*1024*2 = 2M. 
// Out:     None.
// Return:  true -> succeed to set.
//          false-> failed to set.
//------------------------------------------------------------
bool CommInterface::setMaxMsgSize(int size)
{
    if (error_flag_ != 0)
        return false;

    if (nn_setsockopt(fd_, NN_SOL_SOCKET, NN_RCVMAXSIZE, &size, sizeof(size)) < 0)
        return false;
    max_msg_size = size;
    return true;
}

//------------------------------------------------------------
// Function:  convertIpcUrl
// Summary: convert the input string to an available url.
//          This function is called in createChannel().
// In:      name -> the name of the channel. 
// Out:     url -> an available url to bind or connect.
// Return:  None.
//------------------------------------------------------------
void CommInterface::convertIpcUrl(const char *name, char *url)
{
    std::ostringstream oss;
    oss << "ipc:///tmp/ipc_msg_" << name << ".ipc";
    std::string ss = oss.str();
    memcpy(url, ss.c_str(), ss.size()+1);
}

//------------------------------------------------------------
// Function:  convertTcpUrl
// Summary: convert the input ip:port to an available url.
//          This function is called in createChannel().
// In:      name -> the ip:port of the channel. 
// Out:     url -> an available url to bind or connect.
// Return:  None.
//------------------------------------------------------------
void CommInterface::convertTcpUrl(const char *name, char *url)
{
    std::ostringstream oss;
    oss << "tcp://" << name;
    std::string ss = oss.str();
    memcpy(url, ss.c_str(), ss.size()+1);

}

//------------------------------------------------------------
// Function:  convertInprocUrl
// Summary: convert the input string to an available url.
//          This function is called in createChannel().
// In:      name -> the name of the channel. 
// Out:     url -> an available url to bind or connect.
// Return:  None.
//------------------------------------------------------------
void CommInterface::convertInprocUrl(const char *name, char *url)
{
    std::ostringstream oss;
    oss << "inproc://inproc_msg_" << name;
    std::string ss = oss.str();
    memcpy(url, ss.c_str(), ss.size()+1);
}


//------------------------------------------------------------
// Function:  bind
// Summary: bind to URL.
//          This function is called in createChannel().
// In:      url -> url to bind. 
// Out:     None.
// Return:  true -> succeed to bind.
//          false -> failed to bind.
//------------------------------------------------------------
bool CommInterface::bind(char *url)
{
    if (nn_bind(fd_, url) < 0)
    {
        std::cout<<"Error in CommInterface::bind()"<<": "<<nn_strerror(nn_errno())<<std::endl;
        return false;
    }
    return true;
}

//------------------------------------------------------------
// Function:  connect
// Summary: connect to URL.
//          This function is called in createChannel().
// In:      url -> url to connect. 
// Out:     None.
// Return:  true -> succeed to connect.
//          false -> failed to connect.
//------------------------------------------------------------
bool CommInterface::connect(char *url)
{
    if (nn_connect(fd_, url) < 0)
    {
        std::cout<<"Error in CommInterface::connect()"<<": "<<nn_strerror(nn_errno())<<std::endl;
        return false;
    }
    return true;
}

//------------------------------------------------------------
// Function:  setSubOpt
// Summary: the subscriber can receive all the message from 
//          the channel.This function is called in createChannel().
// In:      None. 
// Out:     None.
// Return:  true -> succeed to set the option.
//          false -> failed to set the option.
//------------------------------------------------------------
bool CommInterface::setSubOpt(void)
{
    if (nn_setsockopt(fd_, NN_SUB, NN_SUB_SUBSCRIBE, "", 0) < 0)
    {
        std::cout<<"Error in CommInterface::setSubOpt() to set SUB: "<<nn_strerror(nn_errno())<<std::endl;
        return false;
    };
    return true;
}

//------------------------------------------------------------
// Function:  closeChannel
// Summary: call this function in destructor of this class. 
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void CommInterface::closeChannel(void)
{
    if (error_flag_ == 0)
        nn_close(fd_);
}

} //namespace fst_comm_interface

#endif //MIDDLEWARE_TO_MEM_COMM_INTERFACE_CPP_
