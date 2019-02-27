/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       comm_interface.h
Author:     Feng.Wu 
Create:     04-Nov-2016
Modify:     08-Dec-2016
Summary:    lib to communicate between processes
**********************************************/

#ifndef MIDDLEWARE_TO_MEM_COMM_INTERFACE_H_
#define MIDDLEWARE_TO_MEM_COMM_INTERFACE_H_

#include <vector>
#include <string>
#include "error_code.h"
#include "struct_to_mem/struct_joint_command.h"
#include "struct_to_mem/struct_trajectory_segment.h"
#include "struct_to_mem/struct_feedback_joint_states.h"

#define COMM_REQ 1
#define COMM_REP 2
#define COMM_PUB 3
#define COMM_SUB 4
#define COMM_IPC 21
#define COMM_TCP 22
#define COMM_INPROC 23
#define COMM_WAIT 0
#define COMM_DONTWAIT 1

namespace fst_comm_interface
{

//------------------------------------------------------------
// Function:  getVersion
// Summary: get the version. 
// In:      None
// Out:     None
// Return:  std::string -> the version.
//------------------------------------------------------------
std::string getVersion(void);

//------------------------------------------------------------
// The interface classs for communication between processes.
// Sample usage:
//   CommInterface comm;
//   ERROR_CODE_TYPE result = comm.createChannel(COMM_REQ, COMM_IPC, "test");
//   ServiceRequest req = {JTAC_CMD_SID, ""};
//   ERROR_CODE_TYPE send = comm.send(&req, sizeof(req), IPC_DONTWAIT);
//   ServiceResponse resp = {0, ""};
//   ERROR_CODE_TYPE rec = comm.recv(&resp, sizeof(resp), IPC_WAIT);
//------------------------------------------------------------

class CommInterface
{
public:
    //------------------------------------------------------------
    // Function:  CommInterface
    // Summary: The constructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    CommInterface();

    //------------------------------------------------------------
    // Function:  ~CommInterface
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    ~CommInterface();

    //------------------------------------------------------------
    // Function:  createChannel
    // Summary: Create the channel for process communication
    // In:      type -> COMM_REQ:request, COMM_REP:response, COMM_PUB:publisher, COMM_SUB:subscriber
    //          transport -> COMM_IPC, COMM_TCP, COMM_INPROC.
    //          name -> name for each channel
    // Out:     None
    // Return:  FST_SUCCESS -> succeed to get handle.
    //          CREATE_CHANNEL_FAIL -> failed to create channel 
    //------------------------------------------------------------
    ErrorCode createChannel(int protocol, int transport, const char *name);

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
    ErrorCode send(const void *buf, int buf_size, int flag);

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
    ErrorCode send(std::string str, int flag);

    //------------------------------------------------------------
    // Function:  recv
    // Summary: receive data
    // In:      buff -> data to be read.
    //          buff_size -> date size
    //          flag -> block or not(IPC_WAIT/IPC_DONTWAIT).
    // Out:     None
    // Return:  0 -> succeed to get data.
    //          RECV_MSG_FAIL -> failed to get data.
    //------------------------------------------------------------
    ErrorCode recv(void *buf, int buf_size, int flag);

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
    ErrorCode recv(std::string *str, int flag);
  
    //------------------------------------------------------------
    // Function:  getLocalIP
    // Summary: get the local IP address
    // In:      iface_name -> interface name such as "eth0", "wlan0" or "lo".
    // Out:     *ip -> the IP adress according to the interface name.
    // Return:  true -> succeed to get IP address.
    //          false -> failed to get IP address.
    //------------------------------------------------------------
    static bool getLocalIP(char **ip, const char *iface_name = "eth0");

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
    bool setMaxMsgSize(int size);


    //The maximum string length of the channel name
    static const int NAME_SIZE = 32;

    //the array size to store URL string. 
    static const int URL_SIZE = 64;

    //The maximum size of the data to be delivered
    static const int MAX_MSG_SIZE = 1024*1024;
        
private:
    int fd_;

    int max_msg_size;

    static int obj_num_;

    // Record error status.
    ErrorCode error_flag_;

    //------------------------------------------------------------
    // Function:  convertIpcUrl
    // Summary: convert the input string to an available url.
    //          This function is called in createChannel().
    // In:      name -> the name of the channel. 
    // Out:     url -> an available url to bind or connect.
    // Return:  None.
    //------------------------------------------------------------
    void convertIpcUrl(const char *name, char *url);

    //------------------------------------------------------------
    // Function:  convertTcpUrl
    // Summary: convert the input ip:port to an available url.
    //          This function is called in createChannel().
    // In:      name -> the ip:port of the channel. 
    // Out:     url -> an available url to bind or connect.
    // Return:  None.
    //------------------------------------------------------------
    void convertTcpUrl(const char *name, char *url);

    //------------------------------------------------------------
    // Function:  convertInprocUrl
    // Summary: convert the input string to an available url.
    //          This function is called in createChannel().
    // In:      name -> the name of the channel. 
    // Out:     url -> an available url to bind or connect.
    // Return:  None.
    //------------------------------------------------------------
    void convertInprocUrl(const char *name, char *url);

    //------------------------------------------------------------
    // Function:  bind
    // Summary: bind to URL.
    //          This function is called in createChannel().
    // In:      url -> url to bind. 
    // Out:     None.
    // Return:  true -> succeed to bind.
    //          false -> failed to bind.
    //------------------------------------------------------------
    bool bind(char *url);

    //------------------------------------------------------------
    // Function:  connect
    // Summary: connect to URL.
    //          This function is called in createChannel().
    // In:      url -> url to connect. 
    // Out:     None.
    // Return:  true -> succeed to connect.
    //          false -> failed to connect.
    //------------------------------------------------------------
    bool connect(char *url);
    
    //------------------------------------------------------------
    // Function:  setSubOpt
    // Summary: the subscriber can receive all the message from 
    //          the channel.This function is called in createChannel().
    // In:      None. 
    // Out:     None.
    // Return:  true -> succeed to set the option.
    //          false -> failed to set the option.
    //------------------------------------------------------------
    bool setSubOpt(void);

    //------------------------------------------------------------
    // Function:  closeChannel
    // Summary: call this function in destructor of this class. 
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void closeChannel(void);
       
};


} //namespace fst_comm_interface

#endif //MIDDLEWARE_TO_MEM_COMM_INTERFACE_H_
