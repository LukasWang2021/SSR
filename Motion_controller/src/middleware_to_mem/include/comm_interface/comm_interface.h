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
#include "error_code/error_code.h"
#include "struct_to_mem/struct_joint_command.h"
#include "struct_to_mem/struct_trajectory_segment.h"
#include "struct_to_mem/struct_feedback_joint_states.h"

#define IPC_REQ 48 /*the type of createChannel(), stands for "request"*/
#define IPC_REP 49 /*the type of createChannel(), stands for "response"*/
#define IPC_PUB 32 /*the type of createChannel(), stands for "publisher"*/
#define IPC_SUB 33 /*the type of createChannel(), stands for "subsriber"*/
#define IPC_WAIT 0 /*the flag for send() or recv()*/
#define IPC_DONTWAIT 1 /*the flag for send() or recv()*/

namespace fst_comm_interface
{

//------------------------------------------------------------
// The interface classs for communication between processes.
// Sample usage:
//   CommInterface comm;
//   ERROR_CODE_TYPE result = comm.createChannel(IPC_REQ, "test");
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
    // Summary: Create the channel for IPC communication
    // In:      type -> REQ:request, REP:response, PUB:publisher, SUB:subscriber
    //          name -> name for each channel
    // Out:     None
    // Return:  0 -> succeed to get handle.
    //          CREATE_CHANNEL_FAIL -> failed to create channel 
    //------------------------------------------------------------
    ERROR_CODE_TYPE createChannel(int type, const char *name);

    //------------------------------------------------------------
    // Function:  send
    // Summary: send data
    // In:      buff -> data to be sent.
    //          buff_size -> date size
    //          flag -> block or not(IPC_WAIT/IPC_DONTWAIT).
    // Out:     None
    // Return:  0 -> succeed to send data.
    //          SEND_MSG_FAIL -> failed to send data.
    //------------------------------------------------------------
    ERROR_CODE_TYPE send(const void *buf, int buf_size, int flag);

    //------------------------------------------------------------
    // Function:  send
    // Summary: send data by string
    // In:      str -> string to be sent.
    //          flag -> block or not(IPC_WAIT/IPC_DONTWAIT).
    // Out:     None
    // Return:  0 -> succeed to send data.
    //          SEND_MSG_FAIL -> failed to send data.
    //------------------------------------------------------------
    ERROR_CODE_TYPE send(std::string str, int flag);

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
    ERROR_CODE_TYPE recv(void *buf, int buf_size, int flag);

    //------------------------------------------------------------
    // Function:  recv
    // Summary: receive data
    // In:      string -> string to be read.
    //          flag -> block or not(IPC_WAIT/IPC_DONTWAIT).
    // Out:     None
    // Return:  0 -> succeed to get data.
    //          RECV_MSG_FAIL -> failed to get data.
    //------------------------------------------------------------
    ERROR_CODE_TYPE recv(std::string *str, int flag);
    
    //The maximum string length of the channel name
    static const int NAME_SIZE = 64;

    //The maximum size of the data to be delivered
    static const int MAX_MSG_SIZE = 2048;
        
private:
    int fd_;

    // Record error status.
    ERROR_CODE_TYPE error_flag_;

    //------------------------------------------------------------
    // Function:  close
    // Summary: closeChannel
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void closeChannel(void);

       
};
} //namespace fst_comm_interface

#endif //MIDDLEWARE_TO_MEM_COMM_INTERFACE_H_
