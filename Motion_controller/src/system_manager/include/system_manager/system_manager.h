/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       system_manager.h
Author:     Feng.Wu 
Create:     12-Jun-2017
Modify:     12-Jun-2017
Summary:    
**********************************************/
#ifndef SYSTEM_MANAGER_SYSTEM_MANAGER_H_
#define SYSTEM_MANAGER_SYSTEM_MANAGER_H_

#include <signal.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "system_manager/system_execute.h"
#include "system_manager/system_manager_error_code.h"
#include "comm_interface/comm_interface.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"

namespace fst_system_manager
{

enum SystemCommand
{
    NONE = 0xB0,
    CHECK_STATUS_SID = 0xB1,
    FTP_ON = 0xB2,
    FTP_OFF = 0xB3,
    CONFIG_BACKUP = 0xB4,
    RESTORE = 0xB5,
    ALL_UPGRADE = 0xB6,
    ALL_BACKUP = 0xB7,
    LAST_ITEM = 0xB8,
};

struct SharedData
{
    SystemCommand command;
    U64 error;
    bool execute_enable;
    bool ftp_enable;
};

class SystemManager
{
public:
    //------------------------------------------------------------
    // Function:  SystemManager
    // Summary: The constructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    SystemManager();

    //------------------------------------------------------------
    // Function:  ~SystemManager
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    ~SystemManager();

    //------------------------------------------------------------
    // Function:  init
    // Summary: create a communication channel, and start a thread
    //          to communicate with tp_interface.
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    static void init(void);

    //------------------------------------------------------------
    // Function:  startCommThread
    // Summary: create a thread to communicate with tp_interface.
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    static void startCommThread(void);

    //------------------------------------------------------------
    // Function:  runCommThread
    // Summary: the main function of the thread.
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    static void runCommThread(void);

    //------------------------------------------------------------
    // Function:  receiveCommand
    // Summary: receive command from tp_interface.
    // In:      None
    // Out:     None
    // Return:  true -> receive a command.
    //          false -> no command received.
    //------------------------------------------------------------
    static bool receiveCommand(SystemCommand &command);

    //------------------------------------------------------------
    // Function:  replyCommand
    // Summary: reply command to tp_interface.
    // In:      None
    // Out:     None
    // Return:  true -> reply success.
    //          false -> reply fail.
    //------------------------------------------------------------
    static bool replyCommand(SystemCommand command);

    //------------------------------------------------------------
    // Function:  executeCommand
    // Summary: execute the command.
    // In:      None
    // Out:     None
    // Return:  true -> execute success.
    //          false -> execute fail.
    //------------------------------------------------------------
    static bool executeCommand(void);

    //------------------------------------------------------------
    // Function:  sigHandler
    // Summary: signal handler
    // In:      system signal
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    static void sigHandler(int sig);

    //------------------------------------------------------------
    // Function:  teardown
    // Summary: The destructor
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    static void teardown(void);

    // the loop time of the child thread.
    static const int COMM_LOOP = 100000;
    
    static volatile int exit_flag_;

private:

    // child thread to communicate with tp_interface.
    static boost::thread comm_thread_;

    // shared data between threads.
    static SharedData shared_data_;

    // mutex lock
    static boost::mutex mutex_;

    // channel to communicate with tp_interface.
    static fst_comm_interface::CommInterface comm_system_;

};
}



#endif
