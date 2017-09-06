/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       system_manager.cpp
Author:     Feng.Wu 
Create:     12-Jun-2017
Modify:     31-July-2017
Summary:    
**********************************************/
#ifndef SYSTEM_MANAGER_SYSTEM_MANAGER_CPP_
#define SYSTEM_MANAGER_SYSTEM_MANAGER_CPP_

#include "system_manager/system_manager.h"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

namespace fst_system_manager
{

using namespace fst_system_execute;

// child thread to communicate with tp_interface.
boost::thread SystemManager::comm_thread_;
// shared data between threads.
SharedData SystemManager::shared_data_ = {NONE, FST_SUCCESS, false, false};
// mutex lock
boost::mutex SystemManager::mutex_;
// channel to communicate with tp_interface.
fst_comm_interface::CommInterface SystemManager::comm_system_;
volatile int SystemManager::exit_flag_ = 0;

//------------------------------------------------------------
// Function:  SystemManager
// Summary: The constructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
SystemManager::SystemManager()
{   
}

//------------------------------------------------------------
// Function:  ~SystemManager
// Summary: The destructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
SystemManager::~SystemManager()
{
    comm_thread_.interrupt();
    comm_thread_.join();
}

//------------------------------------------------------------
// Function:  init
// Summary: create a communication channel, and start a thread
//          to communicate with tp_interface.
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
void SystemManager::init(void)
{
    /* create channel with tp_interface. */
    U64 result = comm_system_.createChannel(COMM_REP, COMM_IPC, "system");
    if (result == CREATE_CHANNEL_FAIL)
    {
        std::cout<<"Error in SystemManager::init():"
            <<"Fail to create the communication channel."<<std::endl;
    }
    result = SystemExecute::init();
    if (result != FST_SUCCESS)
        shared_data_.error = SYS_INIT_FAIL;

    /* disable ftp service. */
    result = SystemExecute::stopFTP();
    if (result != FST_SUCCESS)
        shared_data_.error = SYS_INIT_FAIL;

    /* read the versions of all packages. */
    result = SystemExecute::getAllVersion();
    if (result != FST_SUCCESS)
        shared_data_.error = SYS_INIT_FAIL;

    /* start a thread to communicate with tp_interface. */
    startCommThread();
}

//------------------------------------------------------------
// Function:  startCommThread
// Summary: create a thread to communicate with tp_interface.
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
void SystemManager::startCommThread(void)
{
    comm_thread_ = boost::thread(runCommThread);
}

//------------------------------------------------------------
// Function:  runCommThread
// Summary: the main function of the thread.
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
void SystemManager::runCommThread(void)
{
    try
    {
        bool ret = false;
        SystemCommand command = NONE;
        while (true)
        {
            ret = receiveCommand(command);
            if (ret == true)
                replyCommand(command);
            // set interruption point.
            boost::this_thread::sleep(boost::posix_time::microseconds(COMM_LOOP));
        }
    }
    catch (boost::thread_interrupted &)
    {     
    }
}

//------------------------------------------------------------
// Function:  receiveCommand
// Summary: receive command from tp_interface.
// In:      None
// Out:     None
// Return:  true -> receive a command.
//          false -> no command received.
//------------------------------------------------------------
bool SystemManager::receiveCommand(SystemCommand &command)
{
    /* try to receive request from tp_interface. */
    ServiceRequest request;
    U64 result = comm_system_.recv(&request, sizeof(request), COMM_DONTWAIT);

    if (result == FST_SUCCESS)
    {
        command = (SystemCommand)request.req_id;
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        
        /* reply immediately if init fail. */
        if (shared_data_.error == SYS_INIT_FAIL)
            return true;

        /* reply if unrecognize service id. */
        if (command <= NONE || command >= LAST_ITEM)
        {
            shared_data_.error = SYS_UNRECOGNIZED_SERVICE_ID;
            return true;
        }
        /* check and execute the command from tp. */
        if (command != CHECK_STATUS_SID)
        {
            /* ftp should be ON before other execution commands. */
            if (shared_data_.ftp_enable  == false && command != FTP_ON && command != FTP_OFF)
            {
                shared_data_.error = SYS_FTP_ON_FAIL;
                return true;
            }
            /* a new execute command should not be sent before
               the last execution is finished. */
            if (shared_data_.execute_enable == true)
            {
                shared_data_.error = SYS_OPS_BUSY;
                return true;
            }
            /* execute commmand by setting enbale. */
            shared_data_.command = command;
            shared_data_.execute_enable = true;
            shared_data_.error = SYS_OPS_UNFINISHED;
        }
        //delete
//        std::cout<<"||--recv command="<<command<<". shared_Data command="<<shared_data_.command<<". error="
//            <<shared_data_.error<<". enable="<<shared_data_.execute_enable<<". ftp enable="<<shared_data_.ftp_enable<<std::endl;

        return true;
    }
    return false;
}

//------------------------------------------------------------
// Function:  replyCommand
// Summary: reply command to tp_interface.
// In:      None
// Out:     None
// Return:  true -> reply success.
//          false -> reply fail.
//------------------------------------------------------------
bool SystemManager::replyCommand(SystemCommand command)
{
    /* set the response vaule. */
    ServiceResponse response={0,""};
    response.res_id = command;
    U64 error;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        error = shared_data_.error;
        //delete
//        std::cout<<"||--reply command="<<command<<". shared_Data command="<<shared_data_.command<<". error="<<shared_data_.error
//            <<". enable="<<shared_data_.execute_enable<<". ftp enable="<<shared_data_.ftp_enable<<std::endl;
    }
    size_t size = 1;
    memcpy(&(response.res_buff[0]), &size, sizeof(size));
    memcpy(&(response.res_buff[8]), &error, sizeof(error));

    /* send the response to tp_interface. */
    U64 result = comm_system_.send(&response, sizeof(response), COMM_DONTWAIT);
    if (result != FST_SUCCESS)
        return false;
//    printf("system manager error:id = %d, %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n", response.res_id, (unsigned char)response.res_buff[7+8],(unsigned char)response.res_buff[6+8],(unsigned char)response.res_buff[5+8],(unsigned char)response.res_buff[4+8],(unsigned char)response.res_buff[3+8],(unsigned char)response.res_buff[2+8],(unsigned char)response.res_buff[1+8],(unsigned char)response.res_buff[0+8]);
    return true;
}

//------------------------------------------------------------
// Function:  executeCommand
// Summary: execute the command.
// In:      None
// Out:     None
// Return:  true -> execute success.
//          false -> execute fail.
//------------------------------------------------------------
bool SystemManager::executeCommand(void)
{
    SystemCommand command;
    bool ftp_enable;
    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        if (shared_data_.execute_enable == false)
            return false;
        command = shared_data_.command;
        ftp_enable = shared_data_.ftp_enable;
        //delete
//        std::cout<<"||--execute shared_Data command="<<shared_data_.command<<". error="<<shared_data_.error
//            <<". enable="<<shared_data_.execute_enable<<". ftp enable="<<shared_data_.ftp_enable<<std::endl;
    }

    std::vector<std::string> sources;
    std::string archive = SystemExecute::output_archive_;
    U64 result = FST_SUCCESS;    

    /* start time messurement. */
    time_t t_start, t_end;
    t_start = clock();

    switch (command)
    {
        case FTP_ON:
            result = SystemExecute::startFTP();
            ftp_enable = true;
            break;
        case FTP_OFF:
            result = SystemExecute::stopFTP();
            ftp_enable = false;
            break;
        case CONFIG_BACKUP:
            sources.push_back(SystemExecute::config_path_);
            result = SystemExecute::compress(sources, archive);
            break;
        case RESTORE:
            result = SystemExecute::extract(archive);
            break;
        case ALL_UPGRADE:
            result = SystemExecute::upgrade(archive);
            break;
        case ALL_BACKUP:
            sources.push_back(SystemExecute::os_path_);
            sources.push_back(SystemExecute::infra_path_);
            result = SystemExecute::compress(sources, archive);
            break;
        default:
            break;
    }

    /* end time measurement. */
    t_end = clock();
    std::cout<<"The execute time cost = "<<(double)(t_end - t_start)/CLOCKS_PER_SEC<<" s"<<std::endl;

    {
        boost::mutex::scoped_lock lock(mutex_); //------lock mutex-----//
        shared_data_.command = NONE;
        shared_data_.error = result;
        shared_data_.execute_enable = false;
        shared_data_.ftp_enable = ftp_enable;
        //delete
        std::cout<<"||--execute2 shared_Data command="<<shared_data_.command<<". error="<<shared_data_.error
            <<". enable="<<shared_data_.execute_enable<<". ftp enable="<<shared_data_.ftp_enable<<std::endl;
    }

    return true;
}

//------------------------------------------------------------
// Function:  sigHandler
// Summary: signal handler
// In:      system signal
// Out:     None
// Return:  None
//------------------------------------------------------------
void SystemManager::sigHandler(int sig)
{
    if(sig == SIGINT)
        exit_flag_ = 1;
}


void SystemManager::teardown(void)
{
    comm_thread_.interrupt();
    comm_thread_.join();
}

}

int main(int argc, char** argv)
{
    using namespace fst_system_manager;
    SystemManager::init();
    signal(SIGINT, SystemManager::sigHandler);
    while (!SystemManager::exit_flag_)
    {
        SystemManager::executeCommand();
        usleep(SystemManager::COMM_LOOP);
    }
    SystemManager::teardown();
    return 0;
}

#endif //SYSTEM_MANAGER_SYSTEM_MANAGER_CPP_

