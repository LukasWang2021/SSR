/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       program_launching.cpp
Author:     Feng.Wu
Create:     29-Jan-2019
Modify:     29-Jan-2019
Summary:    dealing with UIUO
**********************************************/

#include "program_launching.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include "error_monitor.h"
#include "forsight_inter_control.h"

using namespace std;
using namespace fst_base;
using namespace fst_ctrl;

ProgramLaunching::ProgramLaunching():
    param_ptr_(NULL),
    log_ptr_(NULL),
    io_mapping_ptr_(NULL),
    controller_client_ptr_(NULL),
    launch_info_(NULL),
    launch_mode_setting_(PROGRAM_LOCAL_TRIGGER)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new ProgramLaunchingParam();
    FST_LOG_INIT("program_launching");
}

ProgramLaunching::~ProgramLaunching()
{
    if(log_ptr_ != NULL){
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if(param_ptr_ != NULL){
        delete param_ptr_;
        param_ptr_ = NULL;
    }
 
    if(launch_info_ != NULL){
        delete[] launch_info_;
        launch_info_ = NULL;
    }

}

ErrorCode ProgramLaunching::init(IoMapping* io_mapping_ptr, fst_base::ControllerClient* controller_client_ptr)
{
    if(!param_ptr_->loadParam()){
        FST_ERROR("Failed to load program_launching component parameters");
        ErrorMonitor::instance()->add(PROGRAM_LAUNCHING_LOAD_PARAM_FAILED);
    }else{
        FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
        FST_INFO("Success to load program_launching component parameters");
    }

    io_mapping_ptr_ = io_mapping_ptr;
    controller_client_ptr_ = controller_client_ptr;

    loadProgramsPath();
    updateFileLaunchMode();
    FST_INFO("Launching mode setting = %d", launch_mode_setting_);

    updateFileMacroConfig();

    return SUCCESS;
}

//syn the file launch_mode_setting.json
ErrorCode ProgramLaunching::updateFileLaunchMode()
{
    char file_name[128];

	sprintf(file_name, "%s/launch_mode_setting.json", getProgramsPath());
	FST_INFO("The launch_mode_setting file is %s", file_name);
	bool ret = openFileLaunchMode(file_name, launch_mode_setting_);
	if (ret == false){
		FST_INFO("Failed to read LaunchMode parameters:%s", file_name);
        return PROGRAM_LANNCHING_LOAD_MODE_FILE_FAILED;
	}

    return SUCCESS;
}

// syn the file macro_io_launch.json
ErrorCode ProgramLaunching::updateFileMacroConfig()
{
    char file_name[128];
	macro_vector_.clear();

	sprintf(file_name, "%s/macro_io_launch.json", getProgramsPath());
	FST_INFO("The macro_config file is %s", file_name);
	bool ret = openFileMacroConfig(file_name);
	if (ret == false){
		FST_INFO("Failed to read MacroConfig parameters:%s", file_name);
        return PROGRAM_LAUNCHING_LOAD_MACRO_FILE_FAILED;
	}

    printMacroConfigInfo();

    if(launch_info_ != NULL){
        delete[] launch_info_;
        launch_info_ = NULL;
    }

    macro_num_ = macro_vector_.size();
    launch_info_ = new EnableLaunchInfo[macro_num_];
    initLaunchInfo();
   
    return SUCCESS;
}

// get launch mode
int ProgramLaunching::getLaunchMode(void)
{
    return launch_mode_setting_;
}

//set launch mode
void ProgramLaunching::setLaunchMode(int value)
{
    launch_mode_setting_ = value;
}

bool ProgramLaunching::processMacro(void)
{
    if (launch_mode_setting_ != PROGRAM_MACRO_TRIGGER)
        return false;
    
    ErrorCode err = SUCCESS;
    for (int i = 0; i < macro_num_; ++i)
    {
        if (strcasecmp(macro_vector_[i].ioType, "ui") == 0)
        {
            err = io_mapping_ptr_->getDIByBit(macro_vector_[i].ioPort, launch_info_[i].port_value);
            if (isRisingEdge(i))
            {
                sendInterpreterStart(i);
                return true;
            }         
        } else if (strcasecmp(macro_vector_[i].ioType, "di") == 0)
        {
            err = io_mapping_ptr_->getDIByBit(macro_vector_[i].ioPort, launch_info_[i].port_value);
            if (isRisingEdge(i))
            {
                sendInterpreterStart(i);
                return true;
            }
            
        } else if (strcasecmp(macro_vector_[i].ioType, "ri") == 0)
        {
            err = io_mapping_ptr_->getRIByBit(macro_vector_[i].ioPort, launch_info_[i].port_value);
            if (isRisingEdge(i))
            {
                sendInterpreterStart(i);
                return true;
            }
        }
    }
    return false;
}

// check if there is rising edge.
bool ProgramLaunching::isRisingEdge(int index)
{
    if (launch_info_[index].pre_value == 0 && launch_info_[index].port_value == 1)
    {
        launch_info_[index].pre_value = launch_info_[index].port_value;
        return true;   
    }
    launch_info_[index].pre_value = launch_info_[index].port_value;
    return false;
}

// check if there is rising edge considering signal shaking.
bool ProgramLaunching::isRisingEdgeAntiShake(int index)
{
    if (launch_info_[index].pre_value == 0 && launch_info_[index].port_value == 1)
    {
        launch_info_[index].count_start = true;
    }
    launch_info_[index].pre_value = launch_info_[index].port_value;

    if (launch_info_[index].count_start == true)
    {
        if(launch_info_[index].port_value == 1)
        {
            launch_info_[index].count++;
        }
        else 
        {
            launch_info_[index].count = 0;
            launch_info_[index].count_start = false;
        }

    }

    if (launch_info_[index].count >= param_ptr_->cycle_count_)
        return true;

    return false;
}

void ProgramLaunching::sendInterpreterStart(int index)
{
    FST_INFO("Macro start prgram: %s", macro_vector_[index].macroProgram);
    // controller_client_ptr_->start(std::string(macro_vector_[index].macroProgram));  
    InterpreterControl intprt_ctrl ;
    intprt_ctrl.autoMode = MACRO_TRIGGER_U;
    FST_ERROR("intprt_ctrl.autoMode %d", intprt_ctrl.autoMode);
	intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_START ;
    parseCtrlComand(intprt_ctrl, macro_vector_[index].macroProgram);
}

void ProgramLaunching::initLaunchInfo(void)
{
    for (int i = 0; i < macro_num_; ++i)
    {
        launch_info_[i].pre_value = 1;
        launch_info_[i].port_value = 1;
        launch_info_[i].count = 0;
        launch_info_[i].count_start = false;
    }
}
