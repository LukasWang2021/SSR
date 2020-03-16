/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       program_launching.h
Author:     Feng.Wu
Create:     21-Nov-2018
Modify:     21-Nov-2018
Summary:    dealing with IO macro
**********************************************/

#ifndef PROGRAM_LAUNCHING_H
#define PROGRAM_LAUNCHING_H

#include <vector>
#include <thread>
#include <mutex>
#include <string>
#include <map>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "common_log.h"
#include "error_code.h"
#include "program_launching_param.h"
#include "io_mapping.h"
#include "process_comm.h"

namespace fst_ctrl
{

typedef struct _MacroConfigInfo
{
    char userDefined[64];
    char macroProgram[64];
    char ioType[64];
    uint32_t ioPort;
}MacroConfigInfo;

typedef enum
{
    PROGRAM_LOCAL_TRIGGER = 1,
    PROGRAM_LAUNCH_CODE_FULL = 2,
    PROGRAM_MACRO_TRIGGER = 3,
    PROGRAM_LAUNCH_CODE_SIMPLE = 4,
}ProgramLaunchMode;

typedef struct _EnableLaunchInfo
{
    uint8_t pre_value;
    uint8_t port_value;
    uint8_t count;
    bool count_start;
    bool enable; 
}EnableLaunchInfo;

class ProgramLaunching
{
public:
    ProgramLaunching();
    ~ProgramLaunching();

    ErrorCode init(IoMapping* io_mapping_ptr, fst_base::ControllerClient* controller_client_ptr);

    ErrorCode updateFileLaunchMode(void);
    ErrorCode updateFileMacroConfig(void);
    int getLaunchMode(void);
    void setLaunchMode(int value);

    bool processMacro(void);
	
private:
    void loadProgramsPath(void);
	char * getProgramsPath(void);

    // parse macro_io_launching.json
    void generateMacroConfigInfo(MacroConfigInfo &objInfo);
	bool parseMacroConfigObject(cJSON *contentObject);
	bool parseMacroConfig(char * data);
	bool openFileMacroConfig(char *filename);
    void printMacroConfigInfo(void);

    //parse launch_mode_setting.json
    bool parseLaunchMode(char * data, int &mode);
    bool openFileLaunchMode(char *filename, int &mode);

    // process macro launching and check rising edge  
    bool isRisingEdge(int index);
    bool isRisingEdgeAntiShake(int index);
    void sendInterpreterStart(int index);
    void initLaunchInfo(void);

    ProgramLaunchingParam* param_ptr_;
    fst_log::Logger* log_ptr_;
	IoMapping* io_mapping_ptr_;
    fst_base::ControllerClient* controller_client_ptr_;
    std::string files_manager_data_path_ = "";

    // store the info from macro_io_launch.json
    std::vector<MacroConfigInfo> macro_vector_; 
    int macro_num_;  
    EnableLaunchInfo* launch_info_;

    // store the info from launch_mode_setting.json
    int launch_mode_setting_;                     

};

}


#endif

