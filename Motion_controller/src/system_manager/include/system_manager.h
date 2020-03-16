/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       system_manager.h
Author:     Feng.Wu 
Create:     27-Feb-2019
Modify:     27-Feb-2019
Summary:    
**********************************************/
#ifndef SYSTEM_MANAGER_SYSTEM_MANAGER_H_
#define SYSTEM_MANAGER_SYSTEM_MANAGER_H_

#include "error_code.h"
#include "system_manager_param.h"
#include "common_log.h"
#include "error_code.h"
#include "thread_help.h"

namespace fst_ctrl
{

enum SystemCommand
{
    INSTALL_BACKUP = 1,
    INSTALL_RESTORE = 2,
};

class SystemManager
{
public:
  
    SystemManager();

    ~SystemManager();

    ErrorCode init(void);

    void controllerBackup(void);
    void controllerRestore(void);

    void threadExecuteCommand(void);

    void setRunning(bool is_running);
    bool getRunning(void);
 
private:
    ErrorCode compress(std::string source, std::string destination);
    ErrorCode extract(std::string source, std::string destination);

    bool checkSize(std::string path);

    SystemManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;

    fst_base::ThreadHelp thread_ptr_;
    bool is_running_;
    int command_type_;
    std::string files_manager_data_path_ = "";

    std::string install_path_ = "";
    std::string config_path_ = "";
    std::string runtime_path_ = "";
    std::string backup_install_path_ = "";
    std::string backup_config_path_ = "";
    std::string backup_runtime_path_ = "";
    std::string restore_install_path_ = "";
    std::string restore_config_path_ = "";
    std::string restore_runtime_path_ = "";
    std::string password_ = "";
};
}

void* systemManagerRoutineThreadFunc(void* arg);

#endif
