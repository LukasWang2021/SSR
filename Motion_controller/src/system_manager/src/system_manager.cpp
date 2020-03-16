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

#include "system_manager.h"
#include "file_operations.h"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "error_monitor.h"

using namespace fst_base;
using namespace fst_ctrl;

SystemManager::SystemManager():
    param_ptr_(NULL),
    log_ptr_(NULL),
    is_running_(false),
    command_type_(0)
{   
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new SystemManagerParam();
    FST_LOG_INIT("system_manager");
}

SystemManager::~SystemManager()
{
    thread_ptr_.join();
    if(log_ptr_ != NULL){
        delete log_ptr_;
        log_ptr_ = NULL;
    }
    if(param_ptr_ != NULL){
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

ErrorCode SystemManager::init(void)
{
    if(!param_ptr_->loadParam()){
        FST_ERROR("Failed to load system_manager component parameters");
    }else{
        FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
        FST_INFO("Success to load system_manager component parameters");
    }

    if(getenv("ROBOT_DATA_PREFIX") != NULL)
		files_manager_data_path_ = std::string(getenv("ROBOT_DATA_PREFIX")); //ROBOT_DATA_PREFIX=/root
	else
		files_manager_data_path_ = "/root"; //default

    install_path_ = param_ptr_->install_path_;
    config_path_ = param_ptr_->config_path_;
    runtime_path_ = param_ptr_->runtime_path_;
    backup_install_path_ = files_manager_data_path_ + param_ptr_->backup_install_path_;
    backup_config_path_ = files_manager_data_path_ + param_ptr_->backup_config_path_;
    backup_runtime_path_ = files_manager_data_path_ + param_ptr_->backup_runtime_path_;
    restore_install_path_ = files_manager_data_path_ + param_ptr_->restore_install_path_;
    restore_config_path_ = files_manager_data_path_ + param_ptr_->restore_config_path_;
    restore_runtime_path_ = files_manager_data_path_ + param_ptr_->restore_runtime_path_;
    password_ = param_ptr_->password_;
    
    return SUCCESS;
}

void SystemManager::controllerBackup(void)
{
    command_type_ = INSTALL_BACKUP;
    if(!thread_ptr_.run(systemManagerRoutineThreadFunc, this, 20))
    {
        FST_ERROR("Failed to open system_manager software backup thread");
        ErrorMonitor::instance()->add(SYS_START_THREAD_FAIL);
    }
}

void SystemManager::controllerRestore(void)
{
    command_type_ = INSTALL_RESTORE;
    if(!thread_ptr_.run(systemManagerRoutineThreadFunc, this, 20))
    {
        FST_ERROR("Failed to open system_manager software restore thread");
        ErrorMonitor::instance()->add(SYS_START_THREAD_FAIL);
    }
}


void SystemManager::threadExecuteCommand(void)
{
    setRunning(true);
    switch(command_type_)
    {
        case INSTALL_BACKUP:
        {
            compress(install_path_, backup_install_path_);
            compress(config_path_, backup_config_path_);
            compress(runtime_path_, backup_runtime_path_);
            break;
        }
        case INSTALL_RESTORE:
        {
            extract(restore_install_path_, install_path_);
            extract(restore_config_path_, config_path_);
            extract(restore_runtime_path_, runtime_path_);
            break;
        }
        default: break;
    }
    setRunning(false);
}

void SystemManager::setRunning(bool is_running)
{
    is_running_ = is_running;
}

bool SystemManager::getRunning(void)
{
    return is_running_;
}

ErrorCode SystemManager::compress(std::string source, std::string destination)
{
    // check the available disk size for compression. 
    if (!checkSize(source))
    {
        FST_INFO("compress from %s to %s is failed", source.c_str(), destination.c_str());
        FST_ERROR("software backup: No enough space.");
        ErrorMonitor::instance()->add(SYS_NO_FREE_DISK);
        return SYS_NO_FREE_DISK;
    }

    // check if destination directory is existed.
    std::string dest = destination;
    dest = dest.substr(0, dest.rfind('/') + 1);                   
    if (access(dest.c_str(), 0) == -1)
    {
        if (mkdir(dest.c_str(), 0777) != 0)
        {
            FST_INFO("compress from %s to %s is failed", source.c_str(), destination.c_str());
            FST_ERROR("compress: destination path is not existed.");
            ErrorMonitor::instance()->add(SYS_COMPRESS_FILE_FAIL);
            return SYS_COMPRESS_FILE_FAIL;
        }
        chmod(dest.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
    }

    // compress the zip and output to the destination. 
    const char *src = source.c_str();                 
    ErrorCode result = FileOperations::archiveCreate(&src, destination.c_str(), password_.c_str());
    if (result != FST_SUCCESS)
    {
        // delete the failed zip. 
        remove(destination.c_str());                  
        ErrorMonitor::instance()->add(SYS_COMPRESS_FILE_FAIL);
        FST_INFO("compress from %s to %s is failed", source.c_str(), destination.c_str());
        FST_ERROR("compress: archive create is failed");
        return SYS_COMPRESS_FILE_FAIL;
    }

    FST_INFO("compress from %s to %s is succeed", source.c_str(), destination.c_str());
    return SUCCESS;
}

ErrorCode SystemManager::extract(std::string source, std::string destination)
{
    // check source exist.
    if (access(source.c_str(), 0) == -1)
    {
        FST_INFO("extract source %s is not existed", source.c_str());
        return SYS_EXTRACT_ARCHIVE_FAIL;
    }

    // check the available disk size for extractioin. 
    if (!checkSize(source))
    {
        FST_INFO("extract from %s to %s is failed", source.c_str(), destination.c_str());
        FST_ERROR("software extract: No enough space.");
        ErrorMonitor::instance()->add(SYS_NO_FREE_DISK);
        return SYS_NO_FREE_DISK;
    }

    //extract the zip and output to the destination. 
    std::string dest = destination.c_str();    
    dest = dest.substr(0, dest.rfind('/') + 1);               
    ErrorCode result = FileOperations::archiveExtract(source.c_str(), dest.c_str(), password_.c_str());
    if (result != FST_SUCCESS)
    {
        // delete the failed zip. 
        remove(destination.c_str());                  
        ErrorMonitor::instance()->add(SYS_COMPRESS_FILE_FAIL);
        FST_INFO("extract from %s to %s is failed", source.c_str(), destination.c_str());
        FST_ERROR("extract: archive extract is failed");
        return SYS_EXTRACT_ARCHIVE_FAIL;
    }

    // delete the extract file. 
    remove(source.c_str());

    FST_INFO("extract from %s to %s is succeed", source.c_str(), destination.c_str());
    return SUCCESS;
}



bool SystemManager::checkSize(std::string path)
{
    /* obtain the mode of the file. */
    struct stat info;
    /* obtain the size of the files. */
    long long total_size = 0;
    if (access(path.c_str(), 0) == -1)
            return false;
    lstat(path.c_str(), &info);
    if (S_ISDIR(info.st_mode))
        total_size = FileOperations::getDirSize(path.c_str());
    else if (S_ISREG(info.st_mode))
        total_size = FileOperations::getFileSize(path.c_str());

    /* obtain the free size of the disk. */
    long long free_size = FileOperations::getFreeDiskSize();

    if (total_size < 0 || free_size < 0 || free_size <= total_size * 2)
        return false;
    return true;

}

// thread function
void* systemManagerRoutineThreadFunc(void* arg)
{
    std::cout<<"system_manager routine thread running"<<std::endl;
    /* start time messurement. */
    time_t t_start, t_end;
    t_start = clock();

    fst_ctrl::SystemManager* system_manager = static_cast<fst_ctrl::SystemManager*>(arg);
    system_manager->threadExecuteCommand();

    /* end time measurement. */
    t_end = clock();
    printf("system_manager routine(time:%f s) thread exit", (double)(t_end - t_start)/CLOCKS_PER_SEC);
    return NULL;
}


#endif //SYSTEM_MANAGER_SYSTEM_MANAGER_CPP_

