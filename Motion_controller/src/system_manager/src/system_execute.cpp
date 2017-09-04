/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       system_execute.cpp
Author:     Feng.Wu 
Create:     16-Jun-2017
Modify:     31-July-2017
Summary:    
**********************************************/
#ifndef SYSTEM_MANAGER_SYSTEM_EXECUTE_CPP_
#define SYSTEM_MANAGER_SYSTEM_EXECUTE_CPP_

#include "system_manager/system_execute.h"
#include "system_manager/file_operations.h"
#include <parameter_manager/parameter_manager_param_group.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <sys/statfs.h>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

namespace fst_system_execute
{

using namespace fst_file_operations;

// the robot software paths.
std::string SystemExecute::os_path_ = "";
// the boot image path.
std::string SystemExecute::infra_path_ = "/mnt/infra";
// the configuration file.
std::string SystemExecute::config_path_ = "share/configuration";
// the output zip of the robot software.
std::string SystemExecute::os_name_ = "OS.tgz";
// the output zip of the boot image.
std::string SystemExecute::infra_name_ = "infra.tgz";
// the output zip of the configuration file.
std::string SystemExecute::config_name_ = "config.tgz";
// the final output zip.
std::string SystemExecute::output_archive_ = "fst_control.tgz";

// the vector to store the version name.
std::vector<std::string> SystemExecute::version_name_;
// the map between package name and version numbers.
std::map<std::string, std::string> SystemExecute::version_num_map_;
// the map between the software path and output target
std::map<std::string, std::string> SystemExecute::path_map_;
// the path to the version of the software.
std::string SystemExecute::version_path_ = "share/version";
// the path to the temporay configuration path.
std::string SystemExecute::config_backup_ = "config_backup";
// the configuration path include machine and configurable directories.
std::string SystemExecute::config_machine_ = "machine";
std::string SystemExecute::config_configurable_ = "configurable";
// the ftp path including usb, compress and extract paths.
std::string SystemExecute::ftp_path_ = "/media/ftp";
std::string SystemExecute::usb_path_ = ftp_path_ + "/usb";
std::string SystemExecute::compress_path_ = ftp_path_ + "/fst_compress";
std::string SystemExecute::extract_path_ = ftp_path_ + "/fst_extract";
// the temporary path for compression or extraction.
std::string SystemExecute::temp_path_ = "/tmp/fst_backup";

//------------------------------------------------------------
// Function:  SystemExecute
// Summary: The constructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
SystemExecute::SystemExecute()
{
}

//------------------------------------------------------------
// Function:  ~SystemExecute
// Summary: The destructor of class
// In:      None
// Out:     None
// Return:  None 
//------------------------------------------------------------
SystemExecute::~SystemExecute()
{

}

//------------------------------------------------------------
// Function:  init
// Summary: initialize some map and create some directories.
// In:      None
// Out:     None
// Return:  FST_SUCCESS -> success.
//          SYS_MKDIR_FAIL -> fail.
//          SYS_READ_VERSION_FAIL -> fail.
//------------------------------------------------------------
U64 SystemExecute::init(void)
{
    /* check if ftp path is exist. */
    if (access(ftp_path_.c_str(), 0) == -1)
    {
        if (mkdir(ftp_path_.c_str(), 0777) != 0)
            return SYS_INIT_FAIL;
//        chmod(ftp_path_.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
        chmod(ftp_path_.c_str(), 0777);
    }
    if (access(usb_path_.c_str(), 0) == -1)
    {
        if (mkdir(usb_path_.c_str(), 0777) != 0)
            return SYS_INIT_FAIL;
        chmod(usb_path_.c_str(), 0777);
    }
    if (access(compress_path_.c_str(), 0) == -1)
    {
        if (mkdir(compress_path_.c_str(), 0777) != 0)
            return SYS_INIT_FAIL;
        chmod(compress_path_.c_str(), 0777);
    }
    if (access(extract_path_.c_str(), 0) == -1)
    {
        if (mkdir(extract_path_.c_str(), 0777) != 0)
            return SYS_INIT_FAIL;
        chmod(extract_path_.c_str(), 0777);
    }

    /* covert the relative path to absolute path. */
    char path[256];
    char *p = FileOperations::getExePath(path, sizeof(path));
    if (p == NULL)
        return SYS_INIT_FAIL;
    boost::filesystem::path executable(path);
    os_path_ = executable.parent_path().parent_path().parent_path().string(); // <absolute path>/install
    config_path_ = os_path_ + "/" + config_path_;         // <absolute path>/install/share/configuration
    version_path_ = os_path_ + "/" + version_path_;       // <absolute path>/install/share/version
    config_backup_ = executable.parent_path().parent_path().parent_path().parent_path().string() + "/" + config_backup_; 
 
    /* map the source path to the archive in /tmp path. */
    path_map_[os_path_] = temp_path_ + "/" + os_name_;         // /tmp/fst_backup/OS.tgz
    path_map_[infra_path_] = temp_path_ + "/" + infra_name_;   // /mnt/infra <-> /tmp/fst_backup/infra.tgz
    path_map_[config_path_] = temp_path_ + "/" + config_name_; // /tmp/fst_backup/config.tgz

    /* map the archive name to the original path.*/
    path_map_[os_name_] = os_path_;           // OS.tgz     <->  <absolute path>/install
    path_map_[infra_name_] = infra_path_;     // infra.tgz  <->  /mnt/infra
    path_map_[config_name_] = config_path_;   // config.tgz <->  <absolute path>/install/share/configuration

    /* disable ftp service. */
    U64 result = stopFTP();
    if (result != FST_SUCCESS)
        return result;

    /* read the versions of all packages. */
    result = getAllVersion();
    if (result != FST_SUCCESS)
        return result;

    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  getAllVersion
// Summary: read the version files.
// In:      None
// Out:     None
// Return:  FST_SUCCESS -> success.
//          SYS_READ_VERSION_FAIL -> fail.
//------------------------------------------------------------
U64 SystemExecute::getAllVersion(void)
{
    /* get all the file names in the directory. */
    std::string dir_path = version_path_;
    std::vector<std::string> files;
    files = FileOperations::getFilesName(dir_path);
    for (unsigned int i = 0; i < files.size(); i++)
    {
        /* read all the content in the file. */
        std::string file = dir_path + "/" + files[i];
        std::ifstream in(file);
        std::string line;
        if (!in.is_open())
        {
            std::cout<<"Error in SystemExecute::getALlVersion():"
                <<"Fail open file<"<<file<<">"<<std::endl;
            return SYS_READ_VERSION_FAIL;
        }
        while (!in.eof())
        {
            getline(in, line);
            if (line.length() != 0)
                split(line, ":");
        }     
    }
    /* print all the versions. */
    for (unsigned int i = 0; i < version_name_.size(); i++)
    {
        std::cout<<"<"<<version_name_[i]<<":"
            <<version_num_map_[version_name_[i]]<<">"<<std::endl;
    }
    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  split
// Summary: get the package name and version number by splitting
//          the string read from the version file.
// In:      str -> a string read from the version file.
//          pattern -> the symbol to split a string.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void SystemExecute::split(std::string str, std::string pattern)
{
    std::string::size_type pos;
    str += pattern;
    unsigned int size = str.size();
    for (unsigned int i = 0; i < size; i++)
    {
        pos = str.find(pattern,i);
        if (pos < size)
        {
            std::string s = str.substr(i,pos-i);
            /* the first string is the name of software package. */
            /* the second string is the version number. */
            if (i == 0)
            {
                version_name_.push_back(s);
            }
            else
            {
                int v_pos = version_name_.size() - 1;
                version_num_map_[version_name_[v_pos]] = s;
            }
            i = pos + pattern.size() - 1;
        }
    }
}

//------------------------------------------------------------
// Function:  checkSize
// Summary: check if there is enough disk space to compress or
//          extract the zip.
// In:      the source path which is going to be compressed or extracted.
// Out:     None
// Return:  true -> enough space.
//          false -> not enough space.
//------------------------------------------------------------
bool SystemExecute::checkSize(std::vector<std::string> paths)
{ 
    /* obtain the mode of the file. */
    struct stat info;
    /* obtain the size of the files. */
    long long total_size = 0;
    for (unsigned int i = 0; i < paths.size(); i++)
    {
        lstat(paths[i].c_str(), &info);
        if (S_ISDIR(info.st_mode))
            total_size += FileOperations::getDirSize(paths[i].c_str());
        else if (S_ISREG(info.st_mode))
            total_size += FileOperations::getFileSize(paths[i].c_str());
    }
    /* obtain the free size of the disk. */
    long long free_size = FileOperations::getFreeDiskSize();

    if (total_size < 0 || free_size < 0 || free_size <= total_size * 2)
        return false;
    return true;
}

//------------------------------------------------------------
// Function:  checkConfigIntact
// Summary: check if there the configuration files are intact.
// In:      None
// Out:     None
// Return:  true -> intact.
//          false -> damaged.
//------------------------------------------------------------
bool SystemExecute::checkConfigIntact(void)
{
    fst_parameter::ParamGroup param;
    std::vector<std::string> dirs;
    /* traverse the configuration directory. */
    dirs = FileOperations::getDirsName(config_path_); // <absolute path>/install/share/configuration
    for (unsigned int i = 0; i < dirs.size(); i++)
    {
        std::string dir = config_path_;
        dir += "/" + dirs[i];
        std::vector<std::string> files;
        /* traverse the configuration files. */
        files = FileOperations::getFilesName(dir);
        for (unsigned int j = 0; j < files.size(); j++)
        {
            /* subtract the last string from '.'.
             pass the none .yaml file. */
            std::string file = files[j];
            std::size_t found = file.rfind('.');
            if (found == std::string::npos)
                continue;
            file = file.substr(found);
            if (strcmp(file.c_str(), ".yaml") != 0)
                continue;
            
            /* load file when the postfix is ".yaml". */
            std::string path = dir;
            path += "/" + files[j];
            if (!param.loadParamFile(path))
            {
                std::cout<<"Error in SystemExecute::checkConfigIntact():"
                    <<"Bad configuration file -> "<<path<<std::endl;
                return false;
            }
        }
    }
    return true;
}

//------------------------------------------------------------
// Function:  updateParameters
// Summary: get the parameter name and set the value.
// In:      old_dir -> the directory which we get the parameter values from.
//          new_dir -> the directory which we set the parameter values 
//                      using the values from old dir.
// Out:     None
// Return:  true -> success.
//          false -> failed.
//------------------------------------------------------------
bool SystemExecute::updateParameters(std::string old_dir, std::string new_dir)
{
    fst_parameter::ParamGroup old_param;
    fst_parameter::ParamGroup new_param;

    if (access(old_dir.c_str(), 0) == -1)
        mkdir(old_dir.c_str(), 0777);
    std::vector<std::string> old_files = FileOperations::getFilesName(old_dir);
    /* pick up a yaml file in the old directory. */
    for (unsigned int i = 0; i < old_files.size(); i++)
    {
        /* ignore the none .yaml file in the old dir. */
        std::string file = old_files[i];
        std::size_t found = file.rfind('.');
        if (found == std::string::npos)
            continue;
        file = file.substr(found);
        if (strcmp(file.c_str(), ".yaml") != 0)
            continue;

        /* load .yaml file in old dir to get the parameter list. */
        std::string old_path = old_dir + "/" + old_files[i];
        if (!old_param.loadParamFile(old_path))
        {
            std::cout<<"Error in SystemExecute::updateParameters():"
                <<"Bad configuration file -> "<<old_path<<std::endl;
            return false;
        }
        std::vector<std::string> old_list;
        old_param.getParamList(old_list);

        /* load the same name .yaml file from new dir to get parameter list. */
        std::string new_path = new_dir + "/" + old_files[i];
        if (!new_param.loadParamFile(new_path))
        {
            std::cout<<"Error in SystemExecute::updateParameters():"
                <<"Bad configuration file -> "<<new_path<<std::endl;
            return false;
        }
        std::vector<std::string> new_list;
        new_param.getParamList(new_list);

        /* pick up a parameter from the list to compare the one in the new yaml file. */
        for (unsigned int j = 0; j < old_list.size(); j++)
        {
            std::string par = old_list[j];
            for (std::vector<std::string>::iterator iter = new_list.begin(); iter != new_list.end(); iter++)
            {
                if (*iter == par)
                {
                    /* pass the old parameter value to the new file. */
                    fst_parameter::ParamValue value;
                    if (!old_param.getParam(par, value))
                    {
                        std::cout<<"Error in SystemExecute::updateParameters():failed to read old "<<par<<std::endl;
                        return false;
                    }
                    fst_parameter::ParamValue default_value;
                    if (!new_param.getParam(par, default_value))
                    {
                        std::cout<<"Error in SystemExecute::updateParameters():failed to read new "<<par<<std::endl;
                        return false;
                    }
                    if (value.getType() != default_value.getType())
                    {
                        std::cout<<"Error in SystemExecute::updateParameters():value type changed in "<<par<<std::endl;
                        return false;
                    }
                    if (!new_param.setParam(par, value))
                    {
                        std::cout<<"Error in SystemExecute::updateParameters():failed to set new "<<par<<std::endl;
                        return false;
                    }
                    if (!new_param.dumpParamFile(new_path))
                    {
                        std::cout<<"Error in SystemExecute::updateParameters():failed to dump parameters."<<std::endl;
                        return false;
                    }
                    break;
                }
                /* error if the old list has the parameter the new list doesn't have. */
                if ((iter + 1) == new_list.end())
                {
                    std::cout<<"Warning in SystemExecute::updateParameters():missing parameter "<<par<<" in "<<new_path<<std::endl;
                }
            }

        }

    }
    return true;
}

bool SystemExecute::compareParameters(std::string old_dir, std::string new_dir)
{
    fst_parameter::ParamGroup old_param;
    fst_parameter::ParamGroup new_param;

    if (access(old_dir.c_str(), 0) == -1)
        mkdir(old_dir.c_str(), 0777);
    std::vector<std::string> old_files = FileOperations::getFilesName(old_dir);
    /* pick up a yaml file in the old directory. */
    for (unsigned int i = 0; i < old_files.size(); i++)
    {
        /* ignore the none .yaml file in the old dir. */
        std::string file = old_files[i];
        std::size_t found = file.rfind('.');
        if (found == std::string::npos)
            continue;
        file = file.substr(found);
        if (strcmp(file.c_str(), ".yaml") != 0)
            continue;

        /* load .yaml file in old dir to get the parameter list. */
        std::string old_path = old_dir + "/" + old_files[i];
        if (!old_param.loadParamFile(old_path))
        {
            std::cout<<"Error in SystemExecute::compareParameters():"
                <<"Bad configuration file -> "<<old_path<<std::endl;
            return false;
        }
        std::vector<std::string> old_list;
        old_param.getParamList(old_list);

        /* load the same name .yaml file from new dir to get parameter list. */
        std::string new_path = new_dir + "/" + old_files[i];
        if (!new_param.loadParamFile(new_path))
        {
            std::cout<<"Error in SystemExecute::compareParameters():"
                <<"Bad configuration file -> "<<new_path<<std::endl;
            return false;
        }
        std::vector<std::string> new_list;
        new_param.getParamList(new_list);

        /* compare the two yaml files.*/
        if (old_list != new_list)
        {
            std::cout<<"Error in SystemExecute::compareParameters():"
                <<"the two machine configuratioin files are different."<<new_path<<std::endl;
            return false;
        }
    }
    return true;
}
//------------------------------------------------------------
// Function:  compress
// Summary: compress the source files to zip.
// In:      sources -> the source paths.
//          destination -> the zip path.
// Out:     None
// Return:  FST_SUCCESS -> success.
//          SYS_NO_FREE_DISK -> not enough disk space to operate.
//          SYS_MKDIR_FAIL -> make a directory fail.
//          SYS_RM_FILE_FAIL -> remove file fail.
//------------------------------------------------------------
U64 SystemExecute::compress(std::vector<std::string> sources, std::string destination)
{    
    /*check the available disk size for compression. */
    if (!checkSize(sources))
    {
        std::cout<<"Error in SystemExecute::compress(): "
            <<"No enough space."<<std::endl;
        return SYS_NO_FREE_DISK;
    }

    /* check if destination directory is existed. */
    std::string dest = compress_path_;                       // /media/ftp/fst_compress
    if (access(dest.c_str(), 0) == -1)
    {
        if (mkdir(dest.c_str(), 0777) != 0)
            return SYS_COMPRESS_FILE_FAIL;
        chmod(dest.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
    }
    dest += "/" + destination;

    /* check if configuration files are intact. */
    for (std::vector<std::string>::iterator iter = sources.begin(); iter != sources.end(); iter++)
    {
        if (*iter == config_path_ || *iter == os_path_)
        {
            if (!checkConfigIntact())
            {
                std::cout<<"Error in SystemExecute::compress(): "
                    <<"Bad configuration file."<<std::endl;
                return SYS_CONFIG_DAMAGED;
            }
        }
    }

    /* check if the directory /tmp/fst_backup is existed. */
    if (access(temp_path_.c_str(), 0) == -1)                 // /tmp/fst_backup
    {
        if (mkdir(temp_path_.c_str(), 0777) != 0)
            return SYS_COMPRESS_FILE_FAIL;
    } 
    else
    {
        /* clear /tmp/fst_backup .*/
        std::vector<std::string> files;
        files = FileOperations::getFilesName(temp_path_);    // /tmp/fst_backup
        for (unsigned int i = 0; i < files.size(); i++)
        {
            std::string file = temp_path_ + "/" +files[i];
            remove(file.c_str());
        }
    }

    /* compress and put the zip to the tmp directory*/
    U64 result = FST_SUCCESS;
    for (unsigned int i = 0; i < sources.size(); i++)
    {
        const char *src = sources[i].c_str();
        const char *temp = path_map_[sources[i]].c_str();
        result = FileOperations::archiveCreate(&src, temp);  // /tmp/fst_backup/<name>.tgz
        if (result != FST_SUCCESS)
            return SYS_COMPRESS_FILE_FAIL;
    }

    /* compress the zip and output to the destination. */
    const char *path = temp_path_.c_str();                   // /tmp/fst_backup
    result = FileOperations::archiveCreate(&path, dest.c_str());
    if (result != FST_SUCCESS)
    {
        /* delete the failed zip. */
        remove(dest.c_str());                   // /media/ftp/fst_compress/fst_control.tgz
        return SYS_COMPRESS_FILE_FAIL;
    }

    /* delete the zip file in /tmp. */
    for (unsigned int i = 0; i < sources.size(); i++)
        remove(path_map_[sources[i]].c_str());               // /tmp/fst_backup/<name>.tgz
    remove(temp_path_.c_str());                              // /tmp/fst_backup

    std::cout<<"compress success."<<std::endl;
    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  extract
// Summary: extract the zip.
// In:      sources -> the source paths.
// Out:     None
// Return:  FST_SUCCESS -> success.
//          SYS_NO_FREE_DISK -> not enough disk space.
//          SYS_EXTRACT_ARCHIVE_FAIL -> fail.
//------------------------------------------------------------
U64 SystemExecute::extract(std::string source)
{
    /* check source exist. */
    std::string src = extract_path_;                       // /meida/ftp/fst_extract
    src += "/" +source;                        // /media/ftp/fst_extract/fst_control.tgz
    if (access(src.c_str(), 0) == -1)
    {
        std::cout<<"Error in SystemExecute::extract(): "
            <<"Can't find file in "<<src<<std::endl;
        return SYS_EXTRACT_ARCHIVE_FAIL;
    }

    /*check the available disk size for extraction. */
    std::vector<std::string> srcs;
    srcs.push_back(src);
    if (!checkSize(srcs))
    {
        std::cout<<"Error in SystemExecute::extract(): "
            <<"No enough space."<<std::endl;
        return SYS_NO_FREE_DISK;
    }

    /* make the directory /tmp/fst_backup if not existed. */
    if (access(temp_path_.c_str(), 0) == -1)               // /tmp/fst_backup
    {
        if (mkdir(temp_path_.c_str(), 0777) != 0)
            return SYS_COMPRESS_FILE_FAIL;
    } 
    else
    {
        // clear /tmp/fst_backup ./
        std::vector<std::string> files;
        files = FileOperations::getFilesName(temp_path_);  // /tmp/fst_backup
        for (unsigned int i = 0; i < files.size(); i++)
        {
            std::string file = temp_path_ + "/" +files[i]; // /tmp/fst_backup/<name>.tgz
            remove(file.c_str());
        }
    }

    /* extract zip to the tmp directory. */
    std::string dir = temp_path_;
    dir = dir.substr(0, dir.rfind('/') + 1);               // dir = /tmp/
    U64 result = FileOperations::archiveExtract(src.c_str(), dir.c_str());
    if (result != FST_SUCCESS)
        return SYS_EXTRACT_ARCHIVE_FAIL;

    /* extract the files to the destination. */
    std::vector<std::string> files;
    files = FileOperations::getFilesName(temp_path_);      // /tmp/fst_backup
    for (unsigned int i = 0; i < files.size(); i++)
    {
        if (path_map_.find(files[i]) == path_map_.end())
            continue;
        std::string arch = temp_path_ + "/" + files[i];    // /tmp/fst_backup/<name>.tgz
        std::string dest = path_map_[files[i]];
        dest = dest.substr(0, dest.rfind('/') + 1);
        result = FileOperations::archiveExtract(arch.c_str(), dest.c_str());
        if (result != FST_SUCCESS)
            return SYS_EXTRACT_ARCHIVE_FAIL;
    }

    /* delete the extract file. */
    remove(src.c_str());                       // /media/ftp/fst_extract/fst_control.tgz
    /* delete the temp file. */
    for (unsigned int i = 0; i < files.size(); i++)
    {
        std::string file = temp_path_ + "/" +files[i];     // /tmp/fst_backup/<name>.tgz
        remove(file.c_str());
    }
    remove(temp_path_.c_str());                            // /tmp/fst_backup

    std::cout<<"extract success."<<std::endl;
    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  upgrade
// Summary: upgrade the robot system.
// In:      sources -> the source path.
// Out:     None
// Return:  FST_SUCCESS -> success.
//          SYS_UPGRADE_FAIL -> fail.
//------------------------------------------------------------
U64 SystemExecute::upgrade(std::string source)
{
    /* <absolute path>/install/share/configuration -> <absolut path> */
    bool ret = FileOperations::copy(config_path_.c_str(), config_backup_.c_str());
    if (ret == false)
        return SYS_UPGRADE_FAIL;
    
    /* extract the archive. */
    U64 result = extract(source);
    if (result != FST_SUCCESS)
        return SYS_UPGRADE_FAIL;

    /* update machine parameters.
     <absolute path>/config_backup/machine -> <absolut path>/install/configuration/machine */
    std::string prev_machine = config_backup_ + "/" + config_machine_;
    std::string new_machine = config_path_ + "/" + config_machine_;
    ret = compareParameters(prev_machine, new_machine);
    if (ret == false)
        return SYS_MACHINE_CONFIG_ERROR;
    ret = FileOperations::copy(prev_machine.c_str(), new_machine.c_str());
    if (ret == false)
        return SYS_MACHINE_CONFIG_ERROR;

    /* update configurable parameters.
     <absolute path>/config_backup/configurable -> <absolut path>/install/configuration/configurable */
    std::string prev_configurable = config_backup_ + "/" + config_configurable_;
    std::string new_configurable = config_path_ + "/" + config_configurable_;
    ret = updateParameters(prev_configurable, new_configurable);
    if (ret == false)
        return SYS_CONFIGURABLE_FILE_ERROR;

    std::cout<<"upgrade success."<<std::endl;
    return FST_SUCCESS;
}

//------------------------------------------------------------
// Function:  stopFTP
// Summary: stop the ftp service.
// In:      None.
// Out:     None.
// Return:  FST_SUCCESS -> success.
//          SYS_FTP_OFF_FAIL -> stop the tfp service.
//------------------------------------------------------------
U64 SystemExecute::stopFTP(void)
{
    /* start the service. */
    FILE *fp = NULL;
    fp = popen("sudo /usr/sbin/service vsftpd stop", "r");
    if (!fp)
        return SYS_FTP_OFF_FAIL;
    fclose(fp);

    /* check the service status. */
    fp = popen("sudo /usr/sbin/service vsftpd status", "r");
    if (!fp)
        return SYS_FTP_OFF_FAIL;

    /* check the output of the status whether including "stop".*/
    char buf[128] = {0};
    char out[256] = {0};
    while (fgets(buf, sizeof(buf), fp) != NULL)
        snprintf(out, sizeof(out), "%s", buf);
    pclose(fp);

    std::string str = out;
    std::string str_key = "stop";
    std::size_t found = str.find(str_key);
    if (found != std::string::npos)
    {
        std::cout<<"ftp is off."<<std::endl;
        return FST_SUCCESS;
    }
    std::cout<<"Error in SystemExecute::stopFTP(): fail to stop ftp service."<<std::endl;
    return SYS_FTP_OFF_FAIL;

    /*
    pid_t pid = fork();
    if (pid < 0)
    {
        return SYS_FTP_OFF_FAIL;
    }
    else if (pid == 0)
    {
        if (execl("/bin/sh", "sh", "-c", "sudo /usr/sbin/service vsftpd stop", NULL) < 0)
            exit(0);
    }
    
    return FST_SUCCESS;
    */
}

//------------------------------------------------------------
// Function:  startFTP
// Summary: start the ftp service.
// In:      None.
// Out:     None.
// Return:  FST_SUCCESS -> success.
//          SYS_FTP_ON_FAIL -> start the tfp service.
//------------------------------------------------------------
U64 SystemExecute::startFTP(void)
{
    /* start the service. */
    FILE *fp = NULL;
    fp = popen("sudo /usr/sbin/service vsftpd start", "r");
    if (!fp)
        return SYS_FTP_ON_FAIL;
    fclose(fp);

    /* check the service status. */
    fp = popen("sudo /usr/sbin/service vsftpd status", "r");
    if (!fp)
        return SYS_FTP_ON_FAIL;

    /* check the output of the status whether including "start".*/
    char buf[128] = {0};
    char out[256] = {0};
    while (fgets(buf, sizeof(buf), fp) != NULL)
        snprintf(out, sizeof(out), "%s", buf);
    pclose(fp);

    std::string str = out;
    std::string str_key = "start";
    std::size_t found = str.find(str_key);
    if (found != std::string::npos)
    {
        std::cout<<"ftp is on."<<std::endl;
        return FST_SUCCESS;
    }
    std::cout<<"Error in SystemExecute::startFTP(): failed to start ftp service."<<std::endl;
    return SYS_FTP_ON_FAIL;

    /*
    pid_t pid = fork();
    if (pid < 0)
    {
        return SYS_FTP_ON_FAIL;
    }
    else if (pid == 0)
    {
        if (execl("/bin/sh", "sh", "-c", "sudo /usr/sbin/service vsftpd restart", NULL) < 0)
            exit(0);
    }
    return FST_SUCCESS;
    */
}

}

#endif

