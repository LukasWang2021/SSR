/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       system_excute.h
Author:     Feng.Wu 
Create:     19-Jun-2017
Modify:     31-July-2017
Summary:    
**********************************************/
#ifndef SYSTEM_MANAGER_SYSTEM_EXECUTE_H_
#define SYSTEM_MANAGER_SYSTEM_EXECUTE_H_

#include "system_manager_error_code.h"
#include <vector>
#include <string>
#include <map>

namespace fst_system_execute
{

class SystemExecute
{
public:
    //------------------------------------------------------------
    // Function:  SystemExecute
    // Summary: The constructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    SystemExecute();

    //------------------------------------------------------------
    // Function:  ~SystemExecute
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None 
    //------------------------------------------------------------
    ~SystemExecute();

    //------------------------------------------------------------
    // Function:  init
    // Summary: initialize some map and create some directories.
    // In:      None
    // Out:     None
    // Return:  FST_SUCCESS -> success.
    //          SYS_MKDIR_FAIL -> fail.
    //          SYS_READ_VERSION_FAIL -> fail.
    //------------------------------------------------------------
    static U64 init(void);

    //------------------------------------------------------------
    // Function:  getAllVersion
    // Summary: read the version files.
    // In:      None
    // Out:     None
    // Return:  FST_SUCCESS -> success.
    //          SYS_READ_VERSION_FAIL -> fail.
    //------------------------------------------------------------
    static U64 getAllVersion(void);

    //------------------------------------------------------------
    // Function:  split
    // Summary: get the package name and version number by splitting
    //          the string read from the version file.
    // In:      str -> a string read from the version file.
    //          pattern -> the symbol to split a string.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    static void split(std::string str, std::string pattern);

    //------------------------------------------------------------
    // Function:  checkSize
    // Summary: check if there is enough disk space to compress or
    //          extract the zip.
    // In:      the source path which is going to be compressed or extracted.
    // Out:     None
    // Return:  true -> enough space.
    //          false -> not enough space.
    //------------------------------------------------------------
    static bool checkSize(std::vector<std::string> paths);

    //------------------------------------------------------------
    // Function:  checkConfigIntact
    // Summary: check if there the configuration files are intact.
    // In:      None
    // Out:     None
    // Return:  true -> intact.
    //          false -> damaged.
    //------------------------------------------------------------
    static bool checkConfigIntact(void);

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
    static bool updateParameters(std::string old_dir, std::string new_dir);

    static bool compareParameters(std::string old_dir, std::string new_dir);

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
    static U64 compress(std::vector<std::string> sources, std::string destination);

    //------------------------------------------------------------
    // Function:  extract
    // Summary: extract the zip.
    // In:      sources -> the source paths.
    // Out:     None
    // Return:  FST_SUCCESS -> success.
    //          SYS_NO_FILE_FOUND -> can't find the zip.
    //          SYS_NO_FREE_DISK -> not enough disk space.
    //          SYS_RM_FILE_FAIL -> remove file fail.
    //------------------------------------------------------------
    static U64 extract(std::string source);

    //------------------------------------------------------------
    // Function:  upgrade
    // Summary: upgrade the robot system.
    // In:      sources -> the source path.
    // Out:     None
    // Return:  FST_SUCCESS -> success.
    //          SYS_COPY_FAIL -> copy files fail.
    //------------------------------------------------------------
    static U64 upgrade(std::string source);

    //------------------------------------------------------------
    // Function:  stopFTP
    // Summary: stop the ftp service.
    // In:      None.
    // Out:     None.
    // Return:  FST_SUCCESS -> success.
    //          SYS_FTP_OFF_FAIL -> stop the tfp service.
    //------------------------------------------------------------
    static U64 stopFTP(void);

    //------------------------------------------------------------
    // Function:  startFTP
    // Summary: start the ftp service.
    // In:      None.
    // Out:     None.
    // Return:  FST_SUCCESS -> success.
    //          SYS_FTP_ON_FAIL -> start the tfp service.
    //------------------------------------------------------------
    static U64 startFTP(void);
    
    // the robot software paths.
    static std::string os_path_;
    
    // the boot image path.
    static std::string infra_path_;

    // the configuration file.
    static std::string config_path_;

    // the output zip of the robot software.
    static std::string os_name_;

    // the output zip of the boot image.
    static std::string infra_name_;

    // the output zip of the configuration file.
    static std::string config_name_;

    // the final output zip.
    static std::string output_archive_;

private:
    // the vector to store the version name.
    static std::vector<std::string> version_name_;

    // the map between package name and version numbers.
    static std::map<std::string, std::string> version_num_map_;

    // the map between the software path and output target.
    static std::map<std::string, std::string> path_map_;

    // the path to the version of the software.
    static std::string version_path_;

    // the path to the temporay configuration path.
    static std::string config_backup_;

    // the configuration path include machine and configurable directories.
    static std::string config_machine_;
    static std::string config_configurable_;

    // the ftp path including usb, compress and extract paths.
    static std::string ftp_path_;
    static std::string usb_path_;
    static std::string compress_path_;
    static std::string extract_path_;
    
    // the temporary path for compression or extraction.
    static std::string temp_path_;

};
}



#endif
