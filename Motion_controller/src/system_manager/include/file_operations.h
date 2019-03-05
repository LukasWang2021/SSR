/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       file_operations.h
Author:     Feng.Wu 
Create:     12-Jun-2017
Modify:     22-Feb-2019
Summary:    
**********************************************/
#ifndef SYSTEM_MANAGER_FILE_OPERATIONS_H_
#define SYSTEM_MANAGER_FILE_OPERATIONS_H_

#include "error_code.h"
#include <string>
#include <vector>
#include "error_code.h"
//#include <archive.h>
//#include <archive_entry.h>

namespace fst_ctrl
{

class FileOperations
{
public:
    FileOperations();

    ~FileOperations();

    //------------------------------------------------------------
    // Function:  copyFile
    // Summary: copy regular file.
    // In:      source -> the source of the file.
    //          destination -> the destination path and file name.
    // Out:     None
    // Return:  false -> fail.
    //          true -> success.
    //------------------------------------------------------------
    static bool copyFile(const char *source, const char *destination);

    //------------------------------------------------------------
    // Function:  copyDir
    // Summary: copy directory.
    // In:      source -> the directory path.
    //          destination -> the destination path.
    // Out:     None
    // Return:  false -> fail.
    //          true -> success.
    //------------------------------------------------------------
    static bool copyDir(const char *source, const char *destination);

    //------------------------------------------------------------
    // Function:  copy
    // Summary: copy file or directory.
    // In:      source -> the source path.
    //          destination -> the destination path.
    // Out:     None
    // Return:  false -> fail.
    //          true -> success.
    //------------------------------------------------------------
    static bool copy(const char *source, const char *destination);

    //------------------------------------------------------------
    // Function:  getExePath
    // Summary: get the executable path.
    // In:      buf -> the string to store the path.
    //          size -> the string size.
    // Out:     None
    // Return:  NULL -> fail.
    //          path -> the path string.
    //------------------------------------------------------------
    static char* getExePath(char *buf, int size);

    //------------------------------------------------------------
    // Function:  getFilesName
    // Summary: get all the regular files name under some directory.
    // In:      source -> the directory path.
    // Out:     None
    // Return:  the vector of the files name.
    //------------------------------------------------------------
    static std::vector<std::string> getFilesName(std::string dir_path);

    //------------------------------------------------------------
    // Function:  getDirName
    // Summary: get all the directory names under some directory.
    // In:      source -> the directory path.
    // Out:     None
    // Return:  the vector of the directory name.
    //------------------------------------------------------------
    static std::vector<std::string> getDirsName(std::string dir_path);

    //------------------------------------------------------------
    // Function:  getFreeDiskSize
    // Summary: get the availble disk size.
    // In:      None.
    // Out:     None
    // Return:  the size of the disk. The unit is Byte.
    //          -1 -> fail.
    //------------------------------------------------------------
    static long long getFreeDiskSize(void);

    //------------------------------------------------------------
    // Function:  getFileSize
    // Summary: get the file size.
    // In:      file_path -> the file path.
    // Out:     None
    // Return:  the size of the file. The unit is Byte.
    //          -1 -> fail.
    //------------------------------------------------------------
    static long long getFileSize(const char *file_path);

    //------------------------------------------------------------
    // Function:  getDirSize
    // Summary: get the directory size.
    // In:      dir_path -> the directory path.
    // Out:     None
    // Return:  the size of the directory. The unit is Byte.
    //          -1 -> fail.
    //------------------------------------------------------------
    static long long getDirSize(const char *dir_path);

    //------------------------------------------------------------
    // Function:  archiveCreate
    // Summary: create a zip.
    // In:      source -> the source path to be compress.
    //          destination -> the path of the zip output.
    // Out:     None
    // Return:  FST_SUCCESS -> success.
    //          SYS_COMPRESS_OPEN_FILE_FAIL -> can't find the file.
    //          SYS_COMPRESS_READ_FILE_HEADER_FAIL -> can't read the file.
    //          SYS_COMPRESS_WRITE_FILE_FAIL -> can't copy the file.
    //------------------------------------------------------------
    static ErrorCode archiveCreate(const char **source, const char *destination, const char *passphrase);

    //------------------------------------------------------------
    // Function:  archiveExtract
    // Summary: extract a zip.
    // In:      source -> the source path to be extracted.
    //          destination -> the destination path.
    // Out:     None
    // Return:  FST_SUCCESS -> success.
    //          SYS_EXTRACT_OPEN_ARCHIVE_FAIL -> can't find the zip.
    //          SYS_EXTRACT_READ_ARCHIVE_FAIL -> can't read the file.
    //          SYS_EXTRACT_WRITE_FILE_HEADER_FAIL -> can't write the file header.
    //          SYS_EXTRACT_WRITE_FILE_DATA_FAIL -> can't write the data
    //------------------------------------------------------------
    static ErrorCode archiveExtract(const char *source, const char *destination, const char *passphrase);

    //------------------------------------------------------------
    // Function:  copyData
    // Summary: copy data when extracting a zip.
    // In:      source -> the source path.
    //          destination -> the destination path.
    // Out:     None
    // Return:  ARCHIVE_OK -> success.
    //------------------------------------------------------------
    static int copyData(struct archive *ar, struct archive *aw);

private:
    

};

    // The string length for printing.
    const int MSG_BUFFER_SIZE = 256;

}

#endif
