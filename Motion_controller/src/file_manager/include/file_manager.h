/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       file_manager.h
Author:     Feng.Wu
Create:     12-Dec-2018
Modify:     12-Dec-2018
Summary:    dealing with files
**********************************************/

#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

#include <stdint.h>
#include "error_code.h"
#include "base_datatype.h"


namespace fst_base
{

class FileManager
{
public:
    FileManager();
    ~FileManager();

    ErrorCode readFileStream(uint8_t* &ptr, long &length, const char* file_path);
    ErrorCode writeFileStream(void* ptr, long length, const char* file_path);

private:
    uint8_t buff_[65535];
};

}

#endif

