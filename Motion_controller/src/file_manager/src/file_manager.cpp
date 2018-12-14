/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       file_manager.cpp
Author:     Feng.Wu
Create:     12-Dec-2018
Modify:     12-Dec-2018
Summary:    dealing with files
**********************************************/

#include "file_manager.h"
#include <string.h>
#include <stdio.h>

using namespace std;
using namespace fst_base;

FileManager::FileManager()
{
   memset(&buff_, 0, sizeof(buff_));
}

FileManager::~FileManager()
{

}

ErrorCode FileManager::readFileStream(uint8_t* &ptr, long &length, const char* file_path)
{
    long buff_len = sizeof(buff_);
    memset(&buff_, 0, buff_len);

    FILE *fp = fopen(file_path, "rb");
    if (fp == NULL)
    {
        printf("Open file failed when reading: %s\n", file_path);
        return FILE_MANAGER_READ_FILE_FAILED;
    }
  
    fseek(fp, 0, SEEK_END);                        // point to the end of file
    length = ftell(fp);                            // get the file size
    length = (length < buff_len)?length:buff_len;  // in case the size of file is bigger than buff
    fseek(fp, 0, SEEK_SET);                        // point to the start of file

    fread(&buff_, length, 1, fp);                  //read all in one time
    buff_[length] = '\0';

    fclose(fp);
    ptr = &buff_[0];
    return SUCCESS;
}

ErrorCode FileManager::writeFileStream(void* ptr, long length, const char* file_path)
{
    FILE *fp = fopen(file_path, "wb");
    if (fp == NULL)
    {
        printf("Open file failed when writing: %s\n", file_path);
        return FILE_MANAGER_WRITE_FILE_FAILED;
    }

    fwrite(ptr, length, 1, fp);
    
    fflush(fp);
    fclose(fp);
    return SUCCESS;
}