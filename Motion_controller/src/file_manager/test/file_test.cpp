/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       file_test.cpp
Author:     Feng.Wu 
Create:     13-Dec-2018
Modify:     13-Dec-2018
Summary:    test process
**********************************************/
#ifndef FILE_MANAGER_FILE_TEST_CPP_
#define FILE_MANAGER_FILE_TEST_CPP_

#include "file_manager.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char** argv)
{
    fst_base::FileManager op;
    long length = 0;
    const char *path = "/root/robot_data/io/io_mapping/di_mapping.json";

    uint8_t *r_ptr;
    op.readFileStream(r_ptr, length, path);
    printf("main(): read file result =\n%s\n", r_ptr);

    op.writeFileStream(r_ptr, length, "/root/myFileTestWrite");

    return 0;
}


#endif 
