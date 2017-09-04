/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       system_client.cpp
Author:     Feng.Wu 
Create:     31-July-2017
Modify:     31-July-2017
Summary:    test process
**********************************************/
#ifndef SYSTEM_MANAGER_SYSTEM_CLIENT_CPP_
#define SYSTEM_MANAGER_SYSTEM_CLIENT_CPP_
   
#include "system_manager/system_manager.h"
#include "system_manager/file_operations.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>

int main(int argc, char** argv)
{    

    using namespace fst_file_operations;
    using namespace fst_system_execute;
    long long int ret = 0;
    bool result = false;
    int count = 0;
    DIR *ddp = NULL;

    SystemExecute::init();
    while (true)
    {
        usleep(100*1000);
        
//        ret = FileOperations::archiveExtract("/media/ftp/fst_compress/fst_control.tgz", "/tmp/");
//        if (ret != 0)
//            break;
/*
        result = FileOperations::copy("/home/fst/tutorial/myros3/fst2/install/share/configuration", "/home/fst/tutorial/myros3/fst2/config_backup");
        if (result == false)
        {
            std::cout<<"copy fail"<<std::endl;
            break;
        }
        std::cout<<"copy done"<<std::endl;
        ret = SystemExecute::extract("fst_control.tgz");
        if (ret != 0)
            break;
*/
        if ((ddp = opendir("/home/fst/tutorial/myros3/fst2/install/share/configuration/model")) == NULL)
        {
            std::cout<<"error fail = "<<count<<std::endl;
            printf("string = %s\n", strerror(errno));
            break;
        }

        count++;
        std::cout<<"extract count = "<<count<<std::endl;
    }
    std::cout<<"exit..."<<std::endl;
}


#endif
