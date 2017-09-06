/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       utest_all.cpp
Author:     Feng.Wu 
Create:     04-Sep-2017
Modify:     04-Sep-2017
Summary:    unit test
**********************************************/
#ifndef SYSTEM_MANAGER_UTEST_MAIN_CPP_
#define SYSTEM_MANAGER_UTEST_MAIN_CPP_
   
#include <gtest/gtest.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#endif
