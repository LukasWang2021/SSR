/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       utest.cpp
Author:     Feng.Wu 
Create:     22-Sep-2017
Modify:     22-Sep-2017
Summary:    unit test
**********************************************/
#ifndef SAFETY_UTEST_MAIN_CPP_
#define SAFETY_UTEST_MAIN_CPP_
   
#include <gtest/gtest.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#endif
