/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       utest.cpp
Author:     Feng.Wu 
Create:     22-Sep-2017
Modify:     22-Sep-2017
Summary:    unit test
**********************************************/
#ifndef SYSTEM_MANAGER_UTEST_SAFETY_CPP_
#define SYSTEM_MANAGER_UTEST_SAFETY_CPP_
   
#include <gtest/gtest.h>
#include "safety/safety.h"

class SafetyTest:public ::testing::Test{
    protected:
        virtual void SetUp(){
            openSafety();
        }
        virtual void TearDown(){
            closeSafety();
        }
};

// Declare a test
TEST(openSafetyTest, Open){
    ASSERT_EQ(0, openSafety());
    closeSafety();
}

TEST_F(SafetyTest, autorunSafetyData){
    for (int i = 0; i < 4; i++){
    ASSERT_EQ(0, autorunSafetyData());
    }
}

TEST_F(SafetyTest, getSafety){
    unsigned long long int result;
    ASSERT_NE(0, getSafety(SAFETY_INPUT_FIRSTFRAME, &result));
    ASSERT_NE(0, getSafety(SAFETY_INPUT_SECONDFRAME, &result));
}

TEST_F(SafetyTest, setSafety){
    unsigned long long int result;
    ASSERT_EQ(0, setSafety(0x87654321, SAFETY_OUTPUT_SECONDFRAME));
    ASSERT_EQ(0, autorunSafetyData());
}


#endif
