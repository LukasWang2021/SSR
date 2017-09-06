/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       utest_system_execute.cpp
Author:     Feng.Wu 
Create:     04-Sep-2017
Modify:     04-Sep-2017
Summary:    unit test
**********************************************/
#ifndef SYSTEM_MANAGER_UTEST_SYSTEM_EXECUTE_CPP_
#define SYSTEM_MANAGER_UTEST_SYSTEM_EXECUTE_CPP_
   
#include "system_manager/system_execute.h"
#include <gtest/gtest.h>

using namespace fst_system_execute;

class ExecuteTest:public ::testing::Test{
    protected:
        //SystemExecute  e;
        virtual void SetUp()
        {
            SystemExecute::init();
        }

        virtual void TearDown(){}
};

// TEST_F(the name of the test fixture class, test_name)
TEST_F(ExecuteTest, GetAllVersion){
    EXPECT_EQ(0, SystemExecute::getAllVersion());
}

TEST_F(ExecuteTest, CheckSizeTrue){
    std::vector<std::string> v;
    v.push_back("/tmp");
    EXPECT_TRUE(SystemExecute::checkSize(v));
}

TEST_F(ExecuteTest, CheckSizeFalse){
    std::vector<std::string> v;
    v.push_back("/not_exist");
    EXPECT_FALSE(SystemExecute::checkSize(v));
}

TEST_F(ExecuteTest, CheckConfigIntact){
    EXPECT_TRUE(SystemExecute::checkConfigIntact());
}

TEST_F(ExecuteTest, FtpOn){
    EXPECT_EQ(0, SystemExecute::startFTP());
}

TEST_F(ExecuteTest, FtpOff){
    EXPECT_EQ(0, SystemExecute::stopFTP());
}

#endif
