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
        /*
        virtual void SetUp(){
            e.init();
        }
        virtual void TearDown(){}
        SystemExecute e;
        */
        static void SetUpTestCase(){
            shared_ptr_ = new SystemExecute();
            //printf("set up once in a case.\n");
        }
        static void TearDownTestCase(){
            delete shared_ptr_;
            shared_ptr_ = nullptr;
            //printf("tear down once in a case.\n");
        }
        static SystemExecute *shared_ptr_;

};
SystemExecute * ExecuteTest::shared_ptr_ = nullptr;

// TEST_F(the name of the test fixture class, test_name)
TEST_F(ExecuteTest, init){
    EXPECT_EQ(0, shared_ptr_->init());
}

TEST_F(ExecuteTest, GetAllVersion){
    EXPECT_EQ(0, shared_ptr_->getAllVersion());
}

TEST_F(ExecuteTest, CheckSizeTrue){
    std::vector<std::string> v;
    v.push_back("/tmp");
    EXPECT_TRUE(shared_ptr_->checkSize(v));
}

TEST_F(ExecuteTest, CheckSizeFalse){
    std::vector<std::string> v;
    v.push_back("/not_exist");
    EXPECT_FALSE(shared_ptr_->checkSize(v));
}

TEST_F(ExecuteTest, CheckConfigIntact){
    EXPECT_TRUE(shared_ptr_->checkConfigIntact());
}
/*
TEST_F(ExecuteTest, FtpOn){
    EXPECT_EQ(0, shared_ptr_->startFTP());
}

TEST_F(ExecuteTest, FtpOff){
    EXPECT_EQ(0, shared_ptr_->stopFTP());
}
*/
#endif
