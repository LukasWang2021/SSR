/*************************************************************************
	> File Name: utest_main.cpp
	> Author: 
	> Mail: 
	> Created Time: 2017年09月19日 星期二 15时25分54秒
 ************************************************************************/

#include<iostream>
#include<gtest/gtest.h>

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


