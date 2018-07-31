/**
 * @file main.cpp
 * @brief :main function of gtest
 * @author WangWei
 * @version 1.0.0
 * @date 2017-11-08
 */

#include<iostream>
#include<gtest/gtest.h>

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
