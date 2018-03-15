/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       utest_all.cpp
Author:     Feng.Wu 
Create:     04-Sep-2017
Modify:     04-Sep-2017
Summary:    unit test
**********************************************/
#ifndef SYSTEM_MANAGER_UTEST_FILE_OPERATIONS_CPP_
#define SYSTEM_MANAGER_UTEST_FILE_OPERATIONS_CPP_
   
#include <gtest/gtest.h>
#include "system_manager/file_operations.h"

using namespace fst_file_operations;
// Declare a test
TEST(CopyFileTest, ShouldTrue){
    ASSERT_TRUE(FileOperations::copyFile("/etc/legal", "/tmp/legal_copy_file_test"));
}

TEST(CopyFileTest, CopyNotExistFile){
    ASSERT_FALSE(FileOperations::copyFile("/not_exist", "/tmp/legal_copy_file_test"));
}

TEST(CopyDirTest, ShouldTrue){
    ASSERT_TRUE(FileOperations::copyDir("/etc/default", "/tmp/default_copy_dir_test"));
}

TEST(CopyDirTest, CopyNotExistDir){
    ASSERT_FALSE(FileOperations::copyDir("/not_exist", "/tmp/default_copy_dir_test"));
}

TEST(CopyTest, ShouldTrue){
    ASSERT_TRUE(FileOperations::copy("/etc/legal", "/tmp/legal_copy_test"));
}

TEST(CopyTest, CopyNotExist){
    ASSERT_FALSE(FileOperations::copy("/not_exist", "/tmp/legal_copy_test"));
}

TEST(GetExePathTest, ShouldNotNull){
    char path[128];
    ASSERT_STRNE(NULL, FileOperations::getExePath(path, sizeof(path)));
//    printf("my path = %s\n", path);
//    EXPECT_STREQ("/home/fst/tutorial/myros3/fst2/install/lib/system_manager/utest", path);
}

TEST(getFilesNameTest, ShouldNotZero){
    std::vector<std::string> v = FileOperations::getFilesName("/etc");
    int num = v.size();
//    printf("the files number = %d.\n", num);
    ASSERT_NE(0, num);
}

TEST(getDirsNameTest, ShouldNotZero){
    std::vector<std::string> v = FileOperations::getDirsName("/etc");
    int num = v.size();
//    printf("the diretory number = %d.\n", num);
    ASSERT_NE(0, num);
}

TEST(GetFreeDiskSizeTest, ShouldNotNegative){
    ASSERT_NE(-1, FileOperations::getFreeDiskSize());
}

TEST(GetFileSizeTest, ShouldNotNegative){
    ASSERT_NE(-1, FileOperations::getFileSize("/etc/legal"));
}

TEST(GetDirSizeTest, ShouldNotNegative){
    ASSERT_NE(-1, FileOperations::getDirSize("/etc/default"));
}

TEST(ArchiveCreateTest, ShouldZero){
    const char *src = "/etc/legal";
    ASSERT_EQ(0, FileOperations::archiveCreate(&src, "/tmp/compress"));
}

TEST(ArchiveExtractTest, ShouldZero){
    const char *src = "/tmp/compress";
    ASSERT_EQ(0, FileOperations::archiveExtract(src, "/tmp/extract"));
}

/*
TEST(DISABLED_NumberTest, Equal){
    EXPECT_NEAR(1.00001, 1.000011, 0.0000001);
    EXPECT_NEAR(1.00001, 1.000011, 0.00001);
}
*/
#endif
