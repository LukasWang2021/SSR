/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       system_manager.cpp
Author:     Feng.Wu 
Create:     12-Jun-2017
Modify:     22-Feb-2019
Summary:    compress, extract...
**********************************************/
#ifndef SYSTEM_MANAGER_FILE_OPERATIONS_CPP_
#define SYSTEM_MANAGER_FILE_OPERATIONS_CPP_

#include "file_operations.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <unistd.h>
#include <dirent.h>
#include <string.h>
#include <sys/types.h>
#include <sys/statfs.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <stdarg.h>

using namespace fst_ctrl;

FileOperations::FileOperations()
{
}

FileOperations::~FileOperations()
{
}

//------------------------------------------------------------
// Function:  copyFile
// Summary: copy regular file.
// In:      source -> the source of the file.
//          destination -> the destination path and file name.
// Out:     None
// Return:  false -> fail.
//          true -> success.
//------------------------------------------------------------
bool FileOperations::copyFile(const char *source, const char *destination)
{
    char buf[4096];
    size_t size;

    /*  open input file. */
    FILE *in = fopen(source, "r");
    if (in == NULL)
    {
        printf("FileOperations::copyFile: Can't open source file -> %s", source);
        return false;
    }

    /* open output file. */
    FILE *out = fopen(destination, "w");
    if (out == NULL)
    {
        printf("FileOperations::copyFile: Can't open destination file -> %s", destination);
        return false;
    }

    /* copy data between. */
    while ((size = fread(buf, 1, sizeof(buf), in)) > 0)
    {
        if (fwrite(buf, 1, size, out) < size)
        {
            fclose(in);
            fclose(out);
            return false;
        }
    }
    fclose(in);
    fclose(out);
    return true;
}

//------------------------------------------------------------
// Function:  copyDir
// Summary: copy directory.
// In:      source -> the directory path.
//          destination -> the destination path.
// Out:     None
// Return:  false -> fail.
//          true -> success.
//------------------------------------------------------------
bool FileOperations::copyDir(const char *source, const char *destination)
{
    DIR *dp = NULL, *ddp = NULL;
    struct dirent *entry;
    struct stat info;
    std::string local_source = source;
    /* check the source directory. */
    if ((dp = opendir(source)) == NULL)
    {
        printf("FileOperations::copyDir: Can't open source directory -> %s", source);
        return false;
    } 

    /* create the destination directory. */
    if ((ddp = opendir(destination)) == NULL)
    {
        if (mkdir(destination, 0777) != 0)
        {
            printf("FileOperations::copyDir: Can't open destination directory -> %s", destination);
            return false;
        }
    }
    closedir(ddp);

    /* begin to copy directory. */
    while ((entry = readdir(dp)) != NULL)
    {
        char read_sub[256];
        char write_sub[256];
        sprintf(read_sub, "%s/%s", source, entry->d_name);
        lstat(read_sub, &info);
        sprintf(write_sub, "%s/%s", destination, entry->d_name);

        if (S_ISDIR(info.st_mode))
        {
            if (strcmp(".", entry->d_name) == 0 || strcmp("..", entry->d_name) == 0)
                continue;
            /* recursively call itself to copy directory. */
            copyDir(read_sub, write_sub);
        }
        else 
        {
            /* copy file. */
            copyFile(read_sub, write_sub);
        }
    }
    closedir(dp);
    return true;
}

//------------------------------------------------------------
// Function:  copy
// Summary: copy file or directory.
// In:      source -> the source path.
//          destination -> the destination path.
// Out:     None
// Return:  false -> fail.
//          true -> success.
//------------------------------------------------------------
bool FileOperations::copy(const char *source, const char *destination)
{
    /* check the source mode. */
    struct stat info;
    lstat(source, &info);
    if (S_ISDIR(info.st_mode))
    {
        /* copy directory. */
        return copyDir(source, destination);
    }
    else
    {
        /* copy file. */
        return copyFile(source, destination);
    }
}

//------------------------------------------------------------
// Function:  getExePath
// Summary: get the executable path.
// In:      buf -> the string to store the path.
//          size -> the string size.
// Out:     None
// Return:  NULL -> fail.
//          path -> the path string.
//------------------------------------------------------------
char* FileOperations::getExePath(char *buf, int size)
{
    int len = readlink("/proc/self/exe", buf, size);
    if (len < 0 || len >= size)
    {
        printf("FileOperations::getExePath: Failed to get executable path.");
        return NULL;
    }

    buf[len] = '\0';
    return buf;
}

//------------------------------------------------------------
// Function:  getFilesName
// Summary: get all the regular files name under some directory.
// In:      source -> the directory path.
// Out:     None
// Return:  the vector of the files name.
//------------------------------------------------------------
std::vector<std::string> FileOperations::getFilesName(std::string dir_path)
{
    std::vector<std::string> files;
    DIR *dp;
    struct dirent *entry;

    /* open the directory. */
    if ((dp = opendir(dir_path.c_str())) == NULL)
    {
        printf("FileOperations::getFilesName: No such directory path -> %s", dir_path.c_str());
        return files;
    }

    /* push the file name to the vector. */
    while ((entry = readdir(dp)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;
        else if (entry->d_type == DT_REG) //file
            files.push_back(entry->d_name);
        else if (entry->d_type == DT_DIR)  //dir
            continue;
    }
    closedir(dp);
    sort(files.begin(), files.end());
    return files;
}

//------------------------------------------------------------
// Function:  getDirName
// Summary: get all the directory names under some directory.
// In:      source -> the directory path.
// Out:     None
// Return:  the vector of the directory name.
//------------------------------------------------------------
std::vector<std::string> FileOperations::getDirsName(std::string dir_path)
{
    std::vector<std::string> dirs;
    DIR *dp;
    struct dirent *entry;

    /* open the directory. */
    if ((dp = opendir(dir_path.c_str())) == NULL)
    {
        printf("FileOperations::getDirsName: No such directory path -> %s", dir_path.c_str());
        return dirs;
    }

    /* push the file name to the vector. */
    while ((entry = readdir(dp)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;
        else if (entry->d_type == DT_REG) //file
            continue;
        else if (entry->d_type == DT_DIR)  //dir
            dirs.push_back(entry->d_name);
    }
    closedir(dp);
    sort(dirs.begin(), dirs.end());
    return dirs;
}

//------------------------------------------------------------
// Function:  getFreeDiskSize
// Summary: get the availble disk size.
// In:      None.
// Out:     None
// Return:  the size of the disk. The unit is Byte.
//          -1 -> fail.
//------------------------------------------------------------
long long FileOperations::getFreeDiskSize(void)
{
    /* obtain the free size of the disk. */
    long long free_size = -1;
    struct statfs info;
    if (statfs("/", &info) == 0)
    {
        long long block_size = info.f_bsize;
        free_size = block_size * info.f_bfree;
    }
    return free_size;
}

//------------------------------------------------------------
// Function:  getFileSize
// Summary: get the file size.
// In:      file_path -> the file path.
// Out:     None
// Return:  the size of the file. The unit is Byte.
//          -1 -> fail.
//------------------------------------------------------------
long long FileOperations::getFileSize(const char *file_path)
{
    long long file_size = -1;
    struct stat info;
    if (lstat(file_path, &info) == 0)
        file_size = info.st_size;

    return file_size;
}

//------------------------------------------------------------
// Function:  getDirSize
// Summary: get the directory size.
// In:      dir_path -> the directory path.
// Out:     None
// Return:  the size of the directory. The unit is Byte.
//          -1 -> fail.
//------------------------------------------------------------
long long FileOperations::getDirSize(const char *dir_path)
{
    long long total_size = 0;
    DIR *dp;
    struct dirent *entry;
    struct stat info;
    if ((dp = opendir(dir_path)) == NULL)
        return -1;

    lstat(dir_path, &info);
    total_size += info.st_size;

    while ((entry = readdir(dp)) != NULL)
    {
        char subdir[256];
        sprintf(subdir, "%s/%s", dir_path, entry->d_name);
        lstat(subdir, &info);

        if (S_ISDIR(info.st_mode))
        {
            if (strcmp(".", entry->d_name) == 0 || strcmp("..", entry->d_name) == 0)
                continue;
            long long subdir_size = getDirSize(subdir);
            total_size += subdir_size;
        }
        else
        {
            total_size += info.st_size;
        }
    }
    closedir(dp);
    return total_size;
}

//------------------------------------------------------------
// Function:  archiveCreate
// Summary: create a zip.
// In:      source -> the source path to be compress.
//          destination -> the path of the zip output.
// Out:     None
// Return:  FST_SUCCESS -> success.
//          SYS_COMPRESS_OPEN_FILE_FAIL -> can't find the file.
//          SYS_COMPRESS_READ_FILE_HEADER_FAIL -> can't read the file.
//          SYS_COMPRESS_WRITE_FILE_FAIL -> can't copy the file.
//------------------------------------------------------------
ErrorCode FileOperations::archiveCreate(const char **source, const char *destination, const char *passphrase)
{
    return SUCCESS;
    /*
    struct archive *a;
    struct archive_entry *entry;
    char buff[8192];
    int len;
    int fd;
    U64 result = FST_SUCCESS;

    // allocate the data structure and set the properties. 
    a = archive_write_new(); 
    archive_write_add_filter_gzip(a);
    archive_write_set_format_zip(a);

    // set the form of encryption and the passphrase. 
	if (ARCHIVE_OK != archive_write_set_options(a, "zip:encryption=aes128")) // other option is zipcrypt, aes256.
    {
        printf("FileOperations::archiveCreate: Can't set passphrase because no cryptographic library.");
	}
    archive_write_set_passphrase(a, passphrase_);
    archive_write_open_filename(a, destination);

    // read the directory and start traversal. 
    struct archive *disk = archive_read_disk_new();
    int r = archive_read_disk_open(disk, *source);
    if (r != ARCHIVE_OK)
    {
        //std::cout<<"Error in FileOperations::archiveCreate(): "
        //    <<"Can't open the file:"<<archive_error_string(disk)<<std::endl;
        printf("FileOperations::archiveCreate: Can't open source files -> %s", source);
        return SYS_COMPRESS_OPEN_FILE_FAIL;
    }  

    // get the path of the source. 
    std::string path = *source;
    path = path.substr(0, path.rfind('/') + 1);

    for(;;)
    {
        // iterate through the entries in the input directory. 
        entry = archive_entry_new();
        r = archive_read_next_header2(disk, entry); // info about the file entries.
        const char *currentFile = archive_entry_pathname(entry);
        archive_read_disk_descend(disk); // to get a full traversal.
        if (r == ARCHIVE_EOF)
        {
            archive_entry_free(entry);
            break;
        }
        if (r != ARCHIVE_OK)
        {
            archive_entry_free(entry);
            printf("FileOperations::archiveCreate: Can't read the file header -> %s", currentFile);
            result = SYS_COMPRESS_READ_FILE_HEADER_FAIL;
            break;
        }

        // set the output archive entry. 
        std::string new_entry = currentFile;
        new_entry = new_entry.substr(path.length());

        archive_entry_set_pathname(entry, new_entry.c_str());     
 
        // iterate to wirte the entries to the ouput archive. 
        r = archive_write_header(a, entry);
        if (r > ARCHIVE_FAILED)
        {
            fd = open(archive_entry_sourcepath(entry), O_RDONLY);
            len = read(fd, buff, sizeof(buff));
            while (len > 0)
            {
                archive_write_data(a, buff, len); // write the entry data to the output archive.
                len = read(fd, buff, sizeof(buff));
            }
            close(fd);
        }
        else
        {
            archive_entry_free(entry);
            printf("FileOperations::archiveCreate: Can't write the archive -> %s", currentFile);
            result = SYS_COMPRESS_WRITE_FILE_FAIL;
            break;
        }
        archive_entry_free(entry);
    }

    archive_read_close(disk);
    archive_read_free(disk);
    archive_write_close(a); // return an error code for detecting errors.
    archive_write_free(a); // no return errors.
    return result;
    */
}

//------------------------------------------------------------
// Function:  archiveExtract
// Summary: extract a zip.
// In:      source -> the source path to be extracted.
//          destination -> the destination path.
// Out:     None
// Return:  FST_SUCCESS -> success.
//          SYS_EXTRACT_OPEN_ARCHIVE_FAIL -> can't find the zip.
//          SYS_EXTRACT_READ_ARCHIVE_FAIL -> can't read the file.
//          SYS_EXTRACT_WRITE_FILE_HEADER_FAIL -> can't write the file header.
//          SYS_EXTRACT_WRITE_FILE_DATA_FAIL -> can't write the data
//------------------------------------------------------------
ErrorCode FileOperations::archiveExtract(const char *source, const char *destination, const char *passphrase)
{
    return SUCCESS;
    /*
    struct archive *a;
    struct archive *ext;
    struct archive_entry *entry;
    mode_t mode;
    int flags;
    int r;
    U64 result = FST_SUCCESS;

    // allocate the data structure and set the properties. 
    a = archive_read_new();
    archive_read_support_filter_gzip(a);
    archive_read_support_format_zip(a);
    archive_read_add_passphrase(a, passphrase_);
    if ((r = archive_read_open_filename(a, source, 10240))) // "10240" is the block size.
    {
        printf("FileOperations::archiveExtract: Can't open the archive -> %s", source);
        return SYS_EXTRACT_OPEN_ARCHIVE_FAIL;
    }

    // set the properties for the extration files. 
    ext = archive_write_disk_new();
    flags = ARCHIVE_EXTRACT_TIME;
    flags |= ARCHIVE_EXTRACT_PERM;
    flags |= ARCHIVE_EXTRACT_ACL;
    flags |= ARCHIVE_EXTRACT_FFLAGS;
    archive_write_disk_set_options(ext, flags);
    archive_write_disk_set_standard_lookup(ext);

    for(;;)
    {    
        // iterate through the entries in the input archive. 
        r = archive_read_next_header(a, &entry);
        const char *currentFile = archive_entry_pathname(entry);
        if (r == ARCHIVE_EOF)
            break;
        if (r != ARCHIVE_OK)
        {
            printf("FileOperations::archiveExtract: Can't read the files of the archive -> %s", currentFile);
            result = SYS_EXTRACT_READ_ARCHIVE_FAIL;
            break;
        }

        // set the destination path. 
        std::string fullOutputPath = destination;
        fullOutputPath += currentFile;
        archive_entry_set_pathname(entry, fullOutputPath.c_str());

        // set the files permission. 
        mode = archive_entry_mode(entry);
        archive_entry_set_mode(entry, mode | 0755);
        //const char *seemode = archive_entry_strmode(entry);
        //std::cout<<"entry name="<<fullOutputPath<<". seemode="<<seemode<<std::endl;

        // iterate to wirte the entries to the ouput file. 
        r = archive_write_header(ext, entry);
        if (r != ARCHIVE_OK)
        {
            printf("FileOperations::archiveExtract: Can't write the file header -> %s", fullOutputPath.c_str());
            result = SYS_EXTRACT_WRITE_FILE_HEADER_FAIL;
            break;
        }
        else if (archive_entry_size(entry) > 0)
        {
            // iterate to copy the data to the output file. 
            r = copyData(a, ext);
            if (r != ARCHIVE_OK)
            {
                 printf("FileOperations::archiveExtract: Can't write the file data -> %s", fullOutputPath.c_str());
                 result = SYS_EXTRACT_WRITE_FILE_DATA_FAIL;
                 break;
            }
            r = archive_write_finish_entry(ext);
            if (r != ARCHIVE_OK)
            {
                info("FileOperations::archiveExtract: Can't finish writing entry -> %s", fullOutputPath.c_str());
            }
        }
    }
    archive_read_close(a);
    archive_read_free(a);
    archive_write_close(ext);
    archive_write_free(ext);
    return result;
    */
}

//------------------------------------------------------------
// Function:  copyData
// Summary: copy data when extracting a zip.
// In:      source -> the source path.
//          destination -> the destination path.
// Out:     None
// Return:  ARCHIVE_OK -> success.
//------------------------------------------------------------
int FileOperations::copyData(struct archive *ar, struct archive *aw)
{/*
	int result;
	const void *buff;
	size_t size;
	int64_t offset;

	for (;;) 
    {
        // iterate to read the data from an archive. 
		result = archive_read_data_block(ar, &buff, &size, &offset);
		if (result == ARCHIVE_EOF)
			return (ARCHIVE_OK);
		if (result != ARCHIVE_OK) 
        {
            printf("FileOperations::copyData: Can't read the data -> %s", archive_printf_string(ar));
			return result;
		}

        // iterate to write the data to an archive. 
		result = archive_write_data_block(aw, buff, size, offset);
		if (result != ARCHIVE_OK) 
        {
            printf("FileOperations::copyData: Can't write the data -> %s", archive_error_string(ar));
			return result;
		}
	}*/
}



#endif //FILE_MANAGER_FILE_OPERATIONS_CPP_

