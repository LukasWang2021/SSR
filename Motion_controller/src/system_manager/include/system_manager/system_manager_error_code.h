/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       system_manager_error_code.h
Author:     Feng.Wu 
Create:     12-Jun-2017
Modify:     12-Jun-2017
Summary:    define error codes
**********************************************/

#ifndef SYSTEM_MANAGER_ERROR_CODE_H_
#define SYSTEM_MANAGER_ERROR_CODE_H_

#ifndef FST_SUCCESS
#define FST_SUCCESS (unsigned long long int)0
#endif
typedef unsigned long long int U64;

#define SYS_EXTRACT_ARCHIVE_FAIL (unsigned long long int)0x00010002009703F3   /*fail to extract the archive when restoring the backup files.*/
#define SYS_EXTRACT_OPEN_ARCHIVE_FAIL (unsigned long long int)0x00000002009703F4   /*fail to open the archive when extract*/
#define SYS_EXTRACT_READ_ARCHIVE_FAIL (unsigned long long int)0x00000002009703F5   /*fail to read the file when extract*/
#define SYS_EXTRACT_WRITE_FILE_HEADER_FAIL (unsigned long long int)0x00000002009703F6   /*fail to write the file header when extract*/
#define SYS_EXTRACT_WRITE_FILE_DATA_FAIL (unsigned long long int)0x00000002009703F7   /*fail to write the file data when extract*/
#define SYS_COMPRESS_FILE_FAIL (unsigned long long int)0x00010002009703FD   /*fail to compress when backup*/
#define SYS_COMPRESS_OPEN_FILE_FAIL (unsigned long long int)0x00000002009703FE   /*fail to open the file when compress*/
#define SYS_COMPRESS_READ_FILE_HEADER_FAIL (unsigned long long int)0x00000002009703FF   /*fail to read the file header when compress*/
#define SYS_COMPRESS_WRITE_FILE_FAIL (unsigned long long int)0x0000000200970400   /*fail to write the archive when compress*/
#define SYS_INIT_FAIL (unsigned long long int)0x0011000200970407   /*fail to initialize the system manager.*/
#define SYS_READ_VERSION_FAIL (unsigned long long int)0x0000000200970408   /*fail to read the version of the software*/
#define SYS_NO_FREE_DISK (unsigned long long int)0x0001000200970409   /*not enough space for file operation*/
#define SYS_FTP_ON_FAIL (unsigned long long int)0x000100020097040A   /*fail to start ftp*/
#define SYS_FTP_OFF_FAIL (unsigned long long int)0x000100020097040B   /*fail to stop ftp*/
#define SYS_UPGRADE_FAIL (unsigned long long int)0x000100020097040C   /*fail to upgrade*/
#define SYS_OPS_UNFINISHED (unsigned long long int)0x000000020097040D   /*the operation is unfinished*/
#define SYS_OPS_BUSY (unsigned long long int)0x000000020097040E   /*can not execute because busying in operation.*/
#define SYS_UNRECOGNIZED_SERVICE_ID (unsigned long long int)0x000000020097040F   /*an unrecognized service id is received.*/
#define SYS_CONFIG_DAMAGED (unsigned long long int)0x0001000200970410   /*the configuration files are damaged.*/
#define SYS_CONFIGURABLE_FILE_ERROR (unsigned long long int)0x0001000200970411   /*fail to restore the original configurable file.*/
#define SYS_MACHINE_CONFIG_ERROR (unsigned long long int)0x0001000200970412   /*fail to restore the machine configuration file.*/


#endif //SYSTEM_MANAGER_ERROR_CODE_H_
