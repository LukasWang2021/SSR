/*************************************************************************
	> File Name: parameter_manager_error_code.h
	> Author: 
	> Mail: 
	> Created Time: 2016年12月07日 星期三 09时20分41秒
 ************************************************************************/

#ifndef _PARAMETER_MANAGER_ERROR_CODE_H
#define _PARAMETER_MANAGER_ERROR_CODE_H

#ifndef SUCCESS
#define SUCCESS (unsigned long long int)0x0000000000000000
#endif

#define PARAM_INTERNAL_FAULT (unsigned long long int)0x0001000400790001   /*program internal fault*/
#define PARAM_FAIL_IN_INIT (unsigned long long int)0x00010002007903E9   /*initialization failed*/
#define PARAM_NOT_FOUND (unsigned long long int)0x00010002007903F3   /*cannot find the param*/
#define PARAM_TYPE_ERROR (unsigned long long int)0x00010002007903F4   /*param has a type beyond expectation*/
#define PARSE_ERROR (unsigned long long int)0x00010002007903FD   /*cannot parse a scalar to expected value type*/
#define COMMUNICATION_ERROR (unsigned long long int)0x0001000200790407   /*cannot communicate with remote server*/
#define BAD_FILE_PATH (unsigned long long int)0x00000002007907D1   /*bad path of config file*/
#define BAD_FILE_EXTENSION (unsigned long long int)0x00000002007907D2   /*bad extension of config file*/
#define BAD_FILE_NAME (unsigned long long int)0x00000002007907D3   /*bad name of config file*/
#define FAIL_OPENNING_FILE (unsigned long long int)0x00010002007907DB   /*open config file failed*/
#define FAIL_BUILDING_PARAM_TREE (unsigned long long int)0x00010002007907DC   /*build param tree failed*/
#define FAIL_RESTORING_YAML (unsigned long long int)0x00010002007907DD   /*restore YAML from backup failed*/
#define FAIL_UPDATING_BACKUP (unsigned long long int)0x00010002007907DE   /*update backup file falled*/


typedef unsigned long long int ErrorCode;

#endif
