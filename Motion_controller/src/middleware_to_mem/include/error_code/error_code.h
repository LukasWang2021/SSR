/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       error_code.h
Author:     Feng.Wu 
Create:     30-Nov-2016
Modify:     04-Dec-2016
Summary:    define error codes
**********************************************/

#ifndef MIDDLEWARE_TO_MEM_ERROR_CODE_H_
#define MIDDLEWARE_TO_MEM_ERROR_CODE_H_

#define ERROR_CODE_TYPE unsigned long long int
#ifndef FST_SUCCESS
#define FST_SUCCESS (unsigned long long int)0
#endif

#define OPEN_CORE_MEM_FAIL (unsigned long long int)0x0000000B007103E9   /*fail to open sharedmem of cores*/
#define WRITE_CORE_MEM_FAIL (unsigned long long int)0x00000002007103EA   /*fail to write sharedmem of cores*/
#define READ_CORE_MEM_FAIL (unsigned long long int)0x00000002007103EB   /*fail to read sharedmem of cores*/
#define CREATE_CHANNEL_FAIL (unsigned long long int)0x0000000B006F03F3   /*fail to create channel between process*/
#define SEND_MSG_FAIL (unsigned long long int)0x00000002006F03F4   /*fail to send msg*/
#define RECV_MSG_FAIL (unsigned long long int)0x00000002006F03F5   /*fail to recv msg*/





#endif //MIDDLEWARE_TO_MEM_ERROR_CODE_H_
