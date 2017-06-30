/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       service_manager_error_code.h
Author:     Feng.Wu 
Create:     14-Feb-2017
Modify:     14-Feb-2017
Summary:    define error codes
**********************************************/

#ifndef SERVICE_MANAGER_ERROR_CODE_H_
#define SERVICE_MANAGER_ERROR_CODE_H_

#define ERROR_CODE_TYPE unsigned long long int
#ifndef FST_SUCCESS
#define FST_SUCCESS (unsigned long long int)0
#endif

#define OPEN_CORE_MEM_FAIL (unsigned long long int)0x0000000B007103E9   /*fail to open sharedmem of cores*/
#define CREATE_CHANNEL_FAIL (unsigned long long int)0x0000000B006F03F3   /*fail to create channel between process*/
#define SEND_MSG_FAIL (unsigned long long int)0x00000002006F03F4   /*fail to send msg*/
#define RECV_MSG_FAIL (unsigned long long int)0x00000002006F03F5   /*fail to recv msg*/
#define BARE_CORE_TIMEOUT (unsigned long long int)0x0001000B0071044C   /*no heartbeat from BARE CORE within a limited time.*/
#define INVALID_SERVICE_ID (unsigned long long int)0x00000004006F044D   /*invalid service ID received from other processes.*/
#define SEND_RESP_FAIL (unsigned long long int)0x00000004006F044E   /*fail to send response to other processes within limited tries.*/
#define MCS_TIMEOUT (unsigned long long int)0x00010004006F044F   /*no heartbeat from Motion Controller within a limited time.*/




#endif //MIDDLEWARE_TO_MEM_ERROR_CODE_H_
