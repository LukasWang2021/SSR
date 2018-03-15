/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_manager_error_code.h
Author:     Feng.Wu 
Create:     14-Feb-2017
Modify:     14-Feb-2017
Summary:    define error codes
**********************************************/

#ifndef IO_MANAGER_ERROR_CODE_H_
#define IO_MANAGER_ERROR_CODE_H_

#ifndef FST_SUCCESS
#define FST_SUCCESS (unsigned long long int)0
#endif
typedef unsigned long long int U64;


#define GET_IO_FAIL (unsigned long long int)0x00010006008F03E9   /*fail to get io data from FPGA*/
#define LOAD_IO_CONFIG_FAIL (unsigned long long int)0x00010006008F03EA   /*fail to load io configuration file*/
#define IO_DEVICE_CHANGED (unsigned long long int)0x00010006008F03EB   /*devices are removed*/

#define IO_INIT_FAIL (unsigned long long int)0x00000002008F03F3   /*fail to init the io module*/
#define IO_VERIFY_FALSE (unsigned long long int)0x00000002008F03F4   /*io data is verified to be false*/
#define IO_INVALID_PARAM_ID (unsigned long long int)0x00000002008F03F5   /*invalid parameter id of input*/
#define IO_INVALID_PORT_SEQ (unsigned long long int)0x00000002008F03F6   /*invalid port sequence number*/
#define IO_INVALID_DEV_INDEX (unsigned long long int)0x00000002008F03F7   /*invalid index to get the device info.*/
#define IO_INVALID_PORT_LEN (unsigned long long int)0x00000002008F03F8   /*invalid port number of device.*/
#define IO_THREAD_INIT_STATUS (unsigned long long int)0x00000002008F03F9   /*io thread is in initial status and not ready.*/


#endif //IO_MANAGER_ERROR_CODE_H_
