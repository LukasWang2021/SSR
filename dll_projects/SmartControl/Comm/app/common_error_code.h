#ifndef COMMON_ERROR_CODE_H
#define COMMON_ERROR_CODE_H

#pragma once
#include "comm_def.h"
#include <string>

typedef struct
{
	uint64_t error_code;
	std::string detail;
}ErrorCodeDetail;

#define MAX_ERROR_CODE_NUMBER 48
extern ErrorCodeDetail error_code_detail[MAX_ERROR_CODE_NUMBER];

typedef uint64_t ErrorCode;

#define SUCCESS 0

#define HANDLE_RPC_FAILED 0x10001000
#define HANDLE_SUB_FAILED 0x10001001
#define HANDLE_EVENT_FAILED 0x10001002


#define CORE_COMM_LOAD_PARAM_FAILED 0x1000
#define CORE_COMM_LOAD_CORE_COMM_CONFIG_FAILED 0x1001
#define CORE_COMM_OPEN_INIT_DEVICE_FAILED    0x1002
#define CORE_COMM_OPEN_CONFIG_DEVICE_FAILED    0x1003
#define CORE_COMM_OPEN_COMM_DEVICE_FAILED    0x1004
#define CORE_COMM_COPY_CONFIG_TO_DEVICE_FAILED    0x1005
#define CORE_COMM_COPY_DEVICE_TO_CONFIG_FAILED    0x1006
#define CORE_COMM_SET_BOARDCAST_FAILED  0x1007
#define CORE_COMM_GET_BOARDCAST_FAILED  0x1008
#define CORE_COMM_SEND_EVENT_FAILED 0x1009
#define CORE_COMM_SLAVE_EVENT_CONFIG_INVALID    0x100A
#define CORE_COMM_INVALID_INIT_TYPE    0x100B
#define CORE_COMM_CONFIGURATION_MISMATCH 0x100C
#define CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED 0x100D
#define CORE_COMM_RECV_CORE_PROCESS_CALL_FAILED 0x100E
#define CORE_COMM_RECV_CORE_PROCESS_CALL_TIMEOUT 0x100F
#define CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED 0x1010
#define CONTROLLER_INIT_FAILED 0x2000
#define CONTROLLER_CREATE_ROUTINE_THREAD_FAILED 0x2001
#define CONTROLLER_CREATE_ALG_THREAD_FAILED 0x2002
#define CONTROLLER_CREATE_RT_THREAD_FAILED 0x2003
#define CONTROLLER_PUBLISH_EXIST 0x2011   /*the publishing element of request is exist*/
#define CONTROLLER_INVALID_OPERATION 0x2012   /*Controller failed to operate command because of invalid pre-condition*/
#define CONTROLLER_PUBLISH_NONE 0x2013   /*Controller failed to find the publishing elements of request*/
#define TP_COMM_LOAD_PARAM_FAILED 0x3001   /*TpComm loading param is failed in initialization phase*/
#define TP_COMM_INIT_OBJECT_FAILED 0x3002   /*TpComm failed to initialize internal variables*/
#define TP_COMM_CREATE_ROUTINE_THREAD_FAILED 0x3003   /*TpComm failed to create routine thread*/
#define TP_COMM_INVALID_REQUEST 0x3004   /*TpComm receives a invalid hash for RPC*/
#define TP_COMM_ENCODE_FAILED 0x3005   /*TpComm failed to encode data to send out*/
#define TP_COMM_DECODE_FAILED 0x3006   /*TpComm failed to decode data that has been received*/
#define TP_COMM_MEMORY_OPERATION_FAILED 0x3007   /*TpComm failed to operate memory*/
#define TP_COMM_AUTHORITY_CHECK_FAILED 0x3008   /*TpComm failed to run a unauthorized operation*/
#define TP_COMM_SEND_FAILED 0x3009   /*TpComm failed to send a package*/
#define TP_COMM_RECEIVE_FAILED 0x300A   /*TpComm failed to receive a package*/
#define TP_COMM_RPC_OVERLOAD 0x300B   /*TpComm failed to handle too much rpc request*/
#define RPC_PARAM_INVALID 0x4001
#define RPC_EXECUTE_FAILED 0x4002
#define AXIS_STATE_TRANSFER_INVALID 0x5001
#define AXIS_SEND_CORE_POWER_FAILED 0x5002
#define AXIS_SEND_CORE_RESET_FAILED 0x5003
#define AXIS_SEND_CORE_STOP_FAILED 0x5004
#define AXIS_SEND_CORE_HALT_FAILED 0x5005
#define AXIS_ALG_NOT_DEFINED 0x5006
#define AXIS_ALG_COMPUTE_FAILED 0x5007
#define AXIS_PDO_TIMEOUT 0x6008
#define FILE_MANAGER_READ_FILE_FAILED 0x7001   /*failed to read file*/
#define FILE_MANAGER_WRITE_FILE_FAILED 0x7002   /*failed to write file*/



#endif

