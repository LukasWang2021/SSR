#include "common_error_code.h"







ErrorCodeDetail error_code_detail[MAX_ERROR_CODE_NUMBER] =
{
	{ HANDLE_RPC_FAILED, "DLL_HANDLE_RPC_FAILED"},
	{ CORE_COMM_LOAD_PARAM_FAILED, "CORE_COMM_LOAD_PARAM_FAILED"},
	{ CORE_COMM_LOAD_CORE_COMM_CONFIG_FAILED, "CORE_COMM_LOAD_CORE_COMM_CONFIG_FAILED"},
	{ CORE_COMM_OPEN_INIT_DEVICE_FAILED, "CORE_COMM_OPEN_INIT_DEVICE_FAILED!"},
	{ CORE_COMM_OPEN_CONFIG_DEVICE_FAILED, "CORE_COMM_OPEN_CONFIG_DEVICE_FAILED"},
	{ CORE_COMM_OPEN_COMM_DEVICE_FAILED, "CORE_COMM_OPEN_COMM_DEVICE_FAILED"},
	{ CORE_COMM_COPY_CONFIG_TO_DEVICE_FAILED, "CORE_COMM_COPY_CONFIG_TO_DEVICE_FAILED"},
	{ CORE_COMM_COPY_DEVICE_TO_CONFIG_FAILED, "CORE_COMM_COPY_DEVICE_TO_CONFIG_FAILEDg"},
	{ CORE_COMM_SET_BOARDCAST_FAILED, "CORE_COMM_SET_BOARDCAST_FAILED"},
	{ CORE_COMM_GET_BOARDCAST_FAILED, "CORE_COMM_GET_BOARDCAST_FAILED"},
	{ CORE_COMM_SEND_EVENT_FAILED, "CORE_COMM_SEND_EVENT_FAILED"},
	{ CORE_COMM_SLAVE_EVENT_CONFIG_INVALID, "CORE_COMM_SLAVE_EVENT_CONFIG_INVALID"},
	{ CORE_COMM_INVALID_INIT_TYPE, "CORE_COMM_INVALID_INIT_TYPE"},
	{ CORE_COMM_CONFIGURATION_MISMATCH, "CORE_COMM_CONFIGURATION_MISMATCH!"},
	{ CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED, "CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED"},
	{ CORE_COMM_RECV_CORE_PROCESS_CALL_FAILED, "CORE_COMM_RECV_CORE_PROCESS_CALL_FAILED"},
	{ CORE_COMM_RECV_CORE_PROCESS_CALL_TIMEOUT, "CORE_COMM_RECV_CORE_PROCESS_CALL_TIMEOUT"},
	{ CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED, "CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED"},
    { CONTROLLER_INIT_FAILED, "CONTROLLER_INIT_FAILED"},
	{ CONTROLLER_CREATE_ROUTINE_THREAD_FAILED, "CONTROLLER_CREATE_ROUTINE_THREAD_FAILED"},
	{ CONTROLLER_CREATE_ALG_THREAD_FAILED, "CONTROLLER_CREATE_ALG_THREAD_FAILED"},
    { CONTROLLER_CREATE_RT_THREAD_FAILED, "CONTROLLER_CREATE_RT_THREAD_FAILED"},
	{ CONTROLLER_PUBLISH_EXIST, "CONTROLLER_PUBLISH_EXIST"},
	{ CONTROLLER_INVALID_OPERATION, "CONTROLLER_INVALID_OPERATION"},
	{ CONTROLLER_PUBLISH_NONE, "CONTROLLER_PUBLISH_NONE"},
	{ TP_COMM_LOAD_PARAM_FAILED, "TP_COMM_LOAD_PARAM_FAILED"},
	{ TP_COMM_INIT_OBJECT_FAILED, "TP_COMM_INIT_OBJECT_FAILED"},
	{ TP_COMM_CREATE_ROUTINE_THREAD_FAILED, "TP_COMM_CREATE_ROUTINE_THREAD_FAILED"},
	{ TP_COMM_INVALID_REQUEST, "TP_COMM_INVALID_REQUEST"},
	{ TP_COMM_ENCODE_FAILED, "TP_COMM_ENCODE_FAILED"},
	{ TP_COMM_DECODE_FAILED, "TP_COMM_DECODE_FAILED"},
	{ TP_COMM_MEMORY_OPERATION_FAILED, "TP_COMM_MEMORY_OPERATION_FAILED"},
	{ TP_COMM_AUTHORITY_CHECK_FAILED, "TP_COMM_AUTHORITY_CHECK_FAILED"},
	{ TP_COMM_SEND_FAILED, "TP_COMM_SEND_FAILED"},
	{ TP_COMM_RECEIVE_FAILED, "TP_COMM_RECEIVE_FAILED"},
	{ TP_COMM_RPC_OVERLOAD, "TP_COMM_RPC_OVERLOAD"},
	{ RPC_PARAM_INVALID, "RPC_PARAM_INVALID"},
	{ RPC_EXECUTE_FAILED, "RPC_EXECUTE_FAILED"},
    { AXIS_STATE_TRANSFER_INVALID, "AXIS_STATE_TRANSFER_INVALID"},
	{ AXIS_SEND_CORE_POWER_FAILED, "AXIS_SEND_CORE_POWER_FAILED"},
	{ AXIS_SEND_CORE_RESET_FAILED, "AXIS_SEND_CORE_RESET_FAILED"},
	{ AXIS_SEND_CORE_STOP_FAILED, "AXIS_SEND_CORE_STOP_FAILED"},
	{ AXIS_SEND_CORE_HALT_FAILED, "AXIS_SEND_CORE_HALT_FAILED"},
	{ AXIS_ALG_NOT_DEFINED, "AXIS_ALG_NOT_DEFINED"},
	{ AXIS_ALG_COMPUTE_FAILED, "AXIS_ALG_COMPUTE_FAILED"},
	{ AXIS_PDO_TIMEOUT, "AXIS_PDO_TIMEOUT"},
	{ FILE_MANAGER_READ_FILE_FAILED, "FILE_MANAGER_READ_FILE_FAILED"},
	{ FILE_MANAGER_WRITE_FILE_FAILED, "FILE_MANAGER_WRITE_FILE_FAILED"},
};

