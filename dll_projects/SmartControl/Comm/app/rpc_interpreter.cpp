#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include <windows.h>

uint64_t c_interpreterStart(char* program_name)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_String req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	memcpy(req_data.data.data, program_name, strlen(program_name));
	req_data.data.data[strlen(program_name)] = 0;
	if (!rpc_ptr->handleRpc(0x00006154, &req_data, RequestMessageType_String_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_interpreterPause(int program_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = program_id;
	if (!rpc_ptr->handleRpc(0x0000BA55, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_interpreterResume(int program_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = program_id;
	if (!rpc_ptr->handleRpc(0x0000CF55, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_interpreterAbort(int program_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = program_id;
	if (!rpc_ptr->handleRpc(0x000086F4, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_interpreterForward(int program_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = program_id;
	if (!rpc_ptr->handleRpc(0x0000D974, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_interpreterBackward(int program_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = program_id;
	if (!rpc_ptr->handleRpc(0x00008E74, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_interpreterJump(int program_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = program_id;
	if (!rpc_ptr->handleRpc(0x00015930, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


