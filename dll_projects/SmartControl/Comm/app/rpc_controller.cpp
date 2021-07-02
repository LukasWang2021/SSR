//#include "stdafx.h"
#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include <windows.h>

bool rpc_valid = false;

uint64_t c_initRpc(char* server_ip)
{
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	if (rpc_ptr == NULL)
		return HANDLE_RPC_FAILED;
	if (0 != rpc_ptr->init(std::string(server_ip)))
		return HANDLE_RPC_FAILED;
	rpc_valid = true;
	return 0;
}

uint64_t c_getVersion(uint32_t version[4], int32_t* size)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Void req_data;
	ResponseMessageType_Uint64_Uint32List rep_data;
	
	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	if (!rpc_ptr->handleRpc(0x000093EE, &req_data, RequestMessageType_Void_fields, &rep_data, ResponseMessageType_Uint64_Uint32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	*size = rep_data.data.data_count;
	for (unsigned int i = 0; i < rep_data.data.data_count; ++i)
	{
		version[i] = rep_data.data.data[i];
	}
	return rep_data.error_code.data;
}

uint64_t c_getSystemTime(uint64_t* time)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Void req_data;
	ResponseMessageType_Uint64_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP;
	if (!rpc_ptr->handleRpc(0x000003F5, &req_data, RequestMessageType_Void_fields, &rep_data, ResponseMessageType_Uint64_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*time = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_setWorkMode(uint32_t mode)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.data.data = mode;
	req_data.property.authority = Comm_Authority_TP;
	if (!rpc_ptr->handleRpc(0x00006825, &req_data, RequestMessageType_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_getWorkMode(uint32_t* mode)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Void req_data;
	ResponseMessageType_Uint64_Uint32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP;
	if (!rpc_ptr->handleRpc(0x00003325, &req_data, RequestMessageType_Void_fields, &rep_data, ResponseMessageType_Uint64_Uint32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*mode = rep_data.data.data;
	return rep_data.error_code.data;
}


uint64_t c_setControlMode(uint32_t mode)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.data.data = mode;
	req_data.property.authority = Comm_Authority_TP;
	if (!rpc_ptr->handleRpc(0x0000B555, &req_data, RequestMessageType_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_getControlMode(uint32_t* mode)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Void req_data;
	ResponseMessageType_Uint64_Uint32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP;
	if (!rpc_ptr->handleRpc(0x0000B695, &req_data, RequestMessageType_Void_fields, &rep_data, ResponseMessageType_Uint64_Uint32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*mode = rep_data.data.data;
	return rep_data.error_code.data;
}


uint64_t c_getErrorCodeDetail(uint64_t error_code, char* text_ptr, int* size)
{
	int target_index;
	for (target_index = 0; target_index < MAX_ERROR_CODE_NUMBER; ++target_index)
	{
		if (error_code_detail[target_index].error_code == error_code)
		{
			break;
		}
	}

	if (target_index < MAX_ERROR_CODE_NUMBER)
	{
		*size = (int)error_code_detail[target_index].detail.size();
		memcpy(text_ptr, error_code_detail[target_index].detail.c_str(), *size);
	}
	else
	{
		std::string detail("unknown error code");
		*size = (int)detail.size();
		memcpy(text_ptr, detail.c_str(), *size);
	}

	return 0;
}

uint64_t c_setSamplingChannelGroup(int32_t cpu_id, int32_t channel_index[16], int32_t servo_index[16], int32_t servo_param_index[16], int32_t size)
{
	uint64_t ret = 0;
	for (int i = 0; i < size; ++i)
	{
		ret = c_setSamplingChannel(cpu_id, channel_index[i], servo_index[i], servo_param_index[i]);
		Sleep(10);
		if (ret != 0)
			return ret;
	}
	return ret;
}
