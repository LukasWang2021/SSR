//#include "stdafx.h"
#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include "protoc.h"


uint64_t c_setSamplingConfiguration(int32_t cpu_id, uint32_t interval, uint32_t max_times)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Uint32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data =cpu_id;
	req_data.data2.data_count = 2;
	req_data.data2.data[0] = interval;
	req_data.data2.data[1] = max_times;
	if (!rpc_ptr->handleRpc(0x0000845E, &req_data, RequestMessageType_Int32_Uint32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_getSamplingConfiguration(int32_t cpu_id, uint32_t* interval, uint32_t* max_times)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x000106EE, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	if (rep_data.data.data_count != 2)
	{
		return RPC_PARAM_INVALID;
	}
	*interval = rep_data.data.data[0];
	*max_times = rep_data.data.data[1];
	return rep_data.error_code.data;
}

uint64_t c_activateSamplingConfiguration(int32_t cpu_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x0000CDDE, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_setSamplingSync(int32_t cpu_id, uint32_t sync)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_Uint32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	req_data.data2.data = sync;
	if (!rpc_ptr->handleRpc(0x00003743, &req_data, RequestMessageType_Int32_Uint32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_getSamplingSync(int32_t cpu_id, uint32_t* sync)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x00006343, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*sync = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_setSamplingChannel(int32_t cpu_id, int32_t channel_index, int32_t servo_index, int32_t servo_param_index)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 4;
	req_data.data.data[0] = cpu_id;
	req_data.data.data[1] = channel_index;
	req_data.data.data[2] = servo_index;
	req_data.data.data[3] = servo_param_index;
	if (!rpc_ptr->handleRpc(0x0000BACC, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_getSamplingChannel(int32_t cpu_id, uint32_t value[16], int32_t* size)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint32List rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = cpu_id;
	if (!rpc_ptr->handleRpc(0x0000556C, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint32List_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*size = rep_data.data.data_count;
	for (unsigned int i = 0; i < rep_data.data.data_count; ++i)
	{
		value[i] = rep_data.data.data[i];
	}
	return rep_data.error_code.data;
}

uint64_t c_saveSamplingBufferData(int32_t cpu_id, char* file_path_ptr)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32_String req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data1.data = cpu_id;
	size_t file_path_size = strlen(file_path_ptr);
	memcpy(req_data.data2.data, file_path_ptr, file_path_size);
	req_data.data2.data[file_path_size] = 0;
	if (!rpc_ptr->handleRpc(0x00004E41, &req_data, RequestMessageType_Int32_String_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}