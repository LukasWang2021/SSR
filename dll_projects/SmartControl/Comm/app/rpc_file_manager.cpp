//#include "stdafx.h"
#include "rpc_interface.h"
#include "comm_def.h"
#include "rpc_basic.h"
#include "response_uint64_bytes.pb.h"
#include "request_string_bytes.pb.h"
#include "common_error_code.h"
#include "protoc.h"

uint64_t c_readFile(char* file_path_ptr, uint8_t file_data[65536], int* byte_size_ptr)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_String req_data;
	ResponseMessageType_Uint64_Bytes rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	size_t file_path_size = strlen(file_path_ptr);
	memcpy(req_data.data.data, file_path_ptr, file_path_size);
	req_data.data.data[file_path_size] = 0;

	if (!rpc_ptr->handleRpc(0x0000A545, &req_data, RequestMessageType_String_fields, &rep_data, ResponseMessageType_Uint64_Bytes_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*byte_size_ptr = rep_data.data.data.size;
	memcpy(&file_data[0], &rep_data.data.data.bytes[0], *byte_size_ptr);
	return rep_data.error_code.data;
}

uint64_t c_writeFile(char* file_path_ptr, uint8_t file_data[65536], int byte_size)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_String_Bytes req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	size_t file_path_size = strlen(file_path_ptr);
	memcpy(req_data.data1.data, file_path_ptr, file_path_size);
	req_data.data1.data[file_path_size] = 0;
	memcpy(&req_data.data2.data.bytes, &file_data[0], byte_size);
	req_data.data2.data.size = byte_size;

	if (!rpc_ptr->handleRpc(0x00010D95, &req_data, RequestMessageType_String_Bytes_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}

	return rep_data.data.data;
}