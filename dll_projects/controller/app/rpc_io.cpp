#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include "protoc.h"

uint64_t c_ioReadDI(int32_t port_offset, int32_t* value)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Int32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = port_offset;
	if (!rpc_ptr->handleRpc(0x000185A9, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Int32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*value = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_ioReadDO(int32_t port_offset, int32_t* value)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Int32 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = port_offset;
	if (!rpc_ptr->handleRpc(0x000185AF, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Int32_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*value = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_ioWriteDO(int32_t port_offset, int32_t value)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32List req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data_count = 2;
	req_data.data.data[0] = port_offset;
	req_data.data.data[1] = value;
	if (!rpc_ptr->handleRpc(0x00000C1F, &req_data, RequestMessageType_Int32List_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

