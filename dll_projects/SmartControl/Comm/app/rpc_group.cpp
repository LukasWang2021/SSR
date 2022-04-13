#include "rpc_interface.h"
#include "rpc_basic.h"
#include "common_error_code.h"
#include "protoc.h"


uint64_t c_groupReset(int32_t group_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = group_id;

	if (!rpc_ptr->handleRpc(0x00016FF4, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_groupEnable(int32_t group_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = group_id;

	if (!rpc_ptr->handleRpc(0x00003615, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}

uint64_t c_groupDisable(int32_t group_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = group_id;

	if (!rpc_ptr->handleRpc(0x0000D185, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}


uint64_t c_groupReadError(int32_t group_id, uint64_t* error)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = group_id;

	if (!rpc_ptr->handleRpc(0x00004BE2, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*error = rep_data.data.data;
	return rep_data.error_code.data;
}

uint64_t c_groupReadStatus(int32_t group_id, int32_t* status, int32_t* in_position)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64_GroupStatus_Bool rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = group_id;

	if (!rpc_ptr->handleRpc(0x00002A83, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_GroupStatus_Bool_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	*status = rep_data.data1;
    *in_position = rep_data.data2.data;
	return rep_data.error_code.data;
}

uint64_t c_groupResetAllEncoder(int32_t group_id)
{
	if (!rpc_valid)
		return HANDLE_RPC_FAILED;
	RpcBasic* rpc_ptr = RpcBasic::getInstance();
	RequestMessageType_Int32 req_data;
	ResponseMessageType_Uint64 rep_data;

	req_data.header.time_stamp = 122;
	req_data.property.authority = Comm_Authority_TP_SIMMULATOR;
	req_data.data.data = group_id;

	if (!rpc_ptr->handleRpc(0x000019D2, &req_data, RequestMessageType_Int32_fields, &rep_data, ResponseMessageType_Uint64_fields))
	{
		return HANDLE_RPC_FAILED;
	}
	return rep_data.data.data;
}
